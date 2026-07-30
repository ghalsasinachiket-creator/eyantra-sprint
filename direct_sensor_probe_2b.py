#!/usr/bin/env python3
"""Direct CoppeliaSim sensor probe for Task 2B.

This bypasses bridge_v1_2b entirely. Run it while CoppeliaSim, the bridge, and
the normal robot client are already running; it only reads sensor objects.
"""

import argparse
import statistics
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


PROXIMITY_PATHS = [
    "/LineTracer/proximitySensor",
    "/LineTracer/ProximitySensor",
    "/LineTracer/proximity_sensor",
    "/LineTracer/Proximity_sensor",
    "/LineTracer/proximity",
]

COLOR_PATHS = [
    "/LineTracer/colorsensor",
    "/LineTracer/colorSensor",
    "/LineTracer/ColorSensor",
    "/LineTracer/colourSensor",
    "/LineTracer/visionSensor",
    "/LineTracer/VisionSensor",
]


def try_call(fn, *args):
    try:
        return fn(*args)
    except Exception:
        return None


def get_object_or_none(sim, path):
    handle = try_call(sim.getObject, path, {"noError": True})
    if handle is not None and handle != -1:
        return handle

    handle = try_call(sim.getObject, path)
    if handle is not None and handle != -1:
        return handle

    return None


def object_label(sim, handle):
    labels = []
    get_object_alias = getattr(sim, "getObjectAlias", None)
    if get_object_alias is not None:
        for option in (1, 0, 3, 4, -1):
            label = try_call(get_object_alias, handle, option)
            if isinstance(label, str) and label and label not in labels:
                labels.append(label)

    get_object_path = getattr(sim, "getObjectPath", None)
    if get_object_path is not None:
        path = try_call(get_object_path, handle)
        if isinstance(path, str) and path and path not in labels:
            labels.append(path)

    return " | ".join(labels) if labels else f"handle={handle}"


def get_type_constants(sim):
    constants = {}
    for name in ("object_proximitysensor_type", "object_visionsensor_type"):
        constants[name] = getattr(sim, name, None)
    return constants


def list_scene_objects(sim):
    scene = getattr(sim, "handle_scene", -1)
    objects = try_call(sim.getObjectsInTree, scene, -1, 0)
    if objects is None:
        objects = try_call(sim.getObjects, -1)
    return objects or []


def discover_sensor_candidates(sim):
    constants = get_type_constants(sim)
    prox_type = constants.get("object_proximitysensor_type")
    vision_type = constants.get("object_visionsensor_type")
    candidates = []

    for handle in list_scene_objects(sim):
        obj_type = try_call(sim.getObjectType, handle)
        label = object_label(sim, handle)
        lowered = label.lower()
        name_match = any(
            token in lowered
            for token in ("prox", "color", "colour", "vision", "sensor")
        )
        type_match = obj_type in (prox_type, vision_type)
        if name_match or type_match:
            candidates.append((handle, obj_type, label))

    return candidates


def choose_handle(sim, paths, wanted_type=None, name_tokens=()):
    for path in paths:
        handle = get_object_or_none(sim, path)
        if handle is not None:
            return handle, f"explicit path {path}"

    candidates = discover_sensor_candidates(sim)
    for handle, obj_type, label in candidates:
        lowered = label.lower()
        if wanted_type is not None and obj_type == wanted_type:
            if not name_tokens or any(token in lowered for token in name_tokens):
                return handle, f"discovered {label}"

    for handle, _obj_type, label in candidates:
        lowered = label.lower()
        if any(token in lowered for token in name_tokens):
            return handle, f"discovered {label}"

    return None, "not found"


def normalize_read_vision_result(result):
    if result is None:
        return None
    if isinstance(result, tuple):
        return result
    return (result,)


def average_rgb_from_image(sim, vision_handle):
    result = try_call(sim.getVisionSensorImg, vision_handle)
    if result is None:
        return None

    if not isinstance(result, tuple) or len(result) < 2:
        return None

    image, resolution = result[0], result[1]
    if not resolution or len(resolution) < 2:
        return None

    width, height = int(resolution[0]), int(resolution[1])
    pixel_count = max(width * height, 1)

    if isinstance(image, (bytes, bytearray)):
        data = list(image)
    else:
        data = list(image)

    if len(data) < pixel_count * 3:
        return None

    rgb = data[: pixel_count * 3]
    if max(rgb) > 1.0:
        scale = 255.0
    else:
        scale = 1.0

    red = statistics.fmean(rgb[0::3]) / scale
    green = statistics.fmean(rgb[1::3]) / scale
    blue = statistics.fmean(rgb[2::3]) / scale
    return red, green, blue, (width, height)


def format_raw(value, limit=240):
    text = repr(value)
    if len(text) > limit:
        text = text[: limit - 3] + "..."
    return text


def main():
    parser = argparse.ArgumentParser(
        description="Read Task 2B proximity/color sensors directly via CoppeliaSim ZMQ."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--prox-path", action="append", default=[])
    parser.add_argument("--color-path", action="append", default=[])
    parser.add_argument("--list", action="store_true", help="list sensor-like objects and exit")
    args = parser.parse_args()

    client = RemoteAPIClient(host=args.host, port=args.port)
    sim = client.require("sim")

    constants = get_type_constants(sim)
    prox_type = constants.get("object_proximitysensor_type")
    vision_type = constants.get("object_visionsensor_type")

    candidates = discover_sensor_candidates(sim)
    print("Sensor-like objects discovered:")
    for handle, obj_type, label in candidates:
        print(f"  handle={handle} type={obj_type} label={label}")

    if args.list:
        return

    prox_paths = args.prox_path + PROXIMITY_PATHS
    color_paths = args.color_path + COLOR_PATHS

    prox_handle, prox_source = choose_handle(
        sim, prox_paths, wanted_type=prox_type, name_tokens=("prox",)
    )
    color_handle, color_source = choose_handle(
        sim, color_paths, wanted_type=vision_type, name_tokens=("color", "colour", "vision")
    )

    print(f"Selected proximity: handle={prox_handle} source={prox_source}")
    print(f"Selected color/vision: handle={color_handle} source={color_source}")

    if prox_handle is None and color_handle is None:
        raise SystemExit("No proximity or color/vision sensor handle found.")

    delay = 1.0 / max(args.hz, 0.1)
    sample = 0
    while True:
        t = time.time()

        prox_raw = None
        if prox_handle is not None:
            prox_raw = try_call(sim.readProximitySensor, prox_handle)

        vision_raw = None
        avg_rgb = None
        if color_handle is not None:
            vision_raw = normalize_read_vision_result(
                try_call(sim.readVisionSensor, color_handle)
            )
            avg_rgb = average_rgb_from_image(sim, color_handle)

        if avg_rgb is None:
            avg_text = "avg_rgb=None"
        else:
            r, g, b, resolution = avg_rgb
            avg_text = (
                f"avg_rgb=({r:.4f}, {g:.4f}, {b:.4f}) res={resolution[0]}x{resolution[1]}"
            )

        print(
            f"[{sample:05d}] t={t:.3f} "
            f"prox_raw={format_raw(prox_raw)} "
            f"vision_raw={format_raw(vision_raw)} "
            f"{avg_text}",
            flush=True,
        )
        sample += 1
        time.sleep(delay)


if __name__ == "__main__":
    main()
