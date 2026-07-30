#!/usr/bin/env python3
"""Print Task 2B proximity sensor configuration via CoppeliaSim ZMQ.

This script does not read bridge_v1_2b or the Python client. It queries the
sensor object itself, including its detection entity/collection and volume
properties.
"""

import argparse

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


DEFAULT_PROX_PATH = "/LineTracer/proximitySensor"
BOX_HANDLES = {"red_box": 43, "blue_box": 44}


def safe_getattr(obj, name):
    try:
        return getattr(obj, name)
    except AttributeError:
        return None


def call_or_error(fn, *args):
    if fn is None:
        return None, "api function missing"
    try:
        return fn(*args), None
    except Exception as exc:
        return None, f"{type(exc).__name__}: {exc}"


def get_object(sim, path):
    value, error = call_or_error(safe_getattr(sim, "getObject"), path, {"noError": True})
    if error is None and value not in (None, -1):
        return value
    value, error = call_or_error(safe_getattr(sim, "getObject"), path)
    if error is None and value not in (None, -1):
        return value
    raise SystemExit(f"Could not resolve proximity sensor path {path!r}: {error}")


def label_for(sim, handle):
    aliases = []
    get_alias = safe_getattr(sim, "getObjectAlias")
    if get_alias is not None:
        for option in (1, 0, 3, 4, -1):
            value, error = call_or_error(get_alias, handle, option)
            if error is None and isinstance(value, str) and value and value not in aliases:
                aliases.append(value)
    return " | ".join(aliases) if aliases else f"handle={handle}"


def get_param(sim, handle, constant_name, getter_names):
    constant = safe_getattr(sim, constant_name)
    if constant is None:
        return None, f"constant sim.{constant_name} missing"
    errors = []
    for getter_name in getter_names:
        getter = safe_getattr(sim, getter_name)
        value, error = call_or_error(getter, handle, constant)
        if error is None:
            return value, None
        errors.append(f"{getter_name}: {error}")
    return None, "; ".join(errors)


def get_property(sim, handle, property_name):
    generic = safe_getattr(sim, "getProperty")
    value, error = call_or_error(generic, handle, property_name, {"noError": True})
    if error is None and value is not None:
        return value, None

    getters = (
        "getFloatProperty",
        "getIntProperty",
        "getBoolProperty",
        "getFloatArrayProperty",
        "getIntArrayProperty",
        "getVector3Property",
        "getColorProperty",
    )
    errors = []
    for getter_name in getters:
        getter = safe_getattr(sim, getter_name)
        value, error = call_or_error(getter, handle, property_name, {"noError": True})
        if error is None and value is not None:
            return value, None
        if error is not None:
            errors.append(f"{getter_name}: {error}")

    return None, "; ".join(errors) if errors else "property not available"


def print_property(sim, handle, property_name):
    value, error = get_property(sim, handle, property_name)
    if error is None:
        print(f"  {property_name}: {value!r}")
    else:
        print(f"  {property_name}: <unavailable> ({error})")
    return value


def try_collection_objects(sim, handle):
    value, error = call_or_error(safe_getattr(sim, "getCollectionObjects"), handle)
    if error is None and isinstance(value, list):
        return value, None
    return None, error or "not a collection"


def try_object_type(sim, handle):
    return call_or_error(safe_getattr(sim, "getObjectType"), handle)


def special_property_bits(sim, handle):
    getter = safe_getattr(sim, "getObjectSpecialProperty")
    value, error = call_or_error(getter, handle)
    if error is not None:
        return None, error

    bit_names = (
        "objectspecialproperty_detectable",
        "objectspecialproperty_detectable_all",
        "objectspecialproperty_ultrasonic_detectable",
        "objectspecialproperty_infrared_detectable",
        "objectspecialproperty_laser_detectable",
        "objectspecialproperty_inductive_detectable",
        "objectspecialproperty_capacitive_detectable",
    )
    set_bits = []
    for name in bit_names:
        bit = safe_getattr(sim, name)
        if isinstance(bit, int) and value & bit:
            set_bits.append(f"sim.{name}({bit})")
    return (value, set_bits), None


def print_box_membership(sim, collection_objects, entity):
    print("Box membership/checks:")
    for name, handle in BOX_HANDLES.items():
        label = label_for(sim, handle)
        obj_type, obj_type_error = try_object_type(sim, handle)
        special, special_error = special_property_bits(sim, handle)

        if collection_objects is not None:
            membership = handle in collection_objects
        elif entity == -1:
            membership = "not scoped: sensor entity_to_detect is -1"
        else:
            membership = handle == entity

        print(f"  {name}: handle={handle} label={label}")
        if obj_type_error is None:
            print(f"    objectType: {obj_type}")
        else:
            print(f"    objectType: <unavailable> ({obj_type_error})")
        print(f"    in selected detection entity/collection: {membership}")
        if special_error is None:
            value, set_bits = special
            print(f"    specialProperty: {value} set_bits={set_bits}")
        else:
            print(f"    specialProperty: <unavailable> ({special_error})")


def main():
    parser = argparse.ArgumentParser(
        description="Inspect /LineTracer/proximitySensor config directly via CoppeliaSim."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--prox-path", default=DEFAULT_PROX_PATH)
    args = parser.parse_args()

    client = RemoteAPIClient(host=args.host, port=args.port)
    sim = client.require("sim")

    prox_handle = get_object(sim, args.prox_path)
    print(f"Proximity sensor: path={args.prox_path!r} handle={prox_handle}")
    print(f"  label: {label_for(sim, prox_handle)}")

    print("\nLegacy proximity object parameters:")
    volume_type, volume_type_error = get_param(
        sim,
        prox_handle,
        "proxintparam_volume_type",
        ("getObjectInt32Param", "getObjectInt32Parameter"),
    )
    if volume_type_error is None:
        print(f"  sim.proxintparam_volume_type: {volume_type!r}")
    else:
        print(f"  sim.proxintparam_volume_type: <unavailable> ({volume_type_error})")

    entity, entity_error = get_param(
        sim,
        prox_handle,
        "proxintparam_entity_to_detect",
        ("getObjectInt32Param", "getObjectInt32Parameter"),
    )
    if entity_error is None:
        print(f"  sim.proxintparam_entity_to_detect: {entity!r}")
    else:
        print(f"  sim.proxintparam_entity_to_detect: <unavailable> ({entity_error})")
        entity = None

    print("\nProximity sensor properties:")
    offset = print_property(sim, prox_handle, "volume_offset")
    range_depth = print_property(sim, prox_handle, "volume_range")
    close_threshold = print_property(sim, prox_handle, "closeThreshold")
    print_property(sim, prox_handle, "sensorType")
    print_property(sim, prox_handle, "explicitHandling")
    print_property(sim, prox_handle, "frontFaceDetection")
    print_property(sim, prox_handle, "backFaceDetection")
    print_property(sim, prox_handle, "exactMode")
    print_property(sim, prox_handle, "showVolume")
    print_property(sim, prox_handle, "volume_radius")
    print_property(sim, prox_handle, "volume_angle")
    print_property(sim, prox_handle, "volume_xSize")
    print_property(sim, prox_handle, "volume_ySize")
    print_property(sim, prox_handle, "volume_faces")
    print_property(sim, prox_handle, "volume_subdivisions")

    print("\nInterpreted detection range:")
    if isinstance(offset, (int, float)) and isinstance(range_depth, (int, float)):
        print(f"  volume starts at offset: {offset:.6f} m")
        print(f"  volume ends at offset + range: {offset + range_depth:.6f} m")
    else:
        print("  could not compute offset + range from properties")
    if isinstance(close_threshold, (int, float)) and close_threshold > 0:
        print(f"  detections below closeThreshold are ignored: {close_threshold:.6f} m")
    else:
        print("  closeThreshold is disabled or unavailable")

    print("\nDetection entity / collection:")
    collection_objects = None
    if entity is None:
        print("  entity_to_detect unavailable, cannot determine scope")
    elif entity == -1:
        print("  entity_to_detect = -1: sensor is configured to detect all detectable objects")
    else:
        obj_type, obj_type_error = try_object_type(sim, entity)
        collection_objects, collection_error = try_collection_objects(sim, entity)
        if collection_objects is not None:
            print(f"  entity {entity} behaves as a collection")
            print(f"  collection object count: {len(collection_objects)}")
            print(f"  collection object handles: {collection_objects}")
            for member in collection_objects:
                print(f"    {member}: {label_for(sim, member)}")
        elif obj_type_error is None:
            print(f"  entity {entity} behaves as a single scene object")
            print(f"  objectType: {obj_type}")
            print(f"  label: {label_for(sim, entity)}")
        else:
            print(f"  entity {entity} is neither a readable object nor collection")
            print(f"  getObjectType error: {obj_type_error}")
            print(f"  getCollectionObjects error: {collection_error}")

    print()
    print_box_membership(sim, collection_objects, entity)


if __name__ == "__main__":
    main()
