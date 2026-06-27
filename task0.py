"""
===================================================
    eLSI Sprint 1- [eLSI 2026-27]
===================================================

This script is intended to be a Boilerplate for
Bonus Task 0 of eLSI Sprint 1- [eLSI 2026-27]

Filename:        task0.py
Created:         29/05/2026
Last Modified:   29/05/2026
Author:          e-Yantra Team
Team ID:         [ 403 ]
This software is made available on an "AS IS WHERE IS BASIS".
Licensee/end user indemnifies and will keep e-Yantra indemnified from
any and all claim(s) that emanate from the use of the Software or
breach of the terms of this agreement.

e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*****************************************************************************************
"""

import socket
import threading
import time


class SocketClient:
    """Holds socket client data and sensor information."""

    def __init__(self):
        self.sock = None                 # Socket handle for communication
        self.running = False             # Flag to control thread execution
        self.sensor_values = [0.0] * 32  # Array to store sensor readings (max 32 sensors)
        self.sensor_count = 0            # Actual number of sensors received
        self.recv_thread = None          # Thread for receiving data
        self.control_thread = None       # Thread for control logic


# Global client instance for socket communication
client = SocketClient()


def connect_to_server(c, ip, port):
    """
    Establishes connection to the CoppeliaSim server.

    :param c: SocketClient instance
    :param ip: IP address of the server (typically "127.0.0.1" for localhost)
    :param port: Port number of the server (typically 50002)
    :return: True if connection successful, False if failed
    """
    try:
        # Create TCP socket
        c.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except OSError:
        print("Socket creation failed")
        return False

    # Attempt to connect to server
    try:
        c.sock.connect((ip, port))
    except OSError:
        print("Connection failed")
        c.sock.close()
        c.sock = None
        return False

    c.running = True

    # Start the receive thread to handle incoming sensor data
    c.recv_thread = threading.Thread(target=receive_loop, args=(c,), daemon=True)
    c.recv_thread.start()

    return True


def disconnect(c):
    """
    Cleanly disconnects from the server and cleans up resources.

    :param c: SocketClient instance
    """
    c.running = False  # Signal threads to stop

    # Wait for receive thread to finish
    if c.recv_thread is not None:
        c.recv_thread.join()

    # Close socket if open
    if c.sock is not None:
        c.sock.close()
        c.sock = None


def set_motor(c, left, right):
    """
    Sends motor control commands to the robot.

    :param c: SocketClient instance
    :param left: Left motor speed ( where negative values reverse direction)
    :param right: Right motor speed ( where negative values reverse direction)

    Command format: "L:<left_speed>;R:<right_speed>\\n"
    Example: "L:0.5;R:0.3\\n" sets left motor to 50% forward, right motor to 30% forward
    """
    if c.sock is not None:
        cmd = "L:{:f};R:{:f}\n".format(left, right)
        try:
            c.sock.sendall(cmd.encode())
        except OSError:
            pass


def receive_loop(c):
    """
    Thread function that continuously receives sensor data from the server.

    This function runs in a separate thread and parses incoming sensor data.
    Expected data format: "S:<sensor1>,<sensor2>,<sensor3>,...\\n"
    Example: "S:0.125,0.0,1.0,0.5\\n" represents 4 sensor values
    """
    while c.running:
        # Read data from socket
        try:
            data = c.sock.recv(2048)
        except OSError:
            data = b""

        if data:
            buffer = data.decode(errors="ignore")

            # Check if this is sensor data (starts with "S:")
            if buffer.startswith("S:"):
                values = buffer[2:]                  # Skip the "S:" prefix
                tokens = values.split(",")           # Split by commas

                idx = 0
                # Parse each sensor value
                for token in tokens:
                    if idx >= 32:
                        break
                    token = token.strip()
                    if token == "":
                        continue
                    try:
                        c.sensor_values[idx] = float(token)
                        idx += 1
                    except ValueError:
                        # atof() returns 0.0 for unparseable input
                        c.sensor_values[idx] = 0.0
                        idx += 1
                c.sensor_count = idx  # Store the number of sensors received

        time.sleep(0.05)  # Small delay to prevent excessive CPU usage


def control_loop(c):
    """
    Main control loop thread for robot behavior.

    This is where you should implement your robot's control logic.
    The function runs continuously while the client is connected.

    Available functions for control:
    - set_motor(c, left_speed, right_speed): Control motor speeds
    - Access sensor data via: c.sensor_values[index] and c.sensor_count
    """
    BLACK = 0.2  # Example threshold for black line detection (adjust based on your sensors)
    while c.running:
        if c.sensor_count >=5:
            left = c.sensor_values[1]
            center = c.sensor_values[2]
            right = c.sensor_values[3]

            if center < BLACK:
                set_motor(c,1.5,1.5) #go straight
            elif left < BLACK:
                set_motor(c,0.5,1.5)     #turn left
            elif right < BLACK:
                set_motor(c,1.5,0.5)     #turn right
            else:
                set_motor(c,0.0,0.0)     #stop
        time.sleep(0.1)  # Control loop delay (adjust as needed)
        # TODO: Implement your robot control logic here
        #
        # Examples of what you can do:
        #
        # Basic movement:
        #    set_motor(c, 1.5, 1.5)    # Move forward
        #    set_motor(c, -1.5, 1.5)   # Turn left
        #    set_motor(c, 1.5, -1.5)   # Turn right
        #    set_motor(c, -1.5, -1.5)  # Move backward
        #for _ in range(4):
          #  set_motor(c,1.5,1.5)
          #  time.sleep(2.0)

          #  set_motor(c,-1.5,-1.5)
          #  time.sleep(1.0)

        # Default behavior: Stop the robot
        # Replace this with your control logic
        #set_motor(c, 0.0,0.0)
        #break # Wait 2 seconds before next iteration


def main():
    """
    Main function - Entry point of the program.

    This function:
    1. Connects to the CoppeliaSim server
    2. Starts the control thread for robot behavior
    3. Continuously displays sensor data
    4. Handles cleanup when program exits
    """
    # Attempt to connect to CoppeliaSim server
    # Default: localhost (127.0.0.1) on port 50002
    if not connect_to_server(client, "127.0.0.1", 50002):
        print("Failed to connect to CoppeliaSim server. Make sure:")
        print("1. CoppeliaSim is running")
        print("2. The simulation scene is loaded")
        print("3. The ZMQ remote API is enabled on port 50002")
        return -1

    print("Successfully connected to CoppeliaSim server!")
    print("Starting control thread...")

    # Start the control thread for robot behavior
    client.control_thread = threading.Thread(target=control_loop, args=(client,), daemon=True)
    client.control_thread.start()

    # Main loop: Display sensor data continuously
    print("Monitoring sensor data... (Press Ctrl+C to exit)")
    try:
        while True:
            # Display sensor data if available
            if client.sensor_count > 0:
                values = " ".join("{:.3f}".format(client.sensor_values[i])
                                  for i in range(client.sensor_count))
                print("Sensors ({}): {} ".format(client.sensor_count, values))
            else:
                print("Waiting for sensor data...")

            time.sleep(0.2)  # Update display every 200ms
    except KeyboardInterrupt:
        # Cleanup on Ctrl+C
        print("Disconnecting...")
        disconnect(client)

    return 0


if __name__ == "__main__":
    main()
