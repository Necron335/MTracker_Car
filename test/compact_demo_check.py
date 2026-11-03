"""ROS/PTY smoke check for the short steering demo and its RViz marker stream.

Usage after sourcing the workspace: python3 test/compact_demo_check.py DRIVER DEMO
No physical serial device is opened.
"""

import math
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray


def crc(data):
    value = 65535
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x8005 if value & 0x8000 else 0)) & 65535
    return value


def main():
    master, slave = pty.openpty()
    stop = threading.Event()
    errors = []

    def firmware():
        buffer = bytearray()
        x = y = yaw = velocity = 0.0
        previous = time.monotonic()
        try:
            while not stop.is_set():
                if not select.select([master], [], [], 0.05)[0]:
                    continue
                buffer.extend(os.read(master, 4096))
                while len(buffer) >= 2:
                    if buffer[0] != 170:
                        del buffer[0]
                        continue
                    length = buffer[1]+4
                    if len(buffer) < length:
                        break
                    packet = bytes(buffer[:length])
                    del buffer[:length]
                    assert crc(packet[1:-2]) == int.from_bytes(packet[-2:], 'big')
                    command, robot, sequence = packet[3:6]
                    assert command == 5, 'Demo must not change configuration'
                    data = packet[6:-2]
                    enabled = int.from_bytes(data[:2], 'little') & 3 == 3
                    reference = -struct.unpack_from('<h', data, 2)[0]/256*0.033 if enabled else 0
                    angle = struct.unpack_from('<h', data, 4)[0]/4096 if enabled else 0
                    current = time.monotonic()
                    dt, previous = current-previous, current
                    velocity += (1-math.exp(-dt/0.08))*(reference-velocity)
                    turn = velocity*math.tan(angle)/0.19*dt
                    x += velocity*dt*math.cos(yaw+turn/2)
                    y += velocity*dt*math.sin(yaw+turn/2)
                    yaw += turn
                    reply = (struct.pack('<Hh', 0, round(velocity/0.033*256)) +
                             int(x*32768).to_bytes(3, 'little', signed=True) +
                             int(y*32768).to_bytes(3, 'little', signed=True) +
                             struct.pack('<hb', round(yaw*1024), round(reference*200)) +
                             bytes([75, 0, 0, 0, 0]))
                    raw = bytes([22, 1, command, robot, sequence])+reply
                    os.write(master, bytes([170])+raw+crc(raw).to_bytes(2, 'big'))
        except Exception as error:
            errors.append(str(error))

    thread = threading.Thread(target=firmware)
    thread.start()
    rclpy.init()
    node = rclpy.create_node('compact_demo_check')
    statuses, markers, positions = [], [], []
    node.create_subscription(String, '/compact_demo/status', lambda msg: statuses.append(msg.data), 20)
    node.create_subscription(MarkerArray, '/compact_demo/markers', markers.append, 10)
    node.create_subscription(Odometry, '/wheel/odom',
                             lambda msg: positions.append((msg.pose.pose.position.x,
                                                           msg.pose.pose.position.y)), 20)
    with tempfile.TemporaryDirectory(prefix='compact_demo_check_') as directory:
        with (Path(directory)/'nodes.log').open('w+') as log:
            driver = subprocess.Popen(
                [sys.argv[1], '--ros-args', '-p', 'device:='+os.ttyname(slave),
                 '-p', 'command_timeout:=0.15'], stdout=log, stderr=log)
            demo = subprocess.Popen(
                [sys.argv[2], '--ros-args', '-p', 'enable_motion:=true',
                 '-r', 'odom:=/wheel/odom'], stdout=log, stderr=log)
            try:
                client = node.create_client(Trigger, '/compact_demo/start')
                assert client.wait_for_service(timeout_sec=8)
                deadline = time.monotonic()+2
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done() and future.result().success, str(future.result())
                deadline = time.monotonic()+60
                while not any(s.startswith(('completed;', 'stopped;')) for s in statuses):
                    assert time.monotonic() < deadline, statuses
                    rclpy.spin_once(node, timeout_sec=0.05)
                assert any(s.startswith('completed;') for s in statuses), statuses
                assert markers and len(markers[-1].markers) == 6
                distance = sum(math.hypot(b[0]-a[0], b[1]-a[1])
                               for a, b in zip(positions, positions[1:]))
                assert 4.5 < distance < 5.5, distance
                origin = positions[0]
                assert max(math.hypot(x-origin[0], y-origin[1]) for x, y in positions) < 0.4
                assert not errors, errors
                print(f'PASS: 16 compact passes, RViz markers, total path={distance:.3f} m')
            finally:
                demo.terminate()
                demo.wait(timeout=5)
                driver.terminate()
                driver.wait(timeout=5)
                stop.set()
                thread.join(timeout=2)
                node.destroy_node()
                rclpy.shutdown()
                log.seek(0)
                print(log.read())
                os.close(master)
                os.close(slave)
                assert demo.returncode == 0, 'Demo did not shut down cleanly'


if __name__ == '__main__':
    main()
