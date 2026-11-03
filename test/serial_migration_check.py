# Standalone ROS/PTY integration check. Source the built workspace first.
# Usage: python3 test/serial_migration_check.py /absolute/path/to/mtrackercar_drv
# Requires ROS 2 network access and a writable ROS_LOG_DIR.
import os
import pty
import select
import struct
import subprocess
import sys
import threading
import time

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import rclpy
from std_srvs.srv import Trigger


def crc(data):
    n = 65535
    for x in data:
        n ^= x << 8
        for _ in range(8):
            n = ((n << 1) ^ (0x8005 if n & 0x8000 else 0)) & 65535
    return n


master, slave = pty.openpty()
path = os.ttyname(slave)
commands = []
errors = []
stop = threading.Event()
scenario = sys.argv[2] if len(sys.argv) > 2 else 'normal'


def firmware():
    buf = bytearray()
    save_count = 0
    query_count = 0
    while not stop.is_set():
        if not select.select([master], [], [], 0.05)[0]:
            continue
        buf.extend(os.read(master, 4096))
        while len(buf) >= 2:
            if buf[0] != 170:
                del buf[0]
                continue
            size = buf[1] + 4
            if len(buf) < size:
                break
            frame = bytes(buf[:size])
            del buf[:size]
            if crc(frame[1:-2]) != int.from_bytes(frame[-2:], 'big'):
                errors.append('request CRC')
                continue
            cmd, robot, seq = frame[3:6]
            data = frame[6:-2]
            commands.append((cmd, data))
            if cmd == 5:
                assert len(data) == 16
                response = bytes(18)
            elif cmd == 13:
                assert len(data) == 16
                response = bytes(2) + data
            elif cmd == 8:
                if data[0] == 1:
                    query_count += 1
                    if scenario == 'stuck' or (scenario == 'lost_accept' and query_count <= 2):
                        continue
                    response = bytes(2)
                else:
                    save_count += 1
                    if scenario == 'lost_accept' and save_count == 2:
                        continue
                    response = struct.pack('<H', 3 if save_count == 1 else 4)
            else:
                errors.append('unknown command')
                continue
            raw = bytes([len(response) + 4, 1, cmd, robot, seq]) + response
            os.write(master, bytes([170]) + raw + crc(raw).to_bytes(2, 'big'))


thread = threading.Thread(target=firmware)
thread.start()
rclpy.init()
node = rclpy.create_node('migration_check')
received = []
sub = node.create_subscription(Odometry, '/wheel/odom', received.append, 10)
pub = node.create_publisher(Twist, '/cmd_vel', 10)
drive_pub = node.create_publisher(AckermannDriveStamped, '/drive_command', 10)


def drive_message(speed, angle):
    message = AckermannDriveStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = 'base_link'
    message.drive.speed = speed
    message.drive.steering_angle = angle
    return message
process = subprocess.Popen(
    [sys.argv[1], '--ros-args', '-p', 'device:=' + path, '-p', 'robot_address:=1',
     '-p', 'command_source:=' + ('steering_angle' if scenario == 'angle' else 'cmd_vel')],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
try:
    deadline = time.monotonic() + 8
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert received, 'no odometry'
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = 0.1
    for _ in range(5):
        if scenario == 'angle':
            drive_pub.publish(drive_message(0.1, 0.25))
        else:
            pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
    assert any(
        cmd == 5 and data[0] == 11 and
        int.from_bytes(data[2:4], 'little', signed=True) < 0
        for cmd, data in commands), 'no angle motion'
    if scenario == 'angle':
        assert any(cmd == 5 and data[0] == 11 and
                   int.from_bytes(data[4:6], 'little', signed=True) == 1024
                   for cmd, data in commands), 'physical angle was scaled or negated'
        assert pub.get_subscription_count() == 0, 'normal angle mode also accepts cmd_vel'
        drive_pub.publish(drive_message(-0.05, -10.0))
        for _ in range(8):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert any(cmd == 5 and data[0] == 11 and
                   int.from_bytes(data[4:6], 'little', signed=True) == -1065
                   for cmd, data in commands), 'angle limit was not applied'
        for fault in ('stale', 'future', 'zero', 'frame', 'nan'):
            invalid = drive_message(0.1, 0.2)
            if fault == 'stale':
                invalid.header.stamp.sec -= 2
            elif fault == 'future':
                invalid.header.stamp.sec += 2
            elif fault == 'zero':
                invalid.header.stamp.sec = invalid.header.stamp.nanosec = 0
            elif fault == 'frame':
                invalid.header.frame_id = 'odom'
            else:
                invalid.drive.speed = float('nan')
            drive_pub.publish(invalid)
            for _ in range(6):
                rclpy.spin_once(node, timeout_sec=0.025)
            latest = [data for cmd, data in commands if cmd == 5][-1]
            assert latest[2:6] == bytes(4), 'invalid Ackermann command moved robot: '+fault
    for service in ['/set_steering_calibration', '/save_configuration']:
        client = node.create_client(Trigger, service)
        assert client.wait_for_service(timeout_sec=3)
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=8)
        assert future.done(), 'service did not return within its deadline'
        if scenario == 'stuck' and service == '/save_configuration':
            assert not future.result().success
            assert 'result unknown' in future.result().message
        else:
            assert future.result().success, str(future.result())
    ids = [cmd for cmd, _ in commands]
    index = ids.index(8)
    assert ids[index:index+3] == [8, 8, 8], ids
    assert commands[index+2][1][0] == 1
    saves = [data for cmd, data in commands if cmd == 8]
    assert sum(data[0] == 0 for data in saves) == 2, 'save initiation was repeated'
    if scenario == 'lost_accept':
        assert sum(data[0] == 1 for data in saves) >= 3
    assert not errors, errors
    print('PASS: serial motion, odometry, calibration, EEPROM scenario=' + scenario)
finally:
    process.terminate()
    output = process.communicate(timeout=3)[0].decode()
    stop.set()
    thread.join()
    node.destroy_node()
    rclpy.shutdown()
    print(output)
