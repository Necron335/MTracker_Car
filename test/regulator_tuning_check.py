"""Opt-in ROS/PTY tuning integration check; never opens a real serial device.

Source install/setup.bash, then run with the absolute driver executable path.
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
from rclpy.executors import SingleThreadedExecutor
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from regulator_tuning import RosTelemetryBackend, run_experiment  # noqa: E402


def crc(data):
    """Firmware CRC16."""
    value = 65535
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x8005 if value & 0x8000 else 0)) & 65535
    return value


def main():
    """Run five short physical-response emulations through the real driver."""
    master, slave = pty.openpty()
    finished = threading.Event()
    writes, errors = [], []
    drop_reply = threading.Event()

    def firmware():
        buffer = bytearray()
        velocity, position = 0.0, 0.0
        previous = time.monotonic()
        try:
            while not finished.is_set():
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
                    frame = bytes(buffer[:length])
                    del buffer[:length]
                    assert crc(frame[1:-2]) == int.from_bytes(frame[-2:], 'big')
                    command, robot, sequence = frame[3:6]
                    data = frame[6:-2]
                    if command == 5:
                        now = time.monotonic()
                        dt, previous = now-previous, now
                        enabled = int.from_bytes(data[:2], 'little') & 3 == 3
                        # Current repository protocol negates velocity on the wire.
                        requested = -struct.unpack_from('<h', data, 2)[0]/256*0.033
                        reference = requested if enabled else 0.0
                        velocity += (1-math.exp(-dt/0.04))*(reference-velocity)
                        position += velocity*dt
                        x = int(position*32768).to_bytes(3, 'little', signed=True)
                        reply = (struct.pack('<Hh', 0, round(velocity/0.033*256)) +
                                 x + bytes(3) + struct.pack('<hb', 0, round(reference*500)) +
                                 bytes([75, 0, 0, 0, 0]))
                    elif command == 4:
                        writes.append(struct.unpack('<ffff', data))
                        if drop_reply.is_set():
                            drop_reply.clear()
                            continue  # Applied, but ACK lost: result is unknown to the host.
                        time.sleep(0.12)  # Longer than the motion timeout of 50 ms.
                        reply = bytes(2)+data
                    elif command == 8:
                        raise AssertionError('Unexpected EEPROM save')
                    else:
                        raise AssertionError(f'Unexpected command {command}')
                    raw = bytes([len(reply)+4, 1, command, robot, sequence])+reply
                    os.write(master, bytes([170])+raw+crc(raw).to_bytes(2, 'big'))
        except Exception as error:
            errors.append(str(error))

    thread = threading.Thread(target=firmware)
    thread.start()
    rclpy.init()
    node = rclpy.create_node('tuning_integration_check')
    settings = yaml.safe_load((Path(__file__).resolve().parents[1] /
                               'config/regulator_experiment.yaml').read_text())
    settings.update(baseline_confirmed=True, max_trials=1, min_improvement=0.5,
                    apply_best=False, persist_best=False)
    settings['training'].update(duration=3.0, quiet=0.5)
    settings['validation'].update(kind='ramp', duration=3.0, quiet=0.5)
    backend = RosTelemetryBackend(node, settings)
    # This check bypasses launch, so match its odometry remapping explicitly.
    from nav_msgs.msg import Odometry
    backend.subscriptions.append(node.create_subscription(
        Odometry, '/wheel/odom', backend.on_odometry, 10))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin)
    spinner.start()
    with tempfile.TemporaryDirectory(prefix='regulator_check_') as directory:
        with (Path(directory) / 'driver.log').open('w+') as log:
            process = subprocess.Popen([
                sys.argv[1], '--ros-args', '-p', 'device:='+os.ttyname(slave),
                '-p', 'command_source:=regulator_tuning'], stdout=log, stderr=log)
            try:
                deadline = time.monotonic()+8
                while (not backend.serial_ok or backend.feedback is None or
                       not backend.sample_received) and time.monotonic() < deadline:
                    time.sleep(0.05)
                backend.preflight()
                output = Path(directory) / 'run'
                run_experiment(backend, settings, output, 'tune')
                report = yaml.safe_load((output / 'tuning.yaml').read_text())
                assert report['acquisition'] == 'cmd5_telemetry'
                assert len(report['trials']) == 5
                assert all(t['accepted'] for t in report['trials'])
                assert not report['result']['applied'] and not report['result']['persisted']
                assert len(writes) == 6  # five trials, then restore baseline
                assert (output / 'report.html').exists()
                timer = threading.Timer(0.4, backend.cancelled.set)
                timer.start()
                cancelled_output = Path(directory) / 'cancelled'
                try:
                    run_experiment(backend, settings, cancelled_output, 'tune')
                    raise AssertionError('Cancellation did not abort the experiment')
                except RuntimeError as error:
                    assert 'Cancelled' in str(error), str(error)
                finally:
                    timer.join()
                cancelled = yaml.safe_load((cancelled_output / 'tuning.yaml').read_text())
                assert cancelled['result']['baseline_restored']
                assert not cancelled['result']['applied']
                assert len(cancelled['trials']) == 1
                assert len(writes) == 8  # cancelled trial write, then rollback
                backend.cancelled.clear()
                drop_reply.set()
                lost_output = Path(directory) / 'lost_reply'
                try:
                    run_experiment(backend, settings, lost_output, 'tune')
                    raise AssertionError('Missing reply did not abort the experiment')
                except RuntimeError as error:
                    assert 'CMD=4' in str(error), str(error)
                lost = yaml.safe_load((lost_output / 'tuning.yaml').read_text())
                assert lost['result']['baseline_restored'], lost['result']
                assert not lost['result']['applied']
                assert len(writes) == 10
                assert not errors, errors
                print('PASS: delayed ACK, lost ACK, reconnect, baseline rollback, no EEPROM')
            finally:
                backend.stop()
                process.terminate()
                process.wait(timeout=3)
                log.seek(0)
                print(log.read())
                finished.set()
                thread.join(timeout=2)
                executor.shutdown()
                spinner.join(timeout=2)
                node.destroy_node()
                rclpy.shutdown()
                os.close(master)
                os.close(slave)


if __name__ == '__main__':
    main()
