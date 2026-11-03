#!/usr/bin/env python3
"""Repeated compact forward/reverse passes with steering changes and RViz."""

import math
import time


class Maneuvers:
    """Eight forward/reverse pairs; stop only for reversal or completion."""

    def __init__(self, speed=0.25):
        if not math.isfinite(speed) or not 0 < speed <= 0.25:
            raise ValueError('Use speed in (0, 0.25] m/s')
        self.speed = speed
        self.plan = [(1, 0.26), (-1, -0.26)]*8
        self.step = 0
        self.phase = 'ready'
        self.distance = self.leg_distance = self.opposite = 0.0
        self.previous = self.origin = None
        self.stable = None
        self.phase_start = self.started = None

    @property
    def active(self):
        return self.phase in ('move', 'pause')

    def start(self, pose, now):
        if self.phase != 'ready':
            raise ValueError('Restart the demo node for another run')
        self.origin = self.previous = pose
        self.phase = 'move'
        self.phase_start = self.started = now

    def observe(self, pose, velocity, now):
        if not self.active:
            return
        if not all(math.isfinite(v) for v in (*pose, velocity)):
            raise ValueError('Invalid odometry')
        dx, dy = pose[0]-self.previous[0], pose[1]-self.previous[1]
        distance = math.hypot(dx, dy)
        if distance > 0.04:
            raise ValueError('Odometry discontinuity')
        self.distance += distance
        self.leg_distance += distance
        if self.phase == 'move':
            signed = dx*math.cos(self.previous[2])+dy*math.sin(self.previous[2])
            self.opposite += max(0.0, -self.plan[self.step][0]*signed)
            if self.opposite > 0.015:
                raise ValueError('Motion opposite to requested direction')
        self.previous = pose
        if self.distance >= 6.0 or math.hypot(pose[0]-self.origin[0], pose[1]-self.origin[1]) >= 0.40:
            raise ValueError('Demo odometry distance limit reached')
        if abs(velocity) > 0.45:
            raise ValueError('Excessive measured speed')
        if self.phase == 'pause':
            if abs(velocity) <= 0.01:
                if self.stable is None:
                    self.stable = now
            else:
                self.stable = None

    def command(self, now, velocity):
        if not self.active:
            return 0.0, 0.0
        if now-self.started > 60.0:
            raise ValueError('Demo duration limit reached')
        angle = self.plan[self.step][1]*(1 if self.leg_distance < 0.15 else -1)
        if self.phase == 'pause':
            if now-self.phase_start > 4.0:
                raise ValueError('Robot did not stop before reversing')
            if (self.stable is None or now-self.stable < 0.3 or now-self.phase_start < 0.7):
                return 0.0, angle
            if self.step == len(self.plan)-1:
                self.phase = 'completed'
                return 0.0, 0.0
            self.step += 1
            self.leg_distance = self.opposite = 0.0
            self.phase = 'move'
            self.phase_start = now
            angle = self.plan[self.step][1]
        # Change angle halfway through each pass without stopping the drive.
        stop_at = 0.30-abs(velocity)*0.08
        if self.leg_distance >= stop_at:
            self.phase = 'pause'
            self.phase_start = now
            self.stable = None
            return 0.0, angle
        if now-self.phase_start > 3.0:
            raise ValueError('No sufficient motion during demo section')
        return self.plan[self.step][0]*self.speed, angle


def main():
    import rclpy
    from ackermann_msgs.msg import AckermannDriveStamped
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Point, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init()
    node = Node('compact_demo')
    enabled = node.declare_parameter('enable_motion', False).value
    sequence = Maneuvers(node.declare_parameter('speed', 0.25).value)
    publisher = node.create_publisher(AckermannDriveStamped, 'drive_command', 1)
    status_pub = node.create_publisher(String, 'compact_demo/status', 10)
    marker_pub = node.create_publisher(MarkerArray, 'compact_demo/markers', 10)
    static_tf = StaticTransformBroadcaster(node)
    feedback = {'pose': None, 'velocity': 0.0, 'received': 0.0, 'serial': 0.0,
                'serial_ok': False, 'frame': None, 'anchor': None}
    last_status = [None]
    steering = [0.0]

    def status(message):
        node.get_logger().info(message)
        status_pub.publish(String(data=message))

    def send(speed=0.0, angle=0.0):
        message = AckermannDriveStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.drive.speed, message.drive.steering_angle = speed, angle
        publisher.publish(message)
        steering[0] = angle

    def stop(reason):
        sequence.phase = 'stopped'
        send()
        status(reason+f'; measured total distance={sequence.distance:.3f} m')

    def on_odom(message):
        try:
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            age = node.get_clock().now().nanoseconds*1e-9-stamp
            if stamp <= 0 or not -0.05 <= age <= 0.10:
                raise ValueError('Stale odometry')
            if not message.header.frame_id or message.child_frame_id != 'base_link':
                raise ValueError('Expected odometry for base_link')
            if feedback['frame'] and feedback['frame'] != message.header.frame_id:
                raise ValueError('Odometry frame changed')
            p, q = message.pose.pose.position, message.pose.pose.orientation
            norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
            if not math.isfinite(norm) or abs(norm-1) > 0.01:
                raise ValueError('Invalid odometry quaternion')
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            pose = p.x, p.y, yaw
            velocity = message.twist.twist.linear.x
            if not all(math.isfinite(v) for v in (*pose, velocity)):
                raise ValueError('Nonfinite odometry')
            sequence.observe(pose, velocity, time.monotonic())
            feedback.update(pose=pose, velocity=velocity, received=time.monotonic(),
                            frame=message.header.frame_id)
            if feedback['anchor'] is None:
                feedback['anchor'] = pose
                transform = TransformStamped()
                transform.header = message.header
                transform.child_frame_id = 'compact_demo_start'
                transform.transform.translation.x, transform.transform.translation.y = p.x, p.y
                transform.transform.rotation = q
                static_tf.sendTransform(transform)
        except ValueError as error:
            feedback['received'] = 0.0
            if sequence.active:
                stop(str(error))

    def on_diagnostics(message):
        for item in message.status:
            if item.name == 'mtrackercar/serial':
                feedback['serial_ok'] = item.level == item.OK
                feedback['serial'] = time.monotonic()

    def check_feedback():
        if feedback['pose'] is None or time.monotonic()-feedback['received'] > 0.10:
            raise ValueError('Fresh odometry required')
        if not feedback['serial_ok'] or time.monotonic()-feedback['serial'] > 2.5:
            raise ValueError('Confirmed serial feedback required')

    def start(request, response):
        del request
        try:
            if not enabled:
                raise ValueError('Launch with enable_motion:=true')
            if sequence.phase != 'ready':
                raise ValueError('Restart the demo node for another run')
            if publisher.get_subscription_count() != 1 or node.count_publishers('drive_command') != 1:
                raise ValueError('Require one angle-mode driver and no other drive publisher')
            check_feedback()
            if abs(feedback['velocity']) > 0.01:
                raise ValueError('Stop robot before starting demo')
            x, y, yaw = feedback['pose']
            anchor = TransformStamped()
            anchor.header.frame_id = feedback['frame']
            anchor.header.stamp = node.get_clock().now().to_msg()
            anchor.child_frame_id = 'compact_demo_start'
            anchor.transform.translation.x, anchor.transform.translation.y = x, y
            anchor.transform.rotation.z, anchor.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
            static_tf.sendTransform(anchor)
            sequence.start(feedback['pose'], time.monotonic())
            response.success = True
            response.message = (f'Started 8 forward/reverse pairs; target speed {sequence.speed:.2f} m/s; '
                                '16 passes of about 0.30 m, workspace radius limit 0.40 m')
            status(response.message)
        except ValueError as error:
            response.message = str(error)
        return response

    def cancel(request, response):
        del request
        stop('Cancelled')
        response.success = True
        response.message = 'Stop command sent'
        return response

    def markers():
        result = MarkerArray()
        parts = [(0, 0.095, 0.0, 0.25, 0.13, False),
                 (1, 0.0, 0.08, 0.055, 0.025, False),
                 (2, 0.0, -0.08, 0.055, 0.025, False),
                 (3, 0.19, 0.08, 0.055, 0.025, True),
                 (4, 0.19, -0.08, 0.055, 0.025, True)]
        for index, x, y, length, width, front in parts:
            item = Marker()
            item.header.frame_id = 'base_link'
            item.ns, item.id, item.type, item.action = 'robot_schematic', index, Marker.CUBE, Marker.ADD
            item.pose.position.x, item.pose.position.y = x, y
            item.pose.position.z = 0.025
            angle = steering[0] if front else 0.0
            item.pose.orientation.z, item.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
            item.scale.x, item.scale.y, item.scale.z = length, width, 0.04
            item.color.r, item.color.g, item.color.b, item.color.a = (
                (1.0, 0.6, 0.0, 1.0) if front else (0.1, 0.5, 0.9, 0.65))
            result.markers.append(item)
        boundary = Marker()
        boundary.header.frame_id = 'compact_demo_start'
        boundary.ns, boundary.id, boundary.type = 'odom_limit', 0, Marker.LINE_STRIP
        boundary.pose.orientation.w = 1.0
        boundary.scale.x = 0.003
        boundary.color.r, boundary.color.g, boundary.color.a = 1.0, 0.8, 0.8
        boundary.points = [Point(x=0.4*math.cos(i*2*math.pi/96),
                                 y=0.4*math.sin(i*2*math.pi/96)) for i in range(97)]
        result.markers.append(boundary)
        marker_pub.publish(result)

    def update():
        if sequence.active:
            try:
                check_feedback()
                send(*sequence.command(time.monotonic(), feedback['velocity']))
            except ValueError as error:
                stop(str(error))
        elif sequence.phase in ('completed', 'stopped'):
            send()
        if sequence.phase != last_status[0]:
            status(f'{sequence.phase}; pass={sequence.step+1}/{len(sequence.plan)}; '
                   f'distance={sequence.distance:.3f} m')
            last_status[0] = sequence.phase

    node.create_subscription(Odometry, 'odom', on_odom, 20)
    node.create_subscription(DiagnosticArray, '/diagnostics', on_diagnostics, 10)
    node.create_service(Trigger, '~/start', start)
    node.create_service(Trigger, '~/cancel', cancel)
    node.create_timer(0.02, update)
    node.create_timer(0.1, markers)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            send()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
