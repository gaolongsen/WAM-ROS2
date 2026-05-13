#!/usr/bin/python3
import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from wam_srvs.srv import JointMove


class WamJointNudge(Node):
    def __init__(self, joint_index, delta):
        super().__init__('wam_joint_nudge')
        self.joint_index = joint_index
        self.delta = delta
        self.current = None
        self.move_done = True
        self.sub = self.create_subscription(
            JointState, '/wam/joint_states', self._joint_state_cb, 10)
        self.done_sub = self.create_subscription(
            Bool, '/wam/move_is_done', self._move_done_cb, 10)
        self.client = self.create_client(JointMove, '/wam/joint_move')

    def _joint_state_cb(self, msg):
        if len(msg.position) >= 7:
            self.current = list(msg.position[:7])

    def _move_done_cb(self, msg):
        self.move_done = msg.data

    def run(self):
        deadline = self.get_clock().now().nanoseconds + int(5e9)
        while rclpy.ok() and self.current is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > deadline:
                raise RuntimeError('Timed out waiting for /wam/joint_states')

        target = list(self.current)
        target[self.joint_index] += self.delta

        self.get_logger().info(f'Current joints: {[round(v, 5) for v in self.current]}')
        self.get_logger().info(f'Target joints:  {[round(v, 5) for v in target]}')

        if not self.client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('/wam/joint_move service is not available')

        req = JointMove.Request()
        req.joints = target
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            raise RuntimeError('Service call failed or timed out')

        self.get_logger().info('JointMove request accepted')

        # The wam_node service starts a non-blocking libbarrett moveTo().
        # Wait here so repeated diagnostic calls do not interrupt each other.
        saw_motion = False
        deadline = self.get_clock().now().nanoseconds + int(20e9)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.move_done:
                saw_motion = True
            if saw_motion and self.move_done:
                break
            if self.get_clock().now().nanoseconds > deadline:
                self.get_logger().warn('Timed out waiting for /wam/move_is_done')
                break

        final = list(self.current)
        error = [target[i] - final[i] for i in range(7)]
        self.get_logger().info(f'Final joints:  {[round(v, 5) for v in final]}')
        self.get_logger().info(f'Final error:   {[round(v, 5) for v in error]}')


def main():
    parser = argparse.ArgumentParser(
        description='Nudge one WAM joint by a small delta using /wam/joint_move.')
    parser.add_argument('joint', type=int, choices=range(1, 8), metavar='JOINT',
                        help='1-based joint index, from 1 to 7')
    parser.add_argument('delta', type=float,
                        help='delta in radians, for example 0.02 or -0.02')
    args = parser.parse_args()

    rclpy.init()
    node = WamJointNudge(args.joint - 1, args.delta)
    try:
        node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
