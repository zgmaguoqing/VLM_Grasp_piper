#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import time


class ArmDirectClient(Node):
    def __init__(self):
        super().__init__('arm_direct_client')

        # 关节名字，和 /arm_controller/state 里一致
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # 创建 FollowJointTrajectory 的 ActionClient
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.get_logger().info('ArmDirectClient created, waiting for server...')

    def send_joint_goal(self, positions, duration=3.0):
        # 等待 Action Server 可用
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('arm_controller/follow_joint_trajectory server not available!')
            return

        self.get_logger().info(f'Sending joint goal: {positions}')

        # 构造 JointTrajectory
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration - int(duration)) * 1e9),
        )
        traj.points.append(point)

        # 构造 FollowJointTrajectory.Goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        # 异步发送 goal
        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=2.0)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected by server')
            return

        self.get_logger().info('Arm goal accepted, waiting a bit for motion...')
        # 这里不强制等 result，只是等一会儿
        time.sleep(duration + 1.0)


def main():
    rclpy.init()
    node = ArmDirectClient()

    # 1. 回 home
    home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node.send_joint_goal(home, duration=3.0)

    # 2. 发一个明显不同的姿态
    q_test = [0.0, 1.0, -1.0, 1.0, 0.5, 0.0]
    node.send_joint_goal(q_test, duration=3.0)

    # 3. 再回 home
    node.send_joint_goal(home, duration=3.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
