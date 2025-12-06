import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge

import numpy as np
import time
import threading


class PiperRosEnv:
    def __init__(self):
        # === 初始化 ROS2 Node ===
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('piper_vlm_sim_env')

        # === 相机 QoS：Gazebo 通常用 BEST_EFFORT ===
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()
        self.latest_color_img = None
        self.latest_depth_img = None

        # === 1. 订阅彩色图 & 深度图 ===
        self.sub_color = self.node.create_subscription(
            Image,
            '/sim_cam/my_camera/image_raw',
            self.color_callback,
            img_qos
        )

        # 深度先用同一话题占位
        self.sub_depth = self.node.create_subscription(
            Image,
            '/sim_cam/my_camera/image_raw',
            self.depth_callback,
            img_qos
        )

        # === 2. 订阅 joint_states，用于 get_joint() ===
        self.latest_joint_state = None
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # === 3. 机械臂控制 Publisher ===
        self.traj_pub = self.node.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # 这里 joint_names 必须和 /arm_controller/state 中的 joint_names 一致
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # === 4. 夹爪控制 Publisher ===
        self.gripper_pub = self.node.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        # TODO: 这里把名字改成 /gripper_controller/state 里真正的 joint_names
        self.gripper_joint_names = ['gripper_finger1_joint', 'gripper_finger2_joint']

        # 简单的 home 位姿
        self.home_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # === 5. Executor 线程（保证订阅一直运行） ===
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        print("[INFO] Simulation Env Initialized with QoS=BEST_EFFORT.")
        time.sleep(1.0)

    # ================= 相机回调 & 渲染 =================

    def color_callback(self, msg):
        try:
            self.latest_color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.node.get_logger().error(f"Color Error: {e}")

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.latest_depth_img = depth.copy()
        except Exception as e:
            self.node.get_logger().error(f"Depth Error: {e}")

    def render(self):
        if self.latest_color_img is None:
            return None

        depth_res = self.latest_depth_img
        if depth_res is None:
            h, w = self.latest_color_img.shape[:2]
            depth_res = np.zeros((h, w), dtype=np.float32)

        return {
            'img': self.latest_color_img,
            'depth': depth_res
        }

    # ================= joint_states & get_joint =================

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def get_joint(self):
        """
        返回当前 6 个关节角度（按 self.joint_names 顺序）。
        如果还没收到 joint_states，就用 home_q 兜底。
        """
        if self.latest_joint_state is None:
            print("[PiperRosEnv] get_joint: no joint_states yet, use home_q")
            return list(self.home_q)

        name_to_pos = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        q = [float(name_to_pos.get(name, 0.0)) for name in self.joint_names]
        print("[PiperRosEnv] get_joint:", q)
        return q

    # ================= 机械臂控制接口 =================

    def move_joints(self, joint_positions, duration=2.0):
        """
        发送 JointTrajectory 到 /arm_controller/joint_trajectory
        joint_positions: 长度 6 的关节角数组
        """
        print("[PiperRosEnv] move_joints called, q =", joint_positions, "duration =", duration)

        msg = JointTrajectory()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration - int(duration)) * 1e9),
        )
        msg.points.append(point)

        self.traj_pub.publish(msg)

    def reset(self):
        print("[PiperRosEnv] reset: go to home pose", self.home_q)
        self.move_joints(self.home_q, duration=3.0)
        time.sleep(1.0)

    # ================= 夹爪控制接口 =================

    def gripper_control(self, open_ratio: float):
        """
        open_ratio: 0.0 = 闭合, 1.0 = 打开
        """
        print("[PiperRosEnv] gripper_control called, open_ratio =", open_ratio)

        # 这个范围你后面可以根据实际 gripper 行程调整
        min_pos = 0.0
        max_pos = 0.03
        target = min_pos + (max_pos - min_pos) * float(open_ratio)
        positions = [target] * len(self.gripper_joint_names)

        msg = JointTrajectory()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.joint_names = self.gripper_joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(point)

        self.gripper_pub.publish(msg)

    # ================= 资源清理 =================

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
