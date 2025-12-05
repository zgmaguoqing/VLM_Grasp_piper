#!/usr/bin/env python3
import time
from main_vlm import PiperRosEnv  # 确保这里 import 的就是你刚改好的那个类

import os
import sys
import cv2
import numpy as np
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
from manipulator_grasp.env.piper_ros_env import PiperRosEnv

# --- 路径设置 ---
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline'))           
sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'models'))  
sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'dataset')) 
sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'utils'))   
sys.path.append(os.path.join(ROOT_DIR, 'manipulator_grasp'))

from vlm_process import segment_image
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

from grasp_process import run_grasp_inference, execute_grasp

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge

import numpy as np
import threading
import time


class PiperRosEnv:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('piper_vlm_sim_env')

        # 相机用 BEST_EFFORT，和 gazebo_ros_* 插件默认配置兼容
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.bridge = CvBridge()
        self.latest_color_img = None
        self.latest_depth_img = None

        # ========= 1. 彩色图订阅 =========
        self.sub_color = self.node.create_subscription(
            Image,
            '/sim_cam/rgb_camera/image_raw',     # <<—— 用这个
            self.color_callback,
            qos_profile
        )

        # ========= 2. 深度图订阅 =========
        self.sub_depth = self.node.create_subscription(
            Image,
            '/sim_cam/my_depth_camera/depth/image_raw',
            self.depth_callback,
            qos_profile
        )

        # ========= 3. 机械臂 FollowJointTrajectory =========
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.traj_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.node.get_logger().info("[PiperRosEnv] USING DEPTH CAMERA VERSION (rgb_camera + my_camera/depth)")
        time.sleep(1.0)

    def color_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_color_img = img
            # 调试想看就打开
            # self.node.get_logger().info(f"Got COLOR frame {img.shape}")
        except Exception as e:
            self.node.get_logger().error(f"Color Error: {e}")

    def depth_callback(self, msg):
        try:
            encoding = msg.encoding

            # 1) 先按编码读出原始 depth（单位：米）
            if encoding == "32FC1":
                depth = self.bridge.imgmsg_to_cv2(msg, "32FC1").astype(np.float32)
            elif encoding == "16UC1":
                depth_raw = self.bridge.imgmsg_to_cv2(msg, "16UC1").astype(np.float32)
                depth = depth_raw / 1000.0  # mm -> m
            else:
                self.node.get_logger().warn(
                    f"Unknown depth encoding {encoding}, using passthrough"
                )
                depth = self.bridge.imgmsg_to_cv2(msg, "passthrough").astype(np.float32)

            # 2) 清理非法值：NaN, ±Inf
            bad_mask = ~np.isfinite(depth)
            if np.any(bad_mask):
                depth[bad_mask] = 0.0

            # 3) 把 <=0 的深度视为无效点
            depth[depth <= 0.0] = 0.0

            # 4) 把等于 far (3.0m) 或更远的视为“没打到物体”，也置为 0
            #   这里 3.0 要和你 SDF 里的 <far> 保持一致
            FAR = 3.0
            depth[depth >= FAR] = 0.0

            self.latest_depth_img = depth

            # 5) 只统计有效点（>0 且 finite）来打印 min/max，方便你观察
            valid = depth > 0.0
            if np.any(valid):
                dmin = float(depth[valid].min())
                dmax = float(depth[valid].max())
                nz = int(valid.sum())
            else:
                dmin = 0.0
                dmax = 0.0
                nz = 0

            self.node.get_logger().info(
                f"Got DEPTH frame (cleaned): shape={depth.shape}, "
                f"valid_pixels={nz}, min={dmin:.3f}, max={dmax:.3f}"
            )

        except Exception as e:
            self.node.get_logger().error(f"Depth Error: {e}")


    def render(self):
        """返回一帧 RGB 和 Depth，没有的话返回 None"""
        if self.latest_color_img is None:
            print("[Env] No COLOR image yet")
            return None

        if self.latest_depth_img is None:
            print("[Env] No DEPTH image yet, using zeros")
            h, w = self.latest_color_img.shape[:2]
            depth_res = np.zeros((h, w), dtype=np.float32)
        else:
            depth_res = self.latest_depth_img

        return {
            'img': self.latest_color_img,
            'depth': depth_res
        }

    # ======= 你原来的 reset / move_joints 逻辑照旧，只是别删掉上面的相机部分就行 =======


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
def main():
    env = PiperRosEnv()
    print("[TEST] Waiting for images from Gazebo...")

    for i in range(50):  # 等大概 10 秒
        obs = env.render()
        if obs is not None:
            color = obs['img']
            depth = obs['depth']
            print(f"[TEST] Got obs: color={color.shape}, depth={depth.shape}, "
                  f"depth_min={depth.min():.3f}, depth_max={depth.max():.3f}")
            break
        time.sleep(0.2)

    print("[TEST] Done.")

if __name__ == "__main__":
    main()
