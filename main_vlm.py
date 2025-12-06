# # Group Arm
import os
import sys
import cv2
# # import mujoco
import matplotlib.pyplot as plt 
# import time

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'utils'))
sys.path.append(os.path.join(ROOT_DIR, 'manipulator_grasp'))

# from manipulator_grasp.env.ur5_grasp_env import UR5GraspEnv
# from manipulator_grasp.env.piper_ros_env import PiperRosEnv
from vlm_process import segment_image
from grasp_process import run_grasp_inference, execute_grasp


# # 全局变量
# global color_img, depth_img, env
# color_img = None
# depth_img = None
# env = None


# #获取彩色和深度图像数据
# def get_image(env):
#     global color_img, depth_img
#      # 从环境渲染获取图像数据
#     imgs = env.render()

#     # 提取彩色和深度图像数据
#     color_img = imgs['img']   # 这是RGB格式的图像数据
#     depth_img = imgs['depth'] # 这是深度数据

#     # 将RGB图像转换为OpenCV常用的BGR格式
#     color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)

#     return color_img, depth_img

# #构造回调函数，不断调用
# def callback(color_frame, depth_frame):
#     global color_img, depth_img
#     scaling_factor_x = 1
#     scaling_factor_y = 1

#     color_img = cv2.resize(
#         color_frame, None,
#         fx=scaling_factor_x,
#         fy=scaling_factor_y,
#         interpolation=cv2.INTER_AREA
#     )
#     depth_img = cv2.resize(
#         depth_frame, None,
#         fx=scaling_factor_x,
#         fy=scaling_factor_y,
#         interpolation=cv2.INTER_NEAREST
#     )

#     if color_img is not None and depth_img is not None:
#         test_grasp()


# def test_grasp():
#     global color_img, depth_img, env

#     if color_img is None or depth_img is None:
#         print("[WARNING] Waiting for image data...")
#         return

#     # 图像处理部分
#     masks = segment_image(color_img)  

#     gg = run_grasp_inference(color_img, depth_img, masks)

#     execute_grasp(env, gg)



# if __name__ == '__main__':
    
#     # env = UR5GraspEnv()
#     env = PiperRosEnv()
#     env.reset()
    
#     while True:

#         for i in range(500): # 1000
#             env.step()

#         color_img, depth_img = get_image(env)

#         callback(color_img, depth_img)


#     env.close()

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
        # ROS2 初始化
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('piper_vlm_sim_env')

        # ===== 相机 QoS =====
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.bridge = CvBridge()
        self.latest_color_img = None
        self.latest_depth_img = None

        # ======== 1. 订阅彩色图 =========
        # 对应你现在的 topic: /sim_cam/rgb_camera/image_raw
        # ======== 1. 订阅彩色图 =========
        # 有的 SDF 会发 /sim_cam/rgb_camera/image_raw
        # 有的会发 /sim_cam/my_camera/image_raw
        # 我们两个都订阅，只要有一个在发就能收到
        self.sub_color_rgb = self.node.create_subscription(
            Image,
            '/sim_cam/rgb_camera/image_raw',
            self.color_callback,
            qos_profile
        )
        self.sub_color_my = self.node.create_subscription(
            Image,
            '/sim_cam/my_camera/image_raw',
            self.color_callback,
            qos_profile
        )


        # ======== 2. 订阅深度图 =========
        # 对应 camera_spawner 里 remapping 的：/sim_cam/my_camera/depth/image_raw
        self.sub_depth = self.node.create_subscription(
            Image,
            '/sim_cam/my_depth_camera/depth/image_raw',
            self.depth_callback,
            qos_profile
        )
        # ======== 3. 机械臂控制：FollowJointTrajectory Action ========
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        # 这里就是你 controller 的 action 名
        self.traj_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # ======== 4. home 姿态（修复 home_q 报错的关键） ========
        self.home_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # ======== 5. rclpy executor 线程 ========
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.node.get_logger().info(
            "[PiperRosEnv] USING DEPTH CAMERA VERSION (rgb_camera + my_camera/depth)"
        )
        time.sleep(1.0)

    # =================== 相机回调 ===================
    def color_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_color_img = img
            # # 调试：每收到一帧彩色图就打一条
            # self.node.get_logger().info(
            #     f"Got COLOR frame: shape={img.shape}, encoding={msg.encoding}"
            # )
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

            # # 调试
            # self.node.get_logger().info(
            #     f"Got DEPTH frame (cleaned): shape={depth.shape}, "
            #     f"valid_pixels={nz}, min={dmin:.3f}, max={dmax:.3f}"
            # )

        except Exception as e:
            self.node.get_logger().error(f"Depth Error: {e}")



    def render(self):
        """返回 {'img': BGR图, 'depth': float32深度图}，如果彩色图还没来就返回 None"""
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

    # =================== 机械臂控制 ===================

    def send_joint_goal(self, positions, duration=3.0):
        """通过 FollowJointTrajectory 发送一个关节目标"""
        if not self.traj_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error(
                'FollowJointTrajectory server not available on /arm_controller/follow_joint_trajectory'
            )
            return

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        # === 关键修改：因为后台线程已经在 spin，这里不能再 spin_until_future_complete ===
        future = self.traj_client.send_goal_async(goal_msg)
        
        # 手动等待 future 完成
        # 这里的 timeout 只是为了防止死锁，不是动作执行时间
        start_time = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - start_time > 2.0:
                self.node.get_logger().error('Send goal timed out!')
                return

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error('Arm goal rejected by server')
            return
            
        self.node.get_logger().info(
            f"[PiperRosEnv] Joint goal accepted: {positions}"
        )
        # 注意：这里我们不等待动作真正执行完 (get_result)，而是直接返回，让外面 sleep 等待
        # 这样更符合原来 move_joints 的设计意图


    def move_joints(self, joint_positions, duration=3.0):
        """给 execute_grasp 用的统一接口"""
        print("[PiperRosEnv] move_joints called, q =", joint_positions,
              "duration =", duration)
        self.send_joint_goal(joint_positions, duration=duration)
        # 简单等一下，给机械臂时间执行
        time.sleep(duration + 0.5)

    def reset(self):
        """回到 home 位姿"""
        print("[PiperRosEnv] reset: go to home pose", self.home_q)
        self.move_joints(self.home_q, duration=3.0)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    env = PiperRosEnv()
    env.reset()
    debug_img_dir = os.path.join(ROOT_DIR, "debug_img")
    os.makedirs(debug_img_dir, exist_ok=True)


    print("\n" + "="*40)
    print("操作指南 (请严格遵守):")
    print("1. 等待 'Sim Camera' 窗口出现")
    print("2. 【重要】用鼠标点击那个窗口！")
    print("3. 按 'g' 键开始抓取")
    print("="*40 + "\n")

    last_log_time = time.time()

    try:
        while True:
            data = env.render()
            
            # 没图的时候打印等待
            if data is None:
                if time.time() - last_log_time > 1.0:
                    print("Waiting for image data source...")
                    last_log_time = time.time()
                time.sleep(0.1)
                continue

            color_img = data['img']
            depth_img = data['depth']

            # --- 在画面上写字提示你 ---
            display_img = color_img.copy()
            cv2.putText(display_img, "CLICK ME then press 'g'", (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.imshow("Sim Camera", display_img)

            # 检测按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('g'):
                print("\n>>> [按键检测成功!] 开始推理抓取...")
                try:
                    # ===== 1) SAM 分割，拿第一个 mask 作为目标 =====
                    masks = segment_image(color_img)
                    if isinstance(masks, (list, tuple)):
                        if len(masks) == 0:
                            print("[WARN] segment_image 返回空列表，改用全图作为掩码")
                            mask_for_grasp = None
                        else:
                            mask_for_grasp = masks[0]
                    else:
                        mask_for_grasp = masks

                    # ===== 2) 保存当前 RGB / 深度 / RGB+mask 图片 =====
                    ts = int(time.time())

                    # 2.1 保存原始 RGB 图 (BGR 格式没关系，只是看用)
                    rgb_path = os.path.join(debug_img_dir, f"rgb_{ts}.png")
                    cv2.imwrite(rgb_path, color_img)
                    print(f"[DEBUG] 保存 RGB: {rgb_path}")

                    # 2.2 保存深度可视化图（归一化到 0~255 灰度）
                    depth_vis = depth_img.copy().astype(np.float32)
                    depth_vis = np.nan_to_num(depth_vis, nan=0.0, posinf=0.0, neginf=0.0)
                    d_min, d_max = depth_vis.min(), depth_vis.max()
                    if d_max > d_min:
                        depth_norm = ((depth_vis - d_min) / (d_max - d_min) * 255.0).astype(np.uint8)
                    else:
                        depth_norm = np.zeros_like(depth_vis, dtype=np.uint8)
                    depth_path = os.path.join(debug_img_dir, f"depth_{ts}.png")
                    cv2.imwrite(depth_path, depth_norm)
                    print(f"[DEBUG] 保存深度可视化: {depth_path}")

                    # 2.3 如果有 mask，把 mask 叠加到 RGB 上保存一张
                    if mask_for_grasp is not None:
                        mask = mask_for_grasp
                        if mask.dtype != np.uint8:
                            mask_u8 = (mask > 0).astype(np.uint8) * 255
                        else:
                            mask_u8 = mask

                        mask_color = cv2.applyColorMap(mask_u8, cv2.COLORMAP_JET)
                        overlay = cv2.addWeighted(color_img, 0.7, mask_color, 0.3, 0)
                        overlay_path = os.path.join(debug_img_dir, f"rgb_mask_{ts}.png")
                        cv2.imwrite(overlay_path, overlay)
                        print(f"[DEBUG] 保存 RGB+mask: {overlay_path}")

                    # ===== 3) 调用 GraspNet + 执行抓取 =====
                    gg = run_grasp_inference(color_img, depth_img, mask_for_grasp)
                    print(">>> 推理完成，执行抓取...")
                    execute_grasp(env, gg)

                except Exception as e:
                    print(f"!!! 抓取流程报错: {e}")
                    import traceback
                    traceback.print_exc()


    except KeyboardInterrupt:
        pass
    finally:
        env.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()