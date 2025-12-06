# import os
# import sys
# import numpy as np
# import torch
# import open3d as o3d
# from PIL import Image
# import spatialmath as sm

# from manipulator_grasp.arm.motion_planning import *

# from graspnetAPI import GraspGroup

# ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
# sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'models'))
# sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'dataset'))
# sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'utils'))
# sys.path.append(os.path.join(ROOT_DIR, 'manipulator_grasp'))

# from graspnet import GraspNet, pred_decode
# from graspnet_dataset import GraspNetDataset
# from collision_detector import ModelFreeCollisionDetector
# from data_utils import CameraInfo, create_point_cloud_from_depth_image


# # ==================== 网络加载 ====================
# def get_net():
#     """
#     加载训练好的 GraspNet 模型
#     """
#     net = GraspNet(input_feature_dim=0, 
#                    num_view=300, 
#                    num_angle=12, 
#                    num_depth=4,
#                    cylinder_radius=0.05, 
#                    hmin=-0.02, 
#                    hmax_list=[0.01, 0.02, 0.03, 0.04], 
#                    is_training=False)
#     net.to(torch.device('cuda:0' if torch.cuda.is_available() else 'cpu'))
#     checkpoint = torch.load('./logs/log_rs/checkpoint-rs.tar') # checkpoint_path
#     net.load_state_dict(checkpoint['model_state_dict'])
#     net.eval()
#     return net




# # ================= 数据处理并生成输入 ====================
# def get_and_process_data(color_path, depth_path, mask_path):
#     """
#     根据给定的 RGB 图、深度图、掩码图（可以是 文件路径 或 NumPy 数组），生成输入点云及其它必要数据
#     """
# #---------------------------------------
#     # 1. 加载 color（可能是路径，也可能是数组）
#     if isinstance(color_path, str):
#         color = np.array(Image.open(color_path), dtype=np.float32) / 255.0
#     elif isinstance(color_path, np.ndarray):
#         color = color_path.astype(np.float32)
#         color /= 255.0
#     else:
#         raise TypeError("color_path 既不是字符串路径也不是 NumPy 数组！")

#     # 2. 加载 depth（可能是路径，也可能是数组）
#     if isinstance(depth_path, str):
#         depth_img = Image.open(depth_path)
#         depth = np.array(depth_img)
#     elif isinstance(depth_path, np.ndarray):
#         depth = depth_path
#     else:
#         raise TypeError("depth_path 既不是字符串路径也不是 NumPy 数组！")

#     # 3. 加载 mask（可能是路径，也可能是数组）
#     if isinstance(mask_path, str):
#         workspace_mask = np.array(Image.open(mask_path))
#     elif isinstance(mask_path, np.ndarray):
#         workspace_mask = mask_path
#     else:
#         raise TypeError("mask_path 既不是字符串路径也不是 NumPy 数组！")

#     # print("\n=== 尺寸验证 ===")
#     # print("深度图尺寸:", depth.shape)
#     # print("颜色图尺寸:", color.shape[:2])
#     # print("工作空间尺寸:", workspace_mask.shape)

#     # 构造相机内参矩阵
#     height = color.shape[0]
#     width = color.shape[1]
#     fovy = np.pi / 4 # 定义的仿真相机
#     focal = height / (2.0 * np.tan(fovy / 2.0))  # 焦距计算（基于垂直视场角fovy和高度height）
#     c_x = width / 2.0   # 水平中心
#     c_y = height / 2.0  # 垂直中心
#     intrinsic = np.array([
#         [focal, 0.0, c_x],    
#         [0.0, focal, c_y],   
#         [0.0, 0.0, 1.0]
#     ])
#     factor_depth = 1.0  # 深度因子，根据实际数据调整

#     # 利用深度图生成点云 (H,W,3) 并保留组织结构
#     camera = CameraInfo(width, height, intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2], factor_depth)
#     cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

#     # mask = depth < 2.0
#     mask = (workspace_mask > 0) & (depth < 2.0)
#     cloud_masked = cloud[mask]
#     color_masked = color[mask]
#     # print(f"mask过滤后的点云数量 (color_masked): {len(color_masked)}") # 在采样前打印原始过滤后的点数

#     NUM_POINT = 5000 # 10000或5000
#     # 如果点数足够，随机采样NUM_POINT个点（不重复）
#     if len(cloud_masked) >= NUM_POINT:
#         idxs = np.random.choice(len(cloud_masked), NUM_POINT, replace=False)
#     # 如果点数不足，先保留所有点，再随机重复补足NUM_POINT个点
#     else:
#         idxs1 = np.arange(len(cloud_masked))
#         idxs2 = np.random.choice(len(cloud_masked), NUM_POINT - len(cloud_masked), replace=True)
#         idxs = np.concatenate([idxs1, idxs2], axis=0)
    
#     cloud_sampled = cloud_masked[idxs]
#     color_sampled = color_masked[idxs] # 提取点云和颜色

#     cloud_o3d = o3d.geometry.PointCloud()
#     cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
#     cloud_o3d.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))

#     device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
#     cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(device)
#     # end_points = {'point_clouds': cloud_sampled}

#     end_points = dict()
#     end_points['point_clouds'] = cloud_sampled
#     end_points['cloud_colors'] = color_sampled

#     return end_points, cloud_o3d



# # ==================== 主函数：获取抓取预测 ====================
# def run_grasp_inference(color_path, depth_path, sam_mask_path=None):
#     # 1. 加载网络
#     net = get_net()

#     # 2. 处理数据，此处使用返回的工作空间掩码路径
#     end_points, cloud_o3d = get_and_process_data(color_path, depth_path, sam_mask_path)

#     # 3. 前向推理
#     with torch.no_grad():
#         end_points = net(end_points)
#         grasp_preds = pred_decode(end_points)

#     # 4. 构造 GraspGroup 对象（这里 gg 是列表或类似列表的对象）
#     gg = GraspGroup(grasp_preds[0].detach().cpu().numpy())

#     # 5. 碰撞检测
#     COLLISION_THRESH = 0.01
#     if COLLISION_THRESH > 0:
#         voxel_size = 0.01
#         collision_thresh = 0.01
#         mfcdetector = ModelFreeCollisionDetector(np.asarray(cloud_o3d.points), voxel_size=voxel_size)
#         collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=collision_thresh)
#         gg = gg[~collision_mask]

#     # 6. NMS 去重 + 按照得分排序（降序）
#     gg.nms().sort_by_score()

#     # ===== 新增筛选部分：对抓取预测的接近方向进行垂直角度限制 =====
#     # 将 gg 转换为普通列表
#     all_grasps = list(gg)
#     vertical = np.array([0, 0, 1])  # 期望抓取接近方向（垂直桌面）
#     angle_threshold = np.deg2rad(30)  # 30度的弧度值
#     filtered = []
#     for grasp in all_grasps:
#         # 抓取的接近方向取 grasp.rotation_matrix 的第一列
#         approach_dir = grasp.rotation_matrix[:, 0]
#         # 计算夹角：cos(angle)=dot(approach_dir, vertical)
#         cos_angle = np.dot(approach_dir, vertical)
#         cos_angle = np.clip(cos_angle, -1.0, 1.0)
#         angle = np.arccos(cos_angle)
#         if angle < angle_threshold:
#             filtered.append(grasp)
#     if len(filtered) == 0:
#         print("\n[Warning] No grasp predictions within vertical angle threshold. Using all predictions.")
#         filtered = all_grasps
#     else:
#         print(f"\n[DEBUG] Filtered {len(filtered)} grasps within ±30° of vertical out of {len(all_grasps)} total predictions.")

#     # # ===== 新增：利用 SAM 生成的目标掩码过滤抓取预测（投影到图像坐标判断） =====
#     # if sam_mask_path is not None:
#     #     # 加载 SAM 目标掩码
#     #     if isinstance(sam_mask_path, str):
#     #         sam_mask = np.array(Image.open(sam_mask_path))
#     #     elif isinstance(sam_mask_path, np.ndarray):
#     #         sam_mask = sam_mask_path
#     #     else:
#     #         raise TypeError("sam_mask_path 既不是字符串路径也不是 NumPy 数组！")
#     #     # 假定 SAM 掩码与颜色图尺寸一致（640x640）
#     #     height, width = sam_mask.shape[:2]
#     #     # 动态计算相机内参（基于垂直视场角 fovy）
#     #     fovy = np.pi / 4  # 垂直视场角，假设为 45 度
#     #     focal = height / (2.0 * np.tan(fovy / 2.0))  # 焦距计算（像素单位）
#     #     cx = width / 2.0   # 光心 X 坐标（图像中心）
#     #     cy = height / 2.0  # 光心 Y 坐标（图像中心）

#     #     sam_filtered = []
#     #     for grasp in filtered:
#     #         # grasp.translation 为摄像头坐标系下的 3D 坐标 [X, Y, Z]
#     #         X, Y, Z = grasp.translation
#     #         if Z <= 0:
#     #             continue
#     #         u = focal * X / Z + cx
#     #         v = focal * Y / Z + cy
#     #         u_int = int(round(u))
#     #         v_int = int(round(v))
#     #         # 检查投影点是否在图像范围内（640x640）
#     #         if u_int < 0 or u_int >= 640 or v_int < 0 or v_int >= 640:
#     #             continue
#     #         # 若 SAM 掩码中该像素有效（非0），则保留
#     #         if sam_mask[v_int, u_int] > 0:
#     #             sam_filtered.append(grasp)
#     #     if len(sam_filtered) == 0:
#     #         print("\n[Warning] No grasp predictions fall inside the SAM mask. Using previous predictions.")
#     #     else:
#     #         print(f"\n[DEBUG] Filtered {len(sam_filtered)} grasps inside the SAM mask out of {len(filtered)} predictions.")
#     #         filtered = sam_filtered

#     # ===== 新增部分：计算物体中心点 =====
#     # 使用点云计算物体的中心点
#     points = np.asarray(cloud_o3d.points)
#     object_center = np.mean(points, axis=0) if len(points) > 0 else np.zeros(3)

#     # 计算每个抓取位姿中心点与物体中心点的距离
#     distances = []
#     for grasp in filtered:
#         grasp_center = grasp.translation
#         distance = np.linalg.norm(grasp_center - object_center)
#         distances.append(distance)

#     # 创建一个新的排序列表，包含距离和抓取对象
#     grasp_with_distances = [(g, d) for g, d in zip(filtered, distances)]
    
#     # 按距离升序排序（距离越小越好）
#     grasp_with_distances.sort(key=lambda x: x[1])
    
#     # 提取排序后的抓取列表
#     filtered = [g for g, d in grasp_with_distances]

#     # ===== 新增部分：综合得分和距离进行最终排序 =====
#     # 创建一个新的排序列表，包含综合得分和抓取对象
#     # 综合得分 = 抓取得分 * 0.7 + (1 - 距离/最大距离) * 0.3
#     max_distance = max(distances) if distances else 1.0
#     grasp_with_composite_scores = []

#     for g, d in grasp_with_distances:
#         # 归一化距离分数（距离越小分数越高）
#         distance_score = 1 - (d / max_distance)
        
#         # 综合得分 = 抓取得分 * 权重1 + 距离得分 * 权重2
#         composite_score = g.score * 0.1 + distance_score * 0.9
#         # print(f"\n g.score = {g.score}, distance_score = {distance_score}")
#         grasp_with_composite_scores.append((g, composite_score))

#     # 按综合得分降序排序
#     grasp_with_composite_scores.sort(key=lambda x: x[1], reverse=True)

#     # 提取排序后的抓取列表
#     filtered = [g for g, score in grasp_with_composite_scores]


#     # # 对过滤后的抓取根据 score 排序（降序）
#     # filtered.sort(key=lambda g: g.score, reverse=True)

#     # 取第1个抓取
#     top_grasps = filtered[:1]

#     # 可视化过滤后的抓取，手动转换为 Open3D 物体
#     grippers = [g.to_open3d_geometry() for g in top_grasps]

#     # 选择得分最高的抓取（filtered 列表已按得分降序排序）
#     best_grasp = top_grasps[0]
#     best_translation = best_grasp.translation
#     best_rotation = best_grasp.rotation_matrix
#     best_width = best_grasp.width

#     # 创建一个新的 GraspGroup 并添加最佳抓取
#     new_gg = GraspGroup()            # 初始化空的 GraspGroup
#     new_gg.add(best_grasp)           # 添加最佳抓取

#     visual = True
#     if visual:
#         grippers = new_gg.to_open3d_geometry_list()
#         o3d.visualization.draw_geometries([cloud_o3d, *grippers])

#     return new_gg

#     #return best_translation, best_rotation, best_width



# # ================= 仿真执行抓取动作 ====================
# def execute_grasp(env, gg):
#     """
#     执行抓取动作，控制机器人从初始位置移动到抓取位置，并完成抓取操作。

#     参数:
#     env (UR5GraspEnv): 机器人环境对象。
#     gg (GraspGroup): 抓取预测结果。
#     """
#     robot = env.robot
#     T_wb = robot.base

#     # 0.初始准备阶段
#     # 目标：计算抓取位姿 T_wo（物体相对于世界坐标系的位姿）
#     # n_wc = np.array([0.0, -1.0, 0.0]) # 相机朝向
#     # o_wc = np.array([-1.0, 0.0, -0.5]) # 相机朝向 [0.5, 0.0, -1.0] -> [-1.0, 0.0, -0.5]
#     # t_wc = np.array([1.0, 0.6, 2.0]) # 相机的位置。2.0是相机高度，与scene.xml中保持一致。
#     n_wc = np.array([0.0, -1.0, 0.0]) 
#     o_wc = np.array([-1.0, 0.0, -0.5]) 
#     t_wc = np.array([0.85, 0.8, 1.6]) 

#     T_wc = sm.SE3.Trans(t_wc) * sm.SE3(sm.SO3.TwoVectors(x=n_wc, y=o_wc))
#     T_co = sm.SE3.Trans(gg.translations[0]) * sm.SE3(sm.SO3.TwoVectors(x=gg.rotation_matrices[0][:, 0], y=gg.rotation_matrices[0][:, 1]))
#     T_wo = T_wc * T_co

#     action = np.zeros(7)

#     # 1.机器人运动到预抓取位姿
#     # 目标：将机器人从当前位置移动到预抓取姿态（q1）
#     time1 = 1
#     q0 = robot.get_joint()
#     q1 = np.array([0.0, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0])
#     parameter0 = JointParameter(q0, q1)
#     velocity_parameter0 = QuinticVelocityParameter(time1)
#     trajectory_parameter0 = TrajectoryParameter(parameter0, velocity_parameter0)
#     planner1 = TrajectoryPlanner(trajectory_parameter0)
#     # 执行planner_array = [planner1]
#     time_array = [0.0, time1]
#     planner_array = [planner1]
#     total_time = np.sum(time_array)
#     time_step_num = round(total_time / 0.002) + 1
#     times = np.linspace(0.0, total_time, time_step_num)
#     time_cumsum = np.cumsum(time_array)
#     for timei in times:
#         for j in range(len(time_cumsum)):
#             if timei == 0.0:
#                 break
#             if timei <= time_cumsum[j]:
#                 planner_interpolate = planner_array[j - 1].interpolate(timei - time_cumsum[j - 1])
#                 if isinstance(planner_interpolate, np.ndarray):
#                     joint = planner_interpolate
#                     robot.move_joint(joint)
#                 else:
#                     robot.move_cartesian(planner_interpolate)
#                     joint = robot.get_joint()
#                 action[:6] = joint
#                 env.step(action)
#                 break

#     # 2.接近抓取位姿
#     # 目标：从预抓取位姿直线移动到抓取点附近（T2）
#     # 关键点：T2 是 T_wo 沿负 x 方向偏移 0.1m，确保安全接近物体。
#     time2 = 1
#     robot.set_joint(q1)
#     T1 = robot.get_cartesian()
#     T2 = T_wo * sm.SE3(-0.1, 0.0, 0.0)
#     position_parameter1 = LinePositionParameter(T1.t, T2.t) #  位置规划（直线路径）
#     attitude_parameter1 = OneAttitudeParameter(sm.SO3(T1.R), sm.SO3(T2.R)) # 姿态规划（插值旋转）
#     cartesian_parameter1 = CartesianParameter(position_parameter1, attitude_parameter1) # 组合笛卡尔参数
#     velocity_parameter1 = QuinticVelocityParameter(time2) # 速度曲线（五次多项式插值）
#     trajectory_parameter1 = TrajectoryParameter(cartesian_parameter1, velocity_parameter1) # 将笛卡尔空间路径和速度曲线结合，生成完整的轨迹参数
#     planner2 = TrajectoryPlanner(trajectory_parameter1) # 轨迹规划器，将笛卡尔空间路径和速度曲线结合，生成完整的轨迹参数
#     # 执行planner_array = [planner2]
#     time_array = [0.0, time2]
#     planner_array = [planner2]
#     total_time = np.sum(time_array)
#     time_step_num = round(total_time / 0.002) + 1
#     times = np.linspace(0.0, total_time, time_step_num)
#     time_cumsum = np.cumsum(time_array)
#     for timei in times:
#         for j in range(len(time_cumsum)):
#             if timei == 0.0:
#                 break
#             if timei <= time_cumsum[j]:
#                 planner_interpolate = planner_array[j - 1].interpolate(timei - time_cumsum[j - 1])
#                 if isinstance(planner_interpolate, np.ndarray):
#                     joint = planner_interpolate
#                     robot.move_joint(joint)
#                 else:
#                     robot.move_cartesian(planner_interpolate)
#                     joint = robot.get_joint()
#                 action[:6] = joint
#                 env.step(action)
#                 break

#     # 3.执行抓取
#     # 目标：从 T2 移动到 T3（精确抓取位姿）。通过逐步增加 action[-1]（夹爪控制信号）闭合夹爪，抓取物体。
#     time3 = 1
#     T3 = T_wo
#     position_parameter2 = LinePositionParameter(T2.t, T3.t)
#     attitude_parameter2 = OneAttitudeParameter(sm.SO3(T2.R), sm.SO3(T3.R))
#     cartesian_parameter2 = CartesianParameter(position_parameter2, attitude_parameter2)
#     velocity_parameter2 = QuinticVelocityParameter(time3)
#     trajectory_parameter2 = TrajectoryParameter(cartesian_parameter2, velocity_parameter2)
#     planner3 = TrajectoryPlanner(trajectory_parameter2)
#     # 执行planner_array = [planner3]
#     time_array = [0.0, time3]
#     planner_array = [planner3]
#     total_time = np.sum(time_array)
#     time_step_num = round(total_time / 0.002) + 1
#     times = np.linspace(0.0, total_time, time_step_num) 
#     time_cumsum = np.cumsum(time_array)
#     for timei in times:
#         for j in range(len(time_cumsum)): 
#             if timei == 0.0:
#                 break
#             if timei <= time_cumsum[j]:
#                 planner_interpolate = planner_array[j - 1].interpolate(timei - time_cumsum[j - 1])
#                 if isinstance(planner_interpolate, np.ndarray):
#                     joint = planner_interpolate
#                     robot.move_joint(joint)
#                 else:
#                     robot.move_cartesian(planner_interpolate)
#                     joint = robot.get_joint()
#                 action[:6] = joint
#                 env.step(action)
#                 break
#     for i in range(1000):
#         action[-1] += 0.2
#         action[-1] = np.min([action[-1], 255])
#         env.step(action)

#     # 4.提起物体
#     # 目标：抓取后垂直提升物体（避免碰撞桌面）。
#     time4 = 1
#     T4 = sm.SE3.Trans(0.0, 0.0, 0.3) * T3 # 通过在T3的基础上向上偏移0.3单位得到的，用于控制机器人上升一定的高度
#     position_parameter3 = LinePositionParameter(T3.t, T4.t)
#     attitude_parameter3 = OneAttitudeParameter(sm.SO3(T3.R), sm.SO3(T4.R))
#     cartesian_parameter3 = CartesianParameter(position_parameter3, attitude_parameter3)
#     velocity_parameter3 = QuinticVelocityParameter(time4)
#     trajectory_parameter3 = TrajectoryParameter(cartesian_parameter3, velocity_parameter3)
#     planner4 = TrajectoryPlanner(trajectory_parameter3)

#     # 5.水平移动物体
#     # 目标：将物体水平移动到目标放置位置，保持高度不变。
#     time5 = 1
#     T5 = sm.SE3.Trans(1.4, 0.3, T4.t[2]) * sm.SE3(sm.SO3(T4.R)) #  通过在T4的基础上进行平移得到，这里的1.4, 0.3是场景中的固定点坐标，而不是偏移量
#     position_parameter4 = LinePositionParameter(T4.t, T5.t)
#     attitude_parameter4 = OneAttitudeParameter(sm.SO3(T4.R), sm.SO3(T5.R))
#     cartesian_parameter4 = CartesianParameter(position_parameter4, attitude_parameter4)
#     velocity_parameter4 = QuinticVelocityParameter(time5)
#     trajectory_parameter4 = TrajectoryParameter(cartesian_parameter4, velocity_parameter4)
#     planner5 = TrajectoryPlanner(trajectory_parameter4)

#     # 6.放置物体
#     # 目标：垂直下降物体到接触面（T7）。逐步减小 action[-1]（夹爪信号）以释放物体。
#     time6 = 1
#     T6 = sm.SE3.Trans(0.0, 0.0, -0.1) * T5 # 通过在T5的基础上向下偏移0.1单位得到的，用于控制机器人下降一定的高度
#     position_parameter6 = LinePositionParameter(T5.t, T6.t)
#     attitude_parameter6 = OneAttitudeParameter(sm.SO3(T5.R), sm.SO3(T6.R))
#     cartesian_parameter6 = CartesianParameter(position_parameter6, attitude_parameter6)
#     velocity_parameter6 = QuinticVelocityParameter(time6)
#     trajectory_parameter6 = TrajectoryParameter(cartesian_parameter6, velocity_parameter6)
#     planner6 = TrajectoryPlanner(trajectory_parameter6)

#     # 执行planner_array = [planner4, planner5, planner6]
#     time_array = [0.0, time4, time5, time6]
#     planner_array = [planner4, planner5, planner6]
#     total_time = np.sum(time_array)
#     time_step_num = round(total_time / 0.002) + 1
#     times = np.linspace(0.0, total_time, time_step_num)
#     time_cumsum = np.cumsum(time_array)
#     for timei in times:
#         for j in range(len(time_cumsum)):
#             if timei == 0.0:
#                 break
#             if timei <= time_cumsum[j]:
#                 planner_interpolate = planner_array[j - 1].interpolate(timei - time_cumsum[j - 1])
#                 if isinstance(planner_interpolate, np.ndarray):
#                     joint = planner_interpolate
#                     robot.move_joint(joint)
#                 else:
#                     robot.move_cartesian(planner_interpolate)
#                     joint = robot.get_joint()
#                 action[:6] = joint
#                 env.step(action)
#                 break
#     for i in range(1000):
#         action[-1] -= 0.2
#         action[-1] = np.max([action[-1], 0])
#         env.step(action)

#     # 7.抬起夹爪
#     # 目标：放置后抬起夹爪，避免碰撞物体。
#     time7 = 1
#     T7 = sm.SE3.Trans(0.0, 0.0, 0.1) * T6
#     position_parameter7 = LinePositionParameter(T6.t, T7.t)
#     attitude_parameter7 = OneAttitudeParameter(sm.SO3(T6.R), sm.SO3(T7.R))
#     cartesian_parameter7 = CartesianParameter(position_parameter7, attitude_parameter7)
#     velocity_parameter7 = QuinticVelocityParameter(time7)
#     trajectory_parameter7 = TrajectoryParameter(cartesian_parameter7, velocity_parameter7)
#     planner7 = TrajectoryPlanner(trajectory_parameter7)
#     # 执行planner_array = [planner7]
#     time_array = [0.0, time7]
#     planner_array = [planner7]
#     total_time = np.sum(time_array)
#     time_step_num = round(total_time / 0.002) + 1
#     times = np.linspace(0.0, total_time, time_step_num)
#     time_cumsum = np.cumsum(time_array)
#     for timei in times:
#         for j in range(len(time_cumsum)):
#             if timei == 0.0:
#                 break
#             if timei <= time_cumsum[j]:
#                 planner_interpolate = planner_array[j - 1].interpolate(timei - time_cumsum[j - 1])
#                 if isinstance(planner_interpolate, np.ndarray):
#                     joint = planner_interpolate
#                     robot.move_joint(joint)
#                 else:
#                     robot.move_cartesian(planner_interpolate)
#                     joint = robot.get_joint()
#                 action[:6] = joint
#                 env.step(action)
#                 break

#     # 8.回到初始位置
#     # 目标：机器人返回初始姿态（q0），完成整个任务。
#     time8 = 1
#     q8 = robot.get_joint()
#     q9 = q0
#     parameter8 = JointParameter(q8, q9)
#     velocity_parameter8 = QuinticVelocityParameter(time8)
#     trajectory_parameter8 = TrajectoryParameter(parameter8, velocity_parameter8)
#     planner8 = TrajectoryPlanner(trajectory_parameter8)
#     # 执行planner_array = [planner8]
#     time_array = [0.0, time8]
#     planner_array = [planner8]
#     total_time = np.sum(time_array)
#     time_step_num = round(total_time / 0.002) + 1
#     times = np.linspace(0.0, total_time, time_step_num)
#     time_cumsum = np.cumsum(time_array)
#     for timei in times:
#         for j in range(len(time_cumsum)):
#             if timei == 0.0:
#                 break
#             if timei <= time_cumsum[j]:
#                 planner_interpolate = planner_array[j - 1].interpolate(timei - time_cumsum[j - 1])
#                 if isinstance(planner_interpolate, np.ndarray):
#                     joint = planner_interpolate
#                     robot.move_joint(joint)
#                 else:
#                     robot.move_cartesian(planner_interpolate)
#                     joint = robot.get_joint()
#                 action[:6] = joint
#                 env.step(action)
#                 break


import os
import sys
import numpy as np
import torch
import open3d as o3d
from PIL import Image
import spatialmath as sm
import time
import math

# === 如果你没有 ikpy，请 pip install ikpy ===
try:
    import ikpy.chain
    from ikpy.link import OriginLink, URDFLink
except ImportError:
    print("[Warning] ikpy library not found. Please install: pip install ikpy")

from graspnet import GraspNet, pred_decode
from graspnet_dataset import GraspNetDataset
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image
from graspnetAPI import GraspGroup
DEBUG_GRASP_IMG = True  # 想关掉保存图片就改成 False

DEBUG_CLOUD_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "debug_cloud"
)
# === 添加这段，让 Python 能找到 graspnet 相关代码 ===
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'graspnet-baseline', 'utils'))
sys.path.append(os.path.join(ROOT_DIR, 'manipulator_grasp'))
# ==================== 配置部分 ====================
# 请修改为你本地 piper_description/urdf/piper_ros.xacro 对应的纯 urdf 文件路径
# 如果只有 xacro，你需要先把它转成 urdf: ros2 run xacro xacro piper_ros.xacro > piper.urdf
PIPER_URDF_PATH = "/root/piper_ws/piper_ros.urdf"
from ikpy.chain import Chain

JOINT_INDICES = [1, 2, 3, 4, 5, 6]  # joint1~joint6


import os
import time
import open3d as o3d  # 你本来就有的话就不用再加

# === 调试输出路径 & 开关 ===
DEBUG_SAVE_CLOUD = True
DEBUG_CLOUD_DIR = os.path.join(os.path.dirname(__file__), "debug_cloud")

import os
import time
import numpy as np
import open3d as o3d

# 为了能在没有图形界面的环境里保存图片
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 调试输出目录 & 开关
DEBUG_SAVE_CLOUD = True
DEBUG_CLOUD_DIR = os.path.join(os.path.dirname(__file__), "debug_cloud")


def save_grasp_debug_image(cloud_o3d, grasp, tag="cam"):
    """
    把【点云 + 抓取位姿】画成一张 3D 图并保存为 png。

    - cloud_o3d: Open3D 点云（相机坐标系）
    - grasp: 一个 graspnetAPI.Grasp 对象（有 translation, rotation_matrix）
    - tag: 用来区分是相机系/世界系之类，目前就用 "cam" 即可
    """
    if not DEBUG_GRASP_IMG:
        return

    os.makedirs(DEBUG_CLOUD_DIR, exist_ok=True)
    ts = int(time.time())
    out_path = os.path.join(DEBUG_CLOUD_DIR, f"grasp_{tag}_{ts}.png")

    pts = np.asarray(cloud_o3d.points)
    cols = np.asarray(cloud_o3d.colors) if cloud_o3d.has_colors() else None

    if pts.size == 0:
        print("[DEBUG] save_grasp_debug_image: 点云为空，跳过保存图片")
        return

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")

    # 画点云
    if cols is not None and len(cols) == len(pts):
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=cols, s=1)
    else:
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c="gray", s=1)

    # === 画抓取位姿 ===
    center = grasp.translation              # 抓取中心 (3,)
    R = grasp.rotation_matrix               # 3x3 旋转矩阵
    approach = R[:, 0]                      # 通常 x 轴是接近方向
    finger_dir = R[:, 1]                    # y 轴代表两指张开方向

    L = 0.08  # 画出来的线段长度，自己看着调

    # 抓取中心点
    ax.scatter(center[0], center[1], center[2],
               c="red", s=30, label="grasp center")

    # 接近方向 (红线)
    ax.plot(
        [center[0], center[0] + L * approach[0]],
        [center[1], center[1] + L * approach[1]],
        [center[2], center[2] + L * approach[2]],
        color="red", linewidth=2, label="approach"
    )

    # 手指方向 (绿线)
    ax.plot(
        [center[0], center[0] + L * finger_dir[0]],
        [center[1], center[1] + L * finger_dir[1]],
        [center[2], center[2] + L * finger_dir[2]],
        color="green", linewidth=2, label="finger dir"
    )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # 让三个轴比例接近 1:1:1，看着不变形
    try:
        max_range = (pts.max(axis=0) - pts.min(axis=0)).max()
        mid = (pts.max(axis=0) + pts.min(axis=0)) / 2.0
        for axis, m in zip([ax.set_xlim, ax.set_ylim, ax.set_zlim], mid):
            axis(m - max_range / 2, m + max_range / 2)
    except Exception:
        pass

    # 固定一个比较舒服的视角（可以自己改）
    ax.view_init(elev=30, azim=-60)

    plt.tight_layout()
    fig.savefig(out_path, dpi=200)
    plt.close(fig)

    print(f"[DEBUG] 抓取点云位姿图片已保存: {out_path}")
def save_cloud_debug(cloud_o3d, tag="input_cam"):
    """
    同时把点云保存成 .pcd 和 一张 .png 图片（3D 散点图）

    保存位置: grasp_process.py 同级目录下的 ./debug_cloud/
      cloud_<tag>_<timestamp>.pcd
      cloud_<tag>_<timestamp>.png
    """
    if not DEBUG_SAVE_CLOUD:
        return

    os.makedirs(DEBUG_CLOUD_DIR, exist_ok=True)
    ts = int(time.time())

    # === 1) 保存 PCD（可选，用不到也没关系） ===
    pcd_path = os.path.join(DEBUG_CLOUD_DIR, f"cloud_{tag}_{ts}.pcd")
    o3d.io.write_point_cloud(pcd_path, cloud_o3d)

    # === 2) 画成图片并保存 PNG ===
    pts = np.asarray(cloud_o3d.points)
    cols = np.asarray(cloud_o3d.colors) if cloud_o3d.has_colors() else None

    if pts.size == 0:
        print("[DEBUG] save_cloud_debug: 点云为空，跳过保存图片")
        return

    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, projection="3d")

    # 如果有颜色就用点云的颜色，没有就统一成蓝色
    if cols is not None and len(cols) == len(pts):
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=cols, s=1)
    else:
        ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c="b", s=1)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # 固定一个比较舒服的视角（可以自己改）
    ax.view_init(elev=30, azim=-60)

    # 让三个轴比例接近 1:1:1
    try:
        max_range = (pts.max(axis=0) - pts.min(axis=0)).max()
        mid = (pts.max(axis=0) + pts.min(axis_0)) / 2.0
    except Exception:
        max_range = None
        mid = None
    if max_range is not None and max_range > 0:
        for axis, m in zip([ax.set_xlim, ax.set_ylim, ax.set_zlim], mid):
            axis(m - max_range / 2, m + max_range / 2)

    plt.tight_layout()
    png_path = os.path.join(DEBUG_CLOUD_DIR, f"cloud_{tag}_{ts}.png")
    fig.savefig(png_path, dpi=200)
    plt.close(fig)

    print(f"[DEBUG] 点云已保存: {pcd_path}")
    print(f"[DEBUG] 点云截图已保存: {png_path}")

class PiperKine:
    def __init__(self, urdf_path):
        self.valid = False
        if not os.path.exists(urdf_path):
            print(f"[Error] URDF not found at {urdf_path}")
            return

        print(f"[IK] Loading URDF from {urdf_path}")
        self.chain = Chain.from_urdf_file(urdf_path)
        self.joint_indices = JOINT_INDICES

        print("[IK] Links in chain:")
        for i, l in enumerate(self.chain.links):
            print(f"  {i}: {l.name}")

        self.valid = True

    def solve_ik(self, target_pose, current_joints):
        """
        target_pose: spatialmath.SE3 或 4x4 numpy
        current_joints: 长度 6 的当前关节角（作为 initial guess）
        """
        if not self.valid:
            print("[Error] IK model invalid.")
            return [0.0] * 6

        # 转成 4x4 numpy
        if hasattr(target_pose, "A"):
            T = target_pose.A
        else:
            T = np.array(target_pose, dtype=float)

        target_pos = T[:3, 3]
        target_orient = T[:3, :3]

        # initial_position: 和 links 数量一致
        n_links = len(self.chain.links)
        initial = np.zeros(n_links)
        for j, idx in enumerate(self.joint_indices):
            if j < len(current_joints):
                initial[idx] = current_joints[j]

        sol = self.chain.inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_orient,
            orientation_mode="all",
            initial_position=initial,
            max_iter=100,
        )

        # 做一次 FK 检查，只打印误差用于观察，不再强制 skip
        T_fk = self.chain.forward_kinematics(sol)
        pos_err = np.linalg.norm(T_fk[:3, 3] - target_pos)
        rot_err = np.arccos(
            np.clip((np.trace(target_orient.T @ T_fk[:3, :3]) - 1) / 2.0, -1.0, 1.0)
        )
        print(f"[IK] pos_err={pos_err:.4f} m, rot_err={rot_err*180/np.pi:.2f} deg")

        # **调试阶段：即使误差大也先用这个解，让机械臂动起来**
        # if pos_err > 0.20 or rot_err > np.deg2rad(45):
        #     print("[IK] Warning: IK error too large, skip this target.")
        #     return current_joints

        q = [sol[idx] for idx in self.joint_indices]
        print("[IK] q_solution =", q)
        return q



# 全局实例
piper_ik = PiperKine(PIPER_URDF_PATH)


# ==================== 简单的 IK 求解器封装 ====================
# class PiperKine:
#     def __init__(self, urdf_path):
#         self.valid = False
#         if os.path.exists(urdf_path):
#             # 加载 URDF 链 (排除基座和末端虚体，根据实际 urdf 调整 active_links_mask)
#             self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path)
#             self.valid = True
#             print(f"[IK] Piper model loaded from {urdf_path}")
#         else:
#             print(f"[Error] URDF not found at {urdf_path}. IK will fail.")

#     # def solve_ik(self, target_se3, current_joints):
#     #     """
#     #     输入: target_se3 (spatialmath.SE3) 目标位姿
#     #           current_joints (list) 当前关节角度（作为初始猜测）
#     #     输出: list of 6 floats (关节角度)
#     #     """
#     #     if not self.valid:
#     #         print("[Error] IK model invalid.")
#     #         return [0.0]*6

#     #     # spatialmath SE3 转 numpy 4x4
#     #     target_matrix = target_se3.A 
        
#     #     # ikpy 计算 (注意：ikpy 返回的通常包含 base 节点，长度可能比关节数多1)
#     #     # Piper 是 6 自由度
#     #     # ik_solution = self.chain.inverse_kinematics(target_matrix, initial_position=[0] + list(current_joints) + [0])
#     #     # === 修改开始 ===
#     #     # 1. 从 4x4 矩阵中提取位置 (x, y, z)
#     #     target_pos = target_matrix[:3, 3]
        
#     #     # 2. 从 4x4 矩阵中提取旋转矩阵 (3x3)
#     #     target_orient = target_matrix[:3, :3]

#     #     # 3. 显式地分别传入位置和方向
#     #     ik_solution = self.chain.inverse_kinematics(
#     #         target_position=target_pos,
#     #         target_orientation=target_orient,
#     #         orientation_mode="all",  # 强制要求匹配方向
#     #         initial_position=[0] + list(current_joints) + [0]
#     #     )
#     #     # === 修改结束 ===

#     #     # 截取中间的活动关节 (通常 ikpy 第一个是 base，最后一个是 tip)
#     #     # 具体截取范围取决于 urdf 结构，通常是 [1:7] 对应 6 个关节
#     #     joints = ik_solution[1:7] 
#     #     return joints.tolist()
#     def solve_ik(self, target_pose, current_joints):
#         # === 修复部分：类型强制转换 ===
#         # 如果传入的是 spatialmath 的 SE3 对象，它通常有一个 .A 属性或者是 .data 属性
#         # 我们尝试将其转换为标准的 numpy 4x4 数组
#         if hasattr(target_pose, 'A'): 
#             target_pose = target_pose.A  # .A 通常是 spatialmath 转 numpy 的方法
#         elif not isinstance(target_pose, np.ndarray):
#             # 如果既不是 SE3 也不是 numpy，尝试强制转换
#             target_pose = np.array(target_pose)

#         # === 这里的逻辑保持不变 ===
#         # 现在 target_pose 肯定是 numpy 数组了，可以放心切片
#         target_pos = target_pose[:3, 3]
#         target_orient = target_pose[:3, :3]
        
#         # 1. 获取 URDF 链条总共定义的 Link 数量
#         n_links = len(self.chain.links)
        
#         # 2. 创建一个全 0 的列表
#         initial_full_pose = [0] * n_links
        
#         # 3. 将当前的 6 个关节角度填入列表
#         # 索引保护，防止 current_joints 长度超过 n_links
#         for i in range(min(len(current_joints), n_links - 1)):
#             initial_full_pose[i + 1] = current_joints[i]

#         # 4. 执行 IK 解算
#         ik_solution = self.chain.inverse_kinematics(
#             target_position=target_pos,
#             target_orientation=target_orient,
#             orientation_mode="all", 
#             initial_position=initial_full_pose
#         )
        
#         # 5. 提取结果 (取中间的6个关节)
#         return ik_solution[1:7]
# 全局 IK 求解器实例
# piper_ik = PiperKine(PIPER_URDF_PATH)

# ==================== 网络加载 (保持不变) ====================
def get_net():
    net = GraspNet(input_feature_dim=0, num_view=300, num_angle=12, num_depth=4,
                   cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04], 
                   is_training=False)
    net.to(torch.device('cuda:0' if torch.cuda.is_available() else 'cpu'))
    # 请确保路径正确
    checkpoint_path = './logs/log_rs/checkpoint-rs.tar' 
    if os.path.exists(checkpoint_path):
        checkpoint = torch.load(checkpoint_path)
        net.load_state_dict(checkpoint['model_state_dict'])
    net.eval()
    return net

# ================= 数据处理 (微调相机内参) ====================
import cv2  # 开头记得 import

def get_and_process_data(color_path, depth_path, sam_mask=None):
    """
    color_path, depth_path: 路径或 np.ndarray
    sam_mask: np.ndarray（单通道），和 depth 同尺寸；如果为 None 就用整幅图
    """

    # 1. 颜色图
    if isinstance(color_path, str):
        color = np.array(Image.open(color_path).convert('RGB'), dtype=np.float32)
    elif isinstance(color_path, np.ndarray):
        color = color_path.astype(np.float32)
        if color.ndim == 2:
            color = np.stack([color] * 3, axis=-1)
    else:
        raise TypeError("color_path 既不是字符串路径也不是 NumPy 数组！")
    if color.max() > 1.0:
        color /= 255.0

    # 2. 深度图
    if isinstance(depth_path, str):
        depth_img = Image.open(depth_path)
        depth = np.array(depth_img)
    elif isinstance(depth_path, np.ndarray):
        depth = depth_path
    else:
        raise TypeError("depth_path 既不是字符串路径也不是 NumPy 数组！")

    height, width = depth.shape

    # 3. SAM mask 预处理
    if sam_mask is None:
        workspace_mask = np.ones_like(depth, dtype=bool)
    else:
        m = sam_mask
        if isinstance(m, np.ndarray):
            if m.ndim == 3:         # RGB / RGBA -> 取单通道
                m = m[..., 0]
        else:
            raise TypeError("sam_mask 必须是 NumPy 数组")

        # 尺寸不一致就 resize 到 depth 大小
        if m.shape != depth.shape:
            m = cv2.resize(m, (width, height), interpolation=cv2.INTER_NEAREST)

        # 转成布尔 mask
        workspace_mask = (m > 127)

    # ============ 相机内参 ============
    fovy = np.pi / 4  # 你之前定义的仿真相机
    focal = height / (2.0 * np.tan(fovy / 2.0))
    c_x = width / 2.0
    c_y = height / 2.0
    intrinsic = np.array([
        [focal, 0.0, c_x],
        [0.0, focal, c_y],
        [0.0, 0.0, 1.0]
    ])
    factor_depth = 1.0

    camera = CameraInfo(width, height,
                        intrinsic[0, 0], intrinsic[1, 1],
                        intrinsic[0, 2], intrinsic[1, 2],
                        factor_depth)
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # 只保留：mask 内 + 深度 < 2m
    valid_mask = workspace_mask & (depth < 2.0)

    cloud_masked = cloud[valid_mask]
    color_masked = color[valid_mask]

    NUM_POINT = 5000
    if len(cloud_masked) >= NUM_POINT:
        idxs = np.random.choice(len(cloud_masked), NUM_POINT, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked),
                                 NUM_POINT - len(cloud_masked),
                                 replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)

    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    cloud_o3d = o3d.geometry.PointCloud()
    cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud_o3d.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(device)

    end_points = dict()
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    # 把相机内参一并返回，后面投影用
    K = intrinsic
    return end_points, cloud_o3d, K

# ==================== run_grasp_inference (保持不变) ====================
# 你的筛选逻辑写得很好，不需要大改，直接保留即可
def run_grasp_inference(color_path, depth_path, sam_mask_path=None):
    # 1. 预处理 SAM mask（变成单通道数组）
    sam_mask = None
    if sam_mask_path is not None:
        if isinstance(sam_mask_path, str):
            sam_mask_img = Image.open(sam_mask_path)
            sam_mask_img = sam_mask_img.convert('L')  # 强制单通道
            sam_mask = np.array(sam_mask_img)
        elif isinstance(sam_mask_path, np.ndarray):
            sam_mask = sam_mask_path
            if sam_mask.ndim == 3:
                sam_mask = sam_mask[..., 0]
        else:
            raise TypeError("sam_mask_path 既不是字符串路径也不是 NumPy 数组！")

    # 2. 加载网络
    net = get_net()

    # 3. 处理数据（注意：传的是 sam_mask 数组）
    end_points, cloud_o3d, K = get_and_process_data(color_path, depth_path, sam_mask)

    # 4. 前向推理
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)

    gg = GraspGroup(grasp_preds[0].detach().cpu().numpy())

    # 5. 碰撞检测
    COLLISION_THRESH = 0.01
    if COLLISION_THRESH > 0:
        voxel_size = 0.01
        collision_thresh = 0.01
        mfcdetector = ModelFreeCollisionDetector(
            np.asarray(cloud_o3d.points), voxel_size=voxel_size)
        collision_mask = mfcdetector.detect(
            gg, approach_dist=0.05, collision_thresh=collision_thresh)
        gg = gg[~collision_mask]

    # 6. NMS + 按 score 排序
    gg.nms().sort_by_score()

    # ===== 6.1 接近方向垂直约束（保留你原来的） =====
    all_grasps = list(gg)
    vertical = np.array([0, 0, 1])
    angle_threshold = np.deg2rad(30)
    filtered = []
    for grasp in all_grasps:
        approach_dir = grasp.rotation_matrix[:, 0]
        cos_angle = np.dot(approach_dir, vertical)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        if angle < angle_threshold:
            filtered.append(grasp)
    if len(filtered) == 0:
        print("\n[Warning] No grasp predictions within vertical angle threshold. Using all predictions.")
        filtered = all_grasps
    else:
        print(f"\n[DEBUG] Filtered {len(filtered)} grasps within ±30° of vertical out of {len(all_grasps)} total predictions.")

    # ===== 6.2 利用 SAM mask 再过滤一次抓取（关键！） =====
    if sam_mask is not None:
        h, w = sam_mask.shape[:2]
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]

        sam_filtered = []
        for grasp in filtered:
            X, Y, Z = grasp.translation
            if Z <= 0:
                continue
            u = fx * X / Z + cx
            v = fy * Y / Z + cy
            u_int = int(round(u))
            v_int = int(round(v))
            if (0 <= u_int < w) and (0 <= v_int < h):
                if sam_mask[v_int, u_int] > 127:  # 在 mask 内
                    sam_filtered.append(grasp)

        if len(sam_filtered) == 0:
            print("\n[Warning] No grasp predictions fall inside the SAM mask. Using previous predictions.")
        else:
            print(f"\n[DEBUG] Filtered {len(sam_filtered)} grasps inside the SAM mask out of {len(filtered)} predictions.")
            filtered = sam_filtered

    # ===== 6.3 后面你基于点云中心计算距离、综合 score + 距离的逻辑保持不变 =====
    points = np.asarray(cloud_o3d.points)
    object_center = np.mean(points, axis=0) if len(points) > 0 else np.zeros(3)

    distances = []
    for grasp in filtered:
        grasp_center = grasp.translation
        distance = np.linalg.norm(grasp_center - object_center)
        distances.append(distance)

    grasp_with_distances = [(g, d) for g, d in zip(filtered, distances)]
    grasp_with_distances.sort(key=lambda x: x[1])
    filtered = [g for g, d in grasp_with_distances]

    max_distance = max(distances) if distances else 1.0
    grasp_with_composite_scores = []
    for g, d in grasp_with_distances:
        distance_score = 1 - (d / max_distance)
        composite_score = g.score * 0.1 + distance_score * 0.9
        grasp_with_composite_scores.append((g, composite_score))

    grasp_with_composite_scores.sort(key=lambda x: x[1], reverse=True)
    filtered = [g for g, score in grasp_with_composite_scores]

    # 只取最优抓取
    top_grasps = filtered[:1]
    best_grasp = top_grasps[0]

    new_gg = GraspGroup()
    new_gg.add(best_grasp)

    save_grasp_debug_image(cloud_o3d, best_grasp, tag="cam")

    visual = True
    if visual:
        grippers = new_gg.to_open3d_geometry_list()
        o3d.visualization.draw_geometries([cloud_o3d, *grippers])

    return new_gg
# ==================== [核心修改] 仿真执行抓取动作 ====================
# def execute_grasp(env, gg):

    # """
    # 适配 ROS2 + Piper 的抓取执行函数
    # 原理：不再手动规划每一步的插值，而是计算关键帧的关节角度，发给 ROS 执行。
    # """
    # if gg is None or len(gg) == 0:
    #     print("[Info] No grasp detected, skipping execution.")
    #     return

    # # 0. 坐标系定义 (必须与 Gazebo 实际情况一致！)
    # # 假设相机是眼在手外 (Eye-to-Hand)，固定在世界坐标系
    # # T_wc: 相机在世界坐标系中的位姿
    # # 请用 rviz 或 tf 查看实际的 camera_link 坐标！
    # # 下面是一个示例值，你需要根据你的 Gazebo scene 修改
    # n_wc = np.array([0.0, -1.0, 0.0]) 
    # o_wc = np.array([-1.0, 0.0, -0.5]) 
    # t_wc = np.array([0.5, 0.0, 0.5]) # <--- [重要] 请修改为 Gazebo 中实际相机位置
    
    # T_wc = sm.SE3.Trans(t_wc) * sm.SE3(sm.SO3.TwoVectors(x=n_wc, y=o_wc))
    
    # # 计算物体在世界坐标系下的位姿 T_wo
    # # graspnet 输出的 gg.translations 是相对于相机坐标系的
    # T_co = sm.SE3.Trans(gg.translations[0]) * sm.SE3(sm.SO3.TwoVectors(x=gg.rotation_matrices[0][:, 0], y=gg.rotation_matrices[0][:, 1]))
    # T_wo = T_wc * T_co

    # print(f"[Execute] Target World Pose: {T_wo.t}")

    # # 1. 准备工作：回原点
    # print(">>> Step 1: Moving to Home")
    # env.reset() # 或者 env.move_joints([0,0,0,0,0,0])
    
    # # 2. 移动到预抓取位置 (Pre-Grasp)
    # # 在抓取点沿接近方向回退 10cm
    # print(">>> Step 2: Moving to Pre-Grasp")
    # T_pre = T_wo * sm.SE3(-0.1, 0.0, 0.0) # 假设 x 轴是接近方向
    
    # # IK 解算
    # current_joints = [0.0] * 6 # 初始猜测
    # q_pre = piper_ik.solve_ik(T_pre, current_joints)
    
    # # 执行移动
    # env.move_joints(q_pre, duration=3.0)
    # time.sleep(3.5) # 等待动作完成

    # # 3. 移动到抓取位置 (Grasp)
    # print(">>> Step 3: Approach and Grasp")
    # T_grasp = T_wo
    # q_grasp = piper_ik.solve_ik(T_grasp, q_pre)
    # env.move_joints(q_grasp, duration=2.0)
    # time.sleep(2.5)

    # # 4. 闭合夹爪
    # print(">>> Step 4: Close Gripper")
    # # 注意：需要在 PiperRosEnv 中实现 gripper_control
    # if hasattr(env, 'gripper_control'):
    #     env.gripper_control(0.0) # 假设 0 是闭合
    # else:
    #     print("[Warning] Env has no gripper_control method. Simulating wait.")
    # time.sleep(1.0)

    # # 5. 提起物体 (Lift)
    # print(">>> Step 5: Lift Object")
    # T_lift = sm.SE3.Trans(0.0, 0.0, 0.2) * T_grasp # 向上提 20cm (世界坐标系 Z+)
    # # 注意：如果要在世界坐标系提，用 T_lift = sm.SE3(0,0,0.2) * T_grasp 是错的，那是沿末端坐标系提
    # # 正确的世界坐标系提升：
    # T_lift_world = T_grasp.copy()
    # T_lift_world.t[2] += 0.2
    
    # q_lift = piper_ik.solve_ik(T_lift_world, q_grasp)
    # env.move_joints(q_lift, duration=2.0)
    # time.sleep(2.5)

    # # 6. 移动到放置点 (Place)
    # print(">>> Step 6: Move to Place")
    # # 假设放置点在右侧
    # target_place = np.array([0.3, -0.3, 0.2]) # x, y, z
    # T_place = sm.SE3.Trans(target_place) * sm.SE3(sm.SO3(T_grasp.R)) # 保持姿态
    
    # q_place = piper_ik.solve_ik(T_place, q_lift)
    # env.move_joints(q_place, duration=4.0)
    # time.sleep(4.5)

    # # 7. 松开夹爪
    # print(">>> Step 7: Open Gripper")
    # if hasattr(env, 'gripper_control'):
    #     env.gripper_control(1.0) # 假设 1.0 是张开
    # time.sleep(1.0)

    # # 8. 回家
    # print(">>> Step 8: Back to Home")
    # env.reset()
    # print(">>> Task Completed")
def execute_grasp(env, gg):
    """
    使用 GraspNet 输出 + PiperKine IK 执行抓取
    按你“旧虚拟环境版本”的几何逻辑来做，只是底层变成 ROS2 + IK。
    """

    if gg is None or len(gg) == 0:
        print("[Info] No grasp detected, skipping execution.")
        return

    # ===================== 0. 计算抓取在 base_link 下的位姿 T_bo =====================
    # 对应你旧代码里的：
    #   n_wc, o_wc, t_wc
    #   T_wc = sm.SE3.Trans(t_wc) * sm.SE3(sm.SO3.TwoVectors(x=n_wc, y=o_wc))
    #   T_co = ...
    #   T_wo = T_wc * T_co
    #
    # 这里我们假设旧环境里的 world ≈ 当前的 base_link，
    # 所以直接把 T_wc 看成 T_bc（base -> camera）

    # 1. 物理位置 (必须和 Gazebo 里 spawn 的坐标一致)
    # 对应 camera_spawner_depth.py 里的 <pose>0.5 0.0 0.3 0 0.6 3.14159</pose>
    x, y, z = 0.5, 0.0, 0.3
    roll, pitch, yaw = 0.0, 0.6, 3.14159

    # 2. 生成变换矩阵 T_bc (Base -> Camera)
    # 使用 RPY (Roll-Pitch-Yaw) 构建旋转矩阵
    # Gazebo 的旋转顺序通常对应 spatialmath 的 'xyz' (Extrinsic) 或 'zyx' (Intrinsic)
    # 这里直接用 spatialmath 的 RPY
    T_bc = sm.SE3(x, y, z) * sm.SE3.RPY(roll, pitch, yaw, order='xyz')

    # 打印一下调试
    print(f"[Execute] T_bc (Base->Camera):\n{T_bc}")

    # 下面代码保持不变...
    R_co = gg.rotation_matrices[0]           # 3x3
    p_co = gg.translations[0]                   # (3,)
    T_co = sm.SE3.Trans(p_co) * sm.SE3(
        sm.SO3.TwoVectors(x=R_co[:, 0], y=R_co[:, 1])
    )

    # base -> object，相当于旧代码中的 T_wo（只是把 world 换成 base）
    T_bo = T_bc * T_co
    print(f"[Execute] Grasp pose in base_link: {T_bo.t}")

    # ===================== 1. 回到初始姿态（对应旧代码的“1.机器人运动到预抓取位姿”） =====================
    print(">>> Step 1: Reset / Home")
    # 现在的 env.reset() 里面已经有 move_joints 到 home_q
    env.reset()
    time.sleep(1.0)

    # 取当前关节角度作为 IK 初值
    if hasattr(env, "get_joint"):
        current_q = env.get_joint()
    else:
        print("[Warn] env.get_joint not found, use zero as initial joints")
        current_q = [0.0] * 6

    # ===================== 2. 接近抓取位姿：T2 = T_bo * SE3(-0.1, 0, 0) =====================
    # 对应旧代码：
    #   T2 = T_wo * sm.SE3(-0.1, 0.0, 0.0)
    print(">>> Step 2: Move to Pre-Grasp")

    T2 = T_bo * sm.SE3(-0.1, 0.0, 0.0)   # 在抓取坐标系下沿 -x 退 0.1 m

    q_pre = piper_ik.solve_ik(T2, current_q)
    env.move_joints(q_pre, duration=3.0)
    time.sleep(3.0)
    current_q = q_pre

    # ===================== 3. 执行抓取：从 T2 到 T3 = T_bo =====================
    # 对应旧代码：
    #   T3 = T_wo
    print(">>> Step 3: Move to Grasp Pose")

    T3 = T_bo
    q_grasp = piper_ik.solve_ik(T3, current_q)
    env.move_joints(q_grasp, duration=2.0)
    time.sleep(2.0)
    current_q = q_grasp

    # 夹爪闭合，对应旧代码里 action[-1] += 0.2 的那段
    print(">>> Step 4: Close Gripper")
    if hasattr(env, "gripper_control"):
        # 这里假设 0.0 = 闭合，1.0 = 打开；你根据实际反一下即可
        env.gripper_control(0.0)
    else:
        print("[Warn] env.gripper_control not found, skip gripper command")
    time.sleep(1.0)

    # ===================== 4. 提起物体：T4 = SE3.Trans(0,0,0.3) * T3 =====================
    # 对应旧代码：
    #   T4 = sm.SE3.Trans(0.0, 0.0, 0.3) * T3
    print(">>> Step 5: Lift Object")

    T4 = sm.SE3.Trans(0.0, 0.0, 0.3) * T3  # 沿 base/world 的 Z 轴向上 0.3m
    q_lift = piper_ik.solve_ik(T4, current_q)
    env.move_joints(q_lift, duration=2.0)
    time.sleep(2.0)
    current_q = q_lift

    # ===================== 5. 水平移动到放置点：T5 =====================
    # 对应旧代码：
    #   T5 = sm.SE3.Trans(1.4, 0.3, T4.t[2]) * sm.SE3(sm.SO3(T4.R))
    print(">>> Step 6: Move to Place Pose")

    # 这里直接沿用旧代码的位姿逻辑，只是用 IK 来求关节角
    T5 = sm.SE3.Trans(1.4, 0.3, T4.t[2]) * sm.SE3(sm.SO3(T4.R))
    q_place = piper_ik.solve_ik(T5, current_q)
    env.move_joints(q_place, duration=3.0)
    time.sleep(3.0)
    current_q = q_place

    # ===================== 6. 放下物体：向下 0.1m + 张开夹爪 =====================
    # 对应旧代码里 T6 = sm.SE3.Trans(0.0, 0.0, -0.1) * T5 以及 action[-1] -= 0.2 那一段
    print(">>> Step 7: Lower and Open Gripper")

    T6 = sm.SE3.Trans(0.0, 0.0, -0.1) * T5
    q_down = piper_ik.solve_ik(T6, current_q)
    env.move_joints(q_down, duration=2.0)
    time.sleep(2.0)
    current_q = q_down

    if hasattr(env, "gripper_control"):
        env.gripper_control(1.0)   # 张开
    time.sleep(1.0)

    # ===================== 7. 抬起夹爪：T7 = SE3.Trans(0,0,0.1) * T6 =====================
    # 对应旧代码：
    #   T7 = sm.SE3.Trans(0.0, 0.0, 0.1) * T6
    print(">>> Step 8: Lift Up After Place")

    T7 = sm.SE3.Trans(0.0, 0.0, 0.1) * T6
    q_up = piper_ik.solve_ik(T7, current_q)
    env.move_joints(q_up, duration=2.0)
    time.sleep(2.0)
    current_q = q_up

    # ===================== 8. 回到初始位置（对应旧代码最后一段） =====================
    print(">>> Step 9: Back to Home")
    env.reset()
    print(">>> Task Completed")
