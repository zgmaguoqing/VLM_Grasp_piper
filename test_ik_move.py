import os
import time
import numpy as np
import spatialmath as sm
import rclpy
from main_vlm import PiperRosEnv  # 直接从 main_vlm 引用修复后的环境类
from ikpy.chain import Chain

# ================= IK 类 (保留修复后的 Mask 和 Clip) =================
PIPER_URDF_PATH = "/root/piper_ws/piper_ros.urdf"
JOINT_INDICES = [1, 2, 3, 4, 5, 6]

class PiperKine:
    def __init__(self, urdf_path):
        self.valid = False
        if not os.path.exists(urdf_path):
            print(f"[Error] URDF not found at {urdf_path}")
            return
        print(f"[IK] Loading URDF from {urdf_path}")
        self.chain = Chain.from_urdf_file(urdf_path)
        self.joint_indices = JOINT_INDICES
        
        mask = []
        for i, l in enumerate(self.chain.links):
            is_active = (i in self.joint_indices)
            mask.append(is_active)
        self.chain.active_links_mask = mask
        self.valid = True

    def solve_ik(self, target_pose, current_joints):
        if not self.valid: return [0.0]*6
        if hasattr(target_pose, "A"): T = target_pose.A
        else: T = np.array(target_pose, dtype=float)
        
        n_links = len(self.chain.links)
        initial = np.zeros(n_links)
        for j, idx in enumerate(self.joint_indices):
            if j < len(current_joints): initial[idx] = current_joints[j]

        # Clip initial to bounds
        for i, link in enumerate(self.chain.links):
            if self.chain.active_links_mask[i] and link.bounds:
                low, high = link.bounds
                if low is None: low = -np.inf
                if high is None: high = np.inf
                initial[i] = np.clip(initial[i], low, high)

        sol = self.chain.inverse_kinematics(
            target_position=T[:3, 3],
            target_orientation=T[:3, :3],
            orientation_mode="all",
            initial_position=initial,
            max_iter=100
        )
        return [sol[idx] for idx in self.joint_indices]

# ================= 主测试逻辑 =================
def test_move():
    # 1. 初始化
    ik_solver = PiperKine(PIPER_URDF_PATH)
    env = PiperRosEnv()  # 使用 main_vlm.py 里的类
    env.reset()
    time.sleep(1.0)

    current_q = env.home_q # 或者 env.latest_joint_state... 这里简单起见用 home
    
    # 2. 定义测试点
    test_targets = [
        {"pos": [0.2, 0.0, 0.25], "name": "Center Front"},
        {"pos": [0.2, 0.1, 0.25], "name": "Left Front"},
        {"pos": [0.2, -0.1, 0.25], "name": "Right Front"},
    ]

    R_down = sm.SE3.RPY(0, 90, 0, unit='deg')

    for target in test_targets:
        pos = target['pos']
        name = target['name']
        print(f"\n>>> Testing Target: {name} at {pos}")

        T_target = sm.SE3(pos) * R_down
        
        # IK 解算
        q_sol = ik_solver.solve_ik(T_target, current_q)
        print(f"IK Solution: {q_sol}")
        
        # 移动 (复用 Env 类的接口)
        env.move_joints(q_sol, duration=3.0)
        time.sleep(3.5)
        current_q = q_sol

    print("\n[Test] Done. Back to home.")
    env.reset()
    env.close()

if __name__ == "__main__":
    try:
        test_move()
    except KeyboardInterrupt:
        pass
