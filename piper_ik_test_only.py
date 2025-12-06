import numpy as np
from ikpy.chain import Chain

URDF_PATH = "/root/piper_ws/piper_ros.urdf"
JOINT_INDICES = [1, 2, 3, 4, 5, 6]


class PiperKine:
    def __init__(self, urdf_path):
        self.chain = Chain.from_urdf_file(urdf_path)
        self.joint_indices = JOINT_INDICES
        print("[IK] Links in chain:")
        for i, l in enumerate(self.chain.links):
            print(f"  {i}: {l.name}")

    def solve_ik(self, target_T, current_joints):
        # target_T: 4x4 numpy 矩阵
        T = np.array(target_T, dtype=float)
        target_pos = T[:3, 3]
        target_orient = T[:3, :3]

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

        T_fk = self.chain.forward_kinematics(sol)
        pos_err = np.linalg.norm(T_fk[:3, 3] - target_pos)
        rot_err = np.arccos(
            np.clip((np.trace(target_orient.T @ T_fk[:3, :3]) - 1) / 2.0, -1.0, 1.0)
        )
        print(f"[IK] pos_err={pos_err:.4f} m, rot_err={rot_err*180/np.pi:.2f} deg")
        return sol


if __name__ == "__main__":
    ik = PiperKine(URDF_PATH)

    # 1) 选一组合法关节角 q_test（在 bounds 里，比如刚好在中间附近）
    q_test = np.array([0.0, 1.0, -1.0, 1.0, 0.5, 0.0])
    print("q_test:", q_test)

    # 2) 用 FK 算出末端位姿
    n_links = len(ik.chain.links)
    full = np.zeros(n_links)
    for j, idx in enumerate(JOINT_INDICES):
        full[idx] = q_test[j]

    T_target = ik.chain.forward_kinematics(full)
    print("Target pos from FK:", T_target[:3, 3])

    # 3) 把这个 T_target 丢给 IK 解
    sol = ik.solve_ik(T_target, current_joints=q_test)

    # 4) 从 IK 解里提取 6 个关节角
    q_ik = np.array([sol[idx] for idx in JOINT_INDICES])
    print("IK q_ik:", q_ik)
    print("Joint diff (q_ik - q_test):", q_ik - q_test)
