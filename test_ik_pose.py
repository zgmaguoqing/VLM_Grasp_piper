import numpy as np
import spatialmath as sm

from grasp_process import PiperKine, PIPER_URDF_PATH

if __name__ == "__main__":
    # 初始化 IK
    ik = PiperKine(PIPER_URDF_PATH)

    # 假设 world 里桌面在 z ~ 0.7 左右（你可以按实际改）
    target_pos = np.array([0.4, 0.0, 0.8])  # x, y, z
    # 让手掌朝下：绕 Y 轴旋转 180 度
    T_target = sm.SE3.Trans(target_pos) * sm.SE3.RPY([0, np.pi, 0])

    current = [0.0] * 6
    q = ik.solve_ik(T_target, current)
    print("IK joints:", q)

    # 再做一次 FK 检查
    n_links = len(ik.chain.links)
    full = np.zeros(n_links)
    for j, idx in enumerate(ik.joint_indices):
        full[idx] = q[j]

    T_fk = ik.chain.forward_kinematics(full)
    print("FK pos:", T_fk[:3, 3])
    print("pos_err:", np.linalg.norm(T_fk[:3, 3] - target_pos))
