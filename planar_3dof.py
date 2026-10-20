import mujoco
import numpy as np

def rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    T = np.eye(4)
    # 齐次矩阵左上角 3x3 存旋转；这里是绕 z 轴的二维平面旋转。
    T[:3, :3] = [[c, -s, 0],
                 [s,  c, 0],
                 [0,  0, 1]]
    return T

def trans(x, y, z):
    T = np.eye(4)
    # 齐次矩阵第 4 列前三项存平移向量，也就是子坐标系原点的位置。
    T[:3, 3] = [x, y, z]
    return T

def fk_planar(thetas, L=(0.3, 0.25, 0.15)):
    t1, t2, t3 = thetas
    # 每一组 rot_z(theta_i) @ trans(L_i, 0, 0)
    # 对应一节“先绕关节转，再沿当前连杆 x 方向走 L_i”。
    T = rot_z(t1) @ trans(L[0], 0, 0) \
        @ rot_z(t2) @ trans(L[1], 0, 0) \
        @ rot_z(t3) @ trans(L[2], 0, 0)
    return T     # 末端位置直接读 T[:3, 3]；末端姿态在 T[:3, :3] 里。
if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path('planar_3dof.xml')
    data  = mujoco.MjData(model)
    # 后面比较的是 end_site 的世界坐标，所以先拿到它在 MuJoCo 模型里的 id。
    end_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'end_site')

    max_err = 0.0
    for _ in range(100):
        # 随机采样一组关节角，覆盖不同姿态，而不是只测零位。
        q = np.random.uniform(-np.pi, np.pi, size=3)
        data.qpos[:3] = q
        mujoco.mj_forward(model, data)           # 让 MuJoCo 按 kinematic tree 更新所有位姿
        p_mj = data.site_xpos[end_id]            # MuJoCo 算出的 end_site 世界坐标

        p_ours = fk_planar(q)[:3, 3]             # 自己的 FK 矩阵第 4 列前三项

        max_err = max(max_err, np.linalg.norm(p_mj - p_ours))

    print(f'max |p_ours - p_mj| = {max_err:.2e}')
    # 期望量级在 1e-14 ~ 1e-12;超过 1e-6 基本是参数/轴向写错了
