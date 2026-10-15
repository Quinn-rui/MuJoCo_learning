import mujoco
import numpy as np

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
