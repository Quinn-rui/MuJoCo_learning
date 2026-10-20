from pathlib import Path
import matplotlib.pyplot as plt
import planar_3dof
import numpy as np

pts = []
for _ in range(20000):
    q = np.random.uniform(-np.pi, np.pi, size=3)
    pts.append(planar_3dof.fk_planar(q)[:2, 3])
pts = np.array(pts)

plt.figure(figsize=(5, 5))
plt.scatter(pts[:, 0], pts[:, 1], s=1, alpha=0.3)
plt.axis('equal'); plt.title('3-DoF planar arm workspace')
plt.show()
