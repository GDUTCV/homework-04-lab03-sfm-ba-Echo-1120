import numpy as np
import open3d as o3d
import os

# 这里指向你生成的结果文件
# 如果你想看带 BA 优化的结果，把 no-bundle-adjustment 改成 bundle-adjustment
path = "predictions/mini-temple/results/no-bundle-adjustment/points3d.npy"

if os.path.exists(path):
    points = np.load(path)
    print(f"正在加载 {len(points)} 个 3D 点...")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # 创建坐标轴 (红=x, 绿=y, 蓝=z)
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

    # 打开窗口
    o3d.visualization.draw_geometries([pcd, axes], window_name="SfM Result", width=800, height=600)
else:
    print(f"找不到文件: {path}，请确认你是否运行了 python sfm.py --dataset mini-temple")