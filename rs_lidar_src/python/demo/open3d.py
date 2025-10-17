import open3d as o3d
import numpy as np

# 生成随机点云数据：10000个点，每个点包含x、y、z坐标
n_points = 10000
# 生成范围在[-5, 5)之间的随机坐标
points = np.random.uniform(-5, 5, size=(n_points, 3))

# 生成随机颜色：每个点的RGB值在[0, 1)之间
colors = np.random.uniform(0, 1, size=(n_points, 3))

# 创建Open3D点云对象
pcd = o3d.geometry.PointCloud()
# 设置点坐标
pcd.points = o3d.utility.Vector3dVector(points)
# 设置点颜色
pcd.colors = o3d.utility.Vector3dVector(colors)

# 可视化点云
o3d.visualization.draw_geometries(
    [pcd],
    window_name="随机点云可视化",
    width=800,
    height=600
)