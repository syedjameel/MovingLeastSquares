from moving_least_squares import PyMovingLeastSquares
import open3d as o3d
import time
import numpy as np

point_cloud_file = "sample_pcds/spring1.pcd"
point_cloud = o3d.io.read_point_cloud(point_cloud_file)

point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))


# # Estimate normals

start_time = time.time()

movingleastsquares = PyMovingLeastSquares()
movingleastsquares.set_parameters(0.01)
points = np.asarray(point_cloud.points)
#movingleastsquares.apply(points)
#print(movingleastsquares.apply(points))
smoothed_points, normals = movingleastsquares.apply(points)
print(smoothed_points, normals)
smoothed_point_cloud = o3d.geometry.PointCloud()
smoothed_point_cloud.points = o3d.utility.Vector3dVector(smoothed_points)
smoothed_point_cloud.normals = o3d.utility.Vector3dVector(normals)
o3d.visualization.draw_geometries([smoothed_point_cloud])
