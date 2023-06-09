import open3d as o3d
import numpy as np
#from cython_code import PyRegionGrowingSegmentor
from region_growing_segmentor import PyRegionGrowingSegmentor
import time

point_cloud_file = "spring1.pcd"
point_cloud = o3d.io.read_point_cloud(point_cloud_file)

# Estimate normals
point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

start_time = time.time()
segmentor = PyRegionGrowingSegmentor()

smooth_model_flag = True
min_cluster_size = 50
neighbouts_number = 8
smoothness_threshold = 0.1

# Set the parameters using the setParameters function
segmentor.set_parameters(smooth_model_flag, min_cluster_size, neighbouts_number, smoothness_threshold)


# Convert the point cloud data and the estimated normals to numpy arrays
points = np.asarray(point_cloud.points)
normals = np.asarray(point_cloud.normals)

clusters = segmentor.segment(points, normals)

print("Total time taken : ", round(time.time() - start_time, 4), " seconds")
segmentor.display_clusters(points, normals, clusters)

largest_cluster = max(clusters, key=len)

print(largest_cluster)
print("The length of the largest cluster is : ", len(largest_cluster))

