# cython_code.pyx
from libcpp.vector cimport vector
from libcpp cimport bool, float, int

cdef extern from "RegionGrowingSegmentorWrapper.h":
    cdef cppclass RegionGrowingSegmentorWrapper:
        void setParameters(bool, int, int, float)
        vector[vector[int]] segment(vector[vector[float]]&, vector[vector[float]]&)
        void display_clusters_wrapper(const vector[vector[float]]&, const vector[vector[float]]&, const vector[vector[int]]&)

cdef class PyRegionGrowingSegmentor:
    cdef RegionGrowingSegmentorWrapper _c_instance

    def set_parameters(self, bool smooth_model_flag, int min_cluster_size, int neighbouts_number, float smothness_threshold):
        self._c_instance.setParameters(smooth_model_flag, min_cluster_size, neighbouts_number, smothness_threshold)

    def segment(self, point_cloud, normals):
        cdef vector[vector[float]] point_cloud_cpp
        cdef vector[vector[float]] normals_cpp

        for point in point_cloud:
            point_cloud_cpp.push_back(point)

        for normal in normals:
            normals_cpp.push_back(normal)

        cdef vector[vector[int]] clusters_cpp = self._c_instance.segment(point_cloud_cpp, normals_cpp)

        clusters_py = [list(cluster) for cluster in clusters_cpp]
        return clusters_py

    def display_clusters(self, point_cloud, normals, clusters):
        cdef vector[vector[float]] point_cloud_cpp
        cdef vector[vector[float]] normals_cpp
        cdef vector[vector[int]] clusters_cpp

        for point in point_cloud:
            point_cloud_cpp.push_back(point)

        for normal in normals:
            normals_cpp.push_back(normal)

        for cluster in clusters:
            clusters_cpp.push_back(cluster)

        self._c_instance.display_clusters_wrapper(point_cloud_cpp, normals_cpp, clusters_cpp)
