#pragma once

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

class RegionGrowingSegmentor
{
public:
    void setParameters(bool smooth_model_flag,
                        int min_cluster_size,
                        int neighbouts_number,
                        float smothness_threshold);
    std::vector<std::vector<int>> segment(std::vector<std::vector<float>> & point_cloud, 
                                            std::vector<std::vector<float>> & normals);

    void display_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, const std::vector<std::vector<int>>& clusters);

private:
    bool smooth_model_flag_;
    int min_cluster_size_;
    int neighbouts_number_;
    float smothness_threshold_;
};
