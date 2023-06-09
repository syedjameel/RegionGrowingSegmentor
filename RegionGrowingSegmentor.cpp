
#include "RegionGrowingSegmentor.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

void RegionGrowingSegmentor::setParameters(bool smooth_model_flag,
                                      int min_cluster_size,
                                      int neighbouts_number,
                                      float smothness_threshold)
{
    smooth_model_flag_ = smooth_model_flag;
    min_cluster_size_ = min_cluster_size;
    neighbouts_number_ = neighbouts_number;
    smothness_threshold_ = smothness_threshold;
}

std::vector<std::vector<int>> RegionGrowingSegmentor::segment(std::vector<std::vector<float>> & point_cloud, 
                                                                std::vector<std::vector<float>> & normals)
{
    // Converting to pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < point_cloud.size(); ++i)
    {
        pcl::PointXYZ point{point_cloud[i][0], point_cloud[i][1], point_cloud[i][2]};
        point_cloud_pcl->push_back(point);
    }
    // Converting to pcl normal cloud 
    pcl::PointCloud<pcl::Normal>::Ptr normals_pcl (new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < normals.size(); ++i)
    {
        pcl::Normal normal{normals[i][0], normals[i][1], normals[i][2]};
        normals_pcl->push_back(normal);
    }
    // Segmentation planes by RegionGrowing
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setSmoothModeFlag (smooth_model_flag_);
    reg.setMinClusterSize (min_cluster_size_); 
    reg.setNumberOfNeighbours (neighbouts_number_);
    reg.setInputCloud (point_cloud_pcl);
    reg.setInputNormals (normals_pcl);
    reg.setSmoothnessThreshold (smothness_threshold_);
    std::vector <pcl::PointIndices> clusters_pcl;
    reg.extract (clusters_pcl);
    std::vector <std::vector<int>> clusters;
    for (int i = 0; i < clusters_pcl.size(); ++i)
    {
        clusters.push_back(clusters_pcl[i].indices);
    }
    return clusters;
}



void RegionGrowingSegmentor::display_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, const std::vector<std::vector<int>>& clusters)
{
    pcl::visualization::CloudViewer viewer("Cluster Visualization");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_rgb->points.resize(cloud->size());
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud_rgb->points[i].x = cloud->points[i].x;
        cloud_rgb->points[i].y = cloud->points[i].y;
        cloud_rgb->points[i].z = cloud->points[i].z;
        cloud_rgb->points[i].r = 255;
        cloud_rgb->points[i].g = 255;
        cloud_rgb->points[i].b = 255;
    }

    int r = 255, g = 0, b = 0;
    for (const auto& cluster : clusters)
    {
        for (const auto& index : cluster)
        {
            cloud_rgb->points[index].r = r;
            cloud_rgb->points[index].g = g;
            cloud_rgb->points[index].b = b;
        }

        // Update color for next cluster
        r = (r + 50) % 256;
        g = (g + 100) % 256;
        b = (b + 150) % 256;
    }

    viewer.showCloud(cloud_rgb);
    while (!viewer.wasStopped())
    {
        //viewer.spinOnce(100);
    }
}


