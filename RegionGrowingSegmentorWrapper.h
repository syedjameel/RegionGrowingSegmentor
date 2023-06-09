#include "RegionGrowingSegmentor.h"
#include <vector>

class RegionGrowingSegmentorWrapper : public RegionGrowingSegmentor {
public:
    void display_clusters_wrapper(const std::vector<std::vector<float>>& cloud, const std::vector<std::vector<float>>& normals, const std::vector<std::vector<int>>& clusters);
};

inline void RegionGrowingSegmentorWrapper::display_clusters_wrapper(const std::vector<std::vector<float>>& cloud, const std::vector<std::vector<float>>& normals, const std::vector<std::vector<int>>& clusters) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr pcl_normals(new pcl::PointCloud<pcl::Normal>);

    for (const auto& point : cloud) {
        pcl_cloud->push_back(pcl::PointXYZ(point[0], point[1], point[2]));
    }

    for (const auto& normal : normals) {
        pcl_normals->push_back(pcl::Normal(normal[0], normal[1], normal[2]));
    }

    display_clusters(pcl_cloud, pcl_normals, clusters);
}
