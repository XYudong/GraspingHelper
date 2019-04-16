//
// Created by yudong on 11/7/18.
//

#ifndef FIRSTTEST_PCLPLAY_H
#define FIRSTTEST_PCLPLAY_H

#include <vector>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include "cylinder_segmentation.h"


namespace pclpcl {
    void writePCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::string & filename);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr2 = nullptr );
    boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (
            pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr = nullptr);

    void statisticalFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                           pcl::PointIndices::Ptr& inliers_idx);
    void voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out = nullptr);

    std::vector<double> getCentroid(const pcl::PointCloud<pcl::PointXYZ> & points);
//    void doRegionGrowing(CylinderSegmentation & cy_seg);

}


#endif //FIRSTTEST_PCLPLAY_H


