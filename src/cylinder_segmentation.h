//
// Created by yudong on 11/14/18.
//

#ifndef FIRSTTEST_CYLINDER_SEGMENTATION_H
#define FIRSTTEST_CYLINDER_SEGMENTATION_H

#include <string>
#include <chrono>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>

typedef pcl::PointXYZ PointT;


class CylinderSegmentation {
    // All the objects needed
    pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimationOMP<PointT, pcl::Normal> norm_est;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree;

public:
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud;
    pcl::PointCloud<PointT>::Ptr cloud_filtered;
    pcl::PointCloud<PointT>::Ptr cloud_voxelized;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::PointCloud<PointT>::Ptr cloud_filtered2;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2;
    pcl::ModelCoefficients::Ptr coefficients_plane, coefficients_cylinder;
    pcl::PointIndices::Ptr inliers_plane, inliers_cylinder;
    pcl::PointCloud<PointT>::Ptr cloud_plane, cloud_cylinder;

    explicit CylinderSegmentation(const pcl::PointCloud<PointT>::Ptr & cloud);
    /// initialize object data members and read in the cloud data from file
//    virtual ~cylinder_segmentation();
    void passThroughFilter();
    void downSampling();
    void estNormals();
    void segPlane();
    void writePlane(const std::string & plane_file);
    void segCylinder();
    void writeCylinder(const std::string & cylinder_file);
    void segNextCylinder();
    void segNextPlane();
    void regionGrowing();
    void statFilter(pcl::PointCloud<PointT>::Ptr & cloud_filtered, pcl::PointCloud<pcl::Normal>::Ptr & cloud_normals);
};


void doCySegmentation(CylinderSegmentation & cy_seg);


#endif //FIRSTTEST_CYLINDER_SEGMENTATION_H








