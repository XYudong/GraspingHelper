//
// Created by yudong on 4/14/19.
//

#ifndef FIRSTTEST_PLANE_SEGMENTATION_H
#define FIRSTTEST_PLANE_SEGMENTATION_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>

#include "pclPlay.h"

typedef pcl::PointXYZ PointT;

class PlaneSegmentation {
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimationOMP<PointT, pcl::Normal> norm_est;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree;

public:
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  // Datasets
  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointCloud<PointT>::Ptr cloud_voxelized;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
  pcl::PointCloud<PointT>::Ptr cloud_filtered;
  pcl::PointCloud<PointT>::Ptr cloud_plane;
  pcl::ModelCoefficients::Ptr coefficients_plane;
  pcl::PointIndices::Ptr inliers_plane;

  explicit PlaneSegmentation(const pcl::PointCloud<PointT>::Ptr& cloud);
  void passThroughFilter();
  void downSampling();
  void estNormals();
  void segPlane();
  void regionGrowing(
      pcl::PointCloud <pcl::Normal>::Ptr in_normals,
      pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
      std::vector <pcl::PointIndices>& out_clusters);
};


#endif //FIRSTTEST_PLANE_SEGMENTATION_H
