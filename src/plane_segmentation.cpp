//
// Created by yudong on 4/14/19.
//

#include "plane_segmentation.h"
#include <pcl/io/pcd_io.h>

PlaneSegmentation::PlaneSegmentation(const pcl::PointCloud<PointT>::Ptr &cloud)
: tree (new pcl::search::KdTree<PointT> ())
, cloud (new pcl::PointCloud<PointT>)
, cloud_voxelized (new pcl::PointCloud<PointT>)
, cloud_normals (new pcl::PointCloud<pcl::Normal>)
, cloud_filtered (new pcl::PointCloud<PointT>)
//, cloud_normals2 (new pcl::PointCloud<pcl::Normal>)
, coefficients_plane (new pcl::ModelCoefficients)
, inliers_plane (new pcl::PointIndices)
, cloud_plane (new pcl::PointCloud<PointT>) {
  // constructor
  this -> cloud = cloud;
}

void PlaneSegmentation::passThroughFilter() {
  // Build a PassThrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.3, 1.7);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.5f, 0.5f);
  pass.filter (*cloud);

  std::cerr << "PointCloud after PassThrough filtering:\n" << *cloud << std::endl;
}

void PlaneSegmentation::downSampling() {
  pclpcl::voxelGridFilter(cloud, cloud_voxelized);
}

void PlaneSegmentation::estNormals() {
  // Estimate point normals
  norm_est.setSearchMethod (tree);
  norm_est.setInputCloud (cloud_voxelized);
  norm_est.setSearchSurface(cloud);
  norm_est.setKSearch(35);
  norm_est.compute (*cloud_normals);
}

void PlaneSegmentation::segPlane() {
  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.015);
  seg.setInputCloud (cloud_filtered->empty() ? cloud_voxelized : cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers indices and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);

  // remove normals corresponding to the segmented plane
  extract_normals.setNegative(true);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(inliers_plane);
  extract_normals.filter(*cloud_normals);

  extract.setInputCloud (cloud_filtered->empty() ? cloud_voxelized : cloud_filtered);
  extract.setIndices (inliers_plane);
  // Extract the planar inliers from the input cloud into cloud_plane{
  extract.setNegative (false);
  extract.filter (*cloud_plane);

  // Remove the planar inliers, extract the rest into cloud_filtered
  extract.setNegative (true);
  extract.filter (*cloud_filtered);
}

void PlaneSegmentation::regionGrowing(
    pcl::PointCloud<pcl::Normal>::Ptr in_normals,
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
    std::vector <pcl::PointIndices>& out_clusters) {
  // RegionGrowing segmentation
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (5000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (20);
  reg.setInputCloud (in_cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (in_normals);
  reg.setSmoothnessThreshold (10.0f / 180.0f * (float)M_PI);
  reg.setCurvatureThreshold (15.0);

  reg.extract(out_clusters);
}



