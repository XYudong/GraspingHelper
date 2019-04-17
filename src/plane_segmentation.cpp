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
, coefficients_plane (new pcl::ModelCoefficients)
, idx_plane (new pcl::PointIndices)
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
  norm_est.setKSearch(40);                // better to not less than 30
  norm_est.compute (*cloud_normals);
}

void PlaneSegmentation::segPlane(bool normal) {
  // Create the segmentation object for the planar model and set all the parameters
  auto input_cloud = cloud_filtered->empty() ? cloud_voxelized : cloud_filtered;

  fitPlane(normal, cloud_normals, input_cloud);

  extractNormals(true);

  // Extract the planar inliers from the input cloud into cloud_plane
  extractCloud(false, idx_plane, input_cloud, cloud_plane);

  // Remove the planar inliers, extract the rest into cloud_filtered
  extractCloud(true, idx_plane, input_cloud, cloud_filtered);
}

void PlaneSegmentation::fitPlane(bool normal,
                                 pcl::PointCloud <pcl::Normal>::Ptr& in_normals,
                                 pcl::PointCloud<PointT>::Ptr& in_cloud) {
  seg.setModelType (normal ? pcl::SACMODEL_NORMAL_PLANE : pcl::SACMODEL_PLANE);
  seg.setOptimizeCoefficients (true);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setNormalDistanceWeight (0.05);     // w, important
  seg.setDistanceThreshold (0.03);        // Euclidean distance, with weight (1-w)
  seg.setInputCloud (in_cloud);
  seg.setInputNormals (in_normals);

  // Obtain the plane inliers indices and coefficients
  seg.segment (*idx_plane, *coefficients_plane);
}

void PlaneSegmentation::extractNormals(bool negative) {
  // if negative == true, remove normals corresponding to the idx_plane
  extract_normals.setNegative(negative);
  extract_normals.setInputCloud(cloud_normals);
  extract_normals.setIndices(idx_plane);
  extract_normals.filter(*cloud_normals);
}

void PlaneSegmentation::extractCloud(bool negative,
                                     const pcl::PointIndices::Ptr& idx,
                                     const pcl::PointCloud<PointT>::Ptr& in,
                                     const pcl::PointCloud<PointT>::Ptr& out) {
  // if negative == false, extract pointcloud corresponding idx_plane from "in" into "out"
  extract.setInputCloud(in);
  extract.setIndices(idx);
  extract.setNegative(negative);
  extract.filter(*out);
}

void PlaneSegmentation::regionGrowing(
    pcl::PointCloud<pcl::Normal>::Ptr& in_normals,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& in_cloud,
    std::vector<pcl::PointIndices>& out_clusters) {
  // RegionGrowing segmentation
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (5000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (15);
  reg.setInputCloud (in_cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (in_normals);
  reg.setSmoothnessThreshold (10.0f / 180.0f * (float)M_PI);
  reg.setCurvatureThreshold (3.0);

  reg.extract(out_clusters);
}



