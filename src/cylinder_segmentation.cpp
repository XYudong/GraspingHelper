//
// Created by yudong on 11/14/18.
//

#include "cylinder_segmentation.h"
#include <pcl/features/normal_3d_omp.h>
#include "pclPlay.h"


cylinder_segmentation::cylinder_segmentation(const pcl::PointCloud<PointT>::Ptr & cloud)
: tree (new pcl::search::KdTree<PointT> ())
, cloud (new pcl::PointCloud<PointT>)
, cloud_filtered (new pcl::PointCloud<PointT>)
, cloud_voxelized (new pcl::PointCloud<PointT>)
, cloud_normals (new pcl::PointCloud<pcl::Normal>)
, cloud_filtered2 (new pcl::PointCloud<PointT>)
, cloud_normals2 (new pcl::PointCloud<pcl::Normal>)
, coefficients_plane (new pcl::ModelCoefficients)
, coefficients_cylinder (new pcl::ModelCoefficients)
, inliers_plane (new pcl::PointIndices)
, inliers_cylinder (new pcl::PointIndices)
, cloud_cylinder (new pcl::PointCloud<PointT>)
, cloud_plane (new pcl::PointCloud<PointT>) {
//    reader.read (input_file, *cloud);
//    std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
    this -> cloud = cloud;
}

void cylinder_segmentation::passThroughFilter() {
    // Build a PassThrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.0);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.5f, 0.5f);
    pass.filter (*cloud_filtered);

    std::cerr << "PointCloud after PassThrough filtering:\n" << *cloud_filtered << std::endl;
}

void cylinder_segmentation::downSampling() {
    pclpcl::voxelGridFilter(cloud_filtered, cloud_voxelized);
}

void cylinder_segmentation::estNormals() {
    // Estimate point normals
    norm_est.setSearchMethod (tree);
    norm_est.setInputCloud (cloud_voxelized);
    norm_est.setSearchSurface(cloud_filtered);
    norm_est.setKSearch(15);
    norm_est.compute (*cloud_normals);
}

void cylinder_segmentation::segPlane() {
    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud (cloud_voxelized);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers indices and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
//    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_voxelized);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    extract.filter (*cloud_plane);      // could be commented later to save time
}

void cylinder_segmentation::writePlane(const std::string & plane_file) {
    // Write the planar inliers to disk
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    writer.write (plane_file, *cloud_plane, false);
}

void cylinder_segmentation::segCylinder() {
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);

    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.05);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
//    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud_cylinder);
}

void cylinder_segmentation::segNextCylinder() {
    extract.setNegative(true);
    extract.filter(*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals2);
    extract_normals.setIndices (inliers_cylinder);
    extract_normals.filter (*cloud_normals2);

    seg.setInputCloud(cloud_filtered2);
    seg.setInputNormals(cloud_normals2);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
//    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);
}

void cylinder_segmentation::writeCylinder(const std::string & cylinder_file) {
    // Write the cylinder inliers to disk
    if (cloud_cylinder->points.empty ())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size ()
        << " data points." << std::endl;
        writer.write (cylinder_file, *cloud_cylinder, false);
    }
}











