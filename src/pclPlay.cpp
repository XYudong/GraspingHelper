//
// Created by yudong on 11/7/18.
/*
 * try and play with functions in PCL
 */

#include "pclPlay.h"

#include <ostream>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>


void pclpcl::writePCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::string & filename) {
    pcl::io::savePCDFileASCII(filename, *cloud);
    std::cerr << "Saved " << cloud -> points.size () << " data points to .pcd" << std::endl;
}

void pclpcl::statisticalFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               pcl::PointIndices::Ptr& inliers_idx) {
    /* output indices of resulting cloud
     */
    std::cout << "cloud before statistical filtering:" << std::endl;
    std::cout << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (30);
    sor.setStddevMulThresh (0.5);       // means distance threshold is 1*sigma
    sor.filter(inliers_idx->indices); // use it to extract both point cloud and its normals
//    sor.filter(*cloud);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> pclpcl::viewportsVis(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr ptr2 ) {
    /* to visualize two point cloud data at the same time */

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor (0, 0, 0, v1);
    viewer->addText("view 1", 10, 10, "v1 text", v1);
    viewer->addPointCloud<pcl::PointXYZ> (ptr1, "sample cloud1", v1);

    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    viewer->addText("view 2", 10, 10, "v2 text", v2);
    if (ptr2) {
        viewer->addPointCloud<pcl::PointXYZ> (ptr2, "sample cloud2", v2);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
    }


    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
    viewer->addCoordinateSystem (0.5);  // scale of the three axes

    return viewer;
}

void pclpcl::voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    std::cerr << "PointCloud before Voxel filtering: " << cloud->points.size() << " data points" << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.004f, 0.004f, 0.004f);       // powerful in accelerating computation
    sor.filter ((cloud_out) ? *cloud_out : *cloud);

    std::cerr << "PointCloud after Voxel filtering: " << ((cloud_out) ? cloud_out : cloud)->points.size()
              << " data points" << std::endl;
}

std::vector<double> pclpcl::getCentroid(const pcl::PointCloud<pcl::PointXYZ> &points) {
    size_t size = points.size();
    std::vector<double> centroid = {0, 0, 0};    // x, y, z
    for (auto point : points) {
        centroid[0] += point.x;
        centroid[1] += point.y;
        centroid[2] += point.z;
    }
    centroid[0] /= size;
    centroid[1] /= size;
    centroid[2] /= size;

    return centroid;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> pclpcl::simpleVis(
        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr) {
    /* viewer for just single port
     * */
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0.3, 0.3, 0.3);
    if (cloud_ptr) {
        viewer->addPointCloud<pcl::PointXYZ> (cloud_ptr, "background_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "background_cloud");
    }
    viewer->addCoordinateSystem(0.3);
    viewer->initCameraParameters();
    return viewer;
}

//void pclpcl::doRegionGrowing(CylinderSegmentation& cy_seg) {
//        // main workflow of segmentation using RegionGrowing
//        // I think this method is hard to segment properly because of outliers collections
//
////    auto start = high_resolution_clock::now();
//
//    cout << "\nSegmentation Input Cloud:\n" << *(cy_seg.cloud) << endl;
//    cy_seg.passThroughFilter();
//    cy_seg.downSampling();
//    cy_seg.estNormals();
//
//    cy_seg.segPlane();
//    cy_seg.regionGrowing();
//
//    auto reg = cy_seg.reg;
//
//    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
//    pcl::visualization::CloudViewer viewer ("Cluster viewer");
//    viewer.showCloud(colored_cloud);
//
////    auto stop = high_resolution_clock::now();
////    auto duration = duration_cast<milliseconds>(stop - start);
////    cout << "\nCylinder segmentation task takes: " << duration.count() << " milliseconds" << endl;
//
//    while (!viewer.wasStopped ())
//    {
//    }
//
//}







