//
// Created by yudong on 11/7/18.
/*
 * try and play with functions in PCL
 */

#include "pclPlay.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>



void pclpcl::savePCD(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const std::string & filename) {
    pcl::io::savePCDFileASCII(filename, *cloud);
    std::cerr << "Saved " << cloud -> points.size () << " data points to .pcd." << std::endl;
}

void pclpcl::statisticalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {
    std::cout << "cloud before statistical filtering" << std::endl;
//    std::cerr << &(*cloud) << '\n';
//    std::cerr << &cloud << '\n';
    std::cout << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (0.5);
    sor.filter (*cloud);

    std::cout << "Cloud after filtering: " << std::endl;
    std::cout << *cloud << std::endl;

    // display two data set at the same time
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = viewportsVis(cloud, cloud_filtered);
//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce (100);
//        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }


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

void pclpcl::passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 0.8);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);
}

void pclpcl::voxelGridFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    std::cerr << "PointCloud before Voxel filtering: " << cloud->points.size() << " data points" << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.004f, 0.004f, 0.004f);
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








