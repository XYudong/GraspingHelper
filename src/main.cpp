#include "main.h"

#include <iostream>
#include <chrono>

#include "cylinder_segmentation.h"
#include "pclPlay.h"
#include <librealsense2/rs.hpp>
#include "rsPlay.h"

#include <pcl/visualization/cloud_viewer.h>


using namespace std;
using namespace std::chrono;


int main() {
    std::cout << "Hello, World!" << std::endl;
    //    rsrs::capture(false);
//    rs2::points rs_Points = rsrs::rsPointCloud(false);
//    cloud = rsrs::points_to_pcl(rs_Points);

    pcl::PCDReader reader;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cy_cloud;

    string path_to_data = "../../data/output/";
    for (int i = 13; i < 14 ; i++) {
        reader.read(path_to_data + "test_" + to_string(i) + ".pcd", *cloud);
        cout << "test_" << to_string(i) << endl;

        auto cy_seg = cylinder_segmentation(cloud);
        doCySegmentation(cy_seg);

//        auto viewer = pclpcl::simpleVis(cloud);
//        while (!viewer->wasStopped ())
//        {
//            viewer->spinOnce (100);
//            boost::this_thread::sleep (boost::posix_time::microseconds (10000));
//        }
    }

    return 0;
}

void doCySegmentation(cylinder_segmentation & cy_seg) {
    // main workflow for cylinder(s) segmentation
    auto start = high_resolution_clock::now();

    cout << "\nSegmentation Input Cloud:\n" << *(cy_seg.cloud) << endl;
    cy_seg.passThroughFilter();
    cy_seg.downSampling();
    cy_seg.estNormals();

    cy_seg.segPlane();
    cy_seg.segCylinder();       // segment first cylinder

    // initialize viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = pclpcl::simpleVis();
    // for debugging visualization
    viewer->addPointCloud<pcl::PointXYZ> (cy_seg.cloud_filtered2, "debug_cloud");

    int cy_num = 2;     // # of cylinders(start from 0) to extract provided by user
    for (int i = 0; i <= cy_num; i++) {
        auto cy_cloud = cy_seg.cloud_cylinder;  // Ptr
        std::cout << "#" << i << " Cylinder PointCloud has: " << cy_cloud -> points.size () << " data points." << std::endl;
//        std::cerr << "Cylinder coefficients: " << *(cy_seg.coefficients_cylinder) << std::endl;

        vector<double> cy_centroid = pclpcl::getCentroid(*cy_cloud);
        cout << "Centroid of cylinder:" << '\n';
        cout << cy_centroid[0] << '\n' << cy_centroid[1] << '\n' << cy_centroid[2] << '\n' << endl;
        //visualization
        std::string portID = "cy_cloud_" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cy_cloud, 50+80*i, 180-30*i, 100+30*i);
        viewer->addPointCloud<pcl::PointXYZ> (cy_cloud, single_color, portID);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, portID);
        // next one
        if (i<cy_num) {
            cy_seg.segNextCylinder();
        }
    }

//    auto rem_cloud = cy_seg.cloud_filtered2;
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(rem_cloud, 220, 0, 0);
//    viewer->addPointCloud<pcl::PointXYZ> (rem_cloud, single_color, "2333", 0);
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "2333");

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << "\nCylinder segmentation task takes: " << duration.count() << " milliseconds" << endl;
    // rendering
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (10000));
    }
}






