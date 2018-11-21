#include "main.h"

#include <iostream>
#include <chrono>

#include "cylinder_segmentation.h"
#include "pclPlay.h"
#include <librealsense2/rs.hpp>
#include "rsPlay.h"


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
    reader.read(path_to_data + "test_37.pcd", *cloud);      // fail: 34, 35

//    auto start = high_resolution_clock::now();

    auto cy_seg = doCySegmentation(cloud);
    cy_cloud = cy_seg.cloud_cylinder;

//    auto stop = high_resolution_clock::now();
//    auto duration = duration_cast<milliseconds>(stop - start);
//    cout << "Cylinder segmentation task takes: " << duration.count() << " milliseconds" << endl;

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = pclpcl::viewportsVis(cy_seg.cloud_plane, cy_cloud);
//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce (100);
//        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }

    return 0;
}

cylinder_segmentation doCySegmentation(pcl::PointCloud<PointT>::Ptr & cloud) {
    auto start = high_resolution_clock::now();

    cylinder_segmentation cy_seg(cloud);
    cout << "\nSegmentation Input Cloud:\n" << *(cy_seg.cloud) << endl;
    cy_seg.passThroughFilter();
    cy_seg.downSampling();
    cy_seg.estNormals();

    cy_seg.segPlane();
    cy_seg.segCylinder();

    // display two data set at the same time
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = pclpcl::viewportsVis(cy_seg.cloud_plane);

    for (int i = 0; cy_seg.cloud_cylinder->points.size() > 900; i++) {
        auto cy_cloud = cy_seg.cloud_cylinder;  // Ptr
        std::cout << "#" << i << " Cylinder PointCloud has: " << cy_cloud -> points.size () << " data points." << std::endl;
        std::cerr << "Cylinder coefficients: " << *(cy_seg.coefficients_cylinder) << std::endl;

        vector<double> cy_centroid = pclpcl::getCentroid(*cy_cloud);
        cout << "Centroid of cylinder:" << '\n';
        cout << cy_centroid[0] << '\n' << cy_centroid[1] << '\n' << cy_centroid[2] << '\n' << endl;

        std::string portID = "sample cloud2_" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cy_cloud, 50+80*i, 180-30*i, 100+30*i);
        viewer->addPointCloud<pcl::PointXYZ> (cy_cloud, single_color, portID, 0);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, portID);

//        pclpcl::statisticalFilter(cy_seg.cloud_filtered2);
        cy_seg.segNextCylinder();
    }

//    auto rem_cloud = cy_seg.cloud_filtered2;
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(rem_cloud, 220, 0, 0);
//    viewer->addPointCloud<pcl::PointXYZ> (rem_cloud, single_color, "2333", 0);
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "2333");


    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << "\nCylinder segmentation task takes: " << duration.count() << " milliseconds" << endl;

    std::cout << "\nabandoned Cylinder PointCloud has: " << cy_seg.cloud_cylinder->points.size () << " data points." << std::endl;

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (10000));
    }

    return cy_seg;
}






