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
    reader.read(path_to_data + "test_5.pcd", *cloud);

//    auto start = high_resolution_clock::now();

    auto cy_seg = doCySegmentation(cloud);
    cy_cloud = cy_seg.cloud_cylinder;       //TODO: segment cylinder(s) iteratively

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

vector<double> getCentroid(pcl::PointCloud<PointT> points) {
    size_t size = points.size();
    vector<double> centroid = {0, 0, 0};    // x, ,y, z
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

    int v2(0);
    for (int i = 0; cy_seg.cloud_cylinder->points.size() > 900; i++) {
        auto cy_cloud = cy_seg.cloud_cylinder;  // Ptr
        std::cout << "Cylinder PointCloud has: " << cy_cloud->points.size () << " data points." << std::endl;

        vector<double> cy_centroid = getCentroid(*cy_cloud);
        cout << "Centroid of cylinder: #" << i << '\n';
        cout << cy_centroid[0] << '\n' << cy_centroid[1] << '\n' << cy_centroid[2] << endl;

        std::string portID = "sample cloud2_" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cy_cloud, 70*i, 100+40*i, 100);
        viewer->addPointCloud<pcl::PointXYZ> (cy_cloud, single_color, portID, v2);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, portID);

        cy_seg.segNextCylinder();
    }

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






