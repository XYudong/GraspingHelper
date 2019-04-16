#include "pclPlay.h"
//#include <librealsense2/rs.hpp>
//#include "rsPlay.h"
//#include "cylinder_segmentation.h"
#include "plane_segmentation.h"
#include "utils.h"
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <ostream>


using namespace std;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointColor;
typedef pcl::PointXYZ PointT;

void processPlanes(const pcl::PointCloud<PointT>::Ptr& in_cloud);
void writePCD(const std::string& filename, const pcl::PointCloud<PointT>::Ptr& cloud);

int main() {
  std::cout << "Hello, World!" << std::endl;
  //    rsrs::capture(false);
//    rs2::points rs_Points = rsrs::rsPointCloud(false);
//    cloud = rsrs::points_to_pcl(rs_Points);

  string path_to_data = "../../data/twoObjects/scene2";
  vector<string> file_paths;
  get_all_paths(path_to_data, ".pcd", file_paths);

  pcl::PCDReader reader;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  for (const string& path : file_paths) {
    reader.read(path, *cloud);
    cout << path << endl;

    processPlanes(cloud);

//    auto seger = PlaneSegmentation(cloud);
//    seger.passThroughFilter();
//    writePCD(path, seger.cloud);
  }

  return 0;
}

void processPlanes(const pcl::PointCloud<PointT>::Ptr& in_cloud) {
    //visualization
//    auto viewer = pclpcl::simpleVis();

  auto seger = PlaneSegmentation(in_cloud);
  seger.passThroughFilter();
  seger.downSampling();
  seger.estNormals();

  seger.segPlane();
//    PointColor single_color(seger.cloud_filtered, 50+80, 180-30, 100+30);
//    std::string portID = "cy_cloud_" + std::to_string(1);
//    viewer->addPointCloud<pcl::PointXYZ>(seger.cloud_filtered, single_color, portID);
//    auto plane0_coeffs = seger.coefficients_plane->values;
//    auto plane0_normal = vector<float>(plane0_coeffs.begin(), plane0_coeffs.begin() + 3);

//    float angle = 20;
//    while (angle > 10) {
//      seger.segPlane();
//
//      auto plane1_coeffs = seger.coefficients_plane->values;
//      auto plane1_normal = vector<float>(plane1_coeffs.begin(), plane1_coeffs.begin() + 3);
//      angle = acos(dotProduct<float>(plane0_normal, plane1_normal));
//      angle = radianToDegree(angle);
//      cout << "angle of two planes: " << angle << endl;
//    }
//
//    PointColor obj_color(seger.cloud_plane, 50, 180, 100);
//    std::string objID = "cloud_" + std::to_string(2);
//    viewer->addPointCloud<pcl::PointXYZ>(seger.cloud_plane, obj_color, objID);

  vector<pcl::PointIndices> clusters;
  seger.regionGrowing(seger.cloud_normals, seger.cloud_filtered, clusters);
  cout << "# of clusters: " << clusters.size() << endl;

//    PointColor obj_color(seger.cloud_plane, 50, 180, 100);
//    std::string objID = "cloud_" + std::to_string(2);
//    viewer->addPointCloud<pcl::PointXYZ>(cloud, obj_color, objID);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seger.reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped()) {
//      viewer->spinOnce (100);   // ms
//      boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }
}

void writePCD(const std::string& filename, const pcl::PointCloud<PointT>::Ptr& cloud) {
  pcl::PCDWriter writer;
  writer.write(filename, *cloud, false);
  std::cerr << "Saved " << (cloud->points).size() << " data points to " << filename << std::endl;
}


void processCylinders(std::string& path_to_data) {
  pcl::PCDReader reader;
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cy_cloud;

  for (int i = 13; i < 14 ; i++) {
    reader.read(path_to_data + "test_" + to_string(i) + ".pcd", *cloud);
    cout << "test_" << to_string(i) << endl;

//    auto cy_seg = CylinderSegmentation(cloud);
//    doCySegmentation(cy_seg);

    auto viewer = pclpcl::simpleVis(cloud);
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (10000));
    }
  }
}


