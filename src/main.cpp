#include "pclPlay.h"
//#include <librealsense2/rs.hpp>
//#include "rsPlay.h"
//#include "cylinder_segmentation.h"
#include "plane_segmentation.h"
#include "utils.h"
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/cloud_viewer.h>
#include <ostream>
#include <cstdlib>

using namespace std;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointColor;
typedef pcl::PointXYZ PointT;

void processPlanes(const pcl::PointCloud<PointT>::Ptr& in_cloud);


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
//    pclpcl::writePCD(seger.cloud, path);
  }

  return 0;
}


void processPlanes(const pcl::PointCloud<PointT>::Ptr& in_cloud) {
  // initialization
  auto viewer = pclpcl::simpleVis();
  auto seger = PlaneSegmentation(in_cloud);
  seger.passThroughFilter();
  seger.downSampling();
  seger.estNormals();

  // segment desktop plane(i.e. the largest one)
  seger.segPlane(false);
  pclpcl::statisticalFilter(seger.cloud_filtered, seger.idx_plane);
  seger.extractNormals(false);
  seger.extractCloud(false, seger.idx_plane, seger.cloud_filtered, seger.cloud_filtered);
  cout << "after statisticalFilter: " << endl;
  cout << *(seger.cloud_filtered) << endl;

  // desktop visualization
  PointColor single_color(seger.cloud_filtered, 204, 230, 220);
  std::string portID = "objects_cloud_" + std::to_string(0);
  viewer->addPointCloud<pcl::PointXYZ>(seger.cloud_filtered, single_color, portID);
  // desktop surface normal
  const auto plane0_coeffs = seger.coefficients_plane->values;
  const auto plane0_normal = vector<float>(plane0_coeffs.begin(), plane0_coeffs.begin() + 3);

  // Region Growing segmentation
  vector<pcl::PointIndices> clusters;
  seger.regionGrowing(seger.cloud_normals, seger.cloud_filtered, clusters);
  cout << "# of clusters: " << clusters.size() << endl;

  int count = 1;
  auto idx_plane = pcl::PointIndices::Ptr(new pcl::PointIndices);
  auto normal_plane = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  for (auto& cluster : clusters) {
    if (cluster.indices.size() < 100) {
      continue;
    }
    *idx_plane = cluster;
    seger.extractCloud(false, idx_plane,
                       seger.cloud_filtered,
                       seger.cloud_plane);
    pclpcl::extractNormals(false, idx_plane, seger.cloud_normals, normal_plane);
    seger.fitPlane(false, normal_plane, seger.cloud_plane);

    auto plane1_coeffs = seger.coefficients_plane->values;
    auto plane1_normal = vector<float>(plane1_coeffs.begin(), plane1_coeffs.begin() + 3);
    float angle = acos(dotProduct<float>(plane0_normal, plane1_normal));
    angle = radianToDegree(angle);

    if (angle < 15) {
      PointColor obj_color(seger.cloud_plane, 60 + 40 * count, 100, 100+30);
      std::string objID = "plane_cloud_" + std::to_string(count++);
      viewer->addPointCloud<pcl::PointXYZ>(seger.cloud_plane, obj_color, objID);

      cout << "angle of two planes: " << angle
      << "; # of points: " << (seger.cloud_plane)->size() << endl;
    }

  }

//  float angle = 20;
//  int i = 1;
//  while (angle > 10) {
//    if ((seger.cloud_filtered)->size() < 100) {
//      break;
//    }
//    seger.segPlane(true);
//
//    auto plane1_coeffs = seger.coefficients_plane->values;
//    auto plane1_normal = vector<float>(plane1_coeffs.begin(), plane1_coeffs.begin() + 3);
//    angle = acos(dotProduct<float>(plane0_normal, plane1_normal));
//    angle = radianToDegree(angle);
//    cout << "angle of two planes: " << angle << "; # of points: " << (seger.cloud_plane)->size() << endl;
//
//    PointColor obj_color(seger.cloud_plane, 60 + 40 * i, 100, 100+30);
//    std::string objID = "plane_cloud_" + std::to_string(i++);
//    viewer->addPointCloud<pcl::PointXYZ>(seger.cloud_plane, obj_color, objID);
//  }


//
//  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seger.reg.getColoredCloud();
//  pcl::visualization::CloudViewer viewer ("Cluster viewer");
//  viewer.showCloud(colored_cloud);

  while (!viewer->wasStopped()) {
      viewer->spinOnce (100);   // ms
      boost::this_thread::sleep (boost::posix_time::microseconds (100));
  }
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


