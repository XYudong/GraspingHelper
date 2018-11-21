//
// Created by yudong on 11/7/18.
//

#ifndef FIRSTTEST_MAIN_H
#define FIRSTTEST_MAIN_H

#include <string>
#include <vector>
#include "pclPlay.h"
#include "cylinder_segmentation.h"

//std::vector<double> getCentroid(const pcl::PointCloud<pcl::PointXYZ> & points);
cylinder_segmentation doCySegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

#endif //FIRSTTEST_MAIN_H





