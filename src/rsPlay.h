//
// Created by yudong on 11/16/18.
//

#ifndef FIRSTTEST_RSPLAY_H
#define FIRSTTEST_RSPLAY_H

#include <string>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "openGLHelper.h"         // Include short list of convenience functions for rendering
#include "pclPlay.h"


namespace rsrs {
    int capture(bool source_from_file = false, const std::string & in_data = "from_camera");
    rs2::points rsPointCloud (bool source_from_file = false, const std::string & in_data = "from_camera");
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points & points);

// Helper functions from openGLHelper.h
//void register_glfw_callbacks(window& app, glfw_state& app_state);

}


#endif //FIRSTTEST_RSPLAY_H
