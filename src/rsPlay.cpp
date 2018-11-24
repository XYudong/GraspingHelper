//
// Created by yudong on 11/16/18.
//

#include "rsPlay.h"

#include <algorithm>    // min, max
#include <string>

#include <librealsense2/rs.hpp>     // Include RealSense Cross Platform API
#include "openGLHelper.h"         // Include short list of convenience functions for rendering
#include "pclPlay.h"


typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;

int rsrs::capture(bool source_from_file, const std::string & in_data) {
    ///  streaming and rendering Depth & RGB data to the screen
    try
    {
        rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
        // Create a simple OpenGL window for rendering:
        window app(1280, 720, "RealSense Capture Example");
        // Declare two textures on the GPU, one for color and one for depth
        texture depth_image, color_image;

        // Declare depth colorizer for pretty visualization of depth data
        rs2::colorizer color_map;

        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;
        // Start streaming with default recommended configuration
        if (source_from_file) {
            rs2::config cfg;
            cfg.enable_device_from_file(in_data);
            pipe.start(cfg); // Load from file
        } else {
            pipe.start();
        }


        while(app) // Application still alive?
        {
            rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

            rs2::frame depth = color_map.process(data.get_depth_frame()); // Find and colorize the depth data
            rs2::frame color = data.get_color_frame();            // Find the color data

            // For cameras that don't have RGB sensor, we'll render infrared frames instead of color
            if (!color)
                color = data.get_infrared_frame();

            // Render depth on to the first half of the screen and color on to the second
            depth_image.render(depth, { 0,               0, app.width() / 2, app.height() });
            color_image.render(color, { app.width() / 2, 0, app.width() / 2, app.height() });
        }

        return EXIT_SUCCESS;
    }
    catch (const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

rs2::points rsrs::rsPointCloud (bool source_from_file, const std::string & in_data) {
    try
    {
        // Create a simple OpenGL window for rendering:
        window app(1280, 720, "RealSense Pointcloud Example");
        // Construct an object to manage view state
        glfw_state app_state;
        // register callbacks to allow manipulation of the pointcloud
        glgl::register_glfw_callbacks(app, app_state);

        // Declare pointcloud object, for calculating pointclouds and texture mappings
        rs2::pointcloud pc;
        // We want the points object to be persistent so we can display the last cloud when a frame drops
        rs2::points points;

        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;
        // Start streaming with default recommended configuration
        if (source_from_file) {
            rs2::config cfg;
            cfg.enable_device_from_file(in_data);
            pipe.start(cfg); // Load from file
        } else {
            pipe.start();
        }

        int t = 0;
        while (app) // Application still alive?
        {
            // Wait for the next set of frames from the camera
            auto frames = pipe.wait_for_frames(100000);

            auto depth = frames.get_depth_frame();

            // Generate the rs points and texture mappings
            points = pc.calculate(depth);

            // save each frame to PCD file
            t++;        // to skip the beginning stage
            if (t % 5 == 0) {
                pcl_ptr pcl_cloud = points_to_pcl(points);
                std::string out_file = "../../data/output/test_" + std::to_string(t/5 - 1) + ".pcd";
                pclpcl::savePCD(pcl_cloud, out_file);
                std::cout << "saving PCD file: " + std::to_string(t) << "\n";
            }

            auto color = frames.get_color_frame();

            // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
            if (!color)
                color = frames.get_infrared_frame();

            // Tell pointcloud object to map to this color frame
            pc.map_to(color);

            // Upload the color frame to OpenGL
            app_state.tex.upload(color);

            // Draw the pointcloud
            glgl::draw_pointcloud(app.width(), app.height(), app_state, points);
        }

        // this will just return the last frame
        return points;
//        return EXIT_SUCCESS;
    }
    catch (const rs2::error & e)
    {
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//        return EXIT_FAILURE;
    }
    catch (const std::exception & e)
    {
        std::cerr << e.what() << std::endl;
//        return EXIT_FAILURE;
    }

}

pcl_ptr rsrs::points_to_pcl(const rs2::points& points) {
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}