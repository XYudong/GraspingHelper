//
// Created by yudong on 11/16/18.
//

#ifndef FIRSTTEST_OPENGLHELPER_H
#define FIRSTTEST_OPENGLHELPER_H

//
// Created by yudong on 11/15/18.
//
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

//#pragma once

#define GLFW_INCLUDE_GLU

#include <librealsense2/rs.hpp>
#include <GLFW/glfw3.h>

#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

//////////////////////////////
// Basic Data Types         //
//////////////////////////////

struct float3 { float x, y, z; };
struct float2 { float x, y; };

struct rect
{
    float x, y;
    float w, h;

    // Create new rect within original boundaries with give aspect ration
    rect adjust_ratio(float2 size) const
    {
        auto H = static_cast<float>(h), W = static_cast<float>(h) * size.x / size.y;
        if (W > w)
        {
            auto scale = w / W;
            W *= scale;
            H *= scale;
        }

        return{ x + (w - W) / 2, y + (h - H) / 2, W, H };
    }
};

//////////////////////////////
// Simple font loading code //
//////////////////////////////

#include "../third-party/stb_easy_font.h"

inline void draw_text(int x, int y, const char * text)
{
    char buffer[60000]; // ~300 chars
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 16, buffer);
    glDrawArrays(GL_QUADS, 0, 4 * stb_easy_font_print((float)x, (float)(y - 7), (char *)text, nullptr, buffer, sizeof(buffer)));
    glDisableClientState(GL_VERTEX_ARRAY);
}

////////////////////////
// Image display code //
////////////////////////
class texture
{
public:
    void render(const rs2::frameset& frames, int window_width, int window_height);

    void render(const rs2::video_frame& frame, const rect& r);

    void upload(const rs2::video_frame& frame);

    GLuint get_gl_handle() { return gl_handle; }

    void show(const rect& r) const;

private:
    GLuint gl_handle = 0;
    int width = 0;
    int height = 0;
    rs2_stream stream = RS2_STREAM_ANY;

    bool can_render(const rs2::frame& f) const;

    rect calc_grid(float2 window, int streams);

    std::vector<rect> calc_grid(float2 window, std::vector<rs2::video_frame>& frames);
};

class window
{
public:
    std::function<void(bool)>           on_left_mouse = [](bool) {};
    std::function<void(double, double)> on_mouse_scroll = [](double, double) {};
    std::function<void(double, double)> on_mouse_move = [](double, double) {};
    std::function<void(int)>            on_key_release = [](int) {};

    window(int width, int height, const char* title);

    float width() const { return float(_width); }
    float height() const { return float(_height); }

    operator bool();

    ~window();

    operator GLFWwindow*() { return win; }

private:
    GLFWwindow * win;
    int _width, _height;
};

// Struct for managing rotation of pointcloud view
struct glfw_state {
    glfw_state() : yaw(15.0), pitch(15.0), last_x(0.0), last_y(0.0),
                   ml(false), offset_x(2.f), offset_y(2.f), tex() {}
    double yaw;
    double pitch;
    double last_x;
    double last_y;
    bool ml;
    float offset_x;
    float offset_y;
    texture tex;
};

namespace glgl {
    // Handles all the OpenGL calls needed to display the point cloud
    void draw_pointcloud(float width, float height, glfw_state& app_state, rs2::points& points);

// Registers the state variable and callbacks to allow mouse control of the pointcloud
    void register_glfw_callbacks(window& app, glfw_state& app_state);
}


#endif //FIRSTTEST_OPENGLHELPER_H
