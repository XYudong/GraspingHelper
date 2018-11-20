//
// Created by yudong on 11/7/18.
//
#include "cvPlay.h"

using namespace cv;
using namespace cv::xfeatures2d;

void captureVideo() {
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "fail to open the camera" << std::endl;
    }

    Mat edges;
    namedWindow("edges", 1); // WINDOW_AUTOSIZE

    // read the video stream
    while (true) {
        Mat frame;
        cap >> frame;
        cvtColor(frame, frame, 0);
        float sigmaX = 1.5;
        float sigmaY = 1.5;
        GaussianBlur(frame, frame, Size(7, 7), sigmaX, sigmaY);

        int apertureSize = 3;
        Canny(frame, edges, 40, 70, apertureSize);

        imshow("edges", edges);
        // Press 'q' to escape
        int c = waitKey(10);
        if ((char) c == 'q') { break; }
    }
}
