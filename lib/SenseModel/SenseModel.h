#ifndef __SENSE_MODEL_H__
#define __SENSE_MODEL_H__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include "yolo_v2_class.hpp"

#include "SPAData.h"
#include "realsense.h"

class SenseModel {
   public:
    SenseModel();
    ~SenseModel();

    int SenseInit(Detector& detect);

    void run();
    void showImgs();
    void getImgs(SenseData* senseData);

   private:
    //RealSense cam;
    realsense::RealSense rs;
    sl::Camera zed;
    sl::InitParameters init_parameters;
    sl::RuntimeParameters runtime_parameters;

    std::vector<cv::Mat> rgb;
    std::vector<cv::Mat> depth;

    sl::Mat zedLeftXyz;

    cv::Mat realsenseXyz;
    cv::Mat zedXyz;

    cv::Mat slMat2cvMat(sl::Mat& input);

    void realsenseInit();

    void realsenseProcess();
    void zedProcess();
};

#endif
