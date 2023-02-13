#ifndef __SPADATA_H__
#define __SPADATA_H__

#include <cstdlib>  //needed by HiwinSDK.h
#include <opencv2/opencv.hpp>
#include "HiwinSDK.h"
#include <yaml-cpp/yaml.h>

// Sense
struct SenseData {
    std::vector<cv::Mat> rgb;
    std::vector<cv::Mat> depth;
    cv::Mat xyz;
    cv::Mat realMat;
};

// Plan->Act
struct Act {
    HiwinSDK::CoordType coordType;
    HiwinSDK::MoveType moveType;
    HiwinSDK::CtrlType ctrlType;

    std::vector<double> value;
};

struct Cue {
    bool ON;
    int air;
};

struct Action {
    Act act;
    Cue cuer;
};

struct PlanData {
    std::vector<cv::Point2f> Pos;
    double degree;
    double ballR;

    // for track
    double finalDeg;

    Action action;
};

// Act
struct ActData {
    std::vector<double> currentPos;
};

#endif
