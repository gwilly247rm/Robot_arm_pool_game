#ifndef __PLAN_H__
#define __PLAN_H__

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "SPAData.h"

class Plan {
   public:
    Plan();
    ~Plan();

    void getPos(PlanData* planData);

    int run(SenseData* senseData, PlanData* planData, const YAML::Node& moveNode);
    void easyBall_run(SenseData* senseData, PlanData* planData, const YAML::Node& moveNode);

    int noBall();
    int manyBall(PlanData* planData);
    int easyBall(PlanData* planData);
    int kissBall(PlanData* planData);
    void randomBall(PlanData* planData);

    int dot(cv::Point v1, cv::Point v2);

   private:
    std::vector<cv::Point2f> plan_pos;

    cv::Point predict_point;

    cv::Point2f whiteXYZPos;
    cv::Point2f predictXYZPos;

    void calcDeg(SenseData* senseData, PlanData* planData);
    cv::Point2f pixel2xyzPos(SenseData* senseData, cv::Point pixel);
    void move(PlanData* planData, const YAML::Node& moveNode);

    double ballR;

    void draw();

    cv::Mat input;
};

#endif
