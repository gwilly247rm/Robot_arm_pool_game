#ifndef __FIND_H__
#define __FIND_H__

#include <cmath>
#include <iostream>
#define OPENCV
#include <opencv2/opencv.hpp>
#include <yolo_v2_class.hpp>

#include "SPAData.h"

class Find {
   public:
    Find();
    ~Find();

    int run(Detector& detect, PlanData* planData, const YAML::Node& findNode);

    void getRgbImgs(SenseData* senseData);
    void getDepthImgs(SenseData* senseData);

    int getSampleImg();
    int getTablePosition();

    void yolo_detect(Detector* detect);

    int find_degree(PlanData* planData, const YAML::Node& findNode);
    int find_hole();
    int find_ball(PlanData* planData, const YAML::Node& findNode);

    void drawLines(cv::Mat input, const std::vector<cv::Vec2f>& lines);
    double calcDeg(const std::vector<cv::Vec2f>& lines);
    void cutImage();
    int histogram();
    int white_threshold(const YAML::Node& findNode);

    void save_ball_pos();

   private:
    std::vector<cv::Point> pos;

    cv::Mat drawLine;
    cv::Mat gray;
    cv::Mat canny;
    std::vector<cv::Vec2f> lines;
    std::vector<double> degs;
    double degree;
    double ballR;

    cv::Mat zedRgb;
    cv::Mat zedDepth;
    cv::Mat realSenseRgb;
    cv::Mat realSenseDepth;
    cv::Mat zedSmallRgb;
    cv::Mat zedDetectRgb;
    cv::Mat zedSmallDetectRgb;

    int iResult1;
    int iResult2;
    int iResult3;

    std::vector<cv::Mat> cropImages;

    std::vector<bbox_t> objList;
    std::vector<bbox_t> ballList;
    std::vector<bbox_t> holeList;
    
    std::vector<cv::Point> whiteList;

    cv::Point ballPosition1;
    cv::Point ballPosition2;
    cv::Point previousPoint;

    std::vector<cv::Point> tablePosition;

    void pixel2xyzPos(SenseData* senseData);
    void rotate();
    void DrawBox(cv::Mat img, cv::String name, bbox_t obj);
    static void onMouse(int event, int x, int y, int flags, void* ptr);
    cv::Mat resizeImage(cv::Mat img);
};

#endif
