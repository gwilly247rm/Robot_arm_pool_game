#ifndef __TRACK_H__
#define __TRACK_H__

#include <iostream>
#define OPENCV
#include <opencv2/opencv.hpp>
#include <yolo_v2_class.hpp>
#include "pid.hpp"

#include "SPAData.h"

class Track {
   public:
    Track();
    ~Track();

    void getSenseData(SenseData* senseData);

    int find_white(Detector& detect, const YAML::Node& findNode);

    int yolo_detect(Detector& detect);
    int hough_circle_detect(const YAML::Node& findNode);
    void cutImage();
    void histogram();
    void white_threshold(const YAML::Node& findNode);
    int find_white_center(const YAML::Node& findNode);

    int pidmove(PlanData* planData, PID* pid, YAML::Node& moveNode);

    void final_move(PlanData* planData, const YAML::Node& moveNode, ActData* actData);
    void final_move2(PlanData* planData);
    int exception_move(PlanData* planData, const YAML::Node& moveNode, ActData* actData);
    void final_move_no_rotate(PlanData* planData, const YAML::Node& moveNode, ActData* actData);

    void move(PlanData* planData, std::vector<double> pos);

   private:
    cv::Mat realSenseRgb;
    cv::Mat realSenseDepth;
    cv::Mat realSenseRealMat;

    std::vector<cv::Mat> cropImages;
    std::vector<cv::Rect> ballRect;

    std::vector<bbox_t> objList;
    std::vector<bbox_t> ballList;
    std::vector<bbox_t> holeList;
    cv::Mat yoloImage;

    int whiteNum;
    cv::Mat white;
    cv::Point whiteCenter;
    std::vector<cv::Vec3f> circles;

    int pidLimit;

    std::vector<double> armPos;

    void DrawBox(cv::Mat img, cv::String name, bbox_t obj);
    std::string getFileName();
};

#endif
