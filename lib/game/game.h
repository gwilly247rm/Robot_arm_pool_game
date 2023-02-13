#ifndef __GAME_H__
#define __GAME_H__

#include <iostream>
#include <opencv2/opencv.hpp>
#include "pid.hpp"
#include "yolo_v2_class.hpp"
#include <yaml-cpp/yaml.h>

#include "ActModel.h"
#include "SPAData.h"
#include "SenseModel.h"
#include "find.h"
#include "plan.h"
#include "track.h"

class HideText {
   public:
    HideText();
    ~HideText();
    void openOutput();
};

class Game {
   public:
    Game(std::string cfg, std::string weights, std::string cfg2, std::string weights2,
         const std::string ip, int port, std::string portName, int pingIn,
         double Kp, double Ki, double Kd);

    ~Game();

    int GameInit(const YAML::Node& actNode);
    int champion(YAML::Node& moveNode);
    int process(YAML::Node& moveNode);
    int regulate(YAML::Node& moveNode);

   private:
    Detector* detect;
    Detector* detect2;
    PID pid;
    SenseModel senseModel;
    Find find;
    Plan plan;
    Track track;
    ActModel actModel;

    SenseData senseData;
    PlanData planData;
    ActData actData;

    std::vector<double> regulatePos;

    int count;
    void show_count();

    void armPreparePos();
    void armGoSeeDegree();
    void armGoGameDegree();
    void armGoGamePos();
    void armGohome();
    int armMove();

    double input;
};

#endif
