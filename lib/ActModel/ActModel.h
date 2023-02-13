#ifndef __ACTMODEL_H__
#define __ACTMODEL_H__

#include <iostream>
#include <vector>
#include "HiwinSDK.h"
#include "SPAData.h"
#include "sucker.h"

class ActModel {
   public:
    ActModel(const std::string ip, int port, std::string portName, int pingIn);
    ~ActModel();

    void ActInit(PlanData* planData, const YAML::Node& actNode);

    void run(PlanData* planData, ActData* actData);

    void setSpeedRate(int rate);
    void setSpeedAcceleration(int a);
    void ActSetTool(const YAML::Node& actNode);

   private:
    HiwinSDK arm;
    SUCKER cue;

    void control_arm(Act act, ActData* actData);
    void control_cue(Cue cuer);

    std::string armIp;
    int armPort;
};

#endif
