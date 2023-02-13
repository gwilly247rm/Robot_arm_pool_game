#include "ActModel.h"
#include <thread>
#include <chrono>

using namespace std;

ActModel::ActModel(const std::string ip, int port, std::string portName,
                   int pingIn)
    : cue(portName, pingIn) {
    armIp = ip;
    armPort = port;
}

ActModel::~ActModel() {
    cout << "-----Act destruct-----" << endl;

    double toolPos[6] = {0, 0, 0, 0, 0, 0};
    double homePos[6] = {0, 0, 0, 0, -90, 0};

    //arm.errorReset();

    // go home
    arm.movePtpAbs(homePos, HiwinSDK::CoordType::Joint);
    arm.waitForIdle();

    arm.setOperationMode(0);
    arm.setCurrentToolCoord(toolPos);
    arm.setServoState(0);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    arm.disconnect();

    cue.close();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cue.cvtr(0);
}

void ActModel::ActInit(PlanData* planData, const YAML::Node& actNode) {
    cout << "-----Act init-----" << endl;

    cue.close();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    planData->action.cuer.ON = 0;
    cue.cvtr(0);
    planData->action.cuer.air = 0;

    double basePos[6] = {0, 0, 0, 0, 0, 0};
    double toolPos[6] = {0, 0, 0, 0, 0, 0};

    arm.connect(armIp, armPort);

    //arm.errorReset();
    arm.setServoState(1);//servo state 1=on
    arm.setOperationMode(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    arm.setActiveBaseNum(3);
    arm.setCurrentBaseCoord(basePos);
    arm.setActiveToolNum(2);
    arm.setCurrentToolCoord(toolPos);
    arm.setOperationMode(1);// 1=run

    arm.setAcceleration(actNode["arm"]["acceleration"].as<int>());
    arm.setFeedRate(80);
    arm.setLinSpeed(1000);
    arm.setPTPSpeed(80);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    double homePos[6] = {0, 0, 0, 0, -90, 0};
    // go home
    arm.movePtpAbs(homePos, HiwinSDK::CoordType::Joint);
    arm.waitForIdle();
}

void ActModel::ActSetTool(const YAML::Node& actNode) {
    array<double, 6> toolPos = actNode["arm"]["tool"].as<std::array<double, 6>>();

    arm.setOperationMode(0);

    arm.setCurrentToolCoord(toolPos);

    arm.setOperationMode(1);// 1=run
    arm.setAcceleration(actNode["arm"]["acceleration"].as<int>());
    arm.setFeedRate(80);
    arm.setLinSpeed(1000);
    arm.setPTPSpeed(80);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void ActModel::run(PlanData* planData, ActData* actData) {
    cout << "-----Act run-----" << endl;
    control_arm(planData->action.act, actData);
    control_cue(planData->action.cuer);

    //arm.errorReset();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    planData->action.act.value = {0, 0, 0, 0, 0, 0};
    planData->action.act.moveType = HiwinSDK::MoveType::Relative;
    planData->action.cuer.ON = 0;
}

void ActModel::setSpeedRate(int rate) {
    cout << "-----Act set speed rate-----" << endl;
    arm.setFeedRate(rate);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void ActModel::setSpeedAcceleration(int a) {
    cout << "-----Act set speed acceleration-----" << endl;
    arm.setAcceleration(a);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void ActModel::control_arm(Act act, ActData* actData) {
    arm.move(act.value, act.ctrlType, act.moveType, act.coordType);
    arm.waitForIdle();

    actData->currentPos = arm.getCurrentPosition();

    for(size_t i = 0; i < actData->currentPos.size(); i++)
        cout << "pos[" << i << "]:" << actData->currentPos[i] << endl;
}

void ActModel::control_cue(Cue cuer) {
    cue.cvtr(cuer.air);

    if (cuer.ON == 1) {
        cue.open();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        cue.close();
    } else
        cue.close();
}
