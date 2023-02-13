#include "game.h"
#include "cmath"

using namespace std;
using namespace cv;

Game::Game(std::string cfg, std::string weights, std::string cfg2,
           std::string weights2, const std::string ip, int port,
           std::string portName, int pingIn, double Kp, double Ki, double Kd)
    : pid(Kp, Ki, Kd, 0), actModel(ip, port, portName, pingIn) {
    HideText textHandler;
    detect = new Detector(cfg, weights);
    detect2 = new Detector(cfg2, weights2);

    count = 0;
}

Game::~Game() {}

int Game::GameInit(const YAML::Node& actNode) {
    cout << "-----Game init-----" << endl;

    int iResult;

    iResult = senseModel.SenseInit(*detect);

    if (iResult != 0) cout << "Sense Init failed" << endl;

    actModel.ActInit(&planData, actNode);

    actModel.ActSetTool(actNode);

    return iResult;
}

int Game::champion(YAML::Node& node) {
    cout << "-----Game champion-----" << endl;

    int run = 1;
    int iResult = 0;
    int interrupt = 0;

    char key;

    /*armPreparePos();
    actModel.run(&planData, &actData);*/

    armGohome();
    actModel.run(&planData, &actData);

    armGoGameDegree();
    actModel.run(&planData, &actData);

    count++;

    //press buttom

    while(1)
    {
        show_count();

        key = waitKey(0);

        cout << "key:" << key <<  endl;

        if(key == '7')
		{
            break;
		}

        else if (key == 'q')
        {
            interrupt = 1;
            break;
        }
    }

    destroyAllWindows();

    /*armGoSeeDegree();
    actModel.run(&planData, &actData);*/

    //find degree
    while (run == 1 && interrupt ==  0) {
        senseModel.run();
        senseModel.getImgs(&senseData);
        find.getRgbImgs(&senseData);
        iResult = find.find_degree(&planData,node);

        if(iResult == 1)
        {
            armGoGamePos();
            actModel.run(&planData, &actData);
            run = 0;
        }
    }

    //find hole & ball
    run = 1;
    while (run == 1 && interrupt == 0) {
        senseModel.run();
        senseModel.getImgs(&senseData);
        // senseModel.showImgs();

        find.getRgbImgs(&senseData);
        iResult = find.run(*detect, &planData, node);

        if (iResult == 1) {
            plan.getPos(&planData);
            interrupt = plan.run(&senseData, &planData, node);
            run = 0;
          }
    }

    //plan
    if(interrupt == 0)
    {
        //plan.getPos(&planData);
        //interrupt = plan.run(&senseData, &planData, node);
        actModel.run(&planData, &actData);
        // waitKey(0);
        destroyAllWindows();
    }

    // track
    run = 1;
	actModel.setSpeedAcceleration(80);
    while(run == 1 && interrupt == 0)
    {
        senseModel.run();
        senseModel.getImgs(&senseData);
        track.getSenseData(&senseData);
        iResult = track.find_white(*detect2, node);
        waitKey(1);

        if(iResult == 1){
            run = track.pidmove(&planData, &pid, node);

            if(run == 1)
                actModel.run(&planData, &actData);
            else
                break;
        }
    }

	actModel.setSpeedAcceleration(node["arm"]["acceleration"].as<int>());

    //final move
    if(interrupt == 0)
    {
        track.final_move(&planData, node, &actData);
        actModel.run(&planData, &actData);

        run = track.exception_move(&planData, node, &actData);

        if(run == 1)
            actModel.run(&planData, &actData);

        actModel.setSpeedRate(30);
        track.final_move2(&planData);
        actModel.run(&planData, &actData);

        //waitKey(0);
    }

    actModel.setSpeedRate(80);

    destroyAllWindows();

    return interrupt;
  }

int Game::process(YAML::Node& node) {
    cout << "-----Game process-----" << endl;

    int run = 1;
    int iResult = 0;
    int interrupt = 0;

    char key;

    armGohome();
    actModel.run(&planData, &actData);

    armGoGameDegree();
    actModel.run(&planData, &actData);

    //find degree
    while (run == 1) {
        senseModel.run();
        senseModel.getImgs(&senseData);
        find.getRgbImgs(&senseData);
        iResult = find.find_degree(&planData,node);

        if(iResult == 1)
        {
            armGoGamePos();
            actModel.run(&planData, &actData);
            run = 0;
        }

        key = waitKey(1);

        if (key == 'q')
        {
            interrupt = 1;
            break;
        }
    }

    //find hole & ball
    run = 1;
    while (run = 1 && interrupt ==  0) {
        senseModel.run();
        senseModel.getImgs(&senseData);
        // senseModel.showImgs();

        find.getRgbImgs(&senseData);
        // find.getDepthImgs(&senseData);
        iResult = find.run(*detect, &planData, node);

        key = waitKey(1);

        if (iResult == 1) {
            if (key == 27)
                break;
            plan.getPos(&planData);
            //plan.easyBall_run(&senseData, &planData);
            interrupt = plan.run(&senseData, &planData, node);
            run = 0;
        }

        if (key == 'q')
        {
            interrupt = 1;
            break;
        }
    }

    /*if (waitKey(0) == 'q')
        interrupt = 1;*/

    //plan
    if(interrupt == 0)
    {
        //destroyAllWindows();
        //plan.getPos(&planData);
        //interrupt = plan.run(&senseData, &planData, node);
        actModel.run(&planData, &actData);
        // waitKey(0);
        destroyAllWindows();
    }

    //show senseImgs
    /*while(1)
    {
        senseModel.run();
        senseModel.getImgs(&senseData);
        senseModel.showImgs();

        if (waitKey(10) == 27) break;
    }*/

    // track
    run = 1;
	actModel.setSpeedAcceleration(80);
    while(run == 1 && interrupt == 0)
    {
        senseModel.run();
        senseModel.getImgs(&senseData);
        track.getSenseData(&senseData);
        iResult = track.find_white(*detect2, node);
        waitKey(1);

        if(iResult == 1){
            run = track.pidmove(&planData, &pid, node);

            if(run == 1)
                actModel.run(&planData, &actData);

            else
                break;
        }

        if (waitKey(1) == 'q')
        {
            interrupt = 1;
            break;
        }
    }
	actModel.setSpeedAcceleration(node["arm"]["acceleration"].as<int>());

    //final move
    if(interrupt == 0)
    {
        track.final_move(&planData, node, &actData);
        actModel.run(&planData, &actData);

        run = track.exception_move(&planData, node, &actData);
        if(run == 1)
            actModel.run(&planData, &actData);

        actModel.setSpeedRate(30);
        track.final_move2(&planData);
        actModel.run(&planData, &actData);

        //waitKey(0);
    }

    actModel.setSpeedRate(80);

    destroyAllWindows();

    return interrupt;
}

int Game::regulate(YAML::Node& node) {
    cout << "-----Game regulate-----" << endl;

    regulatePos.resize(3);

    int run = 1;
    int iResult = 0;
    int interrupt = 0;

    char key;

    armGohome();
    actModel.run(&planData, &actData);

    armGoGameDegree();
    actModel.run(&planData, &actData);

    //find degree
    while (run == 1) {
        senseModel.run();
        senseModel.getImgs(&senseData);
        find.getRgbImgs(&senseData);
        iResult = find.find_degree(&planData, node);

        if(iResult == 1)
        {
            armGoGamePos();
            actModel.run(&planData, &actData);
            run = 0;
        }
    }

    //find hole & ball
    run = 1;
    while (run = 1 && interrupt ==  0) {
        senseModel.run();
        senseModel.getImgs(&senseData);

        find.getRgbImgs(&senseData);
        iResult = find.run(*detect, &planData, node);

        if (iResult == 1) {
            plan.getPos(&planData);
            interrupt = plan.run(&senseData, &planData, node);
            run = 0;
          }

        key = waitKey(5);

        if (key == 27)
            break;
        else if (key == 'q')
        {
            interrupt = 1;
            break;
        }
    }

    //plan
    if(interrupt == 0)
    {
        destroyAllWindows();
        actModel.run(&planData, &actData);
    }

    // track
    run = 1;
	actModel.setSpeedAcceleration(80);

    while(run == 1 && interrupt == 0)
    {
        senseModel.run();
        senseModel.getImgs(&senseData);
        track.getSenseData(&senseData);
        iResult = track.find_white(*detect2, node);

        if(iResult == 1){
            run = track.pidmove(&planData, &pid, node);

            if(run == 1)
                actModel.run(&planData, &actData);

            else
                break;
        }

        if (waitKey(1) == 'q')
        {
            interrupt = 1;
            break;
        }
    }

	actModel.setSpeedAcceleration(node["arm"]["acceleration"].as<int>());

    //final move
    if(interrupt == 0)
    {
        track.final_move_no_rotate(&planData, node, &actData);
        actModel.run(&planData, &actData);

        run = track.exception_move(&planData, node, &actData);
        if(run == 1)
            actModel.run(&planData, &actData);

        actModel.setSpeedRate(30);
        track.final_move2(&planData);
        actModel.run(&planData, &actData);
    }

    actModel.setSpeedRate(80);

    input = 1;

	actModel.setSpeedAcceleration(80);
    while (1)
    {
        if (armMove() == 1)
            break;
    }
	actModel.setSpeedAcceleration(node["arm"]["acceleration"].as<int>());

    node["realsense"]["move2cueX"] =
    node["realsense"]["move2cueX"].as<double>() + regulatePos[0];
    node["realsense"]["move2cueY"] =
    node["realsense"]["move2cueY"].as<double>() + regulatePos[1];
    node["realsense"]["move2cueZ"] =
    node["realsense"]["move2cueZ"].as<double>() + regulatePos[2];

    regulatePos.clear();
    regulatePos.resize(3);

    planData.action.act.coordType = HiwinSDK::CoordType::Coord;
    planData.action.act.moveType = HiwinSDK::MoveType::Relative;
    planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
    planData.action.act.value = {0, 0, 50, 0, 0, 0};

    actModel.run(&planData, &actData);

    while(1)
    {
        key = waitKey(0);

        if (key == 'q') {
            break;
        }

        else if (key == 'd') {
            planData.action.act.coordType = HiwinSDK::CoordType::Coord;
            planData.action.act.moveType = HiwinSDK::MoveType::Relative;
            planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
            planData.action.act.value = {0, 0, 0, 0, 0, -90};
        }

        /*else if (key == 'a') {
            planData.action.act.coordType = HiwinSDK::CoordType::Coord;
            planData.action.act.moveType = HiwinSDK::MoveType::Relative;
            planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
            planData.action.act.value = {0, 0, 0, 0, 0, 90};
        }*/

        actModel.run(&planData, &actData);
    }

    planData.action.act.coordType = HiwinSDK::CoordType::Coord;
    planData.action.act.moveType = HiwinSDK::MoveType::Relative;
    planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
    planData.action.act.value = {0, 0, -50, 0, 0, 0};

	actModel.run(&planData, &actData);

	actModel.setSpeedAcceleration(100);
	while (1)
	{
		if (armMove() == 1)
			break;
	}
	actModel.setSpeedAcceleration(node["arm"]["acceleration"].as<int>());

    array<double, 6> toolPos = node["arm"]["tool"].as<std::array<double, 6>>();

    toolPos[0] = toolPos[0] + regulatePos[1]/2;
    toolPos[1] = toolPos[1] + regulatePos[0]/2;

    node["arm"]["tool"] = toolPos;

    destroyAllWindows();
    armGohome();
    actModel.run(&planData, &actData);

    return 1;
}

void Game::armPreparePos()
{
    planData.action.act.coordType = HiwinSDK::CoordType::Joint;
    planData.action.act.moveType = HiwinSDK::MoveType::Absolute;
    planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
    planData.action.act.value = {-90, 13.396, -11.825, 0, -91.571, 0};
}

void Game::armGoSeeDegree()
{
    planData.action.act.coordType = HiwinSDK::CoordType::Joint;
    planData.action.act.moveType = HiwinSDK::MoveType::Relative;
    planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
    planData.action.act.value = {90, 0, 0, 0, 0, 0};
}

void Game::armGoGameDegree()
{
    planData.action.act.coordType = HiwinSDK::CoordType::Coord;
    planData.action.act.moveType = HiwinSDK::MoveType::Relative;
    planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
    planData.action.act.value = {0, -80, 0, 0, 0, 0};
}

void Game::armGoGamePos()
{
    planData.action.act.coordType = HiwinSDK::CoordType::Coord;
    planData.action.act.moveType = HiwinSDK::MoveType::Relative;
    planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
    planData.action.act.value = {0, 20, 120, 0, 0, 0};
}

void Game::armGohome() {
    cout << "-----arm go home-----" << endl;

    planData.action.act.coordType = HiwinSDK::CoordType::Joint;
    planData.action.act.moveType = HiwinSDK::MoveType::Absolute;
    planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
    planData.action.act.value = {0, 0, 0, 0, -90, 0};

    planData.action.cuer.ON = 0;
}

int Game::armMove() {
    cout << "input" << input << endl;

    char key;
    key = waitKey(0);

    if (key == 'q') {
        return 1;
    }

    else if (key == 'd') {
        planData.action.act.coordType = HiwinSDK::CoordType::Coord;
        planData.action.act.moveType = HiwinSDK::MoveType::Relative;
        planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
        planData.action.act.value = {input, 0, 0, 0, 0, 0};

        regulatePos[0] = regulatePos[0] + input;
    }

    else if (key == 'a') {
        planData.action.act.coordType = HiwinSDK::CoordType::Coord;
        planData.action.act.moveType = HiwinSDK::MoveType::Relative;
        planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
        planData.action.act.value = {-input, 0, 0, 0, 0, 0};

        regulatePos[0] = regulatePos[0] - input;
    }

    else if (key == 'w') {
        planData.action.act.coordType = HiwinSDK::CoordType::Coord;
        planData.action.act.moveType = HiwinSDK::MoveType::Relative;
        planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
        planData.action.act.value = {0, input, 0, 0, 0, 0};

        regulatePos[1] = regulatePos[1] + input;
    }

    else if (key == 's') {
        planData.action.act.coordType = HiwinSDK::CoordType::Coord;
        planData.action.act.moveType = HiwinSDK::MoveType::Relative;
        planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
        planData.action.act.value = {0, -input, 0, 0, 0, 0};

        regulatePos[1] = regulatePos[1] - input;
    }

    else if (key == 'r') {
        planData.action.act.coordType = HiwinSDK::CoordType::Coord;
        planData.action.act.moveType = HiwinSDK::MoveType::Relative;
        planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
        planData.action.act.value = {0, 0, input, 0, 0, 0};

        regulatePos[2] = regulatePos[2] + input;
    }

    else if (key == 'f') {
        planData.action.act.coordType = HiwinSDK::CoordType::Coord;
        planData.action.act.moveType = HiwinSDK::MoveType::Relative;
        planData.action.act.ctrlType = HiwinSDK::CtrlType::PTP;
        planData.action.act.value = {0, 0, -input, 0, 0, 0};

        regulatePos[2] = regulatePos[2] - input;
    }

    else if (key == 'j') {
        input = input * 2;
        return 0;
    }

    else if (key == 'k') {
        input = input / 2;
        return 0;
    }

    else
    {
        return 0;
    }

    actModel.run(&planData, &actData);

    cout << "x:" << regulatePos[0] << ", y:" << regulatePos[1]
         << ", z:" << regulatePos[2] << endl;

    return 0;
}

void Game::show_count()
{
    int font_face = FONT_HERSHEY_SCRIPT_SIMPLEX;
    int baseline;

    Mat image = Mat::zeros(Size(500, 500), CV_8UC3);

    image.setTo(Scalar(0, 0, 0));

    string text = to_string(count);

    Size text_size = getTextSize(text, font_face, 10, 8, &baseline);

    Point origin;
    origin.x = image.cols / 2 - text_size.width / 2;
    origin.y = image.rows / 2 + text_size.height / 2;

    putText(image, text, origin, font_face, 10, Scalar(0, 255, 255), 8, 8, 0);

    imshow("count", image);
}

HideText::HideText() {
    FILE* ret = freopen("/dev/null", "w", stderr);
    if (!ret) {
        printf("Close stderr error\n");
    }
    ret = freopen("/dev/null", "w", stdout);
    if (!ret) {
        printf("Close stdout error\n");
    }
}
HideText::~HideText() { this->openOutput(); }
void HideText::openOutput() {
    FILE* ret = freopen("/dev/tty", "w", stderr);
    if (!ret) {
        printf("Reopen stderr error\n");
    }
    ret = freopen("/dev/tty", "w", stdout);
    if (!ret) {
        printf("Reopen stdout error\n");
    }
}
