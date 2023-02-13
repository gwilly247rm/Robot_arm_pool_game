#include <iostream>

#include "game.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "parameter:<path of yaml>" << endl;
        return -1;
    }

    YAML::Node config = YAML::LoadFile(argv[1]);

    /*Game game("./../../test/yolov4.cfg", "./../../test/yolov4_final.weights",
    "./../../test/yolov3-tiny.cfg", "./../../test/yolov3_final-realsense.weights",
              "192.168.0.1", 502, "/dev/ttyACM0", 47,
              1, 0, 0);*/

    Game game("./../../test/yolov4.cfg", "./../../test/yolov4_final.weights",
    "./../../test/yolov4-realsense.cfg", "./../../test/yolov4_final-realsense.weights",
              "192.168.0.1", 502, "/dev/ttyACM0", 47,
              1, 0, 0);

    int iResult;

    iResult = game.GameInit(config);

    while (iResult == 0)
    {
        iResult = game.champion(config);

        /*ofstream fout(argv[1]);
        fout << config;
        fout.close();

        config = YAML::LoadFile(argv[1]);*/
    }

    return 0;
}
