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

	/*Game game("./yolov4.cfg", "./yolov4_final.weights",
	"./yolov3-tiny.cfg", "./yolov3_final-realsense.weights",
	"192.168.0.1", 502, "/dev/ttyACM0", 47,
	1, 0, 0);*/

	Game game("./yolov4.cfg", "./yolov4_final.weights",
	"./yolov4-realsense.cfg", "./yolov4_final-realsense.weights",
	"192.168.0.1", 502, "/dev/ttyACM0", 47,
	1, 0, 0);

    int iResult;

    iResult = game.GameInit(config);

    while (iResult == 0)
        iResult = game.regulate(config);

    ofstream fout(argv[1]);
    fout << config;
    fout.close();

    return 0;
}
