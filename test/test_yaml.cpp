#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

using namespace std;

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "parameter:<path of yaml>" << endl;
        return -1;
    }
        
    YAML::Node config = YAML::LoadFile(argv[1]);
    
    double num1 = config["realsense"]["move2cueX"].as<double>();
    double num2 = config["zed"]["move2realsenseX"].as<double>();
        
    array<double, 6> basePos = config["arm"]["tool"].as<std::array<double, 6>>();
        
    cout << num1 << ", " << num2 << endl;
        
    for(size_t i = 0; i < 6; i++)
        cout << i << ":" << basePos[i] << endl;
    
    ofstream fout(argv[1]);
    fout << config;
    fout.close();

    return 0;
}
