#include <iostream>
#include <vector>
#include <thread>

#include "sucker.h"

using namespace std;

int main()
{
    SUCKER cue("/dev/ttyACM0", 47);

    cout << "set CVX" << endl;
    cue.cvtr(2); //air volume:0-9
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    cue.close();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cue.open();

    cout << "set CVX" << endl;
    cue.cvtr(4); //air volume:0-9

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    cue.close();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    cue.open();

    return 0;
}

