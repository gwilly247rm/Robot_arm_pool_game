#include <iostream>
#include <vector>
#include <thread>

#include "sucker.h"

using namespace std;

int main()
{
    SUCKER cue("/dev/ttyACM0", 47);

    cout << "open cue" << endl;

    cue.close();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    cue.open();

    return 0;
}

