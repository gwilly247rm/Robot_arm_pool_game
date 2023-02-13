#include <iostream>
#include <vector>

//#include "ActModel.h"
#include "HiwinSDK.h"

using namespace std;

int main()
{
    HiwinSDK arm("192.168.0.1", 502);
    
    arm.errorReset();
    arm.setServoState(1);//servo state 1=on
    arm.setOperationMode(1);//1=run
    
    arm.setAcceleration(10);
    arm.setLinSpeed(600);
    
    double homePos[6] = {0, 0, 0, 0, -90, 0};
    // go home
    arm.moveLinAbs(homePos, HiwinSDK::CoordType::Joint);
    arm.waitForIdle();
    
    cout<<"set tool"<<endl;
    double toolPos[6] = {0, 100, -175, 0, 0, 0};
    double Pos[6] = {0, 0, 0, 0, 0, 30};
    
    arm.setOperationMode(0);
    arm.setActiveToolNum(2);
    arm.setCurrentToolCoord(toolPos); 
    arm.setOperationMode(1);
    
    arm.moveLinRel(Pos, HiwinSDK::CoordType::Coord);
    arm.waitForIdle();
    
    // go home
    arm.moveLinAbs(homePos, HiwinSDK::CoordType::Joint);
    arm.waitForIdle();
    
    arm.setOperationMode(0);
    arm.setServoState(0);
  
    return 0;
}

