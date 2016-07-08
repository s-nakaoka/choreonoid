/*!
  @file
  @author 
*/

#include <cnoid/VacuumGripper>

using namespace cnoid;

/*
 */ 
VacuumGripperParams::VacuumGripperParams()
{
    targetObject = "";
    position << 0, 0, 0;
    normalLine << 0, 0, 0;
    maxPullForce = 0;
    maxShearForce = 0;
    maxPeelTorque = 0;
}

/*
 */ 
VacuumGripperParams::~VacuumGripperParams()
{
    ;
}
