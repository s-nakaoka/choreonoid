/*!
  @file
*/

#ifndef CNOID_ODEPLUGIN_VACUUMGRIPPER_H
#define CNOID_ODEPLUGIN_VACUUMGRIPPER_H

#include <cnoid/EigenTypes>
#include <cnoid/Link>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT VacuumGripperParams
{
public:
    VacuumGripperParams();
    ~VacuumGripperParams();

    std::string targetObject;
    Vector3 position;
    Vector3 normalLine;
    double maxPullForce;
    double maxShearForce;
    double maxPeelTorque;
    Link* link;
};

}

#endif
