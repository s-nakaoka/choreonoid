/*!
  @file
*/

#ifndef CNOID_ODEPLUGIN_VACUUMGRIPPER_H
#define CNOID_ODEPLUGIN_VACUUMGRIPPER_H

#include <cnoid/EigenTypes>
#include <cnoid/Body>
#include <cnoid/Link>
#include <string>
#include "exportdecl.h"

#include <ode/ode.h>

namespace cnoid {

class CNOID_EXPORT VacuumGripper : public Device
{
public:
    VacuumGripper();
    VacuumGripper(const VacuumGripper& org, bool copyStateOnly = false);

public:
    virtual const char* typeName();

    virtual void copyStateFrom(const DeviceState& other);
    virtual void copyStateFrom(const VacuumGripper& other);
    virtual DeviceState* cloneState() const;

    virtual int stateSize() const;

    virtual Device* clone() const;

    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    bool on() const { return on_; }
    void on(bool on);
    bool isGripping() const { return jointID != 0; }
    bool isGripping(dBodyID body) const;
    void grip(dWorldID worldId, dBodyID gripped);
    void release();
    int checkContact(int numContacts, dContact* contacts);
    bool limitCheck();
private:
    bool on_;

public:
    Vector3 position;
    Vector3 normal;
    double maxPullForce;
    double maxShearForce;
    double maxPeelTorque;

    dBodyID gripper;
    dJointID jointID;
};

typedef ref_ptr<VacuumGripper> VacuumGripperPtr;
}

#endif
