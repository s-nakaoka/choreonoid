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

class CNOID_EXPORT VacuumGripperParams
{
public:
    static VacuumGripperParams* findParameter(const Body* info);

    VacuumGripperParams();
    ~VacuumGripperParams();

    std::string targetObject;
    Vector3 position;
    Vector3 normalLine;
    bool pullForceUnlimited;
    double maxPullForce;
    bool shearForceUnlimited;
    double maxShearForce;
    bool peelTorqueUnlimited;
    double maxPeelTorque;
};

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
#if 0
    void on(bool on) { on_ = on; }
#else
    void on(bool on);
#endif
    void setParam(const VacuumGripperParams& param);
private:
    bool on_;

public:
    std::string targetObject;
    Vector3 position;
    Vector3 normalLine;
    bool pullForceUnlimited;
    double maxPullForce;
    bool shearForceUnlimited;
    double maxShearForce;
    bool peelTorqueUnlimited;
    double maxPeelTorque;

    dJointID jointID;
};

typedef ref_ptr<VacuumGripper> VacuumGripperPtr;

}

#endif