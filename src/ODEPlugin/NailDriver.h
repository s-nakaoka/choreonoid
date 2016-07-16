/*!
  @file
*/

#ifndef CNOID_ODEPLUGIN_NAILDRIVER_H
#define CNOID_ODEPLUGIN_NAILDRIVER_H

#include <cnoid/EigenTypes>
#include <cnoid/Body>
#include <cnoid/Link>
#include <string>
#include "exportdecl.h"

#include <ode/ode.h>

namespace cnoid {

class CNOID_EXPORT NailDriverParams
{
public:
    static NailDriverParams* findParameter(const Body* info);

    NailDriverParams();
    ~NailDriverParams();

    std::string targetObject;
    Vector3 position;
    Vector3 normalLine;
    double maxFasteningForce;
};

class CNOID_EXPORT NailDriver : public Device
{
public:
    NailDriver();
    NailDriver(const NailDriver& org, bool copyStateOnly = false);

public:
    virtual const char* typeName();

    virtual void copyStateFrom(const DeviceState& other);
    virtual void copyStateFrom(const NailDriver& other);
    virtual DeviceState* cloneState() const;

    virtual int stateSize() const;

    virtual Device* clone() const;

    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }

    void setParam(const NailDriverParams& param);
    int checkContact(int numContacts, dContact* contacts);

private:
    bool on_;

public:
    std::string targetObject;
    Vector3 position;
    Vector3 normalLine;
    double maxFasteningForce;

    dBodyID objId;
};

typedef ref_ptr<NailDriver> NailDriverPtr;

}

#endif
