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
class NailedObject;

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
    void on(bool on);

    int checkContact(int numContacts, dContact* contacts);

    void contact() { near_callback_called = true; }

    void distantCheck();

    bool ready() const { return on_ && ready_; }
    void setReady();

    void fire(NailedObject* nobj);

    void setLatestContact(const double current) { latestContact = current; }
    double getLatestContact() { return latestContact; }
    void resetLatestContact() { latestContact = 0.0; }

private:
    bool on_;
    bool contact_;
    bool ready_;
    int not_called_conunt;
    bool near_callback_called;

public:
    Vector3 position;
    Vector3 normal;
    double maxFasteningForce;
    int distantCheckCount;
    double latestContact;
};

typedef ref_ptr<NailDriver> NailDriverPtr;
}

#endif
