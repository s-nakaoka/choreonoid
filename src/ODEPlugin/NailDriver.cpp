/*!
  @file
  @author 
*/

#include <cnoid/NailDriver>
#include "NailedObjectManager.h"

#include <cnoid/Device>
#include <cnoid/EigenUtil>

#include <cnoid/ValueTree>

#include "gettext.h"

#include <cnoid/MessageView>
#include <boost/format.hpp>

#include <iostream>

using namespace cnoid;
using namespace std;

const char* NailDriver::typeName()
{
    return "NailDriver";
}

DeviceState* NailDriver::cloneState() const
{
    return new NailDriver(*this, true);
}

int NailDriver::stateSize() const
{
    return 1;
}

Device* NailDriver::clone() const
{
    return new NailDriver(*this);
}

/*
 */
NailDriver::NailDriver()
{
    on_ = false;
    contact_ = false;
    ready_ = false;
    position << 0, 0, 0;
    normal << 0, 0, 0;
    maxFasteningForce = std::numeric_limits<double>::max();

    not_called_conunt = 0;
    near_callback_called = false;
    distantCheckCount = 20;

    resetLatestContact();
}

void NailDriver::copyStateFrom(const NailDriver& other)
{
    on_ = other.on_;
}

void NailDriver::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(NailDriver)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const NailDriver&>(other));
}

NailDriver::NailDriver(const NailDriver& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
    position = org.position;
    normal = org.normal;
    maxFasteningForce = org.maxFasteningForce;
    distantCheckCount = org.distantCheckCount;
}


const double* NailDriver::readState(const double* buf)
{
    on_ = buf[0];
    return buf + 1;
}


double* NailDriver::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    return out_buf + 1;
}

void NailDriver::on(bool on) {
    MessageView::instance()->putln(boost::format(_("*** %s: %s ***")) % typeName() % (on ? "ON" : "OFF"));
    cout << boost::format(_("*** %s: %s ***")) % typeName() % (on ? "ON" : "OFF") << endl;

    if (on_ == false && on == true) {
        // By switching from off to on,
        // it becomes possible to injection of a nail.
        setReady();
    }

    on_ = on;
}

void NailDriver::setReady()
{

    MessageView::instance()->putln(boost::format(_("%s: Ready")) % typeName());
    cout << boost::format(_("%s: Ready")) % typeName() << endl;

    ready_ = true;
}

void NailDriver::fire(NailedObject* nobj)
{

    MessageView::instance()->putln(boost::format(_("%s: Fire")) % typeName());
    cout << boost::format(_("%s: Fire")) % typeName() << endl;

    nobj->addNail(maxFasteningForce);
    ready_ = false;
}

void NailDriver::distantCheck()
{
    if (!near_callback_called) {
        // Check number of times nearCallback() was not called continuously.
        // If more than distantCheckCount times, it is processing as a
        // distant from object.
        if (not_called_conunt < distantCheckCount) {
            not_called_conunt++;
        } else {
            if (on() && !ready()) {
                setReady();
            }
        }
    } else {
        // Since the nearCallback() was called, reset the counter.
        not_called_conunt = 0;
        // And reset the flag.
        near_callback_called = false;
    }
}

int NailDriver::checkContact(int numContacts, dContact* contacts)
{
    Link* link_ = link();
    Vector3 muzzle = link_->p() + link_->R() * position;
#ifdef NAILDRIVER_DEBUG
cout << "NailDriver: muzzle:" << str(muzzle) << endl;
#endif // NAILDRIVER_DEBUG
    int n = 0;
    for (int i=0; i < numContacts; ++i) {
	Vector3 pos(contacts[i].geom.pos);
	Vector3 v(contacts[i].geom.normal);
#ifdef NAILDRIVER_DEBUG
cout << "NailDriver: pos:" << str(pos) << endl;
cout << "NailDriver:   v:" << str(v) << endl;
#endif // NAILDRIVER_DEBUG

	float isParallel = (link_->R() * normal).dot(v);
#ifdef NAILDRIVER_DEBUG
cout << "NailDriver: isParallel: " << isParallel << endl;
#endif // NAILDRIVER_DEBUG

	// Distance gripper (P: muzzle) and contact (A:pos)
	Vector3 pa;
	pa[0] = pos[0] - muzzle[0];
	pa[1] = pos[1] - muzzle[1];
	pa[2] = pos[2] - muzzle[2];

	float distance = fabs(muzzle.dot(pa));
#ifdef NAILDRIVER_DEBUG
cout << "NailDriver: distance: " << distance << endl;
#endif // NAILDRIVER_DEBUG
	if (isParallel < -0.9f && distance < 0.04f) {
	    n++;
	}
    }
    return n;
}
