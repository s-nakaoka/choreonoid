/*!
  @file
  @author 
*/

#include <cnoid/VacuumGripper>

#include <cnoid/Device>
#include <cnoid/EigenUtil>

#include <cnoid/ValueTree>

#include "gettext.h"

#include <cnoid/MessageView>
#include <boost/format.hpp>

#include <iostream>

using namespace cnoid;
using namespace std;

const char* VacuumGripper::typeName()
{
    return "VacuumGripper";
}

DeviceState* VacuumGripper::cloneState() const
{
    return new VacuumGripper(*this, true);
}

int VacuumGripper::stateSize() const
{
    return 1;
}

Device* VacuumGripper::clone() const
{
    return new VacuumGripper(*this);
}

/*
 */
VacuumGripper::VacuumGripper()
{
    on_ = false;
    jointID = 0;
    position << 0, 0, 0;
    normal << 0, 0, 0;
    maxPullForce = std::numeric_limits<double>::max();
    maxShearForce = std::numeric_limits<double>::max();
    maxPeelTorque = std::numeric_limits<double>::max();
}

void VacuumGripper::copyStateFrom(const VacuumGripper& other)
{
    on_ = other.on_;
}

void VacuumGripper::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(VacuumGripper)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const VacuumGripper&>(other));
}

VacuumGripper::VacuumGripper(const VacuumGripper& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
    jointID = org.jointID;
    position = org.position;
    normal = org.normal;
    maxPullForce = org.maxPullForce;
    maxShearForce = org.maxShearForce;
    maxPeelTorque = org.maxPeelTorque;
}


const double* VacuumGripper::readState(const double* buf)
{
    on_ = buf[0];
    return buf + 1;
}


double* VacuumGripper::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    return out_buf + 1;
}

void VacuumGripper::on(bool on) {
    MessageView::instance()->putln(boost::format(_("*** %s: %s ***")) % typeName() % (on ? "ON" : "OFF"));
    cout << boost::format(_("*** %s: %s ***")) % typeName() % (on ? "ON" : "OFF") << endl;
    on_ = on;
}

void VacuumGripper::release()
{
    if (!isGripping()) return;

    dJointSetFeedback(jointID, 0);
    dJointDestroy(jointID);
    jointID = 0;
    MessageView::instance()->putln("VacuumGripper: *** joint destroy : turned off ***");
    cout << "VacuumGripper: *** joint destroy : turned off **" << endl;
}

int VacuumGripper::checkContact(int numContacts, dContact* contacts)
{
    Vector3 vacuumPos = link()->p() + link()->R() * position;

    int n = 0;
    for(int i=0; i < numContacts; ++i){
	Vector3 pos(contacts[i].geom.pos);
	Vector3 v(contacts[i].geom.normal);

	float isParallel = (link()->R() * normal).dot(v);

	// Distance gripper (P: vacuumPos) and contact (A:pos)
	Vector3 pa;
	pa[0] = pos[0] - vacuumPos[0];
	pa[1] = pos[1] - vacuumPos[1];
	pa[2] = pos[2] - vacuumPos[2];

	float distance = abs(vacuumPos.dot(pa));
	if (isParallel < -0.9f && distance < 0.01f) {
	    n++;
	}
    }
    return n;
}
