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

/**
 * Create Vector3 object from list parameter.
 */
#if 1 // Later define a common function
static Vector3 listToVector3(const Listing* list)
{
    MessageView::instance()->putln(boost::format(_("list size=%d")) % list->size());
    double a = (*list)[0].toDouble();
    double b = (*list)[1].toDouble();
    double c = (*list)[2].toDouble();

    MessageView::instance()->putln(boost::format(_("[%e, %e, %e]")) % a % b % c);
    return Vector3(a, b, c);
}
#endif // Later define a common function

/**
 * Get VacuumGripper parameter.
 */
VacuumGripperParams* VacuumGripperParams::findParameter(const Body* body)
{
    Mapping* m = body->info()->findMapping("vacuumGripper");

    if (!m->isValid()) {
	MessageView::instance()->putln(boost::format(_("%s not has vacuum gripper")) % body->name());
	return 0;
    }

    MessageView::instance()->putln(boost::format(_("%s has vacuum gripper")) % body->name());

    VacuumGripperParams* params = new VacuumGripperParams();

    std::string targetObject = "";
    double maxPullForce = 0;
    double maxShearForce = 0;
    double maxPeelTorque = 0;

    targetObject = m->get("targetObject", targetObject);
    params->targetObject = targetObject;
    MessageView::instance()->putln(boost::format(_("  targetObject: %s")) % targetObject);

    const Listing* position = m->findListing("position");
    params->position = listToVector3(position);
    cout << "position=[" << str(params->position) << "]" << endl;
    MessageView::instance()->putln(boost::format(_("      position: %s")) % str(params->position));

    const Listing* normalLine = m->findListing("normalLine");
    params->normalLine = listToVector3(normalLine);
    cout << "normalLine=[" << str(params->normalLine) << "]" << endl;
    MessageView::instance()->putln(boost::format(_("    normalLine: %s")) % str(params->normalLine));

    if (!m->find("maxPullForce")->isValid()) {
	params->pullForceUnlimited = true;
	MessageView::instance()->putln("  maxPullForce: Unlimited");
    } else {
	if (m->read("maxPullForce", maxPullForce)) {
	    params->maxPullForce = maxPullForce;
	    MessageView::instance()->putln(boost::format(_("  maxPullForce: %f")) % maxPullForce);
	} else {
	    // todo
	    MessageView::instance()->putln(" maxPullForce is invalid.");
	}
    }

    if (!m->find("maxShearForce")->isValid()) {
	params->shearForceUnlimited = true;
	MessageView::instance()->putln("  maxShearForce: Unlimited");
    } else {
	if (m->read("maxShearForce", maxShearForce)) {
	    params->maxShearForce = maxShearForce;
	    MessageView::instance()->putln(boost::format(_(" maxShearForce: %f")) % maxShearForce);
	} else {
	    // todo
	    MessageView::instance()->putln(" maxShearForce is invalid.");
	}
    }

    if (!m->find("maxPeelTorque")->isValid()) {
	params->peelTorqueUnlimited = true;
	MessageView::instance()->putln("  maxPeelTorque: Unlimited");
    } else {
	if (m->read("maxPeelTorque", maxPeelTorque)) {
	    params->maxPeelTorque = maxPeelTorque;
	    MessageView::instance()->putln(boost::format(_(" maxPeelTorque: %f")) % maxPeelTorque);
	} else {
	    // todo
	    MessageView::instance()->putln(" maxPeelTorque is invalid.");
	}
    }

    return params;
}

/*
 */
VacuumGripperParams::VacuumGripperParams()
{
    targetObject = "";
    position << 0, 0, 0;
    normalLine << 0, 0, 0;
    pullForceUnlimited = false;
    maxPullForce = 0;
    shearForceUnlimited = false;
    maxShearForce = 0;
    peelTorqueUnlimited = false;
    maxPeelTorque = 0;
}

/*
 */
VacuumGripperParams::~VacuumGripperParams()
{
    ;
}

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
    on_ = true;
    jointID = 0;
}

void VacuumGripper::copyStateFrom(const VacuumGripper& other)
{
    on_ = other.on_;
}

void VacuumGripper::copyStateFrom(const DeviceState& other)
{
    ;
}

VacuumGripper::VacuumGripper(const VacuumGripper& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
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

void VacuumGripper::setParam(const VacuumGripperParams& param)
{
    targetObject = param.targetObject;
    position = param.position;
    normalLine = param.normalLine;
    pullForceUnlimited = param.pullForceUnlimited;
    maxPullForce = param.maxPullForce;
    shearForceUnlimited = param.shearForceUnlimited;
    maxShearForce = param.maxShearForce;
    peelTorqueUnlimited = param.peelTorqueUnlimited;
    maxPeelTorque = param.maxPeelTorque;
}
