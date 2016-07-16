/*!
  @file
  @author 
*/

#include <cnoid/VacuumGripper>

#include <cnoid/Device>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>

#include <cnoid/ValueTree>

#include "gettext.h"

#include <cnoid/MessageView>
#include <boost/format.hpp>

#include <iostream>

using namespace cnoid;
using namespace std;

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

    read(*m, "position", params->position);
    cout << "position=[" << str(params->position) << "]" << endl;
    MessageView::instance()->putln(boost::format(_("      position: %s")) % str(params->position));

    read(*m, "normalLine", params->normalLine);
    cout << "normalLine=[" << str(params->normalLine) << "]" << endl;
    MessageView::instance()->putln(boost::format(_("    normalLine: %s")) % str(params->normalLine));

    if (!m->find("maxPullForce")->isValid()) {
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
    maxPullForce = std::numeric_limits<double>::max();
    maxShearForce = std::numeric_limits<double>::max();
    maxPeelTorque = std::numeric_limits<double>::max();
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

void VacuumGripper::on(bool on) {
    MessageView::instance()->putln(boost::format(_("*** VacuumGripper: %s ***")) % (on ? "ON" : "OFF"));
    on_ = on;
}

void VacuumGripper::setParam(const VacuumGripperParams& param)
{
    targetObject = param.targetObject;
    position = param.position;
    normalLine = param.normalLine;
    maxPullForce = param.maxPullForce;
    maxShearForce = param.maxShearForce;
    maxPeelTorque = param.maxPeelTorque;
}
