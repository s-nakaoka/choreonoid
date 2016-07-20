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

std::vector<VacuumGripper *> cnoid::createVacuumGrippers(Body* body)
{
    std::vector<VacuumGripper *> vacuumGrippers;
    const Listing *l = body->info()->findListing("vacuumGrippers");

    if (!l->isValid()) {
	MessageView::instance()->putln(boost::format(_("%s doesn't have vacuum gripper")) % body->name());
	return vacuumGrippers;
    }

    MessageView::instance()->putln(boost::format(_("%s has %d vacuum gripper(s)")) % body->name() % l->size());

    for (int i=0; i<l->size(); i++){
        const Mapping* m = l->at(i)->toMapping();

	VacuumGripper* vacuumGripper = new VacuumGripper();

	std::string targetObject = "";
	double maxPullForce = 0;
	double maxShearForce = 0;
	double maxPeelTorque = 0;

	targetObject = m->get("targetObject", targetObject);
	MessageView::instance()->putln(boost::format(_("  targetObject: %s")) % targetObject);
	vacuumGripper->setLink(body->link(targetObject));

	read(*m, "position", vacuumGripper->position);
	cout << "position=[" << str(vacuumGripper->position) << "]" << endl;
	MessageView::instance()->putln(boost::format(_("      position: %s")) % str(vacuumGripper->position));

	read(*m, "normalLine", vacuumGripper->normalLine);
	cout << "normalLine=[" << str(vacuumGripper->normalLine) << "]" << endl;
	MessageView::instance()->putln(boost::format(_("    normalLine: %s")) % str(vacuumGripper->normalLine));

	if (!m->find("maxPullForce")->isValid()) {
	    MessageView::instance()->putln("  maxPullForce: Unlimited");
	} else {
	    if (m->read("maxPullForce", maxPullForce)) {
	        vacuumGripper->maxPullForce = maxPullForce;
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
	        vacuumGripper->maxShearForce = maxShearForce;
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
	        vacuumGripper->maxPeelTorque = maxPeelTorque;
		MessageView::instance()->putln(boost::format(_(" maxPeelTorque: %f")) % maxPeelTorque);
	    } else {
	        // todo
	        MessageView::instance()->putln(" maxPeelTorque is invalid.");
	    }
	}

	vacuumGrippers.push_back(vacuumGripper);
	body->addDevice(vacuumGripper);
    }

    return vacuumGrippers;
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
    on_ = false;
    jointID = 0;
    position << 0, 0, 0;
    normalLine << 0, 0, 0;
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
    MessageView::instance()->putln(boost::format(_("*** %s: %s ***")) % typeName() % (on ? "ON" : "OFF"));
    cout << boost::format(_("*** %s: %s ***")) % typeName() % (on ? "ON" : "OFF") << endl;
    on_ = on;
}
