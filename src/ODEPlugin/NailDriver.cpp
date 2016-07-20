/*!
  @file
  @author 
*/

#include <cnoid/NailDriver>

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

std::vector<NailDriver*> cnoid::createNailDrivers(Body* body)
{
    std::vector<NailDriver *> nailDrivers;
    const Listing *l = body->info()->findListing("nailDrivers");

    if (!l->isValid()) {
	MessageView::instance()->putln(boost::format(_("%s doesn't have nail driver")) % body->name());
	return nailDrivers;
    }

    MessageView::instance()->putln(boost::format(_("%s has %d nail driver(s)")) % body->name() % l->size());

    for (unsigned int i=0; i<l->size(); i++){
        const Mapping *m = l->at(i)->toMapping();

	NailDriver* nailDriver = new NailDriver();

	std::string targetObject = "";
	double maxFasteningForce = 0;

	targetObject = m->get("targetObject", targetObject);
	MessageView::instance()->putln(boost::format(_("  targetObject: %s")) % targetObject);
	nailDriver->setLink(body->link(targetObject));

	read(*m, "position", nailDriver->position);
	cout << "position=[" << str(nailDriver->position) << "]" << endl;
	MessageView::instance()->putln(boost::format(_("      position: %s")) % str(nailDriver->position));

	read(*m, "normalLine", nailDriver->normalLine);
	cout << "normalLine=[" << str(nailDriver->normalLine) << "]" << endl;
	MessageView::instance()->putln(boost::format(_("    normalLine: %s")) % str(nailDriver->normalLine));

	if (!m->find("maxFasteningForce")->isValid()) {
	    MessageView::instance()->putln("  maxFasteningForce: Unlimited");
	} else {
	    if (m->read("maxFasteningForce", maxFasteningForce)) {
	        nailDriver->maxFasteningForce = maxFasteningForce;
		MessageView::instance()->putln(boost::format(_("  maxFasteningForce: %f")) % maxFasteningForce);
	    } else {
	        // todo
	        MessageView::instance()->putln(" maxFasteningForce is invalid.");
	    }
	}

	nailDrivers.push_back(nailDriver);
	body->addDevice(nailDriver);
    }

    return nailDrivers;
}

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
    position << 0, 0, 0;
    normalLine << 0, 0, 0;
    maxFasteningForce = std::numeric_limits<double>::max();

    resetLatestContact();
}

void NailDriver::copyStateFrom(const NailDriver& other)
{
    on_ = other.on_;
}

void NailDriver::copyStateFrom(const DeviceState& other)
{
    ;
}

NailDriver::NailDriver(const NailDriver& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
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
        resetLatestContact();
    }
    on_ = on;
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

	float isParallel = (link_->R() * normalLine).dot(v);
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
