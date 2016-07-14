/*!
  @file
  @author 
*/

#include <cnoid/NailDriver>

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
 * Get NailDriver parameter.
 */
NailDriverParams* NailDriverParams::findParameter(const Body* body)
{
    Mapping* m = body->info()->findMapping("nailDriver");

    if (!m->isValid()) {
	MessageView::instance()->putln(boost::format(_("%s not has nail driver")) % body->name());
	return 0;
    }

    MessageView::instance()->putln(boost::format(_("%s has nail driver")) % body->name());

    NailDriverParams* params = new NailDriverParams();

    std::string targetObject = "";
    double maxFasteningPower = 0;

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

    if (!m->find("maxFasteningPower")->isValid()) {
	params->fasteningPowerUnlimited = true;
	MessageView::instance()->putln("  maxFasteningPower: Unlimited");
    } else {
	if (m->read("maxFasteningPower", maxFasteningPower)) {
	    params->maxFasteningPower = maxFasteningPower;
	    MessageView::instance()->putln(boost::format(_("  maxFasteningPower: %f")) % maxFasteningPower);
	} else {
	    // todo
	    MessageView::instance()->putln(" maxFasteningPower is invalid.");
	}
    }

    return params;
}

/*
 */
NailDriverParams::NailDriverParams()
{
    targetObject = "";
    position << 0, 0, 0;
    normalLine << 0, 0, 0;
    fasteningPowerUnlimited = false;
    maxFasteningPower = 0;
}

/*
 */
NailDriverParams::~NailDriverParams()
{
    ;
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
    on_ = true;
    jointID = 0;
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

void NailDriver::setParam(const NailDriverParams& param)
{
    targetObject = param.targetObject;
    position = param.position;
    normalLine = param.normalLine;
    fasteningPowerUnlimited = param.fasteningPowerUnlimited;
    maxFasteningPower = param.maxFasteningPower;
}
