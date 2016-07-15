/*!
  @file
*/

#ifndef CNOID_ODEPLUGIN_NAILEDOBJECTMANAGER_H
#define CNOID_ODEPLUGIN_NAILEDOBJECTMANAGER_H

#include <cnoid/NailDriver>

#include <cnoid/Referenced>
#include <cnoid/EigenTypes>
#include <cnoid/Body>
#include <cnoid/Link>
#include <string>
#include <map>
#include "exportdecl.h"

#include <ode/ode.h>

namespace cnoid {

class CNOID_EXPORT NailedObject : public Referenced
{
public:
    NailedObject(dBodyID objID) {
	    numNail = 0;
	    objId_ = objID;
    }
    ~NailedObject();

    void setJointID(dJointID jointID);

    void addNail() {
	    numNail++;
    }

    dBodyID getBodyID() { return objId_; }

    dJointFeedback fb;

private:
    int numNail;
    dBodyID objId_;
    dJointID jointID;
};

typedef ref_ptr<NailedObject> NailedObjectPtr;

typedef std::map<dBodyID, NailedObjectPtr> NailedObjectMap;

class CNOID_EXPORT NailedObjectManager
{
public:
    static NailedObjectManager* getInstance();

private:
    NailedObjectManager();
    ~NailedObjectManager();

public:
    void setParam(const NailDriverParams& param);

    void addObject(NailedObjectPtr obj);

    bool find(dBodyID bodyID);

    NailedObjectPtr get(dBodyID bodyID);

    void clear();

private:
    //std::map<dBodyID, NailedObjectPtr> objectMap;
    NailedObjectMap objectMap;

public:
    bool fasteningForceUnlimited;
    double maxFasteningForce;
};

}

#endif
