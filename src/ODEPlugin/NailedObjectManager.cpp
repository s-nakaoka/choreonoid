/*!
  @file
  @author 
*/

#include <cnoid/NailedObjectManager>
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

NailedObject::~NailedObject()
{
    ;
}

void NailedObject::setJointID(dJointID jointID)
{
    this->jointID = jointID;
}



NailedObjectManager* NailedObjectManager::getInstance()
{
    static NailedObjectManager instance;
    return &instance;
}

/*
 */
NailedObjectManager::NailedObjectManager()
{
}

/*
 */
NailedObjectManager::~NailedObjectManager()
{
}

void NailedObjectManager::clear()
{
//TODO
}

void NailedObjectManager::addObject(NailedObjectPtr obj)
{
    objectMap.insert(make_pair(obj->getBodyID(), obj));
}

bool NailedObjectManager::find(dBodyID bodyID)
{
    NailedObjectMap::iterator p = objectMap.find(bodyID);
    if (p != objectMap.end()){
	return true;
    } else {
	return false;
    }
}

NailedObjectPtr NailedObjectManager::get(dBodyID bodyID)
{
    return objectMap[bodyID];
}


void NailedObjectManager::setParam(const NailDriverParams& param)
{
    fasteningForceUnlimited = param.fasteningForceUnlimited;
    maxFasteningForce = param.maxFasteningForce;
}
