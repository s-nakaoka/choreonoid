#include "AGXSimulatorItemImpl.h"
#include "AGXSimulatorItem.h"
#include <assert.h>

using namespace std;

namespace cnoid {

AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItemPtr self) : self(self)
{
    initialize();
}
AGXSimulatorItemImpl::AGXSimulatorItemImpl(AGXSimulatorItemPtr self, const AGXSimulatorItemImpl& org) 
    : AGXSimulatorItemImpl(self) 
{
    initialize();    
}
AGXSimulatorItemImpl::~AGXSimulatorItemImpl(){}

void AGXSimulatorItemImpl::initialize()
{
    // Write defalut parameter here
    //agxScene = nullptr;
}

void AGXSimulatorItemImpl::doPutProperties(PutPropertyFunction & putProperty)
{
    //putProperty(_("Step mode"), stepMode, changeProperty(stepMode));
    //putProperty(("Step mode"), "hoge");
}

bool AGXSimulatorItemImpl::store(Archive & archive)
{
    // Write agx parameter to save
    //archive.write("friction", friction);
    return false;
}

bool AGXSimulatorItemImpl::restore(const Archive & archive)
{
    // Write agx parameter to restore
    return false;
}

SimulationBody * AGXSimulatorItemImpl::createSimulationBody(Body * orgBody)
{
    // When user click start bottom, this function will be called first.
    return new AGXBody(*orgBody);
}

#define AGX_SCENE
bool AGXSimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    cout << "initializeSimulation" << endl;
    if(agxScene) agxScene->clearAGXScene();
    agxScene = AGXScene::create();
    agxScene->clearAGXScene();
    AGXSimulationDesc sd;
    sd.timeStep = self->worldTimeStep();
    agxScene->createAGXSimulation(sd);

    // temporary code. will read material from choreonoid
    // Create AGX material
    AGXMaterialDesc m_def;
    agxScene->createAGXMaterial(m_def);
    // Create AGX contact material
    AGXContactMaterialDesc cm_def;
    cm_def.nameA = m_def.name;
    cm_def.nameB = m_def.name;
    cm_def.friction = 1.0;
    cm_def.frictionModelType = AGXFrictionModelType::DEFAULT;
    cm_def.solveType = agx::FrictionModel::SolveType::DIRECT_AND_ITERATIVE;
    agxScene->createAGXContactMaterial(cm_def);
    // end temporary

    for(size_t i=0; i < simBodies.size(); ++i){
        // Create rigidbody, geometry, constraints
        AGXBody* body = static_cast<AGXBody*>(simBodies[i]);
        body->createBody();
        for(int j = 0; j < body->getNumLinks(); ++j){
            // Add AGXRigidbody and constraint to AGX simulation
            agxScene->getAGXSimulation()->add(body->getAGXRigidBody(j));
            agxScene->getAGXSimulation()->add(body->getAGXConstraint(j));
            // Set Material
            body->setAGXMaterial(j, agxScene->getAGXMaterial(m_def.name));   // will replace m_def.name to choreonoid material name
        }
        // Set self collision
        if(!body->bodyItem()->isSelfCollisionDetectionEnabled()){
            agxScene->setCollisionPair(body->bodyItem()->name(), body->bodyItem()->name(), false); 
        }
        // Set external collision
        if(!body->bodyItem()->isCollisionDetectionEnabled()){
            body->setCollision(false);
        }
    }

    saveSimulationToAGXFile();

    return true;
}

bool AGXSimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
//    cout << "step" << std::endl;
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        AGXBody* agxBody = static_cast<AGXBody*>(activeSimBodies[i]);
        agxBody->setTorqueToAGX();

        //if(!agxBody->sensorHelper.forceSensors().empty()){
        //    agxBody->updateForceSensors();
        //}
        //if(agxBody->sensorHelper.hasGyroOrAccelerationSensors()){
        //    agxBody->sensorHelper.updateGyroAndAccelerationSensors();
        //}
    }

    agxScene->stepAGXSimulation();

    for(size_t i=0; i < activeSimBodies.size(); ++i){
        AGXBody* agxBody = static_cast<AGXBody*>(activeSimBodies[i]);
        agxBody->setLinkStateToCnoid();

        //if(!agxBody->sensorHelper.forceSensors().empty()){
        //    agxBody->updateForceSensors();
        //}
        //if(agxBody->sensorHelper.hasGyroOrAccelerationSensors()){
        //    agxBody->sensorHelper.updateGyroAndAccelerationSensors();
        //}
    }

    return true;
}

void AGXSimulatorItemImpl::stopSimulation()
{
    cout << "stopSimulation" << endl;
}

void AGXSimulatorItemImpl::pauseSimulation()
{
    cout << "pauseSimulation" << endl;
}

void AGXSimulatorItemImpl::restartSimulation()
{
    cout << "restartSimulation" << endl;
}

bool AGXSimulatorItemImpl::saveSimulationToAGXFile()
{
    return agxScene->saveSceneToAGXFile();
}

}