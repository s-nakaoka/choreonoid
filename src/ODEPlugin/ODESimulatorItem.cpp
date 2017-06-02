/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "ODESimulatorItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/MeshExtractor>
#include <cnoid/SceneDrawables>
#include <cnoid/FloatingNumberString>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/Sensor>
#include <cnoid/BasicSensorSimulationHelper>
#include <cnoid/BodyItem>
#include <cnoid/BodyCollisionDetectorUtil>
#include <QElapsedTimer>
#include "gettext.h"
#include <cnoid/VacuumGripper>
#include <cnoid/NailDriver>
#include "NailedObjectManager.h"

#ifdef GAZEBO_ODE
#include <gazebo/ode/ode.h>
#define ITEM_NAME N_("GazeboODESimulatorItem")
#else
#include <ode/ode.h>
#define ITEM_NAME N_("ODESimulatorItem")
#endif
#include <iostream>

#include <cnoid/MessageView>
#include <boost/format.hpp>

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;
const bool USE_AMOTOR = false;

const bool MEASURE_PHYSICS_CALCULATION_TIME = true;

const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;

typedef Eigen::Matrix<float, 3, 1> Vertex;

struct Triangle {
    int indices[3];
};

dMatrix3 identity = {
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0 };

dMatrix3 flippedIdentity = {
    1.0,  0.0, 0.0, 0.0,
    0.0,  0.0, 1.0, 0.0,
    0.0, -1.0, 0.0, 0.0 };
    

inline void makeInternal(Vector3& v) {
    double a = v.z();
    v.z() = -v.y();
    v.y() = a;
}
inline void toInternal(const Vector3& v, Vector3& out_v) {
    out_v << v.x(), v.z(), -v.y();
}

class ODEBody;

class ODELink : public Referenced
{
public:
    Link* link;
    dBodyID bodyID;
    dJointID jointID;
    vector<dGeomID> geomID;
    dTriMeshDataID triMeshDataID;
    vector<Vertex> vertices;
    vector<Triangle> triangles;
    typedef map< dGeomID, Position, std::less<dGeomID>, 
                 Eigen::aligned_allocator< pair<const dGeomID, Position> > > OffsetMap;
    OffsetMap offsetMap;
    dJointID motorID;

    ODELink(ODESimulatorItemImpl* simImpl, ODEBody* odeBody, ODELink* parent,
            const Vector3& parentOrigin, Link* link);
    ~ODELink();
    void createLinkBody(ODESimulatorItemImpl* simImpl, dWorldID worldID, ODELink* parent, const Vector3& origin);
    void createGeometry(ODEBody* odeBody);
    void setKinematicStateToODE();
    void setKinematicStateToODEflip();
    void setTorqueToODE();
    void setVelocityToODE();
    void getKinematicStateFromODE();
    void getKinematicStateFromODEflip();
    void addMesh(MeshExtractor* extractor, ODEBody* odeBody);
};
typedef ref_ptr<ODELink> ODELinkPtr;


class ODEBody : public SimulationBody
{
public:
    vector<ODELinkPtr> odeLinks;
    dWorldID worldID;
    dSpaceID spaceID;
    vector<dJointFeedback> forceSensorFeedbacks;
    BasicSensorSimulationHelper sensorHelper;
    int geometryId;
        
    ODEBody(const Body& orgBody);
    ~ODEBody();
    void createBody(ODESimulatorItemImpl* simImpl);
    void setExtraJoints(bool flipYZ);
    void setKinematicStateToODE(bool flipYZ);
    void setTorqueToODE();
    void setVelocityToODE();
    void getKinematicStateFromODE(bool flipYZ);
    void updateForceSensors(bool flipYZ);
    void alignToZAxisIn2Dmode();
};
}


namespace cnoid {
  
typedef std::map<dBodyID, Link*> CrawlerLinkMap;
typedef std::map<dBodyID, VacuumGripper*> VacuumGripperMap;
typedef std::map<dBodyID, NailDriver*> NailDriverMap;

#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
typedef std::map<dBodyID, double> MecanumWheelSettingMap;
#endif                      /* MECANUM_WHEEL_ODE */

class ODESimulatorItemImpl
{
public:
    ODESimulatorItem* self;

    bool flipYZ;
        
    dWorldID worldID;
    dSpaceID spaceID;
    dJointGroupID contactJointGroupID;
    double timeStep;
    CrawlerLinkMap crawlerLinks;
    VacuumGripperMap vacuumGripperDevs;
    NailDriverMap nailDriverDevs;
    //NailedObjectManager nailedObjectMngr;
#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
    MecanumWheelSettingMap mecanumWheelSetting;
    // XXX: this variable will erase later.
    bool                   mecanumWheelDebug;
#endif                      /* MECANUM_WHEEL_ODE */
    vector<ODELink*> geometryIdToLink;

    Selection stepMode;
    Vector3 gravity;
    double friction;
    bool isJointLimitMode;
    bool is2Dmode;
    double globalERP;
    FloatingNumberString globalCFM;
    int numIterations;
    double overRelaxation;
    bool enableMaxCorrectingVel;
    FloatingNumberString maxCorrectingVel;
    double surfaceLayerDepth;
    bool useWorldCollision;
    CollisionDetectorPtr collisionDetector;
    bool velocityMode;

    double physicsTime;
    QElapsedTimer physicsTimer;
    double collisionTime;
    QElapsedTimer collisionTimer;

    double vacuumGripperLimitCheckStartTime;
    double nailDriverLimitCheckStartTime;

    ODESimulatorItemImpl(ODESimulatorItem* self);
    ODESimulatorItemImpl(ODESimulatorItem* self, const ODESimulatorItemImpl& org);
    void initialize();
    ~ODESimulatorItemImpl();
    void clear();
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    void addBody(ODEBody* odeBody);
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void doPutProperties(PutPropertyFunction& putProperty);
    void store(Archive& archive);
    void restore(const Archive& archive);
    void collisionCallback(const CollisionPair& collisionPair);
    VacuumGripper *isVacuumGripper(dBodyID body);
    NailDriver *isNailDriver(dBodyID body);

    void nailDriverCheck();
    void nailedObjectLimitCheck();

#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
    void preserveMecanumWheelSetting(ODEBody* odeBody);
#endif                      /* MECANUM_WHEEL_ODE */
};
}


ODELink::ODELink
(ODESimulatorItemImpl* simImpl, ODEBody* odeBody, ODELink* parent, const Vector3& parentOrigin, Link* link)
{
    odeBody->odeLinks.push_back(this);

    this->link = link;
    bodyID = 0;
    jointID = 0;
    triMeshDataID = 0;
    geomID.clear();
    motorID = 0;
    
    Vector3 o = parentOrigin + link->b();
    
    if(odeBody->worldID){
        createLinkBody(simImpl, odeBody->worldID, parent, o);
    }
    if(!simImpl->useWorldCollision){
        createGeometry(odeBody);
    }

    for(Link* child = link->child(); child; child = child->sibling()){
        new ODELink(simImpl, odeBody, this, o, child);
    }

}


void ODELink::createLinkBody(ODESimulatorItemImpl* simImpl, dWorldID worldID, ODELink* parent, const Vector3& origin)
{
    bodyID = dBodyCreate(worldID);
    dBodySetData(bodyID, link);

    dMass mass;
    dMassSetZero(&mass);
    const Matrix3& I = link->I();
#if 0
    Vector3 axis = link->a();
    Matrix3 I0 = I + axis * axis.transpose() * link->Jm2();
    dMassSetParameters(&mass, link->m(),
            0.0, 0.0, 0.0,
            I0(0,0), I0(1,1), I0(2,2),
            I0(0,1), I0(0,2), I0(1,2));
#else
    dMassSetParameters(&mass, link->m(),
                       0.0, 0.0, 0.0,
                       I(0,0), I(1,1), I(2,2),
                       I(0,1), I(0,2), I(1,2));
#endif

    dBodySetMass(bodyID, &mass);

    Vector3 c;
    Vector3 o;
    Vector3 a;
    Vector3 d;

    if(!simImpl->flipYZ){
        c = link->c();
        o = origin;
        a = link->a();
        d = link->d();
        dBodySetRotation(bodyID, identity);
    } else {
        toInternal(link->c(), c);
        toInternal(origin, o);
        toInternal(link->a(), a);
        toInternal(link->d(), d);
        dBodySetRotation(bodyID, flippedIdentity);
    }
        
    // set the default global position to set a joint
    Vector3 p = o + c;
    dBodySetPosition(bodyID, p.x(), p.y(), p.z());

    dBodyID parentBodyID = parent ? parent->bodyID : 0;

cout << boost::format("      link: %s jointType: %s (%d)")% link->name()  % link->jointTypeString() % link->jointType() << endl;
    switch(link->jointType()){
        
    case Link::ROTATIONAL_JOINT:
        jointID = dJointCreateHinge(worldID, 0);
        dJointAttach(jointID, bodyID, parentBodyID);
        dJointSetHingeAnchor(jointID, o.x(), o.y(), o.z());
        dJointSetHingeAxis(jointID, a.x(), a.y(), a.z());
        if(simImpl->isJointLimitMode){
            if(link->q_upper() < numeric_limits<double>::max()){
                dJointSetHingeParam(jointID, dParamHiStop, link->q_upper());
            }
            if(link->q_lower() > -numeric_limits<double>::max()){
                dJointSetHingeParam(jointID, dParamLoStop, link->q_lower());
            }
        }
        if(simImpl->velocityMode){
        	if(!USE_AMOTOR){
#ifdef GAZEBO_ODE
				dJointSetHingeParam(jointID, dParamFMax, 100);   //???
				dJointSetHingeParam(jointID, dParamFudgeFactor, 1);
#else
				dJointSetHingeParam(jointID, dParamFMax, numeric_limits<dReal>::max());
				dJointSetHingeParam(jointID, dParamFudgeFactor, 1);
#endif
        	}else{
				motorID = dJointCreateAMotor(worldID, 0);
				dJointAttach(motorID, bodyID, parentBodyID);
				dJointSetAMotorMode(motorID, dAMotorUser);
				dJointSetAMotorNumAxes(motorID, 1);
				dJointSetAMotorAxis(motorID, 0, 2, a.x(), a.y(), a.z());
#ifdef GAZEBO_ODE
				dJointSetAMotorParam(motorID, dParamFMax, 100);
#else
				dJointSetAMotorParam(motorID, dParamFMax, numeric_limits<dReal>::max() );
#endif
				dJointSetAMotorParam(motorID, dParamFudgeFactor, 1);
        	}
        }
        break;
        
    case Link::SLIDE_JOINT:
        jointID = dJointCreateSlider(worldID, 0);
        dJointAttach(jointID, bodyID, parentBodyID);
        dJointSetSliderAxis(jointID, d.x(), d.y(), d.z());
        if(simImpl->isJointLimitMode){
            if(link->q_upper() < numeric_limits<double>::max()){
                dJointSetSliderParam(jointID, dParamHiStop, link->q_upper());
            }
            if(link->q_lower() > -numeric_limits<double>::max()){
                dJointSetSliderParam(jointID, dParamLoStop, link->q_lower());
            }
        }
        if(simImpl->velocityMode){
        	dJointSetSliderParam(jointID, dParamFMax, numeric_limits<dReal>::max() );
  			dJointSetSliderParam(jointID, dParamFudgeFactor, 1 );
        }
        break;

    case Link::FREE_JOINT:
        break;

    case Link::FIXED_JOINT:
    default:
#ifdef GAZEBO_ODE
    	jointID = dJointCreateFixed(worldID, 0);
        dJointAttach(jointID, bodyID, parentBodyID);
    	dJointSetFixed(jointID);
    	if(link->jointType() == Link::PSEUDO_CONTINUOUS_TRACK || link->jointType() == Link::CRAWLER_JOINT){
    	    simImpl->crawlerLinks.insert(make_pair(bodyID, link));
    	}
#else
        if(parentBodyID){
            jointID = dJointCreateFixed(worldID, 0);
            dJointAttach(jointID, bodyID, parentBodyID);
            dJointSetFixed(jointID);
            if(link->jointType() == Link::PSEUDO_CONTINUOUS_TRACK || link->jointType() == Link::CRAWLER_JOINT){
                simImpl->crawlerLinks.insert(make_pair(bodyID, link));
            }
        } else {
            dBodySetKinematic(bodyID);
        }
#endif
        break;
    }
}


void ODELink::createGeometry(ODEBody* odeBody)
{
    if(link->collisionShape()){
        MeshExtractor* extractor = new MeshExtractor;
        if(extractor->extract(link->collisionShape(), std::bind(&ODELink::addMesh, this, extractor, odeBody))){
            if(!vertices.empty()){
                triMeshDataID = dGeomTriMeshDataCreate();
                dGeomTriMeshDataBuildSingle(triMeshDataID,
                                            &vertices[0], sizeof(Vertex), vertices.size(),
                                            &triangles[0],triangles.size() * 3, sizeof(Triangle));
            
                dGeomID gId = dCreateTriMesh(odeBody->spaceID, triMeshDataID, 0, 0, 0);
                geomID.push_back(gId);
                dGeomSetBody(gId, bodyID);
            }
        }
        delete extractor;
    }
}


void ODELink::addMesh(MeshExtractor* extractor, ODEBody* odeBody)
{
    SgMesh* mesh = extractor->currentMesh();
    const Affine3& T = extractor->currentTransform();
    
    bool meshAdded = false;
    
    if(mesh->primitiveType() != SgMesh::MESH){
        bool doAddPrimitive = false;
        Vector3 scale;
        boost::optional<Vector3> translation;
        if(!extractor->isCurrentScaled()){
            scale.setOnes();
            doAddPrimitive = true;
        } else {
            Affine3 S = extractor->currentTransformWithoutScaling().inverse() *
                extractor->currentTransform();

            if(S.linear().isDiagonal()){
                if(!S.translation().isZero()){
                    translation = S.translation();
                }
                scale = S.linear().diagonal();
                if(mesh->primitiveType() == SgMesh::BOX){
                    doAddPrimitive = true;
                } else if(mesh->primitiveType() == SgMesh::SPHERE){
                    // check if the sphere is uniformly scaled for all the axes
                    if(scale.x() == scale.y() && scale.x() == scale.z()){
                        doAddPrimitive = true;
                    }
                } else if(mesh->primitiveType() == SgMesh::CYLINDER){
                    // check if the bottom circle face is uniformly scaled
                    if(scale.x() == scale.z()){
                        doAddPrimitive = true;
                    }
                }
            }
        }
        if(doAddPrimitive){
            bool created = false;
            dGeomID geomId;
            switch(mesh->primitiveType()){
            case SgMesh::BOX : {
                const Vector3& s = mesh->primitive<SgMesh::Box>().size;
                geomId = dCreateBox(odeBody->spaceID, s.x() * scale.x(), s.y() * scale.y(), s.z() * scale.z());
                created = true;
                break; }
            case SgMesh::SPHERE : {
                SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                geomId = dCreateSphere(odeBody->spaceID, sphere.radius * scale.x());
                created = true;
                break; }
            case SgMesh::CYLINDER : {
                SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                geomId = dCreateCylinder(odeBody->spaceID, cylinder.radius * scale.x(), cylinder.height * scale.y());
                created = true;
                break; }
            default :
                break;
            }
            if(created){
                geomID.push_back(geomId);
                dGeomSetBody(geomId, bodyID);
                Affine3 T_ = extractor->currentTransformWithoutScaling();
                if(translation){
                    T_ *= Translation3(*translation);
                }
                if(mesh->primitiveType()==SgMesh::CYLINDER)
                    T_ *= AngleAxis(radian(90), Vector3::UnitX());
                Vector3 p = T_.translation()-link->c();
                dMatrix3 R = { T_(0,0), T_(0,1), T_(0,2), 0.0,
                               T_(1,0), T_(1,1), T_(1,2), 0.0,
                               T_(2,0), T_(2,1), T_(2,2), 0.0 };
                if(bodyID){
                    dGeomSetOffsetPosition(geomId, p.x(), p.y(), p.z());
                    dGeomSetOffsetRotation(geomId, R);
                }else{
                    offsetMap.insert(OffsetMap::value_type(geomId,T_));
                }
                meshAdded = true;
            }
        }
    }

    if(!meshAdded){
        const int vertexIndexTop = vertices.size();

        const SgVertexArray& vertices_ = *mesh->vertices();
        const int numVertices = vertices_.size();
        for(int i=0; i < numVertices; ++i){
            const Vector3 v = T * vertices_[i].cast<Position::Scalar>() - link->c();
            vertices.push_back(Vertex(v.x(), v.y(), v.z()));
        }

        const int numTriangles = mesh->numTriangles();
        for(int i=0; i < numTriangles; ++i){
            SgMesh::TriangleRef src = mesh->triangle(i);
            Triangle tri;
            tri.indices[0] = vertexIndexTop + src[0];
            tri.indices[1] = vertexIndexTop + src[1];
            tri.indices[2] = vertexIndexTop + src[2];
            triangles.push_back(tri);
        }
    }
}


ODELink::~ODELink()
{
    for(vector<dGeomID>::iterator it=geomID.begin(); it!=geomID.end(); it++)
        dGeomDestroy(*it);
    if(triMeshDataID){
        dGeomTriMeshDataDestroy(triMeshDataID);
    }
}


void ODELink::setKinematicStateToODE()
{
    const Position& T = link->T();
    if(bodyID){
        dMatrix3 R2 = { T(0,0), T(0,1), T(0,2), 0.0,
                        T(1,0), T(1,1), T(1,2), 0.0,
                        T(2,0), T(2,1), T(2,2), 0.0 };
    
        dBodySetRotation(bodyID, R2);
        const Vector3 lc = link->R() * link->c();
        const Vector3 c = link->p() + lc;
        dBodySetPosition(bodyID, c.x(), c.y(), c.z());
        const Vector3& w = link->w();
        const Vector3 v = link->v() + w.cross(lc);
        dBodySetLinearVel(bodyID, v.x(), v.y(), v.z());
        dBodySetAngularVel(bodyID, w.x(), w.y(), w.z());

    }else{
        for(vector<dGeomID>::iterator it = geomID.begin(); it!=geomID.end(); it++){
            OffsetMap::iterator it0 = offsetMap.find(*it);
            Position offset(Position::Identity());
            if(it0!=offsetMap.end())
                offset = it0->second;
            Position T_ = T*offset;
            Vector3 p = T_.translation() + link->c();
            dMatrix3 R2 = { T_(0,0), T_(0,1), T_(0,2), 0.0,
                            T_(1,0), T_(1,1), T_(1,2), 0.0,
                            T_(2,0), T_(2,1), T_(2,2), 0.0 };

            dGeomSetPosition(*it, p.x(), p.y(), p.z());
            dGeomSetRotation(*it, R2);
        }
    }
}


void ODELink::setKinematicStateToODEflip()
{
    const Position& T = link->T();
    dMatrix3 R2 = {  T(0,0),  T(0,1),  T(0,2), 0.0,
                     T(2,0),  T(2,1),  T(2,2), 0.0,
                     -T(1,0), -T(1,1), -T(1,2), 0.0 };
    if(bodyID){
        dBodySetRotation(bodyID, R2);
        const Vector3 lc = link->R() * link->c();
        const Vector3 c = link->p() + lc;
        dBodySetPosition(bodyID, c.x(), c.z(), -c.y());
        const Vector3& w = link->w();
        const Vector3 v = link->v() + w.cross(lc);
        dBodySetLinearVel(bodyID, v.x(), v.z(), -v.y());
        dBodySetAngularVel(bodyID, w.x(), w.z(), -w.y());

    }else{
        const Vector3 c = link->p() + link->R() * link->c();
        for(vector<dGeomID>::iterator it = geomID.begin(); it!=geomID.end(); it++){
            dGeomSetPosition(*it, c.x(), c.y(), -c.z());
            dGeomSetRotation(*it, R2);
        }
    }
}


/**
   \note This method must not be called for a static body.
*/
void ODELink::getKinematicStateFromODE()
{
    if(jointID){
        if(link->isRotationalJoint()){
            link->q() = dJointGetHingeAngle(jointID);
            link->dq() = dJointGetHingeAngleRate(jointID);
        } else if(link->isSlideJoint()){
            link->q() = dJointGetSliderPosition(jointID);
            link->dq() = dJointGetSliderPositionRate(jointID);
        }
    }

    const dReal* R = dBodyGetRotation(bodyID);
    link->R() <<
        R[0], R[1], R[2],
        R[4], R[5], R[6],
        R[8], R[9], R[10];

    typedef Eigen::Map<const Eigen::Matrix<dReal, 3, 1> > toVector3;
    const Vector3 c = link->R() * link->c();
    link->p() = toVector3(dBodyGetPosition(bodyID)) - c;
    link->w() = toVector3(dBodyGetAngularVel(bodyID));
    link->v() = toVector3(dBodyGetLinearVel(bodyID)) - link->w().cross(c);
}


/**
   \note This method must not be called for a static body.
*/
void ODELink::getKinematicStateFromODEflip()
{
    if(jointID){
        if(link->isRotationalJoint()){
            link->q() = dJointGetHingeAngle(jointID);
            link->dq() = dJointGetHingeAngleRate(jointID);
        } else if(link->isSlideJoint()){
            link->q() = dJointGetSliderPosition(jointID);
            link->dq() = dJointGetSliderPositionRate(jointID);
        }
    }

    const dReal* R = dBodyGetRotation(bodyID);
    link->R() <<
        R[0],  R[1],  R[2],
        -R[8], -R[9], -R[10],
        R[4],  R[5],  R[6];
    Vector3 c;
    toInternal(link->R() * link->c(), c);
    
    typedef Eigen::Map<const Eigen::Matrix<dReal, 3, 1> > toVector3;
    const Vector3 p = toVector3(dBodyGetPosition(bodyID)) - c;
    toVector3 w(dBodyGetAngularVel(bodyID));
    const Vector3 v = toVector3(dBodyGetLinearVel(bodyID)) - w.cross(c);
    
    link->p() << p.x(), -p.z(), p.y();
    link->w() << w.x(), -w.z(), w.y();
    link->v() << v.x(), -v.z(), v.y();
}


/**
   \note This method must not be called for the root link and a static body.
*/
void ODELink::setTorqueToODE()
{
    if(link->isRotationalJoint()){
        dJointAddHingeTorque(jointID, link->u());
    } else if(link->isSlideJoint()){
        dJointAddSliderForce(jointID, link->u());
    }
}


void ODELink::setVelocityToODE()
{
    if(link->isRotationalJoint()){
    	dReal v = link->dq();
    	if(!USE_AMOTOR)
    		dJointSetHingeParam(jointID, dParamVel, v);
    	else
    		dJointSetAMotorParam(motorID, dParamVel, v);

    } else if(link->isSlideJoint()){
    	dReal v = link->dq();
    	dJointSetSliderParam(jointID, dParamVel, v);
    }
}


ODEBody::ODEBody(const Body& orgBody)
    : SimulationBody(new Body(orgBody))
{
    worldID = 0;
    spaceID = 0;
    geometryId = 0;
}


ODEBody::~ODEBody()
{
    if(spaceID){
        dSpaceDestroy(spaceID);
    }
}


void ODEBody::createBody(ODESimulatorItemImpl* simImpl)
{
    Body* body = this->body();
    
    worldID = body->isStaticModel() ? 0 : simImpl->worldID;
    if(simImpl->useWorldCollision){
        geometryId = addBodyToCollisionDetector(*body, *simImpl->collisionDetector, 
                                                bodyItem()->isSelfCollisionDetectionEnabled());
    }else{
        spaceID = dHashSpaceCreate(simImpl->spaceID);
        dSpaceSetCleanup(spaceID, 0);
    }

    ODELink* rootLink = new ODELink(simImpl, this, 0, Vector3::Zero(), body->rootLink());

    setKinematicStateToODE(simImpl->flipYZ);

    if(simImpl->useWorldCollision){
        int size = simImpl->geometryIdToLink.size();
        int numLinks = odeLinks.size();
        simImpl->geometryIdToLink.resize(geometryId+numLinks);
        for(int i=0; i < numLinks; i++){
            ODELink* odeLink = odeLinks[i].get();
            int index = odeLink->link->index();
            simImpl->geometryIdToLink[geometryId+index] = odeLink;
            simImpl->collisionDetector->updatePosition(geometryId+index, odeLink->link->T());
        }
    }

    setExtraJoints(simImpl->flipYZ);

    if(simImpl->is2Dmode && worldID){
        dJointID planeJointID = dJointCreatePlane2D(worldID, 0);
        dJointAttach(planeJointID, rootLink->bodyID, 0);
    }
    
    setTorqueToODE();

    sensorHelper.initialize(body, simImpl->timeStep, simImpl->gravity);

    // set joint feedbacks for force sensors
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    forceSensorFeedbacks.resize(forceSensors.size());
    for(size_t i=0; i < forceSensors.size(); ++i){
        dJointSetFeedback(odeLinks[forceSensors[i]->link()->index()]->jointID, &forceSensorFeedbacks[i]);
    }
}


void ODEBody::setExtraJoints(bool flipYZ)
{
    Body* body = this->body();
    const int n = body->numExtraJoints();

    for(int j=0; j < n; ++j){

        ExtraJoint& extraJoint = body->extraJoint(j);

        ODELinkPtr odeLinkPair[2];
        for(int i=0; i < 2; ++i){
            ODELinkPtr odeLink;
            Link* link = extraJoint.link[i];
            if(link->index() < odeLinks.size()){
                odeLink = odeLinks[link->index()];
                if(odeLink->link == link){
                    odeLinkPair[i] = odeLink;
                }
            }
            if(!odeLink){
                break;
            }
        }

        if(odeLinkPair[1]){

            dJointID jointID = 0;
            Link* link = odeLinkPair[0]->link;
            Vector3 p = link->attitude() * extraJoint.point[0] + link->p();
            Vector3 a = link->attitude() * extraJoint.axis;
            if(flipYZ){
                makeInternal(p);
                makeInternal(a);
            }

            // \todo do the destroy management for these joints
            if(extraJoint.type == ExtraJoint::EJ_PISTON){
                jointID = dJointCreatePiston(worldID, 0);
                dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                dJointSetPistonAnchor(jointID, p.x(), p.y(), p.z());
                dJointSetPistonAxis(jointID, a.x(), a.y(), a.z());

            } else if(extraJoint.type == ExtraJoint::EJ_BALL){
                jointID = dJointCreateBall(worldID, 0);
                dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                dJointSetBallAnchor(jointID, p.x(), p.y(), p.z());
            }
        }
    }
}


void ODEBody::setKinematicStateToODE(bool flipYZ)
{
    if(!flipYZ){
        for(size_t i=0; i < odeLinks.size(); ++i){
            odeLinks[i]->setKinematicStateToODE();
        }
    } else {
        for(size_t i=0; i < odeLinks.size(); ++i){
            odeLinks[i]->setKinematicStateToODEflip();
        }
    }
}


void ODEBody::setTorqueToODE()
{
    // Skip the root link
    for(size_t i=1; i < odeLinks.size(); ++i){
        odeLinks[i]->setTorqueToODE();
    }
}


void ODEBody::setVelocityToODE()
{
    // Skip the root link
    for(size_t i=1; i < odeLinks.size(); ++i){
        odeLinks[i]->setVelocityToODE();
    }
}


void ODEBody::getKinematicStateFromODE(bool flipYZ)
{
    if(!flipYZ){
        for(size_t i=0; i < odeLinks.size(); ++i){
            odeLinks[i]->getKinematicStateFromODE();
        }
    } else {
        for(size_t i=0; i < odeLinks.size(); ++i){
            odeLinks[i]->getKinematicStateFromODEflip();
        }
    }
}


void ODEBody::updateForceSensors(bool flipYZ)
{
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    for(int i=0; i < forceSensors.size(); ++i){
        ForceSensor* sensor = forceSensors[i];
        const Link* link = sensor->link();
        const dJointFeedback& fb = forceSensorFeedbacks[i];
        Vector3 f, tau;
        if(!flipYZ){
            f   << fb.f2[0], fb.f2[1], fb.f2[2];
            tau << fb.t2[0], fb.t2[1], fb.t2[2];
        } else {
            f   << fb.f2[0], -fb.f2[2], fb.f2[1];
            tau << fb.t2[0], -fb.t2[2], fb.t2[1];
        }
        const Matrix3 R = link->R() * sensor->R_local();
        const Vector3 p = link->R() * sensor->p_local();

        sensor->f()   = R.transpose() * f;
        sensor->tau() = R.transpose() * (tau - p.cross(f));
        sensor->notifyStateChange();
    }
}


void ODEBody::alignToZAxisIn2Dmode()
{
    static const Quat r(AngleAxis(PI / 2.0, Vector3(1.0, 0.0, 0.0)));

    dBodyID& bodyID = odeLinks.front()->bodyID;

    const dReal* q0 = dBodyGetQuaternion(bodyID);
    Quat q(q0[0], q0[1], q0[2], q0[3]);
    Quat q2 = r * q;
    q2.x() = 0.0;
    q2.z() = 0.0;
    q2.normalize();
    Quat q3 = r.inverse() * q2;
    dReal q4[4];
    q4[0] = q3.w();
    q4[1] = q3.x();    
    q4[2] = q3.y();    
    q4[3] = q3.z();    
    dBodySetQuaternion(bodyID, q4);

    const dReal* w = dBodyGetAngularVel(bodyID);
    dBodySetAngularVel(bodyID, 0, 0, w[2]);
}


void ODESimulatorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<ODESimulatorItem>(ITEM_NAME);
    ext->itemManager().addCreationPanel<ODESimulatorItem>();
}


ODESimulatorItem::ODESimulatorItem()
{
    impl = new ODESimulatorItemImpl(this);
}


ODESimulatorItemImpl::ODESimulatorItemImpl(ODESimulatorItem* self)
    : self(self),
      stepMode(ODESimulatorItem::NUM_STEP_MODES, CNOID_GETTEXT_DOMAIN_NAME)

{
    initialize();

    stepMode.setSymbol(ODESimulatorItem::STEP_ITERATIVE,  N_("Iterative (quick step)"));
    stepMode.setSymbol(ODESimulatorItem::STEP_BIG_MATRIX, N_("Big matrix"));
    stepMode.select(ODESimulatorItem::STEP_ITERATIVE);
    
    gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;
    globalERP = 0.4;
    globalCFM = "1.0e-10";
    numIterations = 50;
    overRelaxation = 1.3;
    enableMaxCorrectingVel = true;
    maxCorrectingVel = "1.0e-3";
    surfaceLayerDepth = 0.0001;
    friction = 1.0;
    isJointLimitMode = false;
    is2Dmode = false;
    flipYZ = false;
    useWorldCollision = false;
    velocityMode = false;

#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
    // XXX: this code will erase later.
    mecanumWheelDebug = false;
#endif                      /* MECANUM_WHEEL_ODE */
}


ODESimulatorItem::ODESimulatorItem(const ODESimulatorItem& org)
    : SimulatorItem(org)
{
    impl = new ODESimulatorItemImpl(this, *org.impl);
}


ODESimulatorItemImpl::ODESimulatorItemImpl(ODESimulatorItem* self, const ODESimulatorItemImpl& org)
    : self(self)
{
    initialize();

    stepMode = org.stepMode;
    gravity = org.gravity;
    globalERP = org.globalERP;
    globalCFM = org.globalCFM;
    numIterations = org.numIterations;
    overRelaxation = org.overRelaxation;
    enableMaxCorrectingVel = org.enableMaxCorrectingVel;
    maxCorrectingVel = org.maxCorrectingVel;
    surfaceLayerDepth = org.surfaceLayerDepth;
    friction = org.friction;
    isJointLimitMode = org.isJointLimitMode;
    is2Dmode = org.is2Dmode;
    flipYZ = org.flipYZ;
    useWorldCollision = org.useWorldCollision;
    velocityMode = org.velocityMode;
}


void ODESimulatorItemImpl::initialize()
{
    worldID = 0;
    spaceID = 0;
    contactJointGroupID = dJointGroupCreate(0);
    self->SimulatorItem::setAllLinkPositionOutputMode(true);
}


ODESimulatorItem::~ODESimulatorItem()
{
    delete impl;
}


ODESimulatorItemImpl::~ODESimulatorItemImpl()
{
    clear();

    if(contactJointGroupID){
        dJointGroupDestroy(contactJointGroupID);
    }
}


void ODESimulatorItem::setStepMode(int value)
{
    impl->stepMode.select(value);
}


void ODESimulatorItem::setGravity(const Vector3& gravity)
{
    impl->gravity = gravity;
}


void ODESimulatorItem::setFriction(double friction)
{
    impl->friction = friction;
}


void ODESimulatorItem::setJointLimitMode(bool on)
{
    impl->isJointLimitMode = on;
}


void ODESimulatorItem::set2Dmode(bool on)
{
    impl->is2Dmode = on;
}


void ODESimulatorItem::setGlobalERP(double erp)
{
    impl->globalERP = erp;
}


void ODESimulatorItem::setGlobalCFM(double value)
{
    impl->globalCFM = value;
}


void ODESimulatorItem::setNumIterations(int n)
{
    impl->numIterations = n;
}


void ODESimulatorItem::setOverRelaxation(double value)
{
    impl->overRelaxation = value;
}


void ODESimulatorItem::setCorrectingVelocityLimitMode(bool on)
{
    impl->enableMaxCorrectingVel = on;
}


void ODESimulatorItem::setMaxCorrectingVelocity(double vel)
{
    impl->maxCorrectingVel = vel;
}


void ODESimulatorItem::setSurfaceLayerDepth(double value)
{
    impl->surfaceLayerDepth = value;
}


void ODESimulatorItem::useWorldCollisionDetector(bool on)
{
    impl->useWorldCollision = on;
}

void ODESimulatorItem::useVacuumGripper(bool on)
{
    for (VacuumGripperMap::iterator p = impl->vacuumGripperDevs.begin();
         p != impl->vacuumGripperDevs.end(); p++) {
        VacuumGripper* vacuumGripper = p->second;
        vacuumGripper->on(on);
    }
}

void ODESimulatorItem::setVacuumGripperLimitCheckStartTime(double limitCheckStartTime)
{
    impl->vacuumGripperLimitCheckStartTime = limitCheckStartTime;
}

void ODESimulatorItem::useNailDriver(bool on)
{
    for (NailDriverMap::iterator p = impl->nailDriverDevs.begin();
         p != impl->nailDriverDevs.end(); p++) {
        NailDriver* nailDriver = p->second;
        nailDriver->on(on);
    }
}

void ODESimulatorItem::setNailDriverLimitCheckStartTime(double limitCheckStartTime)
{
    impl->nailDriverLimitCheckStartTime = limitCheckStartTime;
}

void ODESimulatorItem::setAllLinkPositionOutputMode(bool on)
{
    // The mode is not changed.
    // This simulator only supports the all link position output
    // because joint positions may be slightly changed
}


void ODESimulatorItemImpl::clear()
{
    dJointGroupEmpty(contactJointGroupID);

    if(worldID){
        dWorldDestroy(worldID);
        worldID = 0;
    }
    if(spaceID){
        dSpaceDestroy(spaceID);
        spaceID = 0;
    }

    crawlerLinks.clear();
    vacuumGripperDevs.clear();
    nailDriverDevs.clear();
    //nailedObjectMngr.clear();
#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
    mecanumWheelSetting.clear();
#endif                      /* MECANUM_WHEEL_ODE */
    geometryIdToLink.clear();
}    


Item* ODESimulatorItem::doDuplicate() const
{
    return new ODESimulatorItem(*this);
}



SimulationBody* ODESimulatorItem::createSimulationBody(Body* orgBody)
{
    return new ODEBody(*orgBody);
}


bool ODESimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    return impl->initializeSimulation(simBodies);
}


bool ODESimulatorItemImpl::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
{
    clear();

    flipYZ = is2Dmode;

    Vector3 g = gravity;
    
    if(flipYZ){
        toInternal(gravity, g);
    }

    worldID = dWorldCreate();
    if(useWorldCollision){
        collisionDetector = self->collisionDetector();
        collisionDetector->clearGeometries();
    }else{
        spaceID = dHashSpaceCreate(0);
        dSpaceSetCleanup(spaceID, 0);
    }

    dRandSetSeed(0);
    dWorldSetGravity(worldID, g.x(), g.y(), g.z());
    dWorldSetERP(worldID, globalERP);
    dWorldSetCFM(worldID, globalCFM.value());
    dWorldSetContactSurfaceLayer(worldID, 0.0);
    dWorldSetQuickStepNumIterations(worldID, numIterations);
    dWorldSetQuickStepW(worldID, overRelaxation);
    dWorldSetContactMaxCorrectingVel(worldID, enableMaxCorrectingVel ? maxCorrectingVel.value() : dInfinity);
    dWorldSetContactSurfaceLayer(worldID, surfaceLayerDepth);

    timeStep = self->worldTimeStep();

    for(size_t i=0; i < simBodies.size(); ++i){
        addBody(static_cast<ODEBody*>(simBodies[i]));
    }
    if (!nailDriverDevs.empty()) {
        self->addPostDynamicsFunction(boost::bind(&ODESimulatorItemImpl::nailDriverCheck, this));
        self->addPostDynamicsFunction(boost::bind(&ODESimulatorItemImpl::nailedObjectLimitCheck, this));
    }
    if(useWorldCollision)
        collisionDetector->makeReady();

    if(MEASURE_PHYSICS_CALCULATION_TIME){
        physicsTime = 0;
        collisionTime = 0;
    }

    return true;
}


void ODESimulatorItemImpl::addBody(ODEBody* odeBody)
{
    Body& body = *odeBody->body();
cout << boost::format("ODESimulatorItemImpl::addBody: body.name=%s") % body.name() << endl;

    Link* rootLink = body.rootLink();
    rootLink->v().setZero();
    rootLink->dv().setZero();
    rootLink->w().setZero();
    rootLink->dw().setZero();

cout << boost::format("ODESimulatorItemImpl::addBody: body.numJoints()=%d") % body.numJoints() << endl;
    for(int i=0; i < body.numJoints(); ++i){
        Link* joint = body.joint(i);
        joint->u() = 0.0;
        joint->dq() = 0.0;
        joint->ddq() = 0.0;
    }
    
    body.clearExternalForces();
    body.calcForwardKinematics(true, true);

    odeBody->createBody(this);

    DeviceList<VacuumGripper> vacuumGrippers(body.devices());
    DeviceList<NailDriver> nailDrivers(body.devices());

#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
    preserveMecanumWheelSetting(odeBody);
#endif                      /* MECANUM_WHEEL_ODE */

    for (size_t i=0; i < odeBody->odeLinks.size(); ++i) {
	ODELinkPtr odeLink = odeBody->odeLinks[i];
	for (unsigned int j=0; j<vacuumGrippers.size(); j++){
	    VacuumGripper *vacuumGripper = vacuumGrippers[j];
	    if (odeLink->link == vacuumGripper->link()){
cout << boost::format("Add VacuumGripper: bodyID=%d target=%s")% odeLink->bodyID % vacuumGripper->link()->name() << endl;
MessageView::instance()->putln(boost::format("Add VacuumGripper: bodyID=%d target=%s")% odeLink->bodyID % vacuumGripper->link()->name());
                vacuumGripper->gripper = odeLink->bodyID;
                vacuumGripperDevs.insert(make_pair(odeLink->bodyID,
						   vacuumGripper));
	    }
	}
	for (unsigned int j=0; j<nailDrivers.size(); j++) {
	    NailDriver *nailDriver = nailDrivers[j];
	    if (odeLink->link == nailDriver->link()) {
cout << boost::format("Add NailDriver: bodyID=%d target=%s")% odeLink->bodyID % nailDriver->link()->name() << endl;
MessageView::instance()->putln(boost::format("Add NailDriver: bodyID=%d target=%s")% odeLink->bodyID % nailDriver->link()->name());
                nailDriverDevs.insert(make_pair(odeLink->bodyID,
						nailDriver));
	    }
	}
    }
}

#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
/**
   @brief Preserve mecanum wheel setting.
   @param[in] odeBody Pointer of ODEBody.
   @attention Please this method calling after addBody method.
 */
void ODESimulatorItemImpl::preserveMecanumWheelSetting(ODEBody* odeBody)
{
    Mapping* m;
    Listing* links;
    Listing* angles;

    if (! odeBody) {
        return;
    }

    m = odeBody->body()->info()->findMapping("mecanumWheelSetting");

    if (! m->isValid()) {
        return;
    }

    links  = m->findListing("links");
    angles = m->findListing("barrelAngles");

    if (! links->isValid() || links->size() < 1) {
        return;
    }

    for (size_t i = 0; i < links->size(); i++) {
        try {
            string s = links->at(i)->toString();
            double d = PI_2;
            Link*  p = odeBody->body()->link(s);

            if (! p) {
                MessageView::instance()->putln(
                    MessageView::ERROR,
                    boost::format("link %1% not found in the %2%") % s % odeBody->body()->name()
                    );
                continue;
            } else if (p->jointType() != Link::CRAWLER_JOINT) {
                MessageView::instance()->putln(
                    MessageView::ERROR,
                    boost::format("link %1% is not crawler joint in the %2%") % s % odeBody->body()->name()
                    );
                continue;
            }

            if (angles->isValid() && angles->size() > i) {
                d = angles->at(i)->toDouble();
            }

            for (size_t j = 0; j < odeBody->odeLinks.size(); j++) {
                if (p == odeBody->odeLinks[j]->link) {
                    mecanumWheelSetting.insert(make_pair(odeBody->odeLinks[j]->bodyID, d));
                    break;
                }
            }
        } catch (const ValueNode::NotScalarException ex) {
            MessageView::instance()->putln(MessageView::ERROR, ex.message());
        } catch(const ValueNode::ScalarTypeMismatchException ex) {
            MessageView::instance()->putln(MessageView::ERROR, ex.message());
        }
    }

    return;
}
#endif                      /* MECANUM_WHEEL_ODE */

void ODESimulatorItem::initializeSimulationThread()
{
    dAllocateODEDataForThread(dAllocateMaskAll);
}

static void nearCallback(void* data, dGeomID g1, dGeomID g2)
{
    if(dGeomIsSpace(g1) || dGeomIsSpace(g2)) { 
        dSpaceCollide2(g1, g2, data, &nearCallback);
        if(false) { // Currently just skip same body link pairs. 
            if(dGeomIsSpace(g1)){
                dSpaceCollide((dSpaceID)g1, data, &nearCallback);
            }
            if(dGeomIsSpace(g2)){
                dSpaceCollide((dSpaceID)g2, data, &nearCallback);
            }
        }
    } else {
        ODESimulatorItemImpl* impl = (ODESimulatorItemImpl*)data;
        static const int MaxNumContacts = 100;
        dContact contacts[MaxNumContacts];
        int numContacts = dCollide(g1, g2, MaxNumContacts, &contacts[0].geom, sizeof(dContact));
        
        if(numContacts > 0){
            dBodyID body1ID = dGeomGetBody(g1);
            dBodyID body2ID = dGeomGetBody(g2);
            Link* crawlerlink = 0;
            double sign = 1.0;
#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
            bool isMecanumWheel = false;
            double barrelAngle  = 0.0;
#endif                      /* MECANUM_WHEEL_ODE */
            if(!impl->crawlerLinks.empty()){
                CrawlerLinkMap::iterator p = impl->crawlerLinks.find(body1ID);
                if(p != impl->crawlerLinks.end()){
                    crawlerlink = p->second;
#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
                    {
                        MecanumWheelSettingMap::iterator it = impl->mecanumWheelSetting.find(body1ID);

                        if (it != impl->mecanumWheelSetting.end()) {
                            isMecanumWheel = true;
                            barrelAngle    = it->second;
                        }
                    }
#endif                      /* MECANUM_WHEEL_ODE */
                }
                p = impl->crawlerLinks.find(body2ID);
                if(p != impl->crawlerLinks.end()){
                    crawlerlink = p->second;
                    sign = -1.0;
#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
                    {
                        MecanumWheelSettingMap::iterator it = impl->mecanumWheelSetting.find(body2ID);

                        if (it != impl->mecanumWheelSetting.end()) {
                            isMecanumWheel = true;
                            barrelAngle    = it->second;
                        } else {
                            isMecanumWheel = false;
                            barrelAngle    = 0.0;
                        }
                    }
#endif                      /* MECANUM_WHEEL_ODE */
                }
            }
#if 1    /* Experimental. */
            if(!impl->vacuumGripperDevs.empty()){
                VacuumGripper* vacuumGripper = 0;
                dBodyID objId = 0;
		if ((vacuumGripper = impl->isVacuumGripper(body1ID))){
                    objId = body2ID;
#ifdef VACUUM_GRIPPER_DEBUG
MessageView::instance()->putln("*** vacuum gripper : body1 ***");
#endif // VACUUM_GRIPPER_DEBUG
                } else if ((vacuumGripper = impl->isVacuumGripper(body2ID))){
		    objId = body1ID;
#ifdef VACUUM_GRIPPER_DEBUG
MessageView::instance()->putln("*** vacuum gripper : body2 ***");
#endif // VACUUM_GRIPPER_DEBUG
                }
                if (vacuumGripper != 0) {
                    if (vacuumGripper->on()) {
                        if (vacuumGripper->isGripping()) {
#ifdef VACUUM_GRIPPER_DEBUG
MessageView::instance()->putln("*** vacuum gripper already gripping ***");
cout << "*** vacuum gripper already gripping ***" << endl;
#endif // VACUUM_GRIPPER_DEBUG
                            if (vacuumGripper->isGripping(objId)) {
                                // limit check
                                if (impl->self->currentTime() < impl->vacuumGripperLimitCheckStartTime) {
                                    return;
                                }

                                if (vacuumGripper->limitCheck()){
                                    MessageView::instance()->putln("VacuumGripper: *** joint destroy : exceeded the limit ***");
                                    cout << "VacuumGripper: *** joint destroy : exceeded the limit  **" << endl;
				    vacuumGripper->release();
                                }else{
				    return;
				}
                            } else {
#ifdef VACUUM_GRIPPER_DEBUG
MessageView::instance()->putln(boost::format("VacuumGripper: *** other body jointed %s ***") % objId);
cout << boost::format("VacuumGripper: *** other body jointed %s ***") % objId << endl;
#endif // VACUUM_GRIPPER_DEBUG
                                ;
                            }
                        } else { // !vacuumGripper->isGripping()
                            int n = vacuumGripper->checkContact(numContacts, contacts);
                            if (n != 0) {
				vacuumGripper->grip(impl->worldID, objId);
				return;
                            } else {
#ifdef VACUUM_GRIPPER_DEBUG
MessageView::instance()->putln("VacuumGripper: *** cannot create joint **");
cout << "VacuumGripper: *** cannot create joint **" << endl;
#endif // VACUUM_GRIPPER_DEBUG
                                ;
                            }
                        } // vacuumGripper->isGripping()
                    } else { // vacuumGripper is off
#ifdef VACUUM_GRIPPER_DEBUG
cout << "VacuumGripper OFF **" << endl;
#endif // VACUUM_GRIPPER_DEBUG
                        if (vacuumGripper->isGripping()) {
			    vacuumGripper->release();
                        }
                    } // vacuumGripper->on()
                } // vacuumGripper != 0
            }
#endif    /* Experimental. */
#if 1    /* Experimental. */
            if(!impl->nailDriverDevs.empty()){
                NailedObjectManager* nailedObjMngr = NailedObjectManager::getInstance();
                NailDriver* nailDriver = 0;
                dBodyID objId = 0;
		if ((nailDriver = impl->isNailDriver(body1ID))){
                    objId = body2ID;
#ifdef NAILDRIVER_DEBUG
MessageView::instance()->putln(boost::format(_("NailDriver body1ID=%1%, Object body2ID=%2%")) % body1ID % objId);
#endif // NAILDRIVER_DEBUG
                } else if ((nailDriver = impl->isNailDriver(body2ID))){
		    objId = body1ID;
#ifdef NAILDRIVER_DEBUG
MessageView::instance()->putln(boost::format(_("NailDriver body2ID=%1%, Object body1ID=%2%")) % body2ID % objId);
#endif // NAILDRIVER_DEBUG
                }
                if (nailDriver != 0) {
                    nailDriver->contact();
                    int n = nailDriver->checkContact(numContacts, contacts);
#ifdef NAILDRIVER_DEBUG
MessageView::instance()->putln(boost::format(_("NailDriver: numContacts=%d n=%d")) % numContacts % n);
cout << boost::format(_("NailDriver: numContacts=%d n=%d")) % numContacts % n << endl;
#endif // NAILDRIVER_DEBUG
                    if (n) {
                        if (nailDriver->ready()) {
#ifdef NAILDRIVER_DEBUG
cout << "NailDriver ON **" << endl;
#endif // NAILDRIVER_DEBUG

#ifdef NAILDRIVER_DEBUG
MessageView::instance()->putln(boost::format(_("NailDriver check body1ID=%1%")) % objId);
#endif // NAILDRIVER_DEBUG
                            NailedObjectPtr nobj = nailedObjMngr->get(objId);
                            if (!nobj) {
                                // first
                                nobj = new NailedObject(impl->worldID, objId);
                                nailDriver->fire(nobj);
                                nailedObjMngr->addObject(nobj);
                                MessageView::instance()->putln("NailDriver: *** joint created **");
                                cout << "NailDriver: *** joint created **" << endl;
#ifdef NAILDRIVER_DEBUG
MessageView::instance()->putln(boost::format("NailDriver: nail count = %d") % nobj->getNailCount());
cout << "NailDriver: nail count = " << nobj->getNailCount() << endl;
#endif // NAILDRIVER_DEBUG
                            } else {
                                // second
                                nailDriver->fire(nobj);
#ifdef NAILDRIVER_DEBUG
MessageView::instance()->putln(boost::format("NailDriver: nail count = %d") % nobj->getNailCount());
cout << "NailDriver: nail count = " << nobj->getNailCount() << endl;
#endif // NAILDRIVER_DEBUG
                            }
                        } // nailDriver->ready()
                        else {
#ifdef NAILDRIVER_DEBUG
cout << "NailDriver OFF **" << endl;
#endif // NAILDRIVER_DEBUG
                            ;
                        }
                    } // n != 0
                } // nailDriver == 0
            } // empty()
#endif    /* Experimental. */
	    for(int i=0; i < numContacts; ++i){
                dSurfaceParameters& surface = contacts[i].surface;
                if(!crawlerlink){
                    //surface.mode = dContactApprox1 | dContactBounce;
                    //surface.bounce = 0.0;
                    //surface.bounce_vel = 1.0;
                    surface.mode = dContactApprox1;
                    surface.mu = impl->friction;

                } else {
                    if(contacts[i].geom.depth > 0.001){
                        continue;
                    }
                    surface.mode = dContactFDir1 | dContactMotion1 | dContactMu2 | dContactApprox1_2 | dContactApprox1_1;
                    const Vector3 axis = crawlerlink->R() * crawlerlink->a();
                    const Vector3 n(contacts[i].geom.normal);
                    Vector3 dir = axis.cross(n);

#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
                    if (isMecanumWheel) {
                        Vector3 mwdir = AngleAxis(barrelAngle, n).toRotationMatrix().transpose() * dir;

                        // XXX: this block will erase later.
                        if (impl->mecanumWheelDebug) {
                            cout << crawlerlink->name() << endl;
                            cout << "dir  : " << dir.transpose() << endl;
                            cout << "mwdir: " << mwdir.transpose() << endl;
                            cout << "-----" << endl;
                        }

                        dir = mwdir;
                    }
#endif                      /* MECANUM_WHEEL_ODE */

                    if(dir.norm() < 1.0e-5){
                        surface.mode = dContactApprox1;
                        surface.mu = impl->friction;
                    } else {
                        dir *= sign;
                        dir.normalize();
                        contacts[i].fdir1[0] = dir[0];
                        contacts[i].fdir1[1] = dir[1];
                        contacts[i].fdir1[2] = dir[2];
                        //dVector3& dpos = contacts[i].geom.pos;
                        //Vector3 pos(dpos[0], dpos[1], dpos[2]);
                        //Vector3 v = crawlerlink->v + crawlerlink->w.cross(pos-crawlerlink->p);
                        //surface.motion1 = dir.dot(v) + crawlerlink->u;
                        if(crawlerlink->jointType()==Link::PSEUDO_CONTINUOUS_TRACK)
                            surface.motion1 = crawlerlink->dq();
                        else
                            surface.motion1 = crawlerlink->u();
                        surface.mu = impl->friction;
                        surface.mu2 = 0.5;
                    }
                }
                dJointID jointID = dJointCreateContact(impl->worldID, impl->contactJointGroupID, &contacts[i]);
                dJointAttach(jointID, body1ID, body2ID);
            }
        }
    }
}


bool ODESimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
    return impl->stepSimulation(activeSimBodies);
}


bool ODESimulatorItemImpl::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
{
	for(size_t i=0; i < activeSimBodies.size(); ++i){
        ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);
        odeBody->body()->setVirtualJointForces();
        if(odeBody->worldID){
        	if(velocityMode)
        		odeBody->setVelocityToODE();
        	else
        		odeBody->setTorqueToODE();
        }
    }

	if(MEASURE_PHYSICS_CALCULATION_TIME){
	    physicsTimer.start();
	}

    dJointGroupEmpty(contactJointGroupID);
    if(useWorldCollision){
        for(size_t i=0; i < activeSimBodies.size(); ++i){
            ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);
            for(size_t j=0; j< odeBody->odeLinks.size(); j++){
                int k = odeBody->geometryId+j;
                collisionDetector->updatePosition( k, geometryIdToLink[k]->link->T());
            }
        }
        collisionDetector->detectCollisions(std::bind(&ODESimulatorItemImpl::collisionCallback, this, _1));
    }else{
        if(MEASURE_PHYSICS_CALCULATION_TIME){
            collisionTimer.start();
        }
        dSpaceCollide(spaceID, (void*)this, &nearCallback);
        if(MEASURE_PHYSICS_CALCULATION_TIME){
            collisionTime += collisionTimer.nsecsElapsed();
        }
    }

    if(stepMode.is(ODESimulatorItem::STEP_ITERATIVE)){
        dWorldQuickStep(worldID, timeStep);
    } else {
        dWorldStep(worldID, timeStep);
    }

    if(MEASURE_PHYSICS_CALCULATION_TIME){
        physicsTime += physicsTimer.nsecsElapsed();
    }

    //! \todo Bodies with sensors should be managed by the specialized container to increase the efficiency
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);

        if(odeBody->worldID){
			// Move the following code to the ODEBody class
			if(is2Dmode){
				odeBody->alignToZAxisIn2Dmode();
			}
			if(!odeBody->sensorHelper.forceSensors().empty()){
				odeBody->updateForceSensors(flipYZ);
			}
			odeBody->getKinematicStateFromODE(flipYZ);
			if(odeBody->sensorHelper.hasGyroOrAccelerationSensors()){
				odeBody->sensorHelper.updateGyroAndAccelerationSensors();
			}
        }
    }

    return true;
}


void ODESimulatorItemImpl::collisionCallback(const CollisionPair& collisionPair)
{
    ODELink* link1 = geometryIdToLink[collisionPair.geometryId[0]];
    ODELink* link2 = geometryIdToLink[collisionPair.geometryId[1]];
    const vector<Collision>& collisions = collisionPair.collisions;

cout << "ODESimulatorItemImpl:collisionCallback ***" << endl;
    dBodyID body1ID = link1->bodyID;
    dBodyID body2ID = link2->bodyID;
cout << "ODESimulatorItemImpl:collisionCallback: body1ID:" << body1ID << endl;
cout << "ODESimulatorItemImpl:collisionCallback: body2ID:" << body2ID << endl;
    Link* crawlerlink = 0;
    double sign = 1.0;
    if(!crawlerLinks.empty()){
        CrawlerLinkMap::iterator p = crawlerLinks.find(body1ID);
        if(p != crawlerLinks.end()){
            crawlerlink = p->second;
        }
        p = crawlerLinks.find(body2ID);
        if(p != crawlerLinks.end()){
            crawlerlink = p->second;
            sign = -1.0;
        }
    }

    int numContacts = collisions.size();
    for(int i=0; i < numContacts; ++i){
        dContact contact;
        contact.geom.pos[0] = collisions[i].point[0];
        contact.geom.pos[1] = collisions[i].point[1];
        contact.geom.pos[2] = collisions[i].point[2];
        contact.geom.normal[0] = -collisions[i].normal[0];
        contact.geom.normal[1] = -collisions[i].normal[1];
        contact.geom.normal[2] = -collisions[i].normal[2];
        contact.geom.depth = collisions[i].depth;

        dSurfaceParameters& surface = contact.surface;
        if(!crawlerlink){
            surface.mode = dContactApprox1;
            surface.mu = friction;
        } else {
            if(contact.geom.depth > 0.001){
                continue;
            }
            surface.mode = dContactFDir1 | dContactMotion1 | dContactMu2 | dContactApprox1_2;
            const Vector3 axis = crawlerlink->R() * crawlerlink->a();
            const Vector3 n(contact.geom.normal);
            Vector3 dir = axis.cross(n);
            if(dir.norm() < 1.0e-5){
                surface.mode = dContactApprox1;
                surface.mu = friction;
            } else {
                dir *= sign;
                dir.normalize();
                contact.fdir1[0] = dir[0];
                contact.fdir1[1] = dir[1];
                contact.fdir1[2] = dir[2];
                surface.motion1 = crawlerlink->u();
                surface.mu = friction;
                surface.mu2 = 0.5;
            }
        }
        dJointID jointID = dJointCreateContact(worldID, contactJointGroupID, &contact);
        dJointAttach(jointID, body1ID, body2ID);
    }
}


void ODESimulatorItem::finalizeSimulation()
{
    if(MEASURE_PHYSICS_CALCULATION_TIME){
        cout << "ODE physicsTime= " << impl->physicsTime *1.0e-9 << "[s]"<< endl;
        cout << "ODE collisionTime= " << impl->collisionTime *1.0e-9 << "[s]"<< endl;
    }
}


void ODESimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SimulatorItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void ODESimulatorItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Step mode"), stepMode, changeProperty(stepMode));

    putProperty(_("Gravity"), str(gravity), std::bind(toVector3, _1, std::ref(gravity)));

    putProperty.decimals(2).min(0.0)
        (_("Friction"), friction, changeProperty(friction));

    putProperty(_("Limit joint range"), isJointLimitMode, changeProperty(isJointLimitMode));

    putProperty.decimals(1).min(0.0).max(1.0)
        (_("Global ERP"), globalERP, changeProperty(globalERP));

    putProperty(_("Global CFM"), globalCFM,
                std::bind(&FloatingNumberString::setNonNegativeValue, std::ref(globalCFM), _1));

    putProperty.min(1)
        (_("Iterations"), numIterations, changeProperty(numIterations));

    putProperty.min(0.1).max(1.9)
        (_("Over relaxation"), overRelaxation, changeProperty(overRelaxation));

    putProperty(_("Limit correcting vel."), enableMaxCorrectingVel, changeProperty(enableMaxCorrectingVel));

    putProperty(_("Max correcting vel."), maxCorrectingVel,
                std::bind(&FloatingNumberString::setNonNegativeValue, std::ref(maxCorrectingVel), _1));

    putProperty(_("2D mode"), is2Dmode, changeProperty(is2Dmode));

    putProperty(_("Use WorldItem's Collision Detector"), useWorldCollision, changeProperty(useWorldCollision));

    putProperty(_("Velocity Control Mode"), velocityMode, changeProperty(velocityMode));

#ifdef MECANUM_WHEEL_ODE    /* MECANUM_WHEEL_ODE */
    // XXX: this code will erase later.
    putProperty("Mecanum Wheel Debug Mode", mecanumWheelDebug, changeProperty(mecanumWheelDebug));
#endif                      /* MECANUM_WHEEL_ODE */
}


bool ODESimulatorItem::store(Archive& archive)
{
    SimulatorItem::store(archive);
    impl->store(archive);
    return true;
}


void ODESimulatorItemImpl::store(Archive& archive)
{
    archive.write("stepMode", stepMode.selectedSymbol());
    write(archive, "gravity", gravity);
    archive.write("friction", friction);
    archive.write("jointLimitMode", isJointLimitMode);
    archive.write("globalERP", globalERP);
    archive.write("globalCFM", globalCFM);
    archive.write("numIterations", numIterations);
    archive.write("overRelaxation", overRelaxation);
    archive.write("limitCorrectingVel", enableMaxCorrectingVel);
    archive.write("maxCorrectingVel", maxCorrectingVel);
    archive.write("2Dmode", is2Dmode);
    archive.write("UseWorldItem'sCollisionDetector", useWorldCollision);
    archive.write("velocityMode", velocityMode);
}


bool ODESimulatorItem::restore(const Archive& archive)
{
    SimulatorItem::restore(archive);
    impl->restore(archive);
    return true;
}


void ODESimulatorItemImpl::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("stepMode", symbol)){
        stepMode.select(symbol);
    }
    read(archive, "gravity", gravity);
    archive.read("friction", friction);
    archive.read("jointLimitMode", isJointLimitMode);
    archive.read("globalERP", globalERP);
    globalCFM = archive.get("globalCFM", globalCFM.string());
    archive.read("numIterations", numIterations);
    archive.read("overRelaxation", overRelaxation);
    archive.read("limitCorrectingVel", enableMaxCorrectingVel);
    maxCorrectingVel = archive.get("maxCorrectingVel", maxCorrectingVel.string());
    archive.read("2Dmode", is2Dmode);
    archive.read("UseWorldItem'sCollisionDetector", useWorldCollision);
    archive.read("velocityMode", velocityMode);
}

VacuumGripper *ODESimulatorItemImpl::isVacuumGripper(dBodyID body)
{
    VacuumGripperMap::iterator p = vacuumGripperDevs.find(body);
    if (p != vacuumGripperDevs.end()) {
	return p->second;
    }else{
	return 0;
    }
}

NailDriver *ODESimulatorItemImpl::isNailDriver(dBodyID body)
{
    NailDriverMap::iterator p = nailDriverDevs.find(body);
    if (p != nailDriverDevs.end()) {
	return p->second;
    }else{
	return 0;
    }
}

/*
 */
void ODESimulatorItemImpl::nailDriverCheck()
{
    for (NailDriverMap::iterator p = nailDriverDevs.begin();
         p != nailDriverDevs.end(); p++) {
        NailDriver* nailDriver = p->second;
        nailDriver->distantCheck();
    }
}

/*
 */
void ODESimulatorItemImpl::nailedObjectLimitCheck()
{

    if (self->currentTime() < nailDriverLimitCheckStartTime) {
        return;
    }

    NailedObjectManager* nailedObjMngr = NailedObjectManager::getInstance();

    NailedObjectMap& map = nailedObjMngr->map();

    NailedObjectMap::iterator p = map.begin();
    while (p != map.end()) {
        dBodyID id = p->first;
        NailedObjectPtr nobj = p->second;
        const dReal* pos = dBodyGetPosition(id);

        Vector3 f(nobj->fb.f1);
        Vector3 tau(nobj->fb.t1);

#if 0 /* Experimental. */
        if (nobj->isLimited(f[0])) {
            MessageView::instance()->putln("NailDriver: *** exceeded the limit ***");
            cout << "NailDriver: *** exceeded the limit  **" << endl;
            map.erase(p++);
        } else {
            p++;
        }
#else /* Experimental. */
        p++;
#endif /* Experimental. */
    }
}
