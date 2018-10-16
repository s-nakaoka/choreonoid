#include <cnoid/SimpleController>

#include <iostream>
#include <vector>

using namespace cnoid;
using namespace std;

class Jaco2test : public SimpleController
{
  BodyPtr body;
  double dt;
  vector<double> q_ref;
  vector<double> q_prev;
  
  // enum {
  //   SHOULDER,
  //   ARM,
  //   FOREARM,
  //   WRIST1,
  //   WRIST2,
  //   HAND,
  //   FINGER1,
  //   FINGER2,
  //   FINGER3,
  //   //FINGER1_TIP,
  //   //FINGER2_TIP,
  //   //FINGER3_TIP,
  //   NUM_JOINTS
  // };

public:
  virtual bool initialize(SimpleControllerIO* io) override
  {
    body = io->body();
    dt = io->timeStep();

    for(int i=0; i<body->numJoints(); i++){
      Link* joint = body->joint(i);
      // joint = io->body()->link("SHOULDER");
      joint->setActuationMode(Link::JOINT_TORQUE);
      io->enableIO(joint);
      q_ref.push_back(joint->q());
    }
    
    q_prev = q_ref;
    
    return true;
  }

  virtual bool control() override
  {
    // Super SHIMOE Params
    static const double P = 200.0;
    static const double D = 50.0;

    
    for(int i=0; i < body->numJoints(); ++i){
      Link* joint = body->joint(i);
      double q = joint->q();
      double dq = (q - q_prev[i]) / dt;
      double u = (q_ref[i] - q) * P + (0.0 - dq) * D;
      cout << "=== q : " << q << " === " << endl;
      cout << "=== dq : " << dq << " ===" << endl;
      cout << "=== u : " << u << " ===" << endl;
      q_prev[i] = q;
      joint->u() = u;
    }
    return true;


    // Super SHIMOE TIME
    // // PD gains
    // static const double P = 200.0;
    // static const double D = 50.0;

    // double q = joint->q(); // input
    // double dq = (q - q_prev) / dt;
    // double dq_ref = 0.0;
    // joint->u() = P * (q_ref - q) + D * (dq_ref - dq); // output
    // q_prev = q;

    // return true;
  }
};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Jaco2test)

