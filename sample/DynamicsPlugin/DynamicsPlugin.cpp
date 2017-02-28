/**
   @author Kunio Kojima
*/

#include "DynamicsPlugin.h"

using namespace boost;
using namespace cnoid;
using namespace std;

namespace {
  Vector6 calcInverseDynamicsSub(Link* link, const Vector3& vo_parent, const Vector3& dvo_parent)
  {
    Vector3 dvo, sv, sw;

    Link* parent = link->parent();
    if(!parent){
      dvo = link->dv() - link->dw().cross(link->p()) - link->w().cross(link->v());
      sv.setZero();
      sw.setZero();

    } else {
      switch(link->jointType()){
      case Link::ROTATIONAL_JOINT:
        sw.noalias() = link->R() * link->a();
        sv.noalias() = link->p().cross(sw);
        break;
      case Link::SLIDE_JOINT:
        sw.setZero();
        sv.noalias() = link->R() * link->d();
        break;
      case Link::FIXED_JOINT:
      default:
        sw.setZero();
        sv.setZero();
        break;
      }
      const Vector3 dsv = parent->w().cross(sv) + vo_parent.cross(sw);
      const Vector3 dsw = parent->w().cross(sw);

      link->dw()  = parent->dw()  + dsw * link->dq() + sw * link->ddq();
      dvo = dvo_parent + dsv * link->dq() + sv * link->ddq();
    }

    const Vector3 c = link->R() * link->c() + link->p();
    Matrix3 I = link->R() * link->I() * link->R().transpose();
    const Matrix3 c_hat = hat(c);
    I.noalias() += link->m() * c_hat * c_hat.transpose();
    const Vector3 vo = link->v() - link->w().cross(link->p());
    const Vector3 P = link->m() * (vo + link->w().cross(c));
    const Vector3 L = link->m() * c.cross(vo) + I * link->w();

    Vector6 f;
    f.head<3>() = link->m() * (dvo + link->dw().cross(c)) + link->w().cross(P);
    f.tail<3>() = link->m() * c.cross(dvo) + I * link->dw() + vo.cross(P) + link->w().cross(L);

    for(Link* child = link->child(); child; child = child->sibling()){
      f += calcInverseDynamicsSub(child, vo, dvo);
    }

    link->u() = sv.dot(f.head<3>()) + sw.dot(f.tail<3>()) + link->ddq() * link->Jm2() /* rotor inertia */;

    return f;
  }
}


void DynamicsPlugin::calcZMP( const BodyPtr& body, BodyMotionPtr motion, Vector3SeqPtr& zmpSeqPtr, const bool local ){
  int bodyItemIdx = 0,poseSeqIdx = 0;
  const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;
  // double g = -DEFAULT_GRAVITY_ACCELERATION;
  Vector3d g(0, 0, DEFAULT_GRAVITY_ACCELERATION);
  const double dt = 1/motion->frameRate();
  const double dt2 = dt * dt;

  FILE* fp = fopen("/tmp/tmp.dat","w");

  body->rootLink()->dv() = VectorXd::Zero(3);
  Vector3 lastdv = VectorXd::Zero(3);
  for(int currentFrame = 0; currentFrame < motion->numFrames(); ++currentFrame){
    // 腰座標更新
    motion->frame(currentFrame) >> *body;

    // 関節角速度・角加速度更新
    // int prevFrame = std::max(currentFrame - 1, 0);
    // int nextFrame = std::min(currentFrame + 1, motion->numFrames());
    int prevFrame = std::max(currentFrame - 1, 0);
    int nextFrame = std::min(currentFrame + 1, motion->numFrames()-1);
    MultiValueSeq::Frame q0 = motion->jointPosSeq()->frame(prevFrame);
    MultiValueSeq::Frame q1 = motion->jointPosSeq()->frame(currentFrame);
    MultiValueSeq::Frame q2 = motion->jointPosSeq()->frame(nextFrame);
    for(int k=0; k < motion->numJoints(); ++k){
      Link* joint = body->joint(k);
      joint->q() = q1[k];
      joint->dq() = (q2[k] - q1[k]) / dt;
      joint->ddq() = (q2[k] - 2.0 * q1[k] + q0[k]) / dt2;
    }
    // rootLink速度・加速度
    MultiSE3Seq::Frame p;
    p = motion->linkPosSeq()->frame(prevFrame);
    Vector3 p0 = p[0].translation(); Matrix3 R0 = p[0].rotation().toRotationMatrix();
    p = motion->linkPosSeq()->frame(currentFrame);
    Vector3 p1 = p[0].translation(); Matrix3 R1 = p[0].rotation().toRotationMatrix();
    p = motion->linkPosSeq()->frame(nextFrame);
    Vector3 p2 = p[0].translation(); Matrix3 R2 = p[0].rotation().toRotationMatrix();
    body->rootLink()->v() = (p2- p1) / dt;
    // body->rootLink()->dv() = (p2 - 2.0 * p1 + p0) / dt2 + g;
    Vector3 dv = (p2 - 2.0 * p1 + p0) / dt2;

    // if( (dv-lastdv).norm() < 0.15 ) body->rootLink()->dv() = dv;
    // else cout << " " << currentFrame*dt << " ddv is too large!" << endl;
    // lastdv = dv; fprintf(fp, "%f  %f\n", currentFrame*dt, (dv-lastdv).norm() );

    // body->rootLink()->dv() = dv;// 西脇式
    body->rootLink()->dv() = dv + g;// 長阪式

    // cout << currentFrame*dt << " " << body->rootLink()->p().transpose() << " " << body->rootLink()->v().transpose() << endl;
    fprintf(fp, "%f  %f %f  %f %f\n", currentFrame*dt, body->rootLink()->p().x(), body->rootLink()->p().y(), body->rootLink()->v().x(), body->rootLink()->v().y() );

    // rootLink角速度・角加速度
    // Matrix3 omega_cross0,omega_cross1;
    // omega_cross0 = (R2 - R0) * R1.inverse() / (2*dt);
    // body->rootLink()->w.x() = omega_cross0(2,1); body->rootLink()->w.y() = omega_cross0(0,2); body->rootLink()->w.z() = omega_cross0(1,0);
    // omega_cross0 = (R1 - R0) * R0.inverse() / dt;
    // omega_cross1 = (R2 -R1) * R1.inverse() / dt;
    // omega_cross1 = (omega_cross1 - omega_cross0) / dt;
    // body->rootLink()->dw.x() = omega_cross1(2,1); body->rootLink()->dw.y() = omega_cross1(0,2); body->rootLink()->dw.z() = omega_cross1(1,0);
    AngleAxis aa  = AngleAxis( R2 * R0.inverse() );
    body->rootLink()->w() = aa.axis() * aa.angle() / (2*dt);
    AngleAxis aa0 = AngleAxis( R1 * R0.inverse() );
    AngleAxis aa1 = AngleAxis( R2 * R1.inverse() );
    body->rootLink()->dw() = ( aa1.axis()*aa1.angle() - aa0.axis()*aa0.angle() ) / (dt*dt);

    // AngleAxis aa2 = AngleAxis(  ( R2*R1.inverse() )*(( R1*R0.inverse() ).inverse()) );
    // Vector3d dw_ = aa2.axis()*aa2.angle() / (dt*dt);x

    // if( omega_cross0(2,1) != 0 ){
    // cout << currentFrame << endl;
    // cout << body->rootLink()->w.transpose() << " " << (aa.axis() * aa.angle() / (2*dt) ).transpose() << endl;
    // cout << "dot02: " << omega_cross0(2,1) << " " << omega_cross0(0,2) << " " << omega_cross0(1,0) << endl;
    // cout << "inv:   " << body->rootLink()->w().transpose() << endl;
    // cout << "dw:    " << body->rootLink()->dw().transpose() << endl;
    // cout << "dw_:   " << dw_.transpose() << endl;
    // }

    // リンク速度・加速度計算
    body->calcForwardKinematics(true,true);
    // body->calcCM();
    body->calcCenterOfMass();


    // 長阪方式
    // Vector3d vo = body->rootLink()->v() - body->rootLink()->w().cross( body->rootLink()->p() );
    // Vector3d dvo = body->rootLink()->dv() - body->rootLink()->dw().cross(body->rootLink()->p())
      // - body->rootLink()->w().cross( vo + body->rootLink()->w().cross(body->rootLink()->p()) );
    // Vector6 f = calcInverseDynamicsSub( body->rootLink(), vo, dvo );
    Vector6 f = calcInverseDynamics( body->rootLink() );// これだとrootLinkの空間速度が設定されないがそれで良いみたい
    Vector3d out_f = f.head<3>();
    Vector3d out_tau = f.tail<3>();
        
    zmpSeqPtr->at(currentFrame).x() = - out_tau.y() / out_f.z();
    zmpSeqPtr->at(currentFrame).y() =   out_tau.x() / out_f.z();
    zmpSeqPtr->at(currentFrame).z() = 0;


    // // 西脇方式
    // double denom = 0;
    // Vector3d zmp = VectorXd::Zero(3);
    // for(int i = 0; i < body->numLinks(); ++i){
    //   Link* link = body->link(i);
    //   Matrix3d Iw = link->R().transpose() * link->I() * link->R();
    //   Vector3d accCM = link->dv() + link->dw().cross(link->R()*link->c()) + link->w().cross( link->w().cross( link->R()*link->c() ) );
    //   zmp.x() += link->m() * ( (accCM.z()+g.z())*link->wc().x() - link->wc().z()*accCM.x() );
    //   zmp.y() += link->m() * ( (accCM.z()+g.z())*link->wc().y() - link->wc().z()*accCM.y() );

    //   denom += link->m() * ( accCM.z()+g.z() );
    // }
    // // cout << dt*currentFrame << "   " << zmp.x() << " " << denom << endl;
    // zmp.x() /= denom; zmp.y() /= denom;// z成分は0
    // zmpSeqPtr->at(currentFrame) = zmp;
      
  }// motion loop
  // zmpSeqPtr->at( zmpSeqPtr->numFrames()-1 ) = zmpSeqPtr->at( max( 0, zmpSeqPtr->numFrames()-2 ) );
  return;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(DynamicsPlugin)
