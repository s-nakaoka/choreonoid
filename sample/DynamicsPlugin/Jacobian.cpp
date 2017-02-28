#include "Jacobian.h"


using namespace cnoid;

namespace {
  Matrix3d D(Vector3d r){
    Matrix3d r_cross;
    r_cross <<
      0.0,  -r(2), r(1),
      r(2),    0.0,  -r(0),
      -r(1), r(0),    0.0;
    return r_cross.transpose() * r_cross;
  }

}

namespace cnoid{

  // calcForwardKinematicsでwc等を更新しておく必要がある
  // リンク先の重心・慣性行列を計算
  void calcSubMassInertia(Link* link, std::vector<SubMassInertia>& subMassInertias){
    Matrix3d R = link->R();

    SubMassInertia& sub = subMassInertias[link->index()];
    sub.m = link->m();
    sub.mwc = link->m() * link->wc();

    for(Link* child = link->child(); child; child = child->sibling()){
      calcSubMassInertia(child, subMassInertias);
      SubMassInertia& childSub = subMassInertias[child->index()];
      sub.m += childSub.m;
      sub.mwc += childSub.mwc;
    }

    sub.Iw = R * link->I() * R.transpose() + link->m() * D( link->wc() - sub.mwc/sub.m );
    for(Link* child = link->child(); child; child = child->sibling()){
      SubMassInertia& childSub = subMassInertias[child->index()];
      sub.Iw += childSub.Iw + childSub.m * D( childSub.mwc/childSub.m - sub.mwc/sub.m );
    }

  }

  // calcForwardKinematicsでwc等を更新しておく必要がある
  // 運動量・角運動量ヤコビアンの計算
  void calcMomentumJacobian(const BodyPtr& body, Eigen::MatrixXd& M, Eigen::MatrixXd& H){
    
    // prepare subm, submwc
    const int nj = body->numJoints();
    std::vector<SubMassInertia> subMassInertias(body->numLinks());
    Link* rootLink = body->rootLink();

    calcSubMassInertia(rootLink, subMassInertias);

    // 運動量ヤコビアン計算
    MatrixXd M_ ;
    calcCMJacobian( body, NULL, M_);
    M = M_.block(0,0, 3,nj) * body->mass();

    // 角運動量ヤコビアン計算
    H.resize(3, nj);

    // std::vector<int> sgn(nj, 1);

    for(int i=0; i < nj; ++i){
      Link* joint = body->joint(i);
      if(joint->isRotationalJoint()){
        const Vector3 omega =  joint->R() * joint->a();
        const Vector3 Mcol = M.col(joint->jointId());// index()??
        const SubMassInertia& sub = subMassInertias[joint->index()];
        const Vector3 dp = (sub.mwc/sub.m).cross(Mcol) + sub.Iw * omega;

        H.col(joint->jointId()) = dp;
      } else {
        std::cerr << "calcCMJacobian() : unsupported jointType("
                  << joint->jointType() << std::endl;
      }
    }

    Vector3 c = body->calcCenterOfMass();
    Matrix3d c_cross;
    c_cross <<
      0.0,  -c(2), c(1),
      c(2),    0.0,  -c(0),
      -c(1), c(0),    0.0;
    H -= c_cross * M;
    
  }


}
