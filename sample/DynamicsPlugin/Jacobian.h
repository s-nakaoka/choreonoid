#include <iostream>
#include <sstream>
#include <fstream>
#include <map>

#include <unistd.h>

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <boost/bind.hpp>
#include <src/PoseSeqPlugin/PoseSeqItem.h>
#include <cnoid/Jacobian>

namespace cnoid {

  struct SubMassInertia
  {
    double m;
    Vector3 mwc;
    Matrix3d Iw;
    SubMassInertia& operator+=(const SubMassInertia& rhs){
      m += rhs.m;
      mwc += rhs.mwc;
      Iw += rhs.Iw;
			return *this;
    }
  };


  CNOID_EXPORT void calcMomentumJacobian(const BodyPtr& body, Eigen::MatrixXd& M, Eigen::MatrixXd& H);

  CNOID_EXPORT void calcSubMassInertia(Link* link, std::vector<SubMassInertia>& subMassInertias);


}

