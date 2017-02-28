/**
   @author Kunio Kojima
*/
#include <iostream>
#include <sstream>
#include <fstream>
#include <map>

#include <unistd.h>

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>
#include <cnoid/MessageView>
#include <src/PoseSeqPlugin/PoseSeqItem.h>
#include <src/Body/InverseDynamics.h>
#include <cnoid/Vector3Seq>

#include "Jacobian.h"
#include <src/Base/exportdecl.h>

using namespace cnoid;

class CNOID_EXPORT DynamicsPlugin : public Plugin
{
public:
    
  DynamicsPlugin() : Plugin("Dynamics")
  {
    require("Body");
    require("PoseSeq");
  }
    
  virtual bool initialize()
  {
    return true;
  }

  // zmp計算
  static void calcZMP( const BodyPtr& body, BodyMotionPtr motion, Vector3SeqPtr& zmpSeqPtr, const bool local = false );

};


/* CNOID_IMPLEMENT_PLUGIN_ENTRY(DynamicsPlugin) */
