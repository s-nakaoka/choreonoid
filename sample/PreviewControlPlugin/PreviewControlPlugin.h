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
#include <cnoid/ItemManager>
#include <cnoid/FileUtil>
#include <cnoid/BodyMotionGenerationBar>
#include <cnoid/JointPath>

#include <cnoid/LeggedBodyHelper>
#include <cnoid/AppConfig>

#include <DynamicsPlugin/DynamicsPlugin.h>
#include <UtilPlugin/UtilPlugin.h>

#include "PreviewController.h"

using namespace boost;
using namespace cnoid;
using namespace std;

class PreviewControlPlugin : public Plugin
{
 public:
    
 PreviewControlPlugin() : Plugin("PreviewControl")
    {
      require("Body");
      require("Dynamics");
      require("Util");
    }
    
  virtual bool initialize()
  {
    ToolBar* bar = new ToolBar("PreviewControl");
    bar->addButton("PreviewControl")->sigClicked().connect(bind(&PreviewControlPlugin::PreviewControl, this));
    /* bar->setVisibleByDefault(true); */ /* 関係ない?? */
    addToolBar(bar);

    return true;
  }

  double calcZFromSphere(const Vector3d centerPos, const Vector3d pos, const double radius );

  // 腰位置を可動域内に修正
  void modifyWaistIntoRange
    ( Vector3d& waistPos, const Vector3d lFootPos, const Vector3d rFootPos, const Vector3d lHipPos, const Vector3d rHipPos, const double legLength );

  // 予見制御
  void PreviewControl();

};
