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
#include <cnoid/Vector3Seq>
#include <cnoid/LeggedBodyHelper>

#include <src/Base/exportdecl.h>

namespace cnoid{

  class CNOID_EXPORT UtilPlugin : public Plugin
  {
  public:
    
  UtilPlugin() : Plugin("Util")
      {
        require("Body");
        require("PoseSeq");
      }
    
    virtual bool initialize()
    {
      return true;
    }

    static void getFootLink( Link** lFootLink, Link** rFootLink, const BodyPtr& body );

  };

}

/* CNOID_IMPLEMENT_PLUGIN_ENTRY(UtilPlugin) */
