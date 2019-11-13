/*! @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_BODY_LINK_VIEW_H
#define CNOID_BODYPLUGIN_BODY_LINK_VIEW_H

#include <cnoid/View>

namespace cnoid {

class BodyLinkView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static BodyLinkView* instance();

    BodyLinkView();
    virtual ~BodyLinkView();

    void switchRpyQuat(bool on);

protected:
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
