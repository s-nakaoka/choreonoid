#ifndef CNOID_BODY_HOLDER_DEVICE_H
#define CNOID_BODY_HOLDER_DEVICE_H

#include "Device.h"
#include "exportdecl.h"

namespace cnoid {

class AttachmentDevice;
typedef ref_ptr<AttachmentDevice> AttachmentDevicePtr;
class YAMLBodyLoader;
class Mapping;

class CNOID_EXPORT HolderDevice : public Device
{
public:
    HolderDevice();
    virtual ~HolderDevice();

    virtual const char* typeName() override;
    void copyHolderDeviceStateFrom(const HolderDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    virtual bool on() const override;
    virtual void on(bool on) override;

    std::string category() const;
    void setCategory(const std::string& category);

    int numAttachments() const;
    AttachmentDevice* attachment(int index);
    bool addAttachment(AttachmentDevice* attachment);
    bool removeAttachment(AttachmentDevice* attachment);

    bool readDescription(YAMLBodyLoader& loader, Mapping& node);

protected:
    HolderDevice(const HolderDevice& org, bool copyStateOnly, CloneMap* cloneMap);
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    bool on_;
    class NonState;
    NonState* ns;
};

typedef ref_ptr<HolderDevice> HolderDevicePtr;

}

#endif
