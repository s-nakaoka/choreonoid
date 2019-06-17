/**
   \file
   \author Rafael Cisneros
*/

#include "TactileSensor.h"

using namespace cnoid;


TactileSensor::TactileSensor()
{
  forceData_ = std::make_shared<ForceData>();
}


TactileSensor::TactileSensor(const TactileSensor& org, bool copyStateOnly)
  : Device(org, copyStateOnly)
{
  copyStateFrom(org);
}


const char* TactileSensor::typeName()
{
  return "TactileSensor";
}


void TactileSensor::copyStateFrom(const TactileSensor& other)
{
  forceData_ = other.forceData_;
}


void TactileSensor::copyStateFrom(const DeviceState& other)
{
  if (typeid(other) != typeid(TactileSensor))
    throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
  
  copyStateFrom(static_cast<const TactileSensor&>(other));
}


Device* TactileSensor::clone() const
{
  return new TactileSensor(*this, false);
}


DeviceState* TactileSensor::cloneState() const
{
  return new TactileSensor(*this, true);
}


void TactileSensor::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
  if (!func(typeid(TactileSensor)))
    Device::forEachActualType(func);
}


void TactileSensor::clearState()
{
  //forceData_ = std::make_shared<ForceData>(std::make_pair(Vector2::Zero(), Vector3::Zero()));
  forceData_->clear();
}


int TactileSensor::stateSize() const
{
  return 0;
}


const double* TactileSensor::readState(const double* buf)
{
  return buf;
}


double* TactileSensor::writeState(double* out_buf) const
{
  return out_buf;
}
