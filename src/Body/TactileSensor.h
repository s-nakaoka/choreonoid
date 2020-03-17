/**
   \file
   \author Rafael Cisneros
*/

#ifndef CNOID_BODY_TACTILE_SENSOR_H
#define CNOID_BODY_TACTILE_SENSOR_H

#include "Device.h"
#include <memory>
#include "exportdecl.h"
#include <iostream>
namespace cnoid {

class CNOID_EXPORT TactileSensor : public Device
{
 public:

  TactileSensor();
  TactileSensor(const TactileSensor& org, bool copyStateOnly = false);

  virtual const char* typeName() override;
  void copyStateFrom(const TactileSensor& other);
  virtual void copyStateFrom(const DeviceState& other) override;
  virtual DeviceState* cloneState() const override;
  virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
  virtual void clearState() override;
  virtual int stateSize() const override;
  virtual const double* readState(const double* buf) override;
  virtual double* writeState(double* out_buf) const override;

  typedef std::vector< std::pair<Vector2, Vector3> > ForceData;
  
  const ForceData& forceData() const { return *forceData_; }
  ForceData& forceData() { return *forceData_; }
  
  void setMaxX(double maxX);
  void setMaxY(double maxY); 
  void setMinX(double minX); 
  void setMinY(double minY); 
  void setRows(int rows); 
  void setCols(int cols); 
  
  double maxX();
  double maxY();
  double minY();
  double minX();
  int rows();
  int cols();

 protected:
  virtual Referenced* doClone(CloneMap* cloneMap) const override;
  
 private:

  std::shared_ptr<ForceData> forceData_;
  double maxX_;
  double maxY_;
  double minX_;
  double minY_;
  int rows_;
  int cols_;
};

 typedef ref_ptr<TactileSensor> TactileSensorPtr;

}

#endif
