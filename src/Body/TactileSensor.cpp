/**
   \file
   \author Rafael Cisneros
*/

#include "TactileSensor.h"

using namespace cnoid;


TactileSensor::TactileSensor() //: maxX_(0), maxY_(0), minX_(0), minY_(0), rows_(0), cols_(0)
{
  forceData_ = std::make_shared<ForceData>();
  maxX_ = maxY_ = minX_ = minY_ = rows_ = cols_ = 0;
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
  maxX_ = other.maxX_;
  maxY_ = other.maxY_;
  minX_ = other.minX_;
  minY_ = other.minY_;
  rows_ = other.rows_;
  cols_ = other.cols_;
}


void TactileSensor::copyStateFrom(const DeviceState& other)
{
  if (typeid(other) != typeid(TactileSensor))
    throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
  
  copyStateFrom(static_cast<const TactileSensor&>(other));
}

Referenced* TactileSensor::doClone(CloneMap*) const
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
  forceData_->clear();
}


int TactileSensor::stateSize() const
{
  return forceData_->size() * 5;
}


const double* TactileSensor::readState(const double* buf)
{
  return buf;
}


double* TactileSensor::writeState(double* out_buf) const
{
  for (size_t i = 0; i < forceData_->size(); i++) {
    Eigen::Map<Vector2>(out_buf) << (*forceData_)[i].first;
    out_buf = out_buf + 2;
    Eigen::Map<Vector3>(out_buf) << (*forceData_)[i].second;
    out_buf = out_buf + 3;
  }
  
  return out_buf;
}

void TactileSensor::setMaxX(double maxX) {maxX_ = maxX;}
void TactileSensor::setMaxY(double maxY) {maxY_ = maxY;}
void TactileSensor::setMinX(double minX) {minX_ = minX;}
void TactileSensor::setMinY(double minY) {minY_ = minY;}
void TactileSensor::setRows(int rows) {rows_ = rows;}
void TactileSensor::setCols(int cols) {cols_ = cols;}

double TactileSensor::maxX() {return maxX_;}
double TactileSensor::maxY() {return maxY_;}
double TactileSensor::minY() {return minY_;}
double TactileSensor::minX() {return minX_;}
int TactileSensor::rows() {return rows_;}
int TactileSensor::cols() {return cols_;}
