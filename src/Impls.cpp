#include "SensorData.h"

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

SensorData::SensorData(const Name &rSensorName)
  : Object(), m_StateId(-1), m_UniqueId(-1), m_SensorName(rSensorName),
    m_Time(0.0) {}

SensorData::~SensorData() {
  for (auto iter : m_CustomData) { delete iter; }

  m_CustomData.clear();
}
#include "Object.h"

Object::Object() {}

Object::Object(const Name &rName) : m_Name(rName) {}

Object::~Object() {}
