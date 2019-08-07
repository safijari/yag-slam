#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include "CustomData.h"
#include "Object.h"

class SensorData : public Object {
public:
  /**
   * Destructor
   */
  virtual ~SensorData();

public:
  /**
   * Gets sensor data id
   * @return sensor id
   */
  inline int32_t GetStateId() const { return m_StateId; }

  /**
   * Sets sensor data id
   * @param stateId id
   */
  inline void SetStateId(int32_t stateId) { m_StateId = stateId; }

  /**
   * Gets sensor unique id
   * @return unique id
   */
  inline int32_t GetUniqueId() const { return m_UniqueId; }

  /**
   * Sets sensor unique id
   * @param uniqueId
   */
  inline void SetUniqueId(uint32_t uniqueId) { m_UniqueId = uniqueId; }

  /**
   * Gets sensor data time
   * @return time
   */
  inline double GetTime() const { return m_Time; }

  /**
   * Sets sensor data time
   * @param time
   */
  inline void SetTime(double time) { m_Time = time; }

  /**
   * Get the sensor that created this sensor data
   * @return sensor
   */
  inline const Name &GetSensorName() const { return m_SensorName; }

  /**
   * Add a CustomData object to sensor data
   * @param pCustomData
   */
  inline void AddCustomData(CustomData *pCustomData) {
    m_CustomData.push_back(pCustomData);
  }

  /**
   * Get all custom data objects assigned to sensor data
   * @return CustomDataVector&
   */
  inline const CustomDataVector &GetCustomData() const { return m_CustomData; }

protected:
  /**
   * Construct a SensorData object with a sensor name
   */
  SensorData(const Name &rSensorName);

private:
  /**
   * Restrict the copy constructor
   */
  SensorData(const SensorData &);

  /**
   * Restrict the assignment operator
   */
  const SensorData &operator=(const SensorData &);

private:
  /**
   * ID unique to individual sensor
   */
  int32_t m_StateId;

  /**
   * ID unique across all sensor data
   */
  int32_t m_UniqueId;

  /**
   * Sensor that created this sensor data
   */
  Name m_SensorName;

  /**
   * Time the sensor data was created
   */
  double m_Time;

  CustomDataVector m_CustomData;
};

#endif
