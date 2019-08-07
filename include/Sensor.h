#ifndef SENSOR_H
#define SENSOR_H

#include "Object.h"
#include "Pose.h"
#include "SensorData.h"

class Sensor : public Object {
protected:
  /**
   * Construct a Sensor
   * @param rName sensor name
   */
  Sensor(const Name &rName);

public:
  /**
   * Destructor
   */
  virtual ~Sensor();

public:
  /**
   * Gets this range finder sensor's offset
   * @return offset pose
   */
  inline Pose2 GetOffsetPose() { return m_pOffsetPose; }

  /**
   * Sets this range finder sensor's offset
   * @param rPose
   */
  inline void SetOffsetPose(Pose2 &rPose) { m_pOffsetPose = rPose; }

  /**
   * Validates sensor
   * @return true if valid
   */
  virtual bool Validate() = 0;

  /**
   * Validates sensor data
   * @param pSensorData sensor data
   * @return true if valid
   */
  virtual bool Validate(SensorData *pSensorData) = 0;

private:
  /**
   * Restrict the copy constructor
   */
  Sensor(const Sensor &);

  /**
   * Restrict the assignment operator
   */
  const Sensor &operator=(const Sensor &);

private:
  /**
   * Sensor offset pose
   */
  Pose2 m_pOffsetPose;
}; // Sensor

/**
 * Type declaration of Sensor vector
 */
typedef std::vector<Sensor *> SensorVector;

#endif
