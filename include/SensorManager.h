#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <iostream>
#include <map>

#include "Name.h"
#include "Sensor.h"
#include "Singleton.h"
/**
 * Type declaration of <Name, Sensor*> map
 */
typedef std::map<Name, Sensor *> SensorManagerMap;

/**
 * Manages sensors
 */
class SensorManager {
public:
  /**
   * Constructor
   */
  SensorManager() {}

  /**
   * Destructor
   */
  virtual ~SensorManager() {}

public:
  /**
   * Get singleton instance of SensorManager
   */
  static SensorManager *GetInstance();

public:
  /**
   * Registers a sensor by it's name. The Sensor name must be unique, if not
   * sensor is not registered unless override is set to true
   * @param pSensor sensor to register
   * @param override
   * @return true if sensor is registered with SensorManager, false if Sensor
   * name is not unique
   */
  void RegisterSensor(Sensor *pSensor, bool override = false) {
    Validate(pSensor);

    if ((m_Sensors.find(pSensor->GetName()) != m_Sensors.end()) && !override) {
      throw "Cannot register sensor: already registered: [" +
          pSensor->GetName().ToString() +
          "] (Consider setting 'override' to true)";
    }

    std::cout << "Registering sensor: [" << pSensor->GetName().ToString() << "]"
              << std::endl;

    m_Sensors[pSensor->GetName()] = pSensor;
  }

  /**
   * Unregisters the given sensor
   * @param pSensor sensor to unregister
   */
  void UnregisterSensor(Sensor *pSensor) {
    Validate(pSensor);

    if (m_Sensors.find(pSensor->GetName()) != m_Sensors.end()) {
      std::cout << "Unregistering sensor: " << pSensor->GetName().ToString()
                << std::endl;

      m_Sensors.erase(pSensor->GetName());
    } else {
      throw "Cannot unregister sensor: not registered: [" +
          pSensor->GetName().ToString() + "]";
    }
  }

  /**
   * Gets the sensor with the given name
   * @param rName name of sensor
   * @return sensor
   */
  Sensor *GetSensorByName(const Name &rName) {
    if (m_Sensors.find(rName) != m_Sensors.end()) {
      return m_Sensors[rName];
    }

    throw "Sensor not registered: [" + rName.ToString() +
        "] (Did you add the sensor to the Dataset?)";
  }

  /**
   * Gets the sensor with the given name
   * @param rName name of sensor
   * @return sensor
   */
  template <class T> T *GetSensorByName(const Name &rName) {
    Sensor *pSensor = GetSensorByName(rName);

    return dynamic_cast<T *>(pSensor);
  }

  /**
   * Gets all registered sensors
   * @return vector of all registered sensors
   */
  SensorVector GetAllSensors() {
    SensorVector sensors;

    for (auto iter : m_Sensors) {
      sensors.push_back(iter.second);
    }

    return sensors;
  }

protected:
  /**
   * Checks that given sensor is not NULL and has non-empty name
   * @param pSensor sensor to validate
   */
  static void Validate(Sensor *pSensor) {
    if (pSensor == NULL) {
      throw "Invalid sensor:  NULL";
    } else if (pSensor->GetName().ToString() == "") {
      throw "Invalid sensor:  nameless";
    }
  }

protected:
  /**
   * Sensor map
   */
  SensorManagerMap m_Sensors;
};

#endif
