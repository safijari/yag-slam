#ifndef LOCALIZED_RANGE_SCAN_H
#define LOCALIZED_RANGE_SCAN_H

#include <thread>
#include <shared_mutex>
#include <mutex>

#include "Transform.h"
#include "Vector2.h"
#include "AdditionalMath.h"
#include "CoordinateConverter.h"
#include "AdditionalMath.h"

#include <assert.h>
#include <stdint.h>
#include <vector>

#include "Name.h"
#include "SensorData.h"
#include "LocalizedRangeScan.h"

/**
 * Type declaration of range readings vector
 */
typedef std::vector<double> RangeReadingsVector;

struct LaserScanConfig {
  double minAngle;
  double maxAngle;
  double angularResolution;
  double minRange;
  double maxRange;
  double rangeThreshold;
  std::string sensorName;
};

/**
 * LaserRangeScan representing the range readings from a laser range finder
 * sensor.
 */
class LaserRangeScan : public SensorData {
public:
  /**
   * Constructs a scan from the given sensor with the given readings
   * @param rSensorName
   * @param rRangeReadings
   */
  LaserRangeScan(const LaserScanConfig &_config,
                 const RangeReadingsVector &rRangeReadings)
    : SensorData(_config.sensorName), config(_config), m_pRangeReadings(NULL),
        m_NumberOfRangeReadings(0) {

    SetRangeReadings(rRangeReadings);
  }

  /**
   * Destructor
   */
  virtual ~LaserRangeScan() { delete[] m_pRangeReadings; }

public:
  LaserScanConfig config;
  /**
   * Gets the range readings of this scan
   * @return range readings of this scan
   */
  inline double *GetRangeReadings() const { return m_pRangeReadings; }

  inline RangeReadingsVector GetRangeReadingsVector() const {
    return RangeReadingsVector(m_pRangeReadings,
                               m_pRangeReadings + m_NumberOfRangeReadings);
  }

  /**
   * Sets the range readings for this scan
   * @param rRangeReadings
   */
  inline void SetRangeReadings(const RangeReadingsVector &rRangeReadings) {
    // ignore for now! XXXMAE BUGUBUG 05/21/2010 << TODO(lucbettaieb): What the
    // heck is this?? if (rRangeReadings.size() != GetNumberOfRangeReadings())
    // {
    //   std::stringstream error;
    //   error << "Given number of readings (" << rRangeReadings.size()
    //         << ") does not match expected number of range finder (" <<
    //         GetNumberOfRangeReadings() << ")";
    //   throw Exception(error.str());
    // }

    if (!rRangeReadings.empty()) {
      if (rRangeReadings.size() != m_NumberOfRangeReadings) {
        // delete old readings
        delete[] m_pRangeReadings;

        // store size of array!
        m_NumberOfRangeReadings = static_cast<int32_t>(rRangeReadings.size());

        // allocate range readings
        m_pRangeReadings = new double[m_NumberOfRangeReadings];
      }

      // copy readings
      int32_t index = 0;
      for (auto iter : rRangeReadings) {
        m_pRangeReadings[index++] = iter;
      }

    } else {
      delete[] m_pRangeReadings;
      m_pRangeReadings = NULL;
    }
  }

  /**
   * Gets the number of range readings
   * @return number of range readings
   */
  inline int32_t GetNumberOfRangeReadings() const {
    return m_NumberOfRangeReadings;
  }

  /**
   * Gets this range finder sensor's minimum range
   * @return minimum range
   */
  inline double GetMinimumRange() const { return config.minRange; }

  /**
   * Gets this range finder sensor's maximum range
   * @return maximum range
   */
  inline double GetMaximumRange() { return config.maxRange; }

  /**
   * Gets the range threshold
   * @return range threshold
   */
  inline double GetRangeThreshold() const { return config.rangeThreshold; }

  /**
   * Gets this range finder sensor's minimum angle
   * @return minimum angle
   */
  inline double GetMinimumAngle() const { return config.minAngle; }

  /**
   * Gets this range finder sensor's maximum angle
   * @return maximum angle
   */
  inline double GetMaximumAngle() const { return config.maxAngle; }

  /**
   * Gets this range finder sensor's angular resolution
   * @return angular resolution
   */
  inline double GetAngularResolution() const { return config.angularResolution; }

private:
  LaserRangeScan(const LaserRangeScan &);
  const LaserRangeScan &operator=(const LaserRangeScan &);

private:
  double *m_pRangeReadings;
  int32_t m_NumberOfRangeReadings;
}; // LaserRangeScan

/**
 * The LocalizedRangeScan contains range data from a single sweep of a laser
 * range finder sensor in a two-dimensional space and position information. The
 * odometer position is the position reported by the robot when the range data
 * was recorded. The corrected position is the position calculated by the mapper
 * (or localizer)
 */
class LocalizedRangeScan : public LaserRangeScan {
public:
  /**
   * Constructs a range scan from the given range finder with the given
   * readings
   */
  LocalizedRangeScan(const LaserScanConfig &_config,
                     const RangeReadingsVector &rReadings,
                     Pose2 _odomPose, Pose2 _correctedPose, uint32_t _uniqueId, double _scanTime)
      : LaserRangeScan(_config, rReadings), m_IsDirty(true) {
    SetOdometricPose(_odomPose);
    SetCorrectedPose(_correctedPose);
    SetUniqueId(_uniqueId);
    SetTime(_scanTime);
  }

  /**
   * Destructor
   */
  virtual ~LocalizedRangeScan() {}

private:
  mutable std::shared_timed_mutex m_Lock;

public:
  /**
   * Gets the odometric pose of this scan
   * @return odometric pose of this scan
   */
  inline const Pose2 &GetOdometricPose() const { return m_OdometricPose; }

  /**
   * Sets the odometric pose of this scan
   * @param rPose
   */
  inline void SetOdometricPose(const Pose2 &rPose) { m_OdometricPose = rPose; }

  /**
   * Gets the (possibly corrected) robot pose at which this scan was taken.  The
   * corrected robot pose of the scan is usually set by an external module such
   * as a localization or mapping module when it is determined that the original
   * pose was incorrect.  The external module will set the correct pose based on
   * additional sensor data and any context information it has.  If the pose has
   * not been corrected, a call to this method returns the same pose as
   * GetOdometricPose().
   * @return corrected pose
   */
  inline const Pose2 &GetCorrectedPose() const { return m_CorrectedPose; }

  /**
   * Moves the scan by moving the robot pose to the given location.
   * @param rPose new pose of the robot of this scan
   */
  inline void SetCorrectedPose(const Pose2 &rPose) {
    m_CorrectedPose = rPose;

    m_IsDirty = true;
  }

  /**
   * Gets barycenter of point readings
   */
  inline const Pose2 &GetBarycenterPose() const {
    std::shared_lock<std::shared_timed_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      std::unique_lock<std::shared_timed_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan *>(this)->Update();
    }

    return m_BarycenterPose;
  }

  /**
   * Gets the bounding box of this scan
   * @return bounding box of this scan
   */
  inline const BoundingBox2 &GetBoundingBox() const {
    std::shared_lock<std::shared_timed_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      std::unique_lock<std::shared_timed_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan *>(this)->Update();
    }

    return m_BoundingBox;
  }

  /**
   * Get point readings in local coordinates
   */
  inline const PointVectorDouble &
  GetPointReadings(bool wantFiltered = false) const {
    std::shared_lock<std::shared_timed_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      std::unique_lock<std::shared_timed_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan *>(this)->Update();
    }

    if (wantFiltered == true) {
      return m_PointReadings;
    } else {
      return m_UnfilteredPointReadings;
    }
  }

private:
  /**
   * Compute point readings based on range readings
   * Only range readings within [minimum range; range threshold] are returned
   */
  virtual void Update() {
    if (true) {
      m_PointReadings.clear();
      m_UnfilteredPointReadings.clear();

      double rangeThreshold = this->GetRangeThreshold();
      double minimumAngle = this->GetMinimumAngle();
      double angularResolution = this->GetAngularResolution();
      Pose2 scanPose = GetCorrectedPose();

      // compute point readings
      Vector2<double> rangePointsSum;
      uint32_t beamNum = 0;
      for (uint32_t i = 0; i < this->GetNumberOfRangeReadings();
           i++, beamNum++) {
        double rangeReading = GetRangeReadings()[i];
        if (!amath::InRange(rangeReading, this->GetMinimumRange(),
                            rangeThreshold)) {
          double angle = scanPose.GetHeading() + minimumAngle +
                         beamNum * angularResolution;

          Vector2<double> point;
          point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
          point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

          m_UnfilteredPointReadings.push_back(point);
          continue;
        }

        double angle =
            scanPose.GetHeading() + minimumAngle + beamNum * angularResolution;

        Vector2<double> point;
        point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
        point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

        m_PointReadings.push_back(point);
        m_UnfilteredPointReadings.push_back(point);

        rangePointsSum += point;
      }

      // compute barycenter
      double nPoints = static_cast<double>(m_PointReadings.size());
      if (nPoints != 0.0) {
        Vector2<double> averagePosition =
            Vector2<double>(rangePointsSum / nPoints);
        m_BarycenterPose = Pose2(averagePosition, 0.0);
      } else {
        m_BarycenterPose = scanPose;
      }

      // calculate bounding box of scan
      m_BoundingBox = BoundingBox2();
      m_BoundingBox.Add(scanPose.GetPosition());
      for (auto iter : m_PointReadings) {
        m_BoundingBox.Add(iter);
      }
    }

    m_IsDirty = false;
  }

private:
  LocalizedRangeScan(const LocalizedRangeScan &);
  const LocalizedRangeScan &operator=(const LocalizedRangeScan &);

private:
  /**
   * Odometric pose of robot
   */
  Pose2 m_OdometricPose;

  /**
   * Corrected pose of robot calculated by mapper (or localizer)
   */
  Pose2 m_CorrectedPose;

protected:
  /**
   * Average of all the point readings
   */
  Pose2 m_BarycenterPose;

  /**
   * Vector of point readings
   */
  PointVectorDouble m_PointReadings;

  /**
   * Vector of unfiltered point readings
   */
  PointVectorDouble m_UnfilteredPointReadings;

  /**
   * Bounding box of localized range scan
   */
  BoundingBox2 m_BoundingBox;

  /**
   * Internal flag used to update point readings, barycenter and bounding box
   */
  bool m_IsDirty;
}; // LocalizedRangeScan

/**
 * Type declaration of LocalizedRangeScan vector
 */
typedef std::vector<LocalizedRangeScan *> LocalizedRangeScanVector;

#endif
