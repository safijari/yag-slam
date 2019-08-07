#ifndef LOCALIZED_RANGE_SCAN_H
#define LOCALIZED_RANGE_SCAN_H

#include <boost/thread.hpp>
#include <shared_mutex>

#include "LocalizedRangeScanAndFinder.h"
#include "Transform.h"
#include "Vector2.h"
#include "AdditionalMath.h"
#include "CoordinateConverter.h"
#include "Sensor.h"
#include "AdditionalMath.h"

#include <assert.h>
#include <stdint.h>
#include <vector>

#include "Name.h"
#include "SensorData.h"
#include "SensorManager.h"
#include "LocalizedRangeScanAndFinder.h"

/**
 * Type declaration of range readings vector
 */
typedef std::vector<double> RangeReadingsVector;

class LaserRangeFinder;

/**
 * LaserRangeScan representing the range readings from a laser range finder
 * sensor.
 */
class LaserRangeScan : public SensorData {
public:
  /**
   * Constructs a scan from the given sensor with the given readings
   * @param rSensorName
   */
  LaserRangeScan(const Name &rSensorName)
      : SensorData(rSensorName), m_pRangeReadings(NULL),
        m_NumberOfRangeReadings(0) {}

  /**
   * Constructs a scan from the given sensor with the given readings
   * @param rSensorName
   * @param rRangeReadings
   */
  LaserRangeScan(const Name &rSensorName,
                 const RangeReadingsVector &rRangeReadings)
      : SensorData(rSensorName), m_pRangeReadings(NULL),
        m_NumberOfRangeReadings(0) {
    assert(rSensorName.ToString() != "");

    SetRangeReadings(rRangeReadings);
  }

  /**
   * Destructor
   */
  virtual ~LaserRangeScan() { delete[] m_pRangeReadings; }

public:
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
   * Gets the laser range finder sensor that generated this scan
   * @return laser range finder sensor of this scan
   */
  inline LaserRangeFinder *GetLaserRangeFinder() const {
    return SensorManager::GetInstance()->GetSensorByName<LaserRangeFinder>(
        GetSensorName());
  }

  /**
   * Gets the number of range readings
   * @return number of range readings
   */
  inline int32_t GetNumberOfRangeReadings() const {
    return m_NumberOfRangeReadings;
  }

private:
  LaserRangeScan(const LaserRangeScan &);
  const LaserRangeScan &operator=(const LaserRangeScan &);

private:
  double *m_pRangeReadings;
  int32_t m_NumberOfRangeReadings;
}; // LaserRangeScan

/**
 * The LaserRangeFinder defines a laser sensor that provides the pose offset
 * position of a localized range scan relative to the robot. The user can set an
 * offset pose for the sensor relative to the robot coordinate system. If no
 * value is provided by the user, the sensor is set to be at the origin of the
 * robot coordinate system. The LaserRangeFinder contains parameters for
 * physical laser sensor used by the mapper for scan matching Also contains
 * information about the maximum range of the sensor and provides a threshold
 * for limiting the range of readings.
 * The optimal value for the range threshold depends on the angular resolution
 * of the scan and the desired map resolution.  RangeThreshold should be set as
 * large as possible while still providing "solid" coverage between consecutive
 * range readings.  The diagram below illustrates the relationship between map
 * resolution and the range threshold.
 */

class LocalizedRangeScan;

class LaserRangeFinder : public Sensor {
public:
  /**
   * Destructor
   */
  virtual ~LaserRangeFinder() {}

public:
  /**
   * Gets this range finder sensor's minimum range
   * @return minimum range
   */
  inline double GetMinimumRange() const { return m_pMinimumRange; }

  /**
   * Sets this range finder sensor's minimum range
   * @param minimumRange
   */
  inline void SetMinimumRange(double minimumRange) {
    m_pMinimumRange = minimumRange;

    SetRangeThreshold(GetRangeThreshold());
  }

  /**
   * Gets this range finder sensor's maximum range
   * @return maximum range
   */
  inline double GetMaximumRange() { return m_pMaximumRange; }

  /**
   * Sets this range finder sensor's maximum range
   * @param maximumRange
   */
  inline void SetMaximumRange(double maximumRange) {
    m_pMaximumRange = maximumRange;

    SetRangeThreshold(GetRangeThreshold());
  }

  /**
   * Gets the range threshold
   * @return range threshold
   */
  inline double GetRangeThreshold() const { return m_pRangeThreshold; }

  /**
   * Sets the range threshold
   * @param rangeThreshold
   */
  inline void SetRangeThreshold(double rangeThreshold) {
    // make sure rangeThreshold is within laser range finder range
    m_pRangeThreshold =
        amath::Clip(rangeThreshold, GetMinimumRange(), GetMaximumRange());

    if (amath::DoubleEqual(GetRangeThreshold(), rangeThreshold) == false) {
      std::cout << "Info: clipped range threshold to be within minimum and "
                   "maximum range!"
                << std::endl;
    }
  }

  /**
   * Gets this range finder sensor's minimum angle
   * @return minimum angle
   */
  inline double GetMinimumAngle() const { return m_pMinimumAngle; }

  /**
   * Sets this range finder sensor's minimum angle
   * @param minimumAngle
   */
  inline void SetMinimumAngle(double minimumAngle) {
    m_pMinimumAngle = minimumAngle;

    Update();
  }

  /**
   * Gets this range finder sensor's maximum angle
   * @return maximum angle
   */
  inline double GetMaximumAngle() const { return m_pMaximumAngle; }

  /**
   * Sets this range finder sensor's maximum angle
   * @param maximumAngle
   */
  inline void SetMaximumAngle(double maximumAngle) {
    m_pMaximumAngle = maximumAngle;

    Update();
  }

  /**
   * Gets this range finder sensor's angular resolution
   * @return angular resolution
   */
  inline double GetAngularResolution() const { return m_pAngularResolution; }

  /**
   * Sets this range finder sensor's angular resolution
   * @param angularResolution
   */
  inline void SetAngularResolution(double angularResolution) {
    m_pAngularResolution = angularResolution;
    Update();
  }

  /**
   * Gets the number of range readings each localized range scan must contain to
   * be a valid scan.
   * @return number of range readings
   */
  inline uint32_t GetNumberOfRangeReadings() const {
    return m_NumberOfRangeReadings;
  }

  virtual bool Validate() {
    Update();

    if (amath::InRange(GetRangeThreshold(), GetMinimumRange(),
                       GetMaximumRange()) == false) {
      std::cout << "Please set range threshold to a value between ["
                << GetMinimumRange() << ";" << GetMaximumRange() << "]"
                << std::endl;
      return false;
    }

    return true;
  }

  virtual bool Validate(SensorData *pSensorData);

  /**
   * Get point readings (potentially scale readings if given coordinate
   * converter is not null)
   * @param pLocalizedRangeScan
   * @param pCoordinateConverter
   * @param ignoreThresholdPoints
   * @param flipY
   */
  const PointVectorDouble
  GetPointReadings(LocalizedRangeScan *pLocalizedRangeScan,
                   CoordinateConverter *pCoordinateConverter,
                   bool ignoreThresholdPoints = true, bool flipY = false) const;

public:
  /**
   * Create a laser range finder of the given type and ID
   * @param type
   * @param rName name of sensor - if no name is specified default name will be
   * assigned
   * @return laser range finder
   */
  LaserRangeFinder(const Name &rName)
      : Sensor(rName), m_pMinimumRange(0.0), m_pMaximumRange(80.0),
        m_pMinimumAngle(-KT_PI_2), m_pMaximumAngle(KT_PI_2),
        m_pAngularResolution(amath::DegreesToRadians(1)),
        m_pRangeThreshold(12.0), m_NumberOfRangeReadings(0) {}

  /**
   * Set the number of range readings based on the minimum and
   * maximum angles of the sensor and the angular resolution
   */
  void Update() {
    m_NumberOfRangeReadings = static_cast<uint32_t>(
        amath::Round((GetMaximumAngle() - GetMinimumAngle()) /
                     GetAngularResolution()) +
        1);
  }

private:
  LaserRangeFinder(const LaserRangeFinder &);
  const LaserRangeFinder &operator=(const LaserRangeFinder &);

private:
  // sensor m_Parameters
  double m_pMinimumAngle;
  double m_pMaximumAngle;

  double m_pAngularResolution;

  double m_pMinimumRange;
  double m_pMaximumRange;

  double m_pRangeThreshold;

  uint32_t m_NumberOfRangeReadings;

  // static std::string LaserRangeFinderTypeNames[6];
}; // LaserRangeFinder


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
  LocalizedRangeScan(const Name &rSensorName,
                     const RangeReadingsVector &rReadings)
      : LaserRangeScan(rSensorName, rReadings), m_IsDirty(true) {}

  /**
   * Destructor
   */
  virtual ~LocalizedRangeScan() {}

private:
  mutable boost::shared_mutex m_Lock;

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
    boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan *>(this)->Update();
    }

    return m_BarycenterPose;
  }

  /**
   * Gets barycenter if the given parameter is true, otherwise returns the
   * scanner pose
   * @param useBarycenter
   * @return barycenter if parameter is true, otherwise scanner pose
   */
  inline Pose2 GetReferencePose(bool useBarycenter) const {
    boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan *>(this)->Update();
    }

    return useBarycenter ? GetBarycenterPose() : GetSensorPose();
  }

  /**
   * Computes the position of the sensor
   * @return scan pose
   */
  inline Pose2 GetSensorPose() const { return GetSensorAt(m_CorrectedPose); }

  /**
   * Computes the robot pose given the corrected scan pose
   * @param rScanPose pose of the sensor
   */
  void SetSensorPose(const Pose2 &rScanPose) {
    Pose2 deviceOffsetPose2 = GetLaserRangeFinder()->GetOffsetPose();
    double offsetLength = deviceOffsetPose2.GetPosition().Length();
    double offsetHeading = deviceOffsetPose2.GetHeading();
    double angleoffset =
        atan2(deviceOffsetPose2.GetY(), deviceOffsetPose2.GetX());
    double correctedHeading = amath::NormalizeAngle(rScanPose.GetHeading());
    Pose2 worldSensorOffset = Pose2(
        offsetLength * cos(correctedHeading + angleoffset - offsetHeading),
        offsetLength * sin(correctedHeading + angleoffset - offsetHeading),
        offsetHeading);

    m_CorrectedPose = rScanPose - worldSensorOffset;

    Update();
  }

  /**
   * Computes the position of the sensor if the robot were at the given pose
   * @param rPose
   * @return sensor pose
   */
  inline Pose2 GetSensorAt(const Pose2 &rPose) const {
    return Transform(rPose).TransformPose(
        GetLaserRangeFinder()->GetOffsetPose());
  }

  /**
   * Gets the bounding box of this scan
   * @return bounding box of this scan
   */
  inline const BoundingBox2 &GetBoundingBox() const {
    boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
      const_cast<LocalizedRangeScan *>(this)->Update();
    }

    return m_BoundingBox;
  }

  /**
   * Get point readings in local coordinates
   */
  inline const PointVectorDouble &
  GetPointReadings(bool wantFiltered = false) const {
    boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    if (m_IsDirty) {
      // throw away constness and do an update!
      lock.unlock();
      boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
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
    LaserRangeFinder *pLaserRangeFinder = GetLaserRangeFinder();

    if (pLaserRangeFinder != NULL) {
      m_PointReadings.clear();
      m_UnfilteredPointReadings.clear();

      double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();
      double minimumAngle = pLaserRangeFinder->GetMinimumAngle();
      double angularResolution = pLaserRangeFinder->GetAngularResolution();
      Pose2 scanPose = GetSensorPose();

      // compute point readings
      Vector2<double> rangePointsSum;
      uint32_t beamNum = 0;
      for (uint32_t i = 0; i < pLaserRangeFinder->GetNumberOfRangeReadings();
           i++, beamNum++) {
        double rangeReading = GetRangeReadings()[i];
        if (!amath::InRange(rangeReading, pLaserRangeFinder->GetMinimumRange(),
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

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * The LocalizedRangeScanWithPoints is an extension of the LocalizedRangeScan
 * with precomputed points.
 */
class LocalizedRangeScanWithPoints : public LocalizedRangeScan {
public:
  /**
   * Constructs a range scan from the given range finder with the given
   * readings. Precomptued points should be in the robot frame.
   */
  LocalizedRangeScanWithPoints(const Name &rSensorName,
                               const RangeReadingsVector &rReadings,
                               const PointVectorDouble &rPoints)
      : m_Points(rPoints), LocalizedRangeScan(rSensorName, rReadings) {}

  /**
   * Destructor
   */
  virtual ~LocalizedRangeScanWithPoints() {}

private:
  /**
   * Update the points based on the latest sensor pose.
   */
  void Update() {
    m_PointReadings.clear();
    m_UnfilteredPointReadings.clear();

    Pose2 scanPose = GetSensorPose();
    Pose2 robotPose = GetCorrectedPose();

    // update point readings
    Vector2<double> rangePointsSum;
    for (uint32_t i = 0; i < m_Points.size(); i++) {
      // check if this has a NaN
      if (!std::isfinite(m_Points[i].GetX()) ||
          !std::isfinite(m_Points[i].GetY())) {
        Vector2<double> point(m_Points[i].GetX(), m_Points[i].GetY());
        m_UnfilteredPointReadings.push_back(point);

        continue;
      }

      // transform into world coords
      Pose2 pointPose(m_Points[i].GetX(), m_Points[i].GetY(), 0);
      Pose2 result = Transform(robotPose).TransformPose(pointPose);
      Vector2<double> point(result.GetX(), result.GetY());

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

    m_IsDirty = false;
  }

private:
  LocalizedRangeScanWithPoints(const LocalizedRangeScanWithPoints &);
  const LocalizedRangeScanWithPoints &
  operator=(const LocalizedRangeScanWithPoints &);

private:
  const PointVectorDouble m_Points;
}; // LocalizedRangeScanWithPoints

#endif
