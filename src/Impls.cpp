#include "LocalizedRangeScanAndFinder.h"
#include "AdditionalMath.h"

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

const PointVectorDouble
LaserRangeFinder::GetPointReadings(LocalizedRangeScan *pLocalizedRangeScan,
                                   CoordinateConverter *pCoordinateConverter,
                                   bool ignoreThresholdPoints,
                                   bool flipY) const {
  PointVectorDouble pointReadings;

  Pose2 scanPose = pLocalizedRangeScan->GetSensorPose();

  // compute point readings
  int32_t beamNum = 0;
  double *pRangeReadings = pLocalizedRangeScan->GetRangeReadings();
  for (int32_t i = 0; i < m_NumberOfRangeReadings; i++, beamNum++) {
    double rangeReading = pRangeReadings[i];

    if (ignoreThresholdPoints) {
      if (!amath::InRange(rangeReading, GetMinimumRange(),
                          GetRangeThreshold())) {
        continue;
      }
    } else {
      rangeReading =
          amath::Clip(rangeReading, GetMinimumRange(), GetRangeThreshold());
    }

    double angle = scanPose.GetHeading() + GetMinimumAngle() +
                   beamNum * GetAngularResolution();

    Vector2<double> point;

    point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
    point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

    if (pCoordinateConverter != NULL) {
      Vector2<int32_t> gridPoint =
          pCoordinateConverter->WorldToGrid(point, flipY);
      point.SetX(gridPoint.GetX());
      point.SetY(gridPoint.GetY());
    }

    pointReadings.push_back(point);
  }

  return pointReadings;
}

bool LaserRangeFinder::Validate(SensorData *pSensorData) {
  LaserRangeScan *pLaserRangeScan = dynamic_cast<LaserRangeScan *>(pSensorData);

  // verify number of range readings in LaserRangeScan matches the number of
  // expected range readings
  if (pLaserRangeScan->GetNumberOfRangeReadings() !=
      GetNumberOfRangeReadings()) {
    std::cout << "LaserRangeScan contains "
              << pLaserRangeScan->GetNumberOfRangeReadings()
              << " range readings, expected " << GetNumberOfRangeReadings()
              << std::endl;
    return false;
  }

  return true;
}

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

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

Sensor::Sensor(const Name &rName) : Object(rName) {
  m_pOffsetPose = Pose2();
}

Sensor::~Sensor() {}




Object::Object() {}

Object::Object(const Name &rName) : m_Name(rName) {}

Object::~Object() {}



SensorManager *SensorManager::GetInstance() {
  static Singleton<SensorManager> sInstance;
  return sInstance.Get();
}
