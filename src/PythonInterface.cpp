#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>

#include "LocalizedRangeScanAndFinder.h"
#include "ScanMatcher.h"
#include "SensorManager.h"

struct MatchResult {
  Pose2 best_pose;
  Matrix3 covariance;
  double response;
};

class Wrapper {
  LaserRangeFinder *rangeFinder;
  Name name;
  ScanMatcher *matcher;
  // SensorManager *sensor_manager;

public:
  Wrapper(std::string sensorName, double angularResolution, double angleMin,
          double angleMax, std::shared_ptr<ScanMatcherConfig> config) {

    this->name = Name(sensorName);
    this->rangeFinder = new LaserRangeFinder(this->name);
    this->rangeFinder->SetAngularResolution(angularResolution);
    this->rangeFinder->SetMinimumAngle(angleMin);
    this->rangeFinder->SetMaximumAngle(angleMax);
    this->rangeFinder->Update();
    SensorManager::GetInstance()->RegisterSensor(this->rangeFinder);
    this->matcher = ScanMatcher::Create(config);
  }

  LaserRangeFinder *getRangeFinder() { return this->rangeFinder; }

  Name getName() { return this->name; }

  bool ProcessLocalizedRangeScan(std::vector<double> ranges, double x, double y,
                                 double heading) {
    auto scan = new LocalizedRangeScan(this->name, ranges);
    scan->SetOdometricPose(Pose2(x, y, heading));
    scan->SetCorrectedPose(Pose2(x, y, heading));

    return true;
  }

  LocalizedRangeScan *MakeScan(std::vector<double> ranges, double x, double y,
                               double yaw) {
    auto scan = new LocalizedRangeScan(this->name, ranges);
    scan->SetOdometricPose(Pose2(x, y, yaw));
    scan->SetCorrectedPose(Pose2(x, y, yaw));

    return scan;
  }

  MatchResult MatchScan(LocalizedRangeScan *query,
                        const LocalizedRangeScanVector &base) {
    Pose2 mean;
    Matrix3 covariance;

    auto ret = this->matcher->MatchScan(query, base, mean, covariance);

    return MatchResult{mean, covariance, ret};
  }

  ~Wrapper() {
    delete this->rangeFinder;
    delete this->matcher;
  }
};

namespace py = pybind11;

PYBIND11_MODULE(mp_slam_cpp, m) {
  py::class_<Pose2>(m, "Pose2")
      .def(py::init<double, double, double>())
      .def_property("x", &Pose2::GetX, &Pose2::SetX)
      .def_property("y", &Pose2::GetY, &Pose2::SetY)
      .def_property("yaw", &Pose2::GetHeading, &Pose2::SetHeading)
      .def("__repr__", [](const Pose2 &a) {
        std::stringstream buffer;
        buffer << "(x: " << a.GetX() << ", y: " << a.GetY()
               << ", heading: " << a.GetHeading() << ")\n";
        return buffer.str();
      });

  py::class_<Vector2<double>>(m, "Vec2_d")
      .def(py::init<double, double>())
      .def_property("x", &Vector2<double>::GetX, &Vector2<double>::SetX)
      .def_property("y", &Vector2<double>::GetY, &Vector2<double>::SetY)
      .def("__repr__", [](const Vector2<double> &a) {
        std::stringstream buffer;
        buffer << "(x: " << a.GetX() << ", y:" << a.GetY() << ")\n";
        return buffer.str();
      });

  py::class_<Name>(m, "Name").def(py::init<const std::string &>());

  py::class_<LaserRangeFinder>(m, "LaserRangeFinder")
      .def(py::init<std::string &>())
      .def("set_offset_pose", &LaserRangeFinder::SetOffsetPose)
      .def("set_angular_resolution", &LaserRangeFinder::SetAngularResolution)
      .def("set_minimum_range", &LaserRangeFinder::SetMinimumRange)
      .def("set_minimum_angle", &LaserRangeFinder::SetMinimumAngle)
      .def("set_maximum_range", &LaserRangeFinder::SetMaximumRange)
      .def("set_maximum_angle", &LaserRangeFinder::SetMaximumAngle)
      .def("set_angular_resolution", &LaserRangeFinder::SetAngularResolution)
      .def("set_range_threshold", &LaserRangeFinder::SetRangeThreshold);

  py::class_<LocalizedRangeScan>(m, "LocalizedRangeScan")
      .def(py::init<Name, std::vector<double>>())
      .def("set_odometric_pose", &LocalizedRangeScan::SetOdometricPose)
      .def("get_odometric_pose", &LocalizedRangeScan::GetOdometricPose)
      .def("set_corrected_pose", &LocalizedRangeScan::SetCorrectedPose)
      .def("get_corrected_pose", &LocalizedRangeScan::GetCorrectedPose)
      .def_property("odom_pose", &LocalizedRangeScan::GetOdometricPose,
                    &LocalizedRangeScan::SetOdometricPose)
      .def_property("corrected_pose", &LocalizedRangeScan::GetCorrectedPose,
                    &LocalizedRangeScan::SetCorrectedPose)
      .def_property_readonly("ranges", &LocalizedRangeScan::GetRangeReadingsVector);

  py::class_<Wrapper>(m, "Wrapper")
      .def(py::init<std::string, double, double, double,
                    std::shared_ptr<ScanMatcherConfig>>())
      .def("process_scan", &Wrapper::ProcessLocalizedRangeScan)
      .def("make_scan", &Wrapper::MakeScan, py::return_value_policy::reference)
      .def("match_scan", &Wrapper::MatchScan,
           py::return_value_policy::reference)
      .def_property_readonly("name", &Wrapper::getName)
      .def_property_readonly("range_finder", &Wrapper::getRangeFinder);

  py::class_<MatchResult>(m, "MatchResult")
      .def_readwrite("best_pose", &MatchResult::best_pose)
      .def_property_readonly("covariance",
                             [](const MatchResult &a) {
                               auto ret = std::vector<std::vector<double>>(3);
                               for (int j = 0; j < 3; j++) {
                                 for (int i = 0; i < 3; i++) {
                                   ret[j].push_back(
                                       a.covariance.m_Matrix[j][i]);
                                 }
                               }

                               return ret;
                             })
      .def_readwrite("response", &MatchResult::response);

  py::class_<ScanMatcherConfig, std::shared_ptr<ScanMatcherConfig>>(
      m, "ScanMatcherConfig")
      .def(py::init<>())
      .def_readwrite("coarse_angle_resolution",
                     &ScanMatcherConfig::m_pCoarseAngleResolution)
      .def_readwrite("coarse_search_angle_offset",
                     &ScanMatcherConfig::m_pCoarseSearchAngleOffset)
      .def_readwrite("fine_search_angle_offset",
                     &ScanMatcherConfig::m_pFineSearchAngleOffset)
      .def_readwrite("distance_variance_penalty",
                     &ScanMatcherConfig::m_pDistanceVariancePenalty)
      .def_readwrite("angle_variance_penalty",
                     &ScanMatcherConfig::m_pAngleVariancePenalty)
      .def_readwrite("minimum_distance_penalty",
                     &ScanMatcherConfig::m_pMinimumDistancePenalty)
      .def_readwrite("minimum_angle_penalty",
                     &ScanMatcherConfig::m_pMinimumAnglePenalty)
      .def_readwrite("use_response_expansion",
                     &ScanMatcherConfig::m_pUseResponseExpansion)
      .def_readwrite("search_size", &ScanMatcherConfig::searchSize)
      .def_readwrite("resolution", &ScanMatcherConfig::resolution)
      .def_readwrite("smear_deviation", &ScanMatcherConfig::smearDeviation)
      .def_readwrite("range_threshold", &ScanMatcherConfig::rangeThreshold);

#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
}