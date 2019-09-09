#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>

#include "LocalizedRangeScan.h"
#include "ScanMatcher.h"

struct MatchResult {
  Pose2 best_pose;
  Matrix3 covariance;
  double response;
};

class Wrapper {
  Name name;
  ScanMatcher *matcher;

public:
  Wrapper(std::shared_ptr<ScanMatcherConfig> config) {

    this->matcher = ScanMatcher::Create(config);
  }

  LocalizedRangeScan *MakeScan(LaserScanConfig config, std::vector<double> ranges, double x, double y,
                               double yaw) {
    auto scan = new LocalizedRangeScan(config, ranges, Pose2(x, y, yaw), Pose2(x, y, yaw), 0, 0.0);
    return scan;
  }

  MatchResult MatchScan(LocalizedRangeScan *query,
                        const LocalizedRangeScanVector &base, bool penalize=true, bool refine=true) {
    Pose2 mean;
    Matrix3 covariance;

    auto ret = this->matcher->MatchScan(query, base, mean, covariance, penalize, refine);

    return MatchResult{mean, covariance, ret};
  }

  ~Wrapper() {
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

  py::class_<LocalizedRangeScan>(m, "LocalizedRangeScan")
    .def(py::init<LaserScanConfig, std::vector<double>, Pose2, Pose2, uint32_t, double>())
    .def_readonly("config", &LocalizedRangeScan::config)
    .def_property_readonly("ranges", &LocalizedRangeScan::GetRangeReadingsVector)
    .def_property("odom_pose", &LocalizedRangeScan::GetOdometricPose,
                  &LocalizedRangeScan::SetOdometricPose)
    .def_property("corrected_pose", &LocalizedRangeScan::GetCorrectedPose,
                  &LocalizedRangeScan::SetCorrectedPose)
    .def_property("num", &LocalizedRangeScan::GetStateId, &LocalizedRangeScan::SetStateId)
    .def_property("time", &LocalizedRangeScan::GetTime, &LocalizedRangeScan::SetTime);

  py::class_<Wrapper>(m, "Wrapper")
      .def(py::init<std::shared_ptr<ScanMatcherConfig>>())
      .def("match_scan", &Wrapper::MatchScan,
           py::return_value_policy::reference);

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

      py::class_<LaserScanConfig, std::shared_ptr<LaserScanConfig>>(m, "LaserScanConfig")
        .def(py::init<double, double, double, double, double, double, std::string>())
        .def_readonly("min_angle", &LaserScanConfig::minAngle)
        .def_readonly("max_angle", &LaserScanConfig::maxAngle)
        .def_readonly("angular_resolution", &LaserScanConfig::angularResolution)
        .def_readonly("min_range", &LaserScanConfig::minRange)
        .def_readonly("min_range", &LaserScanConfig::maxRange)
        .def_readonly("min_angle", &LaserScanConfig::minAngle)
        .def_readonly("range_threshold", &LaserScanConfig::rangeThreshold)
        .def_readonly("sensor_name", &LaserScanConfig::sensorName);

#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
}
