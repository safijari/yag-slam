/*
 * Copyright 2019 Jariullah Safi
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <vector>
#include <iostream>

#include "Grid.h"
#include "LocalizedRangeScan.h"
#include "OccupancyGrid.h"
#include "ScanMatcher.h"

namespace py = pybind11;

struct MatchResult {
  Pose2 best_pose;
  Matrix3 covariance;
  double response;
};

class Wrapper {
  Name name;

public:
  std::shared_ptr<ScanMatcherConfig> config;
  ScanMatcher *matcher;

  Wrapper(std::shared_ptr<ScanMatcherConfig> config) {
    this->config = config;
    this->matcher = ScanMatcher::Create(config);
  }

  MatchResult MatchScan(LocalizedRangeScan *query,
                        const LocalizedRangeScanVector &base,
                        bool penalize = true, bool refine = true) {
    py::gil_scoped_release release;
    Pose2 mean;
    Matrix3 covariance;
    auto ret = this->matcher->MatchScan(query, base, mean, covariance, penalize,
                                        refine);
    return MatchResult{mean, covariance, ret};
  }

  ~Wrapper() { delete this->matcher; }
};

OccupancyGrid *CreateOccupancyGrid(LocalizedRangeScanVector *scans,
                                   double resolution, double rangeThreshold) {
  py::gil_scoped_release release;
  auto pOccupancyGrid = OccupancyGrid::CreateFromScans(*scans, resolution, rangeThreshold);
  return pOccupancyGrid;
}

PYBIND11_MODULE(yag_slam_cpp, m) {
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
      .def_property("y", &Vector2<double>::GetY, &Vector2<double>::SetY);

  py::class_<Name>(m, "Name").def(py::init<const std::string &>());

  py::class_<LocalizedRangeScan>(m, "LocalizedRangeScan")
      .def(py::init<LaserScanConfig, std::vector<double>, Pose2, Pose2,
                    uint32_t, double>())
      .def_readonly("config", &LocalizedRangeScan::config)
      .def_property_readonly("ranges",
                             &LocalizedRangeScan::GetRangeReadingsVector)
      .def_property("odom_pose", &LocalizedRangeScan::GetOdometricPose,
                    &LocalizedRangeScan::SetOdometricPose)
      .def_property("corrected_pose", &LocalizedRangeScan::GetCorrectedPose,
                    &LocalizedRangeScan::SetCorrectedPose)
      .def_property("num", &LocalizedRangeScan::GetUniqueId,
                    &LocalizedRangeScan::SetUniqueId)
      .def_property("time", &LocalizedRangeScan::GetTime,
                    &LocalizedRangeScan::SetTime);

  py::class_<Wrapper>(m, "Wrapper")
      .def(py::init<std::shared_ptr<ScanMatcherConfig>>())
      .def_readonly("config", &Wrapper::config)
      .def("match_scan", &Wrapper::MatchScan,
           py::return_value_policy::reference)
    .def("grid", [](const Wrapper &a_) {
                   auto a = a_.matcher->GetCorrelationGrid();
                   auto roi = a->GetROI();
                   auto h = roi.GetHeight();
                   auto w = roi.GetWidth();
                   auto _r = py::array_t<uint8_t>({h, w});
                   auto r = _r.mutable_unchecked<2>();
                   for (auto j = 0; j < h; j++) {
                     for (auto i = 0; i < w; i++) {
                       auto idx = a->GridIndex(Vector2<int32_t>(i, j));
                       uint8_t *pByte = a->GetDataPointer() + idx;
                       r(j, i) = pByte[0];
                     }
                   }
                   return _r;
                 }, py::return_value_policy::reference)
    ;

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
      .def_readwrite("fine_search_angle_resolution",
                     &ScanMatcherConfig::m_pFineSearchAngleResolution)
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

  m.def("create_occupancy_grid", &CreateOccupancyGrid);

  py::class_<LaserScanConfig, std::shared_ptr<LaserScanConfig>>(
      m, "LaserScanConfig")
      .def(py::init<double, double, double, double, double, double,
                    std::string>())
      .def_readonly("min_angle", &LaserScanConfig::minAngle)
      .def_readonly("max_angle", &LaserScanConfig::maxAngle)
      .def_readonly("angular_resolution", &LaserScanConfig::angularResolution)
      .def_readonly("min_range", &LaserScanConfig::minRange)
      .def_readonly("max_range", &LaserScanConfig::maxRange)
      .def_readonly("min_angle", &LaserScanConfig::minAngle)
      .def_readonly("range_threshold", &LaserScanConfig::rangeThreshold)
      .def_readonly("sensor_name", &LaserScanConfig::sensorName);

  py::enum_<GridStates>(m, "GridStates")
      .value("Unknown", GridStates::GridStates_Unknown)
      .value("Occupied", GridStates::GridStates_Occupied)
      .value("Free", GridStates::GridStates_Free);

  py::class_<OccupancyGrid>(m, "OccupancyGrid")
    .def_property_readonly("width", &OccupancyGrid::GetWidth)
    .def_property_readonly("height", &OccupancyGrid::GetHeight)
    .def_property_readonly("offset",
                           [](const OccupancyGrid &a) {
                             auto offset =
                               (a.GetCoordinateConverter()->GetOffset());
                             return offset;
                           })
    .def("get_value", [](const OccupancyGrid &a, int32_t x, int32_t y) {
                        return a.GetValue(Vector2<int32_t>(x, y));
                      })
    .def_property_readonly("image", [](const OccupancyGrid &a) {
                       auto h = a.GetHeight();
                       auto w = a.GetWidth();
                       auto _r = py::array_t<uint8_t>({h, w});
                       auto r = _r.mutable_unchecked<2>();
                       for (auto j = 0; j < h; j++) {
                         for (auto i = 0; i < w; i++) {
                           // free was 255
                           // occupied was 100
                           // unknown was 0
                           auto val = a.GetValue(Vector2<int32_t>(w - i - 1, j));
                           if (val == 0) { val = 200; } else if (val == 100) { val = 0; }
                           // free is 255
                           // unknown is 200
                           // occupied is 0
                           r(j, w - i - 1) = val;
                         }
                       }
                       return _r;
                     })
    ;

#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
}
