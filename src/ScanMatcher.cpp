/*
 * Copyright 2010 SRI International
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

/* Modified 2019 by Jariullah Safi */

#include <algorithm>
#include "ScanMatcher.h"
#include "AdditionalMath.h"
#include <stdio.h>
#include "CorrelationGrid.h"

ScanMatcher::~ScanMatcher() {
  delete m_pCorrelationGrid;
  delete m_pSearchSpaceProbs;
  delete m_pGridLookup;
}

ScanMatcher *ScanMatcher::Create(std::shared_ptr<ScanMatcherConfig> config) {

  double resolution = config->resolution;
  // invalid parameters
  if (resolution <= 0) {
    return NULL;
  }

  double searchSize = config->searchSize;
  if (searchSize <= 0) {
    return NULL;
  }

  double smearDeviation = config->smearDeviation;
  if (smearDeviation < 0) {
    return NULL;
  }

  double rangeThreshold = config->rangeThreshold;
  if (rangeThreshold <= 0) {
    return NULL;
  }

  assert(amath::DoubleEqual(amath::Round(searchSize / resolution),
                           (searchSize / resolution)));

  // calculate search space in grid coordinates
  int32_t searchSpaceSideSize =
      static_cast<int32_t>(amath::Round(searchSize / resolution) + 1);

  // compute requisite size of correlation grid (pad grid so that scan points
  // can't fall off the grid if a scan is on the border of the search space)
  int32_t pointReadingMargin =
      static_cast<int32_t>(ceil(rangeThreshold / resolution));

  int32_t gridSize = searchSpaceSideSize + 2 * pointReadingMargin;

  // create correlation grid
  assert(gridSize % 2 == 1);
  CorrelationGrid *pCorrelationGrid = CorrelationGrid::CreateGrid(
      gridSize, gridSize, resolution, smearDeviation);

  // create search space probabilities
  Grid<double> *pSearchSpaceProbs = Grid<double>::CreateGrid(
      searchSpaceSideSize, searchSpaceSideSize, resolution);

  ScanMatcher *pScanMatcher = new ScanMatcher(config);
  pScanMatcher->m_pCorrelationGrid = pCorrelationGrid;
  pScanMatcher->m_pSearchSpaceProbs = pSearchSpaceProbs;
  pScanMatcher->m_pGridLookup = new GridIndexLookup<uint8_t>(pCorrelationGrid);

  return pScanMatcher;
}

/**
 * Match given scan against set of scans
 * @param pScan scan being scan-matched
 * @param rBaseScans set of scans whose points will mark cells in grid as being
 * occupied
 * @param rMean output parameter of mean (best pose) of match
 * @param rCovariance output parameter of covariance of match
 * @param doPenalize whether to penalize matches further from the search center
 * @param doRefineMatch whether to do finer-grained matching if coarse match is
 * good (default is true)
 * @return strength of response
 */
double ScanMatcher::MatchScan(LocalizedRangeScan *pScan,
                                 const LocalizedRangeScanVector &rBaseScans,
                                 Pose2 &rMean, Matrix3 &rCovariance,
                                 bool doPenalize, bool doRefineMatch) {
  ///////////////////////////////////////
  // set scan pose to be center of grid

  // TODO Below needs to be handled elsewhere, not ScanMatcher's business
  // 1. get scan position
  Pose2 scanPose = pScan->GetCorrectedPose();

  // scan has no readings; cannot do scan matching
  // best guess of pose is based off of adjusted odometer reading
  if (pScan->GetNumberOfRangeReadings() == 0) {
    rMean = scanPose;

    // maximum covariance
    rCovariance(0, 0) = MAX_VARIANCE; // XX
    rCovariance(1, 1) = MAX_VARIANCE; // YY
    rCovariance(2, 2) =
        4 *
        amath::Square(config->m_pCoarseAngleResolution); // TH*TH

    return 0.0;
  }

  // 2. get size of grid
  Rectangle2<int32_t> roi = m_pCorrelationGrid->GetROI();

  // 3. compute offset (in meters - lower left corner)
  Vector2<double> offset;
  offset.SetX(scanPose.GetX() - (0.5 * (roi.GetWidth() - 1) *
                                 m_pCorrelationGrid->GetResolution()));
  offset.SetY(scanPose.GetY() - (0.5 * (roi.GetHeight() - 1) *
                                 m_pCorrelationGrid->GetResolution()));

  // 4. set offset
  m_pCorrelationGrid->GetCoordinateConverter()->SetOffset(offset);

  ///////////////////////////////////////

  // set up correlation grid
  AddScans(rBaseScans, scanPose.GetPosition());

  // compute how far to search in each direction
  Vector2<double> searchDimensions(m_pSearchSpaceProbs->GetWidth(),
                                      m_pSearchSpaceProbs->GetHeight());

  Vector2<double> coarseSearchOffset(
      0.5 * (searchDimensions.GetX() - 1) * m_pCorrelationGrid->GetResolution(),
      0.5 * (searchDimensions.GetY() - 1) *
          m_pCorrelationGrid->GetResolution());

  // a coarse search only checks half the cells in each dimension
  Vector2<double> coarseSearchResolution(
      2 * m_pCorrelationGrid->GetResolution(),
      2 * m_pCorrelationGrid->GetResolution());

  // actual scan-matching
  double bestResponse =
      CorrelateScan(pScan, scanPose, coarseSearchOffset, coarseSearchResolution,
                    config->m_pCoarseSearchAngleOffset,
                    config->m_pCoarseAngleResolution, doPenalize,
                    rMean, rCovariance, false);

  if (config->m_pUseResponseExpansion == true) {
    if (amath::DoubleEqual(bestResponse, 0.0)) {
#ifdef KARTO_DEBUG
      std::cout << "Mapper Info: Expanding response search space!" << std::endl;
#endif
      // try and increase search angle offset with 20 degrees and do another
      // match
      double newSearchAngleOffset =
          config->m_pCoarseSearchAngleOffset;
      for (int32_t i = 0; i < 3; i++) {
        newSearchAngleOffset += amath::DegreesToRadians(20);

        bestResponse =
            CorrelateScan(pScan, scanPose, coarseSearchOffset,
                          coarseSearchResolution, newSearchAngleOffset,
                          config->m_pCoarseAngleResolution,
                          doPenalize, rMean, rCovariance, false);

        if (amath::DoubleEqual(bestResponse, 0.0) == false) {
          break;
        }
      }

      
#ifdef KARTO_DEBUG
      if (amath::DoubleEqual(bestResponse, 0.0)) {
        std::cout << "Mapper Warning: Unable to calculate response!"
                  << std::endl;
      }
#endif
    }
  }

  if (doRefineMatch) {
    Vector2<double> fineSearchOffset(coarseSearchResolution * 0.5);
    Vector2<double> fineSearchResolution(
        m_pCorrelationGrid->GetResolution(),
        m_pCorrelationGrid->GetResolution());
    bestResponse =
        CorrelateScan(pScan, rMean, fineSearchOffset, fineSearchResolution,
                      0.5 * config->m_pCoarseAngleResolution,
                      config->m_pFineSearchAngleResolution,
                      doPenalize, rMean, rCovariance, true);
  }


  
#ifdef KARTO_DEBUG
  std::cout << "  BEST POSE = " << rMean << " BEST RESPONSE = " << bestResponse
            << ",  VARIANCE = " << rCovariance(0, 0) << ", "
            << rCovariance(1, 1) << std::endl;
#endif
  assert(amath::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

  
  return bestResponse;
}

/**
 * Finds the best pose for the scan centering the search in the correlation grid
 * at the given pose and search in the space by the vector and angular offsets
 * in increments of the given resolutions
 * @param rScan scan to match against correlation grid
 * @param rSearchCenter the center of the search space
 * @param rSearchSpaceOffset searches poses in the area offset by this vector
 * around search center
 * @param rSearchSpaceResolution how fine a granularity to search in the search
 * space
 * @param searchAngleOffset searches poses in the angles offset by this angle
 * around search center
 * @param searchAngleResolution how fine a granularity to search in the angular
 * search space
 * @param doPenalize whether to penalize matches further from the search center
 * @param rMean output parameter of mean (best pose) of match
 * @param rCovariance output parameter of covariance of match
 * @param doingFineMatch whether to do a finer search after coarse search
 * @return strength of response
 */
double ScanMatcher::CorrelateScan(
    LocalizedRangeScan *pScan, const Pose2 &rSearchCenter,
    const Vector2<double> &rSearchSpaceOffset,
    const Vector2<double> &rSearchSpaceResolution,
    double searchAngleOffset, double searchAngleResolution,
    bool doPenalize, Pose2 &rMean, Matrix3 &rCovariance,
    bool doingFineMatch) {
  assert(searchAngleResolution != 0.0);

  
  // setup lookup arrays
  m_pGridLookup->ComputeOffsets(pScan, rSearchCenter.GetHeading(),
                                searchAngleOffset, searchAngleResolution);


  
  // only initialize probability grid if computing positional covariance (during
  // coarse match)
  if (!doingFineMatch) {
    m_pSearchSpaceProbs->Clear();

    // position search grid - finds lower left corner of search grid
    Vector2<double> offset(rSearchCenter.GetPosition() - rSearchSpaceOffset);
    m_pSearchSpaceProbs->GetCoordinateConverter()->SetOffset(offset);
  }

  // calculate position arrays

  
  std::vector<double> xPoses;
  int32_t nX =
      static_cast<int32_t>(amath::Round(rSearchSpaceOffset.GetX() * 2.0 /
                                         rSearchSpaceResolution.GetX()) +
                             1);
  
  double startX = -rSearchSpaceOffset.GetX();
  for (int32_t xIndex = 0; xIndex < nX; xIndex++) {
    xPoses.push_back(startX + xIndex * rSearchSpaceResolution.GetX());
  }
  assert(amath::DoubleEqual(xPoses.back(), -startX));

  
  std::vector<double> yPoses;
  int32_t nY =
      static_cast<int32_t>(amath::Round(rSearchSpaceOffset.GetY() * 2.0 /
                                         rSearchSpaceResolution.GetY()) +
                             1);
  double startY = -rSearchSpaceOffset.GetY();
  for (int32_t yIndex = 0; yIndex < nY; yIndex++) {
    yPoses.push_back(startY + yIndex * rSearchSpaceResolution.GetY());
  }

  
  assert(amath::DoubleEqual(yPoses.back(), -startY));

  // calculate pose response array size
  int32_t nAngles = static_cast<int32_t>(
      amath::Round(searchAngleOffset * 2.0 / searchAngleResolution) + 1);

  int32_t poseResponseSize =
      static_cast<int32_t>(xPoses.size() * yPoses.size() * nAngles);

  // allocate array
  std::pair<double, Pose2> *pPoseResponse =
      new std::pair<double, Pose2>[poseResponseSize];

  Vector2<int32_t> startGridPoint =
      m_pCorrelationGrid->WorldToGrid(Vector2<double>(
          rSearchCenter.GetX() + startX, rSearchCenter.GetY() + startY));

  int32_t poseResponseCounter = 0;
  for (auto yIter : yPoses) {
    double y = yIter;
    double newPositionY = rSearchCenter.GetY() + y;
    double squareY = amath::Square(y);

    for (auto xIter : xPoses) {
      double x = xIter;
      double newPositionX = rSearchCenter.GetX() + x;
      double squareX = amath::Square(x);

      Vector2<int32_t> gridPoint = m_pCorrelationGrid->WorldToGrid(
          Vector2<double>(newPositionX, newPositionY));
      int32_t gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);
      assert(gridIndex >= 0);

      double angle = 0.0;
      double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;
      for (int32_t angleIndex = 0; angleIndex < nAngles; angleIndex++) {
        angle = startAngle + angleIndex * searchAngleResolution;

        double response = GetResponse(angleIndex, gridIndex);
        if (doPenalize && (amath::DoubleEqual(response, 0.0) == false)) {
          // simple model (approximate Gaussian) to take odometry into account

          double squaredDistance = squareX + squareY;
          double distancePenalty =
              1.0 - (DISTANCE_PENALTY_GAIN * squaredDistance /
                     config->m_pDistanceVariancePenalty);
          distancePenalty =
              amath::Maximum(distancePenalty,
                            config->m_pMinimumDistancePenalty);

          double squaredAngleDistance =
              amath::Square(angle - rSearchCenter.GetHeading());
          double anglePenalty =
              1.0 - (ANGLE_PENALTY_GAIN * squaredAngleDistance /
                     config->m_pAngleVariancePenalty);
          anglePenalty = amath::Maximum(
              anglePenalty, config->m_pMinimumAnglePenalty);

          response *= (distancePenalty * anglePenalty);
        }

        // store response and pose
        pPoseResponse[poseResponseCounter] = std::pair<double, Pose2>(
            response,
            Pose2(newPositionX, newPositionY, amath::NormalizeAngle(angle)));
        poseResponseCounter++;
      }

      assert(amath::DoubleEqual(angle,
                               rSearchCenter.GetHeading() + searchAngleOffset));
    }
  }

  assert(poseResponseSize == poseResponseCounter);

  // find value of best response (in [0; 1])
  double bestResponse = -1;
  for (int32_t i = 0; i < poseResponseSize; i++) {
    bestResponse = amath::Maximum(bestResponse, pPoseResponse[i].first);

    // will compute positional covariance, save best relative probability for
    // each cell
    if (!doingFineMatch) {
      const Pose2 &rPose = pPoseResponse[i].second;
      Vector2<int32_t> grid =
          m_pSearchSpaceProbs->WorldToGrid(rPose.GetPosition());

      // Changed (double*) to the reinterpret_cast - Luc
      double *ptr = reinterpret_cast<double *>(
          m_pSearchSpaceProbs->GetDataPointer(grid));
      if (ptr == NULL) {
        throw std::runtime_error(
            "Mapper FATAL ERROR - Index out of range in probability search!");
      }

      *ptr = amath::Maximum(pPoseResponse[i].first, *ptr);
    }
  }

  // average all poses with same highest response
  Vector2<double> averagePosition;
  double thetaX = 0.0;
  double thetaY = 0.0;
  int32_t averagePoseCount = 0;
  for (int32_t i = 0; i < poseResponseSize; i++) {
    if (amath::DoubleEqual(pPoseResponse[i].first, bestResponse)) {
      averagePosition += pPoseResponse[i].second.GetPosition();

      double heading = pPoseResponse[i].second.GetHeading();
      thetaX += cos(heading);
      thetaY += sin(heading);

      averagePoseCount++;
    }
  }

  Pose2 averagePose;
  if (averagePoseCount > 0) {
    averagePosition /= averagePoseCount;

    thetaX /= averagePoseCount;
    thetaY /= averagePoseCount;

    averagePose = Pose2(averagePosition, atan2(thetaY, thetaX));
  } else {
    throw std::runtime_error(
        "Mapper FATAL ERROR - Unable to find best position");
  }

  // delete pose response array
  delete[] pPoseResponse;

#ifdef KARTO_DEBUG
  std::cout << "bestPose: " << averagePose << std::endl;
  std::cout << "bestResponse: " << bestResponse << std::endl;
#endif

  if (!doingFineMatch) {
    ComputePositionalCovariance(averagePose, bestResponse, rSearchCenter,
                                rSearchSpaceOffset, rSearchSpaceResolution,
                                searchAngleResolution, rCovariance);
  } else {
    ComputeAngularCovariance(averagePose, bestResponse, rSearchCenter,
                             searchAngleOffset, searchAngleResolution,
                             rCovariance);
  }

  rMean = averagePose;

#ifdef KARTO_DEBUG
  std::cout << "bestPose: " << averagePose << std::endl;
#endif

  if (bestResponse > 1.0) {
    bestResponse = 1.0;
  }

  assert(amath::InRange(bestResponse, 0.0, 1.0));
  assert(amath::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

  return bestResponse;
}

/**
 * Computes the positional covariance of the best pose
 * @param rBestPose
 * @param bestResponse
 * @param rSearchCenter
 * @param rSearchSpaceOffset
 * @param rSearchSpaceResolution
 * @param searchAngleResolution
 * @param rCovariance
 */
void ScanMatcher::ComputePositionalCovariance(
    const Pose2 &rBestPose, double bestResponse, const Pose2 &rSearchCenter,
    const Vector2<double> &rSearchSpaceOffset,
    const Vector2<double> &rSearchSpaceResolution,
    double searchAngleResolution, Matrix3 &rCovariance) {
  // reset covariance to identity matrix
  rCovariance.SetToIdentity();

  // if best response is vary small return max variance
  if (bestResponse < KT_TOLERANCE) {
    rCovariance(0, 0) = MAX_VARIANCE;                            // XX
    rCovariance(1, 1) = MAX_VARIANCE;                            // YY
    rCovariance(2, 2) = 4 * amath::Square(searchAngleResolution); // TH*TH

    return;
  }

  double accumulatedVarianceXX = 0;
  double accumulatedVarianceXY = 0;
  double accumulatedVarianceYY = 0;
  double norm = 0;

  double dx = rBestPose.GetX() - rSearchCenter.GetX();
  double dy = rBestPose.GetY() - rSearchCenter.GetY();

  double offsetX = rSearchSpaceOffset.GetX();
  double offsetY = rSearchSpaceOffset.GetY();

  int32_t nX = static_cast<int32_t>(
      amath::Round(offsetX * 2.0 / rSearchSpaceResolution.GetX()) + 1);
  double startX = -offsetX;
  assert(amath::DoubleEqual(startX + (nX - 1) * rSearchSpaceResolution.GetX(),
                           -startX));

  int32_t nY = static_cast<int32_t>(
      amath::Round(offsetY * 2.0 / rSearchSpaceResolution.GetY()) + 1);
  double startY = -offsetY;
  assert(amath::DoubleEqual(startY + (nY - 1) * rSearchSpaceResolution.GetY(),
                           -startY));

  for (int32_t yIndex = 0; yIndex < nY; yIndex++) {
    double y = startY + yIndex * rSearchSpaceResolution.GetY();

    for (int32_t xIndex = 0; xIndex < nX; xIndex++) {
      double x = startX + xIndex * rSearchSpaceResolution.GetX();

      Vector2<int32_t> gridPoint =
          m_pSearchSpaceProbs->WorldToGrid(Vector2<double>(
              rSearchCenter.GetX() + x, rSearchCenter.GetY() + y));
      double response = *(m_pSearchSpaceProbs->GetDataPointer(gridPoint));

      // response is not a low response
      if (response >= (bestResponse - 0.1)) {
        norm += response;
        accumulatedVarianceXX += (amath::Square(x - dx) * response);
        accumulatedVarianceXY += ((x - dx) * (y - dy) * response);
        accumulatedVarianceYY += (amath::Square(y - dy) * response);
      }
    }
  }

  if (norm > KT_TOLERANCE) {
    double varianceXX = accumulatedVarianceXX / norm;
    double varianceXY = accumulatedVarianceXY / norm;
    double varianceYY = accumulatedVarianceYY / norm;
    double varianceTHTH = 4 * amath::Square(searchAngleResolution);

    // lower-bound variances so that they are not too small;
    // ensures that links are not too tight
    double minVarianceXX = 0.1 * amath::Square(rSearchSpaceResolution.GetX());
    double minVarianceYY = 0.1 * amath::Square(rSearchSpaceResolution.GetY());
    varianceXX = amath::Maximum(varianceXX, minVarianceXX);
    varianceYY = amath::Maximum(varianceYY, minVarianceYY);

    // increase variance for poorer responses
    double multiplier = 1.0 / bestResponse;
    rCovariance(0, 0) = varianceXX * multiplier;
    rCovariance(0, 1) = varianceXY * multiplier;
    rCovariance(1, 0) = varianceXY * multiplier;
    rCovariance(1, 1) = varianceYY * multiplier;
    rCovariance(2, 2) =
        varianceTHTH; // this value will be set in ComputeAngularCovariance
  }

  // if values are 0, set to MAX_VARIANCE
  // values might be 0 if points are too sparse and thus don't hit other points
  if (amath::DoubleEqual(rCovariance(0, 0), 0.0)) {
    rCovariance(0, 0) = MAX_VARIANCE;
  }

  if (amath::DoubleEqual(rCovariance(1, 1), 0.0)) {
    rCovariance(1, 1) = MAX_VARIANCE;
  }
}

/**
 * Computes the angular covariance of the best pose
 * @param rBestPose
 * @param bestResponse
 * @param rSearchCenter
 * @param rSearchAngleOffset
 * @param searchAngleResolution
 * @param rCovariance
 */
void ScanMatcher::ComputeAngularCovariance(const Pose2 &rBestPose,
                                           double bestResponse,
                                           const Pose2 &rSearchCenter,
                                           double searchAngleOffset,
                                           double searchAngleResolution,
                                           Matrix3 &rCovariance) {
  // NOTE: do not reset covariance matrix

  // normalize angle difference
  double bestAngle = amath::NormalizeAngleDifference(
      rBestPose.GetHeading(), rSearchCenter.GetHeading());

  Vector2<int32_t> gridPoint =
      m_pCorrelationGrid->WorldToGrid(rBestPose.GetPosition());
  int32_t gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

  int32_t nAngles = static_cast<int32_t>(
      amath::Round(searchAngleOffset * 2 / searchAngleResolution) + 1);

  double angle = 0.0;
  double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;

  double norm = 0.0;
  double accumulatedVarianceThTh = 0.0;
  for (int32_t angleIndex = 0; angleIndex < nAngles; angleIndex++) {
    angle = startAngle + angleIndex * searchAngleResolution;
    double response = GetResponse(angleIndex, gridIndex);

    // response is not a low response
    if (response >= (bestResponse - 0.1)) {
      norm += response;
      accumulatedVarianceThTh += (amath::Square(angle - bestAngle) * response);
    }
  }
  assert(
      amath::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));

  if (norm > KT_TOLERANCE) {
    if (accumulatedVarianceThTh < KT_TOLERANCE) {
      accumulatedVarianceThTh = amath::Square(searchAngleResolution);
    }

    accumulatedVarianceThTh /= norm;
  } else {
    accumulatedVarianceThTh = 1000 * amath::Square(searchAngleResolution);
  }

  rCovariance(2, 2) = accumulatedVarianceThTh;
}

/**
 * Marks cells where scans' points hit as being occupied
 * @param rScans scans whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view
 * point
 */
void ScanMatcher::AddScans(const LocalizedRangeScanVector &rScans,
                           Vector2<double> viewPoint) {
  m_pCorrelationGrid->Clear();

  // add all scans to grid
  // const_forEach(LocalizedRangeScanVector, &rScans) {
  for (auto iter : rScans) {
    AddScan(iter, viewPoint);
  }
}

/**
 * Marks cells where scans' points hit as being occupied.  Can smear points as
 * they are added.
 * @param pScan scan whose points will mark cells in grid as being occupied
 * @param viewPoint do not add points that belong to scans "opposite" the view
 * point
 * @param doSmear whether the points will be smeared
 */
void ScanMatcher::AddScan(LocalizedRangeScan *pScan,
                          const Vector2<double> &rViewPoint,
                          bool doSmear) {
  PointVectorDouble validPoints = FindValidPoints(pScan, rViewPoint);

  // put in all valid points
  // const_forEach(PointVectorDouble, &validPoints) {
  for (auto iter : validPoints) {
    Vector2<int32_t> gridPoint = m_pCorrelationGrid->WorldToGrid(iter);
    if (!amath::IsUpTo(gridPoint.GetX(),
                      m_pCorrelationGrid->GetROI().GetWidth()) ||
        !amath::IsUpTo(gridPoint.GetY(),
                      m_pCorrelationGrid->GetROI().GetHeight())) {
      // point not in grid
      continue;
    }

    int gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

    // set grid cell as occupied
    if (m_pCorrelationGrid->GetDataPointer()[gridIndex] ==
        GridStates_Occupied) {
      // value already set
      continue;
    }

    m_pCorrelationGrid->GetDataPointer()[gridIndex] = GridStates_Occupied;

    // smear grid
    if (doSmear == true) {
      m_pCorrelationGrid->SmearPoint(gridPoint);
    }
  }
}

/**
 * Compute which points in a scan are on the same side as the given viewpoint
 * @param pScan
 * @param rViewPoint
 * @return points on the same side
 */
PointVectorDouble
ScanMatcher::FindValidPoints(LocalizedRangeScan *pScan,
                             const Vector2<double> &rViewPoint) const {
  const PointVectorDouble &rPointReadings = pScan->GetPointReadings();

  // points must be at least 10 cm away when making comparisons of
  // inside/outside of viewpoint
  const double minSquareDistance = amath::Square(0.1); // in m^2

  // this iterator lags from the main iterator adding points only when the
  // points are on the same side as the viewpoint
  PointVectorDouble::const_iterator trailingPointIter = rPointReadings.begin();
  PointVectorDouble validPoints;

  Vector2<double> firstPoint;
  bool firstTime = true;
  // const_forEach(PointVectorDouble, &rPointReadings) {
  for (auto iter = rPointReadings.begin(); iter < rPointReadings.end(); iter++) {
    Vector2<double> currentPoint = *iter;

    if (firstTime && !std::isnan(currentPoint.GetX()) &&
        !std::isnan(currentPoint.GetY())) {
      firstPoint = currentPoint;
      firstTime = false;
    }

    Vector2<double> delta = firstPoint - currentPoint;
    if (delta.SquaredLength() > minSquareDistance) {
      // This compute the Determinant (viewPoint FirstPoint, viewPoint
      // currentPoint) Which computes the direction of rotation, if the rotation
      // is counterclock wise then we are looking at data we should keep. If
      // it's negative rotation we should not included in in the matching have
      // enough distance, check viewpoint
      double a = rViewPoint.GetY() - firstPoint.GetY();
      double b = firstPoint.GetX() - rViewPoint.GetX();
      double c = firstPoint.GetY() * rViewPoint.GetX() -
                 firstPoint.GetX() * rViewPoint.GetY();
      double ss = currentPoint.GetX() * a + currentPoint.GetY() * b + c;

      // reset beginning point
      firstPoint = currentPoint;

      if (ss < 0.0) // wrong side, skip and keep going
      {
        trailingPointIter = iter;
      } else {
        for (; trailingPointIter != iter; ++trailingPointIter) {
          validPoints.push_back(*trailingPointIter);
        }
      }
    }
  }

  return validPoints;
}

/**
 * Get response at given position for given rotation (only look up valid points)
 * @param angleIndex
 * @param gridPositionIndex
 * @return response
 */
double ScanMatcher::GetResponse(int32_t angleIndex,
                                   int32_t gridPositionIndex) const {
  double response = 0.0;

  // add up value for each point
  uint8_t *pByte = m_pCorrelationGrid->GetDataPointer() + gridPositionIndex;

  const LookupArray *pOffsets = m_pGridLookup->GetLookupArray(angleIndex);
  assert(pOffsets != NULL);

  // get number of points in offset list
  int32_t nPoints = pOffsets->GetSize();
  if (nPoints == 0) {
    return response;
  }

  // calculate response
  int32_t *pAngleIndexPointer = pOffsets->GetArrayPointer();
  for (int32_t i = 0; i < nPoints; i++) {
    // ignore points that fall off the grid
    int32_t pointGridIndex = gridPositionIndex + pAngleIndexPointer[i];
    if (!amath::IsUpTo(pointGridIndex, m_pCorrelationGrid->GetDataSize()) ||
        pAngleIndexPointer[i] == INVALID_SCAN) {
      continue;
    }

    // uses index offsets to efficiently find location of point in the grid
    response += pByte[pAngleIndexPointer[i]];
  }

  // normalize response
  response /= (nPoints * GridStates_Occupied);
  assert(fabs(response) <= 1.0);

  return response;
}
