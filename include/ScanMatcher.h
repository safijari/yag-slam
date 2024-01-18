#ifndef SCAN_MATCH_H
#define SCAN_MATCH_H

#include "Grid.h"
#include "Pose.h"
#include "Matrix.h"
#include "CorrelationGrid.h"
#include "LocalizedRangeScan.h"
#include "GridIndexLookup.h"
#include <memory>

#define MAX_VARIANCE            500.0
#define DISTANCE_PENALTY_GAIN   0.2
#define ANGLE_PENALTY_GAIN      0.2


class ScanMatcherConfig
{
public:
  /**
   * Destructor
   */
  // virtual ~ScanMatcherConfig();

  ScanMatcherConfig(
              // Mapper* pMapper
              )
    :
    m_pCoarseAngleResolution(0.03490658503988659),
    m_pCoarseSearchAngleOffset(0.3490658503988659),
    m_pFineSearchAngleResolution(0.003490658503988659),
    m_pDistanceVariancePenalty(0.5),
    m_pAngleVariancePenalty(0.3490658503988659),
    m_pMinimumDistancePenalty(1.0),
    m_pMinimumAnglePenalty(0.9),
    m_pUseResponseExpansion(true),
    searchSize(0.3),
    resolution(0.01),
    smearDeviation(0.03),
    rangeThreshold(40)
  {
  }

  double m_pCoarseAngleResolution;
  double m_pCoarseSearchAngleOffset;
  double m_pFineSearchAngleResolution;
  double m_pDistanceVariancePenalty;
  double m_pAngleVariancePenalty;
  double m_pMinimumDistancePenalty;
  double m_pMinimumAnglePenalty;
  bool m_pUseResponseExpansion;
  double searchSize;
  double resolution;
  double smearDeviation;
  double rangeThreshold;
};

class ScanMatcher
{
public:
  /**
 * Destructor
    */
  virtual ~ScanMatcher();

public:
  /**
 * Create a scan matcher with the given parameters
    */
  static ScanMatcher* Create(std::shared_ptr<ScanMatcherConfig> config);

  /**
 * Match given scan against set of scans
    * @param pScan scan being scan-matched
    * @param rBaseScans set of scans whose points will mark cells in grid as being occupied
    * @param rMean output parameter of mean (best pose) of match
    * @param rCovariance output parameter of covariance of match
    * @param doPenalize whether to penalize matches further from the search center
    * @param doRefineMatch whether to do finer-grained matching if coarse match is good (default is true)
    * @return strength of response
    */
  double MatchScan(LocalizedRangeScan* pScan,
                   const LocalizedRangeScanVector& rBaseScans,
                   Pose2& rMean, Matrix3& rCovariance,
                   bool doPenalize = true,
                   bool doRefineMatch = true);

  /**
 * Finds the best pose for the scan centering the search in the correlation grid
    * at the given pose and search in the space by the vector and angular offsets
    * in increments of the given resolutions
    * @param pScan scan to match against correlation grid
    * @param rSearchCenter the center of the search space
    * @param rSearchSpaceOffset searches poses in the area offset by this vector around search center
    * @param rSearchSpaceResolution how fine a granularity to search in the search space
    * @param searchAngleOffset searches poses in the angles offset by this angle around search center
    * @param searchAngleResolution how fine a granularity to search in the angular search space
    * @param doPenalize whether to penalize matches further from the search center
    * @param rMean output parameter of mean (best pose) of match
    * @param rCovariance output parameter of covariance of match
    * @param doingFineMatch whether to do a finer search after coarse search
    * @return strength of response
    */
  double CorrelateScan(LocalizedRangeScan* pScan,
                       const Pose2& rSearchCenter,
                       const Vector2<double>& rSearchSpaceOffset,
                       const Vector2<double>& rSearchSpaceResolution,
                       double searchAngleOffset,
                       double searchAngleResolution,
                       bool doPenalize,
                       Pose2& rMean,
                       Matrix3& rCovariance,
                       bool doingFineMatch);

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
  void ComputePositionalCovariance(const Pose2& rBestPose,
                                   double bestResponse,
                                   const Pose2& rSearchCenter,
                                   const Vector2<double>& rSearchSpaceOffset,
                                   const Vector2<double>& rSearchSpaceResolution,
                                   double searchAngleResolution,
                                   Matrix3& rCovariance);

  /**
 * Computes the angular covariance of the best pose
    * @param rBestPose
    * @param bestResponse
    * @param rSearchCenter
    * @param searchAngleOffset
    * @param searchAngleResolution
    * @param rCovariance
    */
  void ComputeAngularCovariance(const Pose2& rBestPose,
                                double bestResponse,
                                const Pose2& rSearchCenter,
                                double searchAngleOffset,
                                double searchAngleResolution,
                                Matrix3& rCovariance);

  /**
 * Gets the correlation grid data (for debugging)
    * @return correlation grid
    */
  inline CorrelationGrid* GetCorrelationGrid() const
  {
    return m_pCorrelationGrid;
  }

private:
  /**
 * Marks cells where scans' points hit as being occupied
    * @param rScans scans whose points will mark cells in grid as being occupied
    * @param viewPoint do not add points that belong to scans "opposite" the view point
    */
  void AddScans(const LocalizedRangeScanVector& rScans, Vector2<double> viewPoint);

  /**
 * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
    * @param pScan scan whose points will mark cells in grid as being occupied
    * @param viewPoint do not add points that belong to scans "opposite" the view point
    * @param doSmear whether the points will be smeared
    */
  void AddScan(LocalizedRangeScan* pScan, const Vector2<double>& rViewPoint, bool doSmear = true);

  /**
 * Compute which points in a scan are on the same side as the given viewpoint
    * @param pScan
    * @param rViewPoint
    * @return points on the same side
    */
  PointVectorDouble FindValidPoints(LocalizedRangeScan* pScan, const Vector2<double>& rViewPoint) const;

  /**
 * Get response at given position for given rotation (only look up valid points)
    * @param angleIndex
    * @param gridPositionIndex
    * @return response
    */
  double GetResponse(int32_t angleIndex, int32_t gridPositionIndex) const;

protected:
  /**
 * Default constructor
    */
  ScanMatcher(std::shared_ptr<ScanMatcherConfig> config_)
    :
    config(config_),
    m_pCorrelationGrid(NULL)
    , m_pSearchSpaceProbs(NULL)
    , m_pGridLookup(NULL)
  {
  }

private:
  std::shared_ptr<ScanMatcherConfig> config;
  CorrelationGrid* m_pCorrelationGrid;
  Grid<double>* m_pSearchSpaceProbs;

  GridIndexLookup<uint8_t>* m_pGridLookup;
};  // ScanMatcher

#endif
