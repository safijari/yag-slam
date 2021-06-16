#ifndef GRID_IDX_LKUP_H
#define GRID_IDX_LKUP_H
#include "Grid.h"
#include "LocalizedRangeScan.h"
#include "LookupArray.h"
#include "Transform.h"
#include <stdio.h>
/**
 * Create lookup tables for point readings at varying angles in grid.
 * For each angle, grid indexes are calculated for each range reading.
 * This is to speed up finding best angle/position for a localized range scan
 *
 * Used heavily in mapper and localizer.
 *
 * In the localizer, this is a huge speed up for calculating possible position.
 * For each particle, a probability is calculated.  The range scan is the same,
 * but all grid indexes at all possible angles are calculated.  So when
 * calculating the particle probability at a specific angle, the index table is
 * used to look up probability in probability grid!
 *
 */
template <typename T> class GridIndexLookup {
public:
  /**
   * Construct a GridIndexLookup with a grid
   * @param pGrid
   */
  GridIndexLookup(Grid<T> *pGrid)
      : m_pGrid(pGrid), m_Capacity(0), m_Size(0), m_ppLookupArray(NULL) {}

  /**
   * Destructor
   */
  virtual ~GridIndexLookup() { DestroyArrays(); }

public:
  /**
   * Gets the lookup array for a particular angle index
   * @param index
   * @return lookup array
   */
  const LookupArray *GetLookupArray(int32_t index) const {
    assert(amath::IsUpTo(index, m_Size));

    return m_ppLookupArray[index];
  }

  /**
   * Get angles
   * @return std::vector<double>& angles
   */
  const std::vector<double> &GetAngles() const { return m_Angles; }

  /**
   * Compute lookup table of the points of the given scan for the given angular
   * space
   * @param pScan the scan
   * @param angleCenter
   * @param angleOffset computes lookup arrays for the angles within this offset
   * around angleStart
   * @param angleResolution how fine a granularity to compute lookup arrays in
   * the angular space
   */
  void ComputeOffsets(LocalizedRangeScan *pScan, double angleCenter,
                      double angleOffset, double angleResolution) {
    assert(angleOffset > 0.0);
    assert(angleResolution > 0.0);

    

    int32_t nAngles = static_cast<int32_t>(
        amath::Round(angleOffset * 2.0 / angleResolution) + 1);
    
    SetSize(nAngles);
    

    //////////////////////////////////////////////////////
    // convert points into local coordinates of scan pose

    const PointVectorDouble &rPointReadings = pScan->GetPointReadings();


    // compute transform to scan pose
    Transform transform(pScan->GetCorrectedPose());


    Pose2Vector localPoints;
    for (auto iter : rPointReadings) {
      // do inverse transform to get points in local coordinates
      Pose2 vec = transform.InverseTransformPose(Pose2(iter, 0.0));
      localPoints.push_back(vec);
    }



    //////////////////////////////////////////////////////
    // create lookup array for different angles
    double angle = 0.0;
    double startAngle = angleCenter - angleOffset;
    for (int32_t angleIndex = 0; angleIndex < nAngles; angleIndex++) {
      angle = startAngle + angleIndex * angleResolution;
      ComputeOffsets(angleIndex, angle, localPoints, pScan);
    }
   
    // assert(amath::DoubleEqual(angle, angleCenter + angleOffset));
  }

private:
  /**
   * Compute lookup value of points for given angle
   * @param angleIndex
   * @param angle
   * @param rLocalPoints
   */
  void ComputeOffsets(int32_t angleIndex, double angle,
                      const Pose2Vector &rLocalPoints,
                      LocalizedRangeScan *pScan) {
    m_ppLookupArray[angleIndex]->SetSize(
        static_cast<int32_t>(rLocalPoints.size()));
    m_Angles.at(angleIndex) = angle;

    // set up point array by computing relative offsets to points readings
    // when rotated by given angle

    const Vector2<double> &rGridOffset =
        m_pGrid->GetCoordinateConverter()->GetOffset();

    double cosine = cos(angle);
    double sine = sin(angle);

    int32_t readingIndex = 0;

    int32_t *pAngleIndexPointer =
        m_ppLookupArray[angleIndex]->GetArrayPointer();

    double maxRange = pScan->GetMaximumRange();

    for (auto iter : rLocalPoints) {
      const Vector2<double> &rPosition = iter.GetPosition();

      if (std::isnan(pScan->GetRangeReadings()[readingIndex]) ||
          std::isinf(pScan->GetRangeReadings()[readingIndex])) {
        pAngleIndexPointer[readingIndex] = INVALID_SCAN;
        readingIndex++;
        continue;
      }

      // counterclockwise rotation and that rotation is about the origin (0, 0).
      Vector2<double> offset;
      offset.SetX(cosine * rPosition.GetX() - sine * rPosition.GetY());
      offset.SetY(sine * rPosition.GetX() + cosine * rPosition.GetY());

      // have to compensate for the grid offset when getting the grid index
      Vector2<int32_t> gridPoint = m_pGrid->WorldToGrid(offset + rGridOffset);

      // use base GridIndex to ignore ROI
      int32_t lookupIndex = m_pGrid->Grid<T>::GridIndex(gridPoint, false);

      pAngleIndexPointer[readingIndex] = lookupIndex;

      readingIndex++;
    }
    assert(readingIndex == rLocalPoints.size());
  }

  /**
   * Sets size of lookup table (resize if not big enough)
   * @param size
   */
  void SetSize(int32_t size) {
    assert(size > 0);

    if (size > m_Capacity) {
      if (m_ppLookupArray != NULL) {
        DestroyArrays();
      }

      m_Capacity = size;
      m_ppLookupArray = new LookupArray *[m_Capacity];
      for (int32_t i = 0; i < m_Capacity; i++) {
        m_ppLookupArray[i] = new LookupArray();
      }
    }

    m_Size = size;

    m_Angles.resize(size);
  }

  /**
   * Delete the arrays
   */
  void DestroyArrays() {
    for (int32_t i = 0; i < m_Capacity; i++) {
      delete m_ppLookupArray[i];
    }

    delete[] m_ppLookupArray;
    m_ppLookupArray = NULL;
  }

private:
  Grid<T> *m_pGrid;

  int32_t m_Capacity;
  int32_t m_Size;

  LookupArray **m_ppLookupArray;

  // for sanity check
  std::vector<double> m_Angles;
}; // class GridIndexLookup

#endif
