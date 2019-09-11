#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include "Vector2.h"
#include "Grid.h"
#include "Functor.h"
#include "CoordinateConverter.h"
#include "LocalizedRangeScan.h"
#include "AdditionalMath.h"

class OccupancyGrid;

class CellUpdater : public Functor {
public:
  CellUpdater(OccupancyGrid *pGrid) : m_pOccupancyGrid(pGrid) {}

  /**
   * Updates the cell at the given index based on the grid's hits and pass
   * counters
   * @param index
   */
  virtual void operator()(uint32_t index);

private:
  OccupancyGrid *m_pOccupancyGrid;
}; // CellUpdater

/**
 * Occupancy grid definition. See GridStates for possible grid values.
 */
class OccupancyGrid : public Grid<uint8_t> {
  friend class CellUpdater;
  friend class IncrementalOccupancyGrid;

public:
  /**
   * Constructs an occupancy grid of given size
   * @param width
   * @param height
   * @param rOffset
   * @param resolution
   */
  OccupancyGrid(int32_t width, int32_t height,
                const Vector2<double> &rOffset,
                double resolution,
                uint32_t minPassThrough = 2,
                double occupancyThreshold = 0.1,
                double _rangeThreshold = 0)
      : Grid<uint8_t>(width, height),
        m_pCellPassCnt(Grid<uint32_t>::CreateGrid(0, 0, resolution)),
        m_pCellHitsCnt(Grid<uint32_t>::CreateGrid(0, 0, resolution)),
        m_pCellUpdater(NULL), rangeThreshold(_rangeThreshold) {
    m_pCellUpdater = new CellUpdater(this);

    // TODO Exceptions?
    // if (karto::math::DoubleEqual(resolution, 0.0)) {
    //   throw Exception("Resolution cannot be 0");
    // }

    m_pMinPassThrough = minPassThrough;
    m_pOccupancyThreshold = occupancyThreshold;

    GetCoordinateConverter()->SetScale(1.0 / resolution);
    GetCoordinateConverter()->SetOffset(rOffset);
  }

  /**
   * Destructor
   */
  virtual ~OccupancyGrid() {
    delete m_pCellUpdater;
    delete m_pCellPassCnt;
    delete m_pCellHitsCnt;
  }

public:
  /**
   * Create an occupancy grid from the given scans using the given resolution
   * @param rScans
   * @param resolution
   */
  static OccupancyGrid *CreateFromScans(const LocalizedRangeScanVector &rScans,
                                        double resolution, double rangeThreshold) {
    if (rScans.empty()) {
      return NULL;
    }

    int32_t width, height;
    Vector2<double> offset;
    ComputeDimensions(rScans, resolution, width, height, offset);
    OccupancyGrid *pOccupancyGrid =
      new OccupancyGrid(width, height, offset, resolution);
    pOccupancyGrid->rangeThreshold = rangeThreshold;
    pOccupancyGrid->CreateFromScans(rScans);

    return pOccupancyGrid;
  }

  /**
   * Make a clone
   * @return occupancy grid clone
   */
  OccupancyGrid *Clone() const {
    OccupancyGrid *pOccupancyGrid = new OccupancyGrid(
        GetWidth(), GetHeight(), GetCoordinateConverter()->GetOffset(),
        1.0 / GetCoordinateConverter()->GetScale());
    memcpy(pOccupancyGrid->GetDataPointer(), GetDataPointer(), GetDataSize());

    pOccupancyGrid->GetCoordinateConverter()->SetSize(
        GetCoordinateConverter()->GetSize());
    pOccupancyGrid->m_pCellPassCnt = m_pCellPassCnt->Clone();
    pOccupancyGrid->m_pCellHitsCnt = m_pCellHitsCnt->Clone();

    return pOccupancyGrid;
  }

  /**
   * Check if grid point is free
   * @param rPose
   * @return whether the cell at the given point is free space
   */
  virtual bool IsFree(const Vector2<int32_t> &rPose) const {
    uint8_t *pOffsets = reinterpret_cast<uint8_t *>(GetDataPointer(rPose));
    if (*pOffsets == GridStates_Free) {
      return true;
    }

    return false;
  }

  /**
   * Casts a ray from the given point (up to the given max range)
   * and returns the distance to the closest obstacle
   * @param rPose2
   * @param maxRange
   * @return distance to closest obstacle
   */
  virtual double RayCast(const Pose2 &rPose2, double maxRange) const {
    double scale = GetCoordinateConverter()->GetScale();

    double x = rPose2.GetX();
    double y = rPose2.GetY();
    double theta = rPose2.GetHeading();

    double sinTheta = sin(theta);
    double cosTheta = cos(theta);

    double xStop = x + maxRange * cosTheta;
    double xSteps = 1 + fabs(xStop - x) * scale;

    double yStop = y + maxRange * sinTheta;
    double ySteps = 1 + fabs(yStop - y) * scale;

    double steps = amath::Maximum(xSteps, ySteps);
    double delta = maxRange / steps;
    double distance = delta;

    for (uint32_t i = 1; i < steps; i++) {
      double x1 = x + distance * cosTheta;
      double y1 = y + distance * sinTheta;

      Vector2<int32_t> gridIndex = WorldToGrid(Vector2<double>(x1, y1));
      if (IsValidGridIndex(gridIndex) && IsFree(gridIndex)) {
        distance = (i + 1) * delta;
      } else {
        break;
      }
    }

    return (distance < maxRange) ? distance : maxRange;
  }

  /**
   * Sets the minimum number of beams that must pass through a cell before it
   * will be considered to be occupied or unoccupied.
   * This prevents stray beams from messing up the map.
   */
  void SetMinPassThrough(uint32_t count) {
    m_pMinPassThrough = count;
  }

  /**
   * Sets the minimum ratio of beams hitting cell to beams passing through
   * cell for cell to be marked as occupied.
   */
  void SetOccupancyThreshold(double thresh) {
    m_pOccupancyThreshold = thresh;
  }

protected:
  /**
   * Get cell hit grid
   * @return Grid<uint32_t>*
   */
  virtual Grid<uint32_t> *GetCellHitsCounts() { return m_pCellHitsCnt; }

  /**
   * Get cell pass grid
   * @return Grid<uint32_t>*
   */
  virtual Grid<uint32_t> *GetCellPassCounts() { return m_pCellPassCnt; }

protected:
  /**
   * Calculate grid dimensions from localized range scans
   * @param rScans
   * @param resolution
   * @param rWidth
   * @param rHeight
   * @param rOffset
   */
  static void ComputeDimensions(const LocalizedRangeScanVector &rScans,
                                double resolution, int32_t &rWidth,
                                int32_t &rHeight,
                                Vector2<double> &rOffset) {
    BoundingBox2 boundingBox;
    for (auto iter : rScans) {
      boundingBox.Add(iter->GetBoundingBox());
    }

    double scale = 1.0 / resolution;
    Size2<double> size = boundingBox.GetSize();

    rWidth = static_cast<int32_t>(amath::Round(size.GetWidth() * scale));
    rHeight = static_cast<int32_t>(amath::Round(size.GetHeight() * scale));
    rOffset = boundingBox.GetMinimum();
  }

  /**
   * Create grid using scans
   * @param rScans
   */
  virtual void CreateFromScans(const LocalizedRangeScanVector &rScans) {
    m_pCellPassCnt->Resize(GetWidth(), GetHeight());
    m_pCellPassCnt->GetCoordinateConverter()->SetOffset(
        GetCoordinateConverter()->GetOffset());

    m_pCellHitsCnt->Resize(GetWidth(), GetHeight());
    m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(
        GetCoordinateConverter()->GetOffset());

    for (auto iter : rScans) {
      LocalizedRangeScan *pScan = iter;
      AddScan(pScan);
    }

    Update();
  }

  /**
   * Adds the scan's information to this grid's counters (optionally
   * update the grid's cells' occupancy status)
   * @param pScan
   * @param doUpdate whether to update the grid's cell's occupancy status
   * @return returns false if an endpoint fell off the grid, otherwise true
   */
  virtual bool AddScan(LocalizedRangeScan *pScan, bool doUpdate = false) {
    double _rangeThreshold = (rangeThreshold > pScan->GetMinimumRange() &&
                              rangeThreshold < pScan->GetMaximumRange())
                                 ? rangeThreshold
                                 : pScan->GetRangeThreshold();
    double maxRange = pScan->GetMaximumRange();
    double minRange = pScan->GetMinimumRange();

    Vector2<double> scanPosition = pScan->GetCorrectedPose().GetPosition();

    // get scan point readings
    const PointVectorDouble &rPointReadings = pScan->GetPointReadings(false);

    bool isAllInMap = true;

    // draw lines from scan position to all point readings
    int pointIndex = 0;
    for (auto pointsIter : rPointReadings) {
      Vector2<double> point = pointsIter;
      double rangeReading = pScan->GetRangeReadings()[pointIndex];
      bool isEndPointValid = rangeReading < (_rangeThreshold - KT_TOLERANCE);

      if (rangeReading <= minRange || rangeReading >= maxRange ||
          std::isnan(rangeReading)) {
        // ignore these readings
        pointIndex++;
        continue;
      } else if (rangeReading >= _rangeThreshold) {
        // trace up to range reading
        double ratio = _rangeThreshold / rangeReading;
        double dx = point.GetX() - scanPosition.GetX();
        double dy = point.GetY() - scanPosition.GetY();
        point.SetX(scanPosition.GetX() + ratio * dx);
        point.SetY(scanPosition.GetY() + ratio * dy);
      }

      bool isInMap =
          RayTrace(scanPosition, point, isEndPointValid, doUpdate);
      if (!isInMap) {
        isAllInMap = false;
      }

      pointIndex++;
    }

    return isAllInMap;
  }

  /**
   * Traces a beam from the start position to the end position marking
   * the bookkeeping arrays accordingly.
   * @param rWorldFrom start position of beam
   * @param rWorldTo end position of beam
   * @param isEndPointValid is the reading within the range threshold?
   * @param doUpdate whether to update the cells' occupancy status immediately
   * @return returns false if an endpoint fell off the grid, otherwise true
   */
  virtual bool RayTrace(const Vector2<double> &rWorldFrom,
                           const Vector2<double> &rWorldTo,
                           bool isEndPointValid, bool doUpdate = false) {
    assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);

    Vector2<int32_t> gridFrom = m_pCellPassCnt->WorldToGrid(rWorldFrom);
    Vector2<int32_t> gridTo = m_pCellPassCnt->WorldToGrid(rWorldTo);

    CellUpdater *pCellUpdater = doUpdate ? m_pCellUpdater : NULL;
    m_pCellPassCnt->TraceLine(gridFrom.GetX(), gridFrom.GetY(), gridTo.GetX(),
                              gridTo.GetY(), pCellUpdater);

    // for the end point
    if (isEndPointValid) {
      if (m_pCellPassCnt->IsValidGridIndex(gridTo)) {
        int32_t index = m_pCellPassCnt->GridIndex(gridTo, false);

        uint32_t *pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
        uint32_t *pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

        // increment cell pass through and hit count
        pCellPassCntPtr[index]++;
        pCellHitCntPtr[index]++;

        if (doUpdate) {
          (*m_pCellUpdater)(index);
        }
      }
    }

    return m_pCellPassCnt->IsValidGridIndex(gridTo);
  }

  /**
   * Updates a single cell's value based on the given counters
   * @param pCell
   * @param cellPassCnt
   * @param cellHitCnt
   */
  virtual void UpdateCell(uint8_t *pCell, uint32_t cellPassCnt,
                          uint32_t cellHitCnt) {
    if (cellPassCnt > m_pMinPassThrough) {
      double hitRatio = static_cast<double>(cellHitCnt) /
                           static_cast<double>(cellPassCnt);

      if (hitRatio > m_pOccupancyThreshold) {
        *pCell = GridStates_Occupied;
      } else {
        *pCell = GridStates_Free;
      }
    }
  }

  /**
   * Update the grid based on the values in m_pCellHitsCnt and m_pCellPassCnt
   */
  virtual void Update() {
    assert(m_pCellPassCnt != NULL && m_pCellHitsCnt != NULL);

    // clear grid
    Clear();

    // set occupancy status of cells
    uint8_t *pDataPtr = GetDataPointer();
    uint32_t *pCellPassCntPtr = m_pCellPassCnt->GetDataPointer();
    uint32_t *pCellHitCntPtr = m_pCellHitsCnt->GetDataPointer();

    uint32_t nBytes = GetDataSize();
    for (uint32_t i = 0; i < nBytes;
         i++, pDataPtr++, pCellPassCntPtr++, pCellHitCntPtr++) {
      UpdateCell(pDataPtr, *pCellPassCntPtr, *pCellHitCntPtr);
    }
  }

  /**
   * Resizes the grid (deletes all old data)
   * @param width
   * @param height
   */
  virtual void Resize(int32_t width, int32_t height) {
    Grid<uint8_t>::Resize(width, height);
    m_pCellPassCnt->Resize(width, height);
    m_pCellHitsCnt->Resize(width, height);
  }

protected:
  /**
   * Counters of number of times a beam passed through a cell
   */
  Grid<uint32_t> *m_pCellPassCnt;

  /**
   * Counters of number of times a beam ended at a cell
   */
  Grid<uint32_t> *m_pCellHitsCnt;

private:
  /**
   * Restrict the copy constructor
   */
  OccupancyGrid(const OccupancyGrid &);

  /**
   * Restrict the assignment operator
   */
  const OccupancyGrid &operator=(const OccupancyGrid &);

private:
  CellUpdater *m_pCellUpdater;

  ////////////////////////////////////////////////////////////
  // NOTE: These two values are dependent on the resolution.  If the resolution
  // is too small, then not many beams will hit the cell!

  // Number of beams that must pass through a cell before it will be considered
  // to be occupied or unoccupied.  This prevents stray beams from messing up
  // the map.
  uint32_t m_pMinPassThrough;

  // Minimum ratio of beams hitting cell to beams passing through cell for cell
  // to be marked as occupied
  double m_pOccupancyThreshold;
  double rangeThreshold;
}; // OccupancyGrid

#endif
