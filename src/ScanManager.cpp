#include <stdint.h>
#include "LocalizedRangeScanAndFinder.h"
#include "AdditionalMath.h"

/**
 * Manages the scan data for a device
 */
class ScanManager {
public:
  /**
   * Default constructor
   */
  ScanManager(int32_t runningBufferMaximumSize,
              double runningBufferMaximumDistance)
      : m_pLastScan(NULL), m_RunningBufferMaximumSize(runningBufferMaximumSize),
        m_RunningBufferMaximumDistance(runningBufferMaximumDistance) {}

  /**
   * Destructor
   */
  virtual ~ScanManager() { Clear(); }

public:
  /**
   * Adds scan to vector of processed scans tagging scan with given unique id
   * @param pScan
   */
  inline void AddScan(LocalizedRangeScan *pScan, int32_t uniqueId) {
    // assign state id to scan
    pScan->SetStateId(static_cast<int32_t>(m_Scans.size()));

    // assign unique id to scan
    pScan->SetUniqueId(uniqueId);

    // add it to scan buffer
    m_Scans.push_back(pScan);
  }

  /**
   * Gets last scan
   * @param deviceId
   * @return last localized range scan
   */
  inline LocalizedRangeScan *GetLastScan() { return m_pLastScan; }

  /**
   * Sets the last scan
   * @param pScan
   */
  inline void SetLastScan(LocalizedRangeScan *pScan) { m_pLastScan = pScan; }

  /**
   * Gets scans
   * @return scans
   */
  inline LocalizedRangeScanVector &GetScans() { return m_Scans; }

  /**
   * Gets running scans
   * @return running scans
   */
  inline LocalizedRangeScanVector &GetRunningScans() { return m_RunningScans; }

  /**
   * Adds scan to vector of running scans
   * @param pScan
   */
  void AddRunningScan(LocalizedRangeScan *pScan) {
    m_RunningScans.push_back(pScan);

    // vector has at least one element (first line of this function), so this is
    // valid
    Pose2 frontScanPose = m_RunningScans.front()->GetSensorPose();
    Pose2 backScanPose = m_RunningScans.back()->GetSensorPose();

    // cap vector size and remove all scans from front of vector that are too
    // far from end of vector
    double squaredDistance =
        frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
    while (m_RunningScans.size() > m_RunningBufferMaximumSize ||
           squaredDistance >
               amath::Square(m_RunningBufferMaximumDistance) - KT_TOLERANCE) {
      // remove front of running scans
      m_RunningScans.erase(m_RunningScans.begin());

      // recompute stats of running scans
      frontScanPose = m_RunningScans.front()->GetSensorPose();
      backScanPose = m_RunningScans.back()->GetSensorPose();
      squaredDistance = frontScanPose.GetPosition().SquaredDistance(
          backScanPose.GetPosition());
    }
  }

  /**
   * Deletes data of this buffered device
   */
  void Clear() {
    m_Scans.clear();
    m_RunningScans.clear();
  }

private:
  LocalizedRangeScanVector m_Scans;
  LocalizedRangeScanVector m_RunningScans;
  LocalizedRangeScan *m_pLastScan;

  int32_t m_RunningBufferMaximumSize;
  double m_RunningBufferMaximumDistance;
}; // ScanManager
