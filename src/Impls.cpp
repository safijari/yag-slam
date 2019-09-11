#include "SensorData.h"
#include "OccupancyGrid.h"

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
#include "Object.h"

Object::Object() {}

Object::Object(const Name &rName) : m_Name(rName) {}

Object::~Object() {}


  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  void CellUpdater::operator() (uint32_t index)
  {
    uint8_t* pDataPtr = m_pOccupancyGrid->GetDataPointer();
    uint32_t* pCellPassCntPtr = m_pOccupancyGrid->m_pCellPassCnt->GetDataPointer();
    uint32_t* pCellHitCntPtr = m_pOccupancyGrid->m_pCellHitsCnt->GetDataPointer();

    m_pOccupancyGrid->UpdateCell(&pDataPtr[index], pCellPassCntPtr[index], pCellHitCntPtr[index]);
  }
