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
