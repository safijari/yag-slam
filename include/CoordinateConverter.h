#ifndef COORD_CVTR_H
#define COORD_CVTR_H

#include "BoundingBox2.h"
#include "AdditionalMath.h"

/**
 * The CoordinateConverter class is used to convert coordinates between world
 * and grid coordinates In world coordinates 1.0 = 1 meter where 1 in grid
 * coordinates = 1 pixel! Default scale for coordinate converter is 20 that
 * converters to 1 pixel = 0.05 meter
 */
class CoordinateConverter {
public:
  /**
   * Default constructor
   */
  CoordinateConverter() : m_Scale(20.0) {}

public:
  /**
   * Scales the value
   * @param value
   * @return scaled value
   */
  inline double Transform(double value) { return value * m_Scale; }

  /**
   * Converts the point from world coordinates to grid coordinates
   * @param rWorld world coordinate
   * @param flipY
   * @return grid coordinate
   */
  inline Vector2<int32_t> WorldToGrid(const Vector2<double> &rWorld,
                                      bool flipY = false) const {
    double gridX = (rWorld.GetX() - m_Offset.GetX()) * m_Scale;
    double gridY = 0.0;

    if (flipY == false) {
      gridY = (rWorld.GetY() - m_Offset.GetY()) * m_Scale;
    } else {
      gridY = (m_Size.GetHeight() / m_Scale - rWorld.GetY() + m_Offset.GetY()) *
              m_Scale;
    }

    return Vector2<int32_t>(static_cast<int32_t>(amath::Round(gridX)),
                            static_cast<int32_t>(amath::Round(gridY)));
  }

  /**
   * Converts the point from grid coordinates to world coordinates
   * @param rGrid world coordinate
   * @param flipY
   * @return world coordinate
   */
  inline Vector2<double> GridToWorld(const Vector2<int32_t> &rGrid,
                                     bool flipY = false) const {
    double worldX = m_Offset.GetX() + rGrid.GetX() / m_Scale;
    double worldY = 0.0;

    if (flipY == false) {
      worldY = m_Offset.GetY() + rGrid.GetY() / m_Scale;
    } else {
      worldY = m_Offset.GetY() + (m_Size.GetHeight() - rGrid.GetY()) / m_Scale;
    }

    return Vector2<double>(worldX, worldY);
  }

  /**
   * Gets the scale
   * @return scale
   */
  inline double GetScale() const { return m_Scale; }

  /**
   * Sets the scale
   * @param scale
   */
  inline void SetScale(double scale) { m_Scale = scale; }

  /**
   * Gets the offset
   * @return offset
   */
  inline const Vector2<double> &GetOffset() const { return m_Offset; }

  /**
   * Sets the offset
   * @param rOffset
   */
  inline void SetOffset(const Vector2<double> &rOffset) { m_Offset = rOffset; }

  /**
   * Sets the size
   * @param rSize
   */
  inline void SetSize(const Size2<int32_t> &rSize) { m_Size = rSize; }

  /**
   * Gets the size
   * @return size
   */
  inline const Size2<int32_t> &GetSize() const { return m_Size; }

  /**
   * Gets the resolution
   * @return resolution
   */
  inline double GetResolution() const { return 1.0 / m_Scale; }

  /**
   * Sets the resolution
   * @param resolution
   */
  inline void SetResolution(double resolution) { m_Scale = 1.0 / resolution; }

  /**
   * Gets the bounding box
   * @return bounding box
   */
  inline BoundingBox2 GetBoundingBox() const {
    BoundingBox2 box;

    double minX = GetOffset().GetX();
    double minY = GetOffset().GetY();
    double maxX = minX + GetSize().GetWidth() * GetResolution();
    double maxY = minY + GetSize().GetHeight() * GetResolution();

    box.SetMinimum(GetOffset());
    box.SetMaximum(Vector2<double>(maxX, maxY));
    return box;
  }

private:
  Size2<int32_t> m_Size;
  double m_Scale;

  Vector2<double> m_Offset;
}; // CoordinateConverter

#endif
