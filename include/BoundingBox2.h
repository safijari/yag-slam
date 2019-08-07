#ifndef BBOX2_H
#define BBOX2_H

#include <vector>
#include "Vector2.h"
#include "Size2.h"

class BoundingBox2 {
public:
  /*
   * Default constructor
   */
  BoundingBox2()
      : m_Minimum(999999999999999999.99999, 999999999999999999.99999),
        m_Maximum(-999999999999999999.99999, -999999999999999999.99999) {}

public:
  /**
   * Get bounding box minimum
   */
  inline const Vector2<double> &GetMinimum() const { return m_Minimum; }

  /**
   * Set bounding box minimum
   */
  inline void SetMinimum(const Vector2<double> &mMinimum) {
    m_Minimum = mMinimum;
  }

  /**
   * Get bounding box maximum
   */
  inline const Vector2<double> &GetMaximum() const { return m_Maximum; }

  /**
   * Set bounding box maximum
   */
  inline void SetMaximum(const Vector2<double> &rMaximum) {
    m_Maximum = rMaximum;
  }

  /**
   * Get the size of the bounding box
   */
  inline Size2<double> GetSize() const {
    Vector2<double> size = m_Maximum - m_Minimum;

    return Size2<double>(size.GetX(), size.GetY());
  }

  /**
   * Add vector to bounding box
   */
  inline void Add(const Vector2<double> &rPoint) {
    m_Minimum.MakeFloor(rPoint);
    m_Maximum.MakeCeil(rPoint);
  }

  /**
   * Add other bounding box to bounding box
   */
  inline void Add(const BoundingBox2 &rBoundingBox) {
    Add(rBoundingBox.GetMinimum());
    Add(rBoundingBox.GetMaximum());
  }

  /**
   * Whether the given point is in the bounds of this box
   * @param rPoint
   * @return in bounds?
   */
  inline bool IsInBounds(const Vector2<double> &rPoint) const {
    return (amath::InRange(rPoint.GetX(), m_Minimum.GetX(), m_Maximum.GetX()) &&
            amath::InRange(rPoint.GetY(), m_Minimum.GetY(), m_Maximum.GetY()));
  }

private:
  Vector2<double> m_Minimum;
  Vector2<double> m_Maximum;
}; // BoundingBox2

#endif
