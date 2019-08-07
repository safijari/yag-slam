#ifndef RECT2_H
#define RECT2_H
#include "Vector2.h"
#include "Size2.h"

template <typename T> class Rectangle2 {
public:
  /**
   * Default constructor
   */
  Rectangle2() {}

  /**
   * Constructor initializing rectangle parameters
   * @param x x-coordinate of left edge of rectangle
   * @param y y-coordinate of bottom edge of rectangle
   * @param width width of rectangle
   * @param height height of rectangle
   */
  Rectangle2(T x, T y, T width, T height)
      : m_Position(x, y), m_Size(width, height) {}

  /**
   * Constructor initializing rectangle parameters
   * @param rPosition (x,y)-coordinate of rectangle
   * @param rSize Size of the rectangle
   */
  Rectangle2(const Vector2<T> &rPosition, const Size2<T> &rSize)
      : m_Position(rPosition), m_Size(rSize) {}

  /**
   * Copy constructor
   */
  Rectangle2(const Rectangle2 &rOther)
      : m_Position(rOther.m_Position), m_Size(rOther.m_Size) {}

public:
  /**
   * Gets the x-coordinate of the left edge of this rectangle
   * @return the x-coordinate of the left edge of this rectangle
   */
  inline T GetX() const { return m_Position.GetX(); }

  /**
   * Sets the x-coordinate of the left edge of this rectangle
   * @param x the x-coordinate of the left edge of this rectangle
   */
  inline void SetX(T x) { m_Position.SetX(x); }

  /**
   * Gets the y-coordinate of the bottom edge of this rectangle
   * @return the y-coordinate of the bottom edge of this rectangle
   */
  inline T GetY() const { return m_Position.GetY(); }

  /**
   * Sets the y-coordinate of the bottom edge of this rectangle
   * @param y the y-coordinate of the bottom edge of this rectangle
   */
  inline void SetY(T y) { m_Position.SetY(y); }

  /**
   * Gets the width of this rectangle
   * @return the width of this rectangle
   */
  inline T GetWidth() const { return m_Size.GetWidth(); }

  /**
   * Sets the width of this rectangle
   * @param width the width of this rectangle
   */
  inline void SetWidth(T width) { m_Size.SetWidth(width); }

  /**
   * Gets the height of this rectangle
   * @return the height of this rectangle
   */
  inline T GetHeight() const { return m_Size.GetHeight(); }

  /**
   * Sets the height of this rectangle
   * @param height the height of this rectangle
   */
  inline void SetHeight(T height) { m_Size.SetHeight(height); }

  /**
   * Gets the position of this rectangle
   * @return the position of this rectangle
   */
  inline const Vector2<T> &GetPosition() const { return m_Position; }

  /**
   * Sets the position of this rectangle
   * @param rX x
   * @param rY y
   */
  inline void SetPosition(const T &rX, const T &rY) {
    m_Position = Vector2<T>(rX, rY);
  }

  /**
   * Sets the position of this rectangle
   * @param rPosition position
   */
  inline void SetPosition(const Vector2<T> &rPosition) {
    m_Position = rPosition;
  }

  /**
   * Gets the size of this rectangle
   * @return the size of this rectangle
   */
  inline const Size2<T> &GetSize() const { return m_Size; }

  /**
   * Sets the size of this rectangle
   * @param rSize size
   */
  inline void SetSize(const Size2<T> &rSize) { m_Size = rSize; }

  /**
   * Gets the center of this rectangle
   * @return the center of this rectangle
   */
  inline const Vector2<T> GetCenter() const {
    return Vector2<T>(m_Position.GetX() + m_Size.GetWidth() * 0.5,
                      m_Position.GetY() + m_Size.GetHeight() * 0.5);
  }

public:
  /**
   * Assignment operator
   */
  Rectangle2 &operator=(const Rectangle2 &rOther) {
    m_Position = rOther.m_Position;
    m_Size = rOther.m_Size;

    return *this;
  }

  /**
   * Equality operator
   */
  inline bool operator==(const Rectangle2 &rOther) const {
    return (m_Position == rOther.m_Position && m_Size == rOther.m_Size);
  }

  /**
   * Inequality operator
   */
  inline bool operator!=(const Rectangle2 &rOther) const {
    return (m_Position != rOther.m_Position || m_Size != rOther.m_Size);
  }

private:
  Vector2<T> m_Position;
  Size2<T> m_Size;
}; // Rectangle2

#endif
