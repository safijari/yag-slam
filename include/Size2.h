#ifndef SIZE2_H
#define SIZE2_H

#include <iostream>

template <typename T> class Size2 {
public:
  /**
   * Default constructor
   */
  Size2() : m_Width(0), m_Height(0) {}

  /**
   * Constructor initializing point location
   * @param width
   * @param height
   */
  Size2(T width, T height) : m_Width(width), m_Height(height) {}

  /**
   * Copy constructor
   * @param rOther
   */
  Size2(const Size2 &rOther)
      : m_Width(rOther.m_Width), m_Height(rOther.m_Height) {}

public:
  /**
   * Gets the width
   * @return the width
   */
  inline const T GetWidth() const { return m_Width; }

  /**
   * Sets the width
   * @param width
   */
  inline void SetWidth(T width) { m_Width = width; }

  /**
   * Gets the height
   * @return the height
   */
  inline const T GetHeight() const { return m_Height; }

  /**
   * Sets the height
   * @param height
   */
  inline void SetHeight(T height) { m_Height = height; }

  /**
   * Assignment operator
   */
  inline Size2 &operator=(const Size2 &rOther) {
    m_Width = rOther.m_Width;
    m_Height = rOther.m_Height;

    return (*this);
  }

  /**
   * Equality operator
   */
  inline bool operator==(const Size2 &rOther) const {
    return (m_Width == rOther.m_Width && m_Height == rOther.m_Height);
  }

  /**
   * Inequality operator
   */
  inline bool operator!=(const Size2 &rOther) const {
    return (m_Width != rOther.m_Width || m_Height != rOther.m_Height);
  }

  /**
   * Write Size2 onto output stream
   * @param rStream output stream
   * @param rSize to write
   */
  friend inline std::ostream &operator<<(std::ostream &rStream,
                                         const Size2 &rSize) {
    rStream << "(" << rSize.m_Width << ", " << rSize.m_Height << ")";
    return rStream;
  }

private:
  T m_Width;
  T m_Height;
}; // Size2<T>

#endif
