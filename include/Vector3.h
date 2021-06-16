#ifndef VECTOR3_H
#define VECTOR3_H

#include <iostream>
#include "AdditionalMath.h"

template <typename T> class Vector3 {
public:
  /**
   * Default constructor
   */
  Vector3() {
    m_Values[0] = 0;
    m_Values[1] = 0;
    m_Values[2] = 0;
  }

  /**
   * Constructor initializing point location
   * @param x
   * @param y
   * @param z
   */
  Vector3(T x, T y, T z) {
    m_Values[0] = x;
    m_Values[1] = y;
    m_Values[2] = z;
  }

  /**
   * Copy constructor
   * @param rOther
   */
  Vector3(const Vector3 &rOther) {
    m_Values[0] = rOther.m_Values[0];
    m_Values[1] = rOther.m_Values[1];
    m_Values[2] = rOther.m_Values[2];
  }

public:
  /**
   * Gets the x-component of this vector
   * @return x-component
   */
  inline const T &GetX() const { return m_Values[0]; }

  /**
   * Sets the x-component of this vector
   * @param x
   */
  inline void SetX(const T &x) { m_Values[0] = x; }

  /**
   * Gets the y-component of this vector
   * @return y-component
   */
  inline const T &GetY() const { return m_Values[1]; }

  /**
   * Sets the y-component of this vector
   * @param y
   */
  inline void SetY(const T &y) { m_Values[1] = y; }

  /**
   * Gets the z-component of this vector
   * @return z-component
   */
  inline const T &GetZ() const { return m_Values[2]; }

  /**
   * Sets the z-component of this vector
   * @param z
   */
  inline void SetZ(const T &z) { m_Values[2] = z; }

  /**
   * Floor vector operator
   * @param rOther Vector3d
   */
  inline void MakeFloor(const Vector3 &rOther) {
    if (rOther.m_Values[0] < m_Values[0])
      m_Values[0] = rOther.m_Values[0];
    if (rOther.m_Values[1] < m_Values[1])
      m_Values[1] = rOther.m_Values[1];
    if (rOther.m_Values[2] < m_Values[2])
      m_Values[2] = rOther.m_Values[2];
  }

  /**
   * Ceiling vector operator
   * @param rOther Vector3d
   */
  inline void MakeCeil(const Vector3 &rOther) {
    if (rOther.m_Values[0] > m_Values[0])
      m_Values[0] = rOther.m_Values[0];
    if (rOther.m_Values[1] > m_Values[1])
      m_Values[1] = rOther.m_Values[1];
    if (rOther.m_Values[2] > m_Values[2])
      m_Values[2] = rOther.m_Values[2];
  }

  /**
   * Returns the square of the length of the vector
   * @return square of the length of the vector
   */
  inline double SquaredLength() const {
    return amath::Square(m_Values[0]) + amath::Square(m_Values[1]) +
           amath::Square(m_Values[2]);
  }

  /**
   * Returns the length of the vector.
   * @return Length of the vector
   */
  inline double Length() const { return sqrt(SquaredLength()); }

  /**
   * Returns a string representation of this vector
   * @return string representation of this vector
   */
  inline std::string ToString() const {
    // std::stringstream converter;
    // converter.precision(std::numeric_limits<double>::digits10);

    // converter << GetX() << " " << GetY() << " " << GetZ();

    // return converter.str();
    return "Notimplemented";
  }

public:
  /**
   * Assignment operator
   */
  inline Vector3 &operator=(const Vector3 &rOther) {
    m_Values[0] = rOther.m_Values[0];
    m_Values[1] = rOther.m_Values[1];
    m_Values[2] = rOther.m_Values[2];

    return *this;
  }

  /**
   * Binary vector add.
   * @param rOther
   * @return vector sum
   */
  inline const Vector3 operator+(const Vector3 &rOther) const {
    return Vector3(m_Values[0] + rOther.m_Values[0],
                   m_Values[1] + rOther.m_Values[1],
                   m_Values[2] + rOther.m_Values[2]);
  }

  /**
   * Binary vector add.
   * @param scalar
   * @return sum
   */
  inline const Vector3 operator+(double scalar) const {
    return Vector3(m_Values[0] + scalar, m_Values[1] + scalar,
                   m_Values[2] + scalar);
  }

  /**
   * Binary vector subtract.
   * @param rOther
   * @return vector difference
   */
  inline const Vector3 operator-(const Vector3 &rOther) const {
    return Vector3(m_Values[0] - rOther.m_Values[0],
                   m_Values[1] - rOther.m_Values[1],
                   m_Values[2] - rOther.m_Values[2]);
  }

  /**
   * Binary vector subtract.
   * @param scalar
   * @return difference
   */
  inline const Vector3 operator-(double scalar) const {
    return Vector3(m_Values[0] - scalar, m_Values[1] - scalar,
                   m_Values[2] - scalar);
  }

  /**
   * Scales the vector by the given scalar
   * @param scalar
   */
  inline const Vector3 operator*(T scalar) const {
    return Vector3(m_Values[0] * scalar, m_Values[1] * scalar,
                   m_Values[2] * scalar);
  }

  /**
   * Equality operator returns true if the corresponding x, y, z values of each
   * Vector3 are the same values.
   * @param rOther
   */
  inline bool operator==(const Vector3 &rOther) const {
    return (m_Values[0] == rOther.m_Values[0] &&
            m_Values[1] == rOther.m_Values[1] &&
            m_Values[2] == rOther.m_Values[2]);
  }

  /**
   * Inequality operator returns true if any of the corresponding x, y, z values
   * of each Vector3 not the same.
   * @param rOther
   */
  inline bool operator!=(const Vector3 &rOther) const {
    return (m_Values[0] != rOther.m_Values[0] ||
            m_Values[1] != rOther.m_Values[1] ||
            m_Values[2] != rOther.m_Values[2]);
  }

  /**
   * Write Vector3 onto output stream
   * @param rStream output stream
   * @param rVector to write
   */
  friend inline std::ostream &operator<<(std::ostream &rStream,
                                         const Vector3 &rVector) {
    rStream << rVector.ToString();
    return rStream;
  }

  /**
   * Read Vector3 from input stream
   * @param rStream input stream
   */
  friend inline std::istream &operator>>(std::istream &rStream,
                                         const Vector3 & /*rVector*/) {
    // Implement me!!
    return rStream;
  }

private:
  T m_Values[3];
}; // Vector3

#endif
