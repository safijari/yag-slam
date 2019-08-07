#ifndef VECTOR2_H
#define VECTOR2_H

#include <iostream>
#include <vector>
#include "AdditionalMath.h"

template <typename T> class Vector2 {
public:
  /**
   * Default constructor
   */
  Vector2() {
    m_Values[0] = 0;
    m_Values[1] = 0;
  }

  /**
   * Constructor initializing vector location
   * @param x
   * @param y
   */
  Vector2(T x, T y) {
    m_Values[0] = x;
    m_Values[1] = y;
  }

public:
  /**
   * Gets the x-coordinate of this vector2
   * @return the x-coordinate of the vector2
   */
  inline const T &GetX() const { return m_Values[0]; }

  /**
   * Sets the x-coordinate of this vector2
   * @param x the x-coordinate of the vector2
   */
  inline void SetX(const T &x) { m_Values[0] = x; }

  /**
   * Gets the y-coordinate of this vector2
   * @return the y-coordinate of the vector2
   */
  inline const T &GetY() const { return m_Values[1]; }

  /**
   * Sets the y-coordinate of this vector2
   * @param y the y-coordinate of the vector2
   */
  inline void SetY(const T &y) { m_Values[1] = y; }

  /**
   * Floor point operator
   * @param rOther
   */
  inline void MakeFloor(const Vector2 &rOther) {
    if (rOther.m_Values[0] < m_Values[0])
      m_Values[0] = rOther.m_Values[0];
    if (rOther.m_Values[1] < m_Values[1])
      m_Values[1] = rOther.m_Values[1];
  }

  /**
   * Ceiling point operator
   * @param rOther
   */
  inline void MakeCeil(const Vector2 &rOther) {
    if (rOther.m_Values[0] > m_Values[0])
      m_Values[0] = rOther.m_Values[0];
    if (rOther.m_Values[1] > m_Values[1])
      m_Values[1] = rOther.m_Values[1];
  }

  /**
   * Returns the square of the length of the vector
   * @return square of the length of the vector
   */
  inline double SquaredLength() const {
    return amath::Square(m_Values[0]) + amath::Square(m_Values[1]);
  }

  /**
   * Returns the length of the vector (x and y).
   * @return length of the vector
   */
  inline double Length() const { return sqrt(SquaredLength()); }

  /**
   * Returns the square distance to the given vector
   * @returns square distance to the given vector
   */
  inline double SquaredDistance(const Vector2 &rOther) const {
    return (*this - rOther).SquaredLength();
  }

  /**
   * Gets the distance to the other vector2
   * @param rOther
   * @return distance to other vector2
   */
  inline double Distance(const Vector2 &rOther) const {
    return sqrt(SquaredDistance(rOther));
  }

public:
  /**
   * In place Vector2 addition.
   */
  inline void operator+=(const Vector2 &rOther) {
    m_Values[0] += rOther.m_Values[0];
    m_Values[1] += rOther.m_Values[1];
  }

  /**
   * In place Vector2 subtraction.
   */
  inline void operator-=(const Vector2 &rOther) {
    m_Values[0] -= rOther.m_Values[0];
    m_Values[1] -= rOther.m_Values[1];
  }

  /**
   * Addition operator
   * @param rOther
   * @return vector resulting from adding this vector with the given vector
   */
  inline const Vector2 operator+(const Vector2 &rOther) const {
    return Vector2(m_Values[0] + rOther.m_Values[0],
                   m_Values[1] + rOther.m_Values[1]);
  }

  /**
   * Subtraction operator
   * @param rOther
   * @return vector resulting from subtracting this vector from the given vector
   */
  inline const Vector2 operator-(const Vector2 &rOther) const {
    return Vector2(m_Values[0] - rOther.m_Values[0],
                   m_Values[1] - rOther.m_Values[1]);
  }

  /**
   * In place scalar division operator
   * @param scalar
   */
  inline void operator/=(T scalar) {
    m_Values[0] /= scalar;
    m_Values[1] /= scalar;
  }

  /**
   * Divides a Vector2
   * @param scalar
   * @return scalar product
   */
  inline const Vector2 operator/(T scalar) const {
    return Vector2(m_Values[0] / scalar, m_Values[1] / scalar);
  }

  /**
   * Computes the dot product between the two vectors
   * @param rOther
   * @return dot product
   */
  inline double operator*(const Vector2 &rOther) const {
    return m_Values[0] * rOther.m_Values[0] + m_Values[1] * rOther.m_Values[1];
  }

  /**
   * Scales the vector by the given scalar
   * @param scalar
   */
  inline const Vector2 operator*(T scalar) const {
    return Vector2(m_Values[0] * scalar, m_Values[1] * scalar);
  }

  /**
   * Subtract the vector by the given scalar
   * @param scalar
   */
  inline const Vector2 operator-(T scalar) const {
    return Vector2(m_Values[0] - scalar, m_Values[1] - scalar);
  }

  /**
   * In place scalar multiplication operator
   * @param scalar
   */
  inline void operator*=(T scalar) {
    m_Values[0] *= scalar;
    m_Values[1] *= scalar;
  }

  /**
   * Equality operator returns true if the corresponding x, y values of each
   * Vector2 are the same values.
   * @param rOther
   */
  inline bool operator==(const Vector2 &rOther) const {
    return (m_Values[0] == rOther.m_Values[0] &&
            m_Values[1] == rOther.m_Values[1]);
  }

  /**
   * Inequality operator returns true if any of the corresponding x, y values of
   * each Vector2 not the same.
   * @param rOther
   */
  inline bool operator!=(const Vector2 &rOther) const {
    return (m_Values[0] != rOther.m_Values[0] ||
            m_Values[1] != rOther.m_Values[1]);
  }

  /**
   * Less than operator
   * @param rOther
   * @return true if left vector is less than right vector
   */
  inline bool operator<(const Vector2 &rOther) const {
    if (m_Values[0] < rOther.m_Values[0])
      return true;
    else if (m_Values[0] > rOther.m_Values[0])
      return false;
    else
      return (m_Values[1] < rOther.m_Values[1]);
  }

  /**
   * Write Vector2 onto output stream
   * @param rStream output stream
   * @param rVector to write
   */
  friend inline std::ostream &operator<<(std::ostream &rStream,
                                         const Vector2 &rVector) {
    rStream << rVector.GetX() << " " << rVector.GetY();
    return rStream;
  }

  /**
   * Read Vector2 from input stream
   * @param rStream input stream
   */
  friend inline std::istream &operator>>(std::istream &rStream,
                                         const Vector2 & /*rVector*/) {
    // Implement me!!  TODO(lucbettaieb): What the what?  Do I need to implement
    // this?
    return rStream;
  }

private:
  T m_Values[2];
}; // Vector2<T>

/**
 * Type declaration of Vector2<double> vector
 */
typedef std::vector<Vector2<double>> PointVectorDouble;

#endif
