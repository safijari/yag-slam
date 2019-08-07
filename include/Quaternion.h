#ifndef QUATERNION_H
#define QUATERNION_H

#include <iostream>
#include <math.h>

#include "AdditionalMath.h"

class Quaternion {
public:
  /**
   * Create a quaternion with default (x=0, y=0, z=0, w=1) values
   */
  inline Quaternion() {
    m_Values[0] = 0.0;
    m_Values[1] = 0.0;
    m_Values[2] = 0.0;
    m_Values[3] = 1.0;
  }

  /**
   * Create a quaternion using x, y, z, w values.
   * @param x
   * @param y
   * @param z
   * @param w
   */
  inline Quaternion(double x, double y, double z, double w) {
    m_Values[0] = x;
    m_Values[1] = y;
    m_Values[2] = z;
    m_Values[3] = w;
  }

  /**
   * Copy constructor
   */
  inline Quaternion(const Quaternion &rQuaternion) {
    m_Values[0] = rQuaternion.m_Values[0];
    m_Values[1] = rQuaternion.m_Values[1];
    m_Values[2] = rQuaternion.m_Values[2];
    m_Values[3] = rQuaternion.m_Values[3];
  }

public:
  /**
   * Returns the X-value
   * @return Return the X-value of the quaternion
   */
  inline double GetX() const { return m_Values[0]; }

  /**
   * Sets the X-value
   * @param x X-value of the quaternion
   */
  inline void SetX(double x) { m_Values[0] = x; }

  /**
   * Returns the Y-value
   * @return Return the Y-value of the quaternion
   */
  inline double GetY() const { return m_Values[1]; }

  /**
   * Sets the Y-value
   * @param y Y-value of the quaternion
   */
  inline void SetY(double y) { m_Values[1] = y; }

  /**
   * Returns the Z-value
   * @return Return the Z-value of the quaternion
   */
  inline double GetZ() const { return m_Values[2]; }

  /**
   * Sets the Z-value
   * @param z Z-value of the quaternion
   */
  inline void SetZ(double z) { m_Values[2] = z; }

  /**
   * Returns the W-value
   * @return Return the W-value of the quaternion
   */
  inline double GetW() const { return m_Values[3]; }

  /**
   * Sets the W-value
   * @param w W-value of the quaternion
   */
  inline void SetW(double w) { m_Values[3] = w; }

  /**
   * Converts this quaternion into Euler angles
   * Source:
   * http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
   * @param rYaw
   * @param rPitch
   * @param rRoll
   */
  void ToEulerAngles(double &rYaw, double &rPitch, double &rRoll) const {
    double test = m_Values[0] * m_Values[1] + m_Values[2] * m_Values[3];

    if (test > 0.499) {
      // singularity at north pole
      rYaw = 2 * atan2(m_Values[0], m_Values[3]);
      ;
      rPitch = KT_PI_2;
      rRoll = 0;
    } else if (test < -0.499) {
      // singularity at south pole
      rYaw = -2 * atan2(m_Values[0], m_Values[3]);
      rPitch = -KT_PI_2;
      rRoll = 0;
    } else {
      double sqx = m_Values[0] * m_Values[0];
      double sqy = m_Values[1] * m_Values[1];
      double sqz = m_Values[2] * m_Values[2];

      rYaw =
          atan2(2 * m_Values[1] * m_Values[3] - 2 * m_Values[0] * m_Values[2],
                1 - 2 * sqy - 2 * sqz);
      rPitch = asin(2 * test);
      rRoll =
          atan2(2 * m_Values[0] * m_Values[3] - 2 * m_Values[1] * m_Values[2],
                1 - 2 * sqx - 2 * sqz);
    }
  }

  /**
   * Set x,y,z,w values of the quaternion based on Euler angles.
   * Source:
   * http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
   * @param yaw
   * @param pitch
   * @param roll
   */
  void FromEulerAngles(double yaw, double pitch, double roll) {
    double angle;

    angle = yaw * 0.5;
    double cYaw = cos(angle);
    double sYaw = sin(angle);

    angle = pitch * 0.5;
    double cPitch = cos(angle);
    double sPitch = sin(angle);

    angle = roll * 0.5;
    double cRoll = cos(angle);
    double sRoll = sin(angle);

    m_Values[0] = sYaw * sPitch * cRoll + cYaw * cPitch * sRoll;
    m_Values[1] = sYaw * cPitch * cRoll + cYaw * sPitch * sRoll;
    m_Values[2] = cYaw * sPitch * cRoll - sYaw * cPitch * sRoll;
    m_Values[3] = cYaw * cPitch * cRoll - sYaw * sPitch * sRoll;
  }

  /**
   * Assignment operator
   * @param rQuaternion
   */
  inline Quaternion &operator=(const Quaternion &rQuaternion) {
    m_Values[0] = rQuaternion.m_Values[0];
    m_Values[1] = rQuaternion.m_Values[1];
    m_Values[2] = rQuaternion.m_Values[2];
    m_Values[3] = rQuaternion.m_Values[3];

    return (*this);
  }

  /**
   * Equality operator returns true if the corresponding x, y, z, w values of
   * each quaternion are the same values.
   * @param rOther
   */
  inline bool operator==(const Quaternion &rOther) const {
    return (m_Values[0] == rOther.m_Values[0] &&
            m_Values[1] == rOther.m_Values[1] &&
            m_Values[2] == rOther.m_Values[2] &&
            m_Values[3] == rOther.m_Values[3]);
  }

  /**
   * Inequality operator returns true if any of the corresponding x, y, z, w
   * values of each quaternion not the same.
   * @param rOther
   */
  inline bool operator!=(const Quaternion &rOther) const {
    return (m_Values[0] != rOther.m_Values[0] ||
            m_Values[1] != rOther.m_Values[1] ||
            m_Values[2] != rOther.m_Values[2] ||
            m_Values[3] != rOther.m_Values[3]);
  }

  /**
   * Write this quaternion onto output stream
   * @param rStream output stream
   * @param rQuaternion
   */
  friend inline std::ostream &operator<<(std::ostream &rStream,
                                         const Quaternion &rQuaternion) {
    rStream << rQuaternion.m_Values[0] << " " << rQuaternion.m_Values[1] << " "
            << rQuaternion.m_Values[2] << " " << rQuaternion.m_Values[3];
    return rStream;
  }

private:
  double m_Values[4];
};

#endif
