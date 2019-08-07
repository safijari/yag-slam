#ifndef POSE_H
#define POSE_H

#include <vector>

#include "Vector2.h"
#include "Vector3.h"
#include "Quaternion.h"

class Pose3;

class Pose2 {
public:
  /**
   * Default Constructor
   */
  Pose2() : m_Heading(0.0) {}

  /**
   * Constructor initializing pose parameters
   * @param rPosition position
   * @param heading heading
   **/
  Pose2(const Vector2<double> &rPosition, double heading)
      : m_Position(rPosition), m_Heading(heading) {}

  /**
   * Constructor initializing pose parameters
   * @param x x-coordinate
   * @param y y-coordinate
   * @param heading heading
   **/
  Pose2(double x, double y, double heading)
      : m_Position(x, y), m_Heading(heading) {}

  /**
   * Constructs a Pose2 object from a Pose3.
   */
  Pose2(const Pose3 &rPose);

  /**
   * Copy constructor
   */
  Pose2(const Pose2 &rOther)
      : m_Position(rOther.m_Position), m_Heading(rOther.m_Heading) {}

public:
  /**
   * Returns the x-coordinate
   * @return the x-coordinate of the pose
   */
  inline double GetX() const { return m_Position.GetX(); }

  /**
   * Sets the x-coordinate
   * @param x the x-coordinate of the pose
   */
  inline void SetX(double x) { m_Position.SetX(x); }

  /**
   * Returns the y-coordinate
   * @return the y-coordinate of the pose
   */
  inline double GetY() const { return m_Position.GetY(); }

  /**
   * Sets the y-coordinate
   * @param y the y-coordinate of the pose
   */
  inline void SetY(double y) { m_Position.SetY(y); }

  /**
   * Returns the position
   * @return the position of the pose
   */
  inline const Vector2<double> &GetPosition() const { return m_Position; }

  /**
   * Sets the position
   * @param rPosition of the pose
   */
  inline void SetPosition(const Vector2<double> &rPosition) {
    m_Position = rPosition;
  }

  /**
   * Returns the heading of the pose (in radians)
   * @return the heading of the pose
   */
  inline double GetHeading() const { return m_Heading; }

  /**
   * Sets the heading
   * @param heading of the pose
   */
  inline void SetHeading(double heading) { m_Heading = heading; }

  /**
   * Return the squared distance between two Pose2
   * @return squared distance
   */
  inline double SquaredDistance(const Pose2 &rOther) const {
    return m_Position.SquaredDistance(rOther.m_Position);
  }

public:
  /**
   * Assignment operator
   */
  inline Pose2 &operator=(const Pose2 &rOther) {
    m_Position = rOther.m_Position;
    m_Heading = rOther.m_Heading;

    return *this;
  }

  /**
   * Equality operator
   */
  inline bool operator==(const Pose2 &rOther) const {
    return (m_Position == rOther.m_Position && m_Heading == rOther.m_Heading);
  }

  /**
   * Inequality operator
   */
  inline bool operator!=(const Pose2 &rOther) const {
    return (m_Position != rOther.m_Position || m_Heading != rOther.m_Heading);
  }

  /**
   * In place Pose2 add.
   */
  inline void operator+=(const Pose2 &rOther) {
    m_Position += rOther.m_Position;
    m_Heading = amath::NormalizeAngle(m_Heading + rOther.m_Heading);
  }

  /**
   * Binary Pose2 add
   * @param rOther
   * @return Pose2 sum
   */
  inline Pose2 operator+(const Pose2 &rOther) const {
    return Pose2(m_Position + rOther.m_Position,
                 amath::NormalizeAngle(m_Heading + rOther.m_Heading));
  }

  /**
   * Binary Pose2 subtract
   * @param rOther
   * @return Pose2 difference
   */
  inline Pose2 operator-(const Pose2 &rOther) const {
    return Pose2(m_Position - rOther.m_Position,
                 amath::NormalizeAngle(m_Heading - rOther.m_Heading));
  }

  /**
   * Read pose from input stream
   * @param rStream input stream
   */
  friend inline std::istream &operator>>(std::istream &rStream,
                                         const Pose2 & /*rPose*/) {
    // Implement me!!
    return rStream;
  }

  /**
   * Write this pose onto output stream
   * @param rStream output stream
   * @param rPose to read
   */
  friend inline std::ostream &operator<<(std::ostream &rStream,
                                         const Pose2 &rPose) {
    rStream << rPose.m_Position.GetX() << " " << rPose.m_Position.GetY() << " "
            << rPose.m_Heading;
    return rStream;
  }

private:
  Vector2<double> m_Position;

  double m_Heading;
}; // Pose2

/**
 * Type declaration of Pose2 vector
 */
typedef std::vector<Pose2> Pose2Vector;

/**
 * Defines a position and orientation in 3-dimensional space.
 * Karto uses a right-handed coordinate system with X, Y as the 2-D ground plane
 * and X is forward and Y is left. Values in Vector3 used to define position
 * must have units of meters. The value of angle when defining orientation in
 * two dimensions must be in units of radians. The definition of orientation in
 * three dimensions uses quaternions.
 */
class Pose3 {
public:
  /**
   * Default constructor
   */
  Pose3() {}

  /**
   * Create a new Pose3 object from the given position.
   * @param rPosition position vector in three space.
   */
  Pose3(const Vector3<double> &rPosition) : m_Position(rPosition) {}

  /**
   * Create a new Pose3 object from the given position and orientation.
   * @param rPosition position vector in three space.
   * @param rOrientation quaternion orientation in three space.
   */
  Pose3(const Vector3<double> &rPosition, const Quaternion &rOrientation)
      : m_Position(rPosition), m_Orientation(rOrientation) {}

  /**
   * Copy constructor
   */
  Pose3(const Pose3 &rOther)
      : m_Position(rOther.m_Position), m_Orientation(rOther.m_Orientation) {}

  /**
   * Constructs a Pose3 object from a Pose2.
   */
  Pose3(const Pose2 &rPose) {
    m_Position = Vector3<double>(rPose.GetX(), rPose.GetY(), 0.0);
    m_Orientation.FromEulerAngles(rPose.GetHeading(), 0.0, 0.0);
  }

public:
  /**
   * Get the position of the pose as a 3D vector as const. Values have units of
   * meters.
   * @return 3-dimensional position vector as const
   */
  inline const Vector3<double> &GetPosition() const { return m_Position; }

  /**
   * Set the position of the pose as a 3D vector. Values have units of meters.
   * @return 3-dimensional position vector
   */
  inline void SetPosition(const Vector3<double> &rPosition) {
    m_Position = rPosition;
  }

  /**
   * Get the orientation quaternion of the pose as const.
   * @return orientation quaternion as const
   */
  inline const Quaternion &GetOrientation() const { return m_Orientation; }

  /**
   * Get the orientation quaternion of the pose.
   * @return orientation quaternion
   */
  inline void SetOrientation(const Quaternion &rOrientation) {
    m_Orientation = rOrientation;
  }

  /**
   * Returns a string representation of this pose
   * @return string representation of this pose
   */
  inline std::string ToString() {
    // std::stringstream converter;
    // converter.precision(std::numeric_limits<double>::digits10);

    // converter << GetPosition() << " " << GetOrientation();

    // return converter.str();
    return "Notimplemented";
  }

public:
  /**
   * Assignment operator
   */
  inline Pose3 &operator=(const Pose3 &rOther) {
    m_Position = rOther.m_Position;
    m_Orientation = rOther.m_Orientation;

    return *this;
  }

  /**
   * Equality operator
   */
  inline bool operator==(const Pose3 &rOther) const {
    return (m_Position == rOther.m_Position &&
            m_Orientation == rOther.m_Orientation);
  }

  /**
   * Inequality operator
   */
  inline bool operator!=(const Pose3 &rOther) const {
    return (m_Position != rOther.m_Position ||
            m_Orientation != rOther.m_Orientation);
  }

  /**
   * Write Pose3 onto output stream
   * @param rStream output stream
   * @param rPose to write
   */
  friend inline std::ostream &operator<<(std::ostream &rStream,
                                         const Pose3 &rPose) {
    rStream << rPose.GetPosition() << ", " << rPose.GetOrientation();
    return rStream;
  }

  /**
   * Read Pose3 from input stream
   * @param rStream input stream
   */
  friend inline std::istream &operator>>(std::istream &rStream,
                                         const Pose3 & /*rPose*/) {
    // Implement me!!
    return rStream;
  }

private:
  Vector3<double> m_Position;
  Quaternion m_Orientation;
}; // Pose3

#endif
