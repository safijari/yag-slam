#ifndef MATRIX_H
#define MATRIX_H

#include <iostream>
#include <sstream>
#include <cstring>
#include <math.h>
#include <assert.h>

#include "AdditionalMath.h"
#include "Pose.h"

class Matrix3 {
public:
  /**
   * Default constructor
   */
  Matrix3() { Clear(); }

  /**
   * Copy constructor
   */
  inline Matrix3(const Matrix3 &rOther) {
    std::memcpy(m_Matrix, rOther.m_Matrix, 9 * sizeof(double));
  }

public:
  /**
   * Sets this matrix to identity matrix
   */
  void SetToIdentity() {
    std::memset(m_Matrix, 0, 9 * sizeof(double));

    for (int32_t i = 0; i < 3; i++) {
      m_Matrix[i][i] = 1.0;
    }
  }

  /**
   * Sets this matrix to zero matrix
   */
  void Clear() { memset(m_Matrix, 0, 9 * sizeof(double)); }

  /**
   * Sets this matrix to be the rotation matrix of rotation around given axis
   * @param x x-coordinate of axis
   * @param y y-coordinate of axis
   * @param z z-coordinate of axis
   * @param radians amount of rotation
   */
  void FromAxisAngle(double x, double y, double z, const double radians) {
    double cosRadians = cos(radians);
    double sinRadians = sin(radians);
    double oneMinusCos = 1.0 - cosRadians;

    double xx = x * x;
    double yy = y * y;
    double zz = z * z;

    double xyMCos = x * y * oneMinusCos;
    double xzMCos = x * z * oneMinusCos;
    double yzMCos = y * z * oneMinusCos;

    double xSin = x * sinRadians;
    double ySin = y * sinRadians;
    double zSin = z * sinRadians;

    m_Matrix[0][0] = xx * oneMinusCos + cosRadians;
    m_Matrix[0][1] = xyMCos - zSin;
    m_Matrix[0][2] = xzMCos + ySin;

    m_Matrix[1][0] = xyMCos + zSin;
    m_Matrix[1][1] = yy * oneMinusCos + cosRadians;
    m_Matrix[1][2] = yzMCos - xSin;

    m_Matrix[2][0] = xzMCos - ySin;
    m_Matrix[2][1] = yzMCos + xSin;
    m_Matrix[2][2] = zz * oneMinusCos + cosRadians;
  }

  /**
   * Returns transposed version of this matrix
   * @return transposed matrix
   */
  Matrix3 Transpose() const {
    Matrix3 transpose;

    for (uint32_t row = 0; row < 3; row++) {
      for (uint32_t col = 0; col < 3; col++) {
        transpose.m_Matrix[row][col] = m_Matrix[col][row];
      }
    }

    return transpose;
  }

  /**
   * Returns the inverse of the matrix
   */
  Matrix3 Inverse() const {
    Matrix3 kInverse = *this;
    bool haveInverse = InverseFast(kInverse, 1e-14);
    if (haveInverse == false) {
      assert(false);
    }
    return kInverse;
  }

  /**
   * Internal helper method for inverse matrix calculation
   * This code is lifted from the OgreMatrix3 class!!
   */
  bool InverseFast(Matrix3 &rkInverse,
                      double fTolerance = KT_TOLERANCE) const {
    // Invert a 3x3 using cofactors.  This is about 8 times faster than
    // the Numerical Recipes code which uses Gaussian elimination.
    rkInverse.m_Matrix[0][0] =
        m_Matrix[1][1] * m_Matrix[2][2] - m_Matrix[1][2] * m_Matrix[2][1];
    rkInverse.m_Matrix[0][1] =
        m_Matrix[0][2] * m_Matrix[2][1] - m_Matrix[0][1] * m_Matrix[2][2];
    rkInverse.m_Matrix[0][2] =
        m_Matrix[0][1] * m_Matrix[1][2] - m_Matrix[0][2] * m_Matrix[1][1];
    rkInverse.m_Matrix[1][0] =
        m_Matrix[1][2] * m_Matrix[2][0] - m_Matrix[1][0] * m_Matrix[2][2];
    rkInverse.m_Matrix[1][1] =
        m_Matrix[0][0] * m_Matrix[2][2] - m_Matrix[0][2] * m_Matrix[2][0];
    rkInverse.m_Matrix[1][2] =
        m_Matrix[0][2] * m_Matrix[1][0] - m_Matrix[0][0] * m_Matrix[1][2];
    rkInverse.m_Matrix[2][0] =
        m_Matrix[1][0] * m_Matrix[2][1] - m_Matrix[1][1] * m_Matrix[2][0];
    rkInverse.m_Matrix[2][1] =
        m_Matrix[0][1] * m_Matrix[2][0] - m_Matrix[0][0] * m_Matrix[2][1];
    rkInverse.m_Matrix[2][2] =
        m_Matrix[0][0] * m_Matrix[1][1] - m_Matrix[0][1] * m_Matrix[1][0];

    double fDet = m_Matrix[0][0] * rkInverse.m_Matrix[0][0] +
                  m_Matrix[0][1] * rkInverse.m_Matrix[1][0] +
                  m_Matrix[0][2] * rkInverse.m_Matrix[2][0];

    if (fabs(fDet) <= fTolerance) {
      return false;
    }

    double fInvDet = 1.0 / fDet;
    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 3; col++) {
        rkInverse.m_Matrix[row][col] *= fInvDet;
      }
    }

    return true;
  }

  /**
   * Returns a string representation of this matrix
   * @return string representation of this matrix
   */
  inline std::string ToString() const {
    std::stringstream converter;
    converter.precision(std::numeric_limits<double>::digits10);

    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        converter << m_Matrix[row][col] << " ";
      }
    }

    return converter.str();
  }

public:
  /**
   * Assignment operator
   */
  inline Matrix3 &operator=(const Matrix3 &rOther) {
    std::memcpy(m_Matrix, rOther.m_Matrix, 9 * sizeof(double));
    return *this;
  }

  /**
   * Matrix element access, allows use of construct mat(r, c)
   * @param row
   * @param column
   * @return reference to mat(r,c)
   */
  inline double &operator()(uint32_t row, uint32_t column) {
    return m_Matrix[row][column];
  }

  /**
   * Read-only matrix element access, allows use of construct mat(r, c)
   * @param row
   * @param column
   * @return mat(r,c)
   */
  inline double operator()(uint32_t row, uint32_t column) const {
    return m_Matrix[row][column];
  }

  /**
   * Binary Matrix3 multiplication.
   * @param rOther
   * @return Matrix3 product
   */
  Matrix3 operator*(const Matrix3 &rOther) const {
    Matrix3 product;

    for (size_t row = 0; row < 3; row++) {
      for (size_t col = 0; col < 3; col++) {
        product.m_Matrix[row][col] =
            m_Matrix[row][0] * rOther.m_Matrix[0][col] +
            m_Matrix[row][1] * rOther.m_Matrix[1][col] +
            m_Matrix[row][2] * rOther.m_Matrix[2][col];
      }
    }

    return product;
  }

  /**
   * Matrix3 and Pose2 multiplication - matrix * pose [3x3 * 3x1 = 3x1]
   * @param rPose2
   * @return Pose2 product
   */
  inline Pose2 operator*(const Pose2 &rPose2) const {
    Pose2 pose2;

    pose2.SetX(m_Matrix[0][0] * rPose2.GetX() + m_Matrix[0][1] * rPose2.GetY() +
               m_Matrix[0][2] * rPose2.GetHeading());
    pose2.SetY(m_Matrix[1][0] * rPose2.GetX() + m_Matrix[1][1] * rPose2.GetY() +
               m_Matrix[1][2] * rPose2.GetHeading());
    pose2.SetHeading(m_Matrix[2][0] * rPose2.GetX() +
                     m_Matrix[2][1] * rPose2.GetY() +
                     m_Matrix[2][2] * rPose2.GetHeading());

    return pose2;
  }

  /**
   * In place Matrix3 add.
   * @param rkMatrix
   */
  inline void operator+=(const Matrix3 &rkMatrix) {
    for (uint32_t row = 0; row < 3; row++) {
      for (uint32_t col = 0; col < 3; col++) {
        m_Matrix[row][col] += rkMatrix.m_Matrix[row][col];
      }
    }
  }

  /**
   * Write Matrix3 onto output stream
   * @param rStream output stream
   * @param rMatrix to write
   */
  friend inline std::ostream &operator<<(std::ostream &rStream,
                                         const Matrix3 &rMatrix) {
    rStream << rMatrix.ToString();
    return rStream;
  }

// private:
  double m_Matrix[3][3];
}; // Matrix3

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

/**
 * Defines a general Matrix class.
 */
class Matrix {
public:
  /**
   * Constructs a matrix of size rows x columns
   */
  Matrix(uint32_t rows, uint32_t columns)
      : m_Rows(rows), m_Columns(columns), m_pData(NULL) {
    Allocate();

    Clear();
  }

  /**
   * Destructor
   */
  virtual ~Matrix() { delete[] m_pData; }

public:
  /**
   * Set all entries to 0
   */
  void Clear() {
    if (m_pData != NULL) {
      memset(m_pData, 0, sizeof(double) * m_Rows * m_Columns);
    }
  }

  /**
   * Gets the number of rows of the matrix
   * @return nubmer of rows
   */
  inline uint32_t GetRows() const { return m_Rows; }

  /**
   * Gets the number of columns of the matrix
   * @return nubmer of columns
   */
  inline uint32_t GetColumns() const { return m_Columns; }

  /**
   * Returns a reference to the entry at (row,column)
   * @param row
   * @param column
   * @return reference to entry at (row,column)
   */
  inline double &operator()(uint32_t row, uint32_t column) {
    RangeCheck(row, column);

    return m_pData[row + column * m_Rows];
  }

  /**
   * Returns a const reference to the entry at (row,column)
   * @param row
   * @param column
   * @return const reference to entry at (row,column)
   */
  inline const double &operator()(uint32_t row, uint32_t column) const {
    RangeCheck(row, column);

    return m_pData[row + column * m_Rows];
  }

private:
  /**
   * Allocate space for the matrix
   */
  void Allocate() {
    try {
      if (m_pData != NULL) {
        delete[] m_pData;
      }

      m_pData = new double[m_Rows * m_Columns];
    } catch (const std::bad_alloc &ex) {
      throw "Matrix allocation error";
    }

    if (m_pData == NULL) {
      throw "Matrix allocation error";
    }
  }

  /**
   * Checks if (row,column) is a valid entry into the matrix
   * @param row
   * @param column
   */
  inline void RangeCheck(uint32_t row, uint32_t column) const {
    if (amath::IsUpTo(row, m_Rows) == false) {
      throw "Matrix - RangeCheck ERROR!!!!";
    }

    if (amath::IsUpTo(column, m_Columns) == false) {
      throw "Matrix - RangeCheck ERROR!!!!";
    }
  }

private:
  uint32_t m_Rows;
  uint32_t m_Columns;

  double *m_pData;
}; // Matrix

#endif
