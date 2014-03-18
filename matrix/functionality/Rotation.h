//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    rrlib/math/matrix/functionality/Rotation.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-21
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__matrix__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef __rrlib__math__matrix__functionality__Rotation_h__
#define __rrlib__math__matrix__functionality__Rotation_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>
#include <stdexcept>
#include <type_traits>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace math
{
namespace matrix
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
template <size_t Trows, size_t Tcolumns, typename TElement>
class Rotation
{
  typedef math::tMatrix<Trows, Tcolumns, TElement> tMatrix;

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Rotation() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

};

/*!
 *
 */
template <typename TElement>
class Rotation<3, 3, TElement>
{
  typedef math::tMatrix<3, 3, TElement> tMatrix;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*! Assumes that the matrix is an orthogonal rotation matrix with det = 1 and calculates the corresponding
   *  rotation axis and angle that the matrix represents.
   * code taken from Coin 2.4.5 SbRotation.cpp
   */
  template <typename TVectorElement, typename TAngle>
  typename std::enable_if<std::is_floating_point<TAngle>::value, void>::type GetRotation(tVector<3, TVectorElement, vector::Cartesian> &axis, TAngle &angle) const
  {
    const tMatrix *that = reinterpret_cast<const tMatrix *>(this);

    // first, convert matrix to quaternion
    double s;
    double quaternion[4];

    float scale = (*that)[0][0] + (*that)[1][1] + (*that)[2][2];

    if (scale > 0)
    {
      s = std::sqrt(scale + 1.0);

      quaternion[3] = s / 2;
      s = 0.5 / s;

      quaternion[0] = ((*that)[2][1] - (*that)[1][2]) * s;
      quaternion[1] = ((*that)[0][2] - (*that)[2][0]) * s;
      quaternion[2] = ((*that)[1][0] - (*that)[0][1]) * s;
    }
    else
    {
      int i = 0;
      if ((*that)[1][1] > (*that)[0][0])
      {
        i = 1;
      }
      if ((*that)[2][2] > (*that)[i][i])
      {
        i = 2;
      }

      int j = (i + 1) % 3;
      int k = (j + 1) % 3;

      s = std::sqrt(((*that)[i][i] - ((*that)[j][j] + (*that)[k][k])) + 1.0);

      quaternion[i] = s / 2;
      s = 0.5 / s;

      quaternion[3] = ((*that)[k][j] - (*that)[j][k]) * s;
      quaternion[j] = ((*that)[j][i] + (*that)[i][j]) * s;
      quaternion[k] = ((*that)[k][i] + (*that)[i][k]) * s;
    }

    // convert quaternion to angle
    axis.Set(0, 0, 1);
    angle = 0.0;
    if ((quaternion[3] >= -1.0) && (quaternion[3] <= 1.0))
    {
      angle = std::acos(quaternion[3]) * 2.0;
      float scale = static_cast<float>(std::sin(angle / 2));

      if (scale != 0)
      {
        axis.Set(quaternion[0] / scale, quaternion[1] / scale, quaternion[2] / scale);
      }
    }
  }

  template <typename TAngleElement, typename TUnitPolicy, typename TSignPolicy>
  void ExtractRollPitchYaw(tAngle<TAngleElement, TUnitPolicy, TSignPolicy> &roll, tAngle<TAngleElement, TUnitPolicy, TSignPolicy> &pitch, tAngle<TAngleElement, TUnitPolicy, TSignPolicy> &yaw, bool use_second_solution = false, double max_error = 1E-6) const
  {
    const tMatrix *that = reinterpret_cast<const tMatrix *>(this);

    assert(IsEqual(that->Determinant(), 1.0, max_error));
    if (!use_second_solution)
    {
      roll = std::atan2((*that)[2][1], (*that)[2][2]);
    }
    else
    {
      roll = std::atan2(-(*that)[2][1], -(*that)[2][2]);
    }
    double sin_roll, cos_roll;
    roll.SinCos(sin_roll, cos_roll);
    pitch = std::atan2(-(*that)[2][0], sin_roll * (*that)[2][1] + cos_roll * (*that)[2][2]);
    yaw = std::atan2(sin_roll * (*that)[0][2] - cos_roll * (*that)[0][1], cos_roll * (*that)[1][1] - sin_roll * (*that)[1][2]);
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Rotation() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

};

/*!
 *
 */
template <typename TElement>
class Rotation<4, 4, TElement>
{
  typedef math::tMatrix<4, 4, TElement> tMatrix;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TVectorElement, typename TAngle>
  typename std::enable_if<std::is_floating_point<TAngle>::value, void>::type GetRotation(tVector<3, TVectorElement, vector::Cartesian> &axis, TAngle &angle) const
  {
    const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
    TElement data[3 * 3];
    for (size_t row = 0; row < 3; ++row)
    {
      for (size_t column = 0; column < 3; ++column)
      {
        data[row * 3 + column] = (*that)[row][column];
      }
    }
    math::tMatrix<3, 3, TElement>(data).GetRotation(axis, angle);
  }

  template <typename TAngleElement, typename TUnitPolicy, typename TSignPolicy>
  void ExtractRollPitchYaw(tAngle<TAngleElement, TUnitPolicy, TSignPolicy> &roll, tAngle<TAngleElement, TUnitPolicy, TSignPolicy> &pitch, tAngle<TAngleElement, TUnitPolicy, TSignPolicy> &yaw, bool use_second_solution = false, double max_error = 1E-6) const
  {
    const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
    TElement data[3 * 3];
    for (size_t row = 0; row < 3; ++row)
    {
      for (size_t column = 0; column < 3; ++column)
      {
        data[row * 3 + column] = (*that)[row][column];
      }
    }
    math::tMatrix<3, 3, TElement>(data).ExtractRollPitchYaw(roll, pitch, yaw, use_second_solution, max_error);
  }

  /*! Sets this matrix to represent a homogeneous rotation matrix
   *  with given angle around given axis.
   *  If axis is invalid, this is a no-op.
   * \returns reference to this matrix
   *
   *  \note: code taken from Coin 2.5.0 SbRotation.cpp
   */
  template <typename TVectorElement>
  tMatrix &SetRotation(const tVector<3, TVectorElement, vector::Cartesian> &axis, double angle)
  {
    tMatrix *that = reinterpret_cast<tMatrix *>(this);
    if (axis.Length() == 0)
    {
      std::runtime_error("Invalid rotation axis (with zero length)");
    }

    tVector<3, TVectorElement, vector::Cartesian> normalized_axis(axis.Normalized());

    double half_angle = angle / 2;
    double sin_angle = std::sin(half_angle);
    double cos_angle = std::cos(half_angle);

    double x = sin_angle * normalized_axis.X();
    double y = sin_angle * normalized_axis.Y();
    double z = sin_angle * normalized_axis.Z();
    double w = cos_angle;

    that->Set(w * w + x * x - y * y - z * z,
              2.0 * x * y - 2.0 * w * z,
              2.0 * x * z + 2.0 * w * y,
              0.0,

              2.0 * x * y + 2.0 * w * z,
              w * w - x * x + y * y - z * z,
              2.0 * y * z - 2.0 * w * x,
              0.0,

              2.0 * x * z - 2.0 * w * y,
              2.0 * y * z + 2.0 * w * x,
              w * w - x * x - y * y + z * z,
              0.0,

              0.0,
              0.0,
              0.0,
              w * w + x * x + y * y + z * z
             );
    return *that;
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Rotation() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
