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
/*!\file    rrlib/math/matrix/functions.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-08-25
 *
 * \b
 *
 * A few words for functions.h
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__matrix__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef __rrlib__math__matrix__functions_hpp__
#define __rrlib__math__matrix__functions_hpp__

//----------------------------------------------------------------------
// External includes with <>
//----------------------------------------------------------------------
#include <cmath>
#include <iostream>
#include <type_traits>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/utilities.h"

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

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

template <size_t Trows, size_t Tcolumns, typename TElement>
bool IsEqual(const tMatrix<Trows, Tcolumns, TElement> &left, const tMatrix<Trows, Tcolumns, TElement> &right, float max_error, tFloatComparisonMethod method)
{
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      if (!IsEqual(left[row][column], right[row][column], max_error, method))
      {
        return false;
      }
    }
  }
  return true;
}

template <size_t Trows, size_t Tcolumns, typename TElement>
const bool operator == (const tMatrix<Trows, Tcolumns, TElement> &left, const tMatrix<Trows, Tcolumns, TElement> &right)
{
  return IsEqual(left, right, 0.0);
}

template <size_t Trows, size_t Tcolumns, typename TElement>
const bool operator != (const tMatrix<Trows, Tcolumns, TElement> &left, const tMatrix<Trows, Tcolumns, TElement> &right)
{
  return !(left == right);
}

template <typename TElement>
const tMatrix<2, 2, TElement> Get2DRotationMatrix(tAngleRad angle)
{
  static_assert(std::is_floating_point<TElement>::value, "Instantiation of this method only valid for float or double!");
  TElement sin_angle, cos_angle;
  angle.SinCos(sin_angle, cos_angle);
  return tMatrix<2, 2, TElement>(cos_angle, -sin_angle, sin_angle, cos_angle);
}

template <typename TElement>
const tMatrix<3, 3, TElement> Get3DRotationMatrixFromRollPitchYaw(tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
{
  double sin_roll, cos_roll;
  roll.SinCos(sin_roll, cos_roll);
  double sin_pitch, cos_pitch;
  pitch.SinCos(sin_pitch, cos_pitch);
  double sin_yaw, cos_yaw;
  yaw.SinCos(sin_yaw, cos_yaw);
  return tMatrix<3, 3, TElement>(cos_pitch * cos_yaw, -cos_roll * sin_yaw + sin_roll * sin_pitch * cos_yaw, sin_roll * sin_yaw + cos_roll * sin_pitch * cos_yaw,
                                 cos_pitch * sin_yaw , cos_roll * cos_yaw + sin_roll * sin_pitch * sin_yaw, -sin_roll * cos_yaw + cos_roll * sin_pitch * sin_yaw,
                                 -sin_pitch, sin_roll * cos_pitch, cos_roll * cos_pitch);
}

template <typename TElement>
const tMatrix<4, 4, TElement> Get4DTransformationMatrixZYXT(TElement x, TElement y, TElement z, tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
{
  double sin_roll, cos_roll;
  roll.SinCos(sin_roll, cos_roll);
  double sin_pitch, cos_pitch;
  pitch.SinCos(sin_pitch, cos_pitch);
  double sin_yaw, cos_yaw;
  yaw.SinCos(sin_yaw, cos_yaw);
  return tMatrix<4, 4, TElement>(
           cos_pitch * cos_yaw,
           cos_pitch * sin_yaw,
           -sin_pitch,
           0,
           sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw,
           sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw,
           sin_roll * cos_pitch,
           0,
           cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw,
           cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw,
           cos_roll * cos_pitch,
           0,
           cos_pitch * cos_yaw * x + (sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw) * y + (cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw) * z,
           cos_pitch * sin_yaw * x + (sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw) * y + (cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw) * z,
           -sin_pitch * x + sin_roll * cos_pitch * y + cos_roll * cos_pitch * z,
           1
         );
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
