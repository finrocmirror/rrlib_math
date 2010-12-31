//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    functions.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-09-26
 *
 * \brief
 *
 * \b
 *
 * A few words for functions.h
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_vector_functions_h_
#define _rrlib_math_vector_functions_h_

//----------------------------------------------------------------------
// External includes with <>
//----------------------------------------------------------------------
#include <cmath>
#include <iostream>

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



template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
inline bool IsEqual(const tVector<Tdimension, TElement, TData> &left, const tVector<Tdimension, TElement, TData> &right, float max_error = 1.0E-6, tFloatComparisonMethod method = eFCM_ABSOLUTE_ERROR)
{
  for (size_t i = 0; i < Tdimension; ++i)
  {
    if (!IsEqual(left[i], right[i], max_error, method))
    {
      return false;
    }
  }
  return true;
}

template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
inline const bool operator == (const tVector<Tdimension, TElement, TData> &left, const tVector<Tdimension, TElement, TData> &right)
{
  return IsEqual(left, right, 0.0);
}

template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
inline const bool operator != (const tVector<Tdimension, TElement, TData> &left, const tVector<Tdimension, TElement, TData> &right)
{
  return !(left == right);
}

template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
inline const bool operator < (const tVector<Tdimension, TElement, TData> &left, const tVector<Tdimension, TElement, TData> &right)
{
  if (&left == &right)
  {
    return false;
  }
  return std::memcmp(&left, &right, sizeof(tVector<Tdimension, TElement, TData>)) < 0;
}

template <size_t Tdimension, typename TLeftElement, typename TRightElement>
inline const tVector<Tdimension, typename until_0x::Auto<TLeftElement, TRightElement>::type, vector::Cartesian> SchurProduct(const tVector<Tdimension, TLeftElement, vector::Cartesian> &left, const tVector<Tdimension, TRightElement, vector::Cartesian> &right)
{
  return left.SchurMultiplied(right);
}

template <typename TLeftElement, typename TRightElement>
inline const tVector<3, typename until_0x::Auto<TLeftElement, TRightElement>::type, vector::Cartesian> CrossProduct(const tVector<3, TLeftElement, vector::Cartesian> &left, const tVector<3, TRightElement, vector::Cartesian> &right)
{
  return left.CrossMultiplied(right);
}

template <size_t Tdimension, typename TLeftElement, typename TRightElement, template <size_t, typename> class TData>
inline const tAngleRad EnclosedAngle(const tVector<Tdimension, TLeftElement, TData> &left, const tVector<Tdimension, TRightElement, TData> &right)
{
  if (left.IsZero() || right.IsZero())
  {
    return M_PI_2;
  }
  double cos_angle = (left * right) / (left.Length() * right.Length());
  if (cos_angle >= 1.0)
  {
    return 0;
  }
  if (cos_angle <= -1)
  {
    return M_PI;
  }
  return std::acos(cos_angle);
}
template <typename TLeftElement, typename TRightElement, template <size_t, typename> class TData>
const tAngleRad EnclosedAngle(const tVector<2, TLeftElement, TData> &left, const tVector<2, TRightElement, TData> &right)
{
  if (CrossProduct(tVector<3, TLeftElement, TData>(left), tVector<3, TRightElement, TData>(right)).Z() < 0)
  {
    return 2 * M_PI - EnclosedAngle(tVector<3, TLeftElement, TData>(left), tVector<3, TRightElement, TData>(right));
  }
  return EnclosedAngle(tVector<3, TLeftElement, TData>(left), tVector<3, TRightElement, TData>(right));
}

/*!
 * \brief Convert Cartesian coordinates into polar coordinates
 *
 * \param polar       A reference to the vector that is supposed to hold the polar coordinates (angle, radius)
 * \param cartesian   The vector that holds the Cartesian coordinates
 */
template <size_t Tdimension, typename TPolarElement, typename TCartesianElement>
inline void GetPolarVectorFromCartesian(tVector<Tdimension, TPolarElement, vector::Polar> &polar, const tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian)
{
  polar = cartesian.GetPolarVector();
}

/*!
 * \brief Convert polar coordinates into Cartesian coordinates
 *
 * \param cartesian   A reference to the vector that is supposed to hold the Cartesian coordinates
 * \param polar       The vector that holds the polar coordinates (angle, radius)
 */
template <size_t Tdimension, typename TCartesianElement, typename TPolarElement>
inline void GetCartesianVectorFromPolar(tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian, const tVector<Tdimension, TPolarElement, vector::Polar> &polar)
{
  cartesian = polar.GetCartesianVector();
}

template <typename TElement>
inline tVector<2, TElement, vector::Polar> GetPolarSignedFromCartesian(tVector<2, TElement, vector::Cartesian> cartesian, double radius = 1)
{
  cartesian.Normalize();
  return tVector<2, TElement, vector::Polar>(cartesian.Y() >= 0 ? std::acos(cartesian.X()) : -std::acos(cartesian.X()), radius);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
