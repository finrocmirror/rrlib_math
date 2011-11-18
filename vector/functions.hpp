//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    functions.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-08-25
 *
 * \brief
 *
 * \b
 *
 * A few words for functions.h
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__vector__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef __rrlib__math__vector__functions_hpp__
#define __rrlib__math__vector__functions_hpp__

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

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// IsEqual
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
bool IsEqual(const tVector<Tdimension, TElement, TData> &left, const tVector<Tdimension, TElement, TData> &right, float max_error = 1.0E-6, tFloatComparisonMethod method = eFCM_ABSOLUTE_ERROR)
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

//----------------------------------------------------------------------
// operator ==
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
const bool operator == (const tVector<Tdimension, TElement, TData> &left, const tVector<Tdimension, TElement, TData> &right)
{
  return IsEqual(left, right, 0.0);
}

//----------------------------------------------------------------------
// operator !=
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
const bool operator != (const tVector<Tdimension, TElement, TData> &left, const tVector<Tdimension, TElement, TData> &right)
{
  return !(left == right);
}

//----------------------------------------------------------------------
// operator <
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
const bool operator < (const tVector<Tdimension, TElement, TData> &left, const tVector<Tdimension, TElement, TData> &right)
{
  if (&left == &right)
  {
    return false;
  }
  for (size_t i = 0; i < Tdimension; ++i)
  {
    if (left[i] < right[i])
    {
      return true;
    }
    if (left[i] > right[i])
    {
      break;
    }
  }
  return false;
}

//----------------------------------------------------------------------
// SchurProduct
//----------------------------------------------------------------------
template <size_t Tdimension, typename TLeftElement, typename TRightElement>
const tVector<Tdimension, typename until_0x::Auto<TLeftElement, TRightElement>::type, vector::Cartesian> SchurProduct(const tVector<Tdimension, TLeftElement, vector::Cartesian> &left, const tVector<Tdimension, TRightElement, vector::Cartesian> &right)
{
  return left.SchurMultiplied(right);
}

//----------------------------------------------------------------------
// CrossProduct
//----------------------------------------------------------------------
template <typename TLeftElement, typename TRightElement>
const tVector<3, typename until_0x::Auto<TLeftElement, TRightElement>::type, vector::Cartesian> CrossProduct(const tVector<3, TLeftElement, vector::Cartesian> &left, const tVector<3, TRightElement, vector::Cartesian> &right)
{
  return left.CrossMultiplied(right);
}

//----------------------------------------------------------------------
// EnclosedAngle
//----------------------------------------------------------------------
template <size_t Tdimension, typename TLeftElement, typename TRightElement, template <size_t, typename> class TData>
const tAngleRad EnclosedAngle(const tVector<Tdimension, TLeftElement, TData> &left, const tVector<Tdimension, TRightElement, TData> &right)
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

//----------------------------------------------------------------------
// EnclosedAngle
//----------------------------------------------------------------------
template <typename TLeftElement, typename TRightElement, template <size_t, typename> class TData>
const tAngleRad EnclosedAngle(const tVector<2, TLeftElement, TData> &left, const tVector<2, TRightElement, TData> &right)
{
  if (CrossProduct(tVector<3, TLeftElement, TData>(left), tVector<3, TRightElement, TData>(right)).Z() < 0)
  {
    return 2 * M_PI - EnclosedAngle(tVector<3, TLeftElement, TData>(left), tVector<3, TRightElement, TData>(right));
  }
  return EnclosedAngle(tVector<3, TLeftElement, TData>(left), tVector<3, TRightElement, TData>(right));
}

//----------------------------------------------------------------------
// GetPolarVectorFromCartesian
//----------------------------------------------------------------------
template <size_t Tdimension, typename TPolarElement, typename TCartesianElement>
void GetPolarVectorFromCartesian(tVector<Tdimension, TPolarElement, vector::Polar> &polar, const tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian)
{
  polar = cartesian.GetPolarVector();
}

//----------------------------------------------------------------------
// GetCartesianVectorFromPolar
//----------------------------------------------------------------------
template <size_t Tdimension, typename TCartesianElement, typename TPolarElement>
void GetCartesianVectorFromPolar(tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian, const tVector<Tdimension, TPolarElement, vector::Polar> &polar)
{
  cartesian = polar.GetCartesianVector();
}

//----------------------------------------------------------------------
// GetPolarSignedVectorFromCartesian
//----------------------------------------------------------------------
template <typename TElement>
tVector<2, TElement, vector::Polar> GetPolarSignedVectorFromCartesian(tVector<2, TElement, vector::Cartesian> cartesian, double radius)
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