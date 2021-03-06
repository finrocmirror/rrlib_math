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
/*!\file    rrlib/math/vector/functions.hpp
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
namespace
{
template <typename TVector>
struct tTrait;

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
struct tTrait<tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>>
{
  enum { eARRAY_DIMENSION = Tdimension };
  enum { eEXPLICIT_LENGTH = false };
};
template <size_t Tdimension, typename TElement, typename ... TAdditionalDataParameters>
struct tTrait<tVector<Tdimension, TElement, vector::Polar, TAdditionalDataParameters...>>
{
  enum { eARRAY_DIMENSION = Tdimension - 1 };
  enum { eEXPLICIT_LENGTH = true };
};
}

//----------------------------------------------------------------------
// IsEqual
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
bool IsEqual(const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right, float max_error, tFloatComparisonMethod method)
{
  for (size_t i = 0; i < tTrait<tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>>::eARRAY_DIMENSION; ++i)
  {
    if (!IsEqual(left[i], right[i], max_error, method))
    {
      return false;
    }
  }
  if (tTrait<tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>>::eEXPLICIT_LENGTH)
  {
    if (!IsEqual(left.Length(), right.Length(), max_error, method))
    {
      return false;
    }
  }
  return true;
}

//----------------------------------------------------------------------
// IsLinearlyDependent
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
bool IsLinearlyDependent(const tVector<Tdimension, TElement, vector::Cartesian> &left, const tVector<Tdimension, TElement, vector::Cartesian> &right, float max_error, tFloatComparisonMethod method)
{
  float factor = 0;

  // using the first non zero element pair to define the factor
  for (size_t i = 0; i < Tdimension; ++i)
  {
    if (!IsEqual(left[i], 0, max_error, method) && !IsEqual(right[i], 0, max_error, method))
    {
      factor = (float)left[i] / right[i];
      break;
    }
  }

  // if right is zero, left and right are linearly dependent for every factor
  if (right.IsZero(max_error))
  {
    return true;
  }

  // otherwise that depends on left and scaled right
  for (size_t i = 0; i < Tdimension; ++i)
  {
    if (!IsEqual(left[i], factor * right[i], max_error, method))
    {
      return false;
    }
  }
  return true;
}

//----------------------------------------------------------------------
// operator ==
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
const bool operator == (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right)
{
  return IsEqual(left, right, 0.0);
}

//----------------------------------------------------------------------
// operator !=
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
const bool operator != (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right)
{
  return !(left == right);
}

//----------------------------------------------------------------------
// operator <
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
const bool operator < (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right)
{
  if (&left == &right)
  {
    return false;
  }
  for (size_t i = 0; i < tTrait<tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>>::eARRAY_DIMENSION; ++i)
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
  if (tTrait<tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>>::eEXPLICIT_LENGTH)
  {
    if (left.Length() < right.Length())
    {
      return true;
    }
  }
  return false;
}

//----------------------------------------------------------------------
// operator >
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
const bool operator > (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right)
{
  if (&left == &right)
  {
    return false;
  }
  for (size_t i = 0; i < tTrait<tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>>::eARRAY_DIMENSION; ++i)
  {
    if (left[i] > right[i])
    {
      return true;
    }
    if (left[i] < right[i])
    {
      break;
    }
  }
  if (tTrait<tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>>::eEXPLICIT_LENGTH)
  {
    if (left.Length() > right.Length())
    {
      return true;
    }
  }
  return false;
}

//----------------------------------------------------------------------
// operator <=
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
const bool operator <= (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right)
{
  return !(left > right);
}

//----------------------------------------------------------------------
// operator >=
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
const bool operator >= (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right)
{
  return !(left < right);
}

//----------------------------------------------------------------------
// SchurProduct
//----------------------------------------------------------------------
template <size_t Tdimension, typename TLeftElement, typename TRightElement>
const tVector <Tdimension, decltype(TLeftElement() * TRightElement()), vector::Cartesian> SchurProduct(const tVector<Tdimension, TLeftElement, vector::Cartesian> &left, const tVector<Tdimension, TRightElement, vector::Cartesian> &right)
{
  return left.SchurMultiplied(right);
}

//----------------------------------------------------------------------
// CrossProduct
//----------------------------------------------------------------------
template <typename TLeftElement, typename TRightElement>
const tVector <3, decltype(TLeftElement() * TRightElement()), vector::Cartesian> CrossProduct(const tVector<3, TLeftElement, vector::Cartesian> &left, const tVector<3, TRightElement, vector::Cartesian> &right)
{
  return left.CrossMultiplied(right);
}

//----------------------------------------------------------------------
// EnclosedAngle
//----------------------------------------------------------------------
template <size_t Tdimension, typename TLeftElement, typename TRightElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
const tAngleRad EnclosedAngle(const tVector<Tdimension, TLeftElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TRightElement, TData, TAdditionalDataParameters...> &right)
{
  if (left.IsZero() || right.IsZero())
  {
    return cPI_2;
  }
  double cos_angle = (left * right) / (left.Length() * right.Length());
  if (cos_angle >= 1.0)
  {
    return tAngleRad();
  }
  if (cos_angle <= -1)
  {
    return cPI;
  }
  return ACos(cos_angle);
}

//----------------------------------------------------------------------
// EnclosedAngle
//----------------------------------------------------------------------
template <typename TLeftElement, typename TRightElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
const tAngleRad EnclosedAngle(const tVector<2, TLeftElement, TData, TAdditionalDataParameters...> &left, const tVector<2, TRightElement, TData, TAdditionalDataParameters...> &right)
{
  if (CrossProduct(tVector<3, TLeftElement, TData, TAdditionalDataParameters...>(left), tVector<3, TRightElement, TData, TAdditionalDataParameters...>(right)).Z() < 0)
  {
    return -EnclosedAngle(tVector<3, TLeftElement, TData, TAdditionalDataParameters...>(left), tVector<3, TRightElement, TData, TAdditionalDataParameters...>(right));
  }
  return EnclosedAngle(tVector<3, TLeftElement, TData, TAdditionalDataParameters...>(left), tVector<3, TRightElement, TData, TAdditionalDataParameters...>(right));
}

//----------------------------------------------------------------------
// GetPolarVectorFromCartesian
//----------------------------------------------------------------------
template <size_t Tdimension, typename TPolarElement, typename TCartesianElement, typename TPolarUnitPolicy, typename TPolarAutoWrapPolicy>
void GetPolarVectorFromCartesian(tVector<Tdimension, TPolarElement, vector::Polar, TPolarUnitPolicy, TPolarAutoWrapPolicy> &polar, const tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian)
{
  polar = cartesian.GetPolarVector();
}

//----------------------------------------------------------------------
// GetCartesianVectorFromPolar
//----------------------------------------------------------------------
template <size_t Tdimension, typename TCartesianElement, typename TPolarElement, typename TPolarUnitPolicy, typename TPolarAutoWrapPolicy>
void GetCartesianVectorFromPolar(tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian, const tVector<Tdimension, TPolarElement, vector::Polar, TPolarUnitPolicy, TPolarAutoWrapPolicy> &polar)
{
  cartesian = polar.GetCartesianVector();
}

//----------------------------------------------------------------------
// GetPolarSignedVectorFromCartesian
//----------------------------------------------------------------------
template <typename TElement, typename TPolarUnitPolicy>
tVector<2, TElement, vector::Polar, TPolarUnitPolicy, angle::Signed> GetPolarSignedVectorFromCartesian(tVector<2, TElement, vector::Cartesian> cartesian, double radius)
{
  cartesian.Normalize();
  return tVector<2, TElement, vector::Polar, TPolarUnitPolicy, angle::Signed>(cartesian.Y() >= 0 ? std::acos(cartesian.X()) : -std::acos(cartesian.X()), radius);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
