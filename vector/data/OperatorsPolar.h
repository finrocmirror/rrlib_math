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
/*!\file    rrlib/math/vector/data/OperatorsPolar.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-09-26
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__vector__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef __rrlib__math__vector__data__OperatorsPolar_h__
#define __rrlib__math__vector__data__OperatorsPolar_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <ostream>
#include <istream>
#include <type_traits>

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_
#include "rrlib/serialization/serialization.h"
#include <sstream>
#endif

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

#include "rrlib/math/tAngle.h"

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
namespace vector
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------

template <size_t Tdimension, typename TElement, typename ... TAdditionalDataParameters>
std::ostream &operator << (std::ostream &stream, const tVector<Tdimension, TElement, Polar, TAdditionalDataParameters...> &vector)
{
  stream << "(";
  for (size_t i = 0; i < Tdimension - 1; ++i)
  {
    stream << tAngleDeg(vector[i]) << ", ";
  }
  stream << vector.Length() << ")";
  return stream;
}

template <size_t Tdimension, typename TElement, typename ... TAdditionalDataParameters>
std::istream &operator >> (std::istream &stream, tVector<Tdimension, TElement, Polar, TAdditionalDataParameters...> &vector)
{
  char temp;
  stream >> temp;

  if (temp == '(')
  {
    for (size_t i = 0; i < Tdimension - 1; ++i)
    {
      tAngleDeg signed_degree_angle;
      stream >> signed_degree_angle >> temp;
      vector[i] = signed_degree_angle;
    }
    stream >> vector.Length() >> temp;
    return stream;
  }
  stream.putback(temp);
  for (size_t i = 0; i < Tdimension - 1; ++i)
  {
    double signed_degree_value;
    stream >> signed_degree_value;
    vector[i] = tAngleDeg(signed_degree_value);
  }
  stream >> vector.Length() >> temp;
  return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <size_t Tdimension, typename TElement, typename ... TAdditionalDataParameters>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tVector<Tdimension, TElement, Polar, TAdditionalDataParameters...> &vector)
{
  for (size_t i = 0; i < Tdimension - 1; ++i)
  {
    stream << vector[i];
  }
  stream << vector.Length();
  return stream;
}

template <size_t Tdimension, typename TElement, typename ... TAdditionalDataParameters>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tVector<Tdimension, TElement, Polar, TAdditionalDataParameters...> &vector)
{
  for (size_t i = 0; i < Tdimension - 1; ++i)
  {
    stream >> vector[i];
  }
  stream >> vector.Length();
  return stream;
}

#endif

template <size_t Tdimension, typename TElement, typename ... TAdditionalDataParameters>
const tVector<Tdimension, TElement, Polar, TAdditionalDataParameters...> operator - (const tVector<Tdimension, TElement, Polar, TAdditionalDataParameters...> &vector)
{
  return (-vector.GetCartesianVector()).GetPolarVector();
}

template <size_t Tdimension, typename TLeftElement, typename TRightElement, template <size_t, typename TElement, typename ... TAdditionalDataParameters> class TData, typename ... TAdditionalDataParameters>
tVector < Tdimension, decltype(TLeftElement() + TRightElement()), TData, TAdditionalDataParameters... > operator + (const tVector<Tdimension, TLeftElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TRightElement, TData, TAdditionalDataParameters...> &right)
{
  return (left.GetCartesianVector() + right.GetCartesianVector()).GetPolarVector();
}

template <size_t Tdimension, typename TLeftElement, typename TRightElement, typename ... TAdditionalDataParameters>
tVector < Tdimension, decltype(TLeftElement() - TRightElement()), Polar, TAdditionalDataParameters... > operator - (const tVector<Tdimension, TLeftElement, Polar, TAdditionalDataParameters...> &left, const tVector<Tdimension, TRightElement, Polar, TAdditionalDataParameters...> &right)
{
  return (left.GetCartesianVector() - right.GetCartesianVector()).GetPolarVector();
}

template <size_t Tdimension, typename TElement, typename TScalar, typename ... TAdditionalDataParameters>
typename std::enable_if<std::is_scalar<TScalar>::value, tVector <Tdimension, decltype(TElement() * TScalar()), Polar, TAdditionalDataParameters...>>::type operator * (const tVector<Tdimension, TElement, Polar, TAdditionalDataParameters...> &vector, const TScalar scalar)
{
  typedef math::tVector <Tdimension, decltype(TElement() * TScalar()), Polar, TAdditionalDataParameters...> tResult;
  tAngle<decltype(TElement() * TScalar()), TAdditionalDataParameters...> angles[Tdimension];
  for (size_t i = 0; i < Tdimension - 1; ++i)
  {
    angles[i] = vector[i];
  }
  return tResult(angles, vector.Length() * scalar);
}
template <size_t Tdimension, typename TElement, typename TScalar, typename ... TAdditionalDataParameters>
typename std::enable_if<std::is_scalar<TScalar>::value, tVector <Tdimension, decltype(TElement() * TScalar()), Polar, TAdditionalDataParameters...>>::type operator * (const TScalar scalar, const tVector<Tdimension, TElement, Polar, TAdditionalDataParameters...> &vector)
{
  return vector * scalar;
}

template <size_t Tdimension, typename TLeftElement, typename TRightElement, typename ... TAdditionalDataParameters>
decltype(TLeftElement() * TRightElement()) operator *(const tVector<Tdimension, TLeftElement, Polar, TAdditionalDataParameters...> &left, const tVector<Tdimension, TRightElement, Polar, TAdditionalDataParameters...> &right)
{
  return left.GetCartesianVector() * right.GetCartesianVector();
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
