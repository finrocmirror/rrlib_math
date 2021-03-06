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
/*!\file    rrlib/math/vector/data/OperatorsCartesian.h
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

#ifndef __rrlib__math__vector__data__OperatorsCartesian_h__
#define __rrlib__math__vector__data__OperatorsCartesian_h__

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

template <size_t Tdimension, typename TElement>
std::ostream &operator << (std::ostream &stream, const tVector<Tdimension, TElement, Cartesian> &vector)
{
  stream << "(" << vector[0];
  for (size_t i = 1; i < Tdimension; ++i)
  {
    stream << ", " << vector[i];
  }
  stream << ")";
  return stream;
}

template <size_t Tdimension>
std::ostream &operator << (std::ostream &stream, const tVector<Tdimension, char, Cartesian> &vector)
{
  stream << "(" << int(vector[0]);
  for (size_t i = 1; i < Tdimension; ++i)
  {
    stream << ", " << int(vector[i]);
  }
  stream << ")";
  return stream;
}

template <size_t Tdimension>
std::ostream &operator << (std::ostream &stream, const tVector<Tdimension, unsigned char, Cartesian> &vector)
{
  stream << "(" << int(vector[0]);
  for (size_t i = 1; i < Tdimension; ++i)
  {
    stream << ", " << int(vector[i]);
  }
  stream << ")";
  return stream;
}

template <size_t Tdimension, typename TElement>
std::istream &operator >> (std::istream &stream, tVector<Tdimension, TElement, Cartesian> &vector)
{
  char temp;
  stream >> temp;

  if (temp == '(')
  {
    for (size_t i = 0; i < Tdimension; ++i)
    {
      stream >> vector[i] >> temp;
    }
    return stream;
  }
  stream.putback(temp);
  for (size_t i = 0; i < Tdimension; ++i)
  {
    stream >> vector[i];
  }
  return stream;
}

template <size_t Tdimension>
std::istream &operator >> (std::istream &stream, tVector<Tdimension, char, Cartesian> &vector)
{
  char temp;
  stream >> temp;

  if (temp == '(')
  {
    for (size_t i = 0; i < Tdimension; ++i)
    {
      int value;
      stream >> value >> temp;
      vector[i] = value;
    }
    return stream;
  }
  stream.putback(temp);
  for (size_t i = 0; i < Tdimension; ++i)
  {
    int value;
    stream >> value >> temp;
    vector[i] = value;
  }
  return stream;
}

template <size_t Tdimension>
std::istream &operator >> (std::istream &stream, tVector<Tdimension, unsigned char, Cartesian> &vector)
{
  char temp;
  stream >> temp;

  if (temp == '(')
  {
    for (size_t i = 0; i < Tdimension; ++i)
    {
      int value;
      stream >> value >> temp;
      vector[i] = value;
    }
    return stream;
  }
  stream.putback(temp);
  for (size_t i = 0; i < Tdimension; ++i)
  {
    int value;
    stream >> value >> temp;
    vector[i] = value;
  }
  return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <size_t Tdimension, typename TElement>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tVector<Tdimension, TElement, Cartesian> &vector)
{
  for (size_t i = 0; i < Tdimension; ++i)
  {
    stream << vector[i];
  }
  return stream;
}

template <size_t Tdimension, typename TElement>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tVector<Tdimension, TElement, Cartesian> &vector)
{
  for (size_t i = 0; i < Tdimension; ++i)
  {
    stream >> vector[i];
  }
  return stream;
}

#endif


template <size_t Tdimension, typename TElement>
const tVector<Tdimension, TElement, Cartesian> operator - (const tVector<Tdimension, TElement, Cartesian> &vector)
{
  typedef math::tVector<Tdimension, TElement, Cartesian> tResult;
  typename tResult::tElement data[Tdimension];
  for (size_t i = 0; i < Tdimension; ++i)
  {
    data[i] = -reinterpret_cast<const TElement *>(&vector)[i];
  }
  return tResult(data);
}

template <size_t Tdimension, typename TLeftElement, typename TRightElement>
const tVector < Tdimension, decltype(TLeftElement() + TRightElement()), Cartesian > operator + (const tVector<Tdimension, TLeftElement, Cartesian> &left, const tVector<Tdimension, TRightElement, Cartesian> &right)
{
  typedef math::tVector < Tdimension, decltype(TLeftElement() + TRightElement()), Cartesian > tResult;
  typename tResult::tElement data[Tdimension];
  for (size_t i = 0; i < Tdimension; ++i)
  {
    data[i] = reinterpret_cast<const TLeftElement *>(&left)[i] + reinterpret_cast<const TRightElement *>(&right)[i];
  }
  return tResult(data);
}

template <size_t Tdimension, typename TLeftElement, typename TRightElement>
const tVector < Tdimension, decltype(TLeftElement() - TRightElement()), Cartesian > operator - (const tVector<Tdimension, TLeftElement, Cartesian> &left, const tVector<Tdimension, TRightElement, Cartesian> &right)
{
  typedef math::tVector < Tdimension, decltype(TLeftElement() - TRightElement()), Cartesian > tResult;
  typename tResult::tElement data[Tdimension];
  for (size_t i = 0; i < Tdimension; ++i)
  {
    data[i] = reinterpret_cast<const TLeftElement *>(&left)[i] - reinterpret_cast<const TRightElement *>(&right)[i];
  }
  return tResult(data);
}

template <size_t Tdimension, typename TElement, typename TScalar>
const typename std::enable_if<std::is_scalar<TScalar>::value, tVector <Tdimension, decltype(TElement() * TScalar()), Cartesian>>::type operator *(const tVector<Tdimension, TElement, Cartesian> &vector, const TScalar scalar)
{
  typedef math::tVector <Tdimension, decltype(TElement() * TScalar()), Cartesian> tResult;
  typename tResult::tElement data[Tdimension];
  for (size_t i = 0; i < Tdimension; ++i)
  {
    data[i] = reinterpret_cast<const TElement *>(&vector)[i] * scalar;
  }
  return tResult(data);
}
template <size_t Tdimension, typename TElement, typename TScalar>
const typename std::enable_if<std::is_scalar<TScalar>::value, tVector <Tdimension, decltype(TElement() * TScalar()), Cartesian>>::type operator *(const TScalar scalar, const tVector<Tdimension, TElement, Cartesian> &vector)
{
  return vector * scalar;
}

template <size_t Tdimension, typename TLeftElement, typename TRightElement>
const decltype(TLeftElement() * TRightElement()) operator *(const tVector<Tdimension, TLeftElement, Cartesian> &left, const tVector<Tdimension, TRightElement, Cartesian> &right)
{
  decltype(TLeftElement() * TRightElement()) result = 0;
  for (size_t i = 0; i < Tdimension; ++i)
  {
    result += reinterpret_cast<const TLeftElement *>(&left)[i] * reinterpret_cast<const TRightElement *>(&right)[i];
  }
  return result;
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
