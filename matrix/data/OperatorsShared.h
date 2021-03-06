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
/*!\file    rrlib/math/matrix/data/OperatorsShared.h
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

#ifndef __rrlib__math__matrix__data__OperatorsShared_h__
#define __rrlib__math__matrix__data__OperatorsShared_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <ostream>
#include <type_traits>

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_
#include "rrlib/serialization/serialization.h"
#include <sstream>
#endif

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"

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
// Function declarations
//----------------------------------------------------------------------

template <size_t Trows, size_t Tcolumns, typename TElement>
std::ostream &operator << (std::ostream &stream, const math::tMatrix<Trows, Tcolumns, TElement> &matrix)
{
  stream << "[";
  for (size_t k = 0; k < Tcolumns; ++k)
  {
    stream << " " << matrix[0][k];
  }
  for (size_t i = 1; i < Trows; ++i)
  {
    stream << " ;";
    for (size_t k = 0; k < Tcolumns; ++k)
    {
      stream << " " << matrix[i][k];
    }
  }
  stream << " ]";
  return stream;
}

template <size_t Trows, size_t Tcolumns>
std::ostream &operator << (std::ostream &stream, const math::tMatrix<Trows, Tcolumns, char> &matrix)
{
  stream << "[";
  for (size_t k = 0; k < Tcolumns; ++k)
  {
    stream << " " << int(matrix[0][k]);
  }
  for (size_t i = 1; i < Trows; ++i)
  {
    stream << " ;";
    for (size_t k = 0; k < Tcolumns; ++k)
    {
      stream << " " << int(matrix[i][k]);
    }
  }
  stream << " ]";
  return stream;
}

template <size_t Trows, size_t Tcolumns>
std::ostream &operator << (std::ostream &stream, const math::tMatrix<Trows, Tcolumns, unsigned char> &matrix)
{
  stream << "[";
  for (size_t k = 0; k < Tcolumns; ++k)
  {
    stream << " " << int(matrix[0][k]);
  }
  for (size_t i = 1; i < Trows; ++i)
  {
    stream << " ;";
    for (size_t k = 0; k < Tcolumns; ++k)
    {
      stream << " " << int(matrix[i][k]);
    }
  }
  stream << " ]";
  return stream;
}

template <size_t Trows, size_t Tcolumns, typename TElement>
std::istream &operator >> (std::istream &stream, math::tMatrix<Trows, Tcolumns, TElement> &matrix)
{
  char temp;
  stream >> temp;

  TElement data[Trows * Tcolumns];
  std::memset(data, 0, sizeof(data));

  if (temp == '[')
  {
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = 0; column < Tcolumns; ++column)
      {
        stream >> data[row * Tcolumns + column];
      }
      stream >> temp;
    }
  }
  else if (temp == '(')
  {
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = 0; column < Tcolumns; ++column)
      {
        stream >> data[row * Tcolumns + column] >> temp;
      }
    }
  }
  else
  {
    stream.putback(temp);
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = 0; column < Tcolumns; ++column)
      {
        stream >> data[row * Tcolumns + column];
      }
    }
  }
  matrix.SetFromArray(data);
  return stream;
}

template <size_t Trows, size_t Tcolumns>
std::istream &operator >> (std::istream &stream, math::tMatrix<Trows, Tcolumns, char> &matrix)
{
  char temp;
  stream >> temp;

  char data[Trows * Tcolumns];
  std::memset(data, 0, sizeof(data));

  if (temp == '[')
  {
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = 0; column < Tcolumns; ++column)
      {
        int value;
        stream >> value;
        data[row * Tcolumns + column] = value;
      }
      stream >> temp;
    }
  }
  else if (temp == '(')
  {
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = 0; column < Tcolumns; ++column)
      {
        int value;
        stream >> value >> temp;
        data[row * Tcolumns + column] = value;
      }
    }
  }
  else
  {
    stream.putback(temp);
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = 0; column < Tcolumns; ++column)
      {
        int value;
        stream >> value;
        data[row * Tcolumns + column] = value;
      }
    }
  }
  matrix.SetFromArray(data);
  return stream;
}

template <size_t Trows, size_t Tcolumns>
std::istream &operator >> (std::istream &stream, math::tMatrix<Trows, Tcolumns, unsigned char> &matrix)
{
  char temp;
  stream >> temp;

  unsigned char data[Trows * Tcolumns];
  std::memset(data, 0, sizeof(data));

  if (temp == '[')
  {
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = 0; column < Tcolumns; ++column)
      {
        int value;
        stream >> value;
        data[row * Tcolumns + column] = value;
      }
      stream >> temp;
    }
  }
  else if (temp == '(')
  {
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = 0; column < Tcolumns; ++column)
      {
        int value;
        stream >> value >> temp;
        data[row * Tcolumns + column] = value;
      }
    }
  }
  else
  {
    stream.putback(temp);
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = 0; column < Tcolumns; ++column)
      {
        int value;
        stream >> value;
        data[row * Tcolumns + column] = value;
      }
    }
  }
  matrix.SetFromArray(data);
  return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <size_t Trows, size_t Tcolumns, typename TElement>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const math::tMatrix<Trows, Tcolumns, TElement> &matrix)
{
  for (size_t i = 0; i < Trows * Tcolumns; ++i)
  {
    stream << reinterpret_cast<const TElement *>(&matrix)[i];
  }
  return stream;
}

template <size_t Trows, size_t Tcolumns, typename TElement>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, math::tMatrix<Trows, Tcolumns, TElement> &matrix)
{
  for (size_t i = 0; i < Trows * Tcolumns; ++i)
  {
    stream >> reinterpret_cast<TElement *>(&matrix)[i];
  }
  return stream;
}

template <size_t Trows, size_t Tcolumns, typename TElement>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const math::tMatrix<Trows, Tcolumns, TElement> &matrix)
{
  std::stringstream string_stream;
  string_stream << matrix;
  stream << string_stream.str();
  return stream;
}

template <size_t Trows, size_t Tcolumns, typename TElement>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, math::tMatrix<Trows, Tcolumns, TElement> &matrix)
{
  stream.GetWrappedStringStream() >> matrix;
  return stream;
}

#endif

template <size_t Trows, size_t Tcolumns, typename TElement>
const math::tMatrix<Trows, Tcolumns, TElement> operator - (const math::tMatrix<Trows, Tcolumns, TElement> &matrix)
{
  typedef math::tMatrix<Trows, Tcolumns, TElement> tResult;
  typename tResult::tElement data[sizeof(tResult) / sizeof(typename tResult::tElement)];
  for (size_t i = 0; i < sizeof(tResult) / sizeof(typename tResult::tElement); ++i)
  {
    data[i] = -reinterpret_cast<const TElement *>(&matrix)[i];
  }
  return *reinterpret_cast<tResult *>(&data);
}

template <size_t Trows, size_t Tcolumns, typename TLeftElement, typename TRightElement>
const math::tMatrix < Trows, Tcolumns, decltype(TLeftElement() + TRightElement()) > operator + (const math::tMatrix<Trows, Tcolumns, TLeftElement> &left, const math::tMatrix<Trows, Tcolumns, TRightElement> &right)
{
  typedef math::tMatrix < Trows, Tcolumns, decltype(TLeftElement() + TRightElement()) > tResult;
  typename tResult::tElement data[sizeof(tResult) / sizeof(typename tResult::tElement)];
  for (size_t i = 0; i < sizeof(tResult) / sizeof(typename tResult::tElement); ++i)
  {
    data[i] = reinterpret_cast<const TLeftElement *>(&left)[i] + reinterpret_cast<const TRightElement *>(&right)[i];
  }
  return *reinterpret_cast<tResult *>(&data);
}

template <size_t Trows, size_t Tcolumns, typename TLeftElement, typename TRightElement>
inline const math::tMatrix < Trows, Tcolumns, decltype(TLeftElement() - TRightElement()) > operator - (const math::tMatrix<Trows, Tcolumns, TLeftElement> &left, const math::tMatrix<Trows, Tcolumns, TRightElement> &right)
{
  return left + -right;
}

template <size_t Trows, size_t Tconnection, size_t Tcolumns, typename TLeftElement, typename TRightElement>
const math::tMatrix < Trows, Tcolumns, decltype((TLeftElement() * TRightElement()) + (TLeftElement() * TRightElement())) > operator *(const math::tMatrix<Trows, Tconnection, TLeftElement> &left, const math::tMatrix<Tconnection, Tcolumns, TRightElement> &right)
{
  typedef math::tMatrix < Trows, Tcolumns, decltype((TLeftElement() * TRightElement()) + (TLeftElement() * TRightElement())) > tResult;
  typename tResult::tElement data[Trows * Tcolumns];
  std::memset(data, 0, sizeof(data));
  size_t index = 0;
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      const size_t left_offset = row * Tconnection;
      for (size_t i = 0; i < Tconnection; ++i)
      {
        data[index] += reinterpret_cast<const TLeftElement *>(&left)[left_offset + i] * right[i][column];
      }
      index++;
    }
  }
  return tResult(data);
}

template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TVectorElement>
const tVector < Trows, decltype((TMatrixElement() * TVectorElement()) + (TMatrixElement() * TVectorElement())), vector::Cartesian > operator *(const math::tMatrix<Trows, Tcolumns, TMatrixElement> &matrix, const tVector<Tcolumns, TVectorElement, vector::Cartesian> &vector)
{
  typedef tVector < Trows, decltype((TMatrixElement() * TVectorElement()) + (TMatrixElement() * TVectorElement())), vector::Cartesian > tResult;
  typename tResult::tElement data[Trows];
  for (size_t row = 0; row < Trows; ++row)
  {
    data[row] = 0;
    const size_t matrix_offset = row * Tcolumns;
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      data[row] += reinterpret_cast<const TMatrixElement *>(&matrix)[matrix_offset + column] * reinterpret_cast<const TVectorElement *>(&vector)[column];
    }
  }
  return tResult(data);
}

template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TVectorElement>
const tVector < Tcolumns, decltype((TMatrixElement() * TVectorElement()) + (TMatrixElement() * TVectorElement())), vector::Cartesian > operator *(const tVector<Trows, TVectorElement, vector::Cartesian> &vector, const math::tMatrix<Trows, Tcolumns, TMatrixElement> &matrix)
{
  typedef tVector < Tcolumns, decltype((TMatrixElement() * TVectorElement()) + (TMatrixElement() * TVectorElement())), vector::Cartesian > tResult;
  typename tResult::tElement data[Tcolumns];
  for (size_t column = 0; column < Tcolumns; ++column)
  {
    data[column] = 0;
    for (size_t row = 0; row < Trows; ++row)
    {
      data[column] += reinterpret_cast<const TVectorElement *>(&vector)[row] * reinterpret_cast<const TMatrixElement *>(&matrix)[row * Tcolumns + column];
    }
  }
  return tResult(data);
}

template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TScalar>
const typename std::enable_if <std::is_scalar<TScalar>::value, math::tMatrix <Trows, Tcolumns, decltype(TMatrixElement() * TScalar())>>::type operator *(const math::tMatrix<Trows, Tcolumns, TMatrixElement> &matrix, const TScalar scalar)
{
  typedef math::tMatrix <Trows, Tcolumns, decltype(TMatrixElement() * TScalar())> tResult;
  typename tResult::tElement data[sizeof(tResult) / sizeof(typename tResult::tElement)];
  for (size_t i = 0; i < sizeof(tResult) / sizeof(typename tResult::tElement); ++i)
  {
    data[i] = reinterpret_cast<const TMatrixElement *>(&matrix)[i] * scalar;
  }
  return *reinterpret_cast<tResult *>(&data);
}

template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TScalar>
const typename std::enable_if <std::is_scalar<TScalar>::value, math::tMatrix <Trows, Tcolumns, decltype(TMatrixElement() * TScalar())>>::type operator *(const TScalar scalar, const math::tMatrix<Trows, Tcolumns, TMatrixElement> &matrix)
{
  return matrix * scalar;
}

template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TScalar>
const typename std::enable_if <std::is_scalar<TScalar>::value, math::tMatrix <Trows, Tcolumns, decltype(TMatrixElement() * TScalar())>>::type operator / (const math::tMatrix<Trows, Tcolumns, TMatrixElement> &matrix, const TScalar scalar)
{
  return matrix * (1 / scalar);
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
