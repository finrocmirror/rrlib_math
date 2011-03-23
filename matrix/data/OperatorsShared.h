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
/*!\file    OperatorsShared.h
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
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_scalar.hpp>

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_
#include "rrlib/serialization/tStringInputStream.h"
#include "rrlib/serialization/tStringOutputStream.h"
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
namespace matrix
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------

template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TData>
std::ostream &operator << (std::ostream &stream, const math::tMatrix<Trows, Tcolumns, TElement, TData> &matrix)
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

template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TData>
std::istream &operator >> (std::istream &stream, math::tMatrix<Trows, Tcolumns, TElement, TData> &matrix)
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

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TData>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const math::tMatrix<Trows, Tcolumns, TElement, TData> &matrix)
{
  for (size_t i = 0; i < Trows * Tcolumns; ++i)
  {
    stream << vector[i];
  }
  return stream;
}

template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TData>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, math::tMatrix<Trows, Tcolumns, TElement, TData> &matrix)
{
  for (size_t i = 0; i < Trows * Tcolumns; ++i)
  {
    stream >> matrix[i];
  }
  return stream;
}

template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TData>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const math::tMatrix<Trows, Tcolumns, TElement, TData> &matrix)
{
  std::stringstream string_stream;
  string_stream << matrix;
  stream << string_stream.str();
  return stream;
}

template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TData>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, math::tMatrix<Trows, Tcolumns, TElement, TData> &matrix)
{
  std::istringstream string_stream(stream.ReadLine());
  string_stream >> matrix;
  return stream;
}

#endif

template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TData>
const math::tMatrix<Trows, Tcolumns, TElement, TData> operator - (const math::tMatrix<Trows, Tcolumns, TElement, TData> &matrix)
{
  typedef math::tMatrix<Trows, Tcolumns, TElement, TData> tResult;
  typename tResult::tElement data[sizeof(tResult) / sizeof(typename tResult::tElement)];
  for (size_t i = 0; i < sizeof(tResult) / sizeof(typename tResult::tElement); ++i)
  {
    data[i] = -reinterpret_cast<const TElement *>(&matrix)[i];
  }
  return *reinterpret_cast<tResult *>(&data);
}

template <size_t Trows, size_t Tcolumns, typename TLeftElement, typename TRightElement, template <size_t, size_t, typename> class TData>
const math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, TData> operator + (const math::tMatrix<Trows, Tcolumns, TLeftElement, TData> &left, const math::tMatrix<Trows, Tcolumns, TRightElement, TData> &right)
{
  typedef math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, TData> tResult;
  typename tResult::tElement data[sizeof(tResult) / sizeof(typename tResult::tElement)];
  for (size_t i = 0; i < sizeof(tResult) / sizeof(typename tResult::tElement); ++i)
  {
    data[i] = reinterpret_cast<const TLeftElement *>(&left)[i] + reinterpret_cast<const TRightElement *>(&right)[i];
  }
  return *reinterpret_cast<tResult *>(&data);
}

namespace
{
template <size_t Trows, size_t Tcolumns, typename TLeftElement, typename TRightElement, template <size_t, size_t, typename> class TLeftData, template <size_t, size_t, typename> class TRightData>
class MatrixAddition
{
  static math::tMatrix<Trows, Tcolumns, TLeftElement, TLeftData> CreateLeft();
  static math::tMatrix<Trows, Tcolumns, TRightElement, TRightData> CreateRight();
public:
  typedef typeof(CreateLeft() + CreateRight()) type;
};
}
template <size_t Trows, size_t Tcolumns, typename TLeftElement, typename TRightElement, template <size_t, size_t, typename> class TLeftData, template <size_t, size_t, typename> class TRightData>
inline const typename MatrixAddition<Trows, Tcolumns, TLeftElement, TRightElement, TLeftData, TRightData>::type operator - (const math::tMatrix<Trows, Tcolumns, TLeftElement, TLeftData> &left, const math::tMatrix<Trows, Tcolumns, TRightElement, TRightData> &right)
{
  return left + -right;
}






//template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TVectorElement, template <size_t, size_t, typename> class TData>
//const tVector<Trows, typename until_0x::Auto<TMatrixElement, TVectorElement>::type, vector::Cartesian> operator * (const math::tMatrix<Trows, Tcolumns, TMatrixElement, TData> &matrix, const tVector<Tcolumns, TVectorElement, vector::Cartesian> &vector)
//{
//  typedef tVector<Trows, typename until_0x::Auto<TMatrixElement, TVectorElement>::type, vector::Cartesian> tResult;
//  typename tResult::tElement data[Trows];
//  for (size_t row = 0; row < Trows; ++row)
//  {
//    data[row] = 0;
//    for (size_t column = 0; column < Tcolumns; ++column)
//    {
//      data[row] += matrix[row][column] * reinterpret_cast<const TVectorElement *>(&vector)[column];
//    }
//  }
//  return tResult(data);
//}
//
//template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TVectorElement, template <size_t, size_t, typename> class TData>
//const tVector<Tcolumns, typename until_0x::Auto<TMatrixElement, TVectorElement>::type, vector::Cartesian> operator * (const tVector<Trows, TVectorElement, vector::Cartesian> &vector, const math::tMatrix<Trows, Tcolumns, TMatrixElement, TData> &matrix)
//{
//  typedef tVector<Tcolumns, typename until_0x::Auto<TMatrixElement, TVectorElement>::type, vector::Cartesian> tResult;
//  typename tResult::tElement data[Tcolumns];
//  for (size_t column = 0; column < Tcolumns; ++column)
//  {
//    data[column] = 0;
//    for (size_t row = 0; row < Trows; ++row)
//    {
//      data[column] += reinterpret_cast<const TVectorElement *>(&vector)[row] * matrix[row][column];
//    }
//  }
//  return tResult(data);
//}







template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TScalar, template <size_t, size_t, typename> class TData>
const typename boost::enable_if<boost::is_scalar<TScalar>, math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TMatrixElement, TScalar>::type, TData> >::type operator *(const math::tMatrix<Trows, Tcolumns, TMatrixElement, TData> &matrix, const TScalar scalar)
{
  typedef math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TMatrixElement, TScalar>::type, TData> tResult;
  typename tResult::tElement data[sizeof(tResult) / sizeof(typename tResult::tElement)];
  for (size_t i = 0; i < sizeof(tResult) / sizeof(typename tResult::tElement); ++i)
  {
    data[i] = reinterpret_cast<const TMatrixElement *>(&matrix)[i] * scalar;
  }
  return *reinterpret_cast<tResult *>(&data);
}

template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TScalar, template <size_t, size_t, typename> class TData>
const typename boost::enable_if<boost::is_scalar<TScalar>, math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TMatrixElement, TScalar>::type, TData> >::type operator *(const TScalar scalar, const math::tMatrix<Trows, Tcolumns, TMatrixElement, TData> &matrix)
{
  return matrix * scalar;
}

template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TScalar, template <size_t, size_t, typename> class TData>
const typename boost::enable_if<boost::is_scalar<TScalar>, math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TMatrixElement, TScalar>::type, TData> >::type operator / (const math::tMatrix<Trows, Tcolumns, TMatrixElement, TData> &matrix, const TScalar scalar)
{
  return matrix *(1 / scalar);
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
