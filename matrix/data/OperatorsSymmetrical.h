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
/*!\file    OperatorsSymmetrical.h
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
#ifndef _rrlib_math_matrix_include_guard_
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef _rrlib_math_matrix_data_OperatorsSymmetrical_h_
#define _rrlib_math_matrix_data_OperatorsSymmetrical_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <istream>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_same.hpp>

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

template <size_t Trows, size_t Tcolumns, typename TLeftElement, typename TRightElement, template <size_t, size_t, typename> class TRightData>
const typename boost::disable_if<boost::is_same<Symmetrical<1, 1, int>, TRightData<1, 1, int> >, tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> >::type operator + (const tMatrix<Trows, Tcolumns, TLeftElement, Symmetrical> &left, const tMatrix<Trows, Tcolumns, TRightElement, TRightData> &right)
{
  typedef tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> tResultType;
  typename tResultType::tElementType data[Trows * Tcolumns];
  size_t index = 0;
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      const size_t left_index = column > row ? column * (column + 1) / 2 + row : row * (row + 1) / 2 + column;
      data[index] = reinterpret_cast<const TLeftElement *>(&left)[left_index] + right[row][column];
      index++;
    }
  }
  return tResultType(data);
}

template <size_t Trows, size_t Tcolumns, typename TLeftElement, typename TRightElement>
const tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> operator + (const tMatrix<Trows, Tcolumns, TLeftElement, Symmetrical> &left, const tMatrix<Trows, Tcolumns, TRightElement, LowerTriangle> &right)
{
  typedef tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> tResultType;
  typename tResultType::tElementType data[Trows * Tcolumns];
  size_t index = 0;
  for (size_t row = 0; row < Trows; ++row)
  {
    const size_t right_offset = row  * (row  + 1) / 2;
    for (size_t column = 0; column <= row; ++column)
    {
      const size_t left_index = column > row ? column * (column + 1) / 2 + row : row * (row + 1) / 2 + column;
      data[index] = reinterpret_cast<const TLeftElement *>(&left)[left_index] + reinterpret_cast<const TRightElement *>(&right)[right_offset + column];
      index++;
    }
    for (size_t column = row + 1; column < Tcolumns; ++column)
    {
      const size_t left_index = column > row ? column * (column + 1) / 2 + row : row * (row + 1) / 2 + column;
      data[index] = reinterpret_cast<const TLeftElement *>(&left)[left_index];
      index++;
    }
  }
  return tResultType(data);
}

template <size_t Trows, size_t Tcolumns, typename TLeftElement, typename TRightElement>
const tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> operator + (const tMatrix<Trows, Tcolumns, TLeftElement, Symmetrical> &left, const tMatrix<Trows, Tcolumns, TRightElement, UpperTriangle> &right)
{
  typedef tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> tResultType;
  typename tResultType::tElementType data[Trows * Tcolumns];
  size_t index = 0;
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < row; ++column)
    {
      const size_t left_index = column > row ? column * (column + 1) / 2 + row : row * (row + 1) / 2 + column;
      data[index] = reinterpret_cast<const TLeftElement *>(&left)[left_index];
      index++;
    }
    for (size_t column = row; column < Tcolumns; ++column)
    {
      const size_t left_index = column > row ? column * (column + 1) / 2 + row : row * (row + 1) / 2 + column;
      data[index] = reinterpret_cast<const TLeftElement *>(&left)[left_index] + reinterpret_cast<const TRightElement *>(&right)[column * (column + 1) / 2 + row];
      index++;
    }
  }
  return tResultType(data);
}

template <size_t Trows, size_t Tconnection, size_t Tcolumns, typename TLeftElement, typename TRightElement, template <size_t, size_t, typename> class TRightData>
const tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> operator *(const tMatrix<Trows, Tconnection, TLeftElement, Symmetrical> &left, const tMatrix<Tconnection, Tcolumns, TRightElement, TRightData> &right)
{
  typedef tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> tResultType;
  typename tResultType::tElementType data[Trows * Tcolumns];
  memset(data, 0, sizeof(data));
  size_t index = 0;
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      for (size_t i = 0; i < Tconnection; ++i)
      {
        const size_t left_index = i > row ? i * (i + 1) / 2 + row : row * (row + 1) / 2 + i;
        data[index] += reinterpret_cast<const TLeftElement *>(&left)[left_index] * right[i][column];
      }
      index++;
    }
  }
  return tResultType(data);
}

template <size_t Trows, size_t Tconnection, size_t Tcolumns, typename TLeftElement, typename TRightElement>
const tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> operator *(const tMatrix<Trows, Tconnection, TLeftElement, Symmetrical> &left, const tMatrix<Tconnection, Tcolumns, TRightElement, LowerTriangle> &right)
{
  typedef tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> tResultType;
  typename tResultType::tElementType data[Trows * Tcolumns];
  memset(data, 0, sizeof(data));
  size_t index = 0;
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      for (size_t i = column; i < Tconnection; ++i)
      {
        const size_t left_index = i > row ? i * (i + 1) / 2 + row : row * (row + 1) / 2 + i;
        data[index] += reinterpret_cast<const TLeftElement *>(&left)[left_index] * reinterpret_cast<const TRightElement *>(&right)[i * (i + 1) / 2 + column];
      }
      index++;
    }
  }
  return tResultType(data);
}

template <size_t Trows, size_t Tconnection, size_t Tcolumns, typename TLeftElement, typename TRightElement>
const tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> operator *(const tMatrix<Trows, Tconnection, TLeftElement, Symmetrical> &left, const tMatrix<Tconnection, Tcolumns, TRightElement, UpperTriangle> &right)
{
  typedef tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> tResultType;
  typename tResultType::tElementType data[Trows * Tcolumns];
  memset(data, 0, sizeof(data));
  size_t index = 0;
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      const size_t right_offset = column * (column + 1) / 2;
      for (size_t i = 0; i <= column; ++i)
      {
        const size_t left_index = i > row ? i * (i + 1) / 2 + row : row * (row + 1) / 2 + i;
        data[index] += reinterpret_cast<const TLeftElement *>(&left)[left_index] * reinterpret_cast<const TRightElement *>(&right)[right_offset + i];
      }
      index++;
    }
  }
  return tResultType(data);
}

template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TVectorElement>
const tVector<Trows, typename until_0x::Auto<TMatrixElement, TVectorElement>::type, vector::Cartesian> operator *(const tMatrix<Trows, Tcolumns, TMatrixElement, Symmetrical> &matrix, const tVector<Tcolumns, TVectorElement, vector::Cartesian> &vector)
{
  typedef tVector<Trows, typename until_0x::Auto<TMatrixElement, TVectorElement>::type, vector::Cartesian> tResultType;
  typename tResultType::tElementType data[Trows];
  for (size_t row = 0; row < Trows; ++row)
  {
    data[row] = 0;
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      const size_t matrix_index = column > row ? column * (column + 1) / 2 + row : row * (row + 1) / 2 + column;
      data[row] += reinterpret_cast<const TMatrixElement *>(&matrix)[matrix_index] * reinterpret_cast<const TVectorElement *>(&vector)[column];
    }
  }
  return tResultType(data);
}

template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TVectorElement>
const tVector<Tcolumns, typename until_0x::Auto<TMatrixElement, TVectorElement>::type, vector::Cartesian> operator *(const tVector<Trows, TVectorElement, vector::Cartesian> &vector, const tMatrix<Trows, Tcolumns, TMatrixElement, Symmetrical> &matrix)
{
  return matrix * vector;
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
