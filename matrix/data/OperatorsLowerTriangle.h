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
/*!\file    OperatorsLowerTriangle.h
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

#ifndef __rrlib__math__matrix__data__OperatorsLowerTriangle_h__
#define __rrlib__math__matrix__data__OperatorsLowerTriangle_h__

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
const typename boost::disable_if<boost::is_same<LowerTriangle<1, 1, int>, TRightData<1, 1, int> >, math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> >::type operator + (const math::tMatrix<Trows, Tcolumns, TLeftElement, LowerTriangle> &left, const math::tMatrix<Trows, Tcolumns, TRightElement, TRightData> &right)
{
  typedef math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> tResult;
  typename tResult::tElement data[Trows * Tcolumns];
  size_t index = 0;
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column <= row; ++column)
    {
      data[index] = reinterpret_cast<const TLeftElement *>(&left)[row * (row + 1) / 2  + column] + right[row][column];
      index++;
    }
    for (size_t column = row + 1; column < Tcolumns; ++column)
    {
      data[index] = right[row][column];
      index++;
    }
  }
  return tResult(data);
}

template <size_t Trows, size_t Tcolumns, typename TLeftElement, typename TRightElement>
const math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> operator + (const math::tMatrix<Trows, Tcolumns, TLeftElement, LowerTriangle> &left, const math::tMatrix<Trows, Tcolumns, TRightElement, UpperTriangle> &right)
{
  typedef math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> tResult;
  typename tResult::tElement data[Trows * Tcolumns];
  size_t index = 0;
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < row; ++column)
    {
      data[index] = reinterpret_cast<const TLeftElement *>(&left)[row * (row + 1) / 2  + column];
      index++;
    }
    index++;
    for (size_t column = row + 1; column < Tcolumns; ++column)
    {
      data[index] = reinterpret_cast<const TRightElement *>(&right)[column * (column + 1) / 2  + row];
      index++;
    }
  }
  for (size_t i = 0; i <= Trows; ++i)
  {
    const size_t index = i * Tcolumns + i;
    const size_t source_index = i * (i + 1) / 2 + i;
    data[index] = reinterpret_cast<const TLeftElement *>(&left)[source_index] + reinterpret_cast<const TRightElement *>(&right)[source_index];
  }
  return tResult(data);
}

template <size_t Trows, size_t Tconnection, size_t Tcolumns, typename TLeftElement, typename TRightElement>
const math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, LowerTriangle> operator *(const math::tMatrix<Trows, Tconnection, TLeftElement, LowerTriangle> &left, const math::tMatrix<Tconnection, Tcolumns, TRightElement, LowerTriangle> &right)
{
  typedef math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, LowerTriangle> tResult;
  typename tResult::tElement data[sizeof(tResult) / sizeof(typename tResult::tElement)];
  std::memset(data, 0, sizeof(data));
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column <= row; ++column)
    {
      const size_t index = row * (row + 1) / 2 + column;
      const size_t left_offset = row * (row + 1) / 2;
      for (size_t i = column; i <= row; ++i)
      {
        data[index] += reinterpret_cast<const TLeftElement *>(&left)[left_offset + i] * reinterpret_cast<const TRightElement *>(&right)[i * (i + 1) / 2 + column];
      }
    }
  }
  return *reinterpret_cast<tResult *>(data);
}

template <size_t Trows, size_t Tconnection, size_t Tcolumns, typename TLeftElement, typename TRightElement, template <size_t, size_t, typename> class TRightData>
const typename boost::disable_if<boost::is_same<LowerTriangle<1, 1, int>, TRightData<1, 1, int> >, math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> >::type operator *(const math::tMatrix<Trows, Tconnection, TLeftElement, LowerTriangle> &left, const math::tMatrix<Tconnection, Tcolumns, TRightElement, TRightData> &right)
{
  typedef math::tMatrix<Trows, Tcolumns, typename until_0x::Auto<TLeftElement, TRightElement>::type, Full> tResult;
  typename tResult::tElement data[Trows * Tcolumns];
  std::memset(data, 0, sizeof(data));
  size_t index = 0;
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      const size_t left_offset = row * (row + 1) / 2;
      for (size_t i = 0; i <= row; ++i)
      {
        data[index] += reinterpret_cast<const TLeftElement *>(&left)[left_offset + i] * right[i][column];
      }
      index++;
    }
  }
  return tResult(data);
}

template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TVectorElement>
const tVector<Trows, typename until_0x::Auto<TMatrixElement, TVectorElement>::type, vector::Cartesian> operator *(const math::tMatrix<Trows, Tcolumns, TMatrixElement, LowerTriangle> &matrix, const tVector<Tcolumns, TVectorElement, vector::Cartesian> &vector)
{
  typedef tVector<Trows, typename until_0x::Auto<TMatrixElement, TVectorElement>::type, vector::Cartesian> tResult;
  typename tResult::tElement data[Trows];
  for (size_t row = 0; row < Trows; ++row)
  {
    data[row] = 0;
    const size_t matrix_offset = row * (row + 1) / 2;
    for (size_t column = 0; column <= row; ++column)
    {
      data[row] += reinterpret_cast<const TMatrixElement *>(&matrix)[matrix_offset + column] * reinterpret_cast<const TVectorElement *>(&vector)[column];
    }
  }
  return tResult(data);
}

template <size_t Trows, size_t Tcolumns, typename TMatrixElement, typename TVectorElement>
const tVector<Tcolumns, typename until_0x::Auto<TMatrixElement, TVectorElement>::type, vector::Cartesian> operator *(const tVector<Trows, TVectorElement, vector::Cartesian> &vector, const math::tMatrix<Trows, Tcolumns, TMatrixElement, LowerTriangle> &matrix)
{
  typedef tVector<Tcolumns, typename until_0x::Auto<TMatrixElement, TVectorElement>::type, vector::Cartesian> tResult;
  typename tResult::tElement data[Tcolumns];
  for (size_t column = 0; column < Tcolumns; ++column)
  {
    data[column] = 0;
    for (size_t row = column; row < Trows; ++row)
    {
      data[column] += reinterpret_cast<const TVectorElement *>(&vector)[row] * reinterpret_cast<const TMatrixElement *>(&matrix)[row * (row + 1) / 2 + column];
    }
  }
  return tResult(data);
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
