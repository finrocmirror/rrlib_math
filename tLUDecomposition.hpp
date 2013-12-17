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
/*!\file    rrlib/math/tLUDecomposition.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-27
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tCholeskyDecomposition.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace math
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tLUDecomposition constructors
//----------------------------------------------------------------------
template <size_t Trank, typename TElement>
template <size_t Trows>
tLUDecomposition<Trank, TElement>::tLUDecomposition(const tMatrix<Trows, Trank, TElement> &matrix)
{
  static_assert(Trows >= Trank, "Matrix can not have given rank");

  tMatrix<Trows, Trank, TElement> temp_matrix(matrix);

  for (size_t step = 0; step < std::min(Trank, Trows); ++step)
  {
    RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "step: ", step);
    TElement relative_maximum = 0;
    this->pivot[step] = step;
    for (size_t row = step; row < Trows; ++row)
    {
      RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "  row: ", row);
      TElement row_sum = 0;
      for (size_t column = step; column < Trank; ++column)
      {
        row_sum += AbsoluteValue(temp_matrix[row][column]);
      }
      RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "    sum: ", row_sum);
      if (row_sum != 0)
      {
        TElement normalized_first = AbsoluteValue(temp_matrix[row][step]) / row_sum;
        RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "    normalized_first: ", normalized_first);
        if (normalized_first > relative_maximum)
        {
          relative_maximum = normalized_first;
          this->pivot[step] = row;
        }
      }
    }

    if (relative_maximum == 0)
    {
      throw std::logic_error("Matrix not of expected rank");
    }

    if (this->pivot[step] != step)
    {
      RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "    pivotize: ", step, " <-> ", this->pivot[step]);
      for (size_t column = 0; column < Trank; ++column)
      {
        std::swap(temp_matrix[step][column], temp_matrix[this->pivot[step]][column]);
      }
    }

    RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "    a[k][k] = a[", step, "][", step, "] = ", temp_matrix[step][step]);
    for (size_t row = step + 1; row < Trows; ++row)
    {
      temp_matrix[row][step] /= temp_matrix[step][step];
      RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "      a[i][k] = a[", row, "][", step, "] = ", temp_matrix[row][step]);
      for (size_t column = step + 1; column < Trank; ++column)
      {
        temp_matrix[row][column] -= temp_matrix[row][step] * temp_matrix[step][column];
        RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "      a[i][j] = a[", row, "][", column, "] = ", temp_matrix[row][column]);
      }
    }
  }
  RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "matrix: ", temp_matrix);
  if (temp_matrix[Trank - 1][Trank - 1] == 0)
  {
    throw std::logic_error("Matrix not of expected rank");
  }

// FIXME: we need a different check if there are linear independent rows left
//        as these are not eliminated but remain as multiples of the first
//        Trank rows
//  for (size_t row = Trank; row < Trows; ++row)
//  {
//    for (size_t column = 0; column < Trank; ++column)
//    {
//      if (temp_matrix[row][column] != 0)
//      {
//        throw std::logic_error(Matrix not of expected rank");
//      }
//    }
//  }

  for (size_t row = 0; row < Trank; ++row)
  {
    for (size_t column = 0; column < row; ++column)
    {
      this->lower[row][column] = temp_matrix[row][column];
    }
    this->lower[row][row] = 1;
    for (size_t column = row; column < Trank; ++column)
    {
      this->upper[row][column] = temp_matrix[row][column];
    }
  }

  RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "lower: ", this->lower);
  RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "upper: ", this->upper);
  RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "pivot: ", this->pivot);
}

//----------------------------------------------------------------------
// tLUDecomposition Solve
//----------------------------------------------------------------------
template <size_t Trank, typename TElement>
template <size_t Tdimension>
const tVector<Trank, TElement> tLUDecomposition<Trank, TElement>::Solve(const tVector<Tdimension, TElement> &right_side) const
{
  static_assert(Tdimension >= Trank, "Dimension of given vector is too small");

  RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "vector: ", right_side);
  RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "pivot: ", this->pivot);

  tVector<Tdimension, TElement> temp(right_side);
  for (size_t i = 0; i < Trank; ++i)
  {
    std::swap(temp[i], temp[this->pivot[i]]);
  }

  RRLIB_LOG_PRINT(DEBUG_VERBOSE_3, "pivotized: ", temp);

  for (size_t row = 0; row < Trank; ++row)
  {
    for (size_t column = 0; column < row; ++column)
    {
      temp[row] -= this->lower[row][column] * temp[column];
    }
  }
  TElement result[Trank];
  for (size_t step = 0; step < Trank; ++step)
  {
    size_t row = Trank - step - 1;
    result[row] = temp[row];
    for (size_t column = row + 1; column < Trank; ++column)
    {
      result[row] -= this->upper[row][column] * result[column];
    }
    result[row] /= this->upper[row][row];
  }
  return tVector<Trank, TElement>(result);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
