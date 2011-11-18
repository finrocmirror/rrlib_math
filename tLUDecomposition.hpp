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
/*!\file    tLUDecomposition.hpp
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
tLUDecomposition<Trank, TElement>::tLUDecomposition(const tMatrix<Trows, Trank, TElement, matrix::Full> &matrix)
{
  this->FullMatrixDecomposition(matrix);
}

template <size_t Trank, typename TElement>
tLUDecomposition<Trank, TElement>::tLUDecomposition(const tMatrix<Trank, Trank, TElement, matrix::LowerTriangle> &matrix)
{
  for (size_t i = 0; i < Trank; ++i)
  {
    if (matrix[i][i] == 0)
    {
      throw std::logic_error("FIXME: Matrix not of expected rank");
    }
  }

  for (size_t i = 0; i < Trank; ++i)
  {
    this->upper[i][i] = matrix[i][i];
  }
  this->lower = matrix;

  for (size_t column = 0; column < Trank; ++column)
  {
    for (size_t row = column; row < Trank; ++row)
    {
      this->lower[row][column] /= this->upper[column][column];
    }
  }

  for (size_t i = 0; i < Trank - 1; ++i)
  {
    this->pivot[i] = i;
  }
}

template <size_t Trank, typename TElement>
tLUDecomposition<Trank, TElement>::tLUDecomposition(const tMatrix<Trank, Trank, TElement, matrix::UpperTriangle> &matrix)
{
  for (size_t i = 0; i < Trank; ++i)
  {
    if (matrix[i][i] == 0)
    {
      throw std::logic_error("FIXME: Matrix not of expected rank");
    }
  }

  for (size_t i = 0; i < Trank; ++i)
  {
    this->lower[i][i] = 1;
  }
  this->upper = matrix;

  for (size_t i = 0; i < Trank - 1; ++i)
  {
    this->pivot[i] = i;
  }
}

template <size_t Trank, typename TElement>
tLUDecomposition<Trank, TElement>::tLUDecomposition(const tMatrix<Trank, Trank, TElement, matrix::Symmetrical> &matrix)
{
  try
  {
    tCholeskyDecomposition<Trank, TElement> cholesky_decomposition(matrix);
    this->lower = cholesky_decomposition.C();
    this->upper = cholesky_decomposition.C().Transposed();

    for (size_t i = 0; i < Trank - 1; ++i)
    {
      this->pivot[i] = i;
    }
  }
  catch (const std::logic_error &e)
  {
    this->FullMatrixDecomposition(matrix);
  }
}

//----------------------------------------------------------------------
// tLUDecomposition Solve
//----------------------------------------------------------------------
template <size_t Trank, typename TElement>
template <size_t Tdimension>
const tVector<Trank, TElement> tLUDecomposition<Trank, TElement>::Solve(const tVector<Tdimension, TElement> &right_side) const
{
  static_assert(Tdimension >= Trank, "Dimension of given vector is too small");

  tVector<Tdimension, TElement> temp(right_side);
  for (size_t i = 0; i < Trank - 1; ++i)
  {
    TElement swap = temp[i];
    temp[i] = temp[this->pivot[i]];
    temp[this->pivot[i]] = swap;
  }

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
// tLUDecomposition FullMatrixDecomposition
//----------------------------------------------------------------------
template <size_t Trank, typename TElement>
template <size_t Trows, template <size_t, size_t, typename> class TData>
void tLUDecomposition<Trank, TElement>::FullMatrixDecomposition(const tMatrix<Trows, Trank, TElement, TData> &matrix)
{
  static_assert(Trows >= Trank, "Matrix can not have given rank");

  tMatrix<Trows, Trank, TElement, matrix::Full> temp_matrix(matrix);

  for (size_t step = 0; step < Trank - 1; ++step)
  {
    TElement relative_maximum = 0;
    this->pivot[step] = step;
    for (size_t row = step; row < Trank; ++row)
    {
      TElement row_sum = 0;
      for (size_t column = step; column < Trank; ++column)
      {
        row_sum += AbsoluteValue(temp_matrix[row][column]);
      }
      if (row_sum != 0)
      {
        TElement normalized_first = AbsoluteValue(temp_matrix[row][step]) / row_sum;
        if (normalized_first > relative_maximum)
        {
          relative_maximum = normalized_first;
          this->pivot[step] = row;
        }
      }
    }

    if (relative_maximum == 0)
    {
      throw std::logic_error("FIXME: Matrix not of expected rank");
    }

    if (this->pivot[step] != step)
    {
      for (size_t column = 0; column < Trank; ++column)
      {
        TElement temp = temp_matrix[step][column];
        temp_matrix[step][column] = temp_matrix[this->pivot[step]][column];
        temp_matrix[this->pivot[step]][column] = temp;
      }
    }

    for (size_t row = step + 1; row < Trank; ++row)
    {
      temp_matrix[row][step] /= temp_matrix[step][step];
      for (size_t column = step + 1; column < Trank; ++column)
      {
        temp_matrix[row][column] -= temp_matrix[row][step] * temp_matrix[step][column];
      }
    }
  }
  if (temp_matrix[Trank - 1][Trank - 1] == 0)
  {
    throw std::logic_error("FIXME: Matrix not of expected rank");
  }

  for (size_t row = Trank; row < Trows; ++row)
  {
    for (size_t column = 0; column < Trank; ++column)
    {
      if (temp_matrix[row][column] != 0)
      {
        throw std::logic_error("FIXME: Matrix not of expected rank");
      }
    }
  }

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
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
