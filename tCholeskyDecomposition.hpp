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
/*!\file    rrlib/math/tCholeskyDecomposition.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-30
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

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
// tCholeskyDecomposition constructors
//----------------------------------------------------------------------
template <size_t Trank, typename TElement>
tCholeskyDecomposition<Trank, TElement>::tCholeskyDecomposition(const tMatrix<Trank, Trank, TElement> &matrix)
{
  // FIXME: check if matrix is symmetric
  for (size_t step = 0; step < Trank; ++step)
  {
    this->cholesky_matrix[step][step] = matrix[step][step];
    for (size_t column = 0; column < step; ++column)
    {
      this->cholesky_matrix[step][step] -= this->cholesky_matrix[step][column] * this->cholesky_matrix[step][column];
    }

    if (this->cholesky_matrix[step][step] <= 0)
    {
      throw std::logic_error("Matrix not positive definite!");
    }

    this->cholesky_matrix[step][step] = std::sqrt(this->cholesky_matrix[step][step]);

    for (size_t row = step + 1; row < Trank; ++row)
    {
      this->cholesky_matrix[row][step] = matrix[row][step];
      for (size_t column = 0; column < step; ++column)
      {
        this->cholesky_matrix[row][step] -= this->cholesky_matrix[row][column] * this->cholesky_matrix[step][column];
      }
      this->cholesky_matrix[row][step] /= this->cholesky_matrix[step][step];
    }
  }
}

//----------------------------------------------------------------------
// tCholeskyDecomposition Solve
//----------------------------------------------------------------------
template <size_t Trank, typename TElement>
const tVector<Trank, TElement> tCholeskyDecomposition<Trank, TElement>::Solve(const tVector<Trank, TElement> &right_side) const
{
  TElement temp[Trank];
  for (size_t row = 0; row < Trank; ++row)
  {
    temp[row] = right_side[row];
    for (size_t column = 0; column < row; ++column)
    {
      temp[row] -= this->cholesky_matrix[row][column] * temp[column];
    }
    temp[row] /= this->cholesky_matrix[row][row];
  }
  TElement result[Trank];
  for (size_t step = 0; step < Trank; ++step)
  {
    size_t row = Trank - step - 1;
    result[row] = temp[row];
    for (size_t column = row + 1; column < Trank; ++column)
    {
      result[row] -= this->cholesky_matrix[column][row] * result[column];
    }
    result[row] /= this->cholesky_matrix[row][row];
  }
  return tVector<Trank, TElement>(result);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
