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
/*!\file    SquareMatrixOperationsSpecialized.h
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

#ifndef _rrlib_math_matrix_functionality_SquareMatrixOperationsSpecialized_h_
#define _rrlib_math_matrix_functionality_SquareMatrixOperationsSpecialized_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <stdexcept>

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
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TData>
class SquareMatrixOperationsSpecialized
{
  typedef tMatrix<Trows, Tcolumns, TElement, TData> tMatrixType;

  SquareMatrixOperationsSpecialized(const SquareMatrixOperationsSpecialized &);
  SquareMatrixOperationsSpecialized &operator = (const SquareMatrixOperationsSpecialized &);

protected:

  inline SquareMatrixOperationsSpecialized() {}
};

/*!
 *
 */
template <size_t Tdimension, typename TElement, template <size_t, size_t, typename> class TData>
class SquareMatrixOperationsSpecialized<Tdimension, Tdimension, TElement, TData>
{
  typedef tMatrix<Tdimension, Tdimension, TElement, TData> tMatrixType;

  SquareMatrixOperationsSpecialized(const SquareMatrixOperationsSpecialized &);
  SquareMatrixOperationsSpecialized &operator = (const SquareMatrixOperationsSpecialized &);

  const tMatrix < Tdimension - 1, Tdimension - 1, TElement, Full > ExtractSubMatrix(size_t cut_row, size_t cut_column) const
  {
    const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
    typedef tMatrix < Tdimension - 1, Tdimension - 1, TElement, Full > tResultType;
    typename tResultType::tElementType data[tResultType::cROWS * tResultType::cCOLUMNS];
    for (size_t row = 0; row < tResultType::cROWS; ++row)
    {
      for (size_t column = 0; column < tResultType::cCOLUMNS; ++column)
      {
        data[row * tResultType::cCOLUMNS + column] = (*that)[row < cut_row ? row : row + 1][column < cut_column ? column : column + 1];
      }
    }
    return tResultType(data);
  }

protected:

  inline SquareMatrixOperationsSpecialized() {}

public:

  inline const TElement Determinant() const
  {
    const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
    TElement determinant = 0;
    int sign = 1;
    for (size_t column = 0; column < Tdimension; ++column)
    {
      if ((*that)[0][column] != 0)
      {
        determinant += sign * (*that)[0][column] * this->ExtractSubMatrix(0, column).Determinant();
      }
      sign = -sign;
    }
    return determinant;
  }

  inline const tMatrixType Inverted() const
  {
    TElement determinant = this->Determinant();
    if (determinant == 0)
    {
      throw std::logic_error("Inverse of singular matrix (determinant = 0) does not exist.");
    }
    TElement data[Tdimension * Tdimension];
    int row_sign = 1;
    for (size_t row = 0; row < Tdimension; ++row)
    {
      int sign = row_sign;
      for (size_t column = 0; column < Tdimension; ++column)
      {
        data[row * Tdimension + column] = sign * this->ExtractSubMatrix(column, row).Determinant();
        sign = -sign;
      }
      row_sign = -row_sign;
    }
    return tMatrixType(data) / determinant;
  }

};

/*!
 *
 */
template <typename TElement, template <size_t, size_t, typename> class TData>
class SquareMatrixOperationsSpecialized<2, 2, TElement, TData>
{
  typedef tMatrix<2, 2, TElement, TData> tMatrixType;

  SquareMatrixOperationsSpecialized(const SquareMatrixOperationsSpecialized &);
  SquareMatrixOperationsSpecialized &operator = (const SquareMatrixOperationsSpecialized &);

protected:

  inline SquareMatrixOperationsSpecialized() {}

public:

  inline const TElement Determinant() const
  {
    const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
    return (*that)[0][0] *(*that)[1][1] - (*that)[1][0] *(*that)[0][1];
  }

  inline const tMatrixType Inverted() const
  {
    const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
    TElement determinant = this->Determinant();
    if (determinant == 0)
    {
      throw std::logic_error("Inverse of singular matrix (determinant = 0) does not exist.");
    }
    return tMatrixType((*that)[1][1], -(*that)[0][1], -(*that)[1][0], (*that)[0][0]) / determinant;
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
