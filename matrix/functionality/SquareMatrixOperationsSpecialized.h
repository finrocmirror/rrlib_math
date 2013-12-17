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
/*!\file    rrlib/math/matrix/functionality/SquareMatrixOperationsSpecialized.h
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

#ifndef __rrlib__math__matrix__functionality__SquareMatrixOperationsSpecialized_h__
#define __rrlib__math__matrix__functionality__SquareMatrixOperationsSpecialized_h__

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
template <size_t Trows, size_t Tcolumns, typename TElement>
class SquareMatrixOperationsSpecialized
{
  typedef math::tMatrix<Trows, Tcolumns, TElement> tMatrix;

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline SquareMatrixOperationsSpecialized() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  SquareMatrixOperationsSpecialized(const SquareMatrixOperationsSpecialized &);
  SquareMatrixOperationsSpecialized &operator = (const SquareMatrixOperationsSpecialized &);

};

/*!
 *
 */
template <size_t Tdimension, typename TElement>
class SquareMatrixOperationsSpecialized<Tdimension, Tdimension, TElement>
{
  typedef math::tMatrix<Tdimension, Tdimension, TElement> tMatrix;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline const TElement Determinant() const __attribute__((always_inline, flatten))
  {
    const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
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

  inline const tMatrix Inverted() const __attribute__((always_inline, flatten))
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
    return tMatrix(data) / determinant;
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline SquareMatrixOperationsSpecialized() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  SquareMatrixOperationsSpecialized(const SquareMatrixOperationsSpecialized &);
  SquareMatrixOperationsSpecialized &operator = (const SquareMatrixOperationsSpecialized &);

  const math::tMatrix < Tdimension - 1, Tdimension - 1, TElement > ExtractSubMatrix(size_t cut_row, size_t cut_column) const
  {
    const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
    typedef math::tMatrix < Tdimension - 1, Tdimension - 1, TElement > tResult;
    typename tResult::tElement data[tResult::cROWS * tResult::cCOLUMNS];
    for (size_t row = 0; row < tResult::cROWS; ++row)
    {
      for (size_t column = 0; column < tResult::cCOLUMNS; ++column)
      {
        data[row * tResult::cCOLUMNS + column] = (*that)[row < cut_row ? row : row + 1][column < cut_column ? column : column + 1];
      }
    }
    return tResult(data);
  }

};

/*!
 *
 */
template <typename TElement>
class SquareMatrixOperationsSpecialized<2, 2, TElement>
{
  typedef math::tMatrix<2, 2, TElement> tMatrix;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline const TElement Determinant() const __attribute__((always_inline, flatten))
  {
    const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
    return (*that)[0][0] * (*that)[1][1] - (*that)[1][0] * (*that)[0][1];
  }

  inline const tMatrix Inverted() const __attribute__((always_inline, flatten))
  {
    const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
    TElement determinant = this->Determinant();
    if (determinant == 0)
    {
      throw std::logic_error("Inverse of singular matrix (determinant = 0) does not exist.");
    }
    return tMatrix((*that)[1][1], -(*that)[0][1], -(*that)[1][0], (*that)[0][0]) / determinant;
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline SquareMatrixOperationsSpecialized() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  SquareMatrixOperationsSpecialized(const SquareMatrixOperationsSpecialized &);
  SquareMatrixOperationsSpecialized &operator = (const SquareMatrixOperationsSpecialized &);

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
