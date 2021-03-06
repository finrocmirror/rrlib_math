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
/*!\file    rrlib/math/matrix/functionality/HomogeneousMultiplication.h
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

#ifndef __rrlib__math__matrix__functionality__HomogeneousMultiplication_h__
#define __rrlib__math__matrix__functionality__HomogeneousMultiplication_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
template <size_t Trows, size_t Tcolumns, typename TElement>
class HomogeneousMultiplication
{
  typedef math::tMatrix<Trows, Tcolumns, TElement> tMatrix;

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline HomogeneousMultiplication() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  HomogeneousMultiplication(const HomogeneousMultiplication &);
  HomogeneousMultiplication &operator = (const HomogeneousMultiplication &);

};

/*!
 *
 */
template <size_t Tdimension, typename TElement>
class HomogeneousMultiplication<Tdimension, Tdimension, TElement>
{
  typedef math::tMatrix<Tdimension, Tdimension, TElement> tMatrix;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TVectorElement>
  inline const tVector < Tdimension - 1, decltype((TElement() * TVectorElement()) + (TElement() * TVectorElement())), vector::Cartesian > MultiplyHomogeneously(const tVector < Tdimension - 1, TVectorElement, vector::Cartesian > &vector) const
  {
    const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
    typedef tVector < Tdimension - 1, decltype((TElement() * TVectorElement()) + (TElement() * TVectorElement())), vector::Cartesian > tResult;
    typename tResult::tElement data[Tdimension - 1];
    std::memset(data, 0, sizeof(data));
    for (size_t row = 0; row < Tdimension - 1; ++row)
    {
      for (size_t column = 0; column < Tdimension - 1; ++column)
      {
        data[row] += (*that)[row][column] * vector[column];
      }
      data[row] += (*that)[row][Tdimension - 1];
    }
    return tResult(data);
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline HomogeneousMultiplication() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  HomogeneousMultiplication(const HomogeneousMultiplication &);
  HomogeneousMultiplication &operator = (const HomogeneousMultiplication &);

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
