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
/*!\file    rrlib/math/matrix/tMatrix.h
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

#ifndef _rrlib_math_matrix_tMatrix_h_
#define _rrlib_math_matrix_tMatrix_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"

#ifdef _LIB_OIV_PRESENT_
#include <Inventor/SbMatrix.h>
#include <type_traits>
#endif
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
template <size_t Trows, size_t Tcolumns, typename TElement = double>
class tMatrix : public matrix::Full<Trows, Tcolumns, TElement>,
  public matrix::FunctionalityShared<Trows, Tcolumns, TElement>,
  public matrix::SquareMatrixOperationsShared<Trows, Tcolumns, TElement>,
  public matrix::SquareMatrixOperationsSpecialized<Trows, Tcolumns, TElement>,
  public matrix::HomogeneousMultiplication<Trows, Tcolumns, TElement>,
  public matrix::Rotation<Trows, Tcolumns, TElement>,
  public matrix::LegacyShared<Trows, Tcolumns, TElement>,
  public matrix::LegacySpecialized<Trows, Tcolumns, TElement>,
  public matrix::ConstantValuesShared<Trows, Tcolumns, TElement>,
  public matrix::ConstantValuesSpecialized<Trows, Tcolumns, TElement>,
  public matrix::Conversions<Trows, Tcolumns, TElement>
{
  typedef matrix::FunctionalityShared<Trows, Tcolumns, TElement> FunctionalityShared;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  inline tMatrix() __attribute__((always_inline));

  inline tMatrix(const tMatrix &other) __attribute__((always_inline));

  explicit inline tMatrix(const TElement data[Trows * Tcolumns]) __attribute__((always_inline));

  template <typename TOtherElement>
  explicit inline tMatrix(const tMatrix<Trows, Tcolumns, TOtherElement> &other) __attribute__((always_inline));

  template <typename TLeftElement, typename TRightElement>
  inline tMatrix(const tVector<Trows, TLeftElement, vector::Cartesian> &left, const tVector<Tcolumns, TRightElement, vector::Cartesian> &right) __attribute__((always_inline));

  template <typename ... TValues>
  explicit inline tMatrix(TElement value, TValues... values) __attribute__((always_inline));

#ifdef _LIB_OIV_PRESENT_

  template <class T = int>
  explicit inline tMatrix(const SbMatrix &m, typename std::enable_if < (Trows == 4 && Tcolumns == 4), T >::type = 0) __attribute__((always_inline));

#endif

  inline tMatrix &operator = (const tMatrix &other) __attribute__((always_inline));

  template <typename TOtherElement>
  inline tMatrix &operator = (const tMatrix<Trows, Tcolumns, TOtherElement> &other) __attribute__((always_inline));

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/math/matrix/tMatrix.hpp"

#endif
