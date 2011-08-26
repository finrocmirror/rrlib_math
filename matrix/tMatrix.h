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
/*!\file    tMatrix.h
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
#include <boost/utility/enable_if.hpp>
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
template < size_t Trows, size_t Tcolumns, typename TElement = double, template <size_t, size_t, typename> class TData = matrix::Full >
class tMatrix : public TData<Trows, Tcolumns, TElement>,
    public matrix::FunctionalityShared<Trows, Tcolumns, TElement, TData>,
    public matrix::FunctionalitySpecialized<Trows, Tcolumns, TElement, TData>,
    public matrix::SquareMatrixOperationsShared<Trows, Tcolumns, TElement, TData>,
    public matrix::SquareMatrixOperationsSpecialized<Trows, Tcolumns, TElement, TData>,
    public matrix::HomogeneousMultiplication<Trows, Tcolumns, TElement, TData>,
    public matrix::Rotation<Trows, Tcolumns, TElement, TData>,
    public matrix::LegacyShared<Trows, Tcolumns, TElement, TData>,
    public matrix::LegacySpecialized<Trows, Tcolumns, TElement, TData>,
    public matrix::ConstantValuesShared<Trows, Tcolumns, TElement, TData>,
    public matrix::ConstantValuesSpecialized<Trows, Tcolumns, TElement, TData>,
    public matrix::Conversions<Trows, Tcolumns, TElement, TData>
{
  typedef matrix::FunctionalityShared<Trows, Tcolumns, TElement, TData> FunctionalityShared;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  inline tMatrix() __attribute__((always_inline));

  inline tMatrix(const tMatrix &other) __attribute__((always_inline));

  explicit inline tMatrix(const TElement data[Trows * Tcolumns]) __attribute__((always_inline));

  template <typename TOtherElement>
  explicit inline tMatrix(const tMatrix<Trows, Tcolumns, TOtherElement, TData> &other) __attribute__((always_inline));

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  explicit inline tMatrix(const tMatrix<Trows, Tcolumns, TOtherElement, TOtherData> &other) __attribute__((always_inline));

  template <typename TLeftElement, typename TRightElement>
  inline tMatrix(const tVector<Trows, TLeftElement, vector::Cartesian> &left, const tVector<Tcolumns, TRightElement, vector::Cartesian> &right) __attribute__((always_inline));

  template <typename ... TValues>
  explicit inline tMatrix(TElement value, TValues... values) __attribute__((always_inline,flatten));

#ifdef _LIB_OIV_PRESENT_

  template < class T = int >
  explicit inline tMatrix(const SbMatrix &m, typename boost::enable_if_c < (Trows == 4 && Tcolumns == 4), T >::type = 0) __attribute__((always_inline,flatten));

#endif

  inline tMatrix &operator = (const tMatrix &other) __attribute__((always_inline,flatten));

  template <typename TOtherElement>
  inline tMatrix &operator = (const tMatrix<Trows, Tcolumns, TOtherElement, TData> &other) __attribute__((always_inline,flatten));

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/math/matrix/tMatrix.hpp"

#endif
