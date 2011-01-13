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
#ifndef _rrlib_math_matrix_include_guard_
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

  inline tMatrix() {}
  inline tMatrix(const tMatrix &other) : FunctionalityShared(other) {}

  explicit inline tMatrix(const TElement data[Trows * Tcolumns]) : FunctionalityShared(data) {}

  template <typename TOtherElement>
  explicit inline tMatrix(const tMatrix<Trows, Tcolumns, TOtherElement, TData> &other) : FunctionalityShared(other) {}

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  explicit inline tMatrix(const tMatrix<Trows, Tcolumns, TOtherElement, TOtherData> &other) : FunctionalityShared(other) {}

  template <typename TLeftElement, typename TRightElement>
  inline tMatrix(const tVector<Trows, TLeftElement, vector::Cartesian> &left, const tVector<Tcolumns, TRightElement, vector::Cartesian> &right) : FunctionalityShared(left, right) {}

  template <typename ... TValues>
  explicit inline tMatrix(TElement value, TValues... values)
  {
    FunctionalityShared::Set(value, values...);
  }

#ifdef _LIB_OIV_PRESENT_
  template <class T>
  explicit inline tMatrix(
    const T& m,
    typename boost::enable_if_c < (boost::is_same<T, SbMatrix>::value && Trows == 4 && Tcolumns == 4), void >::type* = 0)
  {
    FunctionalityShared::Set(
      m[0][0], m[1][0], m[2][0], m[3][0],
      m[0][1], m[1][1], m[2][1], m[3][1],
      m[0][2], m[1][2], m[2][2], m[3][2],
      m[0][3], m[1][3], m[2][3], m[3][3]
    );
  }
#endif


  inline tMatrix &operator = (const tMatrix &other)
  {
    return reinterpret_cast<tMatrix &>(FunctionalityShared::operator=(other));
  }

  template <typename TOtherElement>
  inline tMatrix &operator = (const tMatrix<Trows, Tcolumns, TOtherElement, TData> &other)
  {
    return reinterpret_cast<tMatrix &>(FunctionalityShared::operator=(other));
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
