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
/*!\file    rrlib/math/matrix/tMatrix.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-08-25
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__matrix__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef _rrlib_math_matrix_tMatrix_hpp_
#define _rrlib_math_matrix_tMatrix_hpp_

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
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tMatrix constructors
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
tMatrix<Trows, Tcolumns, TElement>::tMatrix()
{}

template <size_t Trows, size_t Tcolumns, typename TElement>
tMatrix<Trows, Tcolumns, TElement>::tMatrix(const tMatrix &other)
  : FunctionalityShared(other)
{}

template <size_t Trows, size_t Tcolumns, typename TElement>
tMatrix<Trows, Tcolumns, TElement>::tMatrix(const TElement data[Trows * Tcolumns])
  : FunctionalityShared(data)
{}

template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename TOtherElement>
tMatrix<Trows, Tcolumns, TElement>::tMatrix(const tMatrix<Trows, Tcolumns, TOtherElement> &other)
  : FunctionalityShared(other)
{}

template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename TLeftElement, typename TRightElement>
tMatrix<Trows, Tcolumns, TElement>::tMatrix(const tVector<Trows, TLeftElement, vector::Cartesian> &left, const tVector<Tcolumns, TRightElement, vector::Cartesian> &right)
  : FunctionalityShared(left, right)
{}

template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename ... TValues>
tMatrix<Trows, Tcolumns, TElement>::tMatrix(TElement value, TValues... values)
{
  FunctionalityShared::Set(value, values...);
}

#ifdef _LIB_OIV_PRESENT_

template <size_t Trows, size_t Tcolumns, typename TElement>
template <class T>
tMatrix<Trows, Tcolumns, TElement>::tMatrix(const SbMatrix &m, typename boost::enable_if_c < (Trows == 4 && Tcolumns == 4), T >::type)
{
  FunctionalityShared::Set(m[0][0], m[1][0], m[2][0], m[3][0],
                           m[0][1], m[1][1], m[2][1], m[3][1],
                           m[0][2], m[1][2], m[2][2], m[3][2],
                           m[0][3], m[1][3], m[2][3], m[3][3]);
}

#endif

//----------------------------------------------------------------------
// tMatrix operator =
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
tMatrix<Trows, Tcolumns, TElement> &tMatrix<Trows, Tcolumns, TElement>::operator = (const tMatrix &other)
{
  return reinterpret_cast<tMatrix &>(FunctionalityShared::operator=(other));
}

template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename TOtherElement>
tMatrix<Trows, Tcolumns, TElement> &tMatrix<Trows, Tcolumns, TElement>::operator = (const tMatrix<Trows, Tcolumns, TOtherElement> &other)
{
  return reinterpret_cast<tMatrix &>(FunctionalityShared::operator=(other));
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
