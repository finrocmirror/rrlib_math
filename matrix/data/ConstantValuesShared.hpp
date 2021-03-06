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
/*!\file    rrlib/math/matrix/data/ConstantValuesShared.hpp
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

#ifndef __rrlib__math__matrix__data__ConstantValuesShared_hpp__
#define __rrlib__math__matrix__data__ConstantValuesShared_hpp__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

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
// Const values
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
const size_t ConstantValuesShared<Trows, Tcolumns, TElement>::cROWS = Trows;

template <size_t Trows, size_t Tcolumns, typename TElement>
const size_t ConstantValuesShared<Trows, Tcolumns, TElement>::cCOLUMNS = Tcolumns;

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// ConstantValuesShared Zero
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
const tMatrix<Trows, Tcolumns, TElement> &ConstantValuesShared<Trows, Tcolumns, TElement>::Zero()
{
  static tMatrix matrix;
  return matrix;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
