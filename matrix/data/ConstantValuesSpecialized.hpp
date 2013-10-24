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
/*!\file    rrlib/math/matrix/data/ConstantValuesSpecialized.hpp
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

#ifndef __rrlib__math__matrix__data__ConstantValuesSpecialized_hpp__
#define __rrlib__math__matrix__data__ConstantValuesSpecialized_hpp__

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
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// ConstantValuesSpecialized Identity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, size_t, typename> class TData>
const tMatrix<Tdimension, Tdimension, TElement, TData> &ConstantValuesSpecialized<Tdimension, Tdimension, TElement, TData>::Identity()
{
  static tMatrix identity(InitializeIdentity());
  return identity;
}

//----------------------------------------------------------------------
// ConstantValuesSpecialized InitializeIdentity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, size_t, typename> class TData>
const tMatrix<Tdimension, Tdimension, TElement, TData> &ConstantValuesSpecialized<Tdimension, Tdimension, TElement, TData>::InitializeIdentity()
{
  static tMatrix matrix;
  for (size_t i = 0; i < Tdimension; ++i)
  {
    matrix[i][i] = static_cast<TElement>(1);
  }
  return matrix;
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
