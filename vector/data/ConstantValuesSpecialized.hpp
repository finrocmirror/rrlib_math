//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    ConstantValuesSpecialized.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-08-25
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__vector__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef __rrlib__math__vector__data__ConstantValuesSpecialized_hpp__
#define __rrlib__math__vector__data__ConstantValuesSpecialized_hpp__

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
namespace vector
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// ConstantValues 2 XDirection
//----------------------------------------------------------------------
template <typename TElement>
const tVector<2, TElement, Cartesian> &ConstantValuesSpecialized<2, TElement, Cartesian>::XDirection()
{
  static tVector vector(static_cast<TElement>(1), 0);
  return vector;
}

//----------------------------------------------------------------------
// ConstantValues 2 YDirection
//----------------------------------------------------------------------
template <typename TElement>
const tVector<2, TElement, Cartesian> &ConstantValuesSpecialized<2, TElement, Cartesian>::YDirection()
{
  static tVector vector(0, static_cast<TElement>(1));
  return vector;
}

//----------------------------------------------------------------------
// ConstantValues 3 XDirection
//----------------------------------------------------------------------
template <typename TElement>
const tVector<3, TElement, Cartesian> &ConstantValuesSpecialized<3, TElement, Cartesian>::XDirection()
{
  static tVector vector(static_cast<TElement>(1), 0, 0);
  return vector;
}

//----------------------------------------------------------------------
// ConstantValues 3 YDirection
//----------------------------------------------------------------------
template <typename TElement>
const tVector<3, TElement, Cartesian> &ConstantValuesSpecialized<3, TElement, Cartesian>::YDirection()
{
  static tVector vector(0, static_cast<TElement>(1), 0);
  return vector;
}

//----------------------------------------------------------------------
// ConstantValues 3 ZDirection
//----------------------------------------------------------------------
template <typename TElement>
const tVector<3, TElement, Cartesian> &ConstantValuesSpecialized<3, TElement, Cartesian>::ZDirection()
{
  static tVector vector(0, 0, static_cast<TElement>(1));
  return vector;
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
