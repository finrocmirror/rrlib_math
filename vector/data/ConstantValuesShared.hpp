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
/*!\file    rrlib/math/vector/data/ConstantValuesShared.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-08-25
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__vector__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef __rrlib__math__vector__data__ConstantValuesShared_hpp__
#define __rrlib__math__vector__data__ConstantValuesShared_hpp__

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
// ConstantValuesShared Zero
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
const tVector<Tdimension, TElement, TData> &ConstantValuesShared<Tdimension, TElement, TData>::Zero()
{
  static tVector vector;
  return vector;
}

//----------------------------------------------------------------------
// ConstantValuesShared Cartesian Zero
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const tVector<Tdimension, TElement, Cartesian> &ConstantValuesShared<Tdimension, TElement, Cartesian>::Zero()
{
  static tVector vector;
  return vector;
}

//----------------------------------------------------------------------
// ConstantValuesShared Cartesian Identity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const tVector<Tdimension, TElement, Cartesian> &ConstantValuesShared<Tdimension, TElement, Cartesian>::Identity()
{
  static tVector identity(InitializeIdentity());
  return identity;
}

//----------------------------------------------------------------------
// ConstantValuesShared Cartesian InitializeIdentity
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const tVector<Tdimension, TElement, Cartesian> &ConstantValuesShared<Tdimension, TElement, Cartesian>::InitializeIdentity()
{
  static tVector vector;
  for (size_t i = 0; i < Tdimension; ++i)
  {
    vector[i] = 1;
  }
  return vector;
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
