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
/*!\file    rrlib/math/vector/data/Polar.hpp
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

#ifndef __rrlib__math__vector__data__Polar_hpp__
#define __rrlib__math__vector__data__Polar_hpp__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tAngle.h"

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
// Polar Length
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
TElement &Polar<Tdimension, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Length() const
{
  return this->length;
}

template <size_t Tdimension, typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
TElement &Polar<Tdimension, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Length()
{
  return this->length;
}

//----------------------------------------------------------------------
// Polar 2 Alpha
//----------------------------------------------------------------------
template <typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
tAngle<TElement, TOtherUnitPolicy, TOtherSignPolicy> Polar<2, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Alpha() const
{
  return this->alpha;
}

template <typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
tAngle<TElement, TOtherUnitPolicy, TOtherSignPolicy> &Polar<2, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Alpha()
{
  return this->alpha;
}

//----------------------------------------------------------------------
// Polar 2 Length
//----------------------------------------------------------------------
template <typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
const TElement &Polar<2, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Length() const
{
  return this->length;
}

template <typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
TElement &Polar<2, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Length()
{
  return this->length;
}

//----------------------------------------------------------------------
// Polar 3 Alpha
//----------------------------------------------------------------------
template <typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
tAngle<TElement, TOtherUnitPolicy, TOtherSignPolicy> Polar<3, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Alpha() const
{
  return this->alpha;
}

template <typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
tAngle<TElement, TOtherUnitPolicy, TOtherSignPolicy> &Polar<3, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Alpha()
{
  return this->alpha;
}

//----------------------------------------------------------------------
// Polar 3 Beta
//----------------------------------------------------------------------
template <typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
tAngle<TElement, TOtherUnitPolicy, TOtherSignPolicy> Polar<3, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Beta() const
{
  return this->beta;
}

template <typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
tAngle<TElement, TOtherUnitPolicy, TOtherSignPolicy> &Polar<3, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Beta()
{
  return this->beta;
}

//----------------------------------------------------------------------
// Polar 3 Length
//----------------------------------------------------------------------
template <typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
const TElement &Polar<3, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Length() const
{
  return this->length;
}

template <typename TElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
TElement &Polar<3, TElement, TOtherUnitPolicy, TOtherSignPolicy>::Length()
{
  return this->length;
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
