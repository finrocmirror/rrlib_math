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
// Polar 2 Alpha
//----------------------------------------------------------------------
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &Polar<2, TElement, TUnitPolicy, TAutoWrapPolicy>::Alpha() const
{
  return this->alpha;
}

template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &Polar<2, TElement, TUnitPolicy, TAutoWrapPolicy>::Alpha()
{
  return this->alpha;
}

//----------------------------------------------------------------------
// Polar 2 Length
//----------------------------------------------------------------------
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
const TElement &Polar<2, TElement, TUnitPolicy, TAutoWrapPolicy>::Length() const
{
  return this->length;
}

template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
TElement &Polar<2, TElement, TUnitPolicy, TAutoWrapPolicy>::Length()
{
  return this->length;
}

//----------------------------------------------------------------------
// Polar 2 Set
//----------------------------------------------------------------------
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
template <typename TAlpha, typename TLength>
void Polar<2, TElement, TUnitPolicy, TAutoWrapPolicy>::Set(TAlpha alpha, TLength length)
{
  this->alpha = tAngle(alpha);
  this->length = TElement(length);
}

//----------------------------------------------------------------------
// Polar 3 Alpha
//----------------------------------------------------------------------
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &Polar<3, TElement, TUnitPolicy, TAutoWrapPolicy>::Alpha() const
{
  return this->alpha;
}

template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &Polar<3, TElement, TUnitPolicy, TAutoWrapPolicy>::Alpha()
{
  return this->alpha;
}

//----------------------------------------------------------------------
// Polar 3 Beta
//----------------------------------------------------------------------
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &Polar<3, TElement, TUnitPolicy, TAutoWrapPolicy>::Beta() const
{
  return this->beta;
}

template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &Polar<3, TElement, TUnitPolicy, TAutoWrapPolicy>::Beta()
{
  return this->beta;
}

//----------------------------------------------------------------------
// Polar 3 Length
//----------------------------------------------------------------------
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
const TElement &Polar<3, TElement, TUnitPolicy, TAutoWrapPolicy>::Length() const
{
  return this->length;
}

template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
TElement &Polar<3, TElement, TUnitPolicy, TAutoWrapPolicy>::Length()
{
  return this->length;
}

//----------------------------------------------------------------------
// Polar 3 Set
//----------------------------------------------------------------------
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
template <typename TAlpha, typename TBeta, typename TLength>
void Polar<3, TElement, TUnitPolicy, TAutoWrapPolicy>::Set(TAlpha alpha, TBeta beta, TLength length)
{
  this->alpha = tAngle(alpha);
  this->beta = tAngle(beta);
  this->length = TElement(length);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
