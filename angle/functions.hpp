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
/*!\file    rrlib/math/angle/functions.hpp
 *
 * \author  Michael Arndt
 *
 * \date    2013-06-17
 *
 * \b
 *
 * A few words for functions.h
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__angle__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tAngle.h" instead.
#endif

#ifndef __rrlib__math__angle__functions_hpp__
#define __rrlib__math__angle__functions_hpp__

//----------------------------------------------------------------------
// External includes with <>
//----------------------------------------------------------------------
#include <cmath>
#include <iostream>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/utilities.h"

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
// Implementation
//----------------------------------------------------------------------
template <template <typename, typename, typename> class TAngle, typename TElement, typename TUnitPolicy, typename TSignPolicy>
bool IsAngleInbetween(const TAngle<TElement, TUnitPolicy, TSignPolicy> &test, const TAngle<TElement, TUnitPolicy, TSignPolicy> &first, const TAngle<TElement, TUnitPolicy, TSignPolicy> &second)
{
  /* important: this algorithm operates on signed types! */
  TAngle<TElement, TUnitPolicy, angle::Signed> test_(test);
  TAngle<TElement, TUnitPolicy, angle::Signed> first_(first);
  TAngle<TElement, TUnitPolicy, angle::Signed> second_(second);

  // if they are not in "natural order", things are slightly different
  if (first_ > second_)
  {
    return test_ >= first_ || test_ < second_;
  }

  // standard case
  return test_ >= first_ && test_ < second_;
}

template <template <typename, typename, typename> class TAngle, typename TElement, typename TUnitPolicy, typename TSignPolicy>
TAngle<TElement, TUnitPolicy, angle::Unsigned> GetAngleInbetween(const TAngle<TElement, TUnitPolicy, TSignPolicy> &first, const TAngle<TElement, TUnitPolicy, TSignPolicy> &second)
{
  /* important: this function operates on UNsigned types! */
  return TAngle<TElement, TUnitPolicy, angle::Unsigned>((double) tAngle<TElement, TUnitPolicy, angle::Unsigned>(second) - (double) tAngle<TElement, TUnitPolicy, angle::Unsigned>(first));
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
