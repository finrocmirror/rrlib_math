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
/*!\file    rrlib/math/angle/functions.h
 *
 * \author  Michael Arndt
 *
 * \date    2013-06-17
 *
 * \brief
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

#ifndef __rrlib__math__angle__functions_h__
#define __rrlib__math__angle__functions_h__

//----------------------------------------------------------------------
// External includes with <>
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

/*! Analogous function to std::asin but returns a proper tAngle-value directly
 *
 */
template <typename T>
tAngle<T, angle::Radian, angle::NoWrap> ASin(T sine);

/*! Analogous function to std::acos but returns a proper tAngle-value directly
 *
 */
template <typename T>
tAngle<T, angle::Radian, angle::NoWrap> ACos(T cosine);

/*! Analogous function to std::acos but returns a proper tAngle-value directly
 *
 */
template <typename T>
tAngle<T, angle::Radian, angle::NoWrap> ATan(T tangent);

/*! Analogous function to std::atan2 but returns a proper tAngle-value directly
 *
 */
template <typename TY, typename TX>
tAngle < decltype(TY() + TX()), angle::Radian, angle::NoWrap > ATan2(TY y, TX x);

/*!
 * \brief Test if an angle is inbetween two other angles (in the mathematically positive direction)
 *
 * \param test    The angle to test for
 * \param first   First angle
 * \param second  Second angle
 */
template <template <typename, typename, typename> class TAngle, typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
bool IsAngleInbetween(const TAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &test, const TAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &first, const TAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &second);

/*!
 * \brief Get the size of the angle between two other angles (in the mathematically positive direction)
 *
 * \param first   First angle
 * \param second  Second angle
 * \return the difference (in the same unit-policy, but always with an unsigned sign policy)
 */
template <template <typename, typename, typename> class TAngle, typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
TAngle<TElement, TUnitPolicy, angle::Unsigned> GetAngleInbetween(const TAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &first, const TAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &second);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/math/angle/functions.hpp"

#endif
