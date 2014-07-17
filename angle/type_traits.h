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
/*!\file    rrlib/math/angle/type_traits.h
 *
 * \author  Tobias Föhst
 *
 * \date    2014-07-17
 *
 * \brief
 *
 * \b
 *
 * A few words for type_traits.h
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__angle__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tAngle.h" instead.
#endif

#ifndef __rrlib__math__angle__type_traits_h__
#define __rrlib__math__angle__type_traits_h__

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
namespace angle
{

template <typename TLeft, typename TRight>
struct AutoWrapPolicy;

template <>
struct AutoWrapPolicy<angle::Signed, angle::Signed>
{
  typedef angle::Signed tType;
};

template <>
struct AutoWrapPolicy<angle::Unsigned, angle::Unsigned>
{
  typedef angle::Unsigned tType;
};

template <>
struct AutoWrapPolicy<angle::NoWrap, angle::NoWrap>
{
  typedef angle::NoWrap tType;
};

template <typename TOther>
struct AutoWrapPolicy<angle::Signed, TOther>
{
  typedef angle::NoWrap tType;
};

template <typename TOther>
struct AutoWrapPolicy<angle::Unsigned, TOther>
{
  typedef angle::NoWrap tType;
};

template <typename TOther>
struct AutoWrapPolicy<angle::NoWrap, TOther>
{
  typedef angle::NoWrap tType;
};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
