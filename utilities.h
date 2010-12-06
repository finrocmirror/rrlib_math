//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
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
/*!\file    utilities.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-05
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_utilities_h_
#define _rrlib_math_utilities_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_float.hpp>
#include <boost/type_traits/is_integral.hpp>

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Function declaration
//----------------------------------------------------------------------

template <typename T>
inline const typename boost::enable_if<boost::is_integral<T>, T>::type Modulo(T a, T b)
{
  T v = a % b;
  return v < 0 ? v + b : v;
}

template <typename T>
inline const typename boost::enable_if<boost::is_float<T>, T>::type Modulo(T a, T b)
{
  T v = std::fmod(a, b);
  return v < 0 ? v + b : v;
}




//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
