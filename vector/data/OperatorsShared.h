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
/*!\file    rrlib/math/vector/data/OperatorsShared.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-03-14
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

#ifndef __rrlib__math__vector__data__OperatorsShared_h__
#define __rrlib__math__vector__data__OperatorsShared_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_
#include "rrlib/serialization/serialization.h"
#include <sstream>
#endif

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
// Function declarations
//----------------------------------------------------------------------

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tVector<Tdimension, TElement, TData> &vector)
{
  std::stringstream string_stream;
  string_stream << vector;
  stream << string_stream.str();
  return stream;
}

template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tVector<Tdimension, TElement, TData> &vector)
{
  std::istringstream string_stream(stream.ReadLine());
  string_stream >> vector;
  return stream;
}

#endif



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
