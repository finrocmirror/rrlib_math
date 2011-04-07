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
/*!\file    utilities.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-05
 *
 */
//----------------------------------------------------------------------
#include "rrlib/math/utilities.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <algorithm>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
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
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// IsEqual
//----------------------------------------------------------------------
namespace
{
const bool IsEqualUsingDistanceInFloatRepresentation(float a, float b, unsigned int max_distance)
{
  union tFloatToIntRepresentation
  {
    float float_representation;
    int int_representation;
  } a_converter, b_converter;

  if (std::isnan(a) || std::isnan(b) || std::isinf(a) || std::isinf(b))
  {
    return false;
  }

  a_converter.float_representation = a;
  b_converter.float_representation = b;

  int a_int = a_converter.int_representation;
  int b_int = b_converter.int_representation;

  // make lexicographically ordered as twos-complement
  if (a_int < 0)
  {
    a_int = 0x80000000 - a_int;
  }
  if (b_int < 0)
  {
    b_int = 0x80000000 - b_int;
  }

  return static_cast<unsigned int>(std::abs(a_int - b_int)) <= max_distance;
}

}

const bool IsEqual(float a, float b, float max_error, tFloatComparisonMethod method)
{
  if (a == b)
  {
    return true;
  }
  switch (method)
  {
  case eFCM_ABSOLUTE_ERROR:
    return std::abs(a - b) <= max_error;
  case eFCM_RELATIVE_ERROR:
    return std::abs((a - b) / std::max(std::abs(a), std::abs(b))) <= max_error;
  case eFCM_DISTANCE_IN_FLOAT_REPRESENTATION:
    return IsEqualUsingDistanceInFloatRepresentation(a, b, static_cast<unsigned int>(max_error));
  }
  assert(false && "Invalid float comparison method.");
  return false;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
