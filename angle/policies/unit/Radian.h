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
/*!\file    Radian.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-06
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_angle_include_guard_
#error Invalid include directive. Try #include "rrlib/math/tAngle.h" instead.
#endif

#ifndef _rrlib_math_angle_policies_unit_Radian_h_
#define _rrlib_math_angle_policies_unit_Radian_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <string>
#include <cmath>

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
struct Degree;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
struct Radian
{
  static inline const double RangeLimit()
  {
    return 2 * M_PI;
  }

  static inline const double UnitDivider()
  {
    return M_PI;
  }

  static inline const std::string &UnitString()
  {
    static std::string unit_string("pi");
    return unit_string;
  }

  static inline const bool PadUnitString()
  {
    return true;
  }

  static inline const double ConvertFromUnit(double value, const Radian &)
  {
    return value;
  }

  static inline const double ConvertFromUnit(double value, const Degree &)
  {
    return value * M_PI / 180.0;
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
