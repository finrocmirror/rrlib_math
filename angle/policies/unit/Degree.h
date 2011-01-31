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
/*!\file    Degree.h
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
#ifndef __rrlib__math__angle__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tAngle.h" instead.
#endif

#ifndef __rrlib__math__angle__policies__unit__Degree_h__
#define __rrlib__math__angle__policies__unit__Degree_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <string>

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
struct Radian;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
struct Degree
{

  static inline const double RangeLimit()
  {
    return 360.0;
  }

  static inline const double UnitDivider()
  {
    return 1.0;
  }

  static inline const std::string &UnitString()
  {
    static std::string unit_string("°");
    return unit_string;
  }

  static inline const bool PadUnitString()
  {
    return false;
  }

  static inline const double ConvertFromUnit(double value, const Radian &)
  {
    return value * 180.0 / M_PI;
  }

  static inline const double ConvertFromUnit(double value, const Degree &)
  {
    return value;
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
