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
/*!\file    tAngle.h
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
#ifndef _rrlib_math_tAngle_h_
#define _rrlib_math_tAngle_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

#define _rrlib_math_angle_include_guard_

#include "rrlib/math/angle/policies/unit/Radian.h"
#include "rrlib/math/angle/policies/unit/Degree.h"

#include "rrlib/math/angle/policies/signedness/Signed.h"
#include "rrlib/math/angle/policies/signedness/Unsigned.h"

#include "rrlib/math/angle/tAngle.h"

#undef _rrlib_math_angle_include_guard_

//----------------------------------------------------------------------
// Implementation
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

typedef tAngle<double, angle::Radian, angle::Signed> tAngleRadSigned;
typedef tAngle<double, angle::Degree, angle::Signed> tAngleDegSigned;

typedef tAngle<double, angle::Radian, angle::Unsigned> tAngleRadUnsigned;
typedef tAngle<double, angle::Degree, angle::Unsigned> tAngleDegUnsigned;

typedef tAngleRadSigned tAngleRad;
typedef tAngleDegSigned tAngleDeg;

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tAngle<double, angle::Radian, angle::Signed>;
extern template class tAngle<double, angle::Degree, angle::Signed>;

extern template class tAngle<double, angle::Radian, angle::Unsigned>;
extern template class tAngle<double, angle::Degree, angle::Unsigned>;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
