//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    test_angles.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-06
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstdlib>
#include <iostream>
#include <cmath>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tAngle.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::math;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

int main(int argc, char **argv)
{

  std::cout << tAngleRad() << std::endl;
  std::cout << tAngleRadUnsigned() << std::endl;
  std::cout << tAngleDeg() << std::endl;
  std::cout << tAngleDegUnsigned() << std::endl;

  std::cout << tAngleRad(tAngleDeg(90)) << std::endl;
  std::cout << tAngleRadUnsigned(tAngleDeg(90)) << std::endl;
  std::cout << tAngleDeg(90) << std::endl;
  std::cout << tAngleDegUnsigned(90) << std::endl;

  std::cout << tAngleRad(tAngleDeg(-90)) << std::endl;
  std::cout << tAngleRadUnsigned(tAngleDeg(-90)) << std::endl;
  std::cout << tAngleDeg(-90) << std::endl;
  std::cout << tAngleDegUnsigned(-90) << std::endl;

  tAngleDeg a = 180;
  std::cout << a << std::endl;
  std::cout << a + 180 << std::endl;
  a += 90.0;
  std::cout << a << std::endl;
  a += 90.0;
  std::cout << a << std::endl;
  a += 90;
  std::cout << a << std::endl;
  a += 90;
  std::cout << a << std::endl;

  tAngleDegUnsigned b = 180;
  std::cout << b << std::endl;
  std::cout << (b + 180) << std::endl;
  b += 90.0;
  std::cout << b << std::endl;
  b += 90.0;
  std::cout << b << std::endl;
  b += 90;
  std::cout << b << std::endl;
  b += 90;
  std::cout << b << std::endl;


  tAngleRad assignment_target(0);
  assignment_target = tAngleDeg(90);
  std::cout << assignment_target << std::endl;


  std::cout << "OK" << std::endl;

  return EXIT_SUCCESS;
}
