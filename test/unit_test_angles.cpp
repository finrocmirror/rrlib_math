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
/*!\file    rrlib/math/test/unit_test_angles.cpp
 *
 * \author  Michael Arndt
 *
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstdlib>
#include <iostream>

#include "rrlib/util/tUnitTestSuite.h"

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
class tTestAngles : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(tTestAngles);
  RRLIB_UNIT_TESTS_ADD_TEST(Test);
  RRLIB_UNIT_TESTS_ADD_TEST(TestFunctions);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  virtual void InitializeTests() {}
  virtual void CleanUp() {}

  virtual void Test()
  {

    tAngleRad angle;

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle must be zero after initialization", 0.0, (double) angle);

    angle = tAngleRad(M_PI / 2);

    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Angle must be correct after assignment", M_PI / 2, (double) angle, 1E-9);

    // FIXME: many more

  }

  virtual void TestFunctions()
  {

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle between 0 deg and 30 deg must be 30 deg (unsigned)", tAngleDegUnsigned(30), GetAngleInbetween(tAngleDeg(0), tAngleDeg(30)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle between 0 deg and 30 deg must be 30 deg (signed)", tAngleDegUnsigned(30), GetAngleInbetween(tAngleDegSigned(0), tAngleDegSigned(30)));
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Angle between 0 and pi must be pi", tAngleRadUnsigned(M_PI), GetAngleInbetween(tAngleRad(0), tAngleRad(M_PI)), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle between 30 deg and 0 deg must be 330 deg", tAngleDegUnsigned(330), GetAngleInbetween(tAngleDeg(30), tAngleDeg(0)));
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Angle between pi/2 and 3/2 pi must be pi", tAngleRadUnsigned(M_PI), GetAngleInbetween(tAngleRad(M_PI / 2), tAngleRad(3 / 2. * M_PI)), 1E-9);

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle between -30 deg and -40 deg must be 350 deg", tAngleDegUnsigned(350), GetAngleInbetween(tAngleDegSigned(-30), tAngleDegSigned(-40)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle between -40 deg and -30 deg must be 10 deg", tAngleDegUnsigned(10), GetAngleInbetween(tAngleDegSigned(-40), tAngleDegSigned(-30)));

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("15 deg must be between 0 deg and 30 deg", true, IsAngleInbetween(tAngleDeg(15), tAngleDeg(0), tAngleDeg(30)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("-15 deg must not be between 0 deg and 30 deg", false, IsAngleInbetween(tAngleDeg(-15), tAngleDeg(0), tAngleDeg(30)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("345 deg must not be between 0 deg and 30 deg", false, IsAngleInbetween(tAngleDeg(345), tAngleDeg(0), tAngleDeg(30)));

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("179 deg must be between 0 deg and 180 deg", true, IsAngleInbetween(tAngleDeg(179), tAngleDeg(0), tAngleDeg(180)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("181 deg must not be between 0 deg and 180 deg", false, IsAngleInbetween(tAngleDeg(181), tAngleDeg(0), tAngleDeg(180)));

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("179 deg must be between 90 deg and -90 deg", true, IsAngleInbetween(tAngleDeg(179), tAngleDeg(90), tAngleDeg(-90)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("180 deg must be between 90 deg and -90 deg", true, IsAngleInbetween(tAngleDeg(180), tAngleDeg(90), tAngleDeg(-90)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("181 deg must be between 90 deg and -90 deg", true, IsAngleInbetween(tAngleDeg(181), tAngleDeg(90), tAngleDeg(-90)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("89 deg must not be between 90 deg and -90 deg", false, IsAngleInbetween(tAngleDeg(89), tAngleDeg(90), tAngleDeg(-90)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("271 deg must not be between 90 deg and -90 deg", false, IsAngleInbetween(tAngleDeg(271), tAngleDeg(90), tAngleDeg(-90)));

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("99 deg must not be between 100 deg and 260 deg", false, IsAngleInbetween(tAngleDeg(99), tAngleDeg(100), tAngleDeg(260)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("261 deg must not be between 100 deg and 260 deg", false, IsAngleInbetween(tAngleDeg(261), tAngleDeg(100), tAngleDeg(260)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("170 deg must be between 100 deg and 260 deg", true, IsAngleInbetween(tAngleDeg(170), tAngleDeg(100), tAngleDeg(260)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("190 deg must be between 100 deg and 260 deg", true, IsAngleInbetween(tAngleDeg(190), tAngleDeg(100), tAngleDeg(260)));

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("0 deg must be between -30 deg and 30 deg", true, IsAngleInbetween(tAngleDeg(0), tAngleDeg(-30), tAngleDeg(30)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("29 deg must be between -30 deg and 30 deg", true, IsAngleInbetween(tAngleDeg(29), tAngleDeg(-30), tAngleDeg(30)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("-29 deg must be between -30 deg and 30 deg", true, IsAngleInbetween(tAngleDeg(-29), tAngleDeg(-30), tAngleDeg(30)));

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("190 deg must be between 350 deg and 340 deg", true, IsAngleInbetween(tAngleDeg(190), tAngleDeg(350), tAngleDeg(340)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("0 deg must be between 350 deg and 340 deg", true, IsAngleInbetween(tAngleDeg(0), tAngleDeg(350), tAngleDeg(340)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("180 deg must be between 350 deg and 340 deg", true, IsAngleInbetween(tAngleDeg(180), tAngleDeg(350), tAngleDeg(340)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("345 deg must not be between 350 deg and 340 deg", false, IsAngleInbetween(tAngleDeg(345), tAngleDeg(350), tAngleDeg(340)));

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("195 deg must be between 190 deg and 200 deg", true, IsAngleInbetween(tAngleDeg(195), tAngleDeg(190), tAngleDeg(200)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("10 deg must not be between 190 deg and 200 deg", false, IsAngleInbetween(tAngleDeg(10), tAngleDeg(190), tAngleDeg(200)));

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("10 deg must not be between 190 deg and 200 deg", false, IsAngleInbetween(tAngleDegUnsigned(10), tAngleDegUnsigned(190), tAngleDegUnsigned(200)));

  }



};

RRLIB_UNIT_TESTS_REGISTER_SUITE(tTestAngles);
