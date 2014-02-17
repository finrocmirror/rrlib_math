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
/*!\file    rrlib/math/test/unit_test_poses.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2014-02-14
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
#include "rrlib/math/tPose2D.h"
#include "rrlib/math/tPose3D.h"

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
class tTestPoses : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(tTestPoses);
  RRLIB_UNIT_TESTS_ADD_TEST(TestAngles);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  virtual void InitializeTests() {}
  virtual void CleanUp() {}

  virtual void TestAngles()
  {
    tPose2D pose2d;
    pose2d.SetOrientation(tMat2x2d(1, 0, 0, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle must be zero for identity matrix", tAngleRad(), pose2d.Yaw());
    pose2d.SetOrientation(tMat2x2d(0, -1, 1, 0));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle must be PI/2", tAngleRad(M_PI / 2), pose2d.Yaw());
    pose2d.SetOrientation(tMat2x2d(-1, 0, 0, -1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle must be PI", tAngleRad(M_PI), pose2d.Yaw());
    pose2d.SetOrientation(tMat2x2d(0, 1, -1, 0));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle must be -PI/2", tAngleRad(-(M_PI / 2)), pose2d.Yaw());

    tPose3D pose3d;
    pose3d.SetOrientation(tMat3x3d(1, 0, 0, 0, 1, 0, 0, 0, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angles must be zero for identity matrix", tAngleRad(), pose3d.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angles must be zero for identity matrix", tAngleRad(), pose3d.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angles must be zero for identity matrix", tAngleRad(), pose3d.Yaw());
  }

};

RRLIB_UNIT_TESTS_REGISTER_SUITE(tTestPoses);

