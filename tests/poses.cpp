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
#include "rrlib/util/tUnitTestSuite.h"

#include "rrlib/math/tPose2D.h"
#include "rrlib/math/tPose3D.h"

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
class TestPoses : public util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(TestPoses);
  RRLIB_UNIT_TESTS_ADD_TEST(TestAngles);
  RRLIB_UNIT_TESTS_ADD_TEST(Constructors2D);
  RRLIB_UNIT_TESTS_ADD_TEST(Constructors3D);
  RRLIB_UNIT_TESTS_ADD_TEST(AccessOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ComparisonOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(AssignmentOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ArithmeticOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(Streaming);
  RRLIB_UNIT_TESTS_ADD_TEST(Conversion);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  void TestAngles()
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

  void Constructors2D()
  {
    RRLIB_UNIT_TESTS_EQUALITY(3 * sizeof(double), sizeof(tPose2D));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(2), tPose2D::cDIMENSION);

    double raw[3] = { 0.0, 0.0, 0.0 };

    tPose2D zero;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero, &raw, sizeof(raw)) == 0);

    raw[0] = 2.0, raw[1] = 3.0;
    raw[2] = -M_PI_2;
    tPose2D pose1(tVec2d(2.0, 3.0), tAngleDeg(-90));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&pose1, &raw, sizeof(raw)) == 0);

    raw[0] = 1.0, raw[1] = 2.0;
    raw[2] = M_PI_2;
    tPose2D pose2(1.0, 2.0, tAngleDeg(90));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&pose2, &raw, sizeof(raw)) == 0);

    raw[0] = 2.0, raw[1] = 3.0;
    raw[2] = -M_PI_2;
    tPose2D pose3(tMatrix<3, 3, double>(0, 1, 2, -1, 0, 3, 0, 0, 1));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&pose3, &raw, sizeof(raw)) == 0);

    tPose2D copy(pose2);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy, &pose2, sizeof(pose2)) == 0);
  }

  void Constructors3D()
  {
    RRLIB_UNIT_TESTS_EQUALITY(6 * sizeof(double), sizeof(tPose3D));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(3), tPose3D::cDIMENSION);

    double raw[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    tPose3D zero;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero, &raw, sizeof(raw)) == 0);

    raw[0] = 3.0, raw[1] = 4.0;
    raw[2] = 5.0, raw[3] = M_PI_2, raw[4] = -M_PI_2;
    raw[5] = 0.0;
    tPose3D pose2(3.0, 4.0, 5.0, tAngleDeg(90), tAngleDeg(-90), tAngleDeg(0));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&pose2, &raw, sizeof(raw)) == 0);

    raw[0] = 2.0, raw[1] = 3.0;
    raw[2] = 4.0;
    raw[3] = 0.0, raw[4] = 0.0;
    raw[5] = -M_PI_2;
    tPose3D pose3(math::tMatrix<4, 4, double>(0, 1, 0, 2, -1, 0, 0, 3, 0, 0, 1, 4, 0, 0, 0, 1));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&pose3, &raw, sizeof(raw)) == 0);

    tPose3D copy(pose2);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy, &pose2, sizeof(pose2)) == 0);
  }

  void AccessOperators()
  {
    tPose2D pose_2d(1, 2, tAngleDeg(90));
    RRLIB_UNIT_TESTS_EQUALITY(1.0, pose_2d.X());
    RRLIB_UNIT_TESTS_EQUALITY(2.0, pose_2d.Y());
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, pose_2d.Yaw().Value());
    pose_2d.X() = 3;
    pose_2d.Y() = 4;
    pose_2d.Yaw() = tAngleDeg(-90);
    RRLIB_UNIT_TESTS_EQUALITY(3.0, pose_2d.X());
    RRLIB_UNIT_TESTS_EQUALITY(4.0, pose_2d.Y());
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, pose_2d.Yaw().Value());

    tPose3D pose_3d(1, 2, 3, tAngleDeg(90), tAngleDeg(-90), tAngleDeg(0.0));
    RRLIB_UNIT_TESTS_EQUALITY(1.0, pose_3d.X());
    RRLIB_UNIT_TESTS_EQUALITY(2.0, pose_3d.Y());
    RRLIB_UNIT_TESTS_EQUALITY(3.0, pose_3d.Z());
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, pose_3d.Roll().Value());
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, pose_3d.Pitch().Value());
    RRLIB_UNIT_TESTS_EQUALITY(0.0, pose_3d.Yaw().Value());
    pose_3d.X() = 10;
    pose_3d.Y() = 20;
    pose_3d.Z() = 30;
    pose_3d.Roll() = tAngleDeg(0);
    pose_3d.Pitch() = tAngleDeg(90);
    pose_3d.Yaw() = -tAngleDeg(90);
    RRLIB_UNIT_TESTS_EQUALITY(10.0, pose_3d.X());
    RRLIB_UNIT_TESTS_EQUALITY(20.0, pose_3d.Y());
    RRLIB_UNIT_TESTS_EQUALITY(30.0, pose_3d.Z());
    RRLIB_UNIT_TESTS_EQUALITY(0.0, pose_3d.Roll().Value());
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, pose_3d.Pitch().Value());
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, pose_3d.Yaw().Value());
  }

  void ComparisonOperators()
  {
    typedef localization::tPose2D<float> tPose2D;
    typedef tPose2D::tOrientationComponent<> tAngle2D;
    RRLIB_UNIT_TESTS_ASSERT(tPose2D(1, 2, tAngle2D(3)) == tPose2D(1, 2, tAngle2D(3)));
    RRLIB_UNIT_TESTS_ASSERT(tPose2D(1, 2, tAngle2D(3)) != tPose2D(2, 3, tAngle2D(4)));

    typedef localization::tPose3D<double> tPose3D;
    typedef tPose3D::tOrientationComponent<> tAngle3D;
    RRLIB_UNIT_TESTS_ASSERT(tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)) == tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)));
    RRLIB_UNIT_TESTS_ASSERT(tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)) != tPose3D(2, 3, 4, tAngle3D(5), tAngle3D(6), tAngle3D(7)));
  }

  void AssignmentOperators()
  {
    tPose2D pose_2d;
    pose_2d = tPose2D(1, 2, tAngleDeg(2));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(1, 2, tAngleDeg(2)), pose_2d);
    pose_2d.Set(3, 4, tAngleDeg(5));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(3, 4, tAngleDeg(5)), pose_2d);

    tPose3D pose_3d;
    pose_3d = tPose3D(1, 2, 3, tAngleDeg(4), tAngleDeg(5), tAngleDeg(6));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(1, 2, 3, tAngleDeg(4), tAngleDeg(5), tAngleDeg(6)), pose_3d);
    pose_3d.Set(3, 4, 5, tAngleDeg(6), tAngleDeg(7), tAngleDeg(8));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(3, 4, 5, tAngleDeg(6), tAngleDeg(7), tAngleDeg(8)), pose_3d);
  }

  void ArithmeticOperators()
  {
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(-1, -2, -tAngleRad(3)), -tPose2D(1, 2, tAngleRad(3)));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(1 + 2, 2 + 3, tAngleRad(3) + tAngleRad(4)), tPose2D(1, 2, tAngleRad(3)) + tPose2D(2, 3, tAngleRad(4)));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(1 - 2, 2 - 3, tAngleRad(3) - tAngleRad(4)), tPose2D(1, 2, tAngleRad(3)) - tPose2D(2, 3, tAngleRad(4)));

    typedef localization::tPose3D<double> tPose3D;
    typedef tPose3D::tOrientationComponent<> tAngle3D;
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(-1, -2, -3, -tAngle3D(4), -tAngle3D(5), -tAngle3D(6)), -tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(1 + 2, 2 + 3, 3 + 4, tAngle3D(4 + 5), tAngle3D(5 + 6), tAngle3D(6 + 7)), tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)) + tPose3D(2, 3, 4, tAngle3D(5), tAngle3D(6), tAngle3D(7)));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(1 - 2, 2 - 3, 3 - 4, tAngle3D(4 - 5), tAngle3D(5 - 6), tAngle3D(6 - 7)), tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)) - tPose3D(2, 3, 4, tAngle3D(5), tAngle3D(6), tAngle3D(7)));
  }

  void Streaming()
  {
    std::stringstream actual;
    std::stringstream expected;

    actual << tPose2D(1, 2, tAngleDeg(90));
    expected << "(1, 2, 90°)";
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    actual << tPose3D(1, 2, 3, tAngleDeg(4), tAngleDeg(5), tAngleDeg(6));
    expected << "(1, 2, 3, 4°, 5°, 6°)";
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    tPose2D pose_2d;
    tPose3D pose_3d;
    std::stringstream source;
    source.exceptions(std::istream::failbit);
    RRLIB_UNIT_TESTS_EXCEPTION(source >> pose_2d, std::ios_base::failure);
    source.clear();
    RRLIB_UNIT_TESTS_EXCEPTION(source >> pose_3d, std::ios_base::failure);
    source.clear();
    source << "(3, 4, 120°)";
    source >> pose_2d;
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(3, 4, tAngleDeg(120)), pose_2d);
    source.clear();
    source << "(120)";
    RRLIB_UNIT_TESTS_EXCEPTION(source >> pose_2d, std::ios_base::failure);
    source.clear();
    source.str("");
    source << "(4, 5, 6, 10°, 20°, 30°)";
    source >> pose_3d;
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(4, 5, 6, tAngleDeg(10), tAngleDeg(20), tAngleDeg(30)), pose_3d);
    source.clear();
    source << "(120)";
    RRLIB_UNIT_TESTS_EXCEPTION(source >> pose_3d, std::ios_base::failure);

    source.clear();
    source.str("");
    source << "(1, 2, 3°), (2, 3, 4°), ende";
    for (size_t i = 0; i < 2; ++i)
    {
      source >> pose_2d;
      tAngleDeg yaw(i + 3.0);
      RRLIB_UNIT_TESTS_EQUALITY(tPose2D(i + 1.0, i + 2.0, yaw), pose_2d);
      char delim;
      source >> delim;
    }

    source.clear();
    source.str("");
    source << "(1, 2, 3, 1°, 2°, 3°), (2, 3, 4, 2°, 3°, 4°), ende";
    for (size_t i = 0; i < 2; ++i)
    {
      source >> pose_3d;
      tAngleDeg roll(i + 1.0);
      tAngleDeg pitch(i + 2.0);
      tAngleDeg yaw(i + 3.0);
      RRLIB_UNIT_TESTS_EQUALITY(tPose3D(i + 1.0, i + 2.0, i + 3.0, roll, pitch, yaw), pose_3d);
      char delim;
      source >> delim;
    }

    serialization::tMemoryBuffer memory_buffer;
    serialization::tOutputStream output_stream(memory_buffer);
    serialization::tInputStream input_stream(memory_buffer);

    output_stream << tPose2D(10, 20, tAngleRad(5 * M_PI_2));
    output_stream << tPose3D(10, 20, 30, tAngleRad(1), tAngleRad(2), tAngleRad(3));
    output_stream.Flush();

    input_stream >> pose_2d;
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(10, 20, tAngleRad(M_PI_2)), pose_2d);
    input_stream >> pose_3d;
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(10, 20, 30, tAngleRad(1), tAngleRad(2), tAngleRad(3)), pose_3d);
  }

  void Conversion()
  {
    tPose2D pose_2d(tPose3D(1, 2, 3, tAngleDeg(4), tAngleDeg(5), tAngleDeg(6)));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(1, 2, tAngleDeg(6)), pose_2d);
    tPose3D pose_3d(tPose2D(1, 2, tAngleDeg(3)));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(1, 2, 0, tAngleDeg(0), tAngleDeg(0), tAngleDeg(3)), pose_3d);
  }

};

RRLIB_UNIT_TESTS_REGISTER_SUITE(TestPoses);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
