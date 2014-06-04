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
/*!\file    rrlib/math/test/unit_test_angles.cpp
 *
 * \author  Michael Arndt
 * \author  Tobias Föhst
 *
 * \date    2013-06-14
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/util/tUnitTestSuite.h"

#include <cstring>

#include "rrlib/math/tAngle.h"

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
class tTestAngles : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(tTestAngles);
  RRLIB_UNIT_TESTS_ADD_TEST(Constructors);
  RRLIB_UNIT_TESTS_ADD_TEST(ComparisonOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(Conversions);
  RRLIB_UNIT_TESTS_ADD_TEST(AssignmentOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ArithmeticOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(Streaming);
  RRLIB_UNIT_TESTS_ADD_TEST(TestFunctions);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  virtual void InitializeTests() {}
  virtual void CleanUp() {}

  void Constructors()
  {
    RRLIB_UNIT_TESTS_EQUALITY(sizeof(double), sizeof(tAngle<double, angle::Radian, angle::NoWrap>));
    RRLIB_UNIT_TESTS_EQUALITY(sizeof(double), sizeof(tAngle<double, angle::Radian, angle::Signed>));
    RRLIB_UNIT_TESTS_EQUALITY(sizeof(double), sizeof(tAngle<double, angle::Radian, angle::Unsigned>));
    RRLIB_UNIT_TESTS_EQUALITY(sizeof(double), sizeof(tAngle<double, angle::Degree, angle::NoWrap>));
    RRLIB_UNIT_TESTS_EQUALITY(sizeof(double), sizeof(tAngle<double, angle::Degree, angle::Signed>));
    RRLIB_UNIT_TESTS_EQUALITY(sizeof(double), sizeof(tAngle<double, angle::Degree, angle::Unsigned>));

    double raw = 0.0;

    tAngle<double, angle::Radian, angle::NoWrap> zero_rad_no_wrap;
    tAngle<double, angle::Radian, angle::Signed> zero_rad_signed;
    tAngle<double, angle::Radian, angle::Unsigned> zero_rad_unsigned;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero_rad_no_wrap, &raw, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero_rad_signed, &raw, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero_rad_unsigned, &raw, sizeof(double)) == 0);

    tAngle<double, angle::Degree, angle::NoWrap> zero_deg_no_wrap;
    tAngle<double, angle::Degree, angle::Signed> zero_deg_signed;
    tAngle<double, angle::Degree, angle::Unsigned> zero_deg_unsigned;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero_deg_no_wrap, &raw, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero_deg_signed, &raw, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero_deg_unsigned, &raw, sizeof(double)) == 0);

    tAngle<double, angle::Radian, angle::NoWrap> rad_no_wrap(5 * M_PI_2);
    tAngle<double, angle::Radian, angle::Signed> rad_signed(5 * M_PI_2);
    tAngle<double, angle::Radian, angle::Unsigned> rad_unsigned(5 * M_PI_2);
    raw = 5 * M_PI_2;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&rad_no_wrap, &raw, sizeof(double)) == 0);
    raw = M_PI_2;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&rad_signed, &raw, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&rad_unsigned, &raw, sizeof(double)) == 0);

    tAngle<double, angle::Degree, angle::NoWrap> deg_no_wrap(5 * 90.0);
    tAngle<double, angle::Degree, angle::Signed> deg_signed(5 * 90.0);
    tAngle<double, angle::Degree, angle::Unsigned> deg_unsigned(5 * 90.0);
    raw = 5 * 90.0;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&deg_no_wrap, &raw, sizeof(double)) == 0);
    raw = 90.0;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&deg_signed, &raw, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&deg_unsigned, &raw, sizeof(double)) == 0);

    tAngle<double, angle::Radian, angle::NoWrap> copy_rad_no_wrap(rad_no_wrap);
    tAngle<double, angle::Radian, angle::Signed> copy_rad_signed(rad_signed);
    tAngle<double, angle::Radian, angle::Unsigned> copy_rad_unsigned(rad_unsigned);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy_rad_no_wrap, &rad_no_wrap, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy_rad_signed, &rad_signed, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy_rad_unsigned, &rad_unsigned, sizeof(double)) == 0);

    tAngle<double, angle::Degree, angle::NoWrap> copy_deg_no_wrap(deg_no_wrap);
    tAngle<double, angle::Degree, angle::Signed> copy_deg_signed(deg_signed);
    tAngle<double, angle::Degree, angle::Unsigned> copy_deg_unsigned(deg_unsigned);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy_deg_no_wrap, &deg_no_wrap, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy_deg_signed, &deg_signed, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy_deg_unsigned, &deg_unsigned, sizeof(double)) == 0);

    tAngle<double, angle::Degree, angle::NoWrap> converted_deg_no_wrap(rad_no_wrap);
    tAngle<double, angle::Degree, angle::Signed> converted_deg_signed(rad_no_wrap);
    tAngle<double, angle::Degree, angle::Unsigned> converted_deg_unsigned(rad_no_wrap);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted_deg_no_wrap, &deg_no_wrap, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted_deg_signed, &deg_signed, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted_deg_unsigned, &deg_unsigned, sizeof(double)) == 0);

    tAngle<double, angle::Radian, angle::NoWrap> converted_rad_no_wrap(deg_no_wrap);
    tAngle<double, angle::Radian, angle::Signed> converted_rad_signed(deg_no_wrap);
    tAngle<double, angle::Radian, angle::Unsigned> converted_rad_unsigned(deg_no_wrap);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted_rad_no_wrap, &rad_no_wrap, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted_rad_signed, &rad_signed, sizeof(double)) == 0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted_rad_unsigned, &rad_unsigned, sizeof(double)) == 0);
  }

  void ComparisonOperators()
  {
    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Radian, angle::NoWrap>(0.1) == tAngle<double, angle::Radian, angle::NoWrap>(0.1)));
    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Radian, angle::Signed>(0.1) == tAngle<double, angle::Radian, angle::Signed>(0.1)));
    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Radian, angle::Unsigned>(0.1) == tAngle<double, angle::Radian, angle::Unsigned>(0.1)));
    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Degree, angle::NoWrap>(0.1) == tAngle<double, angle::Degree, angle::NoWrap>(0.1)));
    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Degree, angle::Signed>(0.1) == tAngle<double, angle::Degree, angle::Signed>(0.1)));
    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Degree, angle::Unsigned>(0.1) == tAngle<double, angle::Degree, angle::Unsigned>(0.1)));

    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Radian, angle::NoWrap>(0.1) != tAngle<double, angle::Radian, angle::NoWrap>(0.2)));
    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Radian, angle::Signed>(0.1) != tAngle<double, angle::Radian, angle::Signed>(0.2)));
    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Radian, angle::Unsigned>(0.1) != tAngle<double, angle::Radian, angle::Unsigned>(0.2)));
    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Degree, angle::NoWrap>(0.1) != tAngle<double, angle::Degree, angle::NoWrap>(0.2)));
    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Degree, angle::Signed>(0.1) != tAngle<double, angle::Degree, angle::Signed>(0.2)));
    RRLIB_UNIT_TESTS_ASSERT((tAngle<double, angle::Degree, angle::Unsigned>(0.1) != tAngle<double, angle::Degree, angle::Unsigned>(0.2)));
  }

  void Conversions()
  {
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<float, angle::Radian, angle::NoWrap>(5 * M_PI_2)), (tAngle<float, angle::Radian, angle::NoWrap>(tAngle<double, angle::Degree, angle::NoWrap>(5 * 90.0))));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<float, angle::Radian, angle::Signed>(5 * M_PI_2)), (tAngle<float, angle::Radian, angle::Signed>(tAngle<double, angle::Degree, angle::NoWrap>(5 * 90.0))));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<float, angle::Radian, angle::Unsigned>(5 * M_PI_2)), (tAngle<float, angle::Radian, angle::Unsigned>(tAngle<double, angle::Degree, angle::NoWrap>(5 * 90.0))));

    RRLIB_UNIT_TESTS_EQUALITY((tAngle<float, angle::Degree, angle::NoWrap>(5 * 90.0)), (tAngle<float, angle::Degree, angle::NoWrap>(tAngle<double, angle::Radian, angle::NoWrap>(5 * M_PI_2))));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<float, angle::Degree, angle::Signed>(5 * 90.0)), (tAngle<float, angle::Degree, angle::Signed>(tAngle<double, angle::Radian, angle::NoWrap>(5 * M_PI_2))));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<float, angle::Degree, angle::Unsigned>(5 * 90.0)), (tAngle<float, angle::Degree, angle::Unsigned>(tAngle<double, angle::Radian, angle::NoWrap>(5 * M_PI_2))));
  }

  void AssignmentOperators()
  {
    tAngleRad rad;
    rad = tAngleDeg(90);
    RRLIB_UNIT_TESTS_EQUALITY(tAngleRad(M_PI_2), rad);
  }

  void ArithmeticOperators()
  {
    tAngle<double, angle::Radian, angle::NoWrap> rad_no_wrap(2 * M_PI);
    RRLIB_UNIT_TESTS_EQUALITY(2 * M_PI, rad_no_wrap.Value());

    rad_no_wrap += tAngle<double, angle::Radian, angle::NoWrap>(M_PI);
    RRLIB_UNIT_TESTS_EQUALITY(3 * M_PI, rad_no_wrap.Value());

    rad_no_wrap -= tAngle<double, angle::Radian, angle::NoWrap>(M_PI);
    RRLIB_UNIT_TESTS_EQUALITY(2 * M_PI, rad_no_wrap.Value());

    rad_no_wrap += tAngle<float, angle::Degree, angle::NoWrap>(90);
    RRLIB_UNIT_TESTS_EQUALITY(5 * M_PI_2, rad_no_wrap.Value());

    rad_no_wrap -= tAngle<double, angle::Degree, angle::NoWrap>(90);
    RRLIB_UNIT_TESTS_EQUALITY(2 * M_PI, rad_no_wrap.Value());

    rad_no_wrap *= 2.0;
    RRLIB_UNIT_TESTS_EQUALITY(4 * M_PI, rad_no_wrap.Value());

    rad_no_wrap /= 4;
    RRLIB_UNIT_TESTS_EQUALITY(M_PI, rad_no_wrap.Value());

    tAngle<double, angle::Radian, angle::Signed> rad_signed(2 * M_PI);
    RRLIB_UNIT_TESTS_EQUALITY(0.0, rad_signed.Value());

    rad_signed += tAngle<double, angle::Radian, angle::NoWrap>(M_PI);
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI, rad_signed.Value());

    rad_signed -= tAngle<double, angle::Radian, angle::NoWrap>(M_PI);
    RRLIB_UNIT_TESTS_EQUALITY(0.0, rad_signed.Value());

    rad_signed += tAngle<float, angle::Degree, angle::NoWrap>(90);
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, rad_signed.Value());

    rad_signed -= tAngle<float, angle::Degree, angle::NoWrap>(90);
    RRLIB_UNIT_TESTS_EQUALITY(0.0, rad_signed.Value());

    rad_signed += tAngle<float, angle::Degree, angle::NoWrap>(90);
    rad_signed *= 2.0;
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI, rad_signed.Value());

    rad_signed /= 4;
    RRLIB_UNIT_TESTS_EQUALITY(-0.5 * M_PI_2, rad_signed.Value());

    tAngle<double, angle::Radian, angle::Unsigned> rad_unsigned(2 * M_PI);
    RRLIB_UNIT_TESTS_EQUALITY(0.0, rad_unsigned.Value());

    rad_unsigned += tAngle<double, angle::Radian, angle::NoWrap>(M_PI);
    RRLIB_UNIT_TESTS_EQUALITY(M_PI, rad_unsigned.Value());

    rad_unsigned -= tAngle<double, angle::Radian, angle::NoWrap>(M_PI);
    RRLIB_UNIT_TESTS_EQUALITY(0.0, rad_unsigned.Value());

    rad_unsigned += tAngle<float, angle::Degree, angle::NoWrap>(90);
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, rad_unsigned.Value());

    rad_unsigned -= tAngle<float, angle::Degree, angle::NoWrap>(90);
    RRLIB_UNIT_TESTS_EQUALITY(0.0, rad_unsigned.Value());

    rad_unsigned += tAngle<float, angle::Degree, angle::NoWrap>(90);
    rad_unsigned *= 2.0;
    RRLIB_UNIT_TESTS_EQUALITY(M_PI, rad_unsigned.Value());

    rad_unsigned /= 4;
    RRLIB_UNIT_TESTS_EQUALITY(0.5 * M_PI_2, rad_unsigned.Value());

    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::NoWrap>(-12.3)), (-tAngle<double, angle::Radian, angle::NoWrap>(12.3)));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Signed>(-12.3)), (-tAngle<double, angle::Radian, angle::Signed>(12.3)));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Unsigned>(-12.3)), (-tAngle<double, angle::Radian, angle::Unsigned>(12.3)));

    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::NoWrap>(3 * M_PI_2)), (tAngle<double, angle::Radian, angle::NoWrap>(M_PI) + tAngle<double, angle::Radian, angle::NoWrap>(M_PI_2)));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::NoWrap>(3 * M_PI_2)), (tAngle<double, angle::Radian, angle::NoWrap>(M_PI) + tAngle<float, angle::Degree, angle::NoWrap>(90)));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::NoWrap>(M_PI_2)), (tAngle<double, angle::Radian, angle::NoWrap>(M_PI) - tAngle<double, angle::Radian, angle::NoWrap>(M_PI_2)));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::NoWrap>(M_PI_2)), (tAngle<double, angle::Radian, angle::NoWrap>(M_PI) - tAngle<float, angle::Degree, angle::NoWrap>(90)));
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(10.0, (tAngle<double, angle::Radian, angle::NoWrap>(2.0) / tAngle<double, angle::Radian, angle::NoWrap>(0.2)), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(45.0, (tAngle<double, angle::Radian, angle::NoWrap>(M_PI_2) / tAngle<float, angle::Degree, angle::NoWrap>(2.0)), 1E-6);
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::NoWrap>(2.0 * 2.5)), (tAngle<double, angle::Radian, angle::NoWrap>(2.0) * 2.5));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::NoWrap>(2.0) * 2.5), (2.5 * tAngle<double, angle::Radian, angle::NoWrap>(2.0)));

    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Signed>(3 * M_PI_2)), (tAngle<double, angle::Radian, angle::Signed>(M_PI) + tAngle<double, angle::Radian, angle::NoWrap>(M_PI_2)));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Signed>(3 * M_PI_2)), (tAngle<double, angle::Radian, angle::Signed>(M_PI) + tAngle<float, angle::Degree, angle::NoWrap>(90)));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Signed>(M_PI_2)), (tAngle<double, angle::Radian, angle::Signed>(M_PI) - tAngle<double, angle::Radian, angle::NoWrap>(M_PI_2)));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Signed>(M_PI_2)), (tAngle<double, angle::Radian, angle::Signed>(M_PI) - tAngle<float, angle::Degree, angle::NoWrap>(90)));
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(10.0, (tAngle<double, angle::Radian, angle::Signed>(2.0) / tAngle<double, angle::Radian, angle::NoWrap>(0.2)), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(45.0, (tAngle<double, angle::Radian, angle::Signed>(M_PI_2) / tAngle<float, angle::Degree, angle::NoWrap>(2.0)), 1E-6);
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Signed>(2.0 * 2.5)), (tAngle<double, angle::Radian, angle::Signed>(2.0) * 2.5));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Signed>(2.0) * 2.5), (2.5 * tAngle<double, angle::Radian, angle::Signed>(2.0)));

    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Unsigned>(3 * M_PI_2)), (tAngle<double, angle::Radian, angle::Unsigned>(M_PI) + tAngle<double, angle::Radian, angle::NoWrap>(M_PI_2)));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Unsigned>(3 * M_PI_2)), (tAngle<double, angle::Radian, angle::Unsigned>(M_PI) + tAngle<float, angle::Degree, angle::NoWrap>(90)));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Unsigned>(M_PI_2)), (tAngle<double, angle::Radian, angle::Unsigned>(M_PI) - tAngle<double, angle::Radian, angle::NoWrap>(M_PI_2)));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Unsigned>(M_PI_2)), (tAngle<double, angle::Radian, angle::Unsigned>(M_PI) - tAngle<float, angle::Degree, angle::NoWrap>(90)));
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(10.0, (tAngle<double, angle::Radian, angle::Unsigned>(2.0) / tAngle<double, angle::Radian, angle::NoWrap>(0.2)), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(45.0, (tAngle<double, angle::Radian, angle::Unsigned>(M_PI_2) / tAngle<float, angle::Degree, angle::NoWrap>(2.0)), 1E-6);
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Unsigned>(2.0 * 2.5)), (tAngle<double, angle::Radian, angle::Unsigned>(2.0) * 2.5));
    RRLIB_UNIT_TESTS_EQUALITY((tAngle<double, angle::Radian, angle::Unsigned>(2.0) * 2.5), (2.5 * tAngle<double, angle::Radian, angle::Unsigned>(2.0)));
  }

  void Streaming()
  {
    std::stringstream actual;
    std::stringstream expected;

    actual << tAngle<double, angle::Radian, angle::NoWrap>(5 * M_PI_2);
    expected << 5 * M_PI_2;
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    actual.str("");
    expected.str("");
    actual << tAngle<double, angle::Radian, angle::Signed>(3 * M_PI_2);
    expected << -M_PI_2;
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    actual.str("");
    expected.str("");
    actual << tAngle<double, angle::Radian, angle::Unsigned>(3 * M_PI_2);
    expected << 3 * M_PI_2;
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    actual.str("");
    actual << tAngle<double, angle::Degree, angle::NoWrap>(420);
    RRLIB_UNIT_TESTS_EQUALITY(std::string("420°"), actual.str());

    actual.str("");
    actual << tAngle<double, angle::Degree, angle::Signed>(270);
    RRLIB_UNIT_TESTS_EQUALITY(std::string("-90°"), actual.str());

    actual.str("");
    actual << tAngle<double, angle::Degree, angle::Unsigned>(420);
    RRLIB_UNIT_TESTS_EQUALITY(std::string("60°"), actual.str());

    tAngle<double, angle::Radian, angle::NoWrap> rad;
    tAngle<float, angle::Degree, angle::NoWrap> deg;
    std::stringstream source;
    source.exceptions(std::istream::failbit);
    RRLIB_UNIT_TESTS_EXCEPTION(source >> rad, std::ios_base::failure);
    source.clear();
    source << 5 * M_PI_2;
    source >> rad;
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(5 * M_PI_2, rad.Value(), 1E-5);
    RRLIB_UNIT_TESTS_ASSERT(source.eof());
    source.clear();
    source << 5 * M_PI_2;
    RRLIB_UNIT_TESTS_EXCEPTION(source >> deg, std::ios_base::failure);

    source.clear();
    source << "120.0°";
    source >> deg;
    RRLIB_UNIT_TESTS_EQUALITY(120.0f, deg.Value());
    source.clear();
    source << "120.0°";
    source >> rad;
    std::string rest;
    source >> rest;
    RRLIB_UNIT_TESTS_ASSERT(rad.Value() == 120.0 && rest == "°");

    source.clear();
    source << "1°, 2°, ende";
    for (size_t i = 0; i < 2; ++i)
    {
      tAngle<double, angle::Degree, angle::NoWrap> angle;
      source >> angle;
      RRLIB_UNIT_TESTS_EQUALITY(i + 1.0, angle.Value());
      char delim;
      source >> delim;
    }

    source.str("");
    source.clear();
    source << 0.0 << ", " << M_PI_2 << ", " << ", " << "ende";
    for (size_t i = 0; i < 2; ++i)
    {
      tAngle<double, angle::Radian, angle::NoWrap> angle;
      source >> angle;
      RRLIB_UNIT_TESTS_EQUALITY_DOUBLE(i * M_PI_2, angle.Value(), 1E-5);
      char delim;
      source >> delim;
    }
  }

  void TestFunctions()
  {
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle between 0 deg and 30 deg must be 30 deg (unsigned)", tAngleDegUnsigned(30), GetAngleInbetween(tAngleDeg(0), tAngleDeg(30)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle between 0 deg and 30 deg must be 30 deg (signed)", tAngleDegUnsigned(30), GetAngleInbetween(tAngleDegSigned(0), tAngleDegSigned(30)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle between 0 and pi must be pi", tAngleRadUnsigned(M_PI), GetAngleInbetween(tAngleRad(0), tAngleRad(M_PI)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle between 30 deg and 0 deg must be 330 deg", tAngleDegUnsigned(330), GetAngleInbetween(tAngleDeg(30), tAngleDeg(0)));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Angle between pi/2 and 3/2 pi must be pi", tAngleRadUnsigned(M_PI), GetAngleInbetween(tAngleRad(M_PI / 2), tAngleRad(3 / 2. * M_PI)));

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

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
