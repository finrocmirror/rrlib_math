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
/*!\file    rrlib/math/test/unit_test_vectors.cpp
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

#include "rrlib/math/tVector.h"

#ifdef _LIB_OIV_PRESENT_
#include <Inventor/SbVec3f.h>
#endif

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
class TestVectors : public util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(TestVectors);
  RRLIB_UNIT_TESTS_ADD_TEST(Constructors);
  RRLIB_UNIT_TESTS_ADD_TEST(AccessOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ComparisonOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(AssignmentOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ArithmeticOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(Cartesian2D);
  RRLIB_UNIT_TESTS_ADD_TEST(PolarDefault);
  RRLIB_UNIT_TESTS_ADD_TEST(PolarRadianSigned);
  RRLIB_UNIT_TESTS_ADD_TEST(PolarDegreeSigned);
  RRLIB_UNIT_TESTS_ADD_TEST(PolarAssignments);
  RRLIB_UNIT_TESTS_ADD_TEST(PolarOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(Streaming);
#ifdef _LIB_OIV_PRESENT_
  RRLIB_UNIT_TESTS_ADD_TEST(CoinConversions);
#endif
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  void Constructors()
  {
    tVector<2, double> zero_vector;
    double zero_array[2] = { 0.0, 0.0 };
    RRLIB_UNIT_TESTS_EQUALITY(2 * sizeof(double), sizeof(zero_vector));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero_vector, zero_array, sizeof(zero_vector)) == 0);

    double raw[] = { 1.0, 2.0, 3.0, 4.0 };
    tVector<4, double> vector(1, 2, 3, 4);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&vector, raw, sizeof(vector)) == 0);

    tVector<4, double> copy(vector);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy, &vector, sizeof(copy)) == 0);

    tVector<4, float> converted(vector);
    float converted_raw[] = { 1.0, 2.0, 3.0, 4.0 };
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted, &converted_raw, sizeof(converted)) == 0);

    tVector<2, double, vector::Polar> polar_vector(M_PI, 1);
    RRLIB_UNIT_TESTS_EQUALITY(tAngleRad(M_PI), polar_vector[0]);
    RRLIB_UNIT_TESTS_EQUALITY(1.0, polar_vector.Length());
  }

  void AccessOperators()
  {
    tVector<4, double> vector(1, 2, 3, 4);

    RRLIB_UNIT_TESTS_EQUALITY(1.0, vector[0]);
    RRLIB_UNIT_TESTS_EQUALITY(2.0, vector[1]);
    RRLIB_UNIT_TESTS_EQUALITY(3.0, vector[2]);
    RRLIB_UNIT_TESTS_EQUALITY(4.0, vector[3]);

    vector[1] = 5.0;
    RRLIB_UNIT_TESTS_EQUALITY(5.0, vector[1]);
  }

  void ComparisonOperators()
  {
    typedef math::tVector<4, double> tVector;
    RRLIB_UNIT_TESTS_ASSERT(tVector(1, 2, 3, 4) == tVector(1, 2, 3, 4));
    RRLIB_UNIT_TESTS_ASSERT(tVector(1, 2, 3, 4) != tVector(1, 3, 3, 4));
  }

  void AssignmentOperators()
  {
    tVector<4, double> vector;
    vector = tVector<4, double>(1, 2, 3, 4);
    RRLIB_UNIT_TESTS_EQUALITY((tVector<4, double>(1, 2, 3, 4)), vector);
    vector = tVector<4, float>(2, 3, 4, 5);
    RRLIB_UNIT_TESTS_EQUALITY((tVector<4, double>(2, 3, 4, 5)), vector);
    vector.Set(3, 4, 5, 6);
    RRLIB_UNIT_TESTS_EQUALITY((tVector<4, double>(3, 4, 5, 6)), vector);
  }

  void ArithmeticOperators()
  {
    typedef math::tVector<4, double> tVector;
    RRLIB_UNIT_TESTS_EQUALITY(tVector(-1, -2, -3, -4), -tVector(1, 2, 3, 4));
    RRLIB_UNIT_TESTS_EQUALITY(tVector(1 + 2, 2 + 3, 3 + 4, 4 + 5), tVector(1, 2, 3, 4) + tVector(2, 3, 4, 5));
    RRLIB_UNIT_TESTS_EQUALITY(tVector(1 - 2, 2 - 3, 3 - 4, 4 - 5), tVector(1, 2, 3, 4) - tVector(2, 3, 4, 5));
    RRLIB_UNIT_TESTS_EQUALITY(40.0, tVector(1, 2, 3, 4) * tVector(2, 3, 4, 5));
    RRLIB_UNIT_TESTS_EQUALITY(tVector(1 * 2.0, 2 * 2.0, 3 * 2.0, 4 * 2.0), tVector(1, 2, 3, 4) * 2.0);
    RRLIB_UNIT_TESTS_EQUALITY(tVector(1, 2, 3, 4) * 2.0, 2.0 * tVector(1, 2, 3, 4));
  }

  void Cartesian2D()
  {

    typedef tVec2u tVec;

    tVec vec(1, 2);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("X coordinate must be correct", (unsigned int) 1, vec.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Y coordinate must be correct", (unsigned int) 2, vec.Y());

    vec.X() = 99;
    vec.Y() = 100;
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be correct after assignment", vec, tVec(99, 100));


    vec = tVec(10, 0);
    {
      tVector<2, double, vector::Polar, angle::Radian, angle::Signed> polar = tVec2d(vec).GetPolarVector();
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Polar vector (Radian) must be correct", (tVector<2, double, vector::Polar, angle::Radian, angle::Signed>(0, 10)), polar);
    }
    {
      tVector<2, double, vector::Polar, angle::Degree, angle::Signed> polar = tVec2d(vec).GetPolarVector();
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Polar vector (Degree) must be correct", (tVector<2, double, vector::Polar, angle::Degree, angle::Signed>(0, 10)), polar);
    }

    vec = tVec(1, 1);
    {
      tVector<2, double, vector::Polar, angle::Radian, angle::Signed> polar = tVec2d(vec).GetPolarVector();
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Polar vector (Radian) must be correct", (tVector<2, double, vector::Polar, angle::Radian, angle::Signed>(M_PI / 4, std::sqrt(2))), polar);
    }
    {
      tVector<2, double, vector::Polar, angle::Degree, angle::Signed> polar = tVec2d(vec).GetPolarVector();
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Polar vector (Degree) must be correct", (tVector<2, double, vector::Polar, angle::Degree, angle::Signed>(45, std::sqrt(2))), polar);
    }

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", (unsigned int) 5, tVec(5, 0).Length());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of SquaredLength() must be correct", (unsigned int) 25, tVec(5, 0).SquaredLength());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", (unsigned int) 5, tVec(0, 5).Length());

#if 0
    rrlib::serialization::tStringOutputStream string;
    rrlib::serialization::Serialize(string, vec);

    rrlib::xml::tNode node;
    rrlib::serialization::Serialize(node, vec);

#endif

    // FIXME: many more
  }

  void PolarRadianSigned()
  {
    typedef tVector<2, double, vector::Polar, angle::Radian, angle::Signed> tVec;
    tVec vec;

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be zero after initialization", tVec(0, 0), vec);

    vec = tVec(1, 2);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be correct after assignment", tVec(1, 2), vec);

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("First coordinate must be correct", tAngleRad(1), vec[0]);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Length must be correct", (double) 2, vec.Length());

    vec[0] = tAngleRad(3.14);
    vec.Length() = 100.;
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be correct after assignment", tVec(3.14, 100), vec);

    vec = tVec(M_PI, 10);
    tVec2d cart = vec.GetCartesianVector();
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector X must be correct", -10, cart.X(), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector Y must be correct", 0, cart.Y(), 1E-9);

    vec = tVec(M_PI / 2, 10);
    cart = vec.GetCartesianVector();
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector X must be correct", 0, cart.X(), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector Y must be correct", 10, cart.Y(), 1E-9);

    vec = tVec(M_PI / 4, 10);
    cart = vec.GetCartesianVector();
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector X must be correct", 10 * 1.41421356237 / 2, cart.X(), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector Y must be correct", 10 * 1.41421356237 / 2, cart.Y(), 1E-9);


    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of addition must be correct", tVec(0, 100), tVec(0, 99) + tVec(0, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of subtraction must be correct", tVec(0, 99), tVec(0, 100) - tVec(0, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of inverse must be correct", tVec(-M_PI, 1), - tVec(0, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of inverse must be correct", tVec(M_PI / 2, 3.3), - tVec(-M_PI / 2, 3.3));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of multiplication w/ scalar must be correct", tVec(2, 2), 2 * tVec(2, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of multiplication w/ scalar must be correct", tVec(2, 2), tVec(2, 1) * 2);

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", 5.0, tVec(0, 5).Length());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of SquaredLength() must be correct", 25.0, tVec(0, 5).SquaredLength());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", 5.0, tVec(M_PI / 2, 5).Length());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", 5.0, tVec(M_PI, 5).Length());

#if 0
    rrlib::serialization::tOutputStream stream;
    //rrlib::serialization::Serialize(stream, vec);

    rrlib::serialization::tStringOutputStream string;
    rrlib::serialization::Serialize(string, vec);


    //node << vec;
    rrlib::xml::tNode node;
    rrlib::serialization::Serialize(node, vec);
#endif


  }

  void PolarDefault()
  {
#if 1
    typedef tVector<2, double, vector::Polar> tVec;
    tVec vec;

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be zero after initialization", tVec(0, 0), vec);

    vec = tVec(1, 2);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be correct after assignment", tVec(1, 2), vec);

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("First coordinate must be correct", tAngleRad(1), vec[0]);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Length must be correct", (double) 2, vec.Length());

    vec[0] = tAngleRad(3.14);
    vec.Length() = 100.;
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be correct after assignment", tVec(3.14, 100), vec);

    vec = tVec(M_PI, 10);
    tVec2d cart = vec.GetCartesianVector();
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector X must be correct", -10, cart.X(), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector Y must be correct", 0, cart.Y(), 1E-9);

    vec = tVec(M_PI / 2, 10);
    cart = vec.GetCartesianVector();
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector X must be correct", 0, cart.X(), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector Y must be correct", 10, cart.Y(), 1E-9);

    vec = tVec(M_PI / 4, 10);
    cart = vec.GetCartesianVector();
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector X must be correct", 10 * 1.41421356237 / 2, cart.X(), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector Y must be correct", 10 * 1.41421356237 / 2, cart.Y(), 1E-9);


    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of addition must be correct", tVec(0, 100), tVec(0, 99) + tVec(0, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of subtraction must be correct", tVec(0, 99), tVec(0, 100) - tVec(0, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of inverse must be correct", tVec(-M_PI, 1), - tVec(0, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of inverse must be correct", tVec(M_PI / 2, 3.3), - tVec(-M_PI / 2, 3.3));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of multiplication w/ scalar must be correct", tVec(2, 2), 2 * tVec(2, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of multiplication w/ scalar must be correct", tVec(2, 2), tVec(2, 1) * 2);

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", 5.0, tVec(0, 5).Length());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", 5.0, tVec(M_PI / 2, 5).Length());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", 5.0, tVec(M_PI, 5).Length());
#endif
  }

  void PolarDegreeSigned()
  {
    typedef tVector<2, double, vector::Polar, angle::Degree, angle::Signed> tVec;
    tVec vec;

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be zero after initialization", tVec(0, 0), vec);

    vec = tVec(1, 2);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be correct after assignment", tVec(1, 2), vec);

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("First coordinate must be correct", tAngleDeg(1), vec[0]);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Length must be correct", (double) 2, vec.Length());

    vec[0] = tAngleDeg(90);
    vec.Length() = 100.;

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be correct after assignment", tVec(90, 100), vec);

    vec = tVec(0, 10);
    tVec2d cart = vec.GetCartesianVector();
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Cartesian vector must be correct", tVec2d(10, 0), cart);

    vec = tVec(180, 10);
    cart = vec.GetCartesianVector();
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector X must be correct", -10, cart.X(), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector Y must be correct", 0, cart.Y(), 1E-9);

    vec = tVec(90, 10);
    cart = vec.GetCartesianVector();
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector X must be correct", 0, cart.X(), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector Y must be correct", 10, cart.Y(), 1E-9);

    vec = tVec(45, 10);
    cart = vec.GetCartesianVector();
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector X must be correct", 10 * 1.41421356237 / 2, cart.X(), 1E-9);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("Cartesian vector Y must be correct", 10 * 1.41421356237 / 2, cart.Y(), 1E-9);


    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of addition must be correct", tVec(0, 100), tVec(0, 99) + tVec(0, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of subtraction must be correct", tVec(0, 99), tVec(0, 100) - tVec(0, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of inverse must be correct", tVec(-170, 1), - tVec(10, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of inverse must be correct", tVec(135, 3.3), - tVec(-45, 3.3));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of multiplication w/ scalar must be correct", tVec(2, 2), 2 * tVec(2, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of multiplication w/ scalar must be correct", tVec(2, 2), tVec(2, 1) * 2);

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", 5.0, tVec(0, 5).Length());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", 5.0, tVec(M_PI / 2, 5).Length());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", 5.0, tVec(M_PI, 5).Length());

  }

  void PolarAssignments()
  {
    typedef tVector<3, double, vector::Polar, angle::Degree, angle::Signed> tVecDegSigned;
    typedef tVector<3, double, vector::Polar, angle::Radian, angle::Signed> tVecRadSigned;
    typedef tVector<3, double, vector::Polar, angle::Degree, angle::Unsigned> tVecDegUnsigned;
    typedef tVector<3, double, vector::Polar, angle::Radian, angle::Unsigned> tVecRadUnsigned;

    tVecDegUnsigned a(270, 90, 1);
    tVecDegSigned b = a;
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Assignment must yield signed degree angles", (double) - 90, (double) b.Alpha());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Assignment must yield signed degree angles", (double) 90, (double) b.Beta());

    tVecDegUnsigned c = b;
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Assignment must yield unsigned degree angles", (double) 270, (double) c.Alpha());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Assignment must yield unsigned degree angles", (double) 90, (double) c.Beta());

    tVecRadSigned d = a;
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Assignment must yield signed radian angles", (double) - M_PI / 2, (double) d.Alpha());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Assignment must yield signed radian angles", (double) M_PI / 2, (double) d.Beta());

    tVecRadUnsigned e = a;
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Assignment must yield unsigned radian angles", (double) 3 * (M_PI / 2), (double) e.Alpha());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Assignment must yield unsigned radian angles", (double) M_PI / 2, (double) e.Beta());

  }

  void PolarOperators()
  {
#if 0
    typedef tVector<3, double, vector::Polar, angle::Degree, angle::Signed> tVecDegSigned;
    typedef tVector<3, double, vector::Polar, angle::Radian, angle::Signed> tVecRadSigned;
    typedef tVector<3, double, vector::Polar, angle::Degree, angle::Unsigned> tVecDegUnsigned;
    typedef tVector<3, double, vector::Polar, angle::Radian, angle::Unsigned> tVecRadUnsigned;

    tVecDegUnsigned a(0, 0, 1);
    tVecRadSigned b(M_PI / 4, M_PI / 4, 1);

    tVecDegUnsigned c = a + b;
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Addition must be correct", tVecDegUnsigned(90, 90, 1), c);
#endif
  }

  void Streaming()
  {
    std::stringstream actual;
    std::stringstream expected;

    actual << tVector<2, double>(0, 1);
    expected << "(0, 1)";
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    actual << tVector<3, int>(0, 1, 2);
    expected << "(0, 1, 2)";
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    actual << tVector<3, unsigned char>(0, 1, 2);
    expected << "(0, 1, 2)";
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    tVector<2, double> vector_2d_double;
    tVector<3, int> vector_3d_int;
    tVector<3, unsigned char> vector_3d_char;
    std::stringstream source;
    source.exceptions(std::istream::failbit);
    RRLIB_UNIT_TESTS_EXCEPTION(source >> vector_2d_double, std::ios_base::failure);
    source.clear();
    RRLIB_UNIT_TESTS_EXCEPTION(source >> vector_3d_int, std::ios_base::failure);
    source.clear();
    RRLIB_UNIT_TESTS_EXCEPTION(source >> vector_3d_char, std::ios_base::failure);
    source.clear();
    source << "(3.2, 4.1)";
    source >> vector_2d_double;
    RRLIB_UNIT_TESTS_EQUALITY((tVector<2, double>(3.2, 4.1)), vector_2d_double);
    source.clear();
    source << "(120)";
    RRLIB_UNIT_TESTS_EXCEPTION(source >> vector_2d_double, std::ios_base::failure);
    source.clear();
    source.str("");
    source << "(3, 4, 5)";
    source >> vector_3d_int;
    RRLIB_UNIT_TESTS_EQUALITY((tVector<3, int>(3, 4, 5)), vector_3d_int);
    source.clear();
    source << "(120)";
    RRLIB_UNIT_TESTS_EXCEPTION(source >> vector_3d_int, std::ios_base::failure);
    source.clear();
    source.str("");
    source << "(0, 1, 2)";
    source >> vector_3d_char;
    RRLIB_UNIT_TESTS_EQUALITY((tVector<3, unsigned char>(0, 1, 2)), vector_3d_char);
    source.clear();
    source << "(120)";
    RRLIB_UNIT_TESTS_EXCEPTION(source >> vector_3d_char, std::ios_base::failure);
  }

#ifdef _LIB_OIV_PRESENT_
  void CoinConversions()
  {
    tVector<3, double> rrlib_vector(1, 2, 3);
    SbVec3f sb_vector = rrlib_vector.GetCoinVector();
    RRLIB_UNIT_TESTS_ASSERT(sb_vector.equals(SbVec3f(1, 2, 3), 1E-6));
    RRLIB_UNIT_TESTS_EQUALITY(rrlib_vector, (tVector<3, double>(sb_vector)));
  }
#endif

};

RRLIB_UNIT_TESTS_REGISTER_SUITE(TestVectors);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
