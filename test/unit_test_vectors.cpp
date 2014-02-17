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
 *
 * \date    2013-06-14
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
#include "rrlib/math/tVector.h"

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

class tTestVectors : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(tTestVectors);
  RRLIB_UNIT_TESTS_ADD_TEST(TestCartesian2D);
  RRLIB_UNIT_TESTS_ADD_TEST(TestPolarDefault);
  RRLIB_UNIT_TESTS_ADD_TEST(TestPolarRadianSigned);
  RRLIB_UNIT_TESTS_ADD_TEST(TestPolarDegreeSigned);
  RRLIB_UNIT_TESTS_ADD_TEST(TestPolarAssignments);
  RRLIB_UNIT_TESTS_ADD_TEST(TestPolarOperators);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  virtual void InitializeTests() {}
  virtual void CleanUp() {}

  virtual void TestCartesian2D()
  {

    typedef tVec2u tVec;

    tVec vec;

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be zero after initialization", tVec(0, 0), vec);

    vec = tVec(1, 2);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Vector must be correct after assignment", tVec(1, 2), vec);

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


    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of addition must be correct", tVec(100, 100), tVec(99, 99) + tVec(1, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of subtraction must be correct", tVec(98, 98), tVec(99, 99) - tVec(1, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of inverse must be correct", tVec(-1, -1), - tVec(1, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of multiplication w/ scalar must be correct", tVec(2, 2), 2 * tVec(1, 1));
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of multiplication w/ scalar must be correct", tVec(2, 2), tVec(1, 1) * 2);

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", (unsigned int) 5, tVec(5, 0).Length());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of Length() must be correct", (unsigned int) 5, tVec(0, 5).Length());

#if 0
    rrlib::serialization::tStringOutputStream string;
    rrlib::serialization::Serialize(string, vec);

    rrlib::xml::tNode node;
    rrlib::serialization::Serialize(node, vec);

#endif

    // FIXME: many more
  }

  virtual void TestPolarRadianSigned()
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

  virtual void TestPolarDefault()
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

  virtual void TestPolarDegreeSigned()
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

  virtual void TestPolarAssignments()
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

  virtual void TestPolarOperators()
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

};

RRLIB_UNIT_TESTS_REGISTER_SUITE(tTestVectors);
