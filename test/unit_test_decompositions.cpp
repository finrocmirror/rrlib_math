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
/*!\file    rrlib/math/test/unit_test_decompositions.cpp
 *
 * \author  Patrick Fleischmann
 *
 * \date    2013-10-21
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstdlib>
#include <iostream>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/util/tUnitTestSuite.h"

#include "rrlib/math/tLUDecomposition.h"
#include "rrlib/math/tMatrix.h"
#include "rrlib/math/tVector.h"

#include "rrlib/logging/configuration.h"

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
class tTestDecompositions : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(tTestDecompositions);
  RRLIB_UNIT_TESTS_ADD_TEST(Test);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  virtual void InitializeTests()
  {
//    rrlib::logging::SetDomainMaxMessageLevel(".", rrlib::logging::tLogLevel::DEBUG_VERBOSE_3);
  }

  virtual void CleanUp()
  {}

  virtual void Test()
  {
    // 3x3 equation system with doubles
    rrlib::math::tMatrix<3, 3, double, matrix::Full> matrix1(
      -5.0, -1.0,  2.0,
      -2.0,  6.0,  2.0,
      4.0,  2.0, -8.0
    );

    rrlib::math::tVector<3, double> vec1(-20.0, 2.0, -2.0);
    rrlib::math::tVector<3, double> result1 = rrlib::math::tLUDecomposition<3, double>(matrix1).Solve(vec1);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of solved equation system should be (5,1,3)", rrlib::math::tVec3d(5.0, 1.0, 3.0), (rrlib::math::tVec3d) result1);


    // 3x3 equation system with doubles
    rrlib::math::tMatrix<3, 3, double, matrix::Full> matrix2(
      9.0,  3.0,  1.0,
      4.0,  2.0,  1.0,
      1.0,  1.0, -1.0
    );

    rrlib::math::tVector<3, double> vec2(20.0, 19.0, 0.0);
    rrlib::math::tVector<3, double> result2 = rrlib::math::tLUDecomposition<3, double>(matrix2).Solve(vec2);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of solved equation system should be (-1.6,9,7.4)", rrlib::math::tVec3d(-1.6, 9.0, 7.4), (rrlib::math::tVec3d) result2);


    // 5x5 equation system with doubles
    rrlib::math::tMatrix<5, 5, double, matrix::Full> matrix3(
      2.0,  4.0,  2.0,  3.0,  5.0,
      -6.0,  4.0,  4.0, -2.0,  1.0,
      7.0, -3.0,  6.0,  1.0,  2.0,
      9.0, -1.0,  8.0, -4.0,  2.0,
      1.0,  6.0, 10.0,  7.0,  9.0
    );

    rrlib::math::tVector<5, double> vec3(5.0, -5.0, -2.0, -4.0, 7.0);
    rrlib::math::tVector<5, double> result3 = rrlib::math::tLUDecomposition<5, double>(matrix3).Solve(vec3);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of solved equation system should be (2, 4, 0.5, 3, -5)", (rrlib::math::tVector<5, double>(2.0, 4.0, 0.5, 3.0, -5.0)), (rrlib::math::tVector<5, double>) result3);

    // over-determined 3x2 equation system
    rrlib::math::tMatrix<3, 2, double, matrix::Full> matrix4(
      2.0,  0.0,
      2.0,  0.0,
      2.0, -1.0
    );

    rrlib::math::tVector<3, double> vec4(1.0, 1.0, 0.0);
    rrlib::math::tVector<3, double> result4 = rrlib::math::tLUDecomposition<2, double>(matrix4).Solve(vec4);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of solved equation system should be (0.5, 1, 0)", rrlib::math::tVec3d(0.5, 1.0, 0.0), (rrlib::math::tVector<3, double>) result4);

    // over-determined 3x2 equation system
    rrlib::math::tMatrix<3, 2, double, matrix::Full> matrix5(
      0.0,  0.0,
      0.0,  1.0,
      -1.0,  0.0
    );

    rrlib::math::tVector<3, double> vec5(0.0, 1.0, -2.0);
    rrlib::math::tVector<3, double> result5 = rrlib::math::tLUDecomposition<2, double>(matrix5).Solve(vec5);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Result of solved equation system should be (2, 1, 0)", rrlib::math::tVec3d(2.0, 1.0, 0.0), (rrlib::math::tVector<3, double>) result5);
  }
};

RRLIB_UNIT_TESTS_REGISTER_SUITE(tTestDecompositions);
