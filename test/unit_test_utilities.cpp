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
/*!\file    rrlib/math/test/unit_test_utilities.cpp
 *
 * \author  Tobias Foehst
 * \author  Jens Wettach
 *
 * \date    2011-01-11
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
#include "rrlib/math/utilities.h"

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

class tTestUtilities : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(tTestUtilities);
  RRLIB_UNIT_TESTS_ADD_TEST(TestSignum);
  RRLIB_UNIT_TESTS_ADD_TEST(TestLimitedValue);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  virtual void InitializeTests() {}
  virtual void CleanUp() {}

  void TestSignum()
  {
    RRLIB_UNIT_TESTS_EQUALITY(Signum(4711), 1);
    RRLIB_UNIT_TESTS_EQUALITY(Signum(0), 0);
    RRLIB_UNIT_TESTS_EQUALITY(Signum(-0.1), -1);
  }

  void TestLimitedValue()
  {
    RRLIB_UNIT_TESTS_EQUALITY(LimitedValue(123, 12, 1234), 123);
    RRLIB_UNIT_TESTS_EQUALITY(LimitedValue(12, 123, 1234), 123);
    RRLIB_UNIT_TESTS_EQUALITY(LimitedValue(1234, 12, 123), 123);
  }

};

RRLIB_UNIT_TESTS_REGISTER_SUITE(tTestUtilities);
