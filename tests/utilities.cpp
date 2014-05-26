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
#include "rrlib/util/tUnitTestSuite.h"

#include "rrlib/math/utilities.h"

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
class tTestUtilities : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(tTestUtilities);
  RRLIB_UNIT_TESTS_ADD_TEST(TestSignum);
  RRLIB_UNIT_TESTS_ADD_TEST(TestLimitedValue);
  RRLIB_UNIT_TESTS_ADD_TEST(TestBinomialCoefficient);
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

  void TestBinomialCoefficient()
  {
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(1), BinomialCoefficient(0, 0));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(0), BinomialCoefficient(0, 1));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(1), BinomialCoefficient(1, 0));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(1), BinomialCoefficient(1, 1));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(1), BinomialCoefficient(2, 0));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(2), BinomialCoefficient(2, 1));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(1), BinomialCoefficient(2, 2));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(1), BinomialCoefficient(3, 0));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(3), BinomialCoefficient(3, 1));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(3), BinomialCoefficient(3, 2));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(1), BinomialCoefficient(3, 3));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(10), BinomialCoefficient(5, 3));
  }
};

RRLIB_UNIT_TESTS_REGISTER_SUITE(tTestUtilities);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
