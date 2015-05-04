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
/*!\file    rrlib/math/test/test_matrices.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-21
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/util/tUnitTestSuite.h"

#include <cstring>

#include "rrlib/math/tMatrix.h"
#include "rrlib/math/tCholeskyDecomposition.h"
#include "rrlib/math/tLUDecomposition.h"

#ifdef _LIB_OIV_PRESENT_
#include "rrlib/simvis3d/math_functions.h"
#include <Inventor/SbRotation.h>
#include <cmath>
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
class TestMatrices : public util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(TestMatrices);
  RRLIB_UNIT_TESTS_ADD_TEST(Constructors);
  RRLIB_UNIT_TESTS_ADD_TEST(AccessOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ComparisonOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(AssignmentOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ArithmeticOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(Transpose);
  RRLIB_UNIT_TESTS_ADD_TEST(InversionOfHilbertMatrix<5>);
  RRLIB_UNIT_TESTS_ADD_TEST(CholeskyDecomposition);
  RRLIB_UNIT_TESTS_ADD_TEST(LUDecomposition);
#ifdef _LIB_OIV_PRESENT_
  RRLIB_UNIT_TESTS_ADD_TEST(CoinConversions);
#endif
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  void Constructors()
  {
    tMatrix<2, 2, double> zero_matrix;
    double zero_array[4] = { 0.0, 0.0, 0.0, 0.0 };
    RRLIB_UNIT_TESTS_EQUALITY(4 * sizeof(double), sizeof(zero_matrix));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero_matrix, zero_array, sizeof(zero_matrix)) == 0);

    auto identity_matrix = tMatrix<3, 3, double>::Identity();
    double identity_array[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
    RRLIB_UNIT_TESTS_EQUALITY(9 * sizeof(double), sizeof(identity_matrix));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&identity_matrix, identity_array, sizeof(identity_matrix)) == 0);

    double raw[] = { 1.0, 2.0, 3.0, 4.0 };
    tMatrix<2, 2, double> matrix(1, 2, 3, 4);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&matrix, raw, sizeof(matrix)) == 0);

    tMatrix<2, 2, double> copy(matrix);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy, &matrix, sizeof(copy)) == 0);

    tMatrix<2, 2, float> converted(matrix);
    float converted_raw[] = { 1.0, 2.0, 3.0, 4.0 };
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted, &converted_raw, sizeof(converted)) == 0);
  }

  void AccessOperators()
  {
    tMatrix<2, 2, double> matrix(1, 2, 3, 4);

    RRLIB_UNIT_TESTS_EQUALITY(1.0, matrix[0][0]);
    RRLIB_UNIT_TESTS_EQUALITY(2.0, matrix[0][1]);
    RRLIB_UNIT_TESTS_EQUALITY(3.0, matrix[1][0]);
    RRLIB_UNIT_TESTS_EQUALITY(4.0, matrix[1][1]);

    matrix[0][1] = 5.0;
    RRLIB_UNIT_TESTS_EQUALITY(5.0, matrix[0][1]);
  }

  void ComparisonOperators()
  {
    typedef math::tMatrix<2, 2, double> tMatrix;
    RRLIB_UNIT_TESTS_ASSERT(tMatrix(1, 2, 3, 4) == tMatrix(1, 2, 3, 4));
    RRLIB_UNIT_TESTS_ASSERT(tMatrix(1, 2, 3, 4) != tMatrix(1, 3, 3, 4));
  }

  void AssignmentOperators()
  {
    tMatrix<2, 2, double> matrix;
    matrix = tMatrix<2, 2, double>(1, 2, 3, 4);
    RRLIB_UNIT_TESTS_EQUALITY((tMatrix<2, 2, double>(1, 2, 3, 4)), matrix);
    matrix = tMatrix<2, 2, float>(2, 3, 4, 5);
    RRLIB_UNIT_TESTS_EQUALITY((tMatrix<2, 2, double>(2, 3, 4, 5)), matrix);
    matrix.Set(3, 4, 5, 6);
    RRLIB_UNIT_TESTS_EQUALITY((tMatrix<2, 2, double>(3, 4, 5, 6)), matrix);
  }

  void ArithmeticOperators()
  {
    typedef math::tMatrix<2, 2, double> tMatrix;
    typedef math::tVector<2, double> tVector;
    RRLIB_UNIT_TESTS_EQUALITY(tMatrix(-1, -2, -3, -4), -tMatrix(1, 2, 3, 4));
    RRLIB_UNIT_TESTS_EQUALITY(tMatrix(1 + 2, 2 + 3, 3 + 4, 4 + 5), tMatrix(1, 2, 3, 4) + tMatrix(2, 3, 4, 5));
    RRLIB_UNIT_TESTS_EQUALITY(tMatrix(1 - 2, 2 - 3, 3 - 4, 4 - 5), tMatrix(1, 2, 3, 4) - tMatrix(2, 3, 4, 5));
    RRLIB_UNIT_TESTS_EQUALITY(tMatrix(1 * 2 + 2 * 4, 1 * 3 + 2 * 5, 3 * 2 + 4 * 4, 3 * 3 + 4 * 5), tMatrix(1, 2, 3, 4) * tMatrix(2, 3, 4, 5));
    RRLIB_UNIT_TESTS_EQUALITY(tVector(1 * 2 + 2 * 4, 3 * 2 + 4 * 4), tMatrix(1, 2, 3, 4) * tVector(2, 4));
    RRLIB_UNIT_TESTS_EQUALITY(tVector(2 * 1 + 4 * 3, 2 * 2 + 4 * 4), tVector(2, 4) * tMatrix(1, 2, 3, 4));
    RRLIB_UNIT_TESTS_EQUALITY(tMatrix(1 * 2.0, 2 * 2.0, 3 * 2.0, 4 * 2.0), tMatrix(1, 2, 3, 4) * 2.0);
    RRLIB_UNIT_TESTS_EQUALITY(tMatrix(1, 2, 3, 4) * 2.0, 2.0 * tMatrix(1, 2, 3, 4));
  }

  void Transpose()
  {
    typedef math::tMatrix<2, 2, double> tMatrix;
    RRLIB_UNIT_TESTS_EQUALITY(tMatrix(1, 2, 3, 4).Transposed(), tMatrix(1, 3, 2, 4));
  }

  template <unsigned int Tdimension>
  void InversionOfHilbertMatrix()
  {
    tMatrix<Tdimension, Tdimension, double> hilbert_matrix;
    for (size_t row = 0; row < Tdimension; ++row)
    {
      for (size_t column = 0; column < Tdimension; ++column)
      {
        hilbert_matrix[row][column] = 1.0 / (row + column + 1);
      }
    }

    tMatrix<Tdimension, Tdimension, double> inverted_hilbert_matrix;
    for (size_t row = 0; row < Tdimension; ++row)
    {
      for (size_t column = 0; column < Tdimension; ++column)
      {
        inverted_hilbert_matrix[row][column] = std::pow(-1, row + column + 2) * (row + column + 1);
        inverted_hilbert_matrix[row][column] *= BinomialCoefficient(Tdimension + row, Tdimension - column - 1) * BinomialCoefficient(Tdimension + column, Tdimension - row - 1);
        inverted_hilbert_matrix[row][column] *= std::pow(BinomialCoefficient(row + column, row), 2);
      }
    }

    RRLIB_UNIT_TESTS_EQUALITY(inverted_hilbert_matrix, hilbert_matrix.Inverted());
  }

  void CholeskyDecomposition()
  {
    tMatrix<3, 3, double> A(1, 2, 3, 2, 5, 7, 3, 7, 26);
    tMatrix<3, 3, double> C(1, 0, 0, 2, 1, 0, 3, 1, 4);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("A = CC'", C * C.Transposed(), A);

    tCholeskyDecomposition<3, double> cholesky(A);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("chol(A).C = C", C, cholesky.C());

    tVector<3, double> x(3, 2, 1);
    RRLIB_UNIT_TESTS_EQUALITY(x, cholesky.Solve(tVector<3, double>(10, 23, 49)));

    A *= C;
    RRLIB_UNIT_TESTS_EXCEPTION_MESSAGE("A is not symmetrical, positive definite", (tCholeskyDecomposition<3, double>(A)), std::logic_error);
    RRLIB_UNIT_TESTS_EXCEPTION_MESSAGE("A is not symmetrical", (tCholeskyDecomposition<2, double>(tMatrix<2, 2, double>(1, 1, -1, 1))), std::logic_error);
  }

  void LUDecomposition()
  {
    tMatrix<3, 3, double> A(
      -5.0, -1.0,  2.0,
      -2.0,  6.0,  2.0,
      4.0,  2.0, -8.0
    );
    tLUDecomposition<3, double> decomposition(A);
    RRLIB_UNIT_TESTS_EQUALITY(A, decomposition.L() * decomposition.U());

    RRLIB_UNIT_TESTS_EXCEPTION_MESSAGE("Rank does not fit", (tLUDecomposition<2, double>(tMatrix<2, 2, double>(1.0, 1.0, 1.0, 1.0))), std::logic_error);

    RRLIB_UNIT_TESTS_EQUALITY((tVector<3, double>(5.0, 1.0, 3.0)), decomposition.Solve(tVector<3, double>(-20.0, 2.0, -2.0)));

    A.Set(
      9.0, 3.0,  1.0,
      4.0, 2.0,  1.0,
      1.0, 1.0, -1.0
    );
    RRLIB_UNIT_TESTS_EQUALITY((tVector<3, double>(-1.6, 9.0, 7.4)), (tLUDecomposition<3, double>(A).Solve(tVector<3, double>(20.0, 19.0, 0.0))));

    tMatrix<5, 5, double> B(
      2.0,  4.0,  2.0,  3.0,  5.0,
      -6.0,  4.0,  4.0, -2.0,  1.0,
      7.0, -3.0,  6.0,  1.0,  2.0,
      9.0, -1.0,  8.0, -4.0,  2.0,
      1.0,  6.0, 10.0,  7.0,  9.0
    );
    RRLIB_UNIT_TESTS_EQUALITY((tVector<5, double>(2.0, 4.0, 0.5, 3.0, -5.0)), (tLUDecomposition<5, double>(B).Solve(tVector<5, double>(5.0, -5.0, -2.0, -4.0, 7.0))));

    tMatrix<3, 2, double> C(
      2.0,  0.0,
      2.0,  0.0,
      2.0, -1.0
    );
    RRLIB_UNIT_TESTS_EQUALITY((tVector<2, double>(0.5, 1.0)), (tLUDecomposition<2, double>(C).Solve(tVector<3, double>(1.0, 1.0, 0.0))));

    C.Set(
      0.0,  0.0,
      0.0,  1.0,
      -1.0,  0.0
    );
    RRLIB_UNIT_TESTS_EQUALITY((tVector<2, double>(2.0, 1.0)), (tLUDecomposition<2, double>(C).Solve(tVector<3, double>(0.0, 1.0, -2.0))));
  }

#ifdef _LIB_OIV_PRESENT_
  void CoinConversions()
  {
    RRLIB_UNIT_TESTS_EQUALITY((tMatrix<4, 4, double>::Identity()), (rrlib::simvis3d::MatrixFromCoin<double>(SbMatrix::identity())));

    SbMatrix sb_matrix;
    sb_matrix.setRotate(SbRotation(SbVec3f(0, 1, 0), M_PI_2));
    RRLIB_UNIT_TESTS_ASSERT_MESSAGE("Assert internal representation of SbMatrix is transposed", IsEqual(sb_matrix[0][2], -1));

    tMatrix<4, 4, double> rrlib_matrix(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
    sb_matrix = rrlib::simvis3d::GetCoinMatrix(rrlib_matrix);
    RRLIB_UNIT_TESTS_ASSERT(sb_matrix.equals(SbMatrix(1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15, 4, 8, 12, 16), 1E-6));
    RRLIB_UNIT_TESTS_EQUALITY(rrlib_matrix, rrlib::simvis3d::MatrixFromCoin<double>(sb_matrix));
  }
#endif

};

RRLIB_UNIT_TESTS_REGISTER_SUITE(TestMatrices);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
