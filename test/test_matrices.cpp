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
#include <cstdlib>
#include <iostream>
#include <cmath>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tMatrix.h"
#include "rrlib/math/tCholeskyDecomposition.h"

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

int main(int argc, char **argv)
{

  tMatrix<2, 2, double> a(1, 2, 3, 4);

  tMatrix<3, 3, double> b(1, 2, 3, 4, 5, 6, 7, 8, 9);
  b[0][0] = 5.5;
  b[1][1] = 6;
  b[2][2] = 7;

  tMatrix<3, 3, float> c(b);
  // c = b;

  std::cout << a << std::endl;
  std::cout << b << std::endl;
  std::cout << c << std::endl;

  std::cout << -a << std::endl;

  tMatrix<2, 2, double, matrix::LowerTriangle> l(1, 0, 3, 4);
  tMatrix<2, 2, double, matrix::UpperTriangle> u(1, 2, 0, 4);
  tMatrix<2, 2, double, matrix::Symmetrical> s(1, 2, 2, 5);

  tVector<2, double> v(1, 2);

#ifdef _LIB_OIV_PRESENT_
  std::cout << "=== Coin conversions ===" << std::endl;
  std::cout << "tMatrix converted from SbMatrix::identity(): " << tMatrix<4, 4, double>(SbMatrix::identity()) << std::endl;
  tMatrix<4, 4, double> matrix_4_4(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
  SbMatrix m(matrix_4_4.GetCoinMatrix());
  std::cout << "SbMatrix converted from tMatrix: " << matrix_4_4 << std::endl;
  m.print(stdout);
  std::cout << std::endl << "and back to tMatrix" << tMatrix<4, 4, double>(m) << std::endl;
#endif


  std::cout << "=== default ===" << std::endl;

  std::cout << "--- + ---" << std::endl;
  std::cout << a << " + " << a << " = " << a + a << std::endl;
  std::cout << a << " + " << l << " = " << a + l << std::endl;
  std::cout << a << " + " << u << " = " << a + u << std::endl;
  std::cout << a << " + " << s << " = " << a + s << std::endl;

  std::cout << "--- - ---" << std::endl;
  std::cout << a << " - " << a << " = " << a - a << std::endl;
  std::cout << a << " - " << l << " = " << a - l << std::endl;
  std::cout << a << " - " << u << " = " << a - u << std::endl;
  std::cout << a << " - " << s << " = " << a - s << std::endl;

  std::cout << "--- * ---" << std::endl;
  std::cout << a << " * " << a << " = " << a * a << std::endl;
  std::cout << a << " * " << l << " = " << a * l << std::endl;
  std::cout << a << " * " << u << " = " << a * u << std::endl;
  std::cout << a << " * " << s << " = " << a * s << std::endl;

  std::cout << "--- * vector ---" << std::endl;
  std::cout << a << " * " << v << " = " << a * v << std::endl;
  std::cout << v << " * " << a << " = " << v * a << std::endl;

  std::cout << "--- * scalar ---" << std::endl;
  std::cout << a << " * " << 3 << " = " << a * 3 << std::endl;
  std::cout << 3 << " * " << a << " = " << 3 * a << std::endl;

  std::cout << "=== lower ===" << std::endl;

  std::cout << "--- + ---" << std::endl;
  std::cout << l << " + " << a << " = " << l + a << std::endl;
  std::cout << l << " + " << l << " = " << l + l << std::endl;
  std::cout << l << " + " << u << " = " << l + u << std::endl;
  std::cout << l << " + " << s << " = " << l + s << std::endl;

  std::cout << "--- - ---" << std::endl;
  std::cout << l << " - " << a << " = " << l - a << std::endl;
  std::cout << l << " - " << l << " = " << l - l << std::endl;
  std::cout << l << " - " << u << " = " << l - u << std::endl;
  std::cout << l << " - " << s << " = " << l - s << std::endl;

  std::cout << "--- * ---" << std::endl;
  std::cout << l << " * " << a << " = " << l * a << std::endl;
  std::cout << l << " * " << l << " = " << l * l << std::endl;
  std::cout << l << " * " << u << " = " << l * u << std::endl;
  std::cout << l << " * " << s << " = " << l * s << std::endl;

  std::cout << "--- * vector ---" << std::endl;
  std::cout << l << " * " << v << " = " << l * v << std::endl;
  std::cout << v << " * " << l << " = " << v * l << std::endl;

  std::cout << "--- * scalar ---" << std::endl;
  std::cout << l << " * " << 3 << " = " << l * 3 << std::endl;
  std::cout << 3 << " * " << l << " = " << 3 * l << std::endl;

  std::cout << "=== upper ===" << std::endl;

  std::cout << "--- + ---" << std::endl;
  std::cout << u << " + " << a << " = " << u + a << std::endl;
  std::cout << u << " + " << l << " = " << u + l << std::endl;
  std::cout << u << " + " << u << " = " << u + u << std::endl;
  std::cout << u << " + " << s << " = " << u + s << std::endl;

  std::cout << "--- - ---" << std::endl;
  std::cout << u << " - " << a << " = " << u - a << std::endl;
  std::cout << u << " - " << l << " = " << u - l << std::endl;
  std::cout << u << " - " << u << " = " << u - u << std::endl;
  std::cout << u << " - " << s << " = " << u - s << std::endl;

  std::cout << "--- * ---" << std::endl;
  std::cout << u << " * " << a << " = " << u * a << std::endl;
  std::cout << u << " * " << l << " = " << u * l << std::endl;
  std::cout << u << " * " << u << " = " << u * u << std::endl;
  std::cout << u << " * " << s << " = " << u * s << std::endl;

  std::cout << "--- * vector ---" << std::endl;
  std::cout << u << " * " << v << " = " << u * v << std::endl;
  std::cout << v << " * " << u << " = " << v * u << std::endl;

  std::cout << "--- * scalar ---" << std::endl;
  std::cout << u << " * " << 3 << " = " << u * 3 << std::endl;
  std::cout << 3 << " * " << u << " = " << 3 * u << std::endl;

  std::cout << "=== symmetrical ===" << std::endl;

  std::cout << "--- + ---" << std::endl;
  std::cout << s << " + " << a << " = " << s + a << std::endl;
  std::cout << s << " + " << l << " = " << s + l << std::endl;
  std::cout << s << " + " << u << " = " << s + u << std::endl;
  std::cout << s << " + " << s << " = " << s + s << std::endl;

  std::cout << "--- - ---" << std::endl;
  std::cout << s << " - " << a << " = " << s - a << std::endl;
  std::cout << s << " - " << l << " = " << s - l << std::endl;
  std::cout << s << " - " << u << " = " << s - u << std::endl;
  std::cout << s << " - " << s << " = " << s - s << std::endl;

  std::cout << "--- * ---" << std::endl;
  std::cout << s << " * " << a << " = " << s * a << std::endl;
  std::cout << s << " * " << l << " = " << s * l << std::endl;
  std::cout << s << " * " << u << " = " << s * u << std::endl;
  std::cout << s << " * " << s << " = " << s * s << std::endl;

  std::cout << "--- * vector ---" << std::endl;
  std::cout << s << " * " << v << " = " << s * v << std::endl;
  std::cout << v << " * " << s << " = " << v * s << std::endl;

  std::cout << "--- * scalar ---" << std::endl;
  std::cout << s << " * " << 3 << " = " << s * 3 << std::endl;
  std::cout << 3 << " * " << s << " = " << 3 * s << std::endl;

  std::cout << "=== transpose ===" << std::endl;

  std::cout << a << " -> " << a.Transposed() << std::endl;
  std::cout << l << " -> " << l.Transposed() << std::endl;
  std::cout << u << " -> " << u.Transposed() << std::endl;
  std::cout << s << " -> " << s.Transposed() << std::endl;

  a.Transpose();
//  l.Transpose();
//  u.Transpose();
  s.Transpose();

  std::cout << a << std::endl;
  std::cout << l << std::endl;
  std::cout << u << std::endl;
  std::cout << s << std::endl;

  std::cout << a << " -> " << a.Inverse() << std::endl;
  std::cout << l << " -> " << l.Inverse() << std::endl;
  std::cout << u << " -> " << u.Inverse() << std::endl;
  std::cout << s << " -> " << s.Inverse() << std::endl;

  a.Invert();
  l.Invert();
  u.Invert();
  s.Invert();

  std::cout << a << std::endl;
  std::cout << l << std::endl;
  std::cout << u << std::endl;
  std::cout << s << std::endl;

  // test matrix inversion with hilbert-matrix
  typedef tMatrix<5, 5, double, matrix::Full> tInversionTest;
  tInversionTest hilbert_matrix;
  for (size_t row = 0; row < tInversionTest::cROWS; ++row)
  {
    for (size_t column = 0; column < tInversionTest::cCOLUMNS; ++column)
    {
      hilbert_matrix[row][column] = 1.0 / (1.0 + row + column);
    }
  }
  std::cout << hilbert_matrix << " -> " << hilbert_matrix.Inverted() << std::endl;


  tMatrix<7, 3, double> C
  (
    1, 0.04, 0.0016,
    1, 0.32, 0.1024,
    1, 0.51, 0.2601,
    1, 0.73, 0.5329,
    1, 1.03, 1.0609,
    1, 1.42, 2.0164,
    1, 1.60, 2.5600
  );

  tMatrix<3, 3, double, matrix::Symmetrical> A = (tMatrix<3, 3, double, matrix::Symmetrical>)
      (C.Transposed() * C);
  std::cout << "A:" << A << std::endl;

  tVector<7, double> d(-2.63, -1.18, -1.16, -1.54, -2.65, -5.41, -7.67);
  tVector<3, double> _b = C.Transposed() * d;
  std::cout << "b:" << _b << std::endl;

  tMatrix<3, 3, double, matrix::LowerTriangle> L;
  tCholeskyDecomposition<3, double> chol(A);
  L = chol.C();
  std::cout << "----------------------------------------" << std::endl
            << "L:" << L << std::endl;
  tVector<3, double> x = chol.Solve(-_b);
  std::cout << "x:" << x << std::endl;


  std::cout << "OK" << std::endl;

  return EXIT_SUCCESS;
}
