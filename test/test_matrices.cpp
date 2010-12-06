//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    test_matrices.cpp
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

  std::cout << "=== default ===" << std::endl;

  std::cout << "--- + ---" << std::endl;
  std::cout << a + a << std::endl;
  std::cout << a + l << std::endl;
  std::cout << a + u << std::endl;
  std::cout << a + s << std::endl;

  std::cout << "--- - ---" << std::endl;
  std::cout << a - a << std::endl;
  std::cout << a - l << std::endl;
  std::cout << a - u << std::endl;
  std::cout << a - s << std::endl;

  std::cout << "--- * ---" << std::endl;
  std::cout << a * a << std::endl;
  std::cout << a * l << std::endl;
  std::cout << a * u << std::endl;
  std::cout << a * s << std::endl;

  std::cout << "--- * vector ---" << std::endl;
  std::cout << a * v << std::endl;
  std::cout << v * a << std::endl;

  std::cout << "--- * scalar ---" << std::endl;
  std::cout << a * 3 << std::endl;
  std::cout << 3 * a << std::endl;

  std::cout << "=== lower ===" << std::endl;

  std::cout << "--- + ---" << std::endl;
  std::cout << l + a << std::endl;
  std::cout << l + l << std::endl;
  std::cout << l + u << std::endl;
  std::cout << l + s << std::endl;

  std::cout << "--- - ---" << std::endl;
  std::cout << l - a << std::endl;
  std::cout << l - l << std::endl;
  std::cout << l - u << std::endl;
  std::cout << l - s << std::endl;

  std::cout << "--- * ---" << std::endl;
  std::cout << l * a << std::endl;
  std::cout << l * l << std::endl;
  std::cout << l * u << std::endl;
  std::cout << l * s << std::endl;

  std::cout << "--- * vector ---" << std::endl;
  std::cout << l * v << std::endl;
  std::cout << v * l << std::endl;

  std::cout << "--- * scalar ---" << std::endl;
  std::cout << l * 3 << std::endl;
  std::cout << 3 * l << std::endl;

  std::cout << "=== upper ===" << std::endl;

  std::cout << "--- + ---" << std::endl;
  std::cout << u + a << std::endl;
  std::cout << u + l << std::endl;
  std::cout << u + u << std::endl;
  std::cout << u + s << std::endl;

  std::cout << "--- - ---" << std::endl;
  std::cout << u - a << std::endl;
  std::cout << u - l << std::endl;
  std::cout << u - u << std::endl;
  std::cout << u - s << std::endl;

  std::cout << "--- * ---" << std::endl;
  std::cout << u * a << std::endl;
  std::cout << u * l << std::endl;
  std::cout << u * u << std::endl;
  std::cout << u * s << std::endl;

  std::cout << "--- * vector ---" << std::endl;
  std::cout << u * v << std::endl;
  std::cout << v * u << std::endl;

  std::cout << "--- * scalar ---" << std::endl;
  std::cout << u * 3 << std::endl;
  std::cout << 3 * u << std::endl;

  std::cout << "=== symmetrical ===" << std::endl;

  std::cout << "--- + ---" << std::endl;
  std::cout << s + a << std::endl;
  std::cout << s + l << std::endl;
  std::cout << s + u << std::endl;
  std::cout << s + s << std::endl;

  std::cout << "--- - ---" << std::endl;
  std::cout << s - a << std::endl;
  std::cout << s - l << std::endl;
  std::cout << s - u << std::endl;
  std::cout << s - s << std::endl;

  std::cout << "--- * ---" << std::endl;
  std::cout << s * a << std::endl;
  std::cout << s * l << std::endl;
  std::cout << s * u << std::endl;
  std::cout << s * s << std::endl;

  std::cout << "--- * vector ---" << std::endl;
  std::cout << s * v << std::endl;
  std::cout << v * s << std::endl;

  std::cout << "--- * scalar ---" << std::endl;
  std::cout << s * 3 << std::endl;
  std::cout << 3 * s << std::endl;

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
  typedef tMatrix<5, 5, double, matrix::Full> tInversionTestType;
  tInversionTestType hilbert_matrix;
  for (size_t row = 0; row < tInversionTestType::cROWS; ++row)
  {
    for (size_t column = 0; column < tInversionTestType::cCOLUMNS; ++column)
    {
      hilbert_matrix[row][column] = 1.0 / (1.0 + row + column);
    }
  }
  std::cout << hilbert_matrix << " -> " << hilbert_matrix.Inverted() << std::endl;








  std::cout << "OK" << std::endl;

  return EXIT_SUCCESS;
}
