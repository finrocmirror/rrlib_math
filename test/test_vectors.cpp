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
/*!\file    test_vectors.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2008-09-26
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

int main(int argc, char **argv)
{

  tVector<2, double> a(1, 2);

  tVector<3, double> b;
  b[0] = 5.5;
  b[1] = 6;
  b[2] = 7;

  tVector<2, double> c(b);
//  c = b;

  b = tVector<3, double>(-a);

  a = a + c + a;

  std::cout << a << std::endl;
  std::cout << b << std::endl;
  std::cout << c << std::endl;

  std::cout << -a << std::endl;

  std::cout << a + a << std::endl;
  std::cout << a + a + a << std::endl;

  std::cout << c - a << std::endl;

  std::cout << a * 3 << std::endl;
  std::cout << 3 * a << std::endl;

  std::cout << a * a << std::endl;
  std::cout << a *(a + c) << std::endl;
  std::cout << a * 2 << std::endl;
  std::cout << 2 * a << std::endl;
  std::cout << a * c << std::endl;


  std::cout << tVector<2, double>(1, 0) * tVector<2, float>(0, 1) << std::endl;

  std::cout << "polar operations" << std::endl;

  tVector<2, double, vector::Polar> polar1(0.0, 1.0);
  tVector<2, double, vector::Polar> polar2(M_PI_2, 1.0);

  std::cout << -polar1 << std::endl;
  std::cout << polar1 + polar2 << std::endl;
  std::cout << (polar1 + polar2) * 3.0 << std::endl;
  std::cout << 3.0 *(polar1 + polar2) << std::endl;

  std::cout << a.Length() << ", " << a.SquaredLength() << std::endl;
  std::cout << b.Length() << ", " << b.SquaredLength() << std::endl;

  std::cout << tVector<2, double>(1, 0).Normalized() << std::endl;
  std::cout << tVector<2, double>(0, 1).Normalized() << std::endl;
  std::cout << tVector<2, double>(0, 2).Normalized() << std::endl;
  std::cout << tVector<2, double>(2, 0).Normalized() << std::endl;
  std::cout << tVector<2, double>(2, 2).Normalized() << std::endl;


  std::cout << "OK" << std::endl;

  return EXIT_SUCCESS;
}
