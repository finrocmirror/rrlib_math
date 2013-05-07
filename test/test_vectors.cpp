//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    rrlib/math/test/test_vectors.cpp
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

void f(const tVec2d &v)
{
  std::cout << "f(): v=" << v << std::endl;
}

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
  std::cout << 3.0 * (polar1 + polar2) << std::endl;

  std::cout << a.Length() << ", " << a.SquaredLength() << std::endl;
  std::cout << b.Length() << ", " << b.SquaredLength() << std::endl;

  std::cout << tVector<2, double>(1, 0).Normalized() << std::endl;
  std::cout << tVector<2, double>(0, 1).Normalized() << std::endl;
  std::cout << tVector<2, double>(0, 2).Normalized() << std::endl;
  std::cout << tVector<2, double>(2, 0).Normalized() << std::endl;
  std::cout << tVector<2, double>(2, 2).Normalized() << std::endl;

#ifdef _LIB_OIV_PRESENT_
  std::cout << "Coin conversions" << std::endl;
  std::cout << tVector<3, double> (SbVec3f(1., 2., 3.)) << std::endl;
  std::cout << tVector<2, float>(SbVec2f(4., 5.)) << std::endl;

  SbVec3f sb_vec3f(b.GetCoinVector());
  std::cout << "sb_vec3f: ";
  sb_vec3f.print(stdout);
  std::cout << std::endl;
  assert(sb_vec3f[0] == b[0] && sb_vec3f[1] == b[1] && sb_vec3f[2] == b[2]);
#endif

  tVector<2, double> v(0., -2.);
  std::cout << v.GetPolarVector().Alpha() << ", " << tAngle<double, angle::Radian, angle::Unsigned>(v.GetPolarVector().Alpha()) << std::endl;

  tVec3d vec_3d(1., 2., 3.);
  f(vec_3d);

  std::cout << "OK" << std::endl;


  rrlib::math::tVector<3, float> bad(1, 1, 1);
  std::cout << "Bad: " << bad << " => " << bad.GetPolarVector() << " Alpha = " << bad.GetPolarVector().Alpha() << ", Beta = " << bad.GetPolarVector().Beta() << std::endl;

  rrlib::math::tVector<3, float, rrlib::math::vector::Cartesian> worse(1, 1, 1);
  std::cout << "worse: " << worse << " => " << worse.GetPolarVector() << " Alpha = " << worse.GetPolarVector().Alpha() << ", Beta = " << worse.GetPolarVector().Beta()  << std::endl;


  return EXIT_SUCCESS;
}
