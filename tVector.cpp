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
/*!\file    rrlib/math/tVector.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-09-27
 *
 */
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

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

template class tVector<2, double, vector::Cartesian>;
template class tVector<3, double, vector::Cartesian>;
template class tVector<6, double, vector::Cartesian>;

template class tVector<2, float, vector::Cartesian>;
template class tVector<3, float, vector::Cartesian>;
template class tVector<6, float, vector::Cartesian>;

template class tVector<2, int, vector::Cartesian>;
template class tVector<3, int, vector::Cartesian>;
template class tVector<6, int, vector::Cartesian>;

template class tVector<2, unsigned int, vector::Cartesian>;
template class tVector<3, unsigned int, vector::Cartesian>;
template class tVector<6, unsigned int, vector::Cartesian>;

template class tVector<2, double, vector::Polar>;
template class tVector<3, double, vector::Polar>;

template class tVector<2, float, vector::Polar>;
template class tVector<3, float, vector::Polar>;
//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
