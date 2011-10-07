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
/*!\file    Cartesian.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-08-25
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__vector__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef __rrlib__math__vector__data__Cartesian_hpp__
#define __rrlib__math__vector__data__Cartesian_hpp__

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
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace math
{
namespace vector
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Cartesian 2 X
//----------------------------------------------------------------------
template <typename TElement>
TElement Cartesian<2, TElement>::X() const
{
  return this->x;
}

template <typename TElement>
TElement &Cartesian<2, TElement>::X()
{
  return this->x;
}

//----------------------------------------------------------------------
// Cartesian 2 Y
//----------------------------------------------------------------------
template <typename TElement>
TElement Cartesian<2, TElement>::Y() const
{
  return this->y;
}

template <typename TElement>
TElement &Cartesian<2, TElement>::Y()
{
  return this->y;
}

//----------------------------------------------------------------------
// Cartesian 3 X
//----------------------------------------------------------------------
template <typename TElement>
TElement Cartesian<3, TElement>::X() const
{
  return this->x;
}

template <typename TElement>
TElement &Cartesian<3, TElement>::X()
{
  return this->x;
}

//----------------------------------------------------------------------
// Cartesian 3 Y
//----------------------------------------------------------------------
template <typename TElement>
TElement Cartesian<3, TElement>::Y() const
{
  return this->y;
}

template <typename TElement>
TElement &Cartesian<3, TElement>::Y()
{
  return this->y;
}

//----------------------------------------------------------------------
// Cartesian 3 Z
//----------------------------------------------------------------------
template <typename TElement>
TElement Cartesian<3, TElement>::Z() const
{
  return this->z;
}

template <typename TElement>
TElement &Cartesian<3, TElement>::Z()
{
  return this->z;
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
