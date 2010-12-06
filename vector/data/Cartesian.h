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
/*!\file    Cartesian.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-09-26
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_vector_include_guard_
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef _rrlib_math_vector_data_Cartesian_h_
#define _rrlib_math_vector_data_Cartesian_h_

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
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
template <size_t Tdimension, typename TElement>
class Cartesian
{
  TElement values[Tdimension];

  Cartesian(const Cartesian &other);
  Cartesian &operator = (const Cartesian &);

protected:

  inline Cartesian() {};

};

/*!
 *
 */
template <typename TElement>
class Cartesian<2, TElement>
{
  TElement x;
  TElement y;

  Cartesian(const Cartesian &other);
  Cartesian &operator = (const Cartesian &);

protected:

  inline Cartesian() {}

public:

  inline TElement X() const
  {
    return this->x;
  }
  inline TElement &X()
  {
    return this->x;
  }
  inline TElement Y() const
  {
    return this->y;
  }
  inline TElement &Y()
  {
    return this->y;
  }

};

/*!
 *
 */
template <typename TElement>
class Cartesian<3, TElement>
{
  TElement x;
  TElement y;
  TElement z;

  Cartesian(const Cartesian &other);
  Cartesian &operator = (const Cartesian &);

protected:

  inline Cartesian() {}

public:

  inline TElement X() const
  {
    return this->x;
  }
  inline TElement &X()
  {
    return this->x;
  }
  inline TElement Y() const
  {
    return this->y;
  }
  inline TElement &Y()
  {
    return this->y;
  }
  inline TElement Z() const
  {
    return this->z;
  }
  inline TElement &Z()
  {
    return this->z;
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
