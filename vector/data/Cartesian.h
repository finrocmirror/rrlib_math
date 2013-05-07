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
/*!\file    rrlib/math/vector/data/Cartesian.h
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
#ifndef __rrlib__math__vector__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef __rrlib__math__vector__data__Cartesian_h__
#define __rrlib__math__vector__data__Cartesian_h__

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

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Cartesian()
  {};

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  TElement values[Tdimension];

  Cartesian(const Cartesian &other);
  Cartesian &operator = (const Cartesian &);

};

/*!
 *
 */
template <typename TElement>
class Cartesian<2, TElement>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline TElement X() const __attribute__((always_inline, flatten));

  inline TElement &X() __attribute__((always_inline, flatten));

  inline TElement Y() const __attribute__((always_inline, flatten));

  inline TElement &Y() __attribute__((always_inline, flatten));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Cartesian()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  TElement x;
  TElement y;

  Cartesian(const Cartesian &other);
  Cartesian &operator = (const Cartesian &);

};

/*!
 *
 */
template <typename TElement>
class Cartesian<3, TElement>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline TElement X() const __attribute__((always_inline, flatten));

  inline TElement &X() __attribute__((always_inline, flatten));

  inline TElement Y() const __attribute__((always_inline, flatten));

  inline TElement &Y() __attribute__((always_inline, flatten));

  inline TElement Z() const __attribute__((always_inline, flatten));

  inline TElement &Z() __attribute__((always_inline, flatten));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Cartesian()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  TElement x;
  TElement y;
  TElement z;

  Cartesian(const Cartesian &other);
  Cartesian &operator = (const Cartesian &);

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "rrlib/math/vector/data/Cartesian.hpp"

#endif
