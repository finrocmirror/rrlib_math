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
/*!\file    Polar.h
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

#ifndef __rrlib__math__vector__data__Polar_h__
#define __rrlib__math__vector__data__Polar_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tAngle.h"

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
class Polar
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline TElement &Length() const __attribute__((always_inline,flatten));

  inline TElement &Length() __attribute__((always_inline,flatten));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Polar()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tAngle<TElement, angle::Radian, angle::Signed> angles[Tdimension - 1];
  TElement length;

  Polar(const Polar &);
  Polar &operator = (const Polar &);

};

/*!
 *
 */
template <typename TElement>
class Polar<2, TElement>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline tAngle<TElement, angle::Radian, angle::Signed> Alpha() const __attribute__((always_inline,flatten));

  inline tAngle<TElement, angle::Radian, angle::Signed> &Alpha() __attribute__((always_inline,flatten));

  inline const TElement &Length() const __attribute__((always_inline,flatten));

  inline TElement &Length() __attribute__((always_inline,flatten));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Polar()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tAngle<TElement, angle::Radian, angle::Signed> alpha;
  TElement length;

  Polar(const Polar &);
  Polar &operator = (const Polar &);

};

/*!
 *
 */
template <typename TElement>
class Polar<3, TElement>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline tAngle<TElement, angle::Radian, angle::Signed> Alpha() const __attribute__((always_inline,flatten));

  inline tAngle<TElement, angle::Radian, angle::Signed> &Alpha() __attribute__((always_inline,flatten));

  inline tAngle<TElement, angle::Radian, angle::Signed> Beta() const __attribute__((always_inline,flatten));

  inline tAngle<TElement, angle::Radian, angle::Signed> &Beta() __attribute__((always_inline,flatten));

  inline const TElement &Length() const __attribute__((always_inline,flatten));

  inline TElement &Length() __attribute__((always_inline,flatten));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Polar()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tAngle<TElement, angle::Radian, angle::Signed> alpha;
  tAngle<TElement, angle::Radian, angle::Signed> beta;
  TElement length;

  Polar(const Polar &);
  Polar &operator = (const Polar &);

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "rrlib/math/vector/data/Polar.hpp"

#endif
