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
/*!\file    rrlib/math/vector/data/Polar.h
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


template <size_t Tdimension, typename TElement, typename TUnitPolicy = angle::Radian, typename TAutoWrapPolicy = angle::Signed>
class Polar;
//{
//
////----------------------------------------------------------------------
//// Public methods and typedefs
////----------------------------------------------------------------------
//public:
//
//  typedef math::tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> tAngle;
//
//  inline TElement &Length() const __attribute__((always_inline));
//
//  inline TElement &Length() __attribute__((always_inline));
//
////----------------------------------------------------------------------
//// Protected methods
////----------------------------------------------------------------------
//protected:
//
//  inline Polar()
//  {}
//
////----------------------------------------------------------------------
//// Private fields and methods
////----------------------------------------------------------------------
//private:
//
//  tAngle angles[Tdimension - 1];
//  TElement length;
//
//  Polar(const Polar &);
//  Polar &operator = (const Polar &);
//
//};

/*!
 *
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
class Polar<2, TElement, TUnitPolicy, TAutoWrapPolicy>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef math::tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> tAngle;

  inline const tAngle &Alpha() const __attribute__((always_inline));

  inline tAngle &Alpha() __attribute__((always_inline));

  inline const TElement &Length() const __attribute__((always_inline));

  inline TElement &Length() __attribute__((always_inline));

  template <typename TAlpha, typename TLength>
  inline void Set(TAlpha alpha, TLength length);

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

  tAngle alpha;
  TElement length;

  Polar(const Polar &);
  Polar &operator = (const Polar &);

};

/*!
 *
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
class Polar<3, TElement, TUnitPolicy, TAutoWrapPolicy>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef math::tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> tAngle;

  inline const tAngle &Alpha() const __attribute__((always_inline));

  inline tAngle &Alpha() __attribute__((always_inline));

  inline const tAngle &Beta() const __attribute__((always_inline));

  inline tAngle &Beta() __attribute__((always_inline));

  inline const TElement &Length() const __attribute__((always_inline));

  inline TElement &Length() __attribute__((always_inline));

  template <typename TAlpha, typename TBeta, typename TLength>
  inline void Set(TAlpha alpha, TBeta beta, TLength length);

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

  tAngle alpha;
  tAngle beta;
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
