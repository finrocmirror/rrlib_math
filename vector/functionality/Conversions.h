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
/*!\file    rrlib/math/vector/functionality/Conversions.h
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

#ifndef __rrlib__math__vector__functionality__Conversions_h__
#define __rrlib__math__vector__functionality__Conversions_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>

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
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
class Conversions
{

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Conversions() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);

};

/*!
 *
 */
template <typename TElement>
class Conversions<2, TElement, Cartesian>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TPolarUnitPolicy = angle::Radian, typename TPolarAutoWrapPolicy = angle::Signed>
  __attribute__((always_inline)) inline const tVector<2, TElement, Polar, TPolarUnitPolicy, TPolarAutoWrapPolicy> GetPolarVector() const
  {
    const tVector<2, TElement, Cartesian> *that = reinterpret_cast<const tVector<2, TElement, Cartesian> *>(this);
    return tVector<2, TElement, Polar, TPolarUnitPolicy, TPolarAutoWrapPolicy>(tAngle<TElement, angle::Radian>(std::atan2(that->Y(), that->X())), that->Length());
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Conversions() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);

};

/*!
 *
 */
template <typename TElement>
class Conversions<3, TElement, Cartesian>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TPolarUnitPolicy = angle::Radian, typename TPolarAutoWrapPolicy = angle::Signed>
  __attribute__((always_inline)) inline const tVector<3, TElement, Polar, TPolarUnitPolicy, TPolarAutoWrapPolicy> GetPolarVector() const
  {
    const tVector<3, TElement, Cartesian> *that = reinterpret_cast<const tVector<3, TElement, Cartesian> *>(this);
    TElement length = that->Length();
    return tVector<3, TElement, Polar, TPolarUnitPolicy, TPolarAutoWrapPolicy>(tAngle<TElement, angle::Radian>(std::atan2(that->Y(), that->X())), tAngle<TElement, angle::Radian>(std::acos(that->Z() / length)), length);
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Conversions() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);

};

/*!
 *
 */
template <typename TElement, typename ... TAdditionalDataParameters>
class Conversions<2, TElement, Polar, TAdditionalDataParameters...>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline const tVector<2, TElement, Cartesian> GetCartesianVector() const __attribute__((always_inline))
  {
    const tVector<2, TElement, Polar, TAdditionalDataParameters...> *that = reinterpret_cast<const tVector<2, TElement, Polar, TAdditionalDataParameters...> *>(this);
    return tVector<2, TElement, Cartesian>(that->Length() * that->Alpha().Cosine(), that->Length() * that->Alpha().Sine());
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Conversions() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);

};

/*!
 *
 */
template <typename TElement, typename ... TAdditionalDataParameters>
class Conversions<3, TElement, Polar, TAdditionalDataParameters...>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline const tVector<3, TElement, Cartesian> GetCartesianVector() const __attribute__((always_inline))
  {
    const tVector<3, TElement, Polar, TAdditionalDataParameters...> *that = reinterpret_cast<const tVector<3, TElement, Polar, TAdditionalDataParameters...> *>(this);
    TElement sin_alpha = that->Alpha().Sine();
    TElement cos_alpha = that->Alpha().Cosine();
    TElement sin_beta = that->Beta().Sine();
    TElement cos_beta = that->Beta().Cosine();
    return tVector<3, TElement, Cartesian>(that->Length() * cos_alpha * sin_beta, that->Length() * sin_alpha * sin_beta, that->Length() * cos_beta);
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Conversions() __attribute__((always_inline))
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
