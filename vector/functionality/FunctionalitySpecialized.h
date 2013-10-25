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
/*!\file    rrlib/math/vector/functionality/FunctionalitySpecialized.h
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

#ifndef __rrlib__math__vector__functionality__FunctionalitySpecialized_h__
#define __rrlib__math__vector__functionality__FunctionalitySpecialized_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/utility/enable_if.hpp>

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
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
class FunctionalitySpecialized
{
  typedef math::tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> tVector;

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline FunctionalitySpecialized()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

};

/*!
 *
 */
template <size_t Tdimension, typename TElement>
class FunctionalitySpecialized<Tdimension, TElement, Cartesian>
{
  typedef math::tVector<Tdimension, TElement, Cartesian> tVector;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline const TElement operator [](size_t i) const __attribute__((always_inline, flatten));
  inline TElement &operator [](size_t i) __attribute__((always_inline, flatten));

  template <typename ... TValues>
  inline void Set(TValues... values) __attribute__((always_inline, flatten));

  static inline const tVector Direction(size_t i) __attribute__((always_inline, flatten));

  inline const TElement Length() const __attribute__((always_inline, flatten));

  inline const TElement SquaredLength() const __attribute__((always_inline, flatten));

  inline const bool IsZero(double epsilon = 0) const __attribute__((always_inline, flatten));

  template <typename TOtherElement>
  inline void SchurMultiply(const math::tVector<Tdimension, TOtherElement, Cartesian> &other) __attribute__((always_inline, flatten));

  template <typename TOtherElement>
  inline const math::tVector < Tdimension, decltype(TElement() + TOtherElement()), Cartesian > SchurMultiplied(const math::tVector<Tdimension, TOtherElement, Cartesian> &other) const __attribute__((always_inline, flatten));

  template <size_t Tother_dimension, typename TOtherElement>
  inline typename boost::enable_if_c < Tdimension == 3 && Tother_dimension == 3, void >::type CrossMultiply(const math::tVector<Tother_dimension, TOtherElement, Cartesian> &other) __attribute__((always_inline, flatten));

  template <size_t Tother_dimension, typename TOtherElement>
  inline const typename boost::enable_if_c < Tdimension == 3 && Tother_dimension == 3, math::tVector < 3, decltype(TElement() + TOtherElement()), Cartesian > >::type CrossMultiplied(const math::tVector<Tother_dimension, TOtherElement, Cartesian> &other) const __attribute__((always_inline, flatten));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline FunctionalitySpecialized()
  {}

  explicit inline FunctionalitySpecialized(const TElement data[Tdimension]) __attribute__((always_inline, flatten));

  template <typename TOtherElement>
  explicit inline FunctionalitySpecialized(const TOtherElement data[Tdimension]) __attribute__((always_inline, flatten));

  template <size_t Tother_dimension, typename TOtherElement>
  explicit inline FunctionalitySpecialized(const math::tVector<Tother_dimension, TOtherElement, Cartesian> &other) __attribute__((always_inline, flatten));

  template <typename ... TValues>
  explicit inline FunctionalitySpecialized(TElement value, TValues... values) __attribute__((always_inline, flatten));

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

};

/*!
 *
 */
template <size_t Tdimension, typename TElement, typename ... TAdditionalDataParameters>
class FunctionalitySpecialized<Tdimension, TElement, Polar, TAdditionalDataParameters...>
{
  typedef math::tVector<Tdimension, TElement, Polar, TAdditionalDataParameters...> tVector;
  typedef typename Polar<Tdimension, TElement, TAdditionalDataParameters...>::tAngle tAngle;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline const tAngle operator [](size_t i) const __attribute__((always_inline, flatten));
  inline tAngle &operator [](size_t i) __attribute__((always_inline, flatten));

  template <typename ... TValues>
  inline void Set(TValues... values) __attribute__((always_inline, flatten));

  inline const TElement SquaredLength() const __attribute__((always_inline, flatten));

  inline const bool IsZero(double epsilon = 0) const __attribute__((always_inline, flatten));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline FunctionalitySpecialized()
  {}

  template <typename TOtherElement>
  explicit inline FunctionalitySpecialized(const tAngle angles[Tdimension - 1], TOtherElement length) __attribute__((always_inline, flatten));

  template <size_t Tother_dimension, typename TOtherElement>
  explicit inline FunctionalitySpecialized(const math::tVector<Tother_dimension, TOtherElement, Polar, TAdditionalDataParameters...> &other) __attribute__((always_inline, flatten));

  template <typename TOtherUnitPolicy, typename TOtherSignPolicy>
  explicit inline FunctionalitySpecialized(const math::tVector<Tdimension, TElement, Polar, TOtherUnitPolicy, TOtherSignPolicy> &other) __attribute__((always_inline, flatten));

  template <typename ... TValues>
  explicit inline FunctionalitySpecialized(tAngle value, TValues... values) __attribute__((always_inline, flatten));

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

  template <size_t number_of_given_values>
  inline void SetValues(tAngle buffer[Tdimension - 1], TElement length)
  {
    static_assert(number_of_given_values == Tdimension - 1, "Wrong number of values given to store in vector");
    tVector *that = reinterpret_cast<tVector *>(this);
    std::memcpy(this, buffer, sizeof(tVector));
    that->Length() = length;
  }
  template <size_t number_of_given_values, typename ... TValues>
  inline void SetValues(tAngle buffer[Tdimension - 1], tAngle value, TValues... values)
  {
    buffer[number_of_given_values] = value;
    this->SetValues < number_of_given_values + 1 > (buffer, values...);
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "rrlib/math/vector/functionality/FunctionalitySpecialized.hpp"

#endif
