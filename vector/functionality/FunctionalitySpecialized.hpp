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
/*!\file    FunctionalitySpecialized.hpp
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

#ifndef __rrlib__math__vector__functionality__FunctionalitySpecialized_hpp__
#define __rrlib__math__vector__functionality__FunctionalitySpecialized_hpp__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>
#include <boost/utility/enable_if.hpp>
#include <iostream>

#include "rrlib/util/variadic_templates.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/utilities.h"
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
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// FunctionalitySpecialized Cartesian constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
FunctionalitySpecialized<Tdimension, TElement, Cartesian>::FunctionalitySpecialized(const TElement data[Tdimension])
{
  std::memcpy(this, data, sizeof(tVector));
}

template <size_t Tdimension, typename TElement>
template <typename TOtherElement>
FunctionalitySpecialized<Tdimension, TElement, Cartesian>::FunctionalitySpecialized(const TOtherElement data[Tdimension])
{
  for (size_t i = 0; i < Tdimension; ++i)
  {
    (*this)[i] = data[i];
  }
}

template <size_t Tdimension, typename TElement>
template <size_t Tother_dimension, typename TOtherElement>
FunctionalitySpecialized<Tdimension, TElement, Cartesian>::FunctionalitySpecialized(const math::tVector<Tother_dimension, TOtherElement, Cartesian> &other)
{
  std::memset(this, 0, sizeof(tVector));
  size_t size = std::min(Tdimension, Tother_dimension);
  for (size_t i = 0; i < size; ++i)
  {
    reinterpret_cast<TElement *>(this)[i] = reinterpret_cast<const TOtherElement *>(&other)[i];
  }
}

template <size_t Tdimension, typename TElement>
template <typename ... TValues>
FunctionalitySpecialized<Tdimension, TElement, Cartesian>::FunctionalitySpecialized(TElement value, TValues... values)
{
  FunctionalitySpecialized::Set(value, values...);
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Cartesian operator []
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const TElement FunctionalitySpecialized<Tdimension, TElement, Cartesian>::operator [](size_t i) const
{
  return const_cast<FunctionalitySpecialized &>(*this)[i];
}

template <size_t Tdimension, typename TElement>
TElement &FunctionalitySpecialized<Tdimension, TElement, Cartesian>::operator [](size_t i)
{
  if (i > Tdimension - 1)
  {
    std::stringstream stream;
    stream << "Vector index (" << i << ") out of bounds [0.." << Tdimension - 1 << "].";
    throw std::logic_error(stream.str());
  }
  return reinterpret_cast<TElement *>(this)[i];
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Cartesian Set
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <typename ... TValues>
void FunctionalitySpecialized<Tdimension, TElement, Cartesian>::Set(TValues... values)
{
  static_assert(sizeof...(values) == Tdimension, "Wrong number of values given to store in vector");

  TElement *p = reinterpret_cast<TElement *>(this);
  util::ProcessVariadicValues<TElement>([p](TElement x) mutable { *p = x; ++p; }, values...);
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Cartesian Direction
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const tVector<Tdimension, TElement, Cartesian> FunctionalitySpecialized<Tdimension, TElement, Cartesian>::Direction(size_t i)
{
  tVector vector;
  vector[i] = 1;
  return vector;
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Cartesian Length
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const TElement FunctionalitySpecialized<Tdimension, TElement, Cartesian>::Length() const
{
  return std::sqrt(this->SquaredLength());
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Cartesian SquaredLength
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const TElement FunctionalitySpecialized<Tdimension, TElement, Cartesian>::SquaredLength() const
{
  const tVector *that = reinterpret_cast<const tVector *>(this);
  TElement result = 0;
  for (size_t i = 0; i < Tdimension; ++i)
  {
    result += (*that)[i] * (*that)[i];
  }
  return result;
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Cartesian IsZero
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const bool FunctionalitySpecialized<Tdimension, TElement, Cartesian>::IsZero(double epsilon) const
{
  const tVector *that = reinterpret_cast<const tVector *>(this);
  for (size_t i = 0; i < Tdimension; ++i)
  {
    if (!IsEqual((*that)[i], 0, epsilon, eFCM_ABSOLUTE_ERROR))
    {
      return false;
    }
  }
  return true;
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Cartesian SchurMultiply
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <typename TOtherElement>
void FunctionalitySpecialized<Tdimension, TElement, Cartesian>::SchurMultiply(const math::tVector<Tdimension, TOtherElement, Cartesian> &other)
{
  tVector *that = reinterpret_cast<tVector *>(this);
  for (size_t i = 0; i < Tdimension; ++i)
  {
    (*that)[i] *= other[i];
  }
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Cartesian SchurMultiplied
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <typename TOtherElement>
const math::tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, Cartesian> FunctionalitySpecialized<Tdimension, TElement, Cartesian>::SchurMultiplied(const math::tVector<Tdimension, TOtherElement, Cartesian> &other) const
{
  const tVector *that = reinterpret_cast<const tVector *>(this);
  math::tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, Cartesian> temp(*that);
  temp.SchurMultiply(other);
  return temp;
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Cartesian CrossMultiply
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <size_t Tother_dimension, typename TOtherElement>
typename boost::enable_if_c < Tdimension == 3 && Tother_dimension == 3, void >::type FunctionalitySpecialized<Tdimension, TElement, Cartesian>::CrossMultiply(const math::tVector<Tother_dimension, TOtherElement, Cartesian> &other)
{
  tVector *that = reinterpret_cast<tVector *>(this);
  that->Set(that->Y() * other.Z() - that->Z() * other.Y(),
            that->Z() * other.X() - that->X() * other.Z(),
            that->X() * other.Y() - that->Y() * other.X());
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Cartesian CrossMultiplied
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <size_t Tother_dimension, typename TOtherElement>
const typename boost::enable_if_c < Tdimension == 3 && Tother_dimension == 3, math::tVector<3, typename until_0x::Auto<TElement, TOtherElement>::type, Cartesian> >::type FunctionalitySpecialized<Tdimension, TElement, Cartesian>::CrossMultiplied(const math::tVector<Tother_dimension, TOtherElement, Cartesian> &other) const
{
  const tVector *that = reinterpret_cast<const tVector *>(this);
  math::tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, Cartesian> temp(*that);
  temp.CrossMultiply(other);
  return temp;
}



//----------------------------------------------------------------------
// FunctionalitySpecialized Polar constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <typename TOtherElement>
FunctionalitySpecialized<Tdimension, TElement, Polar>::FunctionalitySpecialized(const tAngleRad angles[Tdimension - 1], TOtherElement length)
{
  tVector *that = reinterpret_cast<tVector *>(this);
  std::memcpy(this, angles, sizeof(tVector) - sizeof(TElement));
  that->Length() = length;
}

template <size_t Tdimension, typename TElement>
template <size_t Tother_dimension, typename TOtherElement>
FunctionalitySpecialized<Tdimension, TElement, Polar>::FunctionalitySpecialized(const math::tVector<Tother_dimension, TOtherElement, Polar> &other)
{
  tVector *that = reinterpret_cast<tVector *>(this);
  std::memset(this, 0, sizeof(tVector));
  size_t size = std::min(Tdimension - 1, Tother_dimension - 1);
  for (size_t i = 0; i < size; ++i)
  {
    (*that)[i] = other[i];
  }
  that->Length() = other.Length();
}

template <size_t Tdimension, typename TElement>
template <typename ... TValues>
FunctionalitySpecialized<Tdimension, TElement, Polar>::FunctionalitySpecialized(tAngleRad value, TValues... values)
{
  FunctionalitySpecialized::Set(value, values...);
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Polar operator []
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const tAngleRad FunctionalitySpecialized<Tdimension, TElement, Polar>::operator [](size_t i) const
{
  return const_cast<FunctionalitySpecialized &>(*this)[i];
}

template <size_t Tdimension, typename TElement>
tAngleRad &FunctionalitySpecialized<Tdimension, TElement, Polar>::operator [](size_t i)
{
  if (i > Tdimension - 2)
  {
    std::stringstream stream;
    stream << "Vector index (" << i << ") out of bounds [0.." << Tdimension - 2 << "].";
    throw std::logic_error(stream.str());
  }
  return reinterpret_cast<tAngleRad *>(this)[i];
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Polar Set
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
template <typename ... TValues>
void FunctionalitySpecialized<Tdimension, TElement, Polar>::Set(TValues... values)
{
  static_assert(sizeof...(values) == Tdimension, "Wrong number of values given to store in vector");

  TElement *p = reinterpret_cast<TElement *>(this);
  util::ProcessVariadicValues<TElement>([p](TElement x) mutable { *p = x; ++p; }, values...);
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Polar SquaredLength
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const TElement FunctionalitySpecialized<Tdimension, TElement, Polar>::SquaredLength() const
{
  const tVector *that = reinterpret_cast<const tVector *>(this);
  return that->Length() * that->Length();
}

//----------------------------------------------------------------------
// FunctionalitySpecialized Polar IsZero
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement>
const bool FunctionalitySpecialized<Tdimension, TElement, Polar>::IsZero(double epsilon) const
{
  const tVector *that = reinterpret_cast<const tVector *>(this);
  return that->Length() < epsilon;
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
