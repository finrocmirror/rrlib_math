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
/*!\file    FunctionalitySpecialized.h
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
#include <cmath>
#include <boost/utility/enable_if.hpp>
#include <iostream>

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
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
class FunctionalitySpecialized
{
  typedef math::tVector<Tdimension, TElement, TData> tVector;

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline FunctionalitySpecialized() {}

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

  inline const TElement operator [](size_t i) const
  {
    return const_cast<FunctionalitySpecialized &>(*this)[i];
  }
  inline TElement &operator [](size_t i)
  {
    if (i > Tdimension - 1)
    {
      std::stringstream stream;
      stream << "Vector index (" << i << ") out of bounds [0.." << Tdimension - 1 << "].";
      throw std::logic_error(stream.str());
    }
    return reinterpret_cast<TElement *>(this)[i];
  }

  template <typename ... TValues>
  inline void Set(TValues... values)
  {
    TElement buffer[Tdimension];
    this->SetValues<0>(buffer, values...);
  }

  static inline const tVector Direction(size_t i)
  {
    tVector vector;
    vector[i] = 1;
    return vector;
  }

  inline const TElement Length() const
  {
    return std::sqrt(this->SquaredLength());
  }

  inline const TElement SquaredLength() const
  {
    const tVector *that = reinterpret_cast<const tVector *>(this);
    TElement result = 0;
    for (size_t i = 0; i < Tdimension; ++i)
    {
      result += (*that)[i] * (*that)[i];
    }
    return result;
  }

  inline const bool IsZero(double epsilon = 0) const
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

  template <typename TOtherElement>
  inline void SchurMultiply(const math::tVector<Tdimension, TOtherElement, Cartesian> &other)
  {
    tVector *that = reinterpret_cast<tVector *>(this);
    for (size_t i = 0; i < Tdimension; ++i)
    {
      (*that)[i] *= other[i];
    }
  }

  template <typename TOtherElement>
  inline const math::tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, Cartesian> SchurMultiplied(const math::tVector<Tdimension, TOtherElement, Cartesian> &other) const
  {
    const tVector *that = reinterpret_cast<const tVector *>(this);
    math::tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, Cartesian> temp(*that);
    temp.SchurMultiply(other);
    return temp;
  }

  template <size_t Tother_dimension, typename TOtherElement>
  inline typename boost::enable_if_c < Tdimension == 3 && Tother_dimension == 3, void >::type CrossMultiply(const math::tVector<Tother_dimension, TOtherElement, Cartesian> &other)
  {
    tVector *that = reinterpret_cast<tVector *>(this);
    that->Set(that->Y() * other.Z() - that->Z() * other.Y(),
              that->Z() * other.X() - that->X() * other.Z(),
              that->X() * other.Y() - that->Y() * other.X());
  }

  template <size_t Tother_dimension, typename TOtherElement>
  inline const typename boost::enable_if_c < Tdimension == 3 && Tother_dimension == 3, math::tVector<3, typename until_0x::Auto<TElement, TOtherElement>::type, Cartesian> >::type CrossMultiplied(const math::tVector<Tother_dimension, TOtherElement, Cartesian> &other) const
  {
    const tVector *that = reinterpret_cast<const tVector *>(this);
    math::tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, Cartesian> temp(*that);
    temp.CrossMultiply(other);
    return temp;
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline FunctionalitySpecialized() {}

  explicit inline FunctionalitySpecialized(const TElement data[Tdimension])
  {
    std::memcpy(this, data, sizeof(tVector));
  }

  template <typename TOtherElement>
  explicit inline FunctionalitySpecialized(const TOtherElement data[Tdimension])
  {
    for (size_t i = 0; i < Tdimension; ++i)
    {
      (*this)[i] = data[i];
    }
  }

  template <size_t Tother_dimension, typename TOtherElement>
  explicit inline FunctionalitySpecialized(const math::tVector<Tother_dimension, TOtherElement, Cartesian> &other)
  {
    std::memset(this, 0, sizeof(tVector));
    size_t size = std::min(Tdimension, Tother_dimension);
    for (size_t i = 0; i < size; ++i)
    {
      reinterpret_cast<TElement *>(this)[i] = reinterpret_cast<const TOtherElement *>(&other)[i];
    }
  }

  template <typename ... TValues>
  explicit inline FunctionalitySpecialized(TElement value, TValues... values)
  {
    FunctionalitySpecialized::Set(value, values...);
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

  template <size_t number_of_given_values>
  inline void SetValues(TElement buffer[Tdimension])
  {
    static_assert(number_of_given_values == Tdimension, "Wrong number of values given to store in vector");
    std::memcpy(this, buffer, sizeof(tVector));
  }
  template <size_t number_of_given_values, typename ... TValues>
  inline void SetValues(TElement buffer[Tdimension], TElement value, TValues... values)
  {
    buffer[number_of_given_values] = value;
    this->SetValues < number_of_given_values + 1 > (buffer, values...);
  }

};

/*!
 *
 */
template <size_t Tdimension, typename TElement>
class FunctionalitySpecialized<Tdimension, TElement, Polar>
{
  typedef math::tVector<Tdimension, TElement, Polar> tVector;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline const tAngleRad operator [](size_t i) const
  {
    return const_cast<FunctionalitySpecialized &>(*this)[i];
  }
  inline tAngleRad &operator [](size_t i)
  {
    if (i > Tdimension - 2)
    {
      std::stringstream stream;
      stream << "Vector index (" << i << ") out of bounds [0.." << Tdimension - 2 << "].";
      throw std::logic_error(stream.str());
    }
    return reinterpret_cast<tAngleRad *>(this)[i];
  }

  template <typename ... TValues>
  inline void Set(TValues... values)
  {
    tAngleRad buffer[Tdimension - 1];
    this->SetValues<0>(buffer, values...);
  }

  inline const TElement SquaredLength() const
  {
    const tVector *that = reinterpret_cast<const tVector *>(this);
    return that->Length() * that->Length();
  }

  inline const bool IsZero(double epsilon = 0) const
  {
    const tVector *that = reinterpret_cast<const tVector *>(this);
    return that->Length() < epsilon;
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline FunctionalitySpecialized() {}

  template <typename TOtherElement>
  explicit inline FunctionalitySpecialized(const tAngleRad angles[Tdimension - 1], TOtherElement length)
  {
    tVector *that = reinterpret_cast<tVector *>(this);
    std::memcpy(this, angles, sizeof(tVector) - sizeof(TElement));
    that->Length() = length;
  }

  template <size_t Tother_dimension, typename TOtherElement>
  explicit inline FunctionalitySpecialized(const math::tVector<Tother_dimension, TOtherElement, Polar> &other)
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

  template <typename ... TValues>
  explicit inline FunctionalitySpecialized(tAngleRad value, TValues... values)
  {
    FunctionalitySpecialized::Set(value, values...);
  }


//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

  template <size_t number_of_given_values>
  inline void SetValues(tAngleRad buffer[Tdimension - 1], TElement length)
  {
    static_assert(number_of_given_values == Tdimension - 1, "Wrong number of values given to store in vector");
    tVector *that = reinterpret_cast<tVector *>(this);
    std::memcpy(this, buffer, sizeof(tVector));
    that->Length() = length;
  }
  template <size_t number_of_given_values, typename ... TValues>
  inline void SetValues(tAngleRad buffer[Tdimension - 1], tAngleRad value, TValues... values)
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

#endif
