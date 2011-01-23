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
/*!\file    FunctionalityShared.h
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

#ifndef _rrlib_math_vector_functionality_FunctionalityShared_h_
#define _rrlib_math_vector_functionality_FunctionalityShared_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_scalar.hpp>

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
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
class FunctionalityShared
{
  typedef math::tVector<Tdimension, TElement, TData> tVector;

  FunctionalityShared(const FunctionalityShared &);

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

protected:

  inline FunctionalityShared()
  {
    std::memset(this, 0, sizeof(tVector));
  }
  inline FunctionalityShared(const tVector &other)
  {
    std::memcpy(this, &other, sizeof(tVector));
  }

  explicit inline FunctionalityShared(const TElement data[Tdimension])
  {
    std::memcpy(this, data, sizeof(tVector));
  }

  template <typename TOtherElement>
  explicit inline FunctionalityShared(const TOtherElement data[Tdimension])
  {
    for (size_t i = 0; i < Tdimension; ++i)
    {
      (*this)[i] = data[i];
    }
  }

  template <size_t Tother_dimension, typename TOtherElement>
  explicit inline FunctionalityShared(const math::tVector<Tother_dimension, TOtherElement, TData> &other)
  {
    std::memset(this, 0, sizeof(tVector));
    size_t size = std::min(Tdimension, Tother_dimension);
    for (size_t i = 0; i < size; ++i)
    {
      reinterpret_cast<TElement *>(this)[i] = reinterpret_cast<const TOtherElement *>(&other)[i];
    }
  }

public:

  inline const TElement operator [](size_t i) const
  {
    return const_cast<FunctionalityShared &>(*this)[i];
  }
  inline TElement &operator [](size_t i)
  {
    if (i >= Tdimension)
    {
      std::stringstream stream;
      stream << "Vector index (" << i << " out of bounds [0.." << Tdimension << "].";
      throw std::logic_error(stream.str());
    }
    return reinterpret_cast<TElement *>(this)[i];
  }

  inline FunctionalityShared &operator = (const FunctionalityShared &other)
  {
    const uint8_t *this_addr = reinterpret_cast<uint8_t *>(this);
    const uint8_t *other_addr = reinterpret_cast<const uint8_t *>(&other);
    if (this_addr != other_addr)
    {
      if (std::abs(this_addr - other_addr) < sizeof(tVector))
      {
        std::stringstream stream;
        stream << "Overlapping memory areas in rrlib::math::tVector::operator = (this = " << this << ", other = " << &other << ")!";
        throw std::logic_error(stream.str());
      }
      std::memcpy(this, &other, sizeof(tVector));
    }
    return *this;
  }

  template <size_t Tother_dimension, typename TOtherElement>
  inline FunctionalityShared &operator = (const math::tVector<Tother_dimension, TOtherElement, TData> &other)
  {
    const uint8_t *this_addr = reinterpret_cast<uint8_t *>(this);
    const uint8_t *other_addr = reinterpret_cast<const uint8_t *>(&other);

    if (this_addr != other_addr)
    {
      const size_t safety_area = this_addr < other_addr ? sizeof(tVector) : sizeof(math::tVector<Tother_dimension, TOtherElement, TData>);
      if (static_cast<size_t>(std::abs(this_addr - other_addr)) < safety_area)
      {
        std::stringstream stream;
        stream << "Overlapping memory areas in rrlib::math::tVector::operator = (this = " << this << ", other = " << &other << ")!";
        throw std::logic_error(stream.str());
      }
      std::memset(this, 0, sizeof(tVector));
      size_t size = std::min(Tdimension, Tother_dimension);
      for (size_t i = 0; i < size; ++i)
      {
        reinterpret_cast<TElement *>(this)[i] = reinterpret_cast<const TOtherElement *>(&other)[i];
      }
    }
    return *this;
  }

  template <typename ... TValues>
  inline void Set(TValues... values)
  {
    TElement buffer[Tdimension];
    this->SetValues<0>(buffer, values...);
  }

  template <typename TOtherElement>
  inline const tVector &operator += (const math::tVector<Tdimension, TOtherElement, TData> &other)
  {
    tVector *that = reinterpret_cast<tVector *>(this);
    *that = *that + other;
    return *that;
  }

  template <typename TOtherElement>
  inline const tVector &operator -= (const math::tVector<Tdimension, TOtherElement, TData> &other)
  {
    tVector *that = reinterpret_cast<tVector *>(this);
    *that = *that - other;
    return *that;
  }

  template <typename TScalar>
  inline const typename boost::enable_if<boost::is_scalar<TScalar>, tVector>::type &operator *= (const TScalar &scalar)
  {
    tVector *that = reinterpret_cast<tVector *>(this);
    *that = *that * scalar;
    return *that;
  }

  template <typename TScalar>
  inline const typename boost::enable_if<boost::is_scalar<TScalar>, tVector>::type &operator /= (const TScalar &scalar)
  {
    tVector *that = reinterpret_cast<tVector *>(this);
    *that *= 1.0 / scalar;
    return *that;
  }

  inline void Normalize()
  {
    tVector *that = reinterpret_cast<tVector *>(this);
    *that *= 1.0 / that->Length();
  }

  inline const tVector Normalized() const
  {
    const tVector *that = reinterpret_cast<const tVector *>(this);
    tVector temp(*that);
    temp.Normalize();
    return temp;
  }

  template <typename TOtherElement>
  inline void Project(const math::tVector<Tdimension, TOtherElement, TData> &other)
  {
    tVector *that = reinterpret_cast<tVector *>(this);
    TOtherElement others_norm = other.SquaredLength();
    if (others_norm == 0)
    {
      *that = tVector::Zero();
    }
    else
    {
      double projection_factor = *that * other / others_norm;
      *that = other;
      *that *= projection_factor;
    }
  }

  template <typename TOtherElement>
  inline const math::tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, TData> Projected(const math::tVector<Tdimension, TOtherElement, TData> &other) const
  {
    const tVector *that = reinterpret_cast<const tVector *>(this);
    math::tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, TData> temp(*that);
    temp.Project(other);
    return temp;
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
