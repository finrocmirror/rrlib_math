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
  typedef tVector<Tdimension, TElement, TData> tVectorType;

  FunctionalityShared(const FunctionalityShared &);

  template <size_t number_of_given_values>
  inline void SetValues(TElement buffer[Tdimension])
  {
    static_assert(number_of_given_values == Tdimension, "Wrong number of values given to store in vector");
    memcpy(this, buffer, sizeof(tVectorType));
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
    memset(this, 0, sizeof(tVectorType));
  }
  inline FunctionalityShared(const tVectorType &other)
  {
    memcpy(this, &other, sizeof(tVectorType));
  }

  explicit inline FunctionalityShared(const TElement data[Tdimension])
  {
    memcpy(this, data, sizeof(tVectorType));
  }

  template <size_t Tother_dimension, typename TOtherElement>
  explicit inline FunctionalityShared(const tVector<Tother_dimension, TOtherElement, TData> &other)
  {
    memset(this, 0, sizeof(tVectorType));
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
      if (std::abs(this_addr - other_addr) < sizeof(tVectorType))
      {
        std::stringstream stream;
        stream << "Overlapping memory areas in rrlib::math::tVector::operator = (this = " << this << ", other = " << &other << ")!";
        throw std::logic_error(stream.str());
      }
      memcpy(this, &other, sizeof(tVectorType));
    }
    return *this;
  }

  template <size_t Tother_dimension, typename TOtherElement>
  inline FunctionalityShared &operator = (const tVector<Tother_dimension, TOtherElement, TData> &other)
  {
    const uint8_t *this_addr = reinterpret_cast<uint8_t *>(this);
    const uint8_t *other_addr = reinterpret_cast<const uint8_t *>(&other);

    if (this_addr != other_addr)
    {
      const size_t safety_area = this_addr < other_addr ? sizeof(tVectorType) : sizeof(tVector<Tother_dimension, TOtherElement, TData>);
      if (static_cast<size_t>(std::abs(this_addr - other_addr)) < safety_area)
      {
        std::stringstream stream;
        stream << "Overlapping memory areas in rrlib::math::tVector::operator = (this = " << this << ", other = " << &other << ")!";
        throw std::logic_error(stream.str());
      }
      memset(this, 0, sizeof(tVectorType));
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
  inline const tVectorType &operator += (const tVector<Tdimension, TOtherElement, TData> &other)
  {
    tVectorType *that = reinterpret_cast<tVectorType *>(this);
    *that = *that + other;
    return *that;
  }

  template <typename TOtherElement>
  inline const tVectorType &operator -= (const tVector<Tdimension, TOtherElement, TData> &other)
  {
    tVectorType *that = reinterpret_cast<tVectorType *>(this);
    *that = *that - other;
    return *that;
  }

  template <typename TScalar>
  inline const typename boost::enable_if<boost::is_scalar<TScalar>, tVectorType>::type &operator *= (const TScalar &scalar)
  {
    tVectorType *that = reinterpret_cast<tVectorType *>(this);
    *that = *that * scalar;
    return *that;
  }

  template <typename TScalar>
  inline const typename boost::enable_if<boost::is_scalar<TScalar>, tVectorType>::type &operator /= (const TScalar &scalar)
  {
    tVectorType *that = reinterpret_cast<tVectorType *>(this);
    *that *= 1.0 / scalar;
    return *that;
  }

  inline const bool operator == (const tVectorType &other) const
  {
    for (size_t i = 0; i < Tdimension; ++i)
    {
      if ((*this)[i] != other[i])
      {
        return false;
      }
    }
    return true;
  }

  inline const bool operator != (const tVectorType &other) const
  {
    return !(*this == other);
  }

  inline const bool Equals(const tVectorType &other, double epsilon = 0) const
  {
    for (size_t i = 0; i < Tdimension; ++i)
    {
      if (std::abs((*this)[i] - other[i]) >= epsilon)
      {
        return false;
      }
    }
    return true;
  }

  inline void Normalize()
  {
    tVectorType *that = reinterpret_cast<tVectorType *>(this);
    *that *= 1.0 / that->Length();
  }

  inline const tVectorType Normalized() const
  {
    const tVectorType *that = reinterpret_cast<const tVectorType *>(this);
    tVectorType temp(*that);
    temp.Normalize();
    return temp;
  }

  template <typename TOtherElement>
  inline void Project(const tVector<Tdimension, TOtherElement, TData> &other)
  {
    tVectorType *that = reinterpret_cast<tVectorType *>(this);
    TOtherElement others_norm = other.SquaredLength();
    if (others_norm == 0)
    {
      *that = tVectorType::Zero();
    }
    else
    {
      double projection_factor = *that * other / others_norm;
      *that = other;
      *that *= projection_factor;
    }
  }

  template <typename TOtherElement>
  inline const tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, TData> Projected(const tVector<Tdimension, TOtherElement, TData> &other) const
  {
    const tVectorType *that = reinterpret_cast<const tVectorType *>(this);
    tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, TData> temp(*that);
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
