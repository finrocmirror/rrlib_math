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
/*!\file    rrlib/math/vector/functionality/FunctionalityShared.h
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

#ifndef __rrlib__math__vector__functionality__FunctionalityShared_h__
#define __rrlib__math__vector__functionality__FunctionalityShared_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <type_traits>

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
class FunctionalityShared
{
  typedef math::tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> tVector;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

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
  inline FunctionalityShared &operator = (const math::tVector<Tother_dimension, TOtherElement, TData, TAdditionalDataParameters...> &other)
  {
    uint8_t *this_addr = reinterpret_cast<uint8_t *>(this);
    const uint8_t *other_addr = reinterpret_cast<const uint8_t *>(&other);

    if (this_addr != other_addr)
    {
      const size_t safety_area = this_addr < other_addr ? sizeof(tVector) : sizeof(math::tVector<Tother_dimension, TOtherElement, TData, TAdditionalDataParameters...>);
      if (static_cast<size_t>(std::abs(this_addr - other_addr)) < safety_area)
      {
        std::stringstream stream;
        stream << "Overlapping memory areas in rrlib::math::tVector::operator = (this = " << this << ", other = " << &other << ")!";
        throw std::logic_error(stream.str());
      }
      std::memset(this, 0, sizeof(tVector));
      size_t size = std::min(Tdimension, Tother_dimension);
      if (std::is_same<TData<2, int, TAdditionalDataParameters...>, Cartesian<2, int>>::value)
      {
        for (size_t i = 0; i < size; ++i)
        {
          reinterpret_cast<TElement *>(this)[i] = reinterpret_cast<const TOtherElement *>(&other)[i];
        }
      }
      else
      {
        std::memcpy(this, &other, sizeof(tVector) - sizeof(TElement));
        *reinterpret_cast<TElement *>(this_addr + sizeof(tAngle<TElement, angle::Radian, angle::Signed>[Tdimension - 1])) = *reinterpret_cast<const TOtherElement *>(other_addr + sizeof(tAngle<TElement, angle::Radian, angle::Signed>[Tdimension - 1]));
      }
    }
    return *this;
  }

  template <typename TOtherElement>
  inline const tVector &operator += (const math::tVector<Tdimension, TOtherElement, TData, TAdditionalDataParameters...> &other) __attribute__((always_inline));

  template <typename TOtherElement>
  inline const tVector &operator -= (const math::tVector<Tdimension, TOtherElement, TData, TAdditionalDataParameters...> &other) __attribute__((always_inline));

  template <typename TScalar>
  inline const typename std::enable_if<std::is_scalar<TScalar>::value, tVector>::type &operator *= (const TScalar &scalar) __attribute__((always_inline));

  template <typename TScalar>
  inline const typename std::enable_if<std::is_scalar<TScalar>::value, tVector>::type &operator /= (const TScalar &scalar) __attribute__((always_inline));

  inline void Normalize() __attribute__((always_inline));

  inline const tVector Normalized() const __attribute__((always_inline));

  template <typename TOtherElement>
  inline void Project(const math::tVector<Tdimension, TOtherElement, TData, TAdditionalDataParameters...> &other)
  {
    tVector *that = reinterpret_cast<tVector *>(this);
    double others_norm = static_cast<double>(other.SquaredLength());
    if (others_norm == 0)
    {
      *that = tVector::Zero();
    }
    else
    {
      double projection_factor = static_cast<double>(*that * other) / others_norm;
      *that = other;
      *that *= projection_factor;
    }
  }

  template <typename TOtherElement>
  inline const math::tVector <Tdimension, decltype(TElement() * double()), TData, TAdditionalDataParameters...> Projected(const math::tVector<Tdimension, TOtherElement, TData, TAdditionalDataParameters...> &other) const __attribute__((always_inline));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline FunctionalityShared() __attribute__((always_inline));

  inline FunctionalityShared(const tVector &other) __attribute__((always_inline));

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:


};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "rrlib/math/vector/functionality/FunctionalityShared.hpp"

#endif
