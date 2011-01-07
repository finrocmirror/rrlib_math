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
#ifndef _rrlib_math_vector_include_guard_
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef _rrlib_math_vector_functionality_FunctionalitySpecialized_h_
#define _rrlib_math_vector_functionality_FunctionalitySpecialized_h_

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

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

protected:

  inline FunctionalitySpecialized() {}
};

/*!
 *
 */
template <size_t Tdimension, typename TElement>
class FunctionalitySpecialized<Tdimension, TElement, Cartesian>
{
  typedef math::tVector<Tdimension, TElement, Cartesian> tVector;

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

protected:

  inline FunctionalitySpecialized() {}

public:

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

};

/*!
 *
 */
template <size_t Tdimension, typename TElement>
class FunctionalitySpecialized<Tdimension, TElement, Polar>
{
  typedef math::tVector<Tdimension, TElement, Cartesian> tVector;

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

protected:

  inline FunctionalitySpecialized() {}

public:

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

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
