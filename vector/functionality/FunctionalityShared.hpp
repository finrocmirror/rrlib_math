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
/*!\file    FunctionalityShared.h
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

#ifndef __rrlib__math__vector__functionality__FunctionalityShared_hpp__
#define __rrlib__math__vector__functionality__FunctionalityShared_hpp__

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
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// FunctionalityShared constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
FunctionalityShared<Tdimension, TElement, TData>::FunctionalityShared()
{
  std::memset(this, 0, sizeof(tVector));
}

template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
FunctionalityShared<Tdimension, TElement, TData>::FunctionalityShared(const tVector &other)
{
  std::memcpy(this, &other, sizeof(tVector));
}

//----------------------------------------------------------------------
// FunctionalityShared operator +=
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
template <typename TOtherElement>
const tVector<Tdimension, TElement, TData> &FunctionalityShared<Tdimension, TElement, TData>::operator += (const math::tVector<Tdimension, TOtherElement, TData> &other)
{
  tVector *that = reinterpret_cast<tVector *>(this);
  *that = *that + other;
  return *that;
}

//----------------------------------------------------------------------
// FunctionalityShared operator -=
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
template <typename TOtherElement>
const tVector<Tdimension, TElement, TData> &FunctionalityShared<Tdimension, TElement, TData>::operator -= (const math::tVector<Tdimension, TOtherElement, TData> &other)
{
  tVector *that = reinterpret_cast<tVector *>(this);
  *that = *that - other;
  return *that;
}

//----------------------------------------------------------------------
// FunctionalityShared operator *=
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
template <typename TScalar>
const typename boost::enable_if<boost::is_scalar<TScalar>, tVector<Tdimension, TElement, TData>>::type &FunctionalityShared<Tdimension, TElement, TData>::operator *= (const TScalar &scalar)
{
  tVector *that = reinterpret_cast<tVector *>(this);
  *that = *that * scalar;
  return *that;
}

//----------------------------------------------------------------------
// FunctionalityShared operator /=
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
template <typename TScalar>
const typename boost::enable_if<boost::is_scalar<TScalar>, tVector<Tdimension, TElement, TData>>::type &FunctionalityShared<Tdimension, TElement, TData>::operator /= (const TScalar &scalar)
{
  tVector *that = reinterpret_cast<tVector *>(this);
  if (scalar == 0)
  {
    throw std::runtime_error("Division by zero");
  }
  *that *= 1.0 / scalar;
  return *that;
}

//----------------------------------------------------------------------
// FunctionalityShared Normalize
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
void FunctionalityShared<Tdimension, TElement, TData>::Normalize()
{
  tVector *that = reinterpret_cast<tVector *>(this);

  TElement length = that->Length();
  *that *= length != 0 ? 1.0 / length : 0.0;
}

//----------------------------------------------------------------------
// FunctionalityShared Normalized
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
const tVector<Tdimension, TElement, TData> FunctionalityShared<Tdimension, TElement, TData>::Normalized() const
{
  const tVector *that = reinterpret_cast<const tVector *>(this);
  tVector temp(*that);
  temp.Normalize();
  return temp;
}

//----------------------------------------------------------------------
// FunctionalityShared Projected
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
template <typename TOtherElement>
const math::tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, TData> FunctionalityShared<Tdimension, TElement, TData>::Projected(const math::tVector<Tdimension, TOtherElement, TData> &other) const
{
  const tVector *that = reinterpret_cast<const tVector *>(this);
  math::tVector<Tdimension, typename until_0x::Auto<TElement, TOtherElement>::type, TData> temp(*that);
  temp.Project(other);
  return temp;
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
