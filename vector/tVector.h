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
/*!\file    rrlib/math/vector/tVector.h
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

#ifndef __rrlib__math__vector__tVector_h__
#define __rrlib__math__vector__tVector_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <functional>

#ifdef _LIB_OIV_PRESENT_
#include <Inventor/SbVec2f.h>
#include <Inventor/SbVec3f.h>
#include <boost/utility/enable_if.hpp>
#endif
//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
namespace math
{
template <size_t, typename> class tVector;
}

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
template < size_t Tdimension, typename TElement = double, template <size_t, typename> class TData = vector::Cartesian >
class tVector : public TData<Tdimension, TElement>,
  public vector::FunctionalityShared<Tdimension, TElement, TData>,
  public vector::FunctionalitySpecialized<Tdimension, TElement, TData>,
  public vector::Conversions<Tdimension, TElement, TData>,
  public vector::Rotation<Tdimension, TElement, TData>,
  public vector::LegacyShared<Tdimension, TElement, TData>,
  public vector::LegacySpecialized<Tdimension, TElement, TData>,
  public vector::ConstantValuesShared<Tdimension, TElement, TData>,
  public vector::ConstantValuesSpecialized<Tdimension, TElement, TData>
{
  typedef vector::FunctionalityShared<Tdimension, TElement, TData> FunctionalityShared;
  typedef vector::FunctionalitySpecialized<Tdimension, TElement, TData> FunctionalitySpecialized;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef std::function < TElement(const tVector &, const tVector &) > tMetric;

  static const tMetric cEUCLIDEAN_DISTANCE;
  static const tMetric cMANHATTAN_DISTANCE;
  static const tMetric cCHEBYSHEV_DISTANCE;

  inline tVector() __attribute__((always_inline, flatten));

  inline tVector(const tVector &other) __attribute__((always_inline));

  template <size_t Tother_dimension, typename TOtherElement>
  inline tVector(const tVector<Tother_dimension, TOtherElement> &other) __attribute__((always_inline, flatten));

  template <typename ... TValues>
  explicit inline tVector(TValues... values) __attribute__((always_inline, flatten));

#ifdef _LIB_OIV_PRESENT_

  template < typename T = int >
  explicit inline tVector(const SbVec2f &v, typename boost::enable_if_c < (Tdimension == 2), T >::type = 0) __attribute__((always_inline, flatten));

  template < typename T = int >
  explicit inline tVector(const SbVec3f &v, typename boost::enable_if_c < (Tdimension == 3), T >::type = 0) __attribute__((always_inline, flatten));

#endif

  inline tVector &operator = (const tVector &other) __attribute__((always_inline, flatten));

  template <size_t Tother_dimension, typename TOtherElement>
  inline tVector &operator = (const tVector<Tother_dimension, TOtherElement, TData> &other) __attribute__((always_inline, flatten));

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/math/vector/tVector.hpp"

#endif
