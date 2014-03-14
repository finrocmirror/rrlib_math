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
/*!\file    rrlib/math/vector/tVector.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-08-25
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__vector__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef __rrlib__math__vector__tVector_hpp__
#define __rrlib__math__vector__tVector_hpp__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#ifdef _LIB_OIV_PRESENT_
#include <Inventor/SbVec2f.h>
#include <Inventor/SbVec3f.h>
#include <type_traits>
#endif
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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
TElement tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::EuclideanDistance(const tVector &a, const tVector &b)
{
  return (a - b).Length();
};

//template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
//TElement tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::ManhattanDistance(const tVector &a, const tVector &b)
//{
//  TElement result = 0;
//  for (size_t i = 0; i < Tdimension; ++i)
//  {
//    result += AbsoluteValue(a[i] - b[i]);
//  }
//  return result;
//};
//
//template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
//TElement tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::ChebyshevDistance(const tVector &a, const tVector &b)
//{
//  TElement result = 0;
//  for (size_t i = 0; i < Tdimension; ++i)
//  {
//    result = std::max(result, AbsoluteValue(a[i] - b[i]));
//  }
//  return result;
//};

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tVector constructors
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::tVector()
{}

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::tVector(const tVector &other)
  : FunctionalityShared(other)
{}

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
template <size_t Tother_dimension, typename TOtherElement>
tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::tVector(const tVector<Tother_dimension, TOtherElement> &other)
  : FunctionalitySpecialized(other)
{}

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
template <typename TOtherUnitPolicy, typename TOtherSignPolicy>
tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::tVector(const tVector<Tdimension, TElement, TData, TOtherUnitPolicy, TOtherSignPolicy> &other)
  : FunctionalitySpecialized(other)
{}

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
template <typename ... TValues>
tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::tVector(TValues... values)
  : FunctionalitySpecialized(values...)
{}

#ifdef _LIB_OIV_PRESENT_

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
template <typename T>
tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::tVector(const SbVec2f &v, typename std::enable_if <(Tdimension == 2), T>::type)
{
  FunctionalitySpecialized::Set(v[0], v[1]);
}

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
template <typename T>
tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::tVector(const SbVec3f &v, typename std::enable_if <(Tdimension == 3), T>::type)
{
  FunctionalitySpecialized::Set(v[0], v[1], v[2]);
}

#endif

//----------------------------------------------------------------------
// tVector operator =
//----------------------------------------------------------------------
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::operator = (const tVector &other)
{
  return reinterpret_cast<tVector &>(FunctionalityShared::operator=(other));
}

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
template <size_t Tother_dimension, typename TOtherElement>
tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &tVector<Tdimension, TElement, TData, TAdditionalDataParameters...>::operator = (const tVector<Tother_dimension, TOtherElement, TData, TAdditionalDataParameters...> &other)
{
  return reinterpret_cast<tVector &>(FunctionalityShared::operator=(other));
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
