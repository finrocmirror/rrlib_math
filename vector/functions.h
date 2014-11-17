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
/*!\file    rrlib/math/vector/functions.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-09-26
 *
 * \brief
 *
 * \b
 *
 * A few words for functions.h
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__vector__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef __rrlib__math__vector__functions_h__
#define __rrlib__math__vector__functions_h__

//----------------------------------------------------------------------
// External includes with <>
//----------------------------------------------------------------------

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



template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
inline bool IsEqual(const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right, float max_error = 1.0E-6, tFloatComparisonMethod method = eFCM_ABSOLUTE_ERROR) __attribute__((always_inline));

template <size_t Tdimension, typename TElement>
inline bool IsLinearlyDependent(const tVector<Tdimension, TElement, vector::Cartesian> &left, const tVector<Tdimension, TElement, vector::Cartesian> &right, float = 1.0E-6, tFloatComparisonMethod method = eFCM_ABSOLUTE_ERROR) __attribute__((always_inline));

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
inline const bool operator == (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right) __attribute__((always_inline));

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
inline const bool operator != (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right) __attribute__((always_inline));

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
inline const bool operator < (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right);

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
inline const bool operator > (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right);

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
inline const bool operator <= (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right);

template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
inline const bool operator >= (const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TElement, TData, TAdditionalDataParameters...> &right);

template <size_t Tdimension, typename TLeftElement, typename TRightElement>
inline const tVector <Tdimension, decltype(TLeftElement() * TRightElement()), vector::Cartesian> SchurProduct(const tVector<Tdimension, TLeftElement, vector::Cartesian> &left, const tVector<Tdimension, TRightElement, vector::Cartesian> &right) __attribute__((always_inline));

template <typename TLeftElement, typename TRightElement>
inline const tVector <3, decltype(TLeftElement() * TRightElement()), vector::Cartesian> CrossProduct(const tVector<3, TLeftElement, vector::Cartesian> &left, const tVector<3, TRightElement, vector::Cartesian> &right) __attribute__((always_inline));

template <size_t Tdimension, typename TLeftElement, typename TRightElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
inline const tAngleRad EnclosedAngle(const tVector<Tdimension, TLeftElement, TData, TAdditionalDataParameters...> &left, const tVector<Tdimension, TRightElement, TData, TAdditionalDataParameters...> &right);

template <typename TLeftElement, typename TRightElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
const tAngleRad EnclosedAngle(const tVector<2, TLeftElement, TData, TAdditionalDataParameters...> &left, const tVector<2, TRightElement, TData, TAdditionalDataParameters...> &right);

/*!
 * \brief Convert Cartesian coordinates into polar coordinates
 *
 * \param polar       A reference to the vector that is supposed to hold the polar coordinates (angle, radius)
 * \param cartesian   The vector that holds the Cartesian coordinates
 */
template <size_t Tdimension, typename TPolarElement, typename TCartesianElement, typename TPolarUnitPolicy = angle::Radian, typename TPolarAutoWrapPolicy = angle::Unsigned>
inline void GetPolarVectorFromCartesian(tVector<Tdimension, TPolarElement, vector::Polar, TPolarUnitPolicy, TPolarAutoWrapPolicy> &polar, const tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian) __attribute__((always_inline));

/*!
 * \brief Convert polar coordinates into Cartesian coordinates
 *
 * \param cartesian   A reference to the vector that is supposed to hold the Cartesian coordinates
 * \param polar       The vector that holds the polar coordinates (angle, radius)
 */
template <size_t Tdimension, typename TCartesianElement, typename TPolarElement, typename TPolarUnitPolicy, typename TPolarAutoWrapPolicy>
inline void GetCartesianVectorFromPolar(tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian, const tVector<Tdimension, TPolarElement, vector::Polar, TPolarUnitPolicy, TPolarAutoWrapPolicy> &polar) __attribute__((always_inline));

template <typename TElement, typename TPolarUnitPolicy = angle::Radian>
inline tVector<2, TElement, vector::Polar, TPolarUnitPolicy, angle::Signed> GetPolarSignedVectorFromCartesian(tVector<2, TElement, vector::Cartesian> cartesian, double radius = 1) __attribute__((always_inline));

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/math/vector/functions.hpp"

#endif
