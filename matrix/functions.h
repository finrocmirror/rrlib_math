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
/*!\file    rrlib/math/matrix/functions.h
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
#ifndef __rrlib__math__matrix__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef __rrlib__math__matrix__functions_h__
#define __rrlib__math__matrix__functions_h__

//----------------------------------------------------------------------
// External includes with <>
//----------------------------------------------------------------------

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



template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TLeftData, template <size_t, size_t, typename> class TRightData>
inline bool IsEqual(const tMatrix<Trows, Tcolumns, TElement, TLeftData> &left, const tMatrix<Trows, Tcolumns, TElement, TRightData> &right, float max_error = 1.0E-6, tFloatComparisonMethod method = eFCM_ABSOLUTE_ERROR) __attribute__((always_inline, flatten));

template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TLeftData, template <size_t, size_t, typename> class TRightData>
inline const bool operator == (const tMatrix<Trows, Tcolumns, TElement, TLeftData> &left, const tMatrix<Trows, Tcolumns, TElement, TRightData> &right) __attribute__((always_inline, flatten));

template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TLeftData, template <size_t, size_t, typename> class TRightData>
inline const bool operator != (const tMatrix<Trows, Tcolumns, TElement, TLeftData> &left, const tMatrix<Trows, Tcolumns, TElement, TRightData> &right) __attribute__((always_inline, flatten));

template <typename TElement>
const tMatrix<2, 2, TElement> Get2DRotationMatrix(tAngleRad angle);

template <typename TElement>
const tMatrix<3, 3, TElement> Get3DRotationMatrixFromRollPitchYaw(tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

template <typename TElement>
const tMatrix<4, 4, TElement> Get4DTransformationMatrixZYXT(TElement x, TElement y, TElement z, tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/math/matrix/functions.hpp"

#endif
