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
/*!\file    rrlib/math/tMatrix.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-21
 *
 * \brief   Template implementation of mathematical matrices for RRLib
 *
 * \b tMatrix
 *
 * tMatrix is a mathematical matrix implementation to be used in RRLib
 * context that heavily utilizes policy base template programming. Thus,
 * consisting of far less than 1k lines of code it features matricess of
 * arbitrary dimension and element data types. Included features are e.g.
 * type safety, conversion, basic mathematical operators.
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__tMatrix_h__
#define __rrlib__math__tMatrix_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

#define __rrlib__math__matrix__include_guard__

namespace rrlib
{
namespace math
{
template <size_t, size_t, typename, template <size_t, size_t, typename> class>
class tMatrix;
}
}

#include "rrlib/math/matrix/data/Full.h"
#include "rrlib/math/matrix/data/Symmetrical.h"
#include "rrlib/math/matrix/data/LowerTriangle.h"
#include "rrlib/math/matrix/data/UpperTriangle.h"


#include "rrlib/math/matrix/data/OperatorsFull.h"
#include "rrlib/math/matrix/data/OperatorsLowerTriangle.h"
#include "rrlib/math/matrix/data/OperatorsUpperTriangle.h"
#include "rrlib/math/matrix/data/OperatorsSymmetrical.h"

#include "rrlib/math/matrix/data/OperatorsShared.h"

#include "rrlib/math/matrix/data/ConstantValuesShared.h"
#include "rrlib/math/matrix/data/ConstantValuesSpecialized.h"

#include "rrlib/math/matrix/functionality/FunctionalityShared.h"
#include "rrlib/math/matrix/functionality/FunctionalitySpecialized.h"
#include "rrlib/math/matrix/functionality/SquareMatrixOperationsShared.h"
#include "rrlib/math/matrix/functionality/SquareMatrixOperationsSpecialized.h"
#include "rrlib/math/matrix/functionality/HomogeneousMultiplication.h"
#include "rrlib/math/matrix/functionality/Rotation.h"
#include "rrlib/math/matrix/functionality/LegacyShared.h"
#include "rrlib/math/matrix/functionality/LegacySpecialized.h"
#include "rrlib/math/matrix/functionality/Conversions.h"

#include "rrlib/math/matrix/tMatrix.h"

#include "rrlib/math/matrix/functions.h"

#undef __rrlib__math__matrix__include_guard__

//----------------------------------------------------------------------
// Implementation
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

typedef tMatrix<2, 2, double, matrix::Full> tMat2x2d;
typedef tMatrix<3, 3, double, matrix::Full> tMat3x3d;
typedef tMatrix<4, 4, double, matrix::Full> tMat4x4d;

typedef tMatrix<2, 2, float, matrix::Full> tMat2x2f;
typedef tMatrix<3, 3, float, matrix::Full> tMat3x3f;
typedef tMatrix<4, 4, float, matrix::Full> tMat4x4f;

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tMatrix<2, 2, double, matrix::Full>;
extern template class tMatrix<3, 3, double, matrix::Full>;
extern template class tMatrix<4, 4, double, matrix::Full>;

extern template class tMatrix<2, 2, float, matrix::Full>;
extern template class tMatrix<3, 3, float, matrix::Full>;
extern template class tMatrix<4, 4, float, matrix::Full>;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
