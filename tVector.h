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
/*!\file    tVector.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-09-26
 *
 * \brief   Template implementation of mathematical vectors for RRLib
 *
 * \b tVector
 *
 * tVector is a mathematical vector implementation to be used in RRLib
 * context that heavily utilizes policy base template programming. Thus,
 * consisting of far less than 1k lines of code it features vectors of
 * arbitrary dimension and element data types. Included features are e.g.
 * type safety, conversion, basic mathematical operators, Cartesian or
 * polar interpretation.
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__tVector_h__
#define __rrlib__math__tVector_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/until_0x_helper.h"

#define __rrlib__math__vector__include_guard__

namespace rrlib
{
namespace math
{
template <size_t, typename, template <size_t, typename> class>
class tVector;
}
}

// FIXME: suppress bogus warning when compiling with gcc 4.3
#if (__GNUC__ == 4 && __GNUC_MINOR__ == 3)
#pragma GCC diagnostic ignored "-Warray-bounds"
#endif

#include "rrlib/math/vector/data/Cartesian.h"
#include "rrlib/math/vector/data/Polar.h"

#include "rrlib/math/vector/data/OperatorsCartesian.h"
#include "rrlib/math/vector/data/OperatorsPolar.h"
#include "rrlib/math/vector/data/OperatorsShared.h"

#include "rrlib/math/vector/data/ConstantValuesShared.h"
#include "rrlib/math/vector/data/ConstantValuesSpecialized.h"

#include "rrlib/math/vector/functionality/FunctionalityShared.h"
#include "rrlib/math/vector/functionality/FunctionalitySpecialized.h"
#include "rrlib/math/vector/functionality/Conversions.h"
#include "rrlib/math/vector/functionality/Rotation.h"
#include "rrlib/math/vector/functionality/LegacyShared.h"
#include "rrlib/math/vector/functionality/LegacySpecialized.h"

#include "rrlib/math/vector/tVector.h"

#include "rrlib/math/vector/functions.h"

#undef __rrlib_math__vector__include_guard__

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

typedef tVector<2, double> tVec2d;
typedef tVector<3, double> tVec3d;
typedef tVector<6, double> tVec6d;

typedef tVector<2, float> tVec2f;
typedef tVector<3, float> tVec3f;
typedef tVector<6, float> tVec6f;

typedef tVector<2, int> tVec2i;
typedef tVector<3, int> tVec3i;
typedef tVector<6, int> tVec6i;

typedef tVector<2, unsigned int> tVec2u;
typedef tVector<3, unsigned int> tVec3u;
typedef tVector<6, unsigned int> tVec6u;



//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tVector<2, double, vector::Cartesian>;
extern template class tVector<3, double, vector::Cartesian>;
extern template class tVector<6, double, vector::Cartesian>;

extern template class tVector<2, float, vector::Cartesian>;
extern template class tVector<3, float, vector::Cartesian>;
extern template class tVector<6, float, vector::Cartesian>;

extern template class tVector<2, int, vector::Cartesian>;
extern template class tVector<3, int, vector::Cartesian>;
extern template class tVector<6, int, vector::Cartesian>;

extern template class tVector<2, unsigned int, vector::Cartesian>;
extern template class tVector<3, unsigned int, vector::Cartesian>;
extern template class tVector<6, unsigned int, vector::Cartesian>;

extern template class tVector<2, double, vector::Polar>;
extern template class tVector<3, double, vector::Polar>;

extern template class tVector<2, float, vector::Polar>;
extern template class tVector<3, float, vector::Polar>;



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
