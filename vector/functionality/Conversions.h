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
/*!\file    Conversions.h
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

#ifndef _rrlib_math_vector_functionality_Conversions_h_
#define _rrlib_math_vector_functionality_Conversions_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>

#ifdef _LIB_OIV_PRESENT_
#include <Inventor/SbVec2f.h>
#include <Inventor/SbVec3f.h>
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
class Conversions
{
  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);

protected:

  inline Conversions() {}

};

/*!
 *
 */
template <typename TElement>
class Conversions<2, TElement, Cartesian>
{
  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);

protected:

  inline Conversions() {}

public:

  inline const tVector<2, TElement, Polar> GetPolarVector() const
  {
    const tVector<2, TElement, Cartesian> *that = reinterpret_cast<const tVector<2, TElement, Cartesian> *>(this);
    return tVector<2, TElement, Polar>(std::atan2(that->Y(), that->X()), that->Length());
  }

#ifdef _LIB_OIV_PRESENT_
  inline const SbVec2f GetCoinVector() const
  {
    const tVector<2, TElement, Cartesian> *that = reinterpret_cast<const tVector<3, TElement, Cartesian> *>(this);
    return SbVec2f(that->X(), that->Y());
  }
#endif

};

/*!
 *
 */
template <typename TElement>
class Conversions<3, TElement, Cartesian>
{
  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);

protected:

  inline Conversions() {}

public:

  inline const tVector<3, TElement, Polar> GetPolarVector() const
  {
    const tVector<3, TElement, Cartesian> *that = reinterpret_cast<const tVector<3, TElement, Cartesian> *>(this);
    TElement length = that->Length();
    return tVector<3, TElement, Polar>(std::atan2(that->Y(), that->X()), std::acos(that->Z() / length), length);
  }

#ifdef _LIB_OIV_PRESENT_
  inline const SbVec3f GetCoinVector() const
  {
    const tVector<3, TElement, Cartesian> *that = reinterpret_cast<const tVector<3, TElement, Cartesian> *>(this);
    return SbVec3f(that->X(), that->Y(), that->Z());
  }
#endif
};

/*!
 *
 */
template <typename TElement>
class Conversions<2, TElement, Polar>
{
  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);

protected:

  inline Conversions() {}

public:

  inline const tVector<2, TElement, Cartesian> GetCartesianVector() const
  {
    const tVector<2, TElement, Polar> *that = reinterpret_cast<const tVector<2, TElement, Polar> *>(this);
    return tVector<2, TElement, Cartesian>(that->Length() * std::cos(that->Alpha()), that->Length() * std::sin(that->Alpha()));
  }

#ifdef _LIB_OIV_PRESENT_
  inline const SbVec2f GetCoinVector() const
  {
    const tVector<2, TElement, Cartesian> *that = this->GetCartesianVector();
    return SbVec2f(that->X(), that->Y());
  }
#endif

};

/*!
 *
 */
template <typename TElement>
class Conversions<3, TElement, Polar>
{
  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);

protected:

  inline Conversions() {}

public:

  inline const tVector<3, TElement, Cartesian> GetCartesianVector() const
  {
    const tVector<3, TElement, Polar> *that = reinterpret_cast<const tVector<3, TElement, Polar> *>(this);
    TElement sin_alpha = std::sin(that->Alpha());
    TElement cos_alpha = std::cos(that->Alpha());
    TElement sin_beta = std::sin(that->Beta());
    TElement cos_beta = std::cos(that->Beta());
    return tVector<3, TElement, Cartesian>(that->Length() * cos_alpha * sin_beta, that->Length() * sin_alpha * sin_beta, that->Length() * cos_beta);
  }

#ifdef _LIB_OIV_PRESENT_
  inline const SbVec3f GetCoinVector() const
  {
    const tVector<2, TElement, Cartesian> *that = this->GetCartesianVector();
    return SbVec3f(that->X(), that->Y(), that->Z());
  }
#endif

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
