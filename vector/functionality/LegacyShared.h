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
/*!\file    LegacyShared.h
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

#ifndef _rrlib_math_vector_functionality_LegacyShared_h_
#define _rrlib_math_vector_functionality_LegacyShared_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
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
class LegacyShared
{
  typedef tVector<Tdimension, TElement, TData> tVectorType;

  LegacyShared(const LegacyShared &);
  LegacyShared &operator = (const LegacyShared &);

protected:

  inline LegacyShared() {}

};

template <size_t Tdimension, typename TElement>
class LegacyShared<Tdimension, TElement, Cartesian>
{
  typedef tVector<Tdimension, TElement, Cartesian> tVectorType;

  LegacyShared(const LegacyShared &);
  LegacyShared &operator = (const LegacyShared &);

protected:

  inline LegacyShared() {}

public:

  inline const TElement length() const __attribute__((deprecated));

  inline const TElement length_square() const __attribute__((deprecated));

  inline const tVectorType norm() const __attribute__((deprecated));

  inline const tVectorType norm(TElement &length) const __attribute__((deprecated));

  inline const tVectorType operator ^(const tVectorType &other) const __attribute__((deprecated));

  inline const tVectorType sp(const tVectorType &other) const __attribute__((deprecated));

  const tVectorType &cartesian() __attribute__((deprecated));

  void PolarToCartesian() __attribute__((deprecated));

  const tVectorType polar() __attribute__((deprecated));

  void CartesianToPolar() __attribute__((deprecated));

};



template <size_t Tdimension, typename TElement>
inline const TElement LegacyShared<Tdimension, TElement, Cartesian>::length() const
{
  return reinterpret_cast<const tVectorType *>(this)->Length();
}

template <size_t Tdimension, typename TElement>
inline const TElement LegacyShared<Tdimension, TElement, Cartesian>::length_square() const
{
  return reinterpret_cast<const tVectorType *>(this)->SquaredLength();
}

template <size_t Tdimension, typename TElement>
inline const tVector<Tdimension, TElement, Cartesian> LegacyShared<Tdimension, TElement, Cartesian>::norm() const
{
  return reinterpret_cast<const tVectorType *>(this)->Normalized();
}

template <size_t Tdimension, typename TElement>
inline const tVector<Tdimension, TElement, Cartesian> LegacyShared<Tdimension, TElement, Cartesian>::norm(TElement &length) const
{
  length = reinterpret_cast<const tVectorType *>(this)->Length();
  return reinterpret_cast<const tVectorType *>(this)->Normalized();
}

template <size_t Tdimension, typename TElement>
inline const tVector<Tdimension, TElement, Cartesian> LegacyShared<Tdimension, TElement, Cartesian>::operator ^(const tVectorType &other) const
{
  return reinterpret_cast<const tVectorType *>(this)->Projected(other);
}

template <size_t Tdimension, typename TElement>
inline const tVector<Tdimension, TElement, Cartesian> LegacyShared<Tdimension, TElement, Cartesian>::sp(const tVectorType &other) const
{
  return reinterpret_cast<const tVectorType *>(this)->SchurMultiplied(other);
}

template <size_t Tdimension, typename TElement>
const tVector<Tdimension, TElement, Cartesian> &LegacyShared<Tdimension, TElement, Cartesian>::cartesian()
{
  reinterpret_cast<tVectorType &>(*this) = reinterpret_cast<tVector<Tdimension, TElement, vector::Polar> *>(this)->GetCartesianVector();
  return *this;
}

template <size_t Tdimension, typename TElement>
void LegacyShared<Tdimension, TElement, Cartesian>::PolarToCartesian()
{
  reinterpret_cast<tVectorType &>(*this) = reinterpret_cast<tVector<Tdimension, TElement, vector::Polar> *>(this)->GetCartesianVector();
}

template <size_t Tdimension, typename TElement>
const tVector<Tdimension, TElement, Cartesian> LegacyShared<Tdimension, TElement, Cartesian>::polar()
{
  reinterpret_cast<tVector<Tdimension, TElement, vector::Polar> &>(*this) = reinterpret_cast<tVectorType *>(this)->GetPolarVector();
  return *reinterpret_cast<tVectorType *>(this);
}

template <size_t Tdimension, typename TElement>
void LegacyShared<Tdimension, TElement, Cartesian>::CartesianToPolar()
{
  reinterpret_cast<tVector<Tdimension, TElement, vector::Polar> &>(*this) = reinterpret_cast<tVectorType *>(this)->GetPolarVector();
}




template <size_t Tdimension, typename TCartesianElement, typename TPolarElement>
inline void GetCartesianFromPolar(tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian, const tVector<Tdimension, TPolarElement, vector::Cartesian> &polar) __attribute__((deprecated));
template <size_t Tdimension, typename TCartesianElement, typename TPolarElement>
inline void GetCartesianFromPolar(tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian, const tVector<Tdimension, TPolarElement, vector::Cartesian> &polar)
{
  cartesian = reinterpret_cast<const tVector<Tdimension, TPolarElement, vector::Polar> *>(&polar)->GetCartesianVector();
}

template <size_t Tdimension, typename TPolarElement, typename TCartesianElement>
inline void GetPolarFromCartesian(tVector<Tdimension, TPolarElement, vector::Cartesian> &polar, const tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian) __attribute__((deprecated));
template <size_t Tdimension, typename TPolarElement, typename TCartesianElement>
inline void GetPolarFromCartesian(tVector<Tdimension, TPolarElement, vector::Cartesian> &polar, const tVector<Tdimension, TCartesianElement, vector::Cartesian> &cartesian)
{
  tVector<Tdimension, TCartesianElement, vector::Polar> temp = cartesian.GetPolarVector();
  polar = *reinterpret_cast<tVector<Tdimension, TCartesianElement, vector::Cartesian> *>(&temp);
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
