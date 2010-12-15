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

  inline const TElement length() const
  {
    return reinterpret_cast<const tVectorType *>(this)->Length();
  }

  inline const TElement length_square() const
  {
    return reinterpret_cast<const tVectorType *>(this)->SquaredLength();
  }

  inline const tVectorType norm() const
  {
    return reinterpret_cast<const tVectorType *>(this)->Normalized();
  }

  inline const tVectorType norm(TElement &length) const
  {
    length = reinterpret_cast<const tVectorType *>(this)->Length();
    return reinterpret_cast<const tVectorType *>(this)->Normalized();
  }

  inline const tVectorType operator ^(const tVectorType &other) const
  {
    return reinterpret_cast<const tVectorType *>(this)->Projected(other);
  }

  inline const tVectorType sp(const tVectorType &other) const
  {
    return reinterpret_cast<const tVectorType *>(this)->SchurMultiplied(other);
  }

  const tVectorType &cartesian()
  {
    reinterpret_cast<tVectorType &>(*this) = reinterpret_cast<tVector<Tdimension, TElement, vector::Polar> *>(this)->GetCartesianVector();
    return *this;
  }
  void PolarToCartesian()
  {
    reinterpret_cast<tVectorType &>(*this) = reinterpret_cast<tVector<Tdimension, TElement, vector::Polar> *>(this)->GetCartesianVector();
  }
  const tVectorType Cartesian() const
  {
    return reinterpret_cast<const tVector<Tdimension, TElement, vector::Polar> *>(this)->GetCartesianVector();
  }

  const tVector<Tdimension, TElement, Polar> polar()
  {
    reinterpret_cast<tVector<Tdimension, TElement, vector::Polar> &>(*this) = this->GetCartesianVector();
    return *reinterpret_cast<tVectorType>(this);
  }

  void CartesianToPolar()
  {
    reinterpret_cast<tVector<Tdimension, TElement, vector::Polar> &>(*this) = this->GetCartesianVector();
  }

  const tVector<Tdimension, TElement, Polar> Polar() const
  {
    return reinterpret_cast<const tVectorType *>(this)->GetPolarVector();
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
