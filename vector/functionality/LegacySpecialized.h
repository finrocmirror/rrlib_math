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
/*!\file    LegacySpecialized.h
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

#ifndef _rrlib_math_vector_functionality_LegacySpecialized_h_
#define _rrlib_math_vector_functionality_LegacySpecialized_h_

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
class LegacySpecialized
{
  LegacySpecialized(const LegacySpecialized &);
  LegacySpecialized &operator = (const LegacySpecialized &);

protected:

  inline LegacySpecialized() {}

};

/*!
 *
 */
template <typename TElement>
class LegacySpecialized<2, TElement, Cartesian>
{
  typedef tVector<2, TElement, Cartesian> tVectorType;

  LegacySpecialized(const LegacySpecialized &);
  LegacySpecialized &operator = (const LegacySpecialized &);

protected:

  inline LegacySpecialized() {}

public:

  inline const tVectorType rotate(TElement angle) const
  {
    return reinterpret_cast<const tVectorType *>(this)->Rotated(angle);
  }

  inline const tVectorType rotate(TElement sine, TElement cosine) const
  {
    return reinterpret_cast<const tVectorType *>(this)->Rotated(sine, cosine);
  }

  inline void rotateInPlace(TElement angle)
  {
    reinterpret_cast<tVectorType *>(this)->Rotate(angle);
  }

  inline const TElement operator % (const tVectorType &other) const
  {
    const tVectorType *that = reinterpret_cast<const tVectorType *>(this);
    return that->X() * other.Y() - that->Y() * other.X();
  }

  inline const tVectorType polarSigned(TElement radius = 1) const
  {
    tVector<2, TElement, Polar> temp = GetPolarSignedFromCartesian(reinterpret_cast<const tVectorType &>(*this), radius);
    return *reinterpret_cast<tVector<2, TElement, Cartesian> *>(&temp);
//    const tVectorType *that = reinterpret_cast<const tVectorType *>(this);
//    tVectorType temp(*that);
//    temp.Normalize();
//    temp.Set(temp.Y() >= 0 ? std::acos(temp.X()) : -acos(temp.X()), radius);
//    return temp;
  }

  inline const tVectorType &swap()
  {
    tVectorType *that = reinterpret_cast<tVectorType *>(this);
    that->Set(that->Y(), that->X());
    return *that;
  }

  inline const tVectorType FlipDirection() const
  {
    return -*reinterpret_cast<const tVectorType *>(this);
  }

};

/*!
 *
 */
template <typename TElement>
class LegacySpecialized<3, TElement, Cartesian>
{
  typedef tVector<3, TElement, Cartesian> tVectorType;

  LegacySpecialized(const LegacySpecialized &);
  LegacySpecialized &operator = (const LegacySpecialized &);

protected:

  inline LegacySpecialized() {}

public:

  inline const tVectorType rotate(TElement angle, const tVectorType &axis) const
  {
    return reinterpret_cast<const tVectorType *>(this)->Rotated(angle, axis);
  }

  inline const tVectorType operator % (const tVectorType &other) const
  {
    return reinterpret_cast<const tVectorType *>(this)->CrossMultiplied(other);
  }

};

/*!
 *
 */
template <typename TElement>
class LegacySpecialized<6, TElement, Cartesian>
{
  typedef tVector<6, TElement, Cartesian> tVectorType;

  LegacySpecialized(const LegacySpecialized &);
  LegacySpecialized &operator = (const LegacySpecialized &);

protected:

  inline LegacySpecialized() {}

public:

  inline const TElement X() const
  {
    return reinterpret_cast<const TElement *>(this)[0];
  }
  inline TElement &X()
  {
    return reinterpret_cast<TElement *>(this)[0];
  }
  inline const TElement Y() const
  {
    return reinterpret_cast<const TElement *>(this)[1];
  }
  inline TElement &Y()
  {
    return reinterpret_cast<TElement *>(this)[1];
  }
  inline const TElement Z() const
  {
    return reinterpret_cast<const TElement *>(this)[2];
  }
  inline TElement &Z()
  {
    return reinterpret_cast<TElement *>(this)[2];
  }
  inline const TElement Roll() const
  {
    return reinterpret_cast<const TElement *>(this)[3];
  }
  inline TElement &Roll()
  {
    return reinterpret_cast<TElement *>(this)[3];
  }
  inline const TElement Pitch() const
  {
    return reinterpret_cast<const TElement *>(this)[4];
  }
  inline TElement &Pitch()
  {
    return reinterpret_cast<TElement *>(this)[4];
  }
  inline const TElement Yaw() const
  {
    return reinterpret_cast<const TElement *>(this)[5];
  }
  inline TElement &Yaw()
  {
    return reinterpret_cast<TElement *>(this)[5];
  }

};


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
