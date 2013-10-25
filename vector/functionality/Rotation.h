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
/*!\file    rrlib/math/vector/functionality/Rotation.h
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

#ifndef __rrlib__math__vector__functionality__Rotation_h__
#define __rrlib__math__vector__functionality__Rotation_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tAngle.h"

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
template <size_t Tdimension, typename TElement, template <size_t, typename, typename ...> class TData, typename ... TAdditionalDataParameters>
class Rotation
{

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Rotation()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

};

/*!
 *
 */
template <typename TElement>
class Rotation<2, TElement, Cartesian>
{
  typedef math::tVector<2, TElement, Cartesian> tVector;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline void Rotate(tAngleRad angle)
  {
    this->Rotate(angle.Sine(), angle.Cosine());
  }

  inline void Rotate(double sine, double cosine)
  {
    tVector *that = reinterpret_cast<tVector *>(this);
    that->Set(cosine * that->X() - sine * that->Y(),
              sine * that->X() + cosine * that->Y());
  }

  inline const tVector Rotated(tAngleRad angle) const
  {
    const tVector *that = reinterpret_cast<const tVector *>(this);
    tVector temp(*that);
    temp.Rotate(angle);
    return temp;
  }

  inline const tVector Rotated(double sine, double cosine) const
  {
    const tVector *that = reinterpret_cast<const tVector *>(this);
    tVector temp(*that);
    temp.Rotate(sine, cosine);
    return temp;
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Rotation()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

};

/*!
 *
 */
template <typename TElement>
class Rotation<3, TElement, Cartesian>
{
  typedef math::tVector<3, TElement, Cartesian> tVector;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TAxisElement>
  inline void Rotate(tAngleRad angle, const math::tVector<3, TAxisElement, Cartesian> &axis)
  {
    tVector *that = reinterpret_cast<tVector *>(this);

    double si = angle.Sine();
    double co = angle.Cosine();
    double mco = 1.0 - co;
    math::tVector<3, TAxisElement, Cartesian> normalized_axis(axis.Normalized());

    that->Set(co * that->X() + (normalized_axis.X() * mco) * normalized_axis **that + si * (normalized_axis.Y() * that->Z() - normalized_axis.Z() * that->Y()),
              co * that->Y() + (normalized_axis.Y() * mco) * normalized_axis **that + si * (normalized_axis.Z() * that->X() - normalized_axis.X() * that->Z()),
              co * that->Z() + (normalized_axis.Z() * mco) * normalized_axis **that + si * (normalized_axis.X() * that->Y() - normalized_axis.Y() * that->X()));
  }

  template <typename TAxisElement>
  inline const tVector Rotated(tAngleRad angle, const math::tVector<3, TAxisElement, Cartesian> &axis) const
  {
    const tVector *that = reinterpret_cast<const tVector *>(this);
    tVector temp(*that);
    temp.Rotate(angle, axis);
    return temp;
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Rotation()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

};

/*!
 *
 */
template <typename TElement, typename TUnitPolicy, typename TSignPolicy>
class Rotation<2, TElement, Polar, TUnitPolicy, TSignPolicy>
{
  typedef math::tVector<2, TElement, Polar, TUnitPolicy, TSignPolicy> tVector;

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Rotation()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

};

/*!
 *
 */
template <typename TElement, typename TUnitPolicy, typename TSignPolicy>
class Rotation<3, TElement, Polar, TUnitPolicy, TSignPolicy>
{
  typedef math::tVector<3, TElement, Polar, TUnitPolicy, TSignPolicy> tVector;

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Rotation()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
