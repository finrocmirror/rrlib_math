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
/*!\file    Rotation.h
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

#ifndef _rrlib_math_vector_functionality_Rotation_h_
#define _rrlib_math_vector_functionality_Rotation_h_

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
template <size_t Tdimension, typename TElement, template <size_t, typename> class TData>
class Rotation
{
  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

protected:

  inline Rotation() {}

};

/*!
 *
 */
template <typename TElement>
class Rotation<2, TElement, Cartesian>
{
  typedef tVector<2, TElement, Cartesian> tVectorType;

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

protected:

  inline Rotation() {}

public:

  inline void Rotate(tAngleRad angle)
  {
    this->Rotate(angle.Sine(), angle.Cosine());
  }

  inline void Rotate(double sine, double cosine)
  {
    tVectorType *that = reinterpret_cast<tVectorType *>(this);
    that->Set(cosine * that->X() - sine * that->Y(),
              sine * that->X() + cosine * that->Y());
  }

  inline const tVectorType Rotated(tAngleRad angle) const
  {
    const tVectorType *that = reinterpret_cast<const tVectorType *>(this);
    tVectorType temp(*that);
    temp.Rotate(angle);
    return temp;
  }

  inline const tVectorType Rotated(double sine, double cosine) const
  {
    const tVectorType *that = reinterpret_cast<const tVectorType *>(this);
    tVectorType temp(*that);
    temp.Rotate(sine, cosine);
    return temp;
  }

};

/*!
 *
 */
template <typename TElement>
class Rotation<3, TElement, Cartesian>
{
  typedef tVector<3, TElement, Cartesian> tVectorType;

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

protected:

  inline Rotation() {}

public:

  template <typename TAxisElement>
  inline void Rotate(tAngleRad angle, const tVector<3, TAxisElement, Cartesian> &axis)
  {
    tVectorType *that = reinterpret_cast<tVectorType *>(this);

    double si = angle.Sine();
    double co = angle.Cosine();
    double mco = 1.0 - co;
    tVector<3, TAxisElement, Cartesian> normalized_axis(axis.Normalized());

    that->Set(co * that->X() + (normalized_axis.X() * mco) * normalized_axis * *that + si *(normalized_axis.Y() * that->Z() - normalized_axis.Z() * that->Y()),
              co * that->Y() + (normalized_axis.Y() * mco) * normalized_axis * *that + si *(normalized_axis.Z() * that->X() - normalized_axis.X() * that->Z()),
              co * that->Z() + (normalized_axis.Z() * mco) * normalized_axis * *that + si *(normalized_axis.X() * that->Y() - normalized_axis.Y() * that->X()));
  }

  template <typename TAxisElement>
  inline const tVectorType Rotated(tAngleRad angle, const tVector<3, TAxisElement, Cartesian> &axis) const
  {
    const tVectorType *that = reinterpret_cast<const tVectorType *>(this);
    tVectorType temp(*that);
    temp.Rotate(angle, axis);
    return temp;
  }

};

/*!
 *
 */
template <typename TElement>
class Rotation<2, TElement, Polar>
{
  typedef tVector<2, TElement, Polar> tVectorType;

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

protected:

  inline Rotation() {}

public:

};

/*!
 *
 */
template <typename TElement>
class Rotation<3, TElement, Polar>
{
  typedef tVector<3, TElement, Polar> tVectorType;

  Rotation(const Rotation &);
  Rotation &operator = (const Rotation &);

protected:

  inline Rotation() {}

public:

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
