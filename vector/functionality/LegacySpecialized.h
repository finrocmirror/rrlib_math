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

  inline const tVectorType rotate(TElement angle) const __attribute__((deprecated));

  inline const tVectorType rotate(TElement sine, TElement cosine) const __attribute__((deprecated));

  inline void rotateInPlace(TElement angle) __attribute__((deprecated));

  inline const TElement operator % (const tVectorType &other) const __attribute__((deprecated));

  inline const tVectorType polarSigned(TElement radius = 1) const __attribute__((deprecated));

  inline const tVectorType &swap() __attribute__((deprecated));

  inline const tVectorType FlipDirection() const __attribute__((deprecated));

  float GetDistance2Line(float angle, const tVectorType &line_start, const tVectorType &line_end, float max) __attribute__((deprecated));

};


template <typename TElement>
inline const tVector<2, TElement, Cartesian> LegacySpecialized<2, TElement, Cartesian>::rotate(TElement angle) const
{
  return reinterpret_cast<const tVectorType *>(this)->Rotated(angle);
}

template <typename TElement>
inline const tVector<2, TElement, Cartesian> LegacySpecialized<2, TElement, Cartesian>::rotate(TElement sine, TElement cosine) const
{
  return reinterpret_cast<const tVectorType *>(this)->Rotated(sine, cosine);
}

template <typename TElement>
inline void LegacySpecialized<2, TElement, Cartesian>::rotateInPlace(TElement angle)
{
  reinterpret_cast<tVectorType *>(this)->Rotate(angle);
}

template <typename TElement>
inline const TElement LegacySpecialized<2, TElement, Cartesian>::operator % (const tVectorType &other) const
{
  const tVectorType *that = reinterpret_cast<const tVectorType *>(this);
  return that->X() * other.Y() - that->Y() * other.X();
}

template <typename TElement>
inline const tVector<2, TElement, Cartesian> LegacySpecialized<2, TElement, Cartesian>::polarSigned(TElement radius) const
{
  tVector<2, TElement, Polar> temp = GetPolarSignedFromCartesian(reinterpret_cast<const tVectorType &>(*this), radius);
  return *reinterpret_cast<tVector<2, TElement, Cartesian> *>(&temp);
//    const tVectorType *that = reinterpret_cast<const tVectorType *>(this);
//    tVectorType temp(*that);
//    temp.Normalize();
//    temp.Set(temp.Y() >= 0 ? std::acos(temp.X()) : -std::acos(temp.X()), radius);
//    return temp;
}

template <typename TElement>
inline const tVector<2, TElement, Cartesian> &LegacySpecialized<2, TElement, Cartesian>::swap()
{
  tVectorType *that = reinterpret_cast<tVectorType *>(this);
  that->Set(that->Y(), that->X());
  return *that;
}

template <typename TElement>
inline const tVector<2, TElement, Cartesian> LegacySpecialized<2, TElement, Cartesian>::FlipDirection() const
{
  return -*reinterpret_cast<const tVectorType *>(this);
}

template <typename TElement>
float LegacySpecialized<2, TElement, Cartesian>::GetDistance2Line(float angle, const tVectorType &line_start, const tVectorType &line_end, float max)
{
  tVectorType *that = reinterpret_cast<tVectorType *>(this);

  tVectorType line_direction = (line_end - line_start).Normalized();
  // direction to look into
  tVectorType start_direction(std::cos(angle), std::sin(angle));

  // parallelogram area of unit vectors of scan and line
  // b>0 : towards line
  // b=0 : parallel to line
  // b<0 : away from line
  double b = CrossProduct(tVector<3, TElement, Cartesian>(start_direction), tVector<3, TElement, Cartesian>(line_direction)).Z();
  if (b > 0)
  {
    // direction towards edge (not parallel or away from)
    double a = CrossProduct(tVector<3, TElement, Cartesian>(line_direction), tVector<3, TElement, Cartesian>(*that - line_start)).Z();
    if (a > 0)
    {
      // Division of parallelograms is equal to distance (paint it, if you don't believe it :)
      float dist = a / b;
      // vector of distance
      tVectorType intersection = *that + dist * start_direction;

      //check if really between start and endpoint of line
      if (dist < max &&
          ((intersection.X() >= line_start.X() && intersection.X() <= line_end.X()) ||
           (intersection.X() <= line_start.X() && intersection.X() >= line_end.X()) ||
           line_start.X() == line_end.X()) &&
          ((intersection.Y() >= line_start.Y() && intersection.Y() <= line_end.Y()) ||
           (intersection.Y() <= line_start.Y() && intersection.Y() >= line_end.Y()) ||
           line_start.Y() == line_end.Y()))
      {
        return dist;
      }
    }
  }
  return -1;
}


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

  inline const tVectorType rotate(TElement angle, const tVectorType &axis) const __attribute__((deprecated));

  inline const tVectorType operator % (const tVectorType &other) const __attribute__((deprecated));

};

template <typename TElement>
inline const tVector<3, TElement, Cartesian> LegacySpecialized<3, TElement, Cartesian>::rotate(TElement angle, const tVectorType &axis) const
{
  return reinterpret_cast<const tVectorType *>(this)->Rotated(angle, axis);
}

template <typename TElement>
inline const tVector<3, TElement, Cartesian> LegacySpecialized<3, TElement, Cartesian>::operator % (const tVectorType &other) const
{
  return reinterpret_cast<const tVectorType *>(this)->CrossMultiplied(other);
}

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
