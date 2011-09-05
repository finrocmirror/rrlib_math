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
#ifndef __rrlib__math__vector__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef __rrlib__math__vector__functionality__LegacySpecialized_h__
#define __rrlib__math__vector__functionality__LegacySpecialized_h__

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

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline LegacySpecialized() {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  LegacySpecialized(const LegacySpecialized &);
  LegacySpecialized &operator = (const LegacySpecialized &);

};

/*!
 *
 */
template <typename TElement>
class LegacySpecialized<2, TElement, Cartesian>
{
  typedef math::tVector<2, TElement, Cartesian> tVector;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline const tVector rotate(TElement angle) const __attribute__((deprecated));

  inline const tVector rotate(TElement sine, TElement cosine) const __attribute__((deprecated));

  inline void rotateInPlace(TElement angle) __attribute__((deprecated));

  inline const TElement operator % (const tVector &other) const __attribute__((deprecated));

  inline const tVector polarSigned(TElement radius = 1) const __attribute__((deprecated));

  inline const tVector &swap() __attribute__((deprecated));

  inline const tVector FlipDirection() const __attribute__((deprecated));

  float GetDistance2Line(float angle, const tVector &line_start, const tVector &line_end, float max) __attribute__((deprecated));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline LegacySpecialized() {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  LegacySpecialized(const LegacySpecialized &);
  LegacySpecialized &operator = (const LegacySpecialized &);

};


template <typename TElement>
inline const math::tVector<2, TElement, Cartesian> LegacySpecialized<2, TElement, Cartesian>::rotate(TElement angle) const
{
  return reinterpret_cast<const tVector *>(this)->Rotated(angle);
}

template <typename TElement>
inline const math::tVector<2, TElement, Cartesian> LegacySpecialized<2, TElement, Cartesian>::rotate(TElement sine, TElement cosine) const
{
  return reinterpret_cast<const tVector *>(this)->Rotated(sine, cosine);
}

template <typename TElement>
inline void LegacySpecialized<2, TElement, Cartesian>::rotateInPlace(TElement angle)
{
  reinterpret_cast<tVector *>(this)->Rotate(angle);
}

template <typename TElement>
inline const TElement LegacySpecialized<2, TElement, Cartesian>::operator % (const tVector &other) const
{
  const tVector *that = reinterpret_cast<const tVector *>(this);
  return that->X() * other.Y() - that->Y() * other.X();
}

template <typename TElement>
inline const math::tVector<2, TElement, Cartesian> LegacySpecialized<2, TElement, Cartesian>::polarSigned(TElement radius) const
{
  math::tVector<2, TElement, Polar> temp = GetPolarSignedVectorFromCartesian(reinterpret_cast<const tVector &>(*this), radius);
  return *reinterpret_cast<math::tVector<2, TElement, Cartesian> *>(&temp);
//    const tVector *that = reinterpret_cast<const tVector *>(this);
//    tVector temp(*that);
//    temp.Normalize();
//    temp.Set(temp.Y() >= 0 ? std::acos(temp.X()) : -std::acos(temp.X()), radius);
//    return temp;
}

template <typename TElement>
inline const math::tVector<2, TElement, Cartesian> &LegacySpecialized<2, TElement, Cartesian>::swap()
{
  tVector *that = reinterpret_cast<tVector *>(this);
  that->Set(that->Y(), that->X());
  return *that;
}

template <typename TElement>
inline const math::tVector<2, TElement, Cartesian> LegacySpecialized<2, TElement, Cartesian>::FlipDirection() const
{
  return -*reinterpret_cast<const tVector *>(this);
}

template <typename TElement>
float LegacySpecialized<2, TElement, Cartesian>::GetDistance2Line(float angle, const tVector &line_start, const tVector &line_end, float max)
{

  // A CALL TO THIS METHOD WOULD BEST BE REPLACED BY USING rrlib_geometry STUFF

  tVector *that = reinterpret_cast<tVector *>(this);

  tVector line_direction = (line_end - line_start).Normalized();
  // direction to look into
  tVector start_direction(std::cos(angle), std::sin(angle));

  // parallelogram area of unit vectors of scan and line
  // b>0 : towards line
  // b=0 : parallel to line
  // b<0 : away from line
  double b = CrossProduct(math::tVector<3, TElement, Cartesian>(start_direction), math::tVector<3, TElement, Cartesian>(line_direction)).Z();
  if (b > 0)
  {
    // direction towards edge (not parallel or away from)
    double a = CrossProduct(math::tVector<3, TElement, Cartesian>(line_direction), math::tVector<3, TElement, Cartesian>(*that - line_start)).Z();
    if (a > 0)
    {
      // Division of parallelograms is equal to distance (paint it, if you don't believe it :)
      float dist = a / b;
      // vector of distance
      tVector intersection = *that + dist * start_direction;

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
  typedef math::tVector<3, TElement, Cartesian> tVector;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline const tVector rotate(TElement angle, const tVector &axis) const __attribute__((deprecated));

  inline const tVector operator % (const tVector &other) const __attribute__((deprecated));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline LegacySpecialized() {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  LegacySpecialized(const LegacySpecialized &);
  LegacySpecialized &operator = (const LegacySpecialized &);

};

template <typename TElement>
inline const math::tVector<3, TElement, Cartesian> LegacySpecialized<3, TElement, Cartesian>::rotate(TElement angle, const tVector &axis) const
{
  return reinterpret_cast<const tVector *>(this)->Rotated(angle, axis);
}

template <typename TElement>
inline const math::tVector<3, TElement, Cartesian> LegacySpecialized<3, TElement, Cartesian>::operator % (const tVector &other) const
{
  return reinterpret_cast<const tVector *>(this)->CrossMultiplied(other);
}

/*!
 *
 */
template <typename TElement>
class LegacySpecialized<6, TElement, Cartesian>
{
  typedef math::tVector<6, TElement, Cartesian> tVector;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
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

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline LegacySpecialized() {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  LegacySpecialized(const LegacySpecialized &);
  LegacySpecialized &operator = (const LegacySpecialized &);

};


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
