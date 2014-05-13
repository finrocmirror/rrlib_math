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
/*!\file    rrlib/math/matrix/functionality/LegacySpecialized.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-21
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__matrix__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef __rrlib__math__matrix__functionality__LegacySpecialized_h__
#define __rrlib__math__matrix__functionality__LegacySpecialized_h__

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
namespace matrix
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
template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TData>
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
template <typename TElement, template <size_t, size_t, typename> class TData>
class LegacySpecialized<3, 3, TElement, TData>
{
  typedef math::tMatrix<3, 3, TElement, TData> tMatrix;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TVectorElement>
  const tVector < 2, decltype((TElement() * TVectorElement()) + (TElement() * TVectorElement())), vector::Cartesian > MultHomogeneous(const tVector<2, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  template <typename TVectorElement>
  void MultHomogeneousInPlace(tVector<2, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  void GetRotationAxis(tVector<3, TElement, vector::Cartesian> &axis, TElement &angle) const __attribute__((deprecated));

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

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
const tVector < 2, decltype((TElement() * TVectorElement()) + (TElement() * TVectorElement())), vector::Cartesian > LegacySpecialized<3, 3, TElement, TData>::MultHomogeneous(const tVector<2, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
  return that->MultiplyHomogeneously(vector);
}

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
void LegacySpecialized<3, 3, TElement, TData>::MultHomogeneousInPlace(tVector<2, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
  vector = that->MultiplyHomogeneously(vector);
}

template <typename TElement, template <size_t, size_t, typename> class TData>
void LegacySpecialized<3, 3, TElement, TData>::GetRotationAxis(tVector<3, TElement, vector::Cartesian> &axis, TElement &angle) const
{
  const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
  that->GetRotation(axis, angle);
}

/*!
 *
 */
template <typename TElement, template <size_t, size_t, typename> class TData>
class LegacySpecialized<4, 4, TElement, TData>
{
  typedef math::tMatrix<4, 4, TElement, TData> tMatrix;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename TVectorElement>
  const tVector < 2, decltype((TElement() * TVectorElement()) + (TElement() * TVectorElement())), vector::Cartesian > MultHomogeneous(const tVector<2, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  template <typename TVectorElement>
  void MultHomogeneousInPlace(tVector<2, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  template <typename TVectorElement>
  const tVector < 3, decltype((TElement() * TVectorElement()) + (TElement() * TVectorElement())), vector::Cartesian > MultHomogeneous(const tVector<3, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  template <typename TVectorElement>
  void MultHomogeneousInPlace(tVector<3, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  void GetRotationAxis(tVector<3, TElement, vector::Cartesian> &axis, TElement &angle) const __attribute__((deprecated));

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

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
const tVector < 2, decltype((TElement() * TVectorElement()) + (TElement() * TVectorElement())), vector::Cartesian > LegacySpecialized<4, 4, TElement, TData>::MultHomogeneous(const tVector<2, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
  return tVector < 2, decltype((TElement() * TVectorElement()) + (TElement() * TVectorElement())), vector::Cartesian > (that->MultiplyHomogeneously(tVector<3, TVectorElement, vector::Cartesian>(vector)));
}

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
void LegacySpecialized<4, 4, TElement, TData>::MultHomogeneousInPlace(tVector<2, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
  vector = that->MultHomogeneous(vector);
}

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
const tVector < 3, decltype((TElement() * TVectorElement()) + (TElement() * TVectorElement())), vector::Cartesian > LegacySpecialized<4, 4, TElement, TData>::MultHomogeneous(const tVector<3, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
  return that->MultiplyHomogeneously(vector);
}

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
void LegacySpecialized<4, 4, TElement, TData>::MultHomogeneousInPlace(tVector<3, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
  vector = that->MultHomogeneous(vector);
}

template <typename TElement, template <size_t, size_t, typename> class TData>
void LegacySpecialized<4, 4, TElement, TData>::GetRotationAxis(tVector<3, TElement, vector::Cartesian> &axis, TElement &angle) const
{
  const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
  that->GetRotation(axis, angle);
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
