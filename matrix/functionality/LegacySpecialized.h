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
 * \date    2010-11-21
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_matrix_include_guard_
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef _rrlib_math_matrix_functionality_LegacySpecialized_h_
#define _rrlib_math_matrix_functionality_LegacySpecialized_h_

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
  LegacySpecialized(const LegacySpecialized &);
  LegacySpecialized &operator = (const LegacySpecialized &);

protected:

  inline LegacySpecialized() {}

};

/*!
 *
 */
template <typename TElement, template <size_t, size_t, typename> class TData>
class LegacySpecialized<3, 3, TElement, TData>
{
  typedef tMatrix<3, 3, TElement, TData> tMatrixType;

  LegacySpecialized(const LegacySpecialized &);
  LegacySpecialized &operator = (const LegacySpecialized &);

protected:

  inline LegacySpecialized() {}

public:

  template <typename TVectorElement>
  const tVector<2, typename until_0x::Auto<TElement, TVectorElement>::type, vector::Cartesian> MultHomogeneous(const tVector<2, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  template <typename TVectorElement>
  void MultHomogeneousInPlace(tVector<2, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  void GetRotationAxis(tVector<3, TElement, vector::Cartesian> &axis, TElement &angle) const __attribute__((deprecated));

};

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
const tVector<2, typename until_0x::Auto<TElement, TVectorElement>::type, vector::Cartesian> LegacySpecialized<3, 3, TElement, TData>::MultHomogeneous(const tVector<2, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
  return that->MultiplyHomogeneously(vector);
}

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
void LegacySpecialized<3, 3, TElement, TData>::MultHomogeneousInPlace(tVector<2, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
  vector = that->MultiplyHomogeneously(vector);
}

template <typename TElement, template <size_t, size_t, typename> class TData>
void LegacySpecialized<3, 3, TElement, TData>::GetRotationAxis(tVector<3, TElement, vector::Cartesian> &axis, TElement &angle) const
{
  const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
  that->GetRotation(axis, angle);
}

/*!
 *
 */
template <typename TElement, template <size_t, size_t, typename> class TData>
class LegacySpecialized<4, 4, TElement, TData>
{
  typedef tMatrix<4, 4, TElement, TData> tMatrixType;

  LegacySpecialized(const LegacySpecialized &);
  LegacySpecialized &operator = (const LegacySpecialized &);

protected:

  inline LegacySpecialized() {}

public:

  template <typename TVectorElement>
  const tVector<2, typename until_0x::Auto<TElement, TVectorElement>::type, vector::Cartesian> MultHomogeneous(const tVector<2, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  template <typename TVectorElement>
  void MultHomogeneousInPlace(tVector<2, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  template <typename TVectorElement>
  const tVector<3, typename until_0x::Auto<TElement, TVectorElement>::type, vector::Cartesian> MultHomogeneous(const tVector<3, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  template <typename TVectorElement>
  void MultHomogeneousInPlace(tVector<3, TVectorElement, vector::Cartesian> &vector) const __attribute__((deprecated));

  void GetRotationAxis(tVector<3, TElement, vector::Cartesian> &axis, TElement &angle) const __attribute__((deprecated));

  void SetIdentity() __attribute__((deprecated));

};

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
const tVector<2, typename until_0x::Auto<TElement, TVectorElement>::type, vector::Cartesian> LegacySpecialized<4, 4, TElement, TData>::MultHomogeneous(const tVector<2, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
  return tVector<2, typename until_0x::Auto<TElement, TVectorElement>::type, vector::Cartesian>(that->MultiplyHomogeneously(tVector<3, TVectorElement, vector::Cartesian>(vector)));
}

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
void LegacySpecialized<4, 4, TElement, TData>::MultHomogeneousInPlace(tVector<2, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
  vector = that->MultHomogeneous(vector);
}

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
const tVector<3, typename until_0x::Auto<TElement, TVectorElement>::type, vector::Cartesian> LegacySpecialized<4, 4, TElement, TData>::MultHomogeneous(const tVector<3, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
  return that->MultiplyHomogeneously(vector);
}

template <typename TElement, template <size_t, size_t, typename> class TData>
template <typename TVectorElement>
void LegacySpecialized<4, 4, TElement, TData>::MultHomogeneousInPlace(tVector<3, TVectorElement, vector::Cartesian> &vector) const
{
  const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
  vector = that->MultHomogeneous(vector);
}

template <typename TElement, template <size_t, size_t, typename> class TData>
void LegacySpecialized<4, 4, TElement, TData>::GetRotationAxis(tVector<3, TElement, vector::Cartesian> &axis, TElement &angle) const
{
  const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
  that->GetRotation(axis, angle);
}

template <typename TElement, template <size_t, size_t, typename> class TData>
void LegacySpecialized<4, 4, TElement, TData>::SetIdentity()
{
  tMatrixType *that = reinterpret_cast<tMatrixType *>(this);
  *that = tMatrixType::Identity();
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
