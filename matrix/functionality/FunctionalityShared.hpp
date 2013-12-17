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
/*!\file    rrlib/math/matrix/functionality/FunctionalityShared.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-08-25
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__matrix__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef __rrlib__math__matrix__functionality__FunctionalityShared_hpp__
#define __rrlib__math__matrix__functionality__FunctionalityShared_hpp__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstring>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_scalar.hpp>

#include "rrlib/util/variadic_templates.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"

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
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// FunctionalityShared constructors
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
FunctionalityShared<Trows, Tcolumns, TElement>::FunctionalityShared()
{
  std::memset(this, 0, sizeof(tMatrix));
}

template <size_t Trows, size_t Tcolumns, typename TElement>
FunctionalityShared<Trows, Tcolumns, TElement>::FunctionalityShared(const tMatrix &other)
{
  std::memcpy(this, &other, sizeof(tMatrix));
}

template <size_t Trows, size_t Tcolumns, typename TElement>
FunctionalityShared<Trows, Tcolumns, TElement>::FunctionalityShared(const TElement data[Trows * Tcolumns])
{
  reinterpret_cast<tMatrix *>(this)->SetFromArray(data);
}

template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename TOtherElement>
FunctionalityShared<Trows, Tcolumns, TElement>::FunctionalityShared(const math::tMatrix<Trows, Tcolumns, TOtherElement> &other)
{
  std::memset(this, 0, sizeof(tMatrix));
  for (size_t i = 0; i < sizeof(tMatrix) / sizeof(TElement); ++i)
  {
    reinterpret_cast<TElement *>(this)[i] = reinterpret_cast<const TOtherElement *>(&other)[i];
  }
}

template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename TLeftElement, typename TRightElement>
FunctionalityShared<Trows, Tcolumns, TElement>::FunctionalityShared(const tVector<Trows, TLeftElement, vector::Cartesian> &left, const tVector<Tcolumns, TRightElement, vector::Cartesian> &right)
{
  TElement data[Trows * Tcolumns];
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      data[row * Tcolumns + column] = left[row] * right[column];
    }
  }
  reinterpret_cast<tMatrix *>(this)->SetFromArray(data);
}

//----------------------------------------------------------------------
// FunctionalityShared operator []
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
const typename matrix::Full<Trows, Tcolumns, TElement>::Accessor FunctionalityShared<Trows, Tcolumns, TElement>::operator [](size_t row) const
{
  return const_cast<FunctionalityShared &>(*this)[row];
}

template <size_t Trows, size_t Tcolumns, typename TElement>
typename matrix::Full<Trows, Tcolumns, TElement>::Accessor FunctionalityShared<Trows, Tcolumns, TElement>::operator [](size_t row)
{
  return typename matrix::Full<Trows, Tcolumns, TElement>::Accessor(reinterpret_cast<TElement *>(this), row);
}

//----------------------------------------------------------------------
// FunctionalityShared Set
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename ... TValues>
void FunctionalityShared<Trows, Tcolumns, TElement>::Set(TValues... values)
{
  static_assert(sizeof...(values) == Trows * Tcolumns, "Wrong number of values given to store in matrix");

  TElement buffer[Trows * Tcolumns];
  TElement *p = buffer;
  util::ProcessVariadicValues([&p](TElement x)
  {
    *p++ = x;
  },
  values...);

  reinterpret_cast<tMatrix *>(this)->SetFromArray(buffer);
}

//----------------------------------------------------------------------
// FunctionalityShared SetFromMatrix
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename TOtherElement>
void FunctionalityShared<Trows, Tcolumns, TElement>::SetFromMatrix(const math::tMatrix<Trows, Tcolumns, TOtherElement> &source)
{
  TElement buffer[Trows * Tcolumns];
  for (size_t row  = 0; row < Trows; ++row)
  {
    for (size_t column  = 0; column < Tcolumns; ++column)
    {
      buffer[row * Tcolumns + column] = source[row][column];
    }
  }
  reinterpret_cast<tMatrix *>(this)->SetFromArray(buffer);
}

//----------------------------------------------------------------------
// FunctionalityShared SetIdentity
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
void FunctionalityShared<Trows, Tcolumns, TElement>::SetIdentity()
{
  *this = tMatrix::Identity();
}

//----------------------------------------------------------------------
// FunctionalityShared operator +=
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename TOtherElement>
const tMatrix<Trows, Tcolumns, TElement> &FunctionalityShared<Trows, Tcolumns, TElement>::operator += (const math::tMatrix<Trows, Tcolumns, TOtherElement> &other)
{
  tMatrix *that = reinterpret_cast<tMatrix *>(this);
  *that = *that + other;
  return *that;
}

//----------------------------------------------------------------------
// FunctionalityShared operator -=
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename TOtherElement>
const tMatrix<Trows, Tcolumns, TElement> &FunctionalityShared<Trows, Tcolumns, TElement>::operator -= (const math::tMatrix<Trows, Tcolumns, TOtherElement> &other)
{
  tMatrix *that = reinterpret_cast<tMatrix *>(this);
  *that = *that - other;
  return *that;
}

//----------------------------------------------------------------------
// FunctionalityShared operator *=
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename TOtherElement>
const tMatrix<Trows, Tcolumns, TElement> &FunctionalityShared<Trows, Tcolumns, TElement>::operator *= (const math::tMatrix<Tcolumns, Tcolumns, TOtherElement> &other)
{
  tMatrix *that = reinterpret_cast<tMatrix *>(this);
  *that = *that * other;
  return *that;
}

template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename TScalar>
const typename boost::enable_if<boost::is_scalar<TScalar>, tMatrix<Trows, Tcolumns, TElement>>::type &FunctionalityShared<Trows, Tcolumns, TElement>::operator *= (const TScalar &scalar)
{
  tMatrix *that = reinterpret_cast<tMatrix *>(this);
  *that = *that * scalar;
  return *that;
}

//----------------------------------------------------------------------
// FunctionalityShared operator /=
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename TScalar>
const typename boost::enable_if<boost::is_scalar<TScalar>, tMatrix<Trows, Tcolumns, TElement>>::type &FunctionalityShared<Trows, Tcolumns, TElement>::operator /= (const TScalar &scalar)
{
  tMatrix *that = reinterpret_cast<tMatrix *>(this);
  *that *= 1.0 / scalar;
  return *that;
}

//----------------------------------------------------------------------
// FunctionalityShared IsZero
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
const bool FunctionalityShared<Trows, Tcolumns, TElement>::IsZero(double epsilon) const
{
  for (size_t i = 0; i < sizeof(tMatrix) / sizeof(TElement); ++i)
  {
    if (std::abs(reinterpret_cast<const TElement *>(this)[i]) >= epsilon)
    {
      return false;
    }
  }
  return true;
}


//----------------------------------------------------------------------
// FunctionalityShared GetRow
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
tVector<Tcolumns, TElement, vector::Cartesian> FunctionalityShared<Trows, Tcolumns, TElement>::GetRow(size_t row) const
{
  tMatrix *that = reinterpret_cast<tMatrix *>(this);
  TElement result[Tcolumns];
  for (size_t column = 0; column < Tcolumns; ++column)
  {
    result[column] = (*that)[row][column];
  }
  return tVector<Tcolumns, TElement, vector::Cartesian>(result);
}

//----------------------------------------------------------------------
// FunctionalityShared GetColumn
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
tVector<Trows, TElement, vector::Cartesian> FunctionalityShared<Trows, Tcolumns, TElement>::GetColumn(size_t column) const
{
  tMatrix *that = reinterpret_cast<tMatrix *>(this);
  TElement result[Trows];
  for (size_t row = 0; column < Trows; ++row)
  {
    result[row] = (*that)[row][column];
  }
  return tVector<Trows, TElement, vector::Cartesian>(result);
}

//----------------------------------------------------------------------
// FunctionalityShared Transposed
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
const tMatrix<Tcolumns, Trows, TElement> FunctionalityShared<Trows, Tcolumns, TElement>::Transposed() const
{
  const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
  math::tMatrix<Tcolumns, Trows, TElement> result;
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      result[column][row] = (*that)[row][column];
    }
  }
  return result;
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
