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
/*!\file    rrlib/math/matrix/functionality/FunctionalityShared.h
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

#ifndef __rrlib__math__matrix__functionality__FunctionalityShared_h__
#define __rrlib__math__matrix__functionality__FunctionalityShared_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstring>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_scalar.hpp>

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
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TData>
class FunctionalityShared
{
  typedef math::tMatrix<Trows, Tcolumns, TElement, TData> tMatrix;
  typedef TData<Trows, Tcolumns, TElement> tData;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline const typename tData::Accessor operator [](size_t row) const __attribute__((always_inline));

  inline typename tData::Accessor operator [](size_t row) __attribute__((always_inline));

  inline FunctionalityShared &operator = (const FunctionalityShared &other)
  {
    const uint8_t *this_addr = reinterpret_cast<uint8_t *>(this);
    const uint8_t *other_addr = reinterpret_cast<const uint8_t *>(&other);
    if (this_addr != other_addr)
    {
      if (std::abs(this_addr - other_addr) < sizeof(tMatrix))
      {
        std::stringstream stream;
        stream << "Overlapping memory areas in rrlib::math::tMatrix::operator = (this = " << this << ", other = " << &other << ")!";
        throw std::logic_error(stream.str());
      }
      std::memcpy(this, &other, sizeof(tMatrix));
    }
    return *this;
  }

  template <typename TOtherElement>
  inline FunctionalityShared &operator = (const math::tMatrix<Trows, Tcolumns, TOtherElement, TData> &other)
  {
    const uint8_t *this_addr = reinterpret_cast<uint8_t *>(this);
    const uint8_t *other_addr = reinterpret_cast<const uint8_t *>(&other);

    if (this_addr != other_addr)
    {
      const size_t safety_area = this_addr < other_addr ? sizeof(tMatrix) : sizeof(math::tMatrix<Trows, Tcolumns, TOtherElement, TData>);
      if (static_cast<size_t>(std::abs(this_addr - other_addr)) < safety_area)
      {
        std::stringstream stream;
        stream << "Overlapping memory areas in rrlib::math::tMatrix::operator = (this = " << this << ", other = " << &other << ")!";
        throw std::logic_error(stream.str());
      }
      std::memset(this, 0, sizeof(tMatrix));
      for (size_t i = 0; i < sizeof(tMatrix) / sizeof(TElement); ++i)
      {
        reinterpret_cast<TElement *>(this)[i] = reinterpret_cast<const TOtherElement *>(&other)[i];
      }
    }
    return *this;
  }

  template <typename ... TValues>
  inline void Set(TValues... values) __attribute__((always_inline));

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  inline void SetFromMatrix(const math::tMatrix<Trows, Tcolumns, TOtherElement, TOtherData> &source) __attribute__((always_inline));

  inline void SetIdentity() __attribute__((always_inline));

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  inline const tMatrix &operator += (const math::tMatrix<Trows, Tcolumns, TOtherElement, TOtherData> &other) __attribute__((always_inline));

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  inline const tMatrix &operator -= (const math::tMatrix<Trows, Tcolumns, TOtherElement, TOtherData> &other) __attribute__((always_inline));

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  inline const tMatrix &operator *= (const math::tMatrix<Tcolumns, Tcolumns, TOtherElement, TOtherData> &other) __attribute__((always_inline));

  template <typename TScalar>
  inline const typename boost::enable_if<boost::is_scalar<TScalar>, tMatrix>::type &operator *= (const TScalar &scalar) __attribute__((always_inline));

  template <typename TScalar>
  inline const typename boost::enable_if<boost::is_scalar<TScalar>, tMatrix>::type &operator /= (const TScalar &scalar) __attribute__((always_inline));

  inline const bool IsZero(double epsilon = 0) const __attribute__((always_inline));

  inline tVector<Tcolumns, TElement, vector::Cartesian> GetRow(size_t row) const;

  inline tVector<Trows, TElement, vector::Cartesian> GetColumn(size_t column) const;

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline FunctionalityShared() __attribute__((always_inline));

  inline FunctionalityShared(const tMatrix &other) __attribute__((always_inline));

  explicit inline FunctionalityShared(const TElement data[Trows * Tcolumns]) __attribute__((always_inline));

  template <typename TOtherElement>
  explicit inline FunctionalityShared(const math::tMatrix<Trows, Tcolumns, TOtherElement, TData> &other) __attribute__((always_inline));

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  explicit inline FunctionalityShared(const math::tMatrix<Trows, Tcolumns, TOtherElement, TOtherData> &other) __attribute__((always_inline));

  template <typename TLeftElement, typename TRightElement>
  inline FunctionalityShared(const tVector<Trows, TLeftElement, vector::Cartesian> &left, const tVector<Tcolumns, TRightElement, vector::Cartesian> &right) __attribute__((always_inline));

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  FunctionalityShared(const FunctionalityShared &);

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "rrlib/math/matrix/functionality/FunctionalityShared.hpp"

#endif
