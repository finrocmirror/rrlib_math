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
/*!\file    FunctionalityShared.h
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

#ifndef _rrlib_math_matrix_functionality_FunctionalityShared_h_
#define _rrlib_math_matrix_functionality_FunctionalityShared_h_

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

  FunctionalityShared(const FunctionalityShared &);

  template <size_t number_of_given_values>
  inline void SetValues(TElement buffer[Trows * Tcolumns])
  {
    static_assert(number_of_given_values == Trows * Tcolumns, "Wrong number of values given to store in matrix");
    reinterpret_cast<tMatrix *>(this)->SetFromArray(buffer);
  }
  template <size_t number_of_given_values, typename ... TValues>
  inline void SetValues(TElement buffer[Trows * Tcolumns], TElement value, TValues... values)
  {
    buffer[number_of_given_values] = value;
    this->SetValues < number_of_given_values + 1 > (buffer, values...);
  }

protected:

  inline FunctionalityShared()
  {
    std::memset(this, 0, sizeof(tMatrix));
  }
  inline FunctionalityShared(const tMatrix &other)
  {
    std::memcpy(this, &other, sizeof(tMatrix));
  }

  explicit inline FunctionalityShared(const TElement data[Trows * Tcolumns])
  {
    reinterpret_cast<tMatrix *>(this)->SetFromArray(data);
  }

  template <typename TOtherElement>
  explicit inline FunctionalityShared(const math::tMatrix<Trows, Tcolumns, TOtherElement, TData> &other)
  {
    std::memset(this, 0, sizeof(tMatrix));
    for (size_t i = 0; i < sizeof(tMatrix) / sizeof(TElement); ++i)
    {
      reinterpret_cast<TElement *>(this)[i] = reinterpret_cast<const TOtherElement *>(&other)[i];
    }
  }

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  explicit inline FunctionalityShared(const math::tMatrix<Trows, Tcolumns, TOtherElement, TOtherData> &other)
  {
    this->SetFromMatrix(other);
  }

  template <typename TLeftElement, typename TRightElement>
  inline FunctionalityShared(const tVector<Trows, TLeftElement, vector::Cartesian> &left, const tVector<Tcolumns, TRightElement, vector::Cartesian> &right)
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

public:

  inline const typename tData::Accessor operator [](size_t row) const
  {
    return const_cast<FunctionalityShared &>(*this)[row];
  }
  inline typename tData::Accessor operator [](size_t row)
  {
    return typename tData::Accessor(reinterpret_cast<TElement *>(this), row);
  }

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
      for (size_t i = 0; i < Trows * Tcolumns; ++i)
      {
        reinterpret_cast<TElement *>(this)[i] = reinterpret_cast<const TOtherElement *>(&other)[i];
      }
    }
    return *this;
  }

  template <typename ... TValues>
  inline void Set(TValues... values)
  {
    TElement buffer[Trows * Tcolumns];
    this->SetValues<0>(buffer, values...);
  }

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  inline void SetFromMatrix(const math::tMatrix<Trows, Tcolumns, TOtherElement, TOtherData> &source)
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

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  inline const tMatrix &operator += (const math::tMatrix<Trows, Tcolumns, TOtherElement, TOtherData> &other)
  {
    tMatrix *that = reinterpret_cast<tMatrix *>(this);
    *that = *that + other;
    return *that;
  }

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  inline const tMatrix &operator -= (const math::tMatrix<Trows, Tcolumns, TOtherElement, TOtherData> &other)
  {
    tMatrix *that = reinterpret_cast<tMatrix *>(this);
    *that = *that - other;
    return *that;
  }

  template <typename TOtherElement, template <size_t, size_t, typename> class TOtherData>
  inline const tMatrix &operator *= (const math::tMatrix<Tcolumns, Tcolumns, TOtherElement, TOtherData> &other)
  {
    tMatrix *that = reinterpret_cast<tMatrix *>(this);
    *that = *that * other;
    return *that;
  }

  template <typename TScalar>
  inline const typename boost::enable_if<boost::is_scalar<TScalar>, tMatrix>::type &operator *= (const TScalar &scalar)
  {
    tMatrix *that = reinterpret_cast<tMatrix *>(this);
    *that = *that * scalar;
    return *that;
  }

  template <typename TScalar>
  inline const typename boost::enable_if<boost::is_scalar<TScalar>, tMatrix>::type &operator /= (const TScalar &scalar)
  {
    tMatrix *that = reinterpret_cast<tMatrix *>(this);
    *that *= 1.0 / scalar;
    return *that;
  }

  inline const bool IsZero(double epsilon = 0) const
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

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
