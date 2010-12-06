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
/*!\file    tVector.h
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

#ifndef _rrlib_math_vector_tVector_h_
#define _rrlib_math_vector_tVector_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
namespace math
{
template <size_t, typename> class tVector;
}

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
template < size_t Tdimension, typename TElement, template <size_t, typename> class TData = vector::Cartesian >
class tVector : public TData<Tdimension, TElement>,
    public vector::FunctionalityShared<Tdimension, TElement, TData>,
    public vector::FunctionalitySpecialized<Tdimension, TElement, TData>,
    public vector::Conversions<Tdimension, TElement, TData>,
    public vector::Rotation<Tdimension, TElement, TData>,
#ifdef _MCA2_MATH_LEGACY_
    public vector::LegacyShared<Tdimension, TElement, TData>,
    public vector::LegacySpecialized<Tdimension, TElement, TData>,
#endif
    public vector::ConstantValuesShared<Tdimension, TElement, TData>,
    public vector::ConstantValuesSpecialized<Tdimension, TElement, TData>
{
  typedef vector::FunctionalityShared<Tdimension, TElement, TData> FunctionalityShared;
  typedef tVector<Tdimension, TElement, TData> tVectorType;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  inline tVector() {}
  inline tVector(const tVector &other) : FunctionalityShared(other) {}

  explicit inline tVector(const TElement data[Tdimension]) : FunctionalityShared(data) {}

  template <size_t Tother_dimension, typename TOtherElement>
  explicit inline tVector(const tVector<Tother_dimension, TOtherElement, TData> &other) : FunctionalityShared(other) {}

  template <typename ... TValues>
  inline tVector(TElement value, TValues... values)
  {
    FunctionalityShared::Set(value, values...);
  }

  inline tVector(const ::math::tVector<Tdimension, TElement> &other) : FunctionalityShared(*reinterpret_cast<const tVectorType *>(&other)) {}

  inline tVector &operator = (const tVector &other)
  {
    return reinterpret_cast<tVector &>(FunctionalityShared::operator=(other));
  }

  template <size_t Tother_dimension, typename TOtherElement>
  inline tVector &operator = (const tVector<Tother_dimension, TOtherElement, TData> &other)
  {
    return reinterpret_cast<tVector &>(FunctionalityShared::operator=(other));
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
