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
#ifndef __rrlib__math__vector__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef __rrlib__math__vector__tVector_h__
#define __rrlib__math__vector__tVector_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

#ifdef _LIB_OIV_PRESENT_
#include <Inventor/SbVec2f.h>
#include <Inventor/SbVec3f.h>
#include <boost/utility/enable_if.hpp>
#endif
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
template < size_t Tdimension, typename TElement = double, template <size_t, typename> class TData = vector::Cartesian >
class tVector : public TData<Tdimension, TElement>,
    public vector::FunctionalityShared<Tdimension, TElement, TData>,
    public vector::FunctionalitySpecialized<Tdimension, TElement, TData>,
    public vector::Conversions<Tdimension, TElement, TData>,
    public vector::Rotation<Tdimension, TElement, TData>,
    public vector::LegacyShared<Tdimension, TElement, TData>,
    public vector::LegacySpecialized<Tdimension, TElement, TData>,
    public vector::ConstantValuesShared<Tdimension, TElement, TData>,
    public vector::ConstantValuesSpecialized<Tdimension, TElement, TData>
{
  typedef vector::FunctionalityShared<Tdimension, TElement, TData> FunctionalityShared;
  typedef vector::FunctionalitySpecialized<Tdimension, TElement, TData> FunctionalitySpecialized;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  inline tVector() {}
  inline tVector(const tVector &other) : FunctionalityShared(other) {}

  template <size_t Tother_dimension, typename TOtherElement>
  inline tVector(const tVector<Tother_dimension, TOtherElement> &other) : FunctionalitySpecialized(other) {}

  template <typename ... TValues>
  explicit inline tVector(TValues... values) : FunctionalitySpecialized(values...) {}

#ifdef _LIB_OIV_PRESENT_

  template < typename T = int >
  explicit inline tVector(const SbVec2f &v, typename boost::enable_if_c < (Tdimension == 2), T >::type = 0)
  {
    FunctionalitySpecialized::Set(v[0], v[1]);
  }

  template < typename T = int >
  explicit inline tVector(const SbVec3f &v, typename boost::enable_if_c < (Tdimension == 3), T >::type = 0)
  {
    FunctionalitySpecialized::Set(v[0], v[1], v[2]);
  }

#endif

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
