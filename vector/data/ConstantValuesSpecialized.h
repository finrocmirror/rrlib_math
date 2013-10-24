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
/*!\file    rrlib/math/vector/data/ConstantValuesSpecialized.h
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

#ifndef __rrlib__math__vector__data__ConstantValuesSpecialized_h__
#define __rrlib__math__vector__data__ConstantValuesSpecialized_h__

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
class ConstantValuesSpecialized
{

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline ConstantValuesSpecialized()
  {};

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  ConstantValuesSpecialized(const ConstantValuesSpecialized &other);
  ConstantValuesSpecialized &operator = (const ConstantValuesSpecialized &);

};

/*!
 *
 */
template <typename TElement>
class ConstantValuesSpecialized<2, TElement, Cartesian>
{
  typedef math::tVector<2, TElement, Cartesian> tVector;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static const tVector &XDirection() __attribute__((always_inline, flatten));

  static const tVector &YDirection() __attribute__((always_inline, flatten));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline ConstantValuesSpecialized()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  ConstantValuesSpecialized(const ConstantValuesSpecialized &other);
  ConstantValuesSpecialized &operator = (const ConstantValuesSpecialized &);

};

/*!
 *
 */
template <typename TElement>
class ConstantValuesSpecialized<3, TElement, Cartesian>
{
  typedef math::tVector<3, TElement, Cartesian> tVector;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static inline const tVector &XDirection() __attribute__((always_inline, flatten));

  static inline const tVector &YDirection() __attribute__((always_inline, flatten));

  static inline const tVector &ZDirection() __attribute__((always_inline, flatten));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline ConstantValuesSpecialized()
  {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  ConstantValuesSpecialized(const ConstantValuesSpecialized &other);
  ConstantValuesSpecialized &operator = (const ConstantValuesSpecialized &);

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#include "rrlib/math/vector/data/ConstantValuesSpecialized.hpp"

#endif
