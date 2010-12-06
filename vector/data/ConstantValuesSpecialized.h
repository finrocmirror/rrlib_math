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
/*!\file    ConstantValuesSpecialized.h
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

#ifndef _rrlib_math_vector_data_ConstantValuesSpecialized_h_
#define _rrlib_math_vector_data_ConstantValuesSpecialized_h_

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
  ConstantValuesSpecialized(const ConstantValuesSpecialized &other);
  ConstantValuesSpecialized &operator = (const ConstantValuesSpecialized &);

protected:

  inline ConstantValuesSpecialized() {};

};

/*!
 *
 */
template <typename TElement>
class ConstantValuesSpecialized<2, TElement, Cartesian>
{
  typedef tVector<2, TElement, Cartesian> tVectorType;

  ConstantValuesSpecialized(const ConstantValuesSpecialized &other);
  ConstantValuesSpecialized &operator = (const ConstantValuesSpecialized &);

protected:

  inline ConstantValuesSpecialized() {}

public:

  static const tVectorType &XDirection()
  {
    static tVectorType vector(1, 0);
    return vector;
  }

  static const tVectorType &YDirection()
  {
    static tVectorType vector(0, 1);
    return vector;
  }

};

/*!
 *
 */
template <typename TElement>
class ConstantValuesSpecialized<3, TElement, Cartesian>
{
  typedef tVector<3, TElement, Cartesian> tVectorType;

  ConstantValuesSpecialized(const ConstantValuesSpecialized &other);
  ConstantValuesSpecialized &operator = (const ConstantValuesSpecialized &);

protected:

  inline ConstantValuesSpecialized() {}

public:

  static inline const tVectorType &XDirection()
  {
    static tVectorType vector(1, 0, 0);
    return vector;
  }

  static inline const tVectorType &YDirection()
  {
    static tVectorType vector(0, 1, 0);
    return vector;
  }

  static inline const tVectorType &ZDirection()
  {
    static tVectorType vector(0, 0, 1);
    return vector;
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
