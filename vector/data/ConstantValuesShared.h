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
/*!\file    ConstantValuesShared.h
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

#ifndef _rrlib_math_vector_data_ConstantValuesShared_h_
#define _rrlib_math_vector_data_ConstantValuesShared_h_

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
class ConstantValuesShared
{
  typedef tVector<Tdimension, TElement, TData> tVectorType;

  ConstantValuesShared(const ConstantValuesShared &other);
  ConstantValuesShared &operator = (const ConstantValuesShared &);

protected:

  inline ConstantValuesShared() {};

public:

  static const size_t cDIMENSION = Tdimension;

  typedef TElement tElementType;

  static inline const tVectorType &Zero()
  {
    static tVectorType vector;
    return vector;
  }

};

/*!
 *
 */
template <size_t Tdimension, typename TElement>
class ConstantValuesShared<Tdimension, TElement, Cartesian>
{
  typedef tVector<Tdimension, TElement, Cartesian> tVectorType;

  ConstantValuesShared(const ConstantValuesShared &other);
  ConstantValuesShared &operator = (const ConstantValuesShared &);

  static const tVectorType &InitializeIdentity()
  {
    static tVectorType vector;
    for (size_t i = 0; i < Tdimension; ++i)
    {
      vector[i] = 1;
    }
    return vector;
  }

protected:

  inline ConstantValuesShared() {};

public:

  static const size_t cDIMENSION = Tdimension;

  typedef TElement tElementType;

  static inline const tVectorType &Zero()
  {
    static tVectorType vector;
    return vector;
  }

  static inline const tVectorType &Identity()
  {
    static tVectorType identity(InitializeIdentity());
    return identity;
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
