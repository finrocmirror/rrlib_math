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
/*!\file    OperatorsPolar.h
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

#ifndef __rrlib__math__vector__data__OperatorsPolar_h__
#define __rrlib__math__vector__data__OperatorsPolar_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>

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
// Function declarations
//----------------------------------------------------------------------

template <size_t Tdimension, typename TElement>
std::ostream &operator << (std::ostream &stream, const tVector<Tdimension, TElement, Polar> &vector)
{
  stream << "(";
  for (size_t i = 0; i < Tdimension - 1; ++i)
  {
    stream << vector[i] * 180.0 / M_PI << "°, ";
  }
  stream << vector[Tdimension - 1] << ")";
  return stream;
}

template <size_t Tdimension, typename TElement>
const tVector<Tdimension, TElement, Polar> operator - (const tVector<Tdimension, TElement, Polar> &vector)
{
  return (-vector.GetCartesianVector()).GetPolarVector();
}

template <size_t Tdimension, typename TLeftElement, typename TRightElement>
const tVector<Tdimension, typename until_0x::Auto<TLeftElement, TRightElement>::type, Polar> operator + (const tVector<Tdimension, TLeftElement, Polar> &left, const tVector<Tdimension, TRightElement, Polar> &right)
{
  return (left.GetCartesianVector() + right.GetCartesianVector()).GetPolarVector();
}

template <size_t Tdimension, typename TLeftElement, typename TRightElement>
const tVector<Tdimension, typename until_0x::Auto<TLeftElement, TRightElement>::type, Polar> operator - (const tVector<Tdimension, TLeftElement, Polar> &left, const tVector<Tdimension, TRightElement, Polar> &right)
{
  return (left.GetCartesianVector() - right.GetCartesianVector()).GetPolarVector();
}

template <size_t Tdimension, typename TElement, typename TScalar>
const typename boost::enable_if<boost::is_scalar<TScalar>, tVector<Tdimension, typename until_0x::Auto<TElement, TScalar>::type, Polar> >::type operator *(const tVector<Tdimension, TElement, Polar> &vector, const TScalar scalar)
{
  typedef math::tVector<Tdimension, typename until_0x::Auto<TElement, TScalar>::type, Polar> tResult;
  typename tResult::tElement data[Tdimension];
  for (size_t i = 0; i < Tdimension - 1; ++i)
  {
    data[i] = reinterpret_cast<const TElement *>(&vector)[i];
  }
  data[Tdimension - 1] = vector.Length() * scalar;
  return tResult(data);
}
template <size_t Tdimension, typename TElement, typename TScalar>
const typename boost::enable_if<boost::is_scalar<TScalar>, tVector<Tdimension, typename until_0x::Auto<TElement, TScalar>::type, Polar> >::type operator *(const TScalar scalar, const tVector<Tdimension, TElement, Polar> &vector)
{
  return vector * scalar;
}

template <size_t Tdimension, typename TLeftElement, typename TRightElement>
const typename until_0x::Auto<TLeftElement, TRightElement>::type operator *(const tVector<Tdimension, TLeftElement, Polar> &left, const tVector<Tdimension, TRightElement, Polar> &right)
{
  return left.GetCartesianVector() * right.GetCartesianVector();
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
