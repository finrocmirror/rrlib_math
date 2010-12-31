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
/*!\file    utilities.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-05
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_utilities_h_
#define _rrlib_math_utilities_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_float.hpp>
#include <boost/type_traits/is_integral.hpp>

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------
enum tFloatComparisonMethod
{
  eFCM_ABSOLUTE_ERROR,
  eFCM_RELATIVE_ERROR,
  eFCM_DISTANCE_IN_FLOAT_REPRESENTATION
};

//----------------------------------------------------------------------
// Function declaration
//----------------------------------------------------------------------

/*! Function to perform module operation with arbitrary integral numbers
 *
 * To perform a modulo operation with integral types usually operator %
 * is implemented. This function wraps that operator for unified modulo
 * operations of integral and float types. Additionally, the result is
 * wrapped to be positive which otherwise would be done by hand and is
 * often forgotten.
 *
 * \param a   The dividend of the operation
 * \param b   The divisor of the operation
 *
 * \return The remainder of the operation
 */
template <typename T>
inline const typename boost::enable_if<boost::is_integral<T>, T>::type Modulo(T a, T b)
{
  T v = a % b;
  return v < 0 ? v + b : v;
}

/*! Function to perform module operation with arbitrary float numbers
 *
 * To perform a modulo operation with integral types usually operator %
 * is implemented. This function wraps std::fmod for unified modulo
 * operations of integral and float types. Additionally, the result is
 * wrapped to be positive which otherwise would be done by hand and is
 * often forgotten.
 *
 * \param a   The dividend of the operation
 * \param b   The divisor of the operation
 *
 * \return The remainder of the operation
 */
template <typename T>
inline const typename boost::enable_if<boost::is_float<T>, T>::type Modulo(T a, T b)
{
  T v = std::fmod(a, b);
  return v < 0 ? v + b : v;
}

/*! Function to extract the absolute value of an arbitrary number
 *
 * Usually, extracting abolute values is done by calling std::abs on
 * a signed type. std::abs is not implemented for unsigned types for
 * obvious reasons. However, in terms of generic programming a template
 * function can be written, that takes arbitrary numeric types and has
 * to deal with absolute values at some point. Instead of specializing
 * the whole function for unsigned types the intuitive way would be to
 * use std::abs and let it handle its incoming data type.
 *
 * Instead of extending the std namespace by an implementation for
 * unsigned types getting absolute values is wrapped in a set of
 * functions which are implemented in terms of std::abs or to handle
 * unsigned types.
 *
 * This one is for unsigned int and just forwards its argument
 *
 * \param a   The unsigned int argument to extract an absolute value
 *
 * \return \a, as it is already positive
 */
inline const unsigned int AbsoluteValue(unsigned int value)
{
  return value;
}

/*! Function to extract the absolute value of an arbitrary number
 *
 * Usually, extracting abolute values is done by calling std::abs on
 * a signed type. std::abs is not implemented for unsigned types for
 * obvious reasons. However, in terms of generic programming a template
 * function can be written, that takes arbitrary numeric types and has
 * to deal with absolute values at some point. Instead of specializing
 * the whole function for unsigned types the intuitive way would be to
 * use std::abs and let it handle its incoming data type.
 *
 * Instead of extending the std namespace by an implementation for
 * unsigned types getting absolute values is wrapped in a set of
 * functions which are implemented in terms of std::abs or to handle
 * unsigned types.
 *
 * This one is for every type that was not specially dealt with and can
 * be handled by std::abs.
 *
 * \param a   The argument to extract an absolute value using std::abs
 *
 * \return std::abs(a)
 */
template <typename T>
inline const T AbsoluteValue(T value)
{
  return std::abs(value);
}

/*! Function to compare two float numbers considering numerical errors that come with machine numbers
 *
 * Inherent to the representation of floating point numbers it is very
 * unlikely to compare two variables that should contain the same number
 * coming from different operations and determine their equality using
 * operator ==.
 *
 * Thus, the typical way of comparing float numbers is to compare their
 * difference to a maximum acceptable error (often called epsilon).
 * That maximum acceptable error can be defined as an absolute value,
 * meaning that two numbers a and b are considered equal if |a-b| is
 * below that error:
 * \c eFCM_ABSOLUTE_ERROR
 *
 * Considering the magnitude of the two numbers one might find that an
 * absolute difference of 1 might be very accurate for numbers in the
 * range of millions, but unacceptable in the range of [0..1]. If the
 * expected range is unknown it might be better to define the maximum
 * error as a fraction of the bigger of the involved numbers:
 * \c eFCM_RELATIVE_ERROR
 *
 * There is even a third possibility that uses the internal appearance
 * of floating point numbers and calculates the number of values that
 * are representable btween two given values. This is a more accurate
 * technical way of an relative error limit:
 * \c eFCM_DISTANCE_IN_FLOAT_REPRESENTATION
 *
 * \param a           The first participant in the comparison
 * \param b           The second participant in the comparison
 * \param max_error   The maximum distance in terms of the used method that is allowed to assume \a a == \b b
 * \param method      The method to be used to calculate the distance between \a a and \a b
 *
 * \return If the difference between \a a and \a b is within the given limit using the given method
 */
const bool IsEqual(float a, float b, float max_error = 1.0E-6, tFloatComparisonMethod method = eFCM_ABSOLUTE_ERROR);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
