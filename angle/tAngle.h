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
/*!\file    rrlib/math/angle/tAngle.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-06
 *
 * \brief   A class that helps to avoid errors wrt different angle representations
 *
 * \b tAngle
 *
 * At least two different representations for angles (units) exist: Radian
 * which is used by typical trigonometric functions and the more intuitive
 * Degree used by humans.
 * This class takes care of converting from one representation into another
 * if needed.
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__angle__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tAngle.h" instead.
#endif

#ifndef __rrlib__math__angle__tAngle_h__
#define __rrlib__math__angle__tAngle_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <iostream>
#include <type_traits>
#include <stdexcept>
#include <cstring>

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_
#include "rrlib/serialization/serialization.h"
#include <sstream>
#endif

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/utilities.h"

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
class tAngleBase
{};

//! A class that helps to avoid errors wrt different angle representations
/*!
 * At least two different representations for angles (units) exist: Radian
 * which is used by typical trigonometric functions and the more intuitive
 * Degree used by humans.
 * This class takes care of converting from one representation into another
 * if needed.
 * It has an additional parameter that allows to configure the angle type
 * to automatically wrap its value to stay within on rotation, either
 * signed from -1/2 circle to 1/2 circle (exclusively), unsigned from 0 to
 * 1 full circle (exclusively) or without automatically wrapping its value.
 *
 * The latter allows e.g. counting rotations, accumulating angles or using
 * them in derived quantities like e.g. angular velocity.
 */
template <
typename TElement = double,
         typename TUnitPolicy = angle::Radian,
         typename TAutoWrapPolicy = angle::Signed
         >
class tAngle
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  /*! Default ctor of tAngle
   *
   * Initializes with zero
   */
  inline tAngle()
    : value(0)
  {}

  /*! Conversion from raw value
   *
   * \param value   The raw value used for initialization
   */
  inline tAngle(TElement value)
    : value(value)
  {
    TAutoWrapPolicy::Wrap(*this);
  }

  /*! Copy ctor of tAngle (same type)
   *
   * \param other   Another angle of same type
   */
  inline tAngle(const tAngle &other)
    : value(other.value)
  {
    TAutoWrapPolicy::Wrap(*this);
  }

  /*! Copy ctor of tAngle (with conversion from other specializations)
   *
   * \param other   Another angle with different specialization
   */
  template <typename TOtherElement, typename TOtherUnitPolicy, typename TOtherAutoWrapPolicy>
  inline tAngle(const tAngle<TOtherElement, TOtherUnitPolicy, TOtherAutoWrapPolicy> &other)
    : value(TUnitPolicy::ConvertFromUnit(static_cast<TElement>(other.Value()), TOtherUnitPolicy()))
  {
    TAutoWrapPolicy::Wrap(*this);
  }

  /*! Assignment operator (same type)
   *
   * \param other   Another angle of same type
   *
   * \return \a this angle
   */
  inline tAngle &operator = (const tAngle &other)
  {
    this->value = other.value;
    return *this;
  }

  /*! Assignment operator (with conversion from other specializations)
   *
   * \param other   Another angle with different specialization
   *
   * \return \a this angle
   */
  template <typename TOtherElement, typename TOtherUnitPolicy, typename TOtherAutoWrapPolicy>
  inline tAngle &operator = (const tAngle<TOtherElement, TOtherUnitPolicy, TOtherAutoWrapPolicy> &other)
  {
    *this = tAngle(other);
    return *this;
  }

  /*! Conversion operator for raw value
   *
   * To access the raw value via static_cast to every fundamental arithmetic type \a this->value can be converted to.
   *
   * \return The raw value
   */
  template <typename T, typename = typename std::enable_if<std::is_arithmetic<T>::value, int>::type>
  explicit inline operator T() const
  {
    return static_cast<T>(this->value);
  }

  /*! Access the raw value
   *
   * Read-only access to the raw value
   *
   * \return The raw value
   */
  inline TElement Value() const
  {
    return this->value;
  }

  /*! Addition assignment
   *
   * \param other   The angle that will be added to \a this angle
   *
   * \return \a this angle
   */
  inline tAngle &operator += (const tAngle &other)
  {
    this->value = this->value + other.value;
    TAutoWrapPolicy::Wrap(*this);
    return *this;
  }

  /*! Subtraction assignment
   *
   * \param other   The angle that will be subtracted from \a this angle
   *
   * \return \a this angle
   */
  inline tAngle &operator -= (const tAngle &other)
  {
    this->value = this->value - other.value;
    TAutoWrapPolicy::Wrap(*this);
    return *this;
  }

  /*! Multiplication assignment
   *
   * \param factor   The factor that will be multiplied with \this angle
   *
   * \return \a this angle
   */
  inline tAngle &operator *= (double factor)
  {
    this->value = this->value * factor;
    TAutoWrapPolicy::Wrap(*this);
    return *this;
  }

  /*! Division assignment
   *
   * \param divider   The divider that \a this angle will be divided by
   *
   * \return \a this angle
   */
  inline tAngle &operator /= (double divider)
  {
    this->value = this->value / divider;
    TAutoWrapPolicy::Wrap(*this);
    return *this;
  }

  /*! The sine of this angle
   *
   * Calculates the sine taking into account the current representation
   *
   * \return The sine of \a this angle
   */
  inline TElement Sine() const
  {
    return std::sin(tAngle<TElement, angle::Radian>(*this).Value());
  }

  /*! The cosine of this angle
   *
   * Calculates the cosine taking into account the current representation
   *
   * \return The cosine of \a this angle
   */
  inline TElement Cosine() const
  {
    return std::cos(tAngle<TElement, angle::Radian>(*this).Value());
  }

  /*! The tangent of this angle
   *
   * Calculates the tangent taking into account the current representation
   *
   * \return The tangent of \a this angle
   */
  inline TElement Tangent() const
  {
    return std::tan(tAngle<TElement, angle::Radian>(*this).Value());
  }

  /*! Optimized version to calculate sine and cosine at once
   *
   * Sine and cosine are often used together. Calling this method avoids an extra conversion.
   *
   * \param sine     The variable the sine value will be stored in
   * \param cosine   The variable the cosine value will be stored in
   */
  template <typename T>
  inline void SinCos(T &sine, T &cosine) const
  {
    TElement value = tAngle<TElement, angle::Radian>(*this).Value();
    sine = std::sin(value);
    cosine = std::cos(value);
  }

  /*! Wrap the internal value to the signed circle
   *
   * After calling this method, the internal value will be within
   * the half-open interval [-1/2 circle, 1/2 circle), representing the
   * same direction as before.
   */
  inline void WrapSigned()
  {
    double range = TUnitPolicy::FullRotation();

    double half_range = range / 2;
    this->value = Modulo(this->value + half_range, range) - half_range;
  }

  /*! Get the wrapped signed internal value without changing the object
   *
   * This method does not change \a this angle. It just returns the internal
   * value, wrapped to the half-open interval [-1/2 circle, 1/2 circle).
   *
   * \return The wrapped internal value
   */
  inline tAngle WrappedSigned() const
  {
    tAngle temp(*this);
    temp.WrapSigned();
    return temp;
  }

  /*! Wrap the internal value to the unsigned circle
   *
   * After calling this method, the internal value will be within
   * the half-open interval [0, 1 circle), representing the
   * same direction as before.
   */
  inline void WrapUnsigned()
  {
    double range = TUnitPolicy::FullRotation();
    this->value = Modulo<double>(this->value, range);
  }

  /*! Get the wrapped unsigned internal value without changing the object
   *
   * This method does not change \a this angle. It just returns the internal
   * value, wrapped to the half-open interval [0, 1 circle).
   *
   * \return The wrapped internal value
   */
  inline tAngle WrappedUnsigned() const
  {
    tAngle temp(*this);
    temp.WrapUnsigned();
    return temp;
  }

  /*! Get the number of full rotations represented by the internal value
   *
   * \note Will be limited to [-0.5, 0.5) after signed wrapping and [0, 1) after unsigned wrapping
   *
   * \return The (fractional) number of rotations
   */
  inline double Rotations() const
  {
    return this->value / TUnitPolicy::FullRotation();
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  TElement value;

};

//----------------------------------------------------------------------
// Arithmetic operators
//----------------------------------------------------------------------

/*! Unary minus
 *
 * \param angle   The angle
 *
 * \return -\a angle
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
inline tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> operator - (tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> angle)
{
  return tAngle<TElement, TUnitPolicy, TAutoWrapPolicy>(-static_cast<double>(angle));
}

/*! Addition
 *
 * \param left    The augend
 * \param right   The addend
 *
 * \return \a left + \a right
 */
template <typename TLeftElement, typename TRightElement, typename TLeftUnitPolicy, typename TRightUnitPolicy, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
inline tAngle < decltype(TLeftElement() + TRightElement()), TLeftUnitPolicy, TLeftAutoWrapPolicy > operator + (const tAngle<TLeftElement, TLeftUnitPolicy, TLeftAutoWrapPolicy> &left, const tAngle<TRightElement, TRightUnitPolicy, TRightAutoWrapPolicy> &right)
{
  tAngle < decltype(TLeftElement() + TRightElement()), TLeftUnitPolicy, TLeftAutoWrapPolicy > temp(left);
  temp += right;
  return temp;
}

/*! Subtraction
 *
 * \param left    The minuend
 * \param right   The subtrahend
 *
 * \return \a left - \a right
 */
template <typename TLeftElement, typename TRightElement, typename TLeftUnitPolicy, typename TRightUnitPolicy, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
inline tAngle < decltype(TLeftElement() - TRightElement()), TLeftUnitPolicy, TLeftAutoWrapPolicy > operator - (const tAngle<TLeftElement, TLeftUnitPolicy, TLeftAutoWrapPolicy> &left, const tAngle<TRightElement, TRightUnitPolicy, TRightAutoWrapPolicy> &right)
{
  tAngle < decltype(TLeftElement() - TRightElement()), TLeftUnitPolicy, TLeftAutoWrapPolicy > temp(left);
  temp -= right;
  return temp;
}

/*! Multiplication for scaling of angles
 *
 * \param angle    The angle
 * \param factor   The factor to multiply with
 *
 * \return The \a angle scaled by the \a factor
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy, typename TFactor>
inline tAngle<decltype(TElement() * TFactor()), TUnitPolicy, TAutoWrapPolicy> operator * (const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &angle, TFactor factor)
{
  return tAngle<decltype(TElement() * TFactor()), TUnitPolicy, TAutoWrapPolicy>(angle.Value() * factor);
}

/*! Multiplication for scaling of angles (for commutativity)
 *
 * \param factor   The factor to multiply with
 * \param angle    The angle
 *
 * \return The \a angle scaled by the \a factor
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy, typename TFactor>
inline tAngle<decltype(TElement() * TFactor()), TUnitPolicy, TAutoWrapPolicy> operator * (TFactor factor, const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &angle)
{
  return angle * factor;
}

/*! Division of angles
 *
 * \param left    The dividend
 * \param right   The divisor
 *
 * \return \a left / \a right
 */
template <typename TLeftElement, typename TRightElement, typename TLeftUnitPolicy, typename TRightUnitPolicy, typename TLeftAutoWrapPolicy, typename TRightAutoWrapPolicy>
inline decltype(TLeftElement() / TRightElement()) operator / (const tAngle<TLeftElement, TLeftUnitPolicy, TLeftAutoWrapPolicy> &left, const tAngle<TRightElement, TRightUnitPolicy, TRightAutoWrapPolicy> &right)
{
  return left.Value() / tAngle<TRightElement, TLeftUnitPolicy, TLeftAutoWrapPolicy>(right).Value();
}

/*! Division for scaling angles
 *
 * \param angle     The angle
 * \param divisor   The divisor
 *
 * \return The \a angle divided by the \a divisor
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy, typename TDivisor>
inline tAngle < decltype(TElement() / TDivisor()), TUnitPolicy, TAutoWrapPolicy > operator / (const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &angle, TDivisor divisor)
{
  return tAngle < decltype(TElement() / TDivisor()), TUnitPolicy, TAutoWrapPolicy > (angle.Value() / divisor);
}

//----------------------------------------------------------------------
// Comparison
//----------------------------------------------------------------------

/*! Compare two angles for equality taking into account numeric representations and errors
 *
 * \param left        The first angle in the comparison
 * \param right       The second angle in the comparison
 * \param max_error   The maximum difference to consider \a left and \a right equal (according to the used \a method)
 * \param method      The method to compare two float values
 *
 * \return Whether the two angles are the equal with respect to numeric representation and a \a max_error
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
bool IsEqual(const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &left, const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &right, float max_error = 1E-6, tFloatComparisonMethod method = eFCM_ABSOLUTE_ERROR)
{
  return IsEqual(left.Value(), right.Value(), max_error, method);
}

/*! Equality of two angles
 *
 * \param left    The first angle in the comparison
 * \param right   The second angle in the comparison
 *
 * \return Whether the two angles are equal
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
inline bool operator == (const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &left, const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &right)
{
  return left.Value() == right.Value();
}

/*! Inequality of two angles
 *
 * \param left    The first angle in the comparison
 * \param right   The second angle in the comparison
 *
 * \return Whether the two angles are unequal
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
bool operator != (const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &left, const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &right)
{
  return !(left == right);
}

/*! Smaller-than
 *
 * \param left    The first angle in the comparison
 * \param right   The second angle in the comparison
 *
 * \return Whether \a left is smaller than \a right
 */
template <typename TLeftElement, typename TRightElement, typename TLeftUnitPolicy, typename TRightUnitPolicy, typename TAutoWrapPolicy>
bool operator < (const tAngle<TLeftElement, TLeftUnitPolicy, TAutoWrapPolicy> &left, const tAngle<TRightElement, TRightUnitPolicy, TAutoWrapPolicy> &right)
{
  return left.Value() < tAngle<TLeftElement, TLeftUnitPolicy, TAutoWrapPolicy>(right).Value();
}

/*! Greater-than
 *
 * \param left    The first angle in the comparison
 * \param right   The second angle in the comparison
 *
 * \return Whether \a left is greater than \a right
 */
template <typename TLeftElement, typename TRightElement, typename TLeftUnitPolicy, typename TRightUnitPolicy, typename TAutoWrapPolicy>
bool operator > (const tAngle<TLeftElement, TLeftUnitPolicy, TAutoWrapPolicy> &left, const tAngle<TRightElement, TRightUnitPolicy, TAutoWrapPolicy> &right)
{
  return left.Value() > tAngle<TLeftElement, TLeftUnitPolicy, TAutoWrapPolicy>(right).Value();
}

/*! Not greater-than
 *
 * \param left    The first angle in the comparison
 * \param right   The second angle in the comparison
 *
 * \return Whether \a left is not greater than \a right
 */
template <typename TLeftElement, typename TRightElement, typename TLeftUnitPolicy, typename TRightUnitPolicy, typename TAutoWrapPolicy>
bool operator <= (const tAngle<TLeftElement, TLeftUnitPolicy, TAutoWrapPolicy> &left, const tAngle<TRightElement, TRightUnitPolicy, TAutoWrapPolicy> &right)
{
  return !(left > right);
}

/*! Not smaller-than
 *
 * \param left    The first angle in the comparison
 * \param right   The second angle in the comparison
 *
 * \return Whether \a left is not smaller than \a right
 */
template <typename TLeftElement, typename TRightElement, typename TLeftUnitPolicy, typename TRightUnitPolicy, typename TAutoWrapPolicy>
bool operator >= (const tAngle<TLeftElement, TLeftUnitPolicy, TAutoWrapPolicy> &left, const tAngle<TRightElement, TRightUnitPolicy, TAutoWrapPolicy> &right)
{
  return !(left < right);
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------

/*! Streaming into std::ostream
 *
 * Serializes the \angle as a string representation into the \a stream.
 * This representation includes all information to deserialize again from
 * the stream into an object of the same angle type.
 *
 * \param stream   The std::ostream to stream into
 * \param angle    The angle to stream
 *
 * \return The \a stream
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
std::ostream &operator << (std::ostream &stream, const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &angle)
{
  return stream << angle.Value() << (TUnitPolicy::UnitString() ? TUnitPolicy::UnitString() : "");
}

/*! Extraction from std::istream
 *
 * Deserializes the \angle from a string representation in the \a stream.
 * This will only work if the contained string representation is the same that
 * would be created by the serialization operator.
 *
 * \param stream   The std::ostream to stream into
 * \param angle    The angle to stream
 *
 * \return The \a stream
 *
 * \note Sets the failbit on deserialization error
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
std::istream &operator >> (std::istream &stream, tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &angle)
{
  std::istream::sentry stream_ok(stream, true);
  if (!stream_ok || stream.peek() == std::char_traits<TElement>::eof())
  {
    stream.setstate(std::ios_base::failbit);
    return stream;
  }

  TElement value;
  stream >> value;
  if (TUnitPolicy::UnitString())
  {
    size_t unit_length = std::strlen(TUnitPolicy::UnitString());
    char buffer[unit_length];
    std::memset(buffer, 0, sizeof(buffer));
    for (size_t i = 0; !stream.eof() && i < unit_length; ++i)
    {
      stream >> buffer[i];
    }
    buffer[unit_length] = 0;
    if (std::strncmp(buffer, TUnitPolicy::UnitString(), unit_length))
    {
      stream.setstate(std::ios_base::failbit);
      RRLIB_LOG_PRINT(ERROR, "Could not read expected unit string '", TUnitPolicy::UnitString(), "'! Read '", (char *)buffer, "'.");
      return stream;
    }
  }

  angle = tAngle<TElement, TUnitPolicy, TAutoWrapPolicy>(value);
  return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

/*! Serialization for serialization::tOutputStream
 *
 * \param stream   The output stream to serialize into
 * \param angle    The angle to be serialized
 *
 * \return The \a stream
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &angle)
{
  stream << angle.Value();
  return stream;
}

/*! Deserialization from serialization::tInputStream
 *
 * \param stream   The input stream to deserialize from
 * \param angle    The angle to deserialize into
 *
 * \return The \a stream
 *
 * \note Sets the failbit on deserialization error
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &angle)
{
  TElement value;
  stream >> value;
  angle = tAngle<TElement, TUnitPolicy, TAutoWrapPolicy>(value);
  return stream;
}

/*! String-serialization for rrlib::serialization::tOutputStream
 *
 * \param stream   The output stream to serialize into
 * \param angle    The angle to be serialized
 *
 * \return The \a stream
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &angle)
{
  std::stringstream string_stream;
  string_stream << angle;
  stream << string_stream.str();
  return stream;
}

/*! String-deserialization from serialization::tInputStream
 *
 * \param stream   The input stream to deserialize from
 * \param angle    The angle to deserialize into
 *
 * \return The \a stream
 *
 * \note Sets the failbit on deserialization error
 */
template <typename TElement, typename TUnitPolicy, typename TAutoWrapPolicy>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tAngle<TElement, TUnitPolicy, TAutoWrapPolicy> &angle)
{
  stream.GetWrappedStringStream() >> angle;
  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
