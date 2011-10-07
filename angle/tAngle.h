//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    tAngle.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-06
 *
 * \brief
 *
 * \b
 *
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
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_same.hpp>
#include <stdexcept>
#include <sstream>

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_
#include "rrlib/serialization/tStringInputStream.h"
#include "rrlib/serialization/tStringOutputStream.h"
#include <sstream>
#endif

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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
template <
typename TElement = double,
typename TUnitPolicy = angle::Radian,
typename TSignPolicy = angle::Signed
>
class tAngle
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline tAngle()
      : value(0)
  {}

  inline tAngle(TElement value)
      : value(TSignPolicy::FitIntoRange(value, TUnitPolicy::RangeLimit()))
  {}

  inline tAngle(const tAngle &other)
      : value(other.value)
  {}

  template <typename TOtherElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
  inline tAngle(const tAngle<TOtherElement, TOtherUnitPolicy, TOtherSignPolicy> &other)
      : value(TSignPolicy::FitIntoRange(TUnitPolicy::ConvertFromUnit(static_cast<TOtherElement>(other), TOtherUnitPolicy()), TUnitPolicy::RangeLimit()))
  {}

  inline tAngle &operator = (const tAngle &other)
  {
    this->value = other.value;
    return *this;
  }

  template <typename TOtherElement, typename TOtherUnitPolicy, typename TOtherSignPolicy>
  inline tAngle &operator = (const tAngle<TOtherElement, TOtherUnitPolicy, TOtherSignPolicy> &other)
  {
    this->value = TSignPolicy::FitIntoRange(TUnitPolicy::ConvertFromUnit(static_cast<TOtherElement>(other), TOtherUnitPolicy()), TUnitPolicy::RangeLimit());
    return *this;
  }

  /*explicit*/ inline operator TElement() const // FIXME
  {
    return this->value;
  }

  inline tAngle &operator += (const tAngle &other)
  {
    this->value = TSignPolicy::FitIntoRange(this->value + static_cast<TElement>(other), TUnitPolicy::RangeLimit());
    return *this;
  }

  template <typename TOtherUnitPolicy, typename TOtherSignPolicy>
  inline typename boost::disable_if<boost::is_same<tAngle, tAngle<TElement, TOtherUnitPolicy, TOtherSignPolicy> >, tAngle>::type &operator += (const tAngle<TElement, TOtherUnitPolicy, TOtherSignPolicy> &other)
  {
    this->value = TSignPolicy::FitIntoRange(this->value + TUnitPolicy::ConvertFromUnit(static_cast<TElement>(other), TOtherUnitPolicy()), TUnitPolicy::RangeLimit());
    return *this;
  }

  inline tAngle &operator -= (const tAngle &other)
  {
    this->value = TSignPolicy::FitIntoRange(this->value - static_cast<TElement>(other), TUnitPolicy::RangeLimit());
    return *this;
  }

  template <typename TOtherUnitPolicy, typename TOtherSignPolicy>
  inline typename boost::disable_if<boost::is_same<tAngle, tAngle<TElement, TOtherUnitPolicy, TOtherSignPolicy> >, tAngle>::type &operator -= (const tAngle<TElement, TOtherUnitPolicy, TOtherSignPolicy> &other)
  {
    this->value = TSignPolicy::FitIntoRange(this->value - TUnitPolicy::ConvertFromUnit(static_cast<TElement>(other), TOtherUnitPolicy()), TUnitPolicy::RangeLimit());
    return *this;
  }

  inline tAngle &operator *= (double factor)
  {
    this->value = TSignPolicy::FitIntoRange(this->value * factor, TUnitPolicy::RangeLimit());
    return *this;
  }

  inline tAngle &operator /= (double divider)
  {
    this->value = TSignPolicy::FitIntoRange(this->value / divider, TUnitPolicy::RangeLimit());
    return *this;
  }

  inline const double Sine() const
  {
    return std::sin(angle::Radian::ConvertFromUnit(this->value, TUnitPolicy()));
  }

  inline const double Cosine() const
  {
    return std::cos(angle::Radian::ConvertFromUnit(this->value, TUnitPolicy()));
  }

  inline const double Tangent() const
  {
    return std::tan(angle::Radian::ConvertFromUnit(this->value, TUnitPolicy()));
  }

  inline void SinCos(double &sine, double &cosine) const
  {
    sincos(angle::Radian::ConvertFromUnit(this->value, TUnitPolicy()), &sine, &cosine);
  }

  inline void SinCos(float &sine, float &cosine) const
  {
    sincosf(angle::Radian::ConvertFromUnit(this->value, TUnitPolicy()), &sine, &cosine);
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  TElement value;

};

template <typename TElement, typename TUnitPolicy, typename TSignPolicy>
inline const tAngle<TElement, TUnitPolicy, TSignPolicy> operator - (const tAngle<TElement, TUnitPolicy, TSignPolicy> &angle)
{
  return tAngle<TElement, TUnitPolicy, TSignPolicy>(-static_cast<double>(angle));
}

template <typename TElement, typename TUnitPolicy, typename TSignPolicy>
inline const tAngle<TElement, TUnitPolicy, TSignPolicy> operator + (const tAngle<TElement, TUnitPolicy, TSignPolicy> &left, const tAngle<TElement, TUnitPolicy, TSignPolicy> &right)
{
  tAngle<TElement, TUnitPolicy, TSignPolicy> temp(left);
  temp += right;
  return temp;
}

template <typename TElement, typename TUnitPolicy, typename TSignPolicy>
inline const tAngle<TElement, TUnitPolicy, TSignPolicy> operator - (const tAngle<TElement, TUnitPolicy, TSignPolicy> &left, const tAngle<TElement, TUnitPolicy, TSignPolicy> &right)
{
  tAngle<TElement, TUnitPolicy, TSignPolicy> temp(left);
  temp -= right;
  return temp;
}

//template <typename TUnitPolicy, typename TSignPolicy>
//inline const tAngle<TUnitPolicy, TSignPolicy> operator *(const tAngle<TUnitPolicy, TSignPolicy> &angle, double factor)
//{
//  tAngle<TUnitPolicy, TSignPolicy> temp(angle);
//  temp *= factor;
//  return temp;
//}
//template <typename TUnitPolicy, typename TSignPolicy>
//inline const tAngle<TUnitPolicy, TSignPolicy> operator *(double factor, const tAngle<TUnitPolicy, TSignPolicy> &angle)
//{
//  return angle * factor;
//}

//template <typename TUnitPolicy, typename TSignPolicy>
//inline const tAngle<TUnitPolicy, TSignPolicy> operator / (const tAngle<TUnitPolicy, TSignPolicy> &angle, double divider)
//{
//  tAngle<TUnitPolicy, TSignPolicy> temp(angle);
//  temp /= divider;
//  return temp;
//}

template <typename TElement, typename TUnitPolicy, typename TSignPolicy>
std::ostream &operator << (std::ostream &stream, const tAngle<TElement, TUnitPolicy, TSignPolicy> &angle)
{
  return stream << (static_cast<TElement>(angle) / TUnitPolicy::UnitDivider()) << (TUnitPolicy::PadUnitString() ? " " : "") << TUnitPolicy::UnitString();
}

template <typename TElement, typename TUnitPolicy, typename TSignPolicy>
std::istream &operator >> (std::istream &stream, tAngle<TElement, TUnitPolicy, TSignPolicy> &angle)
{
  double value;
  stream >> value;
  angle = value * TUnitPolicy::UnitDivider();
  size_t unit_string_length = strlen(TUnitPolicy::UnitString());
  if (unit_string_length > 0)
  {
    char temp[unit_string_length + 1];
    for (std::string::size_type i = 0; i < unit_string_length; ++i)
    {
      stream >> temp[i];
    }
    temp[unit_string_length] = 0;
    if (strcmp(TUnitPolicy::UnitString(), temp) != 0)
    {
      std::stringstream error_message;
      error_message << "Could not read expected unit string '" << TUnitPolicy::UnitString() << "'! Read '" << temp << "'.";
      throw std::runtime_error(error_message.str());
    }
  }
  return stream;
}

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

template <typename TElement, typename TUnitPolicy, typename TSignPolicy>
serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tAngle<TElement, TUnitPolicy, TSignPolicy> &angle)
{
  double unsigned_radian_value = tAngle<double, angle::Radian, angle::Unsigned>(angle);
  stream << unsigned_radian_value;
  return stream;
}

template <typename TElement, typename TUnitPolicy, typename TSignPolicy>
serialization::tInputStream &operator >> (serialization::tInputStream &stream, tAngle<TElement, TUnitPolicy, TSignPolicy> &angle)
{
  double unsigned_radian_value;
  stream >> unsigned_radian_value;
  angle = tAngle<double, angle::Radian, angle::Unsigned>(unsigned_radian_value);
  return stream;
}

template <typename TElement, typename TUnitPolicy, typename TSignPolicy>
serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tAngle<TElement, TUnitPolicy, TSignPolicy> &angle)
{
  std::stringstream string_stream;
  string_stream << angle;
  stream << string_stream.str();
  return stream;
}

template <typename TElement, typename TUnitPolicy, typename TSignPolicy>
serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tAngle<TElement, TUnitPolicy, TSignPolicy> &angle)
{
  std::istringstream string_stream(stream.ReadLine());
  string_stream >> angle;
  return stream;
}

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
