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
#ifndef _rrlib_math_angle_include_guard_
#error Invalid include directive. Try #include "rrlib/math/tAngle.h" instead.
#endif

#ifndef _rrlib_math_angle_tAngle_h_
#define _rrlib_math_angle_tAngle_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <ostream>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_same.hpp>

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
typename TUnitPolicy = angle::Radian,
typename TSignPolicy = angle::Signed
>
class tAngle
{
  double value;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  inline tAngle()
      : value(0)
  {}

  inline tAngle(double value)
      : value(TSignPolicy::FitIntoRange(value, TUnitPolicy::RangeLimit()))
  {}

  inline tAngle(const tAngle &other)
      : value(other.value)
  {}

  template <typename TOtherUnitPolicy, typename TOtherSignPolicy>
  inline tAngle(const tAngle<TOtherUnitPolicy, TOtherSignPolicy> &other)
      : value(TSignPolicy::FitIntoRange(TUnitPolicy::ConvertFromUnit(static_cast<double>(other), TOtherUnitPolicy()), TUnitPolicy::RangeLimit()))
  {}

  inline tAngle &operator = (const tAngle &other)
  {
    this->value = other.value;
    return *this;
  }

  template <typename TOtherUnitPolicy, typename TOtherSignPolicy>
  inline tAngle &operator = (const tAngle<TOtherUnitPolicy, TOtherSignPolicy> &other)
  {
    this->value = TSignPolicy::FitIntoRange(TUnitPolicy::ConvertFromUnit(static_cast<double>(other), TOtherUnitPolicy()), TUnitPolicy::RangeLimit());
    return *this;
  }

  inline operator double() const
  {
    return this->value;
  }

  inline tAngle &operator += (const tAngle &other)
  {
    this->value = TSignPolicy::FitIntoRange(this->value + static_cast<double>(other), TUnitPolicy::RangeLimit());
    return *this;
  }

  template <typename TOtherUnitPolicy, typename TOtherSignPolicy>
  inline typename boost::disable_if<boost::is_same<tAngle, tAngle<TOtherUnitPolicy, TOtherSignPolicy> >, tAngle>::type &operator += (const tAngle<TOtherUnitPolicy, TOtherSignPolicy> &other)
  {
    this->value = TSignPolicy::FitIntoRange(this->value + TUnitPolicy::ConvertFromUnit(static_cast<double>(other), TOtherUnitPolicy()), TUnitPolicy::RangeLimit());
    return *this;
  }

  inline tAngle &operator -= (const tAngle &other)
  {
    this->value = TSignPolicy::FitIntoRange(this->value - static_cast<double>(other), TUnitPolicy::RangeLimit());
    return *this;
  }

  template <typename TOtherUnitPolicy, typename TOtherSignPolicy>
  inline typename boost::disable_if<boost::is_same<tAngle, tAngle<TOtherUnitPolicy, TOtherSignPolicy> >, tAngle>::type &operator -= (const tAngle<TOtherUnitPolicy, TOtherSignPolicy> &other)
  {
    this->value = TSignPolicy::FitIntoRange(this->value - TUnitPolicy::ConvertFromUnit(static_cast<double>(other), TOtherUnitPolicy()), TUnitPolicy::RangeLimit());
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

};

template <typename TUnitPolicy, typename TSignPolicy>
inline const tAngle<TUnitPolicy, TSignPolicy> operator - (const tAngle<TUnitPolicy, TSignPolicy> &angle)
{
  return tAngle<TUnitPolicy, TSignPolicy>(-static_cast<double>(angle));
}

template <typename TUnitPolicy, typename TSignPolicy>
inline const tAngle<TUnitPolicy, TSignPolicy> operator + (const tAngle<TUnitPolicy, TSignPolicy> &left, const tAngle<TUnitPolicy, TSignPolicy> &right)
{
  tAngle<TUnitPolicy, TSignPolicy> temp(left);
  temp += right;
  return temp;
}

template <typename TUnitPolicy, typename TSignPolicy>
inline const tAngle<TUnitPolicy, TSignPolicy> operator - (const tAngle<TUnitPolicy, TSignPolicy> &left, const tAngle<TUnitPolicy, TSignPolicy> &right)
{
  tAngle<TUnitPolicy, TSignPolicy> temp(left);
  temp -= right;
  return temp;
}

template <typename TUnitPolicy, typename TSignPolicy>
inline const tAngle<TUnitPolicy, TSignPolicy> operator *(const tAngle<TUnitPolicy, TSignPolicy> &angle, double factor)
{
  tAngle<TUnitPolicy, TSignPolicy> temp(angle);
  temp *= factor;
  return temp;
}
template <typename TUnitPolicy, typename TSignPolicy>
inline const tAngle<TUnitPolicy, TSignPolicy> operator *(double factor, const tAngle<TUnitPolicy, TSignPolicy> &angle)
{
  return angle * factor;
}

template <typename TUnitPolicy, typename TSignPolicy>
inline const tAngle<TUnitPolicy, TSignPolicy> operator / (const tAngle<TUnitPolicy, TSignPolicy> &angle, double divider)
{
  tAngle<TUnitPolicy, TSignPolicy> temp(angle);
  temp /= divider;
  return temp;
}

template <typename TUnitPolicy, typename TSignPolicy>
std::ostream &operator << (std::ostream &stream, const tAngle<TUnitPolicy, TSignPolicy> &angle)
{
  stream << (static_cast<double>(angle) / TUnitPolicy::UnitDivider()) << (TUnitPolicy::PadUnitString() ? " " : "") << TUnitPolicy::UnitString();
  return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
