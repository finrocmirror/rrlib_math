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
/*!\file    rrlib/math/matrix/functionality/LegacyShared.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-21
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__matrix__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef __rrlib__math__matrix__functionality__LegacyShared_h__
#define __rrlib__math__matrix__functionality__LegacyShared_h__

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
namespace matrix
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
template <size_t Trows, size_t Tcolumns, typename TElement>
class LegacyShared
{
  typedef math::tMatrix<Trows, Tcolumns, TElement> tMatrix;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <typename ... TValues>
  void SetMatrix(TValues... values) __attribute__((deprecated));

  double Det() const __attribute__((deprecated));

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline LegacyShared() {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  LegacyShared(const LegacyShared &);
  LegacyShared &operator = (const LegacyShared &);

};

template <size_t Trows, size_t Tcolumns, typename TElement>
template <typename ... TValues>
void LegacyShared<Trows, Tcolumns, TElement>::SetMatrix(TValues... values)
{
  tMatrix *that = reinterpret_cast<tMatrix *>(this);
  that->Set(values...);
}

template <size_t Trows, size_t Tcolumns, typename TElement>
double LegacyShared<Trows, Tcolumns, TElement>::Det() const
{
  const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
  return that->Determinant();
}


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
