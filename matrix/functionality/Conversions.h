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
/*!\file    Conversions.h
 *
 * \author  Tobias Foehst
 * \author  Jens Wettach
 *
 * \date    2011-01-04
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

#ifndef __rrlib__math__matrix__functionality__Conversions_h__
#define __rrlib__math__matrix__functionality__Conversions_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cmath>

#ifdef _LIB_OIV_PRESENT_
#include <Inventor/SbMatrix.h>
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
template < size_t Trows, size_t Tcolumns, typename TElement = double, template <size_t, size_t, typename> class TData = matrix::Full >
class Conversions
{

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Conversions() {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);
};

/*!
 *
 */
template <typename TElement>
class Conversions<4, 4, TElement, matrix::Full>
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

#ifdef _LIB_OIV_PRESENT_
  inline const SbMatrix GetCoinMatrix() const
  {
    const math::tMatrix<4, 4, TElement, matrix::Full>* that = reinterpret_cast<const math::tMatrix<4, 4, TElement, matrix::Full> *>(this);
    return SbMatrix(
             (*that)[0][0], (*that)[1][0], (*that)[2][0], (*that)[3][0],
             (*that)[0][1], (*that)[1][1], (*that)[2][1], (*that)[3][1],
             (*that)[0][2], (*that)[1][2], (*that)[2][2], (*that)[3][2],
             (*that)[0][3], (*that)[1][3], (*that)[2][3], (*that)[3][3]
           );
  }
#endif

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline Conversions() {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  Conversions(const Conversions &);
  Conversions &operator = (const Conversions &);

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
