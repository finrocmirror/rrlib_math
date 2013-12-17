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
/*!\file    rrlib/math/tLUDecomposition.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-28
 *
 * \brief   Contains tLUDecomposition
 *
 * \b tLUDecomposition
 *
 * A few words for tLUDecomposition
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__tLUDecomposition_h__
#define __rrlib__math__tLUDecomposition_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"
#include "rrlib/math/tMatrix.h"

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
//! Short description of tLUDecomposition
/*! A more detailed description of tLUDecomposition, which
    Tobias Foehst hasn't done yet !!
*/
template <size_t Trank, typename TElement = double>
class tLUDecomposition
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  template <size_t Trows>
  tLUDecomposition(const tMatrix<Trows, Trank, TElement> &matrix);

  inline const tMatrix<Trank, Trank, TElement> &L() const
  {
    return this->lower;
  }

  inline const tMatrix<Trank, Trank, TElement> &U() const
  {
    return this->upper;
  }

  template <size_t Tdimension>
  const tVector<Trank, TElement> Solve(const tVector<Tdimension, TElement> &right_side) const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tMatrix<Trank, Trank, TElement> lower;
  tMatrix<Trank, Trank, TElement> upper;
  tVector<Trank, size_t> pivot;

};

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tLUDecomposition<2, float>;
extern template class tLUDecomposition<3, float>;

extern template class tLUDecomposition<2, double>;
extern template class tLUDecomposition<3, double>;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/math/tLUDecomposition.hpp"

#endif
