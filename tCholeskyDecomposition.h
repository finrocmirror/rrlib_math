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
/*!\file    tCholeskyDecomposition.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-12-30
 *
 * \brief   Contains tCholeskyDecomposition
 *
 * \b tCholeskyDecomposition
 *
 * A few words for tCholeskyDecomposition
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_tCholeskyDecomposition_h_
#define _rrlib_math_tCholeskyDecomposition_h_

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
//! Short description of tCholeskyDecomposition
/*! A more detailed description of tCholeskyDecomposition, which
    Tobias Foehst hasn't done yet !!
*/
template <size_t Trank, typename TElement>
class tCholeskyDecomposition
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  tCholeskyDecomposition(const tMatrix<Trank, Trank, TElement, matrix::Symmetrical> &matrix);

  inline const tMatrix<Trank, Trank, TElement, matrix::LowerTriangle> &C() const
  {
    return this->cholesky_matrix;
  }

  const tVector<Trank, TElement> Solve(const tVector<Trank, TElement> &right_side);

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tMatrix<Trank, Trank, TElement, matrix::LowerTriangle> cholesky_matrix;

};

//----------------------------------------------------------------------
// Explicit template instantiation
//----------------------------------------------------------------------

extern template class tCholeskyDecomposition<2, float>;
extern template class tCholeskyDecomposition<3, float>;

extern template class tCholeskyDecomposition<2, double>;
extern template class tCholeskyDecomposition<3, double>;

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/math/tCholeskyDecomposition.hpp"

#endif