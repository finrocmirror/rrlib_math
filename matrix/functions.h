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
/*!\file    functions.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-09-26
 *
 * \brief
 *
 * \b
 *
 * A few words for functions.h
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_matrix_functions_h_
#define _rrlib_math_matrix_functions_h_

//----------------------------------------------------------------------
// External includes with <>
//----------------------------------------------------------------------
#include <cmath>
#include <iostream>

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



template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TLeftData, template <size_t, size_t, typename> class TRightData>
inline bool IsEqual(const tMatrix<Trows, Tcolumns, TElement, TLeftData> &left, const tMatrix<Trows, Tcolumns, TElement, TRightData> &right, float max_error = 0.000001, tFloatComparisonMethod method = eFCM_ABSOLUTE_ERROR)
{
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      if (!IsEqual(left[row][column], right[row][column], max_error, method))
      {
        return false;
      }
    }
  }
  return true;
}

template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TLeftData, template <size_t, size_t, typename> class TRightData>
inline const bool operator == (const tMatrix<Trows, Tcolumns, TElement, TLeftData> &left, const tMatrix<Trows, Tcolumns, TElement, TRightData> &right)
{
  return IsEqual(left, right, 0.0);
}

template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TLeftData, template <size_t, size_t, typename> class TRightData>
inline const bool operator != (const tMatrix<Trows, Tcolumns, TElement, TLeftData> &left, const tMatrix<Trows, Tcolumns, TElement, TRightData> &right)
{
  return !(left == right);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
