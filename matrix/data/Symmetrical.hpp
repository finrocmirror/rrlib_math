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
/*!\file    rrlib/math/matrix/data/Symmetrical.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-08-25
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__matrix__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef __rrlib__math__matrix__data__Symmetrical_hpp__
#define __rrlib__math__matrix__data__Symmetrical_hpp__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <stdexcept>
#include <sstream>

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
namespace matrix
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Symmetrical::Accessor constructors
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
Symmetrical<Trows, Tcolumns, TElement>::Accessor::Accessor(TElement *values, size_t row)
  : values(values), row(row)
{}

//----------------------------------------------------------------------
// Symmetrical::Accessor operator []
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
const TElement Symmetrical<Trows, Tcolumns, TElement>::Accessor::operator [](size_t column) const
{
  return const_cast<Accessor &>(*this)[column];
}

template <size_t Trows, size_t Tcolumns, typename TElement>
TElement &Symmetrical<Trows, Tcolumns, TElement>::Accessor::operator [](size_t column)
{
  if (this->row >= Trows || column >= Tcolumns)
  {
    std::stringstream stream;
    stream << "Array index (" << this->row << ", " << column << " out of bounds [0.." << Trows << "][0.." << Tcolumns << "].";
    throw std::logic_error(stream.str());
  }
  size_t index = column > this->row ? column * (column + 1) / 2 + this->row : this->row * (this->row + 1) / 2  + column;
  return this->values[index];
}

//----------------------------------------------------------------------
// Symmetrical SetFromArray
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
void Symmetrical<Trows, Tcolumns, TElement>::SetFromArray(const TElement data[Trows * Tcolumns])
{
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = row; column < Tcolumns; ++column)
    {
      if (!IsEqual(data[row * Tcolumns + column], data[column * Trows + row], 0.0001, eFCM_RELATIVE_ERROR))
      {
        std::stringstream stream;
        stream << "Trying to initialize symmetric matrix from invalid data set " << math::tMatrix<Trows, Tcolumns, TElement, Full>(data) << ".";
        throw std::runtime_error(stream.str());
      }
      this->values[column * (column + 1) / 2 + row] = data[row * Tcolumns + column];
    }
  }
}



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
