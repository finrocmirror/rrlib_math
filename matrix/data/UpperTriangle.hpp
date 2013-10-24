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
/*!\file    rrlib/math/matrix/data/UpperTriangle.hpp
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

#ifndef __rrlib__math__matrix__data__UpperTriangle_hpp__
#define __rrlib__math__matrix__data__UpperTriangle_hpp__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <stdexcept>
#include <sstream>
#include <iomanip>

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
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// LowerTriangle::Accessor constructors
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
UpperTriangle<Trows, Tcolumns, TElement>::Accessor::Accessor(TElement *values, size_t row)
  : values(values), row(row)
{}

//----------------------------------------------------------------------
// LowerTriangle::Accessor operator []
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
const TElement UpperTriangle<Trows, Tcolumns, TElement>::Accessor::operator [](size_t column) const
{
  if (column < this->row)
  {
    return 0;
  }
  return const_cast<Accessor &>(*this)[column];
}

template <size_t Trows, size_t Tcolumns, typename TElement>
TElement &UpperTriangle<Trows, Tcolumns, TElement>::Accessor::operator [](size_t column)
{
  if (this->row >= Trows || column >= Tcolumns)
  {
    std::stringstream stream;
    stream << "Array index (" << this->row << ", " << column << ") out of bounds [0.." << Trows << "][0.." << Tcolumns << "].";
    throw std::logic_error(stream.str());
  }
  if (column < this->row)
  {
    std::stringstream stream;
    stream << "Non-const access to fixed zero value in upper triangle matrix (" << this->row << ", " << column << ").";
    throw std::logic_error(stream.str());
  }
  return this->values[column * (column + 1) / 2  + this->row];
}

//----------------------------------------------------------------------
// LowerTriangle SetFromArray
//----------------------------------------------------------------------
template <size_t Trows, size_t Tcolumns, typename TElement>
void UpperTriangle<Trows, Tcolumns, TElement>::SetFromArray(const TElement data[Trows * Tcolumns])
{
  for (size_t row = 0; row < Trows; ++row)
  {
    for (size_t column = 0; column < Tcolumns; ++column)
    {
      if (column < row)
      {
        if (!IsEqual(data[row * Tcolumns + column], 0.0))
        {
          std::stringstream stream;
          stream << "Trying to initialize upper triangle matrix from invalid data set " << std::setprecision(20) << std::fixed << math::tMatrix<Trows, Tcolumns, TElement, Full>(data) << ".";
          throw std::runtime_error(stream.str());
        }
        continue;
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
