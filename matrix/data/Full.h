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
/*!\file    Full.h
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
#ifndef _rrlib_math_matrix_include_guard_
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef _rrlib_math_matrix_data_Full_h_
#define _rrlib_math_matrix_data_Full_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <stdexcept>
#include <sstream>
#include <cstring>

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
class Full
{
  TElement values[Trows * Tcolumns];

  Full(const Full &other);
  Full &operator = (const Full &);

protected:

  inline Full() {};

public:

  class Accessor
  {
    TElement *values;
    size_t row;
  public:
    inline Accessor(TElement *values, size_t row) : values(values), row(row) {}
    inline const TElement operator [](size_t column) const
    {
      return const_cast<Accessor &>(*this)[column];
    }
    inline TElement &operator [](size_t column)
    {
      if (this->row >= Trows || column >= Tcolumns)
      {
        std::stringstream stream;
        stream << "Array index (" << this->row << ", " << column << " out of bounds [0.." << Trows << "][0.." << Tcolumns << "].";
        throw std::logic_error(stream.str());
      }
      return this->values[this->row * Tcolumns + column];
    }
  };

  inline void SetFromArray(const TElement data[Trows * Tcolumns])
  {
    std::memcpy(this->values, data, sizeof(Full));
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
