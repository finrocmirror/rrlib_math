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
/*!\file    SquareMatrixOperationsShared.h
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

#ifndef _rrlib_math_matrix_functionality_SquareMatrixOperationsShared_h_
#define _rrlib_math_matrix_functionality_SquareMatrixOperationsShared_h_

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
template <size_t Trows, size_t Tcolumns, typename TElement, template <size_t, size_t, typename> class TData>
class SquareMatrixOperationsShared
{
  typedef tMatrix<Trows, Tcolumns, TElement, TData> tMatrixType;

  SquareMatrixOperationsShared(const SquareMatrixOperationsShared &);
  SquareMatrixOperationsShared &operator = (const SquareMatrixOperationsShared &);

protected:

  inline SquareMatrixOperationsShared() {}

public:

  inline const TElement Determinant() const
  {
    return 0;
  }

};

/*!
 *
 */
template <size_t Tdimension, typename TElement, template <size_t, size_t, typename> class TData>
class SquareMatrixOperationsShared<Tdimension, Tdimension, TElement, TData>
{
  typedef tMatrix<Tdimension, Tdimension, TElement, TData> tMatrixType;

  SquareMatrixOperationsShared(const SquareMatrixOperationsShared &);
  SquareMatrixOperationsShared &operator = (const SquareMatrixOperationsShared &);

protected:

  inline SquareMatrixOperationsShared() {}

public:

  inline void Transpose()
  {
    tMatrixType *that = reinterpret_cast<tMatrixType *>(this);
    *that = that->Transposed();
  }

  inline void Invert()
  {
    tMatrixType *that = reinterpret_cast<tMatrixType *>(this);
    *that = that->Inverted();
  }

  inline const tMatrixType Inverse() const
  {
    const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
    return that->Inverted();
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
