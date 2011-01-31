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
#ifndef __rrlib__math__matrix__include_guard__
#error Invalid include directive. Try #include "rrlib/math/tMatrix.h" instead.
#endif

#ifndef __rrlib__math__matrix__functionality__SquareMatrixOperationsShared_h__
#define __rrlib__math__matrix__functionality__SquareMatrixOperationsShared_h__

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
  typedef math::tMatrix<Trows, Tcolumns, TElement, TData> tMatrix;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline const TElement Determinant() const
  {
    return 0;
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline SquareMatrixOperationsShared() {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  SquareMatrixOperationsShared(const SquareMatrixOperationsShared &);
  SquareMatrixOperationsShared &operator = (const SquareMatrixOperationsShared &);

};

/*!
 *
 */
template <size_t Tdimension, typename TElement, template <size_t, size_t, typename> class TData>
class SquareMatrixOperationsShared<Tdimension, Tdimension, TElement, TData>
{
  typedef math::tMatrix<Tdimension, Tdimension, TElement, TData> tMatrix;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline void Transpose()
  {
    tMatrix *that = reinterpret_cast<tMatrix *>(this);
    *that = that->Transposed();
  }

  inline void Invert()
  {
    tMatrix *that = reinterpret_cast<tMatrix *>(this);
    *that = that->Inverted();
  }

  inline const tMatrix Inverse() const
  {
    const tMatrix *that = reinterpret_cast<const tMatrix *>(this);
    return that->Inverted();
  }

//----------------------------------------------------------------------
// Protected methods
//----------------------------------------------------------------------
protected:

  inline SquareMatrixOperationsShared() {}

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  SquareMatrixOperationsShared(const SquareMatrixOperationsShared &);
  SquareMatrixOperationsShared &operator = (const SquareMatrixOperationsShared &);

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
