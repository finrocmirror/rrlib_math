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
/*!\file    FunctionalitySpecialized.h
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

#ifndef _rrlib_math_matrix_functionality_FunctionalitySpecialized_h_
#define _rrlib_math_matrix_functionality_FunctionalitySpecialized_h_

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
class FunctionalitySpecialized
{
  typedef tMatrix<Trows, Tcolumns, TElement, TData> tMatrixType;

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

protected:

  inline FunctionalitySpecialized() {}
};

/*!
 *
 */
template <size_t Trows, size_t Tcolumns, typename TElement>
class FunctionalitySpecialized<Trows, Tcolumns, TElement, Full>
{
  typedef tMatrix<Trows, Tcolumns, TElement, Full> tMatrixType;

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

protected:

  inline FunctionalitySpecialized() {}

public:

  inline const tMatrix<Tcolumns, Trows, TElement, Full> Transposed() const
  {
    const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
    tMatrix<Tcolumns, Trows, TElement, Full> result;
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = 0; column < Tcolumns; ++column)
      {
        result[column][row] = (*that)[row][column];
      }
    }
    return result;
  }

};

/*!
 *
 */
template <size_t Trows, size_t Tcolumns, typename TElement>
class FunctionalitySpecialized<Trows, Tcolumns, TElement, LowerTriangle>
{
  typedef tMatrix<Trows, Tcolumns, TElement, LowerTriangle> tMatrixType;

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

protected:

  inline FunctionalitySpecialized() {}

public:

  inline const tMatrix<Tcolumns, Trows, TElement, UpperTriangle> Transposed() const
  {
    const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
    tMatrix<Tcolumns, Trows, TElement, UpperTriangle> result;
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = 0; column <= row; ++column)
      {
        result[column][row] = (*that)[row][column];
      }
    }
    return result;
  }

};

/*!
 *
 */
template <size_t Trows, size_t Tcolumns, typename TElement>
class FunctionalitySpecialized<Trows, Tcolumns, TElement, UpperTriangle>
{
  typedef tMatrix<Trows, Tcolumns, TElement, UpperTriangle> tMatrixType;

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

protected:

  inline FunctionalitySpecialized() {}

public:

  inline const tMatrix<Tcolumns, Trows, TElement, LowerTriangle> Transposed() const
  {
    const tMatrixType *that = reinterpret_cast<const tMatrixType *>(this);
    tMatrix<Tcolumns, Trows, TElement, LowerTriangle> result;
    for (size_t row = 0; row < Trows; ++row)
    {
      for (size_t column = row; column < Tcolumns; ++column)
      {
        result[column][row] = (*that)[row][column];
      }
    }
    return result;
  }

};

/*!
 *
 */
template <size_t Trows, size_t Tcolumns, typename TElement>
class FunctionalitySpecialized<Trows, Tcolumns, TElement, Symmetrical>
{
  typedef tMatrix<Trows, Tcolumns, TElement, Symmetrical> tMatrixType;

  FunctionalitySpecialized(const FunctionalitySpecialized &);
  FunctionalitySpecialized &operator = (const FunctionalitySpecialized &);

protected:

  inline FunctionalitySpecialized() {}

public:

  inline const tMatrix<Tcolumns, Trows, TElement, Symmetrical> &Transposed() const
  {
    return reinterpret_cast<const tMatrixType &>(*this);
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
