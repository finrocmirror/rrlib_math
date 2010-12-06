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
/*!\file    Polar.h
 *
 * \author  Tobias Foehst
 *
 * \date    2008-09-26
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_vector_include_guard_
#error Invalid include directive. Try #include "rrlib/math/tVector.h" instead.
#endif

#ifndef _rrlib_math_vector_data_Polar_h_
#define _rrlib_math_vector_data_Polar_h_

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
namespace vector
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
template <size_t Tdimension, typename TElement>
class Polar
{
  TElement angles[Tdimension - 1];
  TElement length;

  Polar(const Polar &);
  Polar &operator = (const Polar &);

protected:

  inline Polar() {}

public:

  inline TElement Length() const
  {
    return this->length;
  }
  inline TElement &Length()
  {
    return this->length;
  }

};

/*!
 *
 */
template <typename TElement>
class Polar<2, TElement>
{
  TElement alpha;
  TElement length;

  Polar(const Polar &);
  Polar &operator = (const Polar &);

protected:

  inline Polar() {}

public:

  inline TElement Alpha() const
  {
    return this->alpha;
  }
  inline TElement &Alpha()
  {
    return this->alpha;
  }
  inline TElement Length() const
  {
    return this->length;
  }
  inline TElement &Length()
  {
    return this->length;
  }

};

/*!
 *
 */
template <typename TElement>
class Polar<3, TElement>
{
  TElement alpha;
  TElement beta;
  TElement length;

  Polar(const Polar &);
  Polar &operator = (const Polar &);

protected:

  inline Polar() {}

public:

  inline TElement Alpha() const
  {
    return this->alpha;
  }
  inline TElement &Alpha()
  {
    return this->alpha;
  }
  inline TElement Beta() const
  {
    return this->beta;
  }
  inline TElement &Beta()
  {
    return this->beta;
  }
  inline TElement Length() const
  {
    return this->length;
  }
  inline TElement &Length()
  {
    return this->length;
  }

};



//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
}

#endif
