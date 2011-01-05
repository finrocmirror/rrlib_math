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
/*!\file    tPolynomial.h
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-01
 *
 * \brief   Contains tPolynomial
 *
 * \b tPolynomial
 *
 * A few words for tPolynomial
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_tPolynomial_h_
#define _rrlib_math_tPolynomial_h_

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

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//! Short description of tPolynomial
/*! A more detailed description of tPolynomial, which
    Tobias Foehst hasn't done yet !!
*/
template <size_t Tdegree>
class tPolynomial
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef tPolynomial < Tdegree - 1 > tDerivative;

  tPolynomial();

  tPolynomial(const double coefficients[Tdegree + 1]);

  tPolynomial(const tPolynomial &other);

  template <typename ... TCoefficients>
  explicit tPolynomial(const TCoefficients... coefficients);

  virtual ~tPolynomial() {};

  tPolynomial &operator = (const tPolynomial &other);

  inline const double GetCoefficient(size_t index) const;

  inline void SetCoefficient(size_t index, double value);

  const double operator()(double x) const;

  const tDerivative GetDerivative() const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  double coefficients[Tdegree + 1];

  template <size_t index>
  inline void SetCoefficients()
  {}
  template <size_t index, typename ... TCoefficients>
  inline void SetCoefficients(double head, TCoefficients... tail)
  {
    this->coefficients[index] = head;
    this->SetCoefficients < index - 1 > (tail...);
  }

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/math/tPolynomial.hpp"

#endif
