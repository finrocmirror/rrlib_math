//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    rrlib/math/tPolynomial.h
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
#ifndef __rrlib__math__tPolynomial_h__
#define __rrlib__math__tPolynomial_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <boost/utility/enable_if.hpp>

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

  explicit tPolynomial(const double coefficients[Tdegree + 1]);

  tPolynomial(const tPolynomial &other);

  template < size_t Tlower_degree, typename T = int >
  tPolynomial(const tPolynomial<Tlower_degree> &other, typename boost::enable_if_c < (Tlower_degree < Tdegree), T >::type = 0);

  template <typename ... TCoefficients>
  explicit tPolynomial(double coefficient, TCoefficients... coefficients);

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

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#include "rrlib/math/tPolynomial.hpp"

#endif
