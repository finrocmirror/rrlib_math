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
/*!\file    tPolynomial.hpp
 *
 * \author  Tobias Foehst
 *
 * \date    2011-01-01
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstring>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

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
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// tPolynomial constructors
//----------------------------------------------------------------------
template <size_t Tdegree>
tPolynomial<Tdegree>::tPolynomial()
{
  std::memset(this->coefficients, 0, sizeof(this->coefficients));
}

template <size_t Tdegree>
tPolynomial<Tdegree>::tPolynomial(const double coefficients[Tdegree + 1])
{
  std::memcpy(this->coefficients, coefficients, sizeof(this->coefficients));
}

template <size_t Tdegree>
template <typename ... TCoefficients>
tPolynomial<Tdegree>::tPolynomial(double coefficient, TCoefficients... coefficients)
{
  static_assert(sizeof...(coefficients) <= Tdegree, "Too many coefficients specified");
  this->SetCoefficients<Tdegree>(coefficient, coefficients...);
}

template <size_t Tdegree>
tPolynomial<Tdegree>::tPolynomial(const tPolynomial &other)
{
  std::memcpy(this->coefficients, other.coefficients, sizeof(this->coefficients));
}

//----------------------------------------------------------------------
// tPolynomial operator =
//----------------------------------------------------------------------
template <size_t Tdegree>
tPolynomial<Tdegree> &tPolynomial<Tdegree>::operator = (const tPolynomial &other)
{
  if (this == &other)
  {
    return;
  }
  std::memcpy(this->coefficients, other.coefficients, sizeof(this->coefficients));
}

//----------------------------------------------------------------------
// tPolynomial GetCoefficient
//----------------------------------------------------------------------
template <size_t Tdegree>
const double tPolynomial<Tdegree>::GetCoefficient(size_t index) const
{
  assert(index < Tdegree + 1);
  return this->coefficients[index];
}

//----------------------------------------------------------------------
// tPolynomial SetCoefficient
//----------------------------------------------------------------------
template <size_t Tdegree>
void tPolynomial<Tdegree>::SetCoefficient(size_t index, double value)
{
  assert(index < Tdegree + 1);
  this->coefficients[index] = value;
}

//----------------------------------------------------------------------
// tPolynomial Evaluation: operator ()
//----------------------------------------------------------------------
template <size_t Tdegree>
const double tPolynomial<Tdegree>::operator()(double x) const
{
  double factor = 1;
  double result = 0;
  for (size_t i = 0; i < Tdegree + 1; ++i)
  {
    result += this->coefficients[i] * factor;
    factor *= x;
  }
  return result;
}

//----------------------------------------------------------------------
// tPolynomial GetDerivative
//----------------------------------------------------------------------
template <size_t Tdegree>
const typename tPolynomial<Tdegree>::tDerivative tPolynomial<Tdegree>::GetDerivative() const
{
  double temp[Tdegree];
  for (size_t i = 1; i < Tdegree + 1; ++i)
  {
    temp[i - 1] = this->coefficients[i] * i;
  }
  return tDerivative(temp);
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
