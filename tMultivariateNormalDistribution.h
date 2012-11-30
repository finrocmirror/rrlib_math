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
/*!\file    tMultivariateNormalDistribution.h
 *
 * \author  Tobias Foehst
 *
 * \date    2012-11-29
 *
 * \brief   Contains tMultivariateNormalDistribution
 *
 * \b tMultivariateNormalDistribution
 *
 * A normal random distribution in multi-dimensional space.
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__tMultivariateNormalDistribution_h__
#define __rrlib__math__tMultivariateNormalDistribution_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <random>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"
#include "rrlib/math/tMatrix.h"
#include "rrlib/math/tCholeskyDecomposition.h"

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
//! SHORT_DESCRIPTION
/*!
 * This class implements a normal continuous distribution of random numbers
 * in multi-dimensional space. It therefore draws from a zero mean, unit
 * variance normal distribution for each dimension and then applies an
 * affine transformation to the resulting sample, reflecting the mean and
 * covariance.
 *
 * If you just want to use a vector of uncorrelated variances, the needed
 * covariance matrix is diag(variance).
 */
template <size_t Tdimension, typename TElement>
class tMultivariateNormalDistribution
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  typedef tVector<Tdimension, TElement> tSample;                   //! The type of the range of the distribution
  typedef tMatrix<Tdimension, Tdimension, TElement> tCovariance;   //! The type of the distribution's covariance matrix

  /*! Constructor
   *
   * Constructs a multivariate normal distribution with parameters \a mean and \a covariance
   *
   * \param mean         The mean of this distribution
   * \param covariance   The covariance matrix of this distribution. Must be symmetric and positive definite
   */
  explicit inline tMultivariateNormalDistribution(const tSample &mean = tSample::Zero(), const tCovariance &covariance = tCovariance::Identity()) :
    normal_distribution(0, 1),
    mean(mean),
    covariance(covariance),
    affine_transformation(tCholeskyDecomposition<Tdimension, TElement>(tMatrix<Tdimension, Tdimension, TElement, matrix::Symmetrical>(covariance)).C())
  {}

  /*! Generating function
   *
   * Draws a sample from this distribution
   *
   * \param engine   A uniform random number generator like e.g. std::mt19937
   *
   * \returns A drawn sample from this distribution.
   */
  template <typename TUniformRandomNumberGenerator>
  inline tSample operator()(TUniformRandomNumberGenerator &engine)
  {
    tSample sample;
    for (size_t i = 0; i < Tdimension; ++i)
    {
      sample[i] = this->normal_distribution(engine);
    }

    return this->mean + this->affine_transformation * sample;
  }

  /*! The mean of this distribution
   *
   * \returns The mean
   */
  inline const tSample &Mean() const
  {
    return this->mean;
  }

  /*! The covariance of this distribution
   *
   * \returns The covariance
   */
  inline const tCovariance &Covariance() const
  {
    return this->covariance;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  std::normal_distribution<TElement> normal_distribution;
  tSample mean;
  tCovariance covariance;
  tMatrix<Tdimension, Tdimension, TElement, matrix::LowerTriangle> affine_transformation;

};

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
