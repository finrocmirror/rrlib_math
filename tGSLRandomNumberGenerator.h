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
/*!\file    sGslRandomNumberGenerator.h
 *
 * \author  Jens Wettach
 *
 * \date    2010-12-29
 *
 * \brief
 *
 * \b
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_tGSLRandomNumberGenerator_h_
#define _rrlib_math_tGSLRandomNumberGenerator_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <gsl/gsl_rng.h>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/util/patterns/singleton.h"

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
//! Short description of tGSLRandomNumberGenerator
/*! A more detailed description of tGSLRandomNumberGenerator, which
    Tobias Foehst hasn't done yet !!
*/
class tGSLRandomNumberGeneratorImplementation
{
  friend class util::singleton::CreateStatic<tGSLRandomNumberGeneratorImplementation>;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  inline gsl_rng *Generator()
  {
    return this->r;
  }

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  gsl_rng *r;  /* global generator */

  inline tGSLRandomNumberGeneratorImplementation()
      : r(0)
  {
    gsl_rng_env_setup();
    this->r = gsl_rng_alloc(gsl_rng_default);
  }

  ~tGSLRandomNumberGeneratorImplementation()
  {
    if (r)
    {
      gsl_rng_free(this->r);
    }
  }

};

typedef util::tSingletonHolder<tGSLRandomNumberGeneratorImplementation, util::singleton::CreateStatic> tGSLRandomNumberGenerator;

namespace sGslRandomNumberGenerator
{

inline gsl_rng* GetGslRandomNumberGenerator() __attribute__((deprecated));
inline gsl_rng* GetGslRandomNumberGenerator()
{
  return tGSLRandomNumberGenerator::GetInstance().Generator();
}

}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
