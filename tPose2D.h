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
/*!\file    tPose2D.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief Contains tPose2D
 *
 * \b tPose2D
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_tPose2D_h_
#define _rrlib_math_tPose2D_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"
#include "rrlib/math/tMatrix.h"
#include "rrlib/math/tAngle.h"

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
//!
/*!
 *
 */
class tPose2D
{
  tVec2d position;
  tAngleRad yaw;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  static const tPose2D &Zero()
  {
    static tPose2D pose;
    return pose;
  }

  tPose2D();

  explicit tPose2D(const tVec2d &position, tAngleRad yaw = 0);

  tPose2D(double x, double y, tAngleRad yaw = 0);

  tPose2D(const tMat3x3d &matrix);

  inline const char *GetDescription() const
  {
    return "tPose2D";
  }

  inline const double X() const
  {
    return this->position.X();
  }
  inline double &X()
  {
    return this->position.X();
  }

  inline const double Y() const
  {
    return this->position.Y();
  }
  inline double &Y()
  {
    return this->position.Y();
  }

  inline const tAngleRad Yaw() const
  {
    return this->yaw;
  }
  inline tAngleRad &Yaw()
  {
    return this->yaw;
  }

  inline const tVec2d &GetPosition() const
  {
    return this->position;
  }

  void SetPosition(const tVec2d &position);

  void SetPosition(double x, double y);

  void SetOrientation(tAngleRad yaw);

  void SetOrientation(const tMat2x2d &matrix);

  void Set(const tVec2d &position, tAngleRad yaw = 0);

  void Set(double x, double y, tAngleRad yaw = 0);

  void Set(const tMat3x3d &matrix);

  void Reset();

  tPose2D &operator += (const tPose2D &other);

  tPose2D &operator -= (const tPose2D &other);

  const tMat2x2d GetRotationMatrix() const;

  void GetRotationMatrix(tMat2x2d &matrix) const;

  const tMat3x3d GetTransformationMatrix() const;

  void GetTransformationMatrix(tMat3x3d &matrix) const;

  const tPose2D GetPoseInParentFrame(const tPose2D &reference) const;

  const tPose2D GetPoseInLocalFrame(const tPose2D &reference) const;

  const tPose2D ToGlobal(const tPose2D &reference) const __attribute__((deprecated));

  const tPose2D ToLocal(const tPose2D &reference) const __attribute__((deprecated));

  void Translate(const tVec2d &translation);

  void Rotate(tAngleRad angle);

  void Rotate(const tMat2x2d &matrix);

  void Scale(double factor);

  void ApplyRelativePoseTransformation(const tPose2D &pose);

  void ApplyPose(const tPose2D &pose) __attribute__((deprecated));

};

const tPose2D operator - (const tPose2D &other);

const tPose2D operator + (const tPose2D &left, const tPose2D &right);

const tPose2D operator - (const tPose2D &left, const tPose2D &right);

const bool operator == (const tPose2D &left, const tPose2D &right);

const bool operator != (const tPose2D &left, const tPose2D &right);

const bool operator < (const tPose2D &left, const tPose2D &right);

const tPose2D Compound(const tPose2D &base, const tPose2D &diff) __attribute__((deprecated));

const tPose2D InverseCompound(const tPose2D &target, const tPose2D &base) __attribute__((deprecated));

std::ostream &operator << (std::ostream &stream, const tPose2D &pose);

std::istream &operator >> (std::istream &stream, tPose2D &pose);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
