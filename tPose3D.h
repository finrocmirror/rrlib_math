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
/*!\file    tPose3D.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief Contains tPose3D
 *
 * \b tPos2D
 *
 */
//----------------------------------------------------------------------
#ifndef _rrlib_math_tPose3D_h_
#define _rrlib_math_tPose3D_h_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <iostream>

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"
#include "rrlib/math/tAngle.h"
#include "rrlib/math/tMatrix.h"
#include "rrlib/math/tPose2D.h"

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
class tPose3D
{
  tVec3d position;
  tAngleRad roll;
  tAngleRad pitch;
  tAngleRad yaw;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  static const tPose3D &Zero()
  {
    static tPose3D pose;
    return pose;
  }

  tPose3D();

  tPose3D(double x, double y, double z, tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

  tPose3D(double x, double y, double z);

  explicit tPose3D(const tVec3d &position);

  tPose3D(const tVec3d &position, tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

  explicit tPose3D(const tPose2D &pose_2d);

  explicit tPose3D(const tMat4x4d &matrix, bool use_second_solution = false);

  inline const char *GetDescription() const
  {
    return "tPose3D";
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

  inline const double Z() const
  {
    return this->position.Z();
  }
  inline double &Z()
  {
    return this->position.Z();
  }

  inline const tAngleRad Roll() const
  {
    return this->roll;
  }
  inline tAngleRad &Roll()
  {
    return this->roll;
  }

  inline const tAngleRad Pitch() const
  {
    return this->pitch;
  }
  inline tAngleRad &Pitch()
  {
    return this->pitch;
  }

  inline const tAngleRad Yaw() const
  {
    return this->yaw;
  }
  inline tAngleRad &Yaw()
  {
    return this->yaw;
  }

  inline const tVec3d &GetPosition() const
  {
    return this->position;
  }

  void SetPosition(double x, double y, double z);

  void SetPosition(const tVec3d &position);

  void SetOrientation(tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

  void SetOrientation(const tMat3x3d &matrix, bool use_second_solution = false);

  void Set(const tVec3d &position, tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

  void Set(const tVec3d &position);

  void Set(double x, double y, double z, tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

  void Set(double x, double y, double z);

  void Set(const tMat4x4d &matrix, bool use_second_solution = false);

  inline void Reset();

  const tPose2D GetPose2D() const;

  tPose3D &operator += (const tPose3D &other);

  tPose3D &operator -= (const tPose3D &other);

  const tMat3x3d GetRotationMatrix() const;

  void GetRotationMatrix(tMat3x3d &matrix) const;

  const tMat4x4d GetTransformationMatrix() const;

  void GetTransformationMatrix(tMat4x4d &matrix) const;

  const tMat4x4d GetTransformationMatrixZYXT() const;

  void GetTransformationMatrixZYXT(tMat4x4d &matrix) const;

  const tPose3D GetPoseInParentFrame(const tPose3D &reference) const;

  const tPose3D GetPoseInLocalFrame(const tPose3D &reference) const;

//  const tPose3D ToGlobal(const tPose3D &reference) const __attribute__((deprecated));

//  const tPose3D ToLocal(const tPose3D &reference) const __attribute__((deprecated));

  void Translate(const tVec3d &translation);

  void Rotate(tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

  void Rotate(const tMat3x3d &matrix);

  void Scale(double factor);

  void ApplyRelativePoseTransformation(const tPose3D &pose);

  void ApplyPose(const tPose3D &pose) __attribute__((deprecated));

};

const tPose3D operator - (const tPose3D &other);

const tPose3D operator + (const tPose3D &left, const tPose3D &right);

const tPose3D operator - (const tPose3D &left, const tPose3D &right);

const bool operator == (const tPose3D &left, const tPose3D &right);

const bool operator != (const tPose3D &left, const tPose3D &right);

const bool operator < (const tPose3D &left, const tPose3D &right);

//const tPose3D Compound(const tPose3D &base, const tPose3D &diff) __attribute__((deprecated));

//const tPose3D InverseCompound(const tPose3D &target, const tPose3D &base) __attribute__((deprecated));

std::ostream &operator << (std::ostream &stream, const tPose3D &pose);

std::istream &operator >> (std::istream &stream, tPose3D &pose);


//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
