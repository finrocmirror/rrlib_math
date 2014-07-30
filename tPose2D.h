//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    rrlib/math/tPose2D.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief   Contains tPose2D
 *
 * \b tPose2D
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__tPose2D_h__
#define __rrlib__math__tPose2D_h__

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tVector.h"
#include "rrlib/math/tMatrix.h"
#include "rrlib/math/tAngle.h"

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_
#include "rrlib/serialization/serialization.h"
#include <sstream>
#endif

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
class tPose3D;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
class tPose2D
{

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static const tPose2D &Zero()
  {
    static tPose2D pose;
    return pose;
  }

  tPose2D();

  explicit tPose2D(const tVec2d &position, tAngleRad yaw = tAngleRad());

  tPose2D(double x, double y, tAngleRad yaw = tAngleRad());

  tPose2D(const tMat3x3d &matrix, double max_error = 1E-6);

  tPose2D(const tPose3D &other);

  inline const char *Description() const
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

  inline const tVec2d &Position() const
  {
    return this->position;
  }
  inline tVec2d &Position()
  {
    return this->position;
  }

  void SetPosition(const tVec2d &position);

  void SetPosition(double x, double y);

  void SetOrientation(tAngleRad yaw);

  void SetOrientation(const tMat2x2d &matrix, double max_error = 1E-6);

  void Set(const tVec2d &position, tAngleRad yaw = tAngleRad());

  void Set(double x, double y, tAngleRad yaw = tAngleRad());

  void Set(const tMat3x3d &matrix, double max_error = 1E-6);

  void Reset();

  tPose2D &operator += (const tPose2D &other);

  tPose2D &operator -= (const tPose2D &other);

  const tMat2x2d GetRotationMatrix() const;

  void GetRotationMatrix(tMat2x2d &matrix) const;

  const tMat3x3d GetTransformationMatrix() const;

  void GetTransformationMatrix(tMat3x3d &matrix) const;

  const tPose2D GetPoseInParentFrame(const tPose2D &reference) const;

  const tPose2D GetPoseInLocalFrame(const tPose2D &reference) const;

  tPose2D &Translate(const tVec2d &translation);

  tPose2D Translated(const tVec2d &translation) const;

  tPose2D &Rotate(tAngleRad angle);

  tPose2D &Rotate(const tMat2x2d &matrix);

  tPose2D Rotated(tAngleRad angle) const;

  tPose2D Rotated(const tMat2x2d &matrix) const;

  tPose2D &Scale(double factor);

  tPose2D Scaled(double factor) const;

  tPose2D &ApplyRelativePoseTransformation(const tPose2D &pose);

  void ApplyPose(const tPose2D &pose) __attribute__((deprecated));

  const double GetEuclideanNorm() const;

  const bool IsZero() const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tVec2d position;
  tAngleRad yaw;

};

const tPose2D operator - (const tPose2D &other);

const tPose2D operator + (const tPose2D &left, const tPose2D &right);

const tPose2D operator - (const tPose2D &left, const tPose2D &right);

bool IsEqual(const tPose2D &left, const tPose2D &right, float max_error = 1E-6, tFloatComparisonMethod method = eFCM_ABSOLUTE_ERROR);

const bool operator == (const tPose2D &left, const tPose2D &right);

const bool operator != (const tPose2D &left, const tPose2D &right);

const bool operator < (const tPose2D &left, const tPose2D &right);

const tPose2D Compound(const tPose2D &base, const tPose2D &diff) __attribute__((deprecated));

const tPose2D InverseCompound(const tPose2D &target, const tPose2D &base) __attribute__((deprecated));

std::ostream &operator << (std::ostream &stream, const tPose2D &pose);

std::istream &operator >> (std::istream &stream, tPose2D &pose);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tPose2D &pose);

serialization::tInputStream &operator >> (serialization::tInputStream &stream, tPose2D &pose);

serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tPose2D &pose);

serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tPose2D &pose);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}


#endif
