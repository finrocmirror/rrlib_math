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
/*!\file    rrlib/math/tPose3D.h
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 * \brief   Contains tPose3D
 *
 * \b tPos2D
 *
 */
//----------------------------------------------------------------------
#ifndef __rrlib__math__tPose3D_h__
#define __rrlib__math__tPose3D_h__

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

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
class tPose3D
{

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

  explicit tPose3D(const tMat4x4d &matrix, bool use_second_solution = false, double max_error = 1E-6);

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

  inline const tVec3d &Position() const
  {
    return this->position;
  }

  inline tVec3d &Position()
  {
    return this->position;
  }

  void SetPosition(const tVec3d &position);

  void SetPosition(double x, double y, double z);

  void SetOrientation(tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

  void SetOrientation(const tMat3x3d &matrix, bool use_second_solution = false, double max_error = 1E-6);

  void Set(const tVec3d &position, tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

  void Set(const tVec3d &position);

  void Set(double x, double y, double z, tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

  void Set(double x, double y, double z);

  void Set(const tMat4x4d &matrix, bool use_second_solution = false, double max_error = 1E-6);

  void Reset();

  const tPose2D GetPose2D() const;

  tPose3D &operator += (const tPose3D &other);

  tPose3D &operator -= (const tPose3D &other);

  const tMat3x3d GetRotationMatrix() const;

  void GetRotationMatrix(tMat3x3d &matrix) const;

  const tMat4x4d GetTransformationMatrix() const;

  void GetTransformationMatrix(tMat4x4d &matrix) const;

  const tPose3D GetPoseInParentFrame(const tPose3D &reference) const;

  const tPose3D GetPoseInLocalFrame(const tPose3D &reference) const;

  /*! Transform given 3D points to a coordinate system defined by a reference pose
   *
   * \param points_begin    Begin iterator of the points to transform
   * \param points_begin    End iterator of the points to transform
   * \param in_local_frame  Assume that "reference" describes the pose of a coordinate system B in the coordinate system A:
   *                        Choose true if you want to convert points from system A to B, false if you want to transform points from B to A
   */
  template<typename TIterator>
  void TransformCoordinateSystem(TIterator points_begin, TIterator points_end, bool in_local_frame) const;

  tPose3D &Translate(const tVec3d &translation);

  tPose3D Translated(const tVec3d &translation) const;

  tPose3D &Rotate(tAngleRad roll, tAngleRad pitch, tAngleRad yaw);

  tPose3D &Rotate(const tMat3x3d &matrix);

  tPose3D Rotated(tAngleRad roll, tAngleRad pitch, tAngleRad yaw) const;

  tPose3D Rotated(const tMat3x3d &matrix) const;

  tPose3D &Scale(double factor);

  tPose3D Scaled(double factor) const;

  tPose3D &ApplyRelativePoseTransformation(const tPose3D &pose);

  void ApplyPose(const tPose3D &pose) __attribute__((deprecated));

  const double GetEuclideanNorm() const;

  const bool IsZero() const;

//----------------------------------------------------------------------
// Private fields and methods
//----------------------------------------------------------------------
private:

  tVec3d position;
  tAngleRad roll;
  tAngleRad pitch;
  tAngleRad yaw;

};

const tPose3D operator - (const tPose3D &other);

const tPose3D operator + (const tPose3D &left, const tPose3D &right);

const tPose3D operator - (const tPose3D &left, const tPose3D &right);

bool IsEqual(const tPose3D &left, const tPose3D &right, float max_error = 1E-6, tFloatComparisonMethod method = eFCM_ABSOLUTE_ERROR);

const bool operator == (const tPose3D &left, const tPose3D &right);

const bool operator != (const tPose3D &left, const tPose3D &right);

const bool operator < (const tPose3D &left, const tPose3D &right);

std::ostream &operator << (std::ostream &stream, const tPose3D &pose);

std::istream &operator >> (std::istream &stream, tPose3D &pose);

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

serialization::tOutputStream &operator << (serialization::tOutputStream &stream, const tPose3D &pose);

serialization::tInputStream &operator >> (serialization::tInputStream &stream, tPose3D &pose);

serialization::tStringOutputStream &operator << (serialization::tStringOutputStream &stream, const tPose3D &pose);

serialization::tStringInputStream &operator >> (serialization::tStringInputStream &stream, tPose3D &pose);

#endif

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#include "rrlib/math/tPose3D.hpp"

#endif
