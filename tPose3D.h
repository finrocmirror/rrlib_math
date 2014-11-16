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
#include "rrlib/localization/tPose.h"

#include "rrlib/math/tPose2D.h"

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
class tPose2D;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
class tPose3D : public localization::tPose3D<>
{

  typedef localization::tPose3D<> tBase;

//----------------------------------------------------------------------
// Public methods
//----------------------------------------------------------------------
public:

  static const tPose3D &Zero()
  {
    static tPose3D pose;
    return pose;
  }

  inline tPose3D() :
    tBase()
  {}

  inline tPose3D(const double x, double y, double z, tAngleRad roll, tAngleRad pitch, tAngleRad yaw) :
    tBase(x, y, z, roll, pitch, yaw)
  {}

  inline tPose3D(const double x, double y, double z) :
    tBase(x, y, z)
  {}

  explicit inline tPose3D(const tVec3d &position) :
    tBase(position.X(), position.Y(), position.Z())
  {}

  inline tPose3D(const tVec3d &position, tAngleRad roll, tAngleRad pitch, tAngleRad yaw) :
    tBase(position.X(), position.Y(), position.Z(), roll, pitch, yaw)
  {}

  explicit inline tPose3D(const localization::tPose2D<> &pose_2d) :
    tBase(pose_2d)
  {}

  explicit inline tPose3D(const tMat4x4d &matrix, bool use_second_solution = false, double max_error = 1E-6) :
    tBase(matrix, max_error)
  {}

  tPose3D(const tBase &base) :
    tBase(base)
  {}

  inline const char *GetDescription() const
  {
    return "tPose3D";
  }

  inline const double X() const
  {
    return reinterpret_cast<const double &>(tBase::X());
  }
  inline double &X()
  {
    return reinterpret_cast<double &>(tBase::X());
  }

  inline const double Y() const
  {
    return reinterpret_cast<const double &>(tBase::Y());
  }
  inline double &Y()
  {
    return reinterpret_cast<double &>(tBase::Y());
  }

  inline const double Z() const
  {
    return reinterpret_cast<const double &>(tBase::Z());
  }
  inline double &Z()
  {
    return reinterpret_cast<double &>(tBase::Z());
  }

  inline const tAngleRad Roll() const
  {
    return reinterpret_cast<const tAngleRad &>(tBase::Roll());
  }
  inline tAngleRad &Roll()
  {
    return reinterpret_cast<tAngleRad &>(tBase::Roll());
  }

  inline const tAngleRad Pitch() const
  {
    return reinterpret_cast<const tAngleRad &>(tBase::Pitch());
  }
  inline tAngleRad &Pitch()
  {
    return reinterpret_cast<tAngleRad &>(tBase::Pitch());
  }

  inline const tAngleRad Yaw() const
  {
    return reinterpret_cast<const tAngleRad &>(tBase::Yaw());
  }
  inline tAngleRad &Yaw()
  {
    return reinterpret_cast<tAngleRad &>(tBase::Yaw());
  }

  inline const tVec3d &Position() const
  {
    return reinterpret_cast<const tVec3d &>(tBase::Position());
  }
  inline tVec3d &Position()
  {
    return reinterpret_cast<tVec3d &>(tBase::Position());
  }

  inline void SetPosition(const tVec3d &position)
  {
    tBase::SetPosition(position.X(), position.Y(), position.Z());
  }

  inline void SetPosition(double x, double y, double z)
  {
    tBase::SetPosition(x, y, z);
  }

  inline void SetOrientation(tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
  {
    tBase::SetOrientation(roll, pitch, yaw);
  }

  inline void SetOrientation(const tMat3x3d &matrix, bool use_second_solution = false, double max_error = 1E-6)
  {
    tBase::SetOrientation(matrix, max_error);
  }

  inline void Set(const tVec3d &position, tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
  {
    tBase::Set(position.X(), position.Y(), position.Z(), roll, pitch, yaw);
  }

  inline void Set(const tVec3d &position)
  {
    tBase::SetPosition(position.X(), position.Y(), position.Z());
  }

  inline void Set(double x, double y, double z, tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
  {
    tBase::Set(x, y, z, roll, pitch, yaw);
  }

  inline void Set(double x, double y, double z)
  {
    tBase::SetPosition(x, y, z);
  }

  inline void Set(const tMat4x4d &matrix, bool use_second_solution = false, double max_error = 1E-6)
  {
    tBase::Set(matrix, max_error);
  }

  inline const tPose2D GetPose2D() const
  {
    return *this;
  }

  inline tPose3D &operator += (const tPose3D &other)
  {
    tBase::operator +=(other);
    return *this;
  }

  inline tPose3D &operator -= (const tPose3D &other)
  {
    tBase::operator -=(other);
    return *this;
  }

  const tMat3x3d GetRotationMatrix() const
  {
    return this->Orientation().GetMatrix();
  }

  void GetRotationMatrix(tMat3x3d &matrix) const
  {
    this->Orientation().GetMatrix(matrix);
  }

  inline const tPose3D GetPoseInParentFrame(const tPose3D &reference) const
  {
    return tPose3D(tBase::GetPoseInParentFrame(reference));
  }

  inline const tPose3D GetPoseInLocalFrame(const tPose3D &reference) const
  {
    return tPose3D(tBase::GetPoseInLocalFrame(reference));
  }

  /*! Transform given 3D points to a coordinate system defined by a reference pose
   *
   * \param points_begin    Begin iterator of the points to transform
   * \param points_begin    End iterator of the points to transform
   * \param in_local_frame  Assume that "reference" describes the pose of a coordinate system B in the coordinate system A:
   *                        Choose true if you want to convert points from system A to B, false if you want to transform points from B to A
   */
  template<typename TIterator>
  void TransformCoordinateSystem(TIterator points_begin, TIterator points_end, bool in_local_frame) const __attribute__((deprecated));

  inline tPose3D &Translate(const tVec3d &translation)
  {
    tBase::Translate(translation);
    return reinterpret_cast<tPose3D &>(*this);
  }

  inline tPose3D Translated(const tVec3d &translation) const
  {
    return tPose3D(tBase::Translated(translation));
  }

  inline tPose3D &Rotate(tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
  {
    tBase::Rotate(roll, pitch, yaw);
    return reinterpret_cast<tPose3D &>(*this);
  }

  inline tPose3D &Rotate(const tMat3x3d &matrix)
  {
    tBase::Rotate(matrix);
    return reinterpret_cast<tPose3D &>(*this);
  }

  inline tPose3D Rotated(tAngleRad roll, tAngleRad pitch, tAngleRad yaw) const
  {
    return tPose3D(tBase::Rotated(roll, pitch, yaw));
  }

  inline tPose3D Rotated(const tMat3x3d &matrix) const
  {
    return tPose3D(tBase::Rotated(matrix));
  }

  inline tPose3D &Scale(double factor)
  {
    tBase::Scale(factor);
    return reinterpret_cast<tPose3D &>(*this);
  }

  tPose3D Scaled(double factor) const
  {
    return tPose3D(tBase::Scaled(factor));
  }

  inline tPose3D &ApplyRelativePoseTransformation(const tPose3D &pose)
  {
    tBase::ApplyRelativePoseTransformation(pose);
    return *this;
  }

  inline void ApplyPose(const tPose3D &pose)
  {
    ApplyRelativePoseTransformation(pose);
  }

};

template<typename TIterator>
void tPose3D::TransformCoordinateSystem(TIterator points_begin, TIterator points_end, bool in_local_frame) const
{
  if (in_local_frame)
  {
    TransformVectors(points_begin, points_end, this->GetTransformationMatrix().Inverse());
  }
  else
  {
    TransformVectors(points_begin, points_end, this->GetTransformationMatrix());
  }
}

inline const tPose3D operator - (const tPose3D &pose)
{
  return tPose3D(-localization::tPose3D<>(pose));
}

inline const tPose3D operator + (const tPose3D &left, const tPose3D &right)
{
  return tPose3D(localization::tPose3D<>(left) + localization::tPose3D<>(right));
}

inline const tPose3D operator - (const tPose3D &left, const tPose3D &right)
{
  return tPose3D(localization::tPose3D<>(left) - localization::tPose3D<>(right));
}

inline std::istream &operator >> (std::istream &stream, tPose3D &pose)
{
  char temp(0);
  stream >> temp;
  if (temp == '(')
  {
    tAngleDeg roll, pitch, yaw;
    stream >> pose.X() >> temp >> pose.Y() >> temp >> pose.Z() >> temp >> roll >> temp >> pitch >> temp >> yaw >> temp;
    pose.Roll() = roll;
    pose.Pitch() = pitch;
    pose.Yaw() = yaw;
  }
  else
  {
    stream.putback(temp);
    double roll(0.0), pitch(0.0), yaw(0.0);
    stream >> pose.X() >> pose.Y() >> pose.Z() >> roll >> pitch >> yaw;
    pose.Roll() = tAngleDeg(roll);
    pose.Pitch() = tAngleDeg(pitch);
    pose.Yaw() = tAngleDeg(yaw);
  }
  return stream;
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}

#endif
