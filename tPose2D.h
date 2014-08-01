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

#include "rrlib/localization/tPose.h"

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
class tPose3D;

//----------------------------------------------------------------------
// Class declaration
//----------------------------------------------------------------------
//!
/*!
 *
 */
class tPose2D : public localization::tPose2D<>
{

  typedef localization::tPose2D<> tBase;

//----------------------------------------------------------------------
// Public methods and typedefs
//----------------------------------------------------------------------
public:

  static const tPose2D &Zero()
  {
    static tPose2D pose;
    return pose;
  }

  inline tPose2D() :
    tBase()
  {}

  explicit inline tPose2D(const tVec2d &position, tAngleRad yaw = tAngleRad()) :
    tBase(position.X(), position.Y(), yaw)
  {}

  inline tPose2D(double x, double y, tAngleRad yaw = tAngleRad()) :
    tBase(x, y, yaw)
  {}

  inline tPose2D(const tMat3x3d &matrix, double max_error = 1E-6) :
    tBase(matrix, max_error)
  {}

  inline tPose2D(const localization::tPose3D<> &other) :
    tBase(other)
  {}

  inline tPose2D(const tBase &base) :
    tBase(base)
  {}

  inline const char *Description() const
  {
    return "tPose2D";
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

  inline const tAngleRad Yaw() const
  {
    return reinterpret_cast<const tAngleRad &>(tBase::Yaw());
  }
  inline tAngleRad &Yaw()
  {
    return reinterpret_cast<tAngleRad &>(tBase::Yaw());
  }

  inline const tVec2d &Position() const
  {
    return reinterpret_cast<const tVec2d &>(tBase::Position());
  }
  inline tVec2d &Position()
  {
    return reinterpret_cast<tVec2d &>(tBase::Position());
  }

  inline void SetPosition(const tVec2d &position)
  {
    tBase::SetPosition(position.X(), position.Y());
  }

  inline void SetPosition(double x, double y)
  {
    tBase::SetPosition(x, y);
  }

  inline void SetOrientation(tAngleRad yaw)
  {
    tBase::SetOrientation(yaw);
  }

  inline void SetOrientation(const tMat2x2d &matrix, double max_error = 1E-6)
  {
    tBase::SetOrientation(matrix, max_error);
  }

  inline void Set(const tVec2d &position, tAngleRad yaw = tAngleRad())
  {
    this->SetPosition(position);
    this->SetOrientation(yaw);
  }

  inline void Set(double x, double y, tAngleRad yaw = tAngleRad())
  {
    this->SetPosition(x, y);
    this->SetOrientation(yaw);
  }

  inline void Set(const tMat3x3d &matrix, double max_error = 1E-6)
  {
    tBase::Set(matrix, max_error);
  }

  inline tPose2D &operator += (const tPose2D &other)
  {
    tBase::operator +=(other);
    return *this;
  }

  tPose2D &operator -= (const tPose2D &other)
  {
    tBase::operator -=(other);
    return *this;
  }

  tMat2x2d GetRotationMatrix() const
  {
    return this->Orientation().GetMatrix();
  }

  void GetRotationMatrix(tMat2x2d &matrix) const
  {
    this->Orientation().GetMatrix(matrix);
  }

  inline const tPose2D GetPoseInParentFrame(const tPose2D &reference) const
  {
    return tPose2D(tBase::GetPoseInParentFrame(reference));
  }

  inline const tPose2D GetPoseInLocalFrame(const tPose2D &reference) const
  {
    return tPose2D(tBase::GetPoseInLocalFrame(reference));
  }

  inline tPose2D &Translate(const tVec2d &translation)
  {
    tBase::Translate(translation);
    return reinterpret_cast<tPose2D &>(*this);
  }

  inline tPose2D Translated(const tVec2d &translation) const
  {
    return tPose2D(tBase::Translated(translation));
  }

  inline tPose2D &Rotate(tAngleRad angle)
  {
    tBase::Rotate(angle);
    return reinterpret_cast<tPose2D &>(*this);
  }

  inline tPose2D &Rotate(const tMat2x2d &matrix)
  {
    tBase::Rotate(matrix);
    return reinterpret_cast<tPose2D &>(*this);
  }

  inline tPose2D Rotated(tAngleRad angle) const
  {
    return tPose2D(tBase::Rotated(angle));
  }

  inline tPose2D Rotated(const tMat2x2d &matrix) const
  {
    return tPose2D(tBase::Rotated(matrix));
  }

  inline tPose2D &Scale(double factor)
  {
    tBase::Scale(factor);
    return reinterpret_cast<tPose2D &>(*this);
  }

  inline tPose2D Scaled(double factor) const
  {
    return tPose2D(tBase::Scaled(factor));
  }

  inline tPose2D &ApplyRelativePoseTransformation(const tPose2D &pose)
  {
    tBase::ApplyRelativePoseTransformation(pose);
    return *this;
  }

  inline void ApplyPose(const tPose2D &pose)
  {
    ApplyRelativePoseTransformation(pose);
  }

};

inline const tPose2D operator - (const tPose2D &pose)
{
  return tPose2D(-localization::tPose2D<>(pose));
}

inline const tPose2D operator + (const tPose2D &left, const tPose2D &right)
{
  return tPose2D(localization::tPose2D<>(left) + localization::tPose2D<>(right));
}

inline const tPose2D operator - (const tPose2D &left, const tPose2D &right)
{
  return tPose2D(localization::tPose2D<>(left) - localization::tPose2D<>(right));
}

inline const tPose2D Compound(const tPose2D &base, const tPose2D &diff)
{
  return diff.GetPoseInParentFrame(base);
}
inline const tPose2D InverseCompound(const tPose2D &target, const tPose2D &base)
{
  return target.GetPoseInLocalFrame(base);
}

inline std::istream &operator >> (std::istream &stream, tPose2D &pose)
{
  char temp;
  stream >> temp;
  if (temp == '(')
  {
    tAngleDeg yaw;
    stream >> pose.X() >> temp >> pose.Y() >> temp >> yaw >> temp;
    pose.Yaw() = yaw;
  }
  else
  {
    stream.putback(temp);
    double yaw;
    stream >> pose.X() >> pose.Y() >> yaw;
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
