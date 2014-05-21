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
/*!\file    rrlib/math/tPose2D.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 */
//----------------------------------------------------------------------
#include "rrlib/math/tPose2D.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/utilities.h"
#include "rrlib/math/tPose3D.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::math;

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
// tPose2D constructors
//----------------------------------------------------------------------
tPose2D::tPose2D()
  : yaw(0)
{}

tPose2D::tPose2D(const tVec2d &position, tAngleRad yaw)
  : position(position),
    yaw(yaw)
{}

tPose2D::tPose2D(double x, double y, tAngleRad yaw)
  : position(x, y),
    yaw(yaw)
{}

tPose2D::tPose2D(const tMat3x3d &matrix, double max_error)
  : yaw(0)
{
  this->Set(matrix, max_error);
}

tPose2D::tPose2D(const tPose3D &other)
  : position(other.Position()),
    yaw(other.Yaw())
{}

//----------------------------------------------------------------------
// tPose2D SetPosition
//----------------------------------------------------------------------
void tPose2D::SetPosition(const tVec2d &position)
{
  this->position = position;
}

void tPose2D::SetPosition(double x, double y)
{
  this->position.Set(x, y);
}

//----------------------------------------------------------------------
// tPose2D SetOrientation
//----------------------------------------------------------------------
void tPose2D::SetOrientation(tAngleRad yaw)
{
  this->yaw = yaw;
}

void tPose2D::SetOrientation(const tMat2x2d &matrix, double max_error)
{
  assert(IsEqual(matrix.Determinant(), 1.0, max_error));
  this->yaw = std::atan2(matrix[1][0], matrix[0][0]);
}

//----------------------------------------------------------------------
// tPose2D Set
//----------------------------------------------------------------------
void tPose2D::Set(const tVec2d &position, tAngleRad yaw)
{
  this->SetPosition(position);
  this->SetOrientation(yaw);
}

void tPose2D::Set(double x, double y, tAngleRad yaw)
{
  this->position.Set(x, y);
  this->SetOrientation(yaw);
}

void tPose2D::Set(const tMat3x3d &matrix, double max_error)
{
  assert(IsEqual(matrix[2][0], 0.0) && IsEqual(matrix[2][1], 0.0) && IsEqual(matrix[2][2], 1.0));
  this->position.Set(matrix[0][2], matrix[1][2]);
  this->SetOrientation(tMat2x2d(matrix[0][0], matrix[0][1], matrix[1][0], matrix[1][1]), max_error);
}

//----------------------------------------------------------------------
// tPose2D Reset
//----------------------------------------------------------------------
void tPose2D::Reset()
{
  this->position = tVec2d::Zero();
  this->yaw = 0;
}

//----------------------------------------------------------------------
// tPose2D operator +=
//----------------------------------------------------------------------
tPose2D &tPose2D::operator += (const tPose2D & other)
{
  this->position += other.position;
  this->yaw += other.yaw;
  return *this;
}

//----------------------------------------------------------------------
// tPose2D operator -=
//----------------------------------------------------------------------
tPose2D &tPose2D::operator -= (const tPose2D & other)
{
  this->position -= other.position;
  this->yaw -= other.yaw;
  return *this;
}

//----------------------------------------------------------------------
// tPose2D GetRotationMatrix
//----------------------------------------------------------------------
const tMat2x2d tPose2D::GetRotationMatrix() const
{
  return Get2DRotationMatrix<double>(this->yaw);
}

void tPose2D::GetRotationMatrix(tMat2x2d &matrix) const
{
  matrix = this->GetRotationMatrix();
}

//----------------------------------------------------------------------
// tPose2D GetTransformationMatrix
//----------------------------------------------------------------------
const tMat3x3d tPose2D::GetTransformationMatrix() const
{
  tMat2x2d rotation = this->GetRotationMatrix();
  return tMat3x3d(rotation[0][0], rotation[0][1], this->X(),
                  rotation[1][0], rotation[1][1], this->Y(),
                  0, 0, 1);
}

void tPose2D::GetTransformationMatrix(tMat3x3d &matrix) const
{
  matrix = this->GetTransformationMatrix();
}

//----------------------------------------------------------------------
// tPose2D GetPoseInParentFrame
//----------------------------------------------------------------------
const tPose2D tPose2D::GetPoseInParentFrame(const tPose2D &reference) const
{
  return tPose2D(reference.position + this->position.Rotated(reference.yaw), reference.yaw + this->yaw);
}

//----------------------------------------------------------------------
// tPose2D GetPoseInLocalFrame
//----------------------------------------------------------------------
const tPose2D tPose2D::GetPoseInLocalFrame(const tPose2D &reference) const
{
  return tPose2D((this->position - reference.position).Rotated(-reference.yaw), this->yaw - reference.yaw);
}

//----------------------------------------------------------------------
// tPose2D Translate
//----------------------------------------------------------------------
tPose2D &tPose2D::Translate(const tVec2d &translation)
{
  this->position += translation;
  return *this;
}

//----------------------------------------------------------------------
// tPose2D Translated
//----------------------------------------------------------------------
tPose2D tPose2D::Translated(const tVec2d &translation) const
{
  return tPose2D(this->position + translation, this->yaw);
}

//----------------------------------------------------------------------
// tPose2D Rotate
//----------------------------------------------------------------------
tPose2D &tPose2D::Rotate(tAngleRad angle)
{
  this->yaw += angle;
  return *this;
}

tPose2D &tPose2D::Rotate(const tMat2x2d &matrix)
{
  this->SetOrientation(matrix * this->GetRotationMatrix());
  return *this;
}

//----------------------------------------------------------------------
// tPose2D Rotated
//----------------------------------------------------------------------
tPose2D tPose2D::Rotated(tAngleRad angle) const
{
  return tPose2D(this->position, this->yaw + angle);
}

tPose2D tPose2D::Rotated(const tMat2x2d &matrix) const
{
  tPose2D temp(*this);
  temp.Rotate(matrix);
  return temp;
}

//----------------------------------------------------------------------
// tPose2D Scale
//----------------------------------------------------------------------
tPose2D &tPose2D::Scale(double factor)
{
  this->position *= factor;
  return *this;
}

//----------------------------------------------------------------------
// tPose2D Scaled
//----------------------------------------------------------------------
tPose2D tPose2D::Scaled(double factor) const
{
  return tPose2D(this->position * factor, this->yaw);
}

//----------------------------------------------------------------------
// tPose2D ApplyRelativePoseTransformation
//----------------------------------------------------------------------
tPose2D &tPose2D::ApplyRelativePoseTransformation(const tPose2D &relative_transformation)
{
  this->Translate(relative_transformation.position.Rotated(this->yaw));
  this->Rotate(relative_transformation.yaw);
  return *this;
}

//----------------------------------------------------------------------
// tPose2D ApplyPose
//----------------------------------------------------------------------
void tPose2D::ApplyPose(const tPose2D &relative_transformation)
{
  this->ApplyRelativePoseTransformation(relative_transformation);
}

//----------------------------------------------------------------------
// tPose2D GetEuclideanNorm
//----------------------------------------------------------------------
const double tPose2D::GetEuclideanNorm() const
{
  return tVector<3, double>(this->position.X(), this->position.Y(), this->yaw).Length();
}

//----------------------------------------------------------------------
// tPose2D IsZero
//----------------------------------------------------------------------
const bool tPose2D::IsZero() const
{
  return IsEqual(*this, tPose2D::Zero());
}

//----------------------------------------------------------------------
// Unary Minus for tPose2D objects
//----------------------------------------------------------------------
const tPose2D rrlib::math::operator - (const tPose2D &pose)
{
  return tPose2D(-pose.Position(), -pose.Yaw());
}

//----------------------------------------------------------------------
// Addition of tPose2D objects
//----------------------------------------------------------------------
const tPose2D rrlib::math::operator + (const tPose2D &left, const tPose2D &right)
{
  return tPose2D(left.Position() + right.Position(), left.Yaw() + right.Yaw());
}

//----------------------------------------------------------------------
// Subtraction of tPose2D objects
//----------------------------------------------------------------------
const tPose2D rrlib::math::operator - (const tPose2D &left, const tPose2D &right)
{
  return tPose2D(left.Position() - right.Position(), left.Yaw() - right.Yaw());
}

//----------------------------------------------------------------------
// Numeric equality of tPose2D objects
//----------------------------------------------------------------------
bool rrlib::math::IsEqual(const tPose2D &left, const tPose2D &right, float max_error, tFloatComparisonMethod method)
{
  return IsEqual(left.Position(), right.Position(), max_error, method)
         && IsEqual(left.Yaw(), right.Yaw(), max_error, method);
}

//----------------------------------------------------------------------
// Equality of tPose2D objects
//----------------------------------------------------------------------
const bool rrlib::math::operator == (const tPose2D &left, const tPose2D &right)
{
  return left.Position() == right.Position() && left.Yaw() == right.Yaw();
}

//----------------------------------------------------------------------
// Inequality of tPose2D objects
//----------------------------------------------------------------------
const bool rrlib::math::operator != (const tPose2D &left, const tPose2D &right)
{
  return !(left == right);
}

//----------------------------------------------------------------------
// Ordering tPose2D objects
//----------------------------------------------------------------------
const bool rrlib::math::operator < (const tPose2D &left, const tPose2D &right)
{
  return left.Position() < right.Position() || (left.Position() == right.Position() && left.Yaw() < right.Yaw());
}

//----------------------------------------------------------------------
// Composition of tPose2D objects
//----------------------------------------------------------------------
const tPose2D rrlib::math::Compound(const tPose2D &base, const tPose2D &diff)
{
  return diff.GetPoseInParentFrame(base);
}

//----------------------------------------------------------------------
// Decomposition of tPose2D objects
//----------------------------------------------------------------------
const tPose2D rrlib::math::InverseCompound(const tPose2D &target, const tPose2D &base)
{
  return target.GetPoseInLocalFrame(base);
}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
std::ostream &rrlib::math::operator << (std::ostream &stream, const tPose2D &pose)
{
  return stream << "(" << pose.X() << ", " << pose.Y() << ", " << tAngleDeg(pose.Yaw()) << ")";
}

std::istream &rrlib::math::operator >> (std::istream &stream, tPose2D &pose)
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

#ifdef _LIB_RRLIB_SERIALIZATION_PRESENT_

rrlib::serialization::tOutputStream &rrlib::math::operator << (rrlib::serialization::tOutputStream &stream, const tPose2D &pose)
{
  stream << pose.X() << pose.Y() << pose.Yaw();
  return stream;
}

rrlib::serialization::tInputStream &rrlib::math::operator >> (rrlib::serialization::tInputStream &stream, tPose2D &pose)
{
  stream >> pose.X() >> pose.Y() >> pose.Yaw();
  return stream;
}

rrlib::serialization::tStringOutputStream &rrlib::math::operator << (rrlib::serialization::tStringOutputStream &stream, const tPose2D &pose)
{
  std::stringstream s;
  s << pose;
  stream << s.str();
  return stream;
}

rrlib::serialization::tStringInputStream &rrlib::math::operator >> (rrlib::serialization::tStringInputStream &stream, tPose2D &pose)
{
  stream.GetWrappedStringStream() >> pose;
  return stream;
}

#endif
