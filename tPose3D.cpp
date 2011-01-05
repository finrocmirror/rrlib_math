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
/*!\file    tPose3D.cpp
 *
 * \author  Tobias Foehst
 *
 * \date    2010-11-28
 *
 */
//----------------------------------------------------------------------
#include "rrlib/math/tPose3D.h"

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/utilities.h"

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
// tPose3D constructors
//----------------------------------------------------------------------
tPose3D::tPose3D()
    : roll(0),
    pitch(0),
    yaw(0)
{}

tPose3D::tPose3D(double x, double y, double z)
    : position(x, y, z),
    roll(0),
    pitch(0),
    yaw(0)
{}

tPose3D::tPose3D(double x, double y, double z, tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
    : position(x, y, z),
    roll(roll),
    pitch(pitch),
    yaw(yaw)
{}

tPose3D::tPose3D(const tVec3d &position)
    : position(position),
    roll(0),
    pitch(0),
    yaw(0)
{}

tPose3D::tPose3D(const tVec3d &position, tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
    : position(position),
    roll(roll),
    pitch(pitch),
    yaw(yaw)
{}

tPose3D::tPose3D(const tMat4x4d &matrix, bool use_second_solution)
    : roll(0),
    pitch(0),
    yaw(0)
{
  this->Set(matrix, use_second_solution);
}

//----------------------------------------------------------------------
// tPose3D SetPosition
//----------------------------------------------------------------------
void tPose3D::SetPosition(double x, double y, double z)
{
  this->position.Set(x, y, z);
}

void tPose3D::SetPosition(const tVec3d &position)
{
  this->position = position;
}

//----------------------------------------------------------------------
// tPose3D SetOrientation
//----------------------------------------------------------------------
void tPose3D::SetOrientation(tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
{
  this->roll = roll;
  this->pitch = pitch;
  this->yaw = yaw;
}

void tPose3D::SetOrientation(const tMat3x3d &matrix, bool use_second_solution)
{
  assert(IsEqual(matrix.Determinant(), 1.0));
  if (!use_second_solution)
  {
    this->roll = std::atan2(matrix[2][1], matrix[2][2]);
  }
  else
  {
    this->roll = std::atan2(-matrix[2][1], -matrix[2][2]);
  }
  double sin_roll, cos_roll;
  this->roll.SinCos(sin_roll, cos_roll);
  this->pitch = std::atan2(-matrix[2][0], sin_roll * matrix[2][1] + cos_roll * matrix[2][2]);
  this->yaw = std::atan2(sin_roll * matrix[0][2] - cos_roll * matrix[0][1], cos_roll * matrix[1][1] - sin_roll * matrix[1][2]);
}

//----------------------------------------------------------------------
// tPose3D Set
//----------------------------------------------------------------------
void tPose3D::Set(const tVec3d &position, tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
{
  this->SetPosition(position);
  this->SetOrientation(roll, pitch, yaw);
}

void tPose3D::Set(const tVec3d &position)
{
  this->SetPosition(position);
}

void tPose3D::Set(double x, double y, double z, tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
{
  this->position.Set(x, y, z);
  this->SetOrientation(roll, pitch, yaw);
}

void tPose3D::Set(double x, double y, double z)
{
  this->position.Set(x, y, z);
}

void tPose3D::Set(const tMat4x4d &matrix, bool use_second_solution)
{
  assert(IsEqual(matrix[3][0], 0.0) && IsEqual(matrix[3][1], 0.0) && IsEqual(matrix[3][2], 0.0) && IsEqual(matrix[3][3], 1.0));
  this->position.Set(matrix[0][3], matrix[1][3], matrix[2][3]);
  this->SetOrientation(tMat3x3d(matrix[0][0], matrix[0][1], matrix[0][2],
                                matrix[1][0], matrix[1][1], matrix[1][2],
                                matrix[2][0], matrix[2][1], matrix[2][2]));
}

//----------------------------------------------------------------------
// tPose3D Reset
//----------------------------------------------------------------------
void tPose3D::Reset()
{
  this->position = tVec3d::Zero();
  this->roll = this->pitch = this->yaw = 0;
}

//----------------------------------------------------------------------
// tPose3D GetPose2D
//----------------------------------------------------------------------
const tPose2D tPose3D::GetPose2D() const
{
  return tPose2D(this->X(), this->Y(), this->Yaw());
}

//----------------------------------------------------------------------
// tPose3D operator +=
//----------------------------------------------------------------------
tPose3D &tPose3D::operator += (const tPose3D & other)
{
  this->position += other.position;
  this->roll += other.roll;
  this->pitch += other.pitch;
  this->yaw += other.yaw;
  return *this;
}

//----------------------------------------------------------------------
// tPose3D operator -=
//----------------------------------------------------------------------
tPose3D &tPose3D::operator -= (const tPose3D & other)
{
  this->position -= other.position;
  this->roll -= other.roll;
  this->pitch -= other.pitch;
  this->yaw -= other.yaw;
  return *this;
}

//----------------------------------------------------------------------
// tPose3D GetRotationMatrix
//----------------------------------------------------------------------
const tMat3x3d tPose3D::GetRotationMatrix() const
{
  return Get3DRotationMatrixFromRollPitchYaw<double>(this->roll, this->pitch, this->yaw);
}

void tPose3D::GetRotationMatrix(tMat3x3d &matrix) const
{
  matrix = this->GetRotationMatrix();
}

//----------------------------------------------------------------------
// tPose3D GetTransformationMatrix
//----------------------------------------------------------------------
const tMat4x4d tPose3D::GetTransformationMatrix() const
{
  tMat3x3d rotation = this->GetRotationMatrix();
  return tMat4x4d(rotation[0][0], rotation[0][1], rotation[0][2], this->X(),
                  rotation[1][0], rotation[1][1], rotation[1][2], this->Y(),
                  rotation[2][0], rotation[2][1], rotation[2][2], this->Z(),
                  0, 0, 0, 1);
}

void tPose3D::GetTransformationMatrix(tMat4x4d &matrix) const
{
  matrix = this->GetTransformationMatrix();
}

//----------------------------------------------------------------------
// tPose3D GetTransformationMatrixZYXT
//----------------------------------------------------------------------
const tMat4x4d tPose3D::GetTransformationMatrixZYXT() const
{
  return Get4DTransformationMatrixZYXT(this->X(), this->Y(), this->Z(), this->Roll(), this->Pitch(), this->Yaw());
}

void tPose3D::GetTransformationMatrixZYXT(tMat4x4d &matrix) const
{
  matrix = this->GetTransformationMatrixZYXT();
}

//----------------------------------------------------------------------
// tPose2D GetPoseInParentFrame
//----------------------------------------------------------------------
const tPose3D tPose3D::GetPoseInParentFrame(const tPose3D &reference) const
{
  return tPose3D(reference.GetTransformationMatrix() * this->GetTransformationMatrix());
}

//----------------------------------------------------------------------
// tPose2D GetPoseInLocalFrame
//----------------------------------------------------------------------
const tPose3D tPose3D::GetPoseInLocalFrame(const tPose3D &reference) const
{
  return tPose3D(reference.GetTransformationMatrix().Inverse() * this->GetTransformationMatrix());
}

//----------------------------------------------------------------------
// tPose3D ToGlobal
//----------------------------------------------------------------------
//const tPose3D tPose3D::ToGlobal(const tPose3D &reference) const
//{
//  return this->GetPoseInParentFrame(reference);
//}

//----------------------------------------------------------------------
// tPose3D ToLocal
//----------------------------------------------------------------------
//const tPose3D tPose3D::ToLocal(const tPose3D &reference) const
//{
//  return this->GetPoseInLocalFrame(reference);
//}

//----------------------------------------------------------------------
// tPose3D Translate
//----------------------------------------------------------------------
void tPose3D::Translate(const tVec3d &translation)
{
  this->position += translation;
}

//----------------------------------------------------------------------
// tPose3D Rotate
//----------------------------------------------------------------------
void tPose3D::Rotate(tAngleRad roll, tAngleRad pitch, tAngleRad yaw)
{
  this->roll += roll;
  this->pitch += pitch;
  this->yaw += yaw;
}

void tPose3D::Rotate(const tMat3x3d &matrix)
{
  assert(matrix.Determinant() == 1);
  this->SetOrientation(matrix * this->GetRotationMatrix());
}

//----------------------------------------------------------------------
// tPose3D Scale
//----------------------------------------------------------------------
void tPose3D::Scale(double factor)
{
  this->position *= factor;
}

//----------------------------------------------------------------------
// tPose3D ApplyRelativePoseTransformation
//----------------------------------------------------------------------
void tPose3D::ApplyRelativePoseTransformation(const tPose3D &relative_transformation)
{
  this->Set(this->GetTransformationMatrix() * relative_transformation.GetTransformationMatrix());
}

//----------------------------------------------------------------------
// tPose3D ApplyPose
//----------------------------------------------------------------------
void tPose3D::ApplyPose(const tPose3D &relative_transformation)
{
  this->ApplyRelativePoseTransformation(relative_transformation);
}

//----------------------------------------------------------------------
// Unary Minus for tPose3D objects
//----------------------------------------------------------------------
const tPose3D operator - (const tPose3D &pose)
{
  return tPose3D(-pose.GetPosition(), -pose.Roll(), -pose.Pitch(), -pose.Yaw());
}

//----------------------------------------------------------------------
// Addition of tPose3D objects
//----------------------------------------------------------------------
const tPose3D rrlib::math::operator + (const tPose3D &left, const tPose3D &right)
{
  return tPose3D(left.GetPosition() + right.GetPosition(), left.Roll() + right.Roll(), left.Pitch() + right.Pitch(), left.Yaw() + right.Yaw());
}

//----------------------------------------------------------------------
// Subtraction of tPose3D objects
//----------------------------------------------------------------------
const tPose3D rrlib::math::operator - (const tPose3D &left, const tPose3D &right)
{
  return tPose3D(left.GetPosition() - right.GetPosition(), left.Roll() - right.Roll(), left.Pitch() - right.Pitch(), left.Yaw() - right.Yaw());
}

//----------------------------------------------------------------------
// Equality of tPose3D objects
//----------------------------------------------------------------------
const bool rrlib::math::operator == (const tPose3D &left, const tPose3D &right)
{
  return left.GetPosition() == right.GetPosition() && left.Roll() == right.Roll() && left.Pitch() == right.Pitch() && left.Yaw() == right.Yaw();
}

//----------------------------------------------------------------------
// Inequality of tPose3D objects
//----------------------------------------------------------------------
const bool rrlib::math::operator != (const tPose3D &left, const tPose3D &right)
{
  return !(left == right);
}

//----------------------------------------------------------------------
// Ordering tPose3D objects
//----------------------------------------------------------------------
const bool rrlib::math::operator < (const tPose3D &left, const tPose3D &right)
{
  return left.GetPosition() < right.GetPosition() || left.Roll() < right.Roll() || left.Pitch() < right.Pitch() || left.Yaw() < right.Yaw();
}

//----------------------------------------------------------------------
// Composition of tPose3D objects
//----------------------------------------------------------------------
//const tPose3D rrlib::math::Compound(const tPose3D &base, const tPose3D &diff)
//{
//  return diff.GetPoseInParentFrame(base);
//}

//----------------------------------------------------------------------
// Decomposition of tPose3D objects
//----------------------------------------------------------------------
//const tPose3D rrlib::math::InverseCompound(const tPose3D &target, const tPose3D &base)
//{
//  return target.GetPoseInLocalFrame(base);
//}

//----------------------------------------------------------------------
// Streaming
//----------------------------------------------------------------------
std::ostream &rrlib::math::operator << (std::ostream &stream, const tPose3D &pose)
{
  return stream << "(" << pose.X() << ", " << pose.Y() << ", " << pose.Z() << ", " << pose.Roll() << ", " << pose.Pitch() << ", " << pose.Yaw() << ")";
}

std::istream &rrlib::math::operator >> (std::istream &stream, tPose3D &pose)
{
  char temp;
  stream >> temp;
  if (temp == '(')
  {
    stream >> pose.X() >> temp >> pose.Y() >> temp >> pose.Z() >> temp >> pose.Roll() >> temp >> pose.Pitch() >> temp >> pose.Yaw() >> temp;
  }
  else
  {
    stream.putback(temp);
    stream >> pose.X() >> pose.Y() >> pose.Z() >> pose.Roll() >> pose.Pitch() >> pose.Yaw();
  }
  return stream;
}
