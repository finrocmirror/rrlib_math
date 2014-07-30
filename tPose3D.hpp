//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) AG Robotersysteme TU Kaiserslautern
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
/*!\file    rrlib/math/tPose3D.hpp
 *
 * \author  Patrick Fleischmann
 *
 * \date    2014-03-19
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

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
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// tPose3D TransformCoordinateSystem
//----------------------------------------------------------------------
template<typename TIterator>
void tPose3D::TransformCoordinateSystem(TIterator points_begin, TIterator points_end, bool in_local_frame) const
{
  // fixed transformation matrix
  rrlib::math::tMat4x4d reference_transformation_matrix = this->GetTransformationMatrix();
  if (in_local_frame)
  {
    reference_transformation_matrix.Invert();
  }
  // transformation matrix for points of the cloud, rotation matrix is always zero, translation vector is different for each point
  rrlib::math::tMat4x4d point_transformation_matrix = rrlib::math::tPose3D::Zero().GetTransformationMatrix();
  //
  rrlib::math::tMat4x4d result_matrix;

  for (auto it = points_begin; it < points_end; ++it)
  {
    // set translation component of the transformation matrix
    point_transformation_matrix[0][3] = (*it).X();
    point_transformation_matrix[1][3] = (*it).Y();
    point_transformation_matrix[2][3] = (*it).Z();

    result_matrix = reference_transformation_matrix * point_transformation_matrix;

    // copy position from result matrix
    (*it).Set(result_matrix[0][3], result_matrix[1][3], result_matrix[2][3]);
  }
}

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
