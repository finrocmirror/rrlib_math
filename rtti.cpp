//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
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
/*!\file    rrlib/math/rtti.cpp
 *
 * \author  Max Reichardt
 *
 * \date    2011-10-30
 *
 */
//----------------------------------------------------------------------

#ifdef _LIB_RRLIB_RTTI_PRESENT_

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/rtti/rtti.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/math/tPose2D.h"
#include "rrlib/math/tPose3D.h"
#include "rrlib/math/tVector.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::math;
using namespace rrlib::rtti;

//----------------------------------------------------------------------
// Type initializers
//----------------------------------------------------------------------
static tDataType<tPose2D> init_type_pose_2d;
static tDataType<tPose3D> init_type_pose_3d;
static tDataType<tVec2d> init_type_vector_2d("Vector2d");
static tDataType<tVec3d> init_type_vector_3d("Vector3d");
static tDataType<tVec6d> init_type_vector_6d("Vector6d");
static tDataType<tVec2i> init_type_vector_2i("Vector2i");
static tDataType<tVec3i> init_type_vector_3i("Vector3i");
static tDataType<tVec6i> init_type_vector_6i("Vector6i");
static tDataType<tAngleRad> init_type_angle("Angle");

#endif

