/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

// ============================================================================
// Type definitions (must match Rust #[repr(C)] structs)
// ============================================================================

typedef struct {
    double x;
    double y;
} TPoint2D_Rust;

typedef struct {
    double x;
    double y;
    double z;
} TPoint3D_Rust;

typedef struct {
    double x;
    double y;
    double phi;
} TPose2D_Rust;

typedef struct {
    double x;
    double y;
    double z;
    double yaw;
    double pitch;
    double roll;
} TPose3D_Rust;

typedef struct {
    double coefs[3];  // [A, B, C] for Ax + By + C = 0
} TLine2D_Rust;

typedef struct {
    TPoint3D_Rust p_base;
    TPoint3D_Rust director;
} TLine3D_Rust;

typedef struct {
    double coefs[4];  // [A, B, C, D] for Ax + By + Cz + D = 0
} TPlane_Rust;

// ============================================================================
// Point2D Functions
// ============================================================================

TPoint2D_Rust mrpt_math_point2d_new(double x, double y);
double mrpt_math_point2d_distance(const TPoint2D_Rust* p1, const TPoint2D_Rust* p2);
double mrpt_math_point2d_norm(const TPoint2D_Rust* p);
double mrpt_math_point2d_dot(const TPoint2D_Rust* p1, const TPoint2D_Rust* p2);

// ============================================================================
// Point3D Functions
// ============================================================================

TPoint3D_Rust mrpt_math_point3d_new(double x, double y, double z);
double mrpt_math_point3d_distance(const TPoint3D_Rust* p1, const TPoint3D_Rust* p2);
double mrpt_math_point3d_norm(const TPoint3D_Rust* p);
double mrpt_math_point3d_dot(const TPoint3D_Rust* p1, const TPoint3D_Rust* p2);
void mrpt_math_point3d_cross(const TPoint3D_Rust* p1, const TPoint3D_Rust* p2, TPoint3D_Rust* out);

// ============================================================================
// Pose2D Functions
// ============================================================================

TPose2D_Rust mrpt_math_pose2d_new(double x, double y, double phi);
void mrpt_math_pose2d_compose(const TPose2D_Rust* this_pose, const TPose2D_Rust* other, TPose2D_Rust* out);
void mrpt_math_pose2d_inverse(const TPose2D_Rust* pose, TPose2D_Rust* out);
double mrpt_math_pose2d_distance(const TPose2D_Rust* p1, const TPose2D_Rust* p2);

// ============================================================================
// Pose3D Functions
// ============================================================================

TPose3D_Rust mrpt_math_pose3d_new(double x, double y, double z, double yaw, double pitch, double roll);
double mrpt_math_pose3d_distance(const TPose3D_Rust* p1, const TPose3D_Rust* p2);

// ============================================================================
// Line2D Functions
// ============================================================================

TLine2D_Rust mrpt_math_line2d_new(double a, double b, double c);
void mrpt_math_line2d_from_two_points(const TPoint2D_Rust* p1, const TPoint2D_Rust* p2, TLine2D_Rust* out);
double mrpt_math_line2d_distance(const TLine2D_Rust* line, const TPoint2D_Rust* point);
double mrpt_math_line2d_signed_distance(const TLine2D_Rust* line, const TPoint2D_Rust* point);
bool mrpt_math_line2d_contains(const TLine2D_Rust* line, const TPoint2D_Rust* point);
void mrpt_math_line2d_unitarize(TLine2D_Rust* line);

// ============================================================================
// Line3D Functions
// ============================================================================

TLine3D_Rust mrpt_math_line3d_new(const TPoint3D_Rust* base, const TPoint3D_Rust* director);
void mrpt_math_line3d_from_two_points(const TPoint3D_Rust* p1, const TPoint3D_Rust* p2, TLine3D_Rust* out);
double mrpt_math_line3d_distance(const TLine3D_Rust* line, const TPoint3D_Rust* point);
bool mrpt_math_line3d_contains(const TLine3D_Rust* line, const TPoint3D_Rust* point);
void mrpt_math_line3d_closest_point(const TLine3D_Rust* line, const TPoint3D_Rust* point, TPoint3D_Rust* out);
void mrpt_math_line3d_unitarize(TLine3D_Rust* line);

// ============================================================================
// Plane Functions
// ============================================================================

TPlane_Rust mrpt_math_plane_new(double a, double b, double c, double d);
void mrpt_math_plane_from_three_points(const TPoint3D_Rust* p1, const TPoint3D_Rust* p2, const TPoint3D_Rust* p3, TPlane_Rust* out);
void mrpt_math_plane_from_point_and_normal(const TPoint3D_Rust* point, const TPoint3D_Rust* normal, TPlane_Rust* out);
double mrpt_math_plane_distance(const TPlane_Rust* plane, const TPoint3D_Rust* point);
double mrpt_math_plane_signed_distance(const TPlane_Rust* plane, const TPoint3D_Rust* point);
bool mrpt_math_plane_contains(const TPlane_Rust* plane, const TPoint3D_Rust* point);
bool mrpt_math_plane_contains_line(const TPlane_Rust* plane, const TLine3D_Rust* line);
void mrpt_math_plane_normal_vector(const TPlane_Rust* plane, TPoint3D_Rust* out);
void mrpt_math_plane_unitarize(TPlane_Rust* plane);

// ============================================================================
// Epsilon Functions
// ============================================================================

double mrpt_math_get_epsilon(void);
void mrpt_math_set_epsilon(double epsilon);

#ifdef __cplusplus
}
#endif
