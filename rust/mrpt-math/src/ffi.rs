//                    _
//                   | |    Mobile Robot Programming Toolkit (MRPT)
// _ __ ___  _ __ _ __ | |_
//| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
//| | | | | | |  | |_) | |_
//|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
//               | |
//               |_|
//
// Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
// See: https://www.mrpt.org/Authors - All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

//! FFI (Foreign Function Interface) bridge for C++ interoperability
//!
//! This module provides C-compatible interfaces to allow the C++ codebase
//! to call into the Rust math library implementation.

use crate::point2d::TPoint2D;
use crate::point3d::TPoint3D;
use crate::pose::{TPose2D, TPose3D};
use crate::line::{TLine2D, TLine3D};
use crate::plane::TPlane;

// ============================================================================
// Point2D FFI
// ============================================================================

/// Creates a 2D point
#[no_mangle]
pub extern "C" fn mrpt_math_point2d_new(x: f64, y: f64) -> TPoint2D {
    TPoint2D::new(x, y)
}

/// Computes distance between two 2D points
#[no_mangle]
pub extern "C" fn mrpt_math_point2d_distance(p1: &TPoint2D, p2: &TPoint2D) -> f64 {
    p1.distance_to(p2)
}

/// Computes norm (magnitude) of a 2D point as a vector
#[no_mangle]
pub extern "C" fn mrpt_math_point2d_norm(p: &TPoint2D) -> f64 {
    p.norm()
}

/// Computes dot product of two 2D points
#[no_mangle]
pub extern "C" fn mrpt_math_point2d_dot(p1: &TPoint2D, p2: &TPoint2D) -> f64 {
    p1.dot(p2)
}

// ============================================================================
// Point3D FFI
// ============================================================================

/// Creates a 3D point
#[no_mangle]
pub extern "C" fn mrpt_math_point3d_new(x: f64, y: f64, z: f64) -> TPoint3D {
    TPoint3D::new(x, y, z)
}

/// Computes distance between two 3D points
#[no_mangle]
pub extern "C" fn mrpt_math_point3d_distance(p1: &TPoint3D, p2: &TPoint3D) -> f64 {
    p1.distance_to(p2)
}

/// Computes norm (magnitude) of a 3D point as a vector
#[no_mangle]
pub extern "C" fn mrpt_math_point3d_norm(p: &TPoint3D) -> f64 {
    p.norm()
}

/// Computes dot product of two 3D points
#[no_mangle]
pub extern "C" fn mrpt_math_point3d_dot(p1: &TPoint3D, p2: &TPoint3D) -> f64 {
    p1.dot(p2)
}

/// Computes cross product of two 3D points
#[no_mangle]
pub extern "C" fn mrpt_math_point3d_cross(p1: &TPoint3D, p2: &TPoint3D, out: &mut TPoint3D) {
    *out = p1.cross(p2);
}

// ============================================================================
// Pose2D FFI
// ============================================================================

/// Creates a 2D pose
#[no_mangle]
pub extern "C" fn mrpt_math_pose2d_new(x: f64, y: f64, phi: f64) -> TPose2D {
    TPose2D::new(x, y, phi)
}

/// Composes two 2D poses (this ⊕ other)
#[no_mangle]
pub extern "C" fn mrpt_math_pose2d_compose(this: &TPose2D, other: &TPose2D, out: &mut TPose2D) {
    *out = this.compose(other);
}

/// Computes the inverse of a 2D pose
#[no_mangle]
pub extern "C" fn mrpt_math_pose2d_inverse(pose: &TPose2D, out: &mut TPose2D) {
    *out = pose.inverse();
}

/// Computes distance between two 2D poses
#[no_mangle]
pub extern "C" fn mrpt_math_pose2d_distance(p1: &TPose2D, p2: &TPose2D) -> f64 {
    p1.distance_to(p2)
}

// ============================================================================
// Pose3D FFI
// ============================================================================

/// Creates a 3D pose
#[no_mangle]
pub extern "C" fn mrpt_math_pose3d_new(
    x: f64,
    y: f64,
    z: f64,
    yaw: f64,
    pitch: f64,
    roll: f64,
) -> TPose3D {
    TPose3D::new(x, y, z, yaw, pitch, roll)
}

/// Computes distance between two 3D poses
#[no_mangle]
pub extern "C" fn mrpt_math_pose3d_distance(p1: &TPose3D, p2: &TPose3D) -> f64 {
    p1.distance_to(p2)
}

// ============================================================================
// Line2D FFI
// ============================================================================

/// Creates a 2D line from coefficients A, B, C (Ax + By + C = 0)
#[no_mangle]
pub extern "C" fn mrpt_math_line2d_new(a: f64, b: f64, c: f64) -> TLine2D {
    TLine2D::new(a, b, c)
}

/// Creates a 2D line from two points
#[no_mangle]
pub extern "C" fn mrpt_math_line2d_from_two_points(p1: &TPoint2D, p2: &TPoint2D, out: &mut TLine2D) {
    *out = TLine2D::from_two_points(p1, p2);
}

/// Computes distance from a point to a 2D line
#[no_mangle]
pub extern "C" fn mrpt_math_line2d_distance(line: &TLine2D, point: &TPoint2D) -> f64 {
    line.distance(point)
}

/// Computes signed distance from a point to a 2D line
#[no_mangle]
pub extern "C" fn mrpt_math_line2d_signed_distance(line: &TLine2D, point: &TPoint2D) -> f64 {
    line.signed_distance(point)
}

/// Checks if a point is on a 2D line
#[no_mangle]
pub extern "C" fn mrpt_math_line2d_contains(line: &TLine2D, point: &TPoint2D) -> bool {
    line.contains(point)
}

/// Unitarizes a 2D line (normalizes coefficients)
#[no_mangle]
pub extern "C" fn mrpt_math_line2d_unitarize(line: &mut TLine2D) {
    line.unitarize();
}

// ============================================================================
// Line3D FFI
// ============================================================================

/// Creates a 3D line from base point and director vector
#[no_mangle]
pub extern "C" fn mrpt_math_line3d_new(base: &TPoint3D, director: &TPoint3D) -> TLine3D {
    TLine3D::new(*base, *director)
}

/// Creates a 3D line from two points
#[no_mangle]
pub extern "C" fn mrpt_math_line3d_from_two_points(p1: &TPoint3D, p2: &TPoint3D, out: &mut TLine3D) {
    *out = TLine3D::from_two_points(p1, p2);
}

/// Computes distance from a point to a 3D line
#[no_mangle]
pub extern "C" fn mrpt_math_line3d_distance(line: &TLine3D, point: &TPoint3D) -> f64 {
    line.distance(point)
}

/// Checks if a point is on a 3D line
#[no_mangle]
pub extern "C" fn mrpt_math_line3d_contains(line: &TLine3D, point: &TPoint3D) -> bool {
    line.contains(point)
}

/// Finds the closest point on a 3D line to a given point
#[no_mangle]
pub extern "C" fn mrpt_math_line3d_closest_point(line: &TLine3D, point: &TPoint3D, out: &mut TPoint3D) {
    *out = line.closest_point_to(point);
}

/// Unitarizes a 3D line (normalizes director vector)
#[no_mangle]
pub extern "C" fn mrpt_math_line3d_unitarize(line: &mut TLine3D) {
    line.unitarize();
}

// ============================================================================
// Plane FFI
// ============================================================================

/// Creates a plane from coefficients A, B, C, D (Ax + By + Cz + D = 0)
#[no_mangle]
pub extern "C" fn mrpt_math_plane_new(a: f64, b: f64, c: f64, d: f64) -> TPlane {
    TPlane::new(a, b, c, d)
}

/// Creates a plane from three points
#[no_mangle]
pub extern "C" fn mrpt_math_plane_from_three_points(
    p1: &TPoint3D,
    p2: &TPoint3D,
    p3: &TPoint3D,
    out: &mut TPlane,
) {
    *out = TPlane::from_three_points(p1, p2, p3);
}

/// Creates a plane from a point and normal vector
#[no_mangle]
pub extern "C" fn mrpt_math_plane_from_point_and_normal(
    point: &TPoint3D,
    normal: &TPoint3D,
    out: &mut TPlane,
) {
    *out = TPlane::from_point_and_normal(point, normal);
}

/// Computes distance from a point to a plane
#[no_mangle]
pub extern "C" fn mrpt_math_plane_distance(plane: &TPlane, point: &TPoint3D) -> f64 {
    plane.distance(point)
}

/// Computes signed distance from a point to a plane
#[no_mangle]
pub extern "C" fn mrpt_math_plane_signed_distance(plane: &TPlane, point: &TPoint3D) -> f64 {
    plane.signed_distance(point)
}

/// Checks if a point is on a plane
#[no_mangle]
pub extern "C" fn mrpt_math_plane_contains(plane: &TPlane, point: &TPoint3D) -> bool {
    plane.contains(point)
}

/// Checks if a line is contained in a plane
#[no_mangle]
pub extern "C" fn mrpt_math_plane_contains_line(plane: &TPlane, line: &TLine3D) -> bool {
    plane.contains_line(line)
}

/// Gets the normal vector of a plane
#[no_mangle]
pub extern "C" fn mrpt_math_plane_normal_vector(plane: &TPlane, out: &mut TPoint3D) {
    *out = plane.normal_vector();
}

/// Unitarizes a plane (normalizes coefficients)
#[no_mangle]
pub extern "C" fn mrpt_math_plane_unitarize(plane: &mut TPlane) {
    plane.unitarize();
}

// ============================================================================
// Epsilon FFI
// ============================================================================

/// Gets the global geometric epsilon
#[no_mangle]
pub extern "C" fn mrpt_math_get_epsilon() -> f64 {
    crate::epsilon::get_epsilon()
}

/// Sets the global geometric epsilon
#[no_mangle]
pub extern "C" fn mrpt_math_set_epsilon(epsilon: f64) {
    crate::epsilon::set_epsilon(epsilon);
}
