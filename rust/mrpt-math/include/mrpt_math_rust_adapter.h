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

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPlane.h>

#include "mrpt_math_rust.h"

namespace mrpt::math::rust
{

/** @name Conversion functions between C++ and Rust types
    @{ */

// Point2D conversions
inline TPoint2D_Rust toRust(const TPoint2D& p)
{
    return TPoint2D_Rust{p.x, p.y};
}

inline TPoint2D fromRust(const TPoint2D_Rust& p)
{
    return TPoint2D{p.x, p.y};
}

// Point3D conversions
inline TPoint3D_Rust toRust(const TPoint3D& p)
{
    return TPoint3D_Rust{p.x, p.y, p.z};
}

inline TPoint3D fromRust(const TPoint3D_Rust& p)
{
    return TPoint3D{p.x, p.y, p.z};
}

// Pose2D conversions
inline TPose2D_Rust toRust(const TPose2D& p)
{
    return TPose2D_Rust{p.x, p.y, p.phi};
}

inline TPose2D fromRust(const TPose2D_Rust& p)
{
    return TPose2D{p.x, p.y, p.phi};
}

// Pose3D conversions
inline TPose3D_Rust toRust(const TPose3D& p)
{
    return TPose3D_Rust{p.x, p.y, p.z, p.yaw, p.pitch, p.roll};
}

inline TPose3D fromRust(const TPose3D_Rust& p)
{
    return TPose3D{p.x, p.y, p.z, p.yaw, p.pitch, p.roll};
}

// Line2D conversions
inline TLine2D_Rust toRust(const TLine2D& line)
{
    TLine2D_Rust result;
    result.coefs[0] = line.coefs[0];
    result.coefs[1] = line.coefs[1];
    result.coefs[2] = line.coefs[2];
    return result;
}

inline TLine2D fromRust(const TLine2D_Rust& line)
{
    return TLine2D{line.coefs[0], line.coefs[1], line.coefs[2]};
}

// Line3D conversions
inline TLine3D_Rust toRust(const TLine3D& line)
{
    TLine3D_Rust result;
    result.p_base = toRust(line.pBase);
    result.director = toRust(line.director);
    return result;
}

inline TLine3D fromRust(const TLine3D_Rust& line)
{
    TLine3D result;
    result.pBase = fromRust(line.p_base);
    result.director = fromRust(line.director);
    return result;
}

// Plane conversions
inline TPlane_Rust toRust(const TPlane& plane)
{
    TPlane_Rust result;
    result.coefs[0] = plane.coefs[0];
    result.coefs[1] = plane.coefs[1];
    result.coefs[2] = plane.coefs[2];
    result.coefs[3] = plane.coefs[3];
    return result;
}

inline TPlane fromRust(const TPlane_Rust& plane)
{
    return TPlane{plane.coefs[0], plane.coefs[1], plane.coefs[2], plane.coefs[3]};
}

/** @} */

/** @name Rust-powered implementations
    @{ */

// Example wrapper functions that call Rust implementations
inline double point2d_distance_rust(const TPoint2D& p1, const TPoint2D& p2)
{
    auto r1 = toRust(p1);
    auto r2 = toRust(p2);
    return mrpt_math_point2d_distance(&r1, &r2);
}

inline double point3d_distance_rust(const TPoint3D& p1, const TPoint3D& p2)
{
    auto r1 = toRust(p1);
    auto r2 = toRust(p2);
    return mrpt_math_point3d_distance(&r1, &r2);
}

inline TPoint3D point3d_cross_rust(const TPoint3D& p1, const TPoint3D& p2)
{
    auto r1 = toRust(p1);
    auto r2 = toRust(p2);
    TPoint3D_Rust result;
    mrpt_math_point3d_cross(&r1, &r2, &result);
    return fromRust(result);
}

inline TPose2D pose2d_compose_rust(const TPose2D& p1, const TPose2D& p2)
{
    auto r1 = toRust(p1);
    auto r2 = toRust(p2);
    TPose2D_Rust result;
    mrpt_math_pose2d_compose(&r1, &r2, &result);
    return fromRust(result);
}

inline TLine2D line2d_from_two_points_rust(const TPoint2D& p1, const TPoint2D& p2)
{
    auto r1 = toRust(p1);
    auto r2 = toRust(p2);
    TLine2D_Rust result;
    mrpt_math_line2d_from_two_points(&r1, &r2, &result);
    return fromRust(result);
}

inline double plane_distance_rust(const TPlane& plane, const TPoint3D& point)
{
    auto rPlane = toRust(plane);
    auto rPoint = toRust(point);
    return mrpt_math_plane_distance(&rPlane, &rPoint);
}

/** @} */

}  // namespace mrpt::math::rust
