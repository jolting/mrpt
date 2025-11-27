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

/**
 * Test program for Rust math library FFI integration
 */

#include <iostream>
#include <cmath>
#include <cassert>

#include "mrpt_math_rust_adapter.h"

using namespace mrpt::math;
using namespace mrpt::math::rust;

void test_point2d()
{
    std::cout << "Testing Point2D..." << std::endl;
    
    TPoint2D p1{3.0, 4.0};
    TPoint2D p2{0.0, 0.0};
    
    double dist = point2d_distance_rust(p1, p2);
    std::cout << "  Distance: " << dist << " (expected 5.0)" << std::endl;
    assert(std::abs(dist - 5.0) < 1e-10);
    
    std::cout << "  ✓ Point2D tests passed" << std::endl;
}

void test_point3d()
{
    std::cout << "Testing Point3D..." << std::endl;
    
    TPoint3D p1{1.0, 0.0, 0.0};
    TPoint3D p2{0.0, 1.0, 0.0};
    
    // Test distance
    double dist = point3d_distance_rust(p1, p2);
    std::cout << "  Distance: " << dist << " (expected ~1.414)" << std::endl;
    assert(std::abs(dist - std::sqrt(2.0)) < 1e-10);
    
    // Test cross product
    TPoint3D cross = point3d_cross_rust(p1, p2);
    std::cout << "  Cross product: (" << cross.x << ", " << cross.y << ", " << cross.z << ")" << std::endl;
    assert(std::abs(cross.x - 0.0) < 1e-10);
    assert(std::abs(cross.y - 0.0) < 1e-10);
    assert(std::abs(cross.z - 1.0) < 1e-10);
    
    std::cout << "  ✓ Point3D tests passed" << std::endl;
}

void test_pose2d()
{
    std::cout << "Testing Pose2D..." << std::endl;
    
    TPose2D p1{1.0, 0.0, 0.0};
    TPose2D p2{1.0, 0.0, 0.0};
    
    TPose2D composed = pose2d_compose_rust(p1, p2);
    std::cout << "  Composed pose: (" << composed.x << ", " << composed.y << ", " << composed.phi << ")" << std::endl;
    assert(std::abs(composed.x - 2.0) < 1e-10);
    assert(std::abs(composed.y - 0.0) < 1e-10);
    
    std::cout << "  ✓ Pose2D tests passed" << std::endl;
}

void test_line2d()
{
    std::cout << "Testing Line2D..." << std::endl;
    
    TPoint2D p1{0.0, 0.0};
    TPoint2D p2{1.0, 1.0};
    
    TLine2D line = line2d_from_two_points_rust(p1, p2);
    std::cout << "  Line coefficients: [" << line.coefs[0] << ", " << line.coefs[1] << ", " << line.coefs[2] << "]" << std::endl;
    
    // The line should pass through both points
    auto r_line = toRust(line);
    auto r_p1 = toRust(p1);
    auto r_p2 = toRust(p2);
    
    assert(mrpt_math_line2d_contains(&r_line, &r_p1));
    assert(mrpt_math_line2d_contains(&r_line, &r_p2));
    
    std::cout << "  ✓ Line2D tests passed" << std::endl;
}

void test_plane()
{
    std::cout << "Testing Plane..." << std::endl;
    
    // XY plane (z = 0)
    TPlane plane{0.0, 0.0, 1.0, 0.0};
    TPoint3D point{5.0, 5.0, 3.0};
    
    double dist = plane_distance_rust(plane, point);
    std::cout << "  Distance to plane: " << dist << " (expected 3.0)" << std::endl;
    assert(std::abs(dist - 3.0) < 1e-10);
    
    std::cout << "  ✓ Plane tests passed" << std::endl;
}

void test_epsilon()
{
    std::cout << "Testing Epsilon..." << std::endl;
    
    double eps = mrpt_math_get_epsilon();
    std::cout << "  Current epsilon: " << eps << std::endl;
    
    mrpt_math_set_epsilon(1e-6);
    double new_eps = mrpt_math_get_epsilon();
    std::cout << "  New epsilon: " << new_eps << std::endl;
    assert(std::abs(new_eps - 1e-6) < 1e-15);
    
    // Restore default
    mrpt_math_set_epsilon(1e-9);
    
    std::cout << "  ✓ Epsilon tests passed" << std::endl;
}

int main()
{
    std::cout << "========================================" << std::endl;
    std::cout << "MRPT Rust Math Library FFI Test" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    try
    {
        test_point2d();
        std::cout << std::endl;
        
        test_point3d();
        std::cout << std::endl;
        
        test_pose2d();
        std::cout << std::endl;
        
        test_line2d();
        std::cout << std::endl;
        
        test_plane();
        std::cout << std::endl;
        
        test_epsilon();
        std::cout << std::endl;
        
        std::cout << "========================================" << std::endl;
        std::cout << "All tests passed! ✓" << std::endl;
        std::cout << "========================================" << std::endl;
        
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
