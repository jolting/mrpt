// Mobile Robot Programming Toolkit (MRPT)
// https://www.mrpt.org/
//
// Copyright (c) 2005-2024, Individual contributors, see AUTHORS file
// See: https://www.mrpt.org/Authors - All rights reserved.
// Released under BSD License. See: https://www.mrpt.org/License

//! Angle wrapping and distance utilities
//!
//! This module provides functions for wrapping angles to specific ranges
//! and computing angular distances.

use std::f64::consts::PI;

/// Wrap angle to [0, 2π) range
pub fn wrap_to_2pi(mut a: f64) -> f64 {
    let was_neg = a < 0.0;
    a = a.rem_euclid(2.0 * PI);
    if was_neg && a == 0.0 {
        a = 2.0 * PI;
    }
    a
}

/// Wrap angle to (-π, π] range
pub fn wrap_to_pi(a: f64) -> f64 {
    wrap_to_2pi(a + PI) - PI
}

/// Compute shortest angular distance from one angle to another
///
/// Returns the shortest angular increment (or distance) between two planar
/// orientations, constrained to [-π, π].
///
/// # Examples
/// ```
/// use mrpt_core::wrap2pi::ang_distance;
/// use std::f64::consts::PI;
///
/// assert!((ang_distance(0.0, PI) - PI).abs() < 1e-10);
/// assert!((ang_distance(PI, 0.0) + PI).abs() < 1e-10);
/// ```
pub fn ang_distance(from: f64, to: f64) -> f64 {
    let from_wrapped = wrap_to_pi(from);
    let to_wrapped = wrap_to_pi(to);
    let mut d = to_wrapped - from_wrapped;
    
    if d > PI {
        d -= 2.0 * PI;
    } else if d < -PI {
        d += 2.0 * PI;
    }
    
    d
}

/// Wrap angle to [0, 2π) range (f32 version)
pub fn wrap_to_2pi_f32(mut a: f32) -> f32 {
    let was_neg = a < 0.0;
    a = a.rem_euclid(2.0 * std::f32::consts::PI);
    if was_neg && a == 0.0 {
        a = 2.0 * std::f32::consts::PI;
    }
    a
}

/// Wrap angle to (-π, π] range (f32 version)
pub fn wrap_to_pi_f32(a: f32) -> f32 {
    wrap_to_2pi_f32(a + std::f32::consts::PI) - std::f32::consts::PI
}

/// Compute shortest angular distance (f32 version)
pub fn ang_distance_f32(from: f32, to: f32) -> f32 {
    let from_wrapped = wrap_to_pi_f32(from);
    let to_wrapped = wrap_to_pi_f32(to);
    let mut d = to_wrapped - from_wrapped;
    
    if d > std::f32::consts::PI {
        d -= 2.0 * std::f32::consts::PI;
    } else if d < -std::f32::consts::PI {
        d += 2.0 * std::f32::consts::PI;
    }
    
    d
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wrap_to_2pi() {
        assert!((wrap_to_2pi(0.0) - 0.0).abs() < 1e-10);
        assert!((wrap_to_2pi(PI) - PI).abs() < 1e-10);
        assert!((wrap_to_2pi(2.0 * PI) - 0.0).abs() < 1e-10);
        assert!((wrap_to_2pi(-PI) - PI).abs() < 1e-10);
        assert!((wrap_to_2pi(3.0 * PI) - PI).abs() < 1e-10);
    }

    #[test]
    fn test_wrap_to_pi() {
        assert!((wrap_to_pi(0.0) - 0.0).abs() < 1e-10);
        assert!((wrap_to_pi(PI) + PI).abs() < 1e-10); // PI wraps to -PI
        assert!((wrap_to_pi(-PI) + PI).abs() < 1e-10); // -PI wraps to -PI
        assert!((wrap_to_pi(2.0 * PI) - 0.0).abs() < 1e-10);
        assert!((wrap_to_pi(0.5) - 0.5).abs() < 1e-10);
        assert!((wrap_to_pi(-0.5) + 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_ang_distance() {
        assert!((ang_distance(0.0, 1.0) - 1.0).abs() < 1e-10);
        assert!((ang_distance(1.0, 1.0) - 0.0).abs() < 1e-10);
        assert!((ang_distance(1.0, 0.0) + 1.0).abs() < 1e-10);
        
        assert!((ang_distance(-(PI - 0.1), (PI - 0.1)) + 0.2).abs() < 1e-6);
        assert!((ang_distance((PI - 0.1), -(PI - 0.1)) - 0.2).abs() < 1e-6);
    }
}
