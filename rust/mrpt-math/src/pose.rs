/// 2D and 3D pose types.
///
/// This module provides lightweight pose structures (position + orientation).

use std::fmt;
use crate::point2d::TPoint2D;
use crate::point3d::TPoint3D;

/// Lightweight 2D pose (x, y, phi).
///
/// Represents a position and orientation in 2D space.
///
/// # Examples
///
/// ```
/// use mrpt_math::pose::{TPose2D, TPose3D};
///
/// let pose = TPose2D::new(1.0, 2.0, 0.5);
/// assert_eq!(pose.x, 1.0);
/// assert_eq!(pose.y, 2.0);
/// assert_eq!(pose.phi, 0.5);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct TPose2D {
    /// X coordinate
    pub x: f64,
    /// Y coordinate
    pub y: f64,
    /// Orientation angle (radians)
    pub phi: f64,
}

/// Lightweight 3D pose (x, y, z, yaw, pitch, roll).
///
/// Represents a position and orientation in 3D space using Euler angles.
///
/// # Examples
///
/// ```
/// use mrpt_math::pose::TPose3D;
///
/// let pose = TPose3D::new(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
/// assert_eq!(pose.x, 1.0);
/// assert_eq!(pose.yaw, 0.1);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct TPose3D {
    /// X coordinate
    pub x: f64,
    /// Y coordinate
    pub y: f64,
    /// Z coordinate
    pub z: f64,
    /// Yaw angle (rotation around Z axis, radians)
    pub yaw: f64,
    /// Pitch angle (rotation around Y axis, radians)
    pub pitch: f64,
    /// Roll angle (rotation around X axis, radians)
    pub roll: f64,
}

// Implementation for TPose2D
impl TPose2D {
    /// Creates a new 2D pose.
    ///
    /// # Arguments
    ///
    /// * `x` - X coordinate
    /// * `y` - Y coordinate
    /// * `phi` - Orientation angle in radians
    #[inline]
    pub const fn new(x: f64, y: f64, phi: f64) -> Self {
        Self { x, y, phi }
    }

    /// Returns the identity transformation (0, 0, 0).
    #[inline]
    pub const fn identity() -> Self {
        Self { x: 0.0, y: 0.0, phi: 0.0 }
    }

    /// Creates a 2D pose from a point, with zero orientation.
    #[inline]
    pub fn from_point(p: TPoint2D) -> Self {
        Self { x: p.x, y: p.y, phi: 0.0 }
    }

    /// Extracts the position as a TPoint2D.
    #[inline]
    pub fn translation(&self) -> TPoint2D {
        TPoint2D { x: self.x, y: self.y }
    }

    /// Composes this pose with another (this ⊕ other).
    ///
    /// Implements pose composition: result = this + other in the SE(2) group.
    pub fn compose(&self, other: &Self) -> Self {
        let cos_phi = self.phi.cos();
        let sin_phi = self.phi.sin();
        
        Self {
            x: self.x + other.x * cos_phi - other.y * sin_phi,
            y: self.y + other.x * sin_phi + other.y * cos_phi,
            phi: mrpt_core::wrap2pi::wrap_to_pi(self.phi + other.phi),
        }
    }

    /// Inverse pose (-this).
    pub fn inverse(&self) -> Self {
        let cos_phi = self.phi.cos();
        let sin_phi = self.phi.sin();
        let inv_phi = -self.phi;
        
        Self {
            x: -self.x * cos_phi - self.y * sin_phi,
            y: self.x * sin_phi - self.y * cos_phi,
            phi: inv_phi,
        }
    }

    /// Composes inverse of this pose with another (⊖ this ⊕ other).
    pub fn inverse_compose(&self, other: &Self) -> Self {
        self.inverse().compose(other)
    }

    /// Distance to another pose (Euclidean distance of positions).
    #[inline]
    pub fn distance_to(&self, other: &Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Normalizes the phi angle to [-π, π].
    pub fn normalize(&mut self) {
        self.phi = mrpt_core::wrap2pi::wrap_to_pi(self.phi);
    }

    /// Returns a normalized copy.
    pub fn normalized(&self) -> Self {
        let mut copy = *self;
        copy.normalize();
        copy
    }

    /// Access by index: 0=x, 1=y, 2=phi.
    pub fn get(&self, index: usize) -> Option<f64> {
        match index {
            0 => Some(self.x),
            1 => Some(self.y),
            2 => Some(self.phi),
            _ => None,
        }
    }

    /// Converts to array [x, y, phi].
    #[inline]
    pub fn to_array(&self) -> [f64; 3] {
        [self.x, self.y, self.phi]
    }

    /// Creates from array [x, y, phi].
    #[inline]
    pub fn from_array(arr: [f64; 3]) -> Self {
        Self { x: arr[0], y: arr[1], phi: arr[2] }
    }
}

// Implementation for TPose3D
impl TPose3D {
    /// Creates a new 3D pose.
    ///
    /// # Arguments
    ///
    /// * `x`, `y`, `z` - Position coordinates
    /// * `yaw`, `pitch`, `roll` - Euler angles in radians
    #[inline]
    pub const fn new(x: f64, y: f64, z: f64, yaw: f64, pitch: f64, roll: f64) -> Self {
        Self { x, y, z, yaw, pitch, roll }
    }

    /// Returns the identity transformation.
    #[inline]
    pub const fn identity() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
        }
    }

    /// Creates a 3D pose from a 2D pose, zeroing z, pitch, and roll.
    /// The yaw is set from phi.
    #[inline]
    pub fn from_2d(pose: TPose2D) -> Self {
        Self {
            x: pose.x,
            y: pose.y,
            z: 0.0,
            yaw: pose.phi,
            pitch: 0.0,
            roll: 0.0,
        }
    }

    /// Creates from a 3D point with zero orientation.
    #[inline]
    pub fn from_point(p: TPoint3D) -> Self {
        Self {
            x: p.x,
            y: p.y,
            z: p.z,
            yaw: 0.0,
            pitch: 0.0,
            roll: 0.0,
        }
    }

    /// Extracts the position as a TPoint3D.
    #[inline]
    pub fn translation(&self) -> TPoint3D {
        TPoint3D { x: self.x, y: self.y, z: self.z }
    }

    /// Projects to 2D pose by dropping z, pitch, and roll.
    #[inline]
    pub fn to_2d(&self) -> TPose2D {
        TPose2D {
            x: self.x,
            y: self.y,
            phi: self.yaw,
        }
    }

    /// Distance to another pose (Euclidean distance of positions).
    #[inline]
    pub fn distance_to(&self, other: &Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Normalizes all angles to [-π, π].
    pub fn normalize(&mut self) {
        self.yaw = mrpt_core::wrap2pi::wrap_to_pi(self.yaw);
        self.pitch = mrpt_core::wrap2pi::wrap_to_pi(self.pitch);
        self.roll = mrpt_core::wrap2pi::wrap_to_pi(self.roll);
    }

    /// Returns a normalized copy.
    pub fn normalized(&self) -> Self {
        let mut copy = *self;
        copy.normalize();
        copy
    }

    /// Access by index: 0=x, 1=y, 2=z, 3=yaw, 4=pitch, 5=roll.
    pub fn get(&self, index: usize) -> Option<f64> {
        match index {
            0 => Some(self.x),
            1 => Some(self.y),
            2 => Some(self.z),
            3 => Some(self.yaw),
            4 => Some(self.pitch),
            5 => Some(self.roll),
            _ => None,
        }
    }

    /// Converts to array [x, y, z, yaw, pitch, roll].
    #[inline]
    pub fn to_array(&self) -> [f64; 6] {
        [self.x, self.y, self.z, self.yaw, self.pitch, self.roll]
    }

    /// Creates from array [x, y, z, yaw, pitch, roll].
    #[inline]
    pub fn from_array(arr: [f64; 6]) -> Self {
        Self {
            x: arr[0],
            y: arr[1],
            z: arr[2],
            yaw: arr[3],
            pitch: arr[4],
            roll: arr[5],
        }
    }
}

// Conversions
impl From<TPoint2D> for TPose2D {
    fn from(p: TPoint2D) -> Self {
        TPose2D::from_point(p)
    }
}

impl From<TPoint3D> for TPose3D {
    fn from(p: TPoint3D) -> Self {
        TPose3D::from_point(p)
    }
}

impl From<TPose2D> for TPose3D {
    fn from(p: TPose2D) -> Self {
        TPose3D::from_2d(p)
    }
}

// Display implementations
impl fmt::Display for TPose2D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {}, {})", self.x, self.y, self.phi)
    }
}

impl fmt::Display for TPose3D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "({}, {}, {}, {}, {}, {})",
            self.x, self.y, self.z, self.yaw, self.pitch, self.roll
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_pose2d_creation() {
        let pose = TPose2D::new(1.0, 2.0, 0.5);
        assert_eq!(pose.x, 1.0);
        assert_eq!(pose.y, 2.0);
        assert_eq!(pose.phi, 0.5);

        let identity = TPose2D::identity();
        assert_eq!(identity.x, 0.0);
        assert_eq!(identity.y, 0.0);
        assert_eq!(identity.phi, 0.0);
    }

    #[test]
    fn test_pose2d_from_point() {
        let p = TPoint2D::new(3.0, 4.0);
        let pose = TPose2D::from_point(p);
        assert_eq!(pose.x, 3.0);
        assert_eq!(pose.y, 4.0);
        assert_eq!(pose.phi, 0.0);
    }

    #[test]
    fn test_pose2d_compose() {
        let p1 = TPose2D::new(1.0, 0.0, PI / 2.0);
        let p2 = TPose2D::new(1.0, 0.0, 0.0);
        let result = p1.compose(&p2);
        
        assert!((result.x - 1.0).abs() < 1e-10);
        assert!((result.y - 1.0).abs() < 1e-10);
        assert!((result.phi - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_pose2d_inverse() {
        let pose = TPose2D::new(1.0, 2.0, 0.5);
        let inv = pose.inverse();
        let result = pose.compose(&inv);
        
        assert!(result.x.abs() < 1e-10);
        assert!(result.y.abs() < 1e-10);
        assert!(result.phi.abs() < 1e-10);
    }

    #[test]
    fn test_pose2d_distance() {
        let p1 = TPose2D::new(0.0, 0.0, 0.0);
        let p2 = TPose2D::new(3.0, 4.0, 0.0);
        assert_eq!(p1.distance_to(&p2), 5.0);
    }

    #[test]
    fn test_pose2d_normalize() {
        let mut pose = TPose2D::new(0.0, 0.0, 3.0 * PI);
        pose.normalize();
        assert!((pose.phi - PI).abs() < 1e-10 || (pose.phi + PI).abs() < 1e-10);
    }

    #[test]
    fn test_pose3d_creation() {
        let pose = TPose3D::new(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        assert_eq!(pose.x, 1.0);
        assert_eq!(pose.y, 2.0);
        assert_eq!(pose.z, 3.0);
        assert_eq!(pose.yaw, 0.1);
        assert_eq!(pose.pitch, 0.2);
        assert_eq!(pose.roll, 0.3);
    }

    #[test]
    fn test_pose3d_from_2d() {
        let p2 = TPose2D::new(1.0, 2.0, 0.5);
        let p3 = TPose3D::from_2d(p2);
        assert_eq!(p3.x, 1.0);
        assert_eq!(p3.y, 2.0);
        assert_eq!(p3.z, 0.0);
        assert_eq!(p3.yaw, 0.5);
        assert_eq!(p3.pitch, 0.0);
        assert_eq!(p3.roll, 0.0);
    }

    #[test]
    fn test_pose3d_to_2d() {
        let p3 = TPose3D::new(1.0, 2.0, 3.0, 0.5, 0.1, 0.2);
        let p2 = p3.to_2d();
        assert_eq!(p2.x, 1.0);
        assert_eq!(p2.y, 2.0);
        assert_eq!(p2.phi, 0.5);
    }

    #[test]
    fn test_pose3d_distance() {
        let p1 = TPose3D::identity();
        let p2 = TPose3D::new(1.0, 2.0, 2.0, 0.0, 0.0, 0.0);
        assert_eq!(p1.distance_to(&p2), 3.0);
    }

    #[test]
    fn test_pose_conversions() {
        let point = TPoint2D::new(1.0, 2.0);
        let pose: TPose2D = point.into();
        assert_eq!(pose.x, 1.0);
        assert_eq!(pose.y, 2.0);
        assert_eq!(pose.phi, 0.0);
    }

    #[test]
    fn test_display() {
        let pose2 = TPose2D::new(1.0, 2.0, 0.5);
        assert_eq!(format!("{}", pose2), "(1, 2, 0.5)");

        let pose3 = TPose3D::new(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        assert_eq!(format!("{}", pose3), "(1, 2, 3, 0.1, 0.2, 0.3)");
    }
}
