/// 3D point types.
///
/// This module provides 3D point structures with f64 and f32 precision.

use std::ops::{Add, Sub, Mul, Div, AddAssign, SubAssign, MulAssign, DivAssign};
use std::fmt;
use crate::point2d::{TPoint2D, TPoint2Df};

/// A 3D point with double precision (f64).
///
/// # Examples
///
/// ```
/// use mrpt_math::point3d::TPoint3D;
///
/// let p1 = TPoint3D::new(1.0, 2.0, 3.0);
/// let p2 = TPoint3D::new(4.0, 5.0, 6.0);
/// let sum = p1 + p2;
/// assert_eq!(sum.x, 5.0);
/// assert_eq!(sum.y, 7.0);
/// assert_eq!(sum.z, 9.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct TPoint3D {
    /// X coordinate
    pub x: f64,
    /// Y coordinate
    pub y: f64,
    /// Z coordinate
    pub z: f64,
}

/// A 3D point with single precision (f32).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct TPoint3Df {
    /// X coordinate
    pub x: f32,
    /// Y coordinate
    pub y: f32,
    /// Z coordinate
    pub z: f32,
}

// Implementation for TPoint3D
impl TPoint3D {
    /// Creates a new 3D point.
    #[inline]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Creates a point at the origin (0, 0, 0).
    #[inline]
    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    /// Creates a 3D point from a 2D point, zeroing the z coordinate.
    #[inline]
    pub fn from_2d(p: TPoint2D) -> Self {
        Self { x: p.x, y: p.y, z: 0.0 }
    }

    /// Computes the Euclidean distance to another point.
    #[inline]
    pub fn distance_to(&self, other: &Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Computes the squared Euclidean distance to another point.
    #[inline]
    pub fn sqr_distance_to(&self, other: &Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Computes the norm (magnitude) of the point as a vector.
    #[inline]
    pub fn norm(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Computes the squared norm.
    #[inline]
    pub fn sqr_norm(&self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    /// Normalizes the point (treats it as a vector and scales to unit length).
    pub fn normalize(&self) -> Option<Self> {
        let n = self.norm();
        if n == 0.0 {
            None
        } else {
            Some(Self {
                x: self.x / n,
                y: self.y / n,
                z: self.z / n,
            })
        }
    }

    /// Dot product with another point.
    #[inline]
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product with another point.
    #[inline]
    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    /// Converts to an array [x, y, z].
    #[inline]
    pub fn to_array(&self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    /// Creates from an array [x, y, z].
    #[inline]
    pub fn from_array(arr: [f64; 3]) -> Self {
        Self { x: arr[0], y: arr[1], z: arr[2] }
    }

    /// Projects to 2D by dropping the z coordinate.
    #[inline]
    pub fn to_2d(&self) -> TPoint2D {
        TPoint2D { x: self.x, y: self.y }
    }
}

// Implementation for TPoint3Df
impl TPoint3Df {
    /// Creates a new 3D point.
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Creates a point at the origin (0, 0, 0).
    #[inline]
    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    /// Creates a 3D point from a 2D point.
    #[inline]
    pub fn from_2d(p: TPoint2Df) -> Self {
        Self { x: p.x, y: p.y, z: 0.0 }
    }

    /// Computes the Euclidean distance to another point.
    #[inline]
    pub fn distance_to(&self, other: &Self) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Computes the squared Euclidean distance to another point.
    #[inline]
    pub fn sqr_distance_to(&self, other: &Self) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Computes the norm (magnitude).
    #[inline]
    pub fn norm(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Dot product with another point.
    #[inline]
    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    /// Cross product with another point.
    #[inline]
    pub fn cross(&self, other: &Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    /// Converts to TPoint3D (f64 precision).
    #[inline]
    pub fn to_f64(&self) -> TPoint3D {
        TPoint3D {
            x: self.x as f64,
            y: self.y as f64,
            z: self.z as f64,
        }
    }

    /// Projects to 2D by dropping the z coordinate.
    #[inline]
    pub fn to_2d(&self) -> TPoint2Df {
        TPoint2Df { x: self.x, y: self.y }
    }
}

// Conversions
impl From<TPoint3D> for TPoint3Df {
    fn from(p: TPoint3D) -> Self {
        Self {
            x: p.x as f32,
            y: p.y as f32,
            z: p.z as f32,
        }
    }
}

impl From<TPoint2D> for TPoint3D {
    fn from(p: TPoint2D) -> Self {
        TPoint3D::from_2d(p)
    }
}

// Arithmetic operations for TPoint3D
impl Add for TPoint3D {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for TPoint3D {
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f64> for TPoint3D {
    type Output = Self;
    #[inline]
    fn mul(self, scalar: f64) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

impl Div<f64> for TPoint3D {
    type Output = Self;
    #[inline]
    fn div(self, scalar: f64) -> Self {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
            z: self.z / scalar,
        }
    }
}

impl AddAssign for TPoint3D {
    #[inline]
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}

impl SubAssign for TPoint3D {
    #[inline]
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
        self.z -= other.z;
    }
}

// Arithmetic operations for TPoint3Df
impl Add for TPoint3Df {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for TPoint3Df {
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f32> for TPoint3Df {
    type Output = Self;
    #[inline]
    fn mul(self, scalar: f32) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

// Display implementations
impl fmt::Display for TPoint3D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {}, {})", self.x, self.y, self.z)
    }
}

impl fmt::Display for TPoint3Df {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {}, {})", self.x, self.y, self.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point3d_creation() {
        let p = TPoint3D::new(1.0, 2.0, 3.0);
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);
        assert_eq!(p.z, 3.0);

        let z = TPoint3D::zero();
        assert_eq!(z.x, 0.0);
        assert_eq!(z.y, 0.0);
        assert_eq!(z.z, 0.0);
    }

    #[test]
    fn test_point3d_from_2d() {
        let p2 = TPoint2D::new(1.0, 2.0);
        let p3 = TPoint3D::from_2d(p2);
        assert_eq!(p3.x, 1.0);
        assert_eq!(p3.y, 2.0);
        assert_eq!(p3.z, 0.0);
    }

    #[test]
    fn test_point3d_distance() {
        let p1 = TPoint3D::new(0.0, 0.0, 0.0);
        let p2 = TPoint3D::new(1.0, 2.0, 2.0);
        assert_eq!(p1.distance_to(&p2), 3.0);
        assert_eq!(p1.sqr_distance_to(&p2), 9.0);
    }

    #[test]
    fn test_point3d_norm() {
        let p = TPoint3D::new(2.0, 3.0, 6.0);
        assert_eq!(p.norm(), 7.0);
        assert_eq!(p.sqr_norm(), 49.0);
    }

    #[test]
    fn test_point3d_normalize() {
        let p = TPoint3D::new(3.0, 4.0, 0.0);
        let normalized = p.normalize().unwrap();
        assert!((normalized.norm() - 1.0).abs() < 1e-10);

        let zero = TPoint3D::zero();
        assert!(zero.normalize().is_none());
    }

    #[test]
    fn test_point3d_arithmetic() {
        let p1 = TPoint3D::new(1.0, 2.0, 3.0);
        let p2 = TPoint3D::new(4.0, 5.0, 6.0);

        let sum = p1 + p2;
        assert_eq!(sum, TPoint3D::new(5.0, 7.0, 9.0));

        let diff = p2 - p1;
        assert_eq!(diff, TPoint3D::new(3.0, 3.0, 3.0));

        let scaled = p1 * 2.0;
        assert_eq!(scaled, TPoint3D::new(2.0, 4.0, 6.0));

        let divided = p2 / 2.0;
        assert_eq!(divided, TPoint3D::new(2.0, 2.5, 3.0));
    }

    #[test]
    fn test_point3d_dot() {
        let p1 = TPoint3D::new(1.0, 2.0, 3.0);
        let p2 = TPoint3D::new(4.0, 5.0, 6.0);
        assert_eq!(p1.dot(&p2), 32.0); // 1*4 + 2*5 + 3*6 = 32
    }

    #[test]
    fn test_point3d_cross() {
        let i = TPoint3D::new(1.0, 0.0, 0.0);
        let j = TPoint3D::new(0.0, 1.0, 0.0);
        let k = i.cross(&j);
        assert_eq!(k, TPoint3D::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn test_point3d_projections() {
        let p3 = TPoint3D::new(1.0, 2.0, 3.0);
        let p2 = p3.to_2d();
        assert_eq!(p2.x, 1.0);
        assert_eq!(p2.y, 2.0);
    }

    #[test]
    fn test_display() {
        let p = TPoint3D::new(1.5, 2.5, 3.5);
        let s = format!("{}", p);
        assert_eq!(s, "(1.5, 2.5, 3.5)");
    }
}
