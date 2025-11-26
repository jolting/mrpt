/// 2D point types.
///
/// This module provides 2D point structures with f64 and f32 precision.

use std::ops::{Add, Sub, Mul, Div, AddAssign, SubAssign, MulAssign, DivAssign};
use std::fmt;

/// A 2D point with double precision (f64).
///
/// # Examples
///
/// ```
/// use mrpt_math::point2d::TPoint2D;
///
/// let p1 = TPoint2D::new(1.0, 2.0);
/// let p2 = TPoint2D::new(3.0, 4.0);
/// let sum = p1 + p2;
/// assert_eq!(sum.x, 4.0);
/// assert_eq!(sum.y, 6.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct TPoint2D {
    /// X coordinate
    pub x: f64,
    /// Y coordinate
    pub y: f64,
}

/// A 2D point with single precision (f32).
///
/// # Examples
///
/// ```
/// use mrpt_math::point2d::TPoint2Df;
///
/// let p = TPoint2Df::new(1.0, 2.0);
/// assert_eq!(p.x, 1.0);
/// assert_eq!(p.y, 2.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct TPoint2Df {
    /// X coordinate
    pub x: f32,
    /// Y coordinate
    pub y: f32,
}

// Implementation for TPoint2D
impl TPoint2D {
    /// Creates a new 2D point.
    ///
    /// # Examples
    ///
    /// ```
    /// use mrpt_math::point2d::TPoint2D;
    ///
    /// let p = TPoint2D::new(1.0, 2.0);
    /// assert_eq!(p.x, 1.0);
    /// assert_eq!(p.y, 2.0);
    /// ```
    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    /// Creates a point at the origin (0, 0).
    #[inline]
    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    /// Computes the Euclidean distance to another point.
    ///
    /// # Examples
    ///
    /// ```
    /// use mrpt_math::point2d::TPoint2D;
    ///
    /// let p1 = TPoint2D::new(0.0, 0.0);
    /// let p2 = TPoint2D::new(3.0, 4.0);
    /// assert_eq!(p1.distance_to(&p2), 5.0);
    /// ```
    #[inline]
    pub fn distance_to(&self, other: &Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Computes the squared Euclidean distance to another point.
    ///
    /// This is faster than `distance_to` as it avoids the square root.
    #[inline]
    pub fn sqr_distance_to(&self, other: &Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }

    /// Computes the norm (magnitude) of the point as a vector.
    #[inline]
    pub fn norm(&self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Computes the squared norm.
    #[inline]
    pub fn sqr_norm(&self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    /// Normalizes the point (treats it as a vector and scales to unit length).
    ///
    /// Returns None if the norm is zero.
    pub fn normalize(&self) -> Option<Self> {
        let n = self.norm();
        if n == 0.0 {
            None
        } else {
            Some(Self {
                x: self.x / n,
                y: self.y / n,
            })
        }
    }

    /// Dot product with another point.
    #[inline]
    pub fn dot(&self, other: &Self) -> f64 {
        self.x * other.x + self.y * other.y
    }

    /// Converts to an array [x, y].
    #[inline]
    pub fn to_array(&self) -> [f64; 2] {
        [self.x, self.y]
    }

    /// Creates from an array [x, y].
    #[inline]
    pub fn from_array(arr: [f64; 2]) -> Self {
        Self { x: arr[0], y: arr[1] }
    }
}

// Implementation for TPoint2Df
impl TPoint2Df {
    /// Creates a new 2D point.
    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Creates a point at the origin (0, 0).
    #[inline]
    pub const fn zero() -> Self {
        Self { x: 0.0, y: 0.0 }
    }

    /// Computes the Euclidean distance to another point.
    #[inline]
    pub fn distance_to(&self, other: &Self) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Computes the squared Euclidean distance to another point.
    #[inline]
    pub fn sqr_distance_to(&self, other: &Self) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }

    /// Computes the norm (magnitude) of the point as a vector.
    #[inline]
    pub fn norm(&self) -> f32 {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    /// Computes the squared norm.
    #[inline]
    pub fn sqr_norm(&self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    /// Dot product with another point.
    #[inline]
    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    /// Converts to an array [x, y].
    #[inline]
    pub fn to_array(&self) -> [f32; 2] {
        [self.x, self.y]
    }

    /// Creates from an array [x, y].
    #[inline]
    pub fn from_array(arr: [f32; 2]) -> Self {
        Self { x: arr[0], y: arr[1] }
    }

    /// Converts to TPoint2D (f64 precision).
    #[inline]
    pub fn to_f64(&self) -> TPoint2D {
        TPoint2D {
            x: self.x as f64,
            y: self.y as f64,
        }
    }
}

// Conversion from TPoint2D to TPoint2Df
impl From<TPoint2D> for TPoint2Df {
    fn from(p: TPoint2D) -> Self {
        Self {
            x: p.x as f32,
            y: p.y as f32,
        }
    }
}

// Arithmetic operations for TPoint2D
impl Add for TPoint2D {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for TPoint2D {
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Mul<f64> for TPoint2D {
    type Output = Self;
    #[inline]
    fn mul(self, scalar: f64) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

impl Div<f64> for TPoint2D {
    type Output = Self;
    #[inline]
    fn div(self, scalar: f64) -> Self {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
        }
    }
}

impl AddAssign for TPoint2D {
    #[inline]
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
    }
}

impl SubAssign for TPoint2D {
    #[inline]
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
    }
}

impl MulAssign<f64> for TPoint2D {
    #[inline]
    fn mul_assign(&mut self, scalar: f64) {
        self.x *= scalar;
        self.y *= scalar;
    }
}

impl DivAssign<f64> for TPoint2D {
    #[inline]
    fn div_assign(&mut self, scalar: f64) {
        self.x /= scalar;
        self.y /= scalar;
    }
}

// Arithmetic operations for TPoint2Df
impl Add for TPoint2Df {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for TPoint2Df {
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Mul<f32> for TPoint2Df {
    type Output = Self;
    #[inline]
    fn mul(self, scalar: f32) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }
}

impl Div<f32> for TPoint2Df {
    type Output = Self;
    #[inline]
    fn div(self, scalar: f32) -> Self {
        Self {
            x: self.x / scalar,
            y: self.y / scalar,
        }
    }
}

// Display implementations
impl fmt::Display for TPoint2D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

impl fmt::Display for TPoint2Df {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point2d_creation() {
        let p = TPoint2D::new(1.0, 2.0);
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);

        let z = TPoint2D::zero();
        assert_eq!(z.x, 0.0);
        assert_eq!(z.y, 0.0);
    }

    #[test]
    fn test_point2d_distance() {
        let p1 = TPoint2D::new(0.0, 0.0);
        let p2 = TPoint2D::new(3.0, 4.0);
        assert_eq!(p1.distance_to(&p2), 5.0);
        assert_eq!(p1.sqr_distance_to(&p2), 25.0);
    }

    #[test]
    fn test_point2d_norm() {
        let p = TPoint2D::new(3.0, 4.0);
        assert_eq!(p.norm(), 5.0);
        assert_eq!(p.sqr_norm(), 25.0);
    }

    #[test]
    fn test_point2d_normalize() {
        let p = TPoint2D::new(3.0, 4.0);
        let normalized = p.normalize().unwrap();
        assert!((normalized.norm() - 1.0).abs() < 1e-10);

        let zero = TPoint2D::zero();
        assert!(zero.normalize().is_none());
    }

    #[test]
    fn test_point2d_arithmetic() {
        let p1 = TPoint2D::new(1.0, 2.0);
        let p2 = TPoint2D::new(3.0, 4.0);

        let sum = p1 + p2;
        assert_eq!(sum.x, 4.0);
        assert_eq!(sum.y, 6.0);

        let diff = p2 - p1;
        assert_eq!(diff.x, 2.0);
        assert_eq!(diff.y, 2.0);

        let scaled = p1 * 2.0;
        assert_eq!(scaled.x, 2.0);
        assert_eq!(scaled.y, 4.0);

        let divided = p2 / 2.0;
        assert_eq!(divided.x, 1.5);
        assert_eq!(divided.y, 2.0);
    }

    #[test]
    fn test_point2d_dot() {
        let p1 = TPoint2D::new(1.0, 2.0);
        let p2 = TPoint2D::new(3.0, 4.0);
        assert_eq!(p1.dot(&p2), 11.0); // 1*3 + 2*4 = 11
    }

    #[test]
    fn test_point2d_array_conversion() {
        let p = TPoint2D::new(1.0, 2.0);
        let arr = p.to_array();
        assert_eq!(arr, [1.0, 2.0]);

        let p2 = TPoint2D::from_array([3.0, 4.0]);
        assert_eq!(p2.x, 3.0);
        assert_eq!(p2.y, 4.0);
    }

    #[test]
    fn test_point2df_creation() {
        let p = TPoint2Df::new(1.0, 2.0);
        assert_eq!(p.x, 1.0);
        assert_eq!(p.y, 2.0);
    }

    #[test]
    fn test_point_conversion() {
        let p64 = TPoint2D::new(1.0, 2.0);
        let p32: TPoint2Df = p64.into();
        assert_eq!(p32.x, 1.0);
        assert_eq!(p32.y, 2.0);

        let p64_back = p32.to_f64();
        assert_eq!(p64_back.x, 1.0);
        assert_eq!(p64_back.y, 2.0);
    }

    #[test]
    fn test_display() {
        let p = TPoint2D::new(1.5, 2.5);
        let s = format!("{}", p);
        assert_eq!(s, "(1.5, 2.5)");
    }
}
