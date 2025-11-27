/// Line types for 2D and 3D geometry.
///
/// This module provides line representations without bounds.

use std::fmt;
use crate::point2d::TPoint2D;
use crate::point3d::TPoint3D;
use crate::pose::TPose2D;
use crate::epsilon::get_epsilon;

/// A 2D line represented by its equation Ax + By + C = 0.
///
/// The line is stored as coefficients [A, B, C] where the line equation is:
/// Ax + By + C = 0
///
/// # Examples
///
/// ```
/// use mrpt_math::line::TLine2D;
/// use mrpt_math::point2d::TPoint2D;
///
/// // Create a line from two points
/// let p1 = TPoint2D::new(0.0, 0.0);
/// let p2 = TPoint2D::new(1.0, 1.0);
/// let line = TLine2D::from_two_points(&p1, &p2);
///
/// // Check if a point is on the line
/// assert!(line.contains(&p1));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct TLine2D {
    /// Line coefficients [A, B, C] for equation Ax + By + C = 0
    pub coefs: [f64; 3],
}

impl TLine2D {
    /// Creates a new line from coefficients A, B, C.
    ///
    /// The line equation is: Ax + By + C = 0
    #[inline]
    pub fn new(a: f64, b: f64, c: f64) -> Self {
        Self { coefs: [a, b, c] }
    }

    /// Creates a line from two points.
    ///
    /// # Panics
    ///
    /// Panics if both points are the same.
    pub fn from_two_points(p1: &TPoint2D, p2: &TPoint2D) -> Self {
        let dx = p2.x - p1.x;
        let dy = p2.y - p1.y;
        
        assert!(
            dx.abs() > get_epsilon() || dy.abs() > get_epsilon(),
            "Cannot create line from two identical points"
        );
        
        // Line through two points: (y2-y1)x - (x2-x1)y + (x2-x1)y1 - (y2-y1)x1 = 0
        // Simplify to: A = dy, B = -dx, C = dx*y1 - dy*x1
        Self {
            coefs: [dy, -dx, dx * p1.y - dy * p1.x],
        }
    }

    /// Evaluates the line equation at a given point.
    ///
    /// Returns Ax + By + C. The point is on the line if this returns ~0.
    #[inline]
    pub fn evaluate_point(&self, point: &TPoint2D) -> f64 {
        self.coefs[0] * point.x + self.coefs[1] * point.y + self.coefs[2]
    }

    /// Checks whether a point is on the line (within epsilon tolerance).
    pub fn contains(&self, point: &TPoint2D) -> bool {
        self.evaluate_point(point).abs() < get_epsilon()
    }

    /// Computes the absolute distance from a point to the line.
    pub fn distance(&self, point: &TPoint2D) -> f64 {
        self.signed_distance(point).abs()
    }

    /// Computes the signed distance from a point to the line.
    ///
    /// The sign indicates which side of the line the point is on.
    pub fn signed_distance(&self, point: &TPoint2D) -> f64 {
        let num = self.evaluate_point(point);
        let denom = (self.coefs[0] * self.coefs[0] + self.coefs[1] * self.coefs[1]).sqrt();
        num / denom
    }

    /// Returns the normal vector to the line [A, B].
    ///
    /// This vector is perpendicular to the line direction.
    #[inline]
    pub fn normal_vector(&self) -> [f64; 2] {
        [self.coefs[0], self.coefs[1]]
    }

    /// Returns the director (direction) vector of the line [-B, A].
    ///
    /// This vector is parallel to the line direction.
    #[inline]
    pub fn director_vector(&self) -> [f64; 2] {
        [-self.coefs[1], self.coefs[0]]
    }

    /// Normalizes the line coefficients so that the normal vector is unitary.
    ///
    /// After this operation, A² + B² = 1.
    pub fn unitarize(&mut self) {
        let norm = (self.coefs[0] * self.coefs[0] + self.coefs[1] * self.coefs[1]).sqrt();
        if norm > get_epsilon() {
            self.coefs[0] /= norm;
            self.coefs[1] /= norm;
            self.coefs[2] /= norm;
        }
    }

    /// Returns a unitarized copy of the line.
    pub fn unitarized(&self) -> Self {
        let mut result = *self;
        result.unitarize();
        result
    }

    /// Returns the unitary normal vector [A, B] after unitarizing.
    pub fn unitary_normal_vector(&self) -> [f64; 2] {
        let unitarized = self.unitarized();
        unitarized.normal_vector()
    }

    /// Returns the unitary director vector [-B, A] after unitarizing.
    pub fn unitary_director_vector(&self) -> [f64; 2] {
        let unitarized = self.unitarized();
        unitarized.director_vector()
    }

    /// Converts to a 2D pose.
    ///
    /// The pose will be at the origin projected onto the line, with the
    /// orientation parallel to the line direction.
    pub fn as_pose_2d(&self) -> TPose2D {
        // Find point on line closest to origin
        let [a, b, c] = self.coefs;
        let denom = a * a + b * b;
        
        if denom < get_epsilon() {
            return TPose2D::default();
        }
        
        let x = -a * c / denom;
        let y = -b * c / denom;
        let phi = (-self.coefs[1]).atan2(self.coefs[0]);
        
        TPose2D::new(x, y, phi)
    }

    /// Converts to a 2D pose forcing a specific origin point on the line.
    pub fn as_pose_2d_forcing_origin(&self, origin: &TPoint2D) -> TPose2D {
        let phi = (-self.coefs[1]).atan2(self.coefs[0]);
        TPose2D::new(origin.x, origin.y, phi)
    }

    /// Converts to a 3D line by setting z = 0.
    pub fn to_3d(&self) -> TLine3D {
        // Find a point on the 2D line
        let [a, b, c] = self.coefs;
        let base = if b.abs() > get_epsilon() {
            TPoint3D::new(0.0, -c / b, 0.0)
        } else if a.abs() > get_epsilon() {
            TPoint3D::new(-c / a, 0.0, 0.0)
        } else {
            TPoint3D::new(0.0, 0.0, 0.0)
        };
        
        // Director is perpendicular to normal in XY plane
        let director = TPoint3D::new(-b, a, 0.0);
        
        TLine3D::from_point_and_director(&base, &director)
    }

    /// Returns all coefficients as [A, B, C].
    #[inline]
    pub fn coefficients(&self) -> [f64; 3] {
        self.coefs
    }
}

impl Default for TLine2D {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
}

impl fmt::Display for TLine2D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "[{}, {}, {}]", self.coefs[0], self.coefs[1], self.coefs[2])
    }
}

/// A 3D line represented by a base point and a director vector.
///
/// The line is defined parametrically as: P(t) = pBase + t * director
///
/// # Examples
///
/// ```
/// use mrpt_math::line::TLine3D;
/// use mrpt_math::point3d::TPoint3D;
///
/// // Create a line from two points
/// let p1 = TPoint3D::new(0.0, 0.0, 0.0);
/// let p2 = TPoint3D::new(1.0, 1.0, 1.0);
/// let line = TLine3D::from_two_points(&p1, &p2);
///
/// // Check if a point is on the line
/// assert!(line.contains(&p1));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct TLine3D {
    /// Base point on the line
    pub p_base: TPoint3D,
    /// Director (direction) vector
    pub director: TPoint3D,
}

impl TLine3D {
    /// Creates a new 3D line from a base point and director vector.
    #[inline]
    pub fn new(p_base: TPoint3D, director: TPoint3D) -> Self {
        Self { p_base, director }
    }

    /// Creates a line from a base point and director vector.
    #[inline]
    pub fn from_point_and_director(base_point: &TPoint3D, director_vector: &TPoint3D) -> Self {
        Self {
            p_base: *base_point,
            director: *director_vector,
        }
    }

    /// Creates a line from two points.
    ///
    /// # Panics
    ///
    /// Panics if both points are the same.
    pub fn from_two_points(p1: &TPoint3D, p2: &TPoint3D) -> Self {
        let director = *p2 - *p1;
        let norm = director.norm();
        
        assert!(
            norm > get_epsilon(),
            "Cannot create line from two identical points"
        );
        
        Self {
            p_base: *p1,
            director,
        }
    }

    /// Creates a 3D line from a 2D line by setting z = 0.
    pub fn from_2d(line: &TLine2D) -> Self {
        line.to_3d()
    }

    /// Checks whether a point is on the line (within epsilon tolerance).
    pub fn contains(&self, point: &TPoint3D) -> bool {
        // Point is on line if (point - pBase) × director = 0
        let v = *point - self.p_base;
        let cross = v.cross(&self.director);
        cross.norm() < get_epsilon()
    }

    /// Computes the absolute distance from a point to the line.
    pub fn distance(&self, point: &TPoint3D) -> f64 {
        // Distance = ||(point - pBase) × director|| / ||director||
        let v = *point - self.p_base;
        let cross = v.cross(&self.director);
        let director_norm = self.director.norm();
        
        if director_norm < get_epsilon() {
            return v.norm();
        }
        
        cross.norm() / director_norm
    }

    /// Finds the closest point on the line to a given point.
    ///
    /// This is the intersection of this line with the plane perpendicular
    /// to this line that passes through the given point.
    pub fn closest_point_to(&self, p: &TPoint3D) -> TPoint3D {
        let v = *p - self.p_base;
        let director_norm_sq = self.director.sqr_norm();
        
        if director_norm_sq < get_epsilon() {
            return self.p_base;
        }
        
        let t = v.dot(&self.director) / director_norm_sq;
        self.p_base + self.director * t
    }

    /// Computes the minimum distance between this line and another line.
    ///
    /// Returns None if the lines are parallel, Some(0.0) if they intersect,
    /// or Some(distance) otherwise. Optionally returns the midpoint of the
    /// shortest segment connecting the lines.
    pub fn distance_to_line(&self, other: &TLine3D) -> Option<f64> {
        self.distance_to_line_with_midpoint(other).map(|(d, _)| d)
    }

    /// Computes the minimum distance between this line and another line,
    /// along with the midpoint of the shortest segment.
    pub fn distance_to_line_with_midpoint(&self, other: &TLine3D) -> Option<(f64, TPoint3D)> {
        let w0 = self.p_base - other.p_base;
        let a = self.director.dot(&self.director);
        let b = self.director.dot(&other.director);
        let c = other.director.dot(&other.director);
        let d = self.director.dot(&w0);
        let e = other.director.dot(&w0);
        
        let denom = a * c - b * b;
        
        // Check if lines are parallel
        if denom.abs() < get_epsilon() {
            // Lines are parallel - compute distance from point to line
            let dist = other.distance(&self.p_base);
            let mid = (self.p_base + other.p_base) * 0.5;
            return Some((dist, mid));
        }
        
        let sc = (b * e - c * d) / denom;
        let tc = (a * e - b * d) / denom;
        
        let p1 = self.p_base + self.director * sc;
        let p2 = other.p_base + other.director * tc;
        
        let dist = (p1 - p2).norm();
        let mid = (p1 + p2) * 0.5;
        
        Some((dist, mid))
    }

    /// Normalizes the director vector to unit length.
    pub fn unitarize(&mut self) {
        let norm = self.director.norm();
        if norm > get_epsilon() {
            self.director = self.director * (1.0 / norm);
        }
    }

    /// Returns a copy with unitarized director vector.
    pub fn unitarized(&self) -> Self {
        let mut result = *self;
        result.unitarize();
        result
    }

    /// Returns the director vector.
    #[inline]
    pub fn director_vector(&self) -> TPoint3D {
        self.director
    }

    /// Returns the unitary director vector.
    pub fn unitary_director_vector(&self) -> TPoint3D {
        let norm = self.director.norm();
        if norm > get_epsilon() {
            self.director * (1.0 / norm)
        } else {
            TPoint3D::new(0.0, 0.0, 0.0)
        }
    }

    /// Projects the line into 2D by discarding the Z coordinate.
    ///
    /// # Panics
    ///
    /// Panics if the director vector is orthogonal to the XY plane (vertical line).
    pub fn to_2d(&self) -> TLine2D {
        assert!(
            self.director.x.abs() > get_epsilon() || self.director.y.abs() > get_epsilon(),
            "Cannot project vertical line (orthogonal to XY plane) to 2D"
        );
        
        let p1 = TPoint2D::new(self.p_base.x, self.p_base.y);
        let p2 = TPoint2D::new(
            self.p_base.x + self.director.x,
            self.p_base.y + self.director.y,
        );
        
        TLine2D::from_two_points(&p1, &p2)
    }
}

impl Default for TLine3D {
    fn default() -> Self {
        Self {
            p_base: TPoint3D::default(),
            director: TPoint3D::new(0.0, 0.0, 0.0),
        }
    }
}

impl fmt::Display for TLine3D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "P=[{}, {}, {}] u=[{}, {}, {}]",
            self.p_base.x, self.p_base.y, self.p_base.z,
            self.director.x, self.director.y, self.director.z
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::epsilon::set_epsilon;

    #[test]
    fn test_line2d_from_two_points() {
        let p1 = TPoint2D::new(0.0, 0.0);
        let p2 = TPoint2D::new(1.0, 1.0);
        let line = TLine2D::from_two_points(&p1, &p2);
        
        assert!(line.contains(&p1));
        assert!(line.contains(&p2));
    }

    #[test]
    fn test_line2d_coefficients() {
        // Horizontal line y = 2 => 0x + 1y - 2 = 0
        let line = TLine2D::new(0.0, 1.0, -2.0);
        
        let p_on = TPoint2D::new(5.0, 2.0);
        let p_off = TPoint2D::new(5.0, 3.0);
        
        assert!(line.contains(&p_on));
        assert!(!line.contains(&p_off));
    }

    #[test]
    fn test_line2d_distance() {
        // Horizontal line y = 0 => 0x + 1y + 0 = 0
        let line = TLine2D::new(0.0, 1.0, 0.0);
        
        let p = TPoint2D::new(5.0, 3.0);
        assert!((line.distance(&p) - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_line2d_signed_distance() {
        // Line x = 0 => 1x + 0y + 0 = 0
        let line = TLine2D::new(1.0, 0.0, 0.0);
        
        let p_positive = TPoint2D::new(5.0, 0.0);
        let p_negative = TPoint2D::new(-5.0, 0.0);
        
        assert!(line.signed_distance(&p_positive) > 0.0);
        assert!(line.signed_distance(&p_negative) < 0.0);
    }

    #[test]
    fn test_line2d_unitarize() {
        let mut line = TLine2D::new(3.0, 4.0, 5.0);
        line.unitarize();
        
        let norm_sq = line.coefs[0] * line.coefs[0] + line.coefs[1] * line.coefs[1];
        assert!((norm_sq - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_line2d_vectors() {
        let line = TLine2D::new(1.0, 2.0, 3.0);
        
        let normal = line.normal_vector();
        assert_eq!(normal, [1.0, 2.0]);
        
        let director = line.director_vector();
        assert_eq!(director, [-2.0, 1.0]);
        
        // Director and normal should be perpendicular
        let dot = director[0] * normal[0] + director[1] * normal[1];
        assert!(dot.abs() < 1e-10);
    }

    #[test]
    fn test_line3d_from_two_points() {
        let p1 = TPoint3D::new(0.0, 0.0, 0.0);
        let p2 = TPoint3D::new(1.0, 1.0, 1.0);
        let line = TLine3D::from_two_points(&p1, &p2);
        
        assert!(line.contains(&p1));
        assert!(line.contains(&p2));
    }

    #[test]
    fn test_line3d_contains() {
        let p_base = TPoint3D::new(1.0, 2.0, 3.0);
        let director = TPoint3D::new(1.0, 0.0, 0.0);
        let line = TLine3D::new(p_base, director);
        
        let p_on_line = TPoint3D::new(5.0, 2.0, 3.0);
        let p_off_line = TPoint3D::new(5.0, 3.0, 3.0);
        
        assert!(line.contains(&p_on_line));
        assert!(!line.contains(&p_off_line));
    }

    #[test]
    fn test_line3d_distance() {
        // Line along X axis through origin
        let line = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 0.0),
            TPoint3D::new(1.0, 0.0, 0.0),
        );
        
        let p = TPoint3D::new(5.0, 3.0, 4.0);
        let expected_dist = 5.0; // sqrt(3^2 + 4^2)
        
        assert!((line.distance(&p) - expected_dist).abs() < 1e-10);
    }

    #[test]
    fn test_line3d_closest_point() {
        // Line along X axis
        let line = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 0.0),
            TPoint3D::new(1.0, 0.0, 0.0),
        );
        
        let p = TPoint3D::new(5.0, 3.0, 4.0);
        let closest = line.closest_point_to(&p);
        
        assert!((closest.x - 5.0).abs() < 1e-10);
        assert!(closest.y.abs() < 1e-10);
        assert!(closest.z.abs() < 1e-10);
    }

    #[test]
    fn test_line3d_distance_to_line_parallel() {
        let line1 = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 0.0),
            TPoint3D::new(1.0, 0.0, 0.0),
        );
        let line2 = TLine3D::new(
            TPoint3D::new(0.0, 1.0, 0.0),
            TPoint3D::new(1.0, 0.0, 0.0),
        );
        
        let dist = line1.distance_to_line(&line2);
        assert!(dist.is_some());
        assert!((dist.unwrap() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_line3d_distance_to_line_intersecting() {
        let line1 = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 0.0),
            TPoint3D::new(1.0, 0.0, 0.0),
        );
        let line2 = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 0.0),
            TPoint3D::new(0.0, 1.0, 0.0),
        );
        
        let dist = line1.distance_to_line(&line2);
        assert!(dist.is_some());
        assert!(dist.unwrap() < 1e-10);
    }

    #[test]
    fn test_line3d_unitarize() {
        let mut line = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 0.0),
            TPoint3D::new(3.0, 4.0, 0.0),
        );
        
        line.unitarize();
        assert!((line.director.norm() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_line2d_to_3d() {
        let line2d = TLine2D::from_two_points(
            &TPoint2D::new(0.0, 0.0),
            &TPoint2D::new(1.0, 1.0),
        );
        
        let line3d = line2d.to_3d();
        
        // Check that 2D points project to 3D line
        let p3d = TPoint3D::new(2.0, 2.0, 0.0);
        assert!(line3d.contains(&p3d));
    }

    #[test]
    fn test_line3d_to_2d() {
        let line3d = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 5.0),
            TPoint3D::new(1.0, 1.0, 0.0),
        );
        
        let line2d = line3d.to_2d();
        
        // Check that projection works
        let p2d = TPoint2D::new(1.0, 1.0);
        assert!(line2d.contains(&p2d));
    }
}
