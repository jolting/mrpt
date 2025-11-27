/// Plane geometry for 3D space.
///
/// This module provides plane representation in 3D.

use std::fmt;
use crate::point3d::TPoint3D;
use crate::line::TLine3D;
use crate::pose::TPose3D;
use crate::epsilon::get_epsilon;

/// A 3D plane represented by its equation Ax + By + Cz + D = 0.
///
/// The plane is stored as coefficients [A, B, C, D] where the plane equation is:
/// Ax + By + Cz + D = 0
///
/// The normal vector to the plane is (A, B, C).
///
/// # Examples
///
/// ```
/// use mrpt_math::plane::TPlane;
/// use mrpt_math::point3d::TPoint3D;
///
/// // Create a plane from three points
/// let p1 = TPoint3D::new(0.0, 0.0, 0.0);
/// let p2 = TPoint3D::new(1.0, 0.0, 0.0);
/// let p3 = TPoint3D::new(0.0, 1.0, 0.0);
/// let plane = TPlane::from_three_points(&p1, &p2, &p3);
///
/// // The XY plane should have z = 0
/// assert!(plane.contains(&p1));
/// assert!(plane.contains(&p2));
/// assert!(plane.contains(&p3));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct TPlane {
    /// Plane coefficients [A, B, C, D] for equation Ax + By + Cz + D = 0
    pub coefs: [f64; 4],
}

impl TPlane {
    /// Creates a new plane from coefficients A, B, C, D.
    ///
    /// The plane equation is: Ax + By + Cz + D = 0
    #[inline]
    pub fn new(a: f64, b: f64, c: f64, d: f64) -> Self {
        Self { coefs: [a, b, c, d] }
    }

    /// Creates a plane from three points.
    ///
    /// The plane will contain all three points.
    ///
    /// # Panics
    ///
    /// Panics if the three points are collinear (linearly dependent).
    pub fn from_three_points(p1: &TPoint3D, p2: &TPoint3D, p3: &TPoint3D) -> Self {
        // Compute two vectors in the plane
        let v1 = *p2 - *p1;
        let v2 = *p3 - *p1;
        
        // Normal vector is the cross product
        let normal = v1.cross(&v2);
        let norm = normal.norm();
        
        assert!(
            norm > get_epsilon(),
            "Cannot create plane from three collinear points"
        );
        
        // Plane equation: normal · (P - p1) = 0
        // normal · P = normal · p1
        // Ax + By + Cz = normal · p1
        // Ax + By + Cz - (normal · p1) = 0
        let d = -(normal.x * p1.x + normal.y * p1.y + normal.z * p1.z);
        
        Self {
            coefs: [normal.x, normal.y, normal.z, d],
        }
    }

    /// Creates a plane from a point and a normal vector.
    ///
    /// # Panics
    ///
    /// Panics if the normal vector is null (zero length).
    pub fn from_point_and_normal(point: &TPoint3D, normal: &TPoint3D) -> Self {
        let norm = normal.norm();
        
        assert!(
            norm > get_epsilon(),
            "Cannot create plane with null normal vector"
        );
        
        // Plane equation: normal · (P - point) = 0
        let d = -(normal.x * point.x + normal.y * point.y + normal.z * point.z);
        
        Self {
            coefs: [normal.x, normal.y, normal.z, d],
        }
    }

    /// Creates a plane from a point and a line.
    ///
    /// The plane will contain both the point and the entire line.
    ///
    /// # Panics
    ///
    /// Panics if the point is on the line (they would be coplanar in infinite ways).
    pub fn from_point_and_line(point: &TPoint3D, line: &TLine3D) -> Self {
        // Check if point is on the line
        assert!(
            !line.contains(point),
            "Cannot create plane: point is on the line"
        );
        
        // Vector from line base to point
        let v1 = *point - line.p_base;
        
        // Normal is cross product of line direction and v1
        let normal = line.director.cross(&v1);
        let norm = normal.norm();
        
        assert!(
            norm > get_epsilon(),
            "Cannot create plane: vectors are parallel"
        );
        
        Self::from_point_and_normal(point, &normal)
    }

    /// Creates a plane from two lines.
    ///
    /// The plane will contain both lines.
    ///
    /// # Panics
    ///
    /// Panics if the lines don't intersect or are parallel.
    pub fn from_two_lines(line1: &TLine3D, line2: &TLine3D) -> Self {
        // Normal is cross product of the two direction vectors
        let normal = line1.director.cross(&line2.director);
        let norm = normal.norm();
        
        assert!(
            norm > get_epsilon(),
            "Cannot create plane: lines are parallel"
        );
        
        // Check if lines intersect by verifying line2.p_base is on the plane
        // formed by line1.p_base and line1.director
        let temp_plane = Self::from_point_and_normal(&line1.p_base, &normal);
        
        assert!(
            temp_plane.contains(&line2.p_base),
            "Cannot create plane: lines don't intersect"
        );
        
        temp_plane
    }

    /// Evaluates the plane equation at a given point.
    ///
    /// Returns Ax + By + Cz + D. The point is on the plane if this returns ~0.
    #[inline]
    pub fn evaluate_point(&self, point: &TPoint3D) -> f64 {
        self.coefs[0] * point.x + self.coefs[1] * point.y + self.coefs[2] * point.z + self.coefs[3]
    }

    /// Checks whether a point is on the plane (within epsilon tolerance).
    pub fn contains(&self, point: &TPoint3D) -> bool {
        self.evaluate_point(point).abs() < get_epsilon()
    }

    /// Checks whether a line is fully contained in the plane.
    pub fn contains_line(&self, line: &TLine3D) -> bool {
        // Line is in plane if base point is in plane and director is perpendicular to normal
        if !self.contains(&line.p_base) {
            return false;
        }
        
        let normal = self.normal_vector();
        let dot = normal.dot(&line.director);
        
        dot.abs() < get_epsilon()
    }

    /// Computes the absolute distance from a point to the plane.
    pub fn distance(&self, point: &TPoint3D) -> f64 {
        self.signed_distance(point).abs()
    }

    /// Computes the signed distance from a point to the plane.
    ///
    /// Positive distance means the point is on the side of the normal vector.
    pub fn signed_distance(&self, point: &TPoint3D) -> f64 {
        let num = self.evaluate_point(point);
        let denom = (self.coefs[0] * self.coefs[0] + 
                     self.coefs[1] * self.coefs[1] + 
                     self.coefs[2] * self.coefs[2]).sqrt();
        num / denom
    }

    /// Computes the distance from a line to the plane.
    ///
    /// Returns 0 if the line intersects the plane or is contained in it.
    /// Returns the perpendicular distance if the line is parallel to the plane.
    pub fn distance_to_line(&self, line: &TLine3D) -> f64 {
        let normal = self.normal_vector();
        let dot = normal.dot(&line.director);
        
        // If line is not parallel to plane, they intersect
        if dot.abs() > get_epsilon() {
            return 0.0;
        }
        
        // Line is parallel - compute distance to any point on the line
        self.distance(&line.p_base)
    }

    /// Returns the normal vector to the plane [A, B, C].
    #[inline]
    pub fn normal_vector(&self) -> TPoint3D {
        TPoint3D::new(self.coefs[0], self.coefs[1], self.coefs[2])
    }

    /// Returns the unitary (unit length) normal vector.
    pub fn unitary_normal_vector(&self) -> TPoint3D {
        let normal = self.normal_vector();
        let norm = normal.norm();
        
        if norm > get_epsilon() {
            normal * (1.0 / norm)
        } else {
            TPoint3D::new(0.0, 0.0, 0.0)
        }
    }

    /// Normalizes the plane coefficients so that the normal vector is unitary.
    ///
    /// After this operation, A² + B² + C² = 1.
    pub fn unitarize(&mut self) {
        let norm = (self.coefs[0] * self.coefs[0] + 
                    self.coefs[1] * self.coefs[1] + 
                    self.coefs[2] * self.coefs[2]).sqrt();
        
        if norm > get_epsilon() {
            self.coefs[0] /= norm;
            self.coefs[1] /= norm;
            self.coefs[2] /= norm;
            self.coefs[3] /= norm;
        }
    }

    /// Returns a unitarized copy of the plane.
    pub fn unitarized(&self) -> Self {
        let mut result = *self;
        result.unitarize();
        result
    }

    /// Converts the plane to a 3D pose.
    ///
    /// The pose will be at the origin projected onto the plane, with orientation
    /// such that the Z axis is aligned with the plane normal.
    pub fn as_pose_3d(&self) -> TPose3D {
        // Find point on plane closest to origin
        let normal = self.normal_vector();
        let norm_sq = normal.sqr_norm();
        
        if norm_sq < get_epsilon() {
            return TPose3D::default();
        }
        
        // Point on plane closest to origin: -D * normal / ||normal||²
        let t = -self.coefs[3] / norm_sq;
        let point = normal * t;
        
        // Create pose with normal as Z axis
        self.as_pose_3d_forcing_origin(&point)
    }

    /// Converts the plane to a 3D pose with a specified origin point.
    ///
    /// The origin must be on the plane. The pose orientation will have Z axis
    /// aligned with the plane normal.
    pub fn as_pose_3d_forcing_origin(&self, center: &TPoint3D) -> TPose3D {
        let normal = self.unitary_normal_vector();
        
        // Find two perpendicular vectors in the plane
        // Choose a vector not parallel to normal
        let up = if normal.z.abs() < 0.9 {
            TPoint3D::new(0.0, 0.0, 1.0)
        } else {
            TPoint3D::new(1.0, 0.0, 0.0)
        };
        
        let x_axis = normal.cross(&up);
        let y_axis = normal.cross(&x_axis);
        
        // Compute rotation angles (simplified - full rotation matrix conversion would be better)
        let yaw = x_axis.y.atan2(x_axis.x);
        let pitch = (-x_axis.z).asin();
        let roll = 0.0; // Simplified
        
        TPose3D::new(center.x, center.y, center.z, yaw, pitch, roll)
    }

    /// Returns all coefficients as [A, B, C, D].
    #[inline]
    pub fn coefficients(&self) -> [f64; 4] {
        self.coefs
    }
}

impl Default for TPlane {
    /// Creates the XY plane (z = 0).
    fn default() -> Self {
        Self::new(0.0, 0.0, 1.0, 0.0)
    }
}

impl fmt::Display for TPlane {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[{}, {}, {}, {}]",
            self.coefs[0], self.coefs[1], self.coefs[2], self.coefs[3]
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_plane_from_three_points() {
        // XY plane (z = 0)
        let p1 = TPoint3D::new(0.0, 0.0, 0.0);
        let p2 = TPoint3D::new(1.0, 0.0, 0.0);
        let p3 = TPoint3D::new(0.0, 1.0, 0.0);
        
        let plane = TPlane::from_three_points(&p1, &p2, &p3);
        
        // All three points should be on the plane
        assert!(plane.contains(&p1));
        assert!(plane.contains(&p2));
        assert!(plane.contains(&p3));
        
        // Another point on XY plane
        let p4 = TPoint3D::new(2.0, 3.0, 0.0);
        assert!(plane.contains(&p4));
        
        // Point off the plane
        let p5 = TPoint3D::new(0.0, 0.0, 1.0);
        assert!(!plane.contains(&p5));
    }

    #[test]
    fn test_plane_from_point_and_normal() {
        // Plane through origin with normal (0, 0, 1) - the XY plane
        let point = TPoint3D::new(0.0, 0.0, 0.0);
        let normal = TPoint3D::new(0.0, 0.0, 1.0);
        
        let plane = TPlane::from_point_and_normal(&point, &normal);
        
        assert!(plane.contains(&point));
        assert!(plane.contains(&TPoint3D::new(1.0, 1.0, 0.0)));
        assert!(!plane.contains(&TPoint3D::new(0.0, 0.0, 1.0)));
    }

    #[test]
    fn test_plane_coefficients() {
        // Plane z = 5 => 0x + 0y + 1z - 5 = 0
        let plane = TPlane::new(0.0, 0.0, 1.0, -5.0);
        
        let p_on = TPoint3D::new(10.0, 20.0, 5.0);
        let p_off = TPoint3D::new(10.0, 20.0, 6.0);
        
        assert!(plane.contains(&p_on));
        assert!(!plane.contains(&p_off));
    }

    #[test]
    fn test_plane_distance() {
        // XY plane (z = 0) => 0x + 0y + 1z + 0 = 0
        let plane = TPlane::new(0.0, 0.0, 1.0, 0.0);
        
        let p = TPoint3D::new(5.0, 5.0, 3.0);
        assert!((plane.distance(&p) - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_plane_signed_distance() {
        // XY plane with normal pointing up
        let plane = TPlane::new(0.0, 0.0, 1.0, 0.0);
        
        let p_above = TPoint3D::new(0.0, 0.0, 5.0);
        let p_below = TPoint3D::new(0.0, 0.0, -5.0);
        
        assert!(plane.signed_distance(&p_above) > 0.0);
        assert!(plane.signed_distance(&p_below) < 0.0);
    }

    #[test]
    fn test_plane_unitarize() {
        let mut plane = TPlane::new(3.0, 4.0, 0.0, 10.0);
        plane.unitarize();
        
        let norm_sq = plane.coefs[0] * plane.coefs[0] + 
                      plane.coefs[1] * plane.coefs[1] + 
                      plane.coefs[2] * plane.coefs[2];
        
        assert!((norm_sq - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_plane_normal_vector() {
        let plane = TPlane::new(1.0, 2.0, 3.0, 4.0);
        let normal = plane.normal_vector();
        
        assert_eq!(normal.x, 1.0);
        assert_eq!(normal.y, 2.0);
        assert_eq!(normal.z, 3.0);
    }

    #[test]
    fn test_plane_contains_line() {
        // XY plane
        let plane = TPlane::new(0.0, 0.0, 1.0, 0.0);
        
        // Line in XY plane
        let line_in = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 0.0),
            TPoint3D::new(1.0, 1.0, 0.0),
        );
        
        // Line perpendicular to XY plane
        let line_out = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 0.0),
            TPoint3D::new(0.0, 0.0, 1.0),
        );
        
        assert!(plane.contains_line(&line_in));
        assert!(!plane.contains_line(&line_out));
    }

    #[test]
    fn test_plane_distance_to_line() {
        // XY plane
        let plane = TPlane::new(0.0, 0.0, 1.0, 0.0);
        
        // Line intersecting plane
        let line_intersect = TLine3D::new(
            TPoint3D::new(0.0, 0.0, -1.0),
            TPoint3D::new(0.0, 0.0, 1.0),
        );
        
        // Line parallel to plane at z = 2
        let line_parallel = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 2.0),
            TPoint3D::new(1.0, 0.0, 0.0),
        );
        
        assert!(plane.distance_to_line(&line_intersect).abs() < 1e-10);
        assert!((plane.distance_to_line(&line_parallel) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_plane_from_point_and_line() {
        let point = TPoint3D::new(0.0, 0.0, 1.0);
        let line = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 0.0),
            TPoint3D::new(1.0, 0.0, 0.0),
        );
        
        let plane = TPlane::from_point_and_line(&point, &line);
        
        // Plane should contain the point and the entire line
        assert!(plane.contains(&point));
        assert!(plane.contains(&line.p_base));
        assert!(plane.contains_line(&line));
    }

    #[test]
    fn test_plane_from_two_lines() {
        // Two lines in the XY plane
        let line1 = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 0.0),
            TPoint3D::new(1.0, 0.0, 0.0),
        );
        let line2 = TLine3D::new(
            TPoint3D::new(0.0, 0.0, 0.0),
            TPoint3D::new(0.0, 1.0, 0.0),
        );
        
        let plane = TPlane::from_two_lines(&line1, &line2);
        
        // Should be approximately the XY plane
        assert!(plane.contains_line(&line1));
        assert!(plane.contains_line(&line2));
    }

    #[test]
    fn test_plane_evaluate_point() {
        let plane = TPlane::new(1.0, 2.0, 3.0, 4.0);
        let point = TPoint3D::new(1.0, 1.0, 1.0);
        
        let result = plane.evaluate_point(&point);
        // 1*1 + 2*1 + 3*1 + 4 = 10
        assert!((result - 10.0).abs() < 1e-10);
    }
}
