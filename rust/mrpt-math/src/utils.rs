/// Math utility functions.
///
/// This module provides various mathematical utilities for floating-point
/// comparisons, sequences, and other operations.

use num_traits::Float;
use std::ops::{Add, Sub, Mul, Div};

/// Compare two floats and determine whether they are approximately equal.
///
/// # Arguments
///
/// * `a` - First number
/// * `b` - Second number
/// * `epsilon` - Difference below which a, b are considered equal
///
/// # Examples
///
/// ```
/// use mrpt_math::utils::approximately_equal_eps;
///
/// assert!(approximately_equal_eps(1.0, 1.0000001, 1e-5));
/// assert!(!approximately_equal_eps(1.0, 1.001, 1e-5));
/// ```
pub fn approximately_equal_eps<T, E>(a: T, b: T, epsilon: E) -> bool
where
    T: Float,
    E: Into<T>,
{
    let eps = epsilon.into();
    let diff = (a - b).abs();
    let max_val = a.abs().max(b.abs());
    diff <= max_val * eps
}

/// Compare two floats and determine whether they are approximately equal
/// using the type's default epsilon.
///
/// # Examples
///
/// ```
/// use mrpt_math::utils::approximately_equal;
///
/// assert!(approximately_equal(1.0_f64, 1.0_f64));
/// assert!(!approximately_equal(1.0_f64, 2.0_f64));
/// ```
pub fn approximately_equal<T>(a: T, b: T) -> bool
where
    T: Float,
{
    approximately_equal_eps(a, b, T::epsilon())
}

/// Compute the absolute difference between two numbers.
///
/// # Examples
///
/// ```
/// use mrpt_math::utils::abs_diff;
///
/// assert_eq!(abs_diff(10, 5), 5);
/// assert_eq!(abs_diff(5, 10), 5);
/// assert_eq!(abs_diff(3.5, 1.2), 2.3);
/// ```
#[inline]
pub fn abs_diff<T>(lhs: T, rhs: T) -> T
where
    T: PartialOrd + Sub<Output = T>,
{
    if lhs > rhs {
        lhs - rhs
    } else {
        rhs - lhs
    }
}

/// Generates an equidistant sequence of numbers.
///
/// Creates `count` evenly-spaced values from `first` to `last` (inclusive).
///
/// # Arguments
///
/// * `first` - First value in the sequence
/// * `last` - Last value in the sequence
/// * `count` - Number of points to generate
///
/// # Examples
///
/// ```
/// use mrpt_math::utils::linspace;
///
/// let seq = linspace(0.0, 10.0, 5);
/// assert_eq!(seq, vec![0.0, 2.5, 5.0, 7.5, 10.0]);
///
/// let seq = linspace(0.0, 1.0, 3);
/// assert_eq!(seq, vec![0.0, 0.5, 1.0]);
/// ```
pub fn linspace<T>(first: T, last: T, count: usize) -> Vec<T>
where
    T: Float + Add<Output = T> + Sub<Output = T> + Mul<Output = T> + Div<Output = T>,
{
    if count == 0 {
        return Vec::new();
    }
    
    if count == 1 {
        return vec![last];
    }

    let mut result = Vec::with_capacity(count);
    let count_t = T::from(count - 1).unwrap();
    let incr = (last - first) / count_t;
    
    for i in 0..count {
        let i_t = T::from(i).unwrap();
        result.push(first + incr * i_t);
    }
    
    result
}

/// Generates a sequence of numbers with a given increment.
///
/// Creates values from `first` to `last` with step size `step`.
///
/// # Examples
///
/// ```
/// use mrpt_math::utils::sequence;
///
/// let seq = sequence(0.0, 10.0, 2.0);
/// assert_eq!(seq, vec![0.0, 2.0, 4.0, 6.0, 8.0, 10.0]);
/// ```
pub fn sequence<T>(first: T, last: T, step: T) -> Vec<T>
where
    T: Float + Add<Output = T> + PartialOrd,
{
    let mut result = Vec::new();
    let mut current = first;
    
    while current <= last {
        result.push(current);
        current = current + step;
    }
    
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_approximately_equal_eps() {
        assert!(approximately_equal_eps(1.0, 1.0, 1e-5));
        assert!(approximately_equal_eps(1.0, 1.0000001, 1e-5));
        assert!(!approximately_equal_eps(1.0, 1.001, 1e-5));
        
        assert!(approximately_equal_eps(0.0, 0.0, 1e-10));
        assert!(approximately_equal_eps(-1.0, -1.0, 1e-10));
    }

    #[test]
    fn test_approximately_equal() {
        assert!(approximately_equal(1.0_f64, 1.0_f64));
        assert!(approximately_equal(1.0_f32, 1.0_f32));
        assert!(!approximately_equal(1.0_f64, 2.0_f64));
    }

    #[test]
    fn test_abs_diff() {
        assert_eq!(abs_diff(10, 5), 5);
        assert_eq!(abs_diff(5, 10), 5);
        assert_eq!(abs_diff(10, 10), 0);
        
        assert!((abs_diff(3.5, 1.2) - 2.3).abs() < 1e-10);
        assert!((abs_diff(1.2, 3.5) - 2.3).abs() < 1e-10);
    }

    #[test]
    fn test_linspace() {
        let seq = linspace(0.0, 10.0, 5);
        assert_eq!(seq.len(), 5);
        assert!((seq[0] - 0.0).abs() < 1e-10);
        assert!((seq[1] - 2.5).abs() < 1e-10);
        assert!((seq[2] - 5.0).abs() < 1e-10);
        assert!((seq[3] - 7.5).abs() < 1e-10);
        assert!((seq[4] - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_linspace_edge_cases() {
        let empty: Vec<f64> = linspace(0.0, 10.0, 0);
        assert_eq!(empty.len(), 0);
        
        let single = linspace(0.0, 10.0, 1);
        assert_eq!(single.len(), 1);
        assert!((single[0] - 10.0).abs() < 1e-10);
        
        let two = linspace(0.0, 10.0, 2);
        assert_eq!(two.len(), 2);
        assert!((two[0] - 0.0).abs() < 1e-10);
        assert!((two[1] - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_sequence() {
        let seq = sequence(0.0, 10.0, 2.0);
        assert_eq!(seq, vec![0.0, 2.0, 4.0, 6.0, 8.0, 10.0]);
        
        let seq = sequence(0.0, 5.0, 1.0);
        assert_eq!(seq, vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0]);
    }

    #[test]
    fn test_sequence_partial() {
        // When last is not exactly reachable
        let seq = sequence(0.0, 10.1, 2.0);
        assert_eq!(seq, vec![0.0, 2.0, 4.0, 6.0, 8.0, 10.0]);
    }
}
