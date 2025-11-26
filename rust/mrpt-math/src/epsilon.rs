/// Geometric epsilon for floating-point comparisons.
///
/// This module provides functions to get and set the geometric epsilon value
/// used throughout MRPT for comparing floating-point numbers.

use std::sync::atomic::{AtomicU64, Ordering};

/// Default epsilon value (1e-5)
const DEFAULT_EPSILON: f64 = 1e-5;

/// Global epsilon storage using atomic operations for thread safety
static EPSILON: AtomicU64 = AtomicU64::new(DEFAULT_EPSILON.to_bits());

/// Gets the value of the geometric epsilon (default = 1e-5).
///
/// This value is used throughout MRPT for floating-point comparisons.
///
/// # Examples
///
/// ```
/// use mrpt_math::epsilon::get_epsilon;
///
/// let eps = get_epsilon();
/// assert!((eps - 1e-5).abs() < 1e-10);
/// ```
#[inline]
pub fn get_epsilon() -> f64 {
    let bits = EPSILON.load(Ordering::Relaxed);
    f64::from_bits(bits)
}

/// Changes the value of the geometric epsilon (default = 1e-5).
///
/// # Examples
///
/// ```
/// use mrpt_math::epsilon::{set_epsilon, get_epsilon};
///
/// set_epsilon(1e-6);
/// assert!((get_epsilon() - 1e-6).abs() < 1e-10);
///
/// // Reset to default
/// set_epsilon(1e-5);
/// ```
#[inline]
pub fn set_epsilon(new_epsilon: f64) {
    let bits = new_epsilon.to_bits();
    EPSILON.store(bits, Ordering::Relaxed);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_epsilon() {
        // Reset to default first
        set_epsilon(1e-5);
        let eps = get_epsilon();
        assert!((eps - 1e-5).abs() < 1e-10);
    }

    #[test]
    fn test_set_epsilon() {
        set_epsilon(1e-6);
        assert!((get_epsilon() - 1e-6).abs() < 1e-10);

        set_epsilon(1e-4);
        assert!((get_epsilon() - 1e-4).abs() < 1e-10);

        // Reset
        set_epsilon(1e-5);
    }

    #[test]
    fn test_epsilon_persistence() {
        let original = get_epsilon();
        set_epsilon(2.5e-7);
        assert!((get_epsilon() - 2.5e-7).abs() < 1e-10);
        
        // Reset
        set_epsilon(original);
    }
}
