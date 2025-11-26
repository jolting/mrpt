/// Round and rounding utility functions.
///
/// This module provides optimized rounding functions similar to MRPT's `round.h`.

/// Returns the closer integer (i32) to a floating-point value.
///
/// # Examples
///
/// ```
/// use mrpt_core::round::round;
///
/// assert_eq!(round(3.7), 4);
/// assert_eq!(round(3.3), 3);
/// assert_eq!(round(-2.5), -2);
/// ```
#[inline]
pub fn round<T>(value: T) -> i32
where
    T: Into<f64>,
{
    let v: f64 = value.into();
    v.round() as i32
}

/// Returns the closer integer (i64) to a floating-point value.
///
/// # Examples
///
/// ```
/// use mrpt_core::round::round_long;
///
/// assert_eq!(round_long(3.7), 4);
/// assert_eq!(round_long(3.3), 3);
/// ```
#[inline]
pub fn round_long<T>(value: T) -> i64
where
    T: Into<f64>,
{
    let v: f64 = value.into();
    v.round() as i64
}

/// Round a decimal number up to the given 10th power.
///
/// This function rounds a value to a specific decimal place or power of 10:
/// - `power10 = 1` -> round to 10
/// - `power10 = 2` -> round to 100
/// - `power10 = -1` -> round to 0.1
/// - `power10 = -2` -> round to 0.01
///
/// # Examples
///
/// ```
/// use mrpt_core::round::round_10power;
///
/// // Round to nearest 10
/// assert_eq!(round_10power(37.8, 1), 40.0);
///
/// // Round to nearest 100
/// assert_eq!(round_10power(456.7, 2), 500.0);
///
/// // Round to one decimal place
/// assert!((round_10power(3.14159, -1) - 3.1).abs() < 1e-10);
///
/// // Round to two decimal places
/// assert!((round_10power(3.14159, -2) - 3.14).abs() < 1e-10);
/// ```
pub fn round_10power<T>(val: T, power10: i32) -> f64
where
    T: Into<f64>,
{
    let v: f64 = val.into();
    let f = 10.0_f64.powi(-power10);
    let t = round_long(v * f);
    t as f64 / f
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_round() {
        assert_eq!(round(3.7), 4);
        assert_eq!(round(3.3), 3);
        assert_eq!(round(3.5), 4);
        assert_eq!(round(-2.7), -3);
        assert_eq!(round(-2.3), -2);
        assert_eq!(round(0.0), 0);
    }

    #[test]
    fn test_round_long() {
        assert_eq!(round_long(3.7), 4);
        assert_eq!(round_long(3.3), 3);
        assert_eq!(round_long(123456.789), 123457);
        assert_eq!(round_long(-987.654), -988);
    }

    #[test]
    fn test_round_10power_positive() {
        // Round to nearest 10
        assert_eq!(round_10power(37.8, 1), 40.0);
        assert_eq!(round_10power(32.1, 1), 30.0);

        // Round to nearest 100
        assert_eq!(round_10power(456.7, 2), 500.0);
        assert_eq!(round_10power(449.0, 2), 400.0);

        // Round to nearest 1000
        assert_eq!(round_10power(1234.0, 3), 1000.0);
        assert_eq!(round_10power(1678.0, 3), 2000.0);
    }

    #[test]
    fn test_round_10power_negative() {
        // Round to one decimal place
        let result = round_10power(3.14159, -1);
        assert!((result - 3.1).abs() < 1e-10);

        let result = round_10power(3.17, -1);
        assert!((result - 3.2).abs() < 1e-10);

        // Round to two decimal places
        let result = round_10power(3.14159, -2);
        assert!((result - 3.14).abs() < 1e-10);

        let result = round_10power(3.14789, -2);
        assert!((result - 3.15).abs() < 1e-10);

        // Round to three decimal places
        let result = round_10power(3.14159265, -3);
        assert!((result - 3.142).abs() < 1e-10);
    }

    #[test]
    fn test_round_10power_zero() {
        // power10 = 0 means round to nearest unit
        assert_eq!(round_10power(3.7, 0), 4.0);
        assert_eq!(round_10power(3.3, 0), 3.0);
    }

    #[test]
    fn test_round_10power_negative_values() {
        let result = round_10power(-3.14159, -1);
        assert!((result + 3.1).abs() < 1e-10);

        let result = round_10power(-45.678, 1);
        assert!((result + 50.0).abs() < 1e-10);
    }
}
