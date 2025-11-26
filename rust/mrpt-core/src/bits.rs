//                    _
//                   | |    Mobile Robot Programming Toolkit (MRPT)
// _ __ ___  _ __ _ __ | |_
//| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
//| | | | | | |  | |_) | |_
//|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
//               | |
//               |_|
//
// Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
// See: https://www.mrpt.org/Authors - All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

//! Bit manipulation utilities for MRPT
//!
//! This module provides low-level bit manipulation operations including byte reversal,
//! bit casting, and memory operations.

use std::mem;

/// Reverse the bytes of a value (endianness swap)
pub trait ReverseBytesExt {
    /// Reverse the byte order of this value
    fn reverse_bytes(self) -> Self;
}

macro_rules! impl_reverse_bytes {
    ($($t:ty),*) => {
        $(
            impl ReverseBytesExt for $t {
                #[inline]
                fn reverse_bytes(self) -> Self {
                    <$t>::swap_bytes(self)
                }
            }
        )*
    };
}

impl_reverse_bytes!(u16, u32, u64, u128, i16, i32, i64, i128);

impl ReverseBytesExt for f32 {
    #[inline]
    fn reverse_bytes(self) -> Self {
        f32::from_bits(self.to_bits().swap_bytes())
    }
}

impl ReverseBytesExt for f64 {
    #[inline]
    fn reverse_bytes(self) -> Self {
        f64::from_bits(self.to_bits().swap_bytes())
    }
}

/// Bit cast between types of the same size
///
/// # Safety
/// This is safe only when the bit pattern of `T` is valid as `U`
#[inline]
pub fn bit_cast<T, U>(value: T) -> U
where
    T: Copy,
    U: Copy,
{
    assert_eq!(mem::size_of::<T>(), mem::size_of::<U>());
    unsafe { mem::transmute_copy(&value) }
}

/// Extract bits from a value
#[inline]
pub fn extract_bits<T>(value: T, start_bit: u32, num_bits: u32) -> T
where
    T: Copy + std::ops::Shl<u32, Output = T> + std::ops::Shr<u32, Output = T> + std::ops::BitAnd<Output = T> + std::ops::Sub<Output = T> + From<u8>,
{
    let mask = (T::from(1u8) << num_bits) - T::from(1u8);
    (value >> start_bit) & mask
}

/// Set specific bits in a value
#[inline]
pub fn set_bits<T>(value: T, start_bit: u32, num_bits: u32, new_bits: T) -> T
where
    T: Copy
        + std::ops::Shl<u32, Output = T>
        + std::ops::Shr<u32, Output = T>
        + std::ops::BitAnd<Output = T>
        + std::ops::BitOr<Output = T>
        + std::ops::Not<Output = T>
        + std::ops::Sub<Output = T>
        + From<u8>,
{
    let mask = ((T::from(1u8) << num_bits) - T::from(1u8)) << start_bit;
    (value & !mask) | ((new_bits << start_bit) & mask)
}

/// Count the number of set bits (population count)
pub trait PopCount {
    /// Count the number of 1 bits
    fn pop_count(self) -> u32;
}

macro_rules! impl_pop_count {
    ($($t:ty),*) => {
        $(
            impl PopCount for $t {
                #[inline]
                fn pop_count(self) -> u32 {
                    self.count_ones()
                }
            }
        )*
    };
}

impl_pop_count!(u8, u16, u32, u64, u128, i8, i16, i32, i64, i128);

/// Round to the nearest power of 2 (upward)
#[inline]
pub fn round_up_to_power_of_2(mut n: u32) -> u32 {
    if n == 0 {
        return 1;
    }
    n -= 1;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n + 1
}

/// Check if a number is a power of 2
#[inline]
pub fn is_power_of_2(n: u32) -> bool {
    n != 0 && (n & (n - 1)) == 0
}

/// Compute absolute difference between two numbers
#[inline]
pub fn abs_diff<T>(a: T, b: T) -> T
where
    T: Copy + PartialOrd + std::ops::Sub<Output = T>,
{
    if a >= b {
        a - b
    } else {
        b - a
    }
}

/// Additional math utilities
pub mod math {
    use std::f64::consts::PI;

    /// Square of a number
    #[inline]
    pub fn square<T>(x: T) -> T
    where
        T: Copy + std::ops::Mul<Output = T>,
    {
        x * x
    }

    /// Fast hypot (no overflow checking)
    #[inline]
    pub fn hypot_fast<T>(x: T, y: T) -> T
    where
        T: Copy + std::ops::Mul<Output = T> + std::ops::Add<Output = T>,
        f64: From<T>,
        T: From<f64>,
    {
        let sum = x * x + y * y;
        let sqrt_val = f64::from(sum).sqrt();
        T::from(sqrt_val)
    }

    /// Convert degrees to radians
    #[inline]
    pub fn deg2rad(degrees: f64) -> f64 {
        degrees * PI / 180.0
    }

    /// Convert radians to degrees
    #[inline]
    pub fn rad2deg(radians: f64) -> f64 {
        radians * 180.0 / PI
    }

    /// Return the sign of a number (-1 or 1)
    #[inline]
    pub fn sign<T>(x: T) -> i32
    where
        T: Copy + PartialOrd + From<i32>,
    {
        if x < T::from(0) {
            -1
        } else {
            1
        }
    }

    /// Return the sign of a number (-1, 0, or 1)
    #[inline]
    pub fn sign_with_zero<T>(x: T) -> i32
    where
        T: Copy + PartialEq + PartialOrd + From<i32>,
    {
        if x == T::from(0) {
            0
        } else if x < T::from(0) {
            -1
        } else {
            1
        }
    }

    /// Return the smallest positive number among two values
    #[inline]
    pub fn lowest_positive<T>(a: T, b: T) -> T
    where
        T: Copy + PartialOrd + From<i32>,
    {
        let zero = T::from(0);
        if a > zero && a <= b {
            a
        } else if b > zero {
            b
        } else {
            a
        }
    }

    /// Return minimum of three values
    #[inline]
    pub fn min3<T>(a: T, b: T, c: T) -> T
    where
        T: Copy + PartialOrd,
    {
        if a < b {
            if a < c {
                a
            } else {
                c
            }
        } else if b < c {
            b
        } else {
            c
        }
    }

    /// Return maximum of three values
    #[inline]
    pub fn max3<T>(a: T, b: T, c: T) -> T
    where
        T: Copy + PartialOrd,
    {
        if a > b {
            if a > c {
                a
            } else {
                c
            }
        } else if b > c {
            b
        } else {
            c
        }
    }

    /// Round toward zero (truncate)
    #[inline]
    pub fn fix(x: f64) -> i32 {
        x.trunc() as i32
    }

    /// Clamp a mutable value to min/max range
    #[inline]
    pub fn saturate<T>(var: &mut T, sat_min: T, sat_max: T)
    where
        T: Copy + PartialOrd,
    {
        if *var > sat_max {
            *var = sat_max;
        }
        if *var < sat_min {
            *var = sat_min;
        }
    }

    /// Clamp a value to min/max range (returns value)
    #[inline]
    pub fn saturate_val<T>(value: T, sat_min: T, sat_max: T) -> T
    where
        T: Copy + PartialOrd,
    {
        if value > sat_max {
            sat_max
        } else if value < sat_min {
            sat_min
        } else {
            value
        }
    }

    /// Update a variable to keep it below or equal to a test value
    #[inline]
    pub fn keep_min<T>(var: &mut T, test_val: T)
    where
        T: Copy + PartialOrd,
    {
        if test_val < *var {
            *var = test_val;
        }
    }

    /// Update a variable to keep it above or equal to a test value
    #[inline]
    pub fn keep_max<T>(var: &mut T, test_val: T)
    where
        T: Copy + PartialOrd,
    {
        if test_val > *var {
            *var = test_val;
        }
    }
}

/// Low-level memory operations
pub mod mem_ops {
    use std::ptr;

    /// Copy memory from src to dst
    ///
    /// # Safety
    /// Both pointers must be valid and properly aligned
    #[inline]
    pub unsafe fn memcpy<T>(dst: *mut T, src: *const T, count: usize) {
        ptr::copy_nonoverlapping(src, dst, count);
    }

    /// Set memory to a specific byte value
    ///
    /// # Safety
    /// Pointer must be valid and properly aligned
    #[inline]
    pub unsafe fn memset<T>(dst: *mut T, value: u8, count: usize) {
        ptr::write_bytes(dst, value, count);
    }

    /// Compare two memory regions
    ///
    /// # Safety
    /// Both pointers must be valid and properly aligned
    #[inline]
    pub unsafe fn memcmp<T>(a: *const T, b: *const T, count: usize) -> i32 {
        let a_slice = std::slice::from_raw_parts(a as *const u8, count * std::mem::size_of::<T>());
        let b_slice = std::slice::from_raw_parts(b as *const u8, count * std::mem::size_of::<T>());
        
        for i in 0..a_slice.len() {
            match a_slice[i].cmp(&b_slice[i]) {
                std::cmp::Ordering::Less => return -1,
                std::cmp::Ordering::Greater => return 1,
                std::cmp::Ordering::Equal => continue,
            }
        }
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reverse_bytes() {
        assert_eq!(0x1234u16.reverse_bytes(), 0x3412u16);
        assert_eq!(0x12345678u32.reverse_bytes(), 0x78563412u32);
    }

    #[test]
    fn test_bit_cast() {
        let f: f32 = 1.0;
        let u: u32 = bit_cast(f);
        assert_eq!(u, 0x3F800000);
        
        let f2: f32 = bit_cast(u);
        assert_eq!(f, f2);
    }

    #[test]
    fn test_extract_bits() {
        let value = 0b11010110u8;
        assert_eq!(extract_bits(value, 2, 3), 0b101u8);
    }

    #[test]
    fn test_set_bits() {
        let value = 0b11010110u8;
        let result = set_bits(value, 2, 3, 0b011u8);
        assert_eq!(result, 0b11001110u8);
    }

    #[test]
    fn test_pop_count() {
        assert_eq!(0b11010110u8.pop_count(), 5);
        assert_eq!(0b00000000u8.pop_count(), 0);
        assert_eq!(0b11111111u8.pop_count(), 8);
    }

    #[test]
    fn test_round_up_to_power_of_2() {
        assert_eq!(round_up_to_power_of_2(0), 1);
        assert_eq!(round_up_to_power_of_2(1), 1);
        assert_eq!(round_up_to_power_of_2(5), 8);
        assert_eq!(round_up_to_power_of_2(16), 16);
        assert_eq!(round_up_to_power_of_2(17), 32);
    }

    #[test]
    fn test_is_power_of_2() {
        assert!(!is_power_of_2(0));
        assert!(is_power_of_2(1));
        assert!(is_power_of_2(2));
        assert!(!is_power_of_2(3));
        assert!(is_power_of_2(4));
        assert!(is_power_of_2(16));
        assert!(!is_power_of_2(17));
    }

    #[test]
    fn test_abs_diff() {
        assert_eq!(abs_diff(10, 5), 5);
        assert_eq!(abs_diff(5, 10), 5);
        assert_eq!(abs_diff(10, 10), 0);
    }

    // Math utilities tests
    use super::math::*;

    #[test]
    fn test_square() {
        assert_eq!(square(5), 25);
        assert_eq!(square(-3), 9);
        assert!((square(2.5_f64) - 6.25).abs() < 1e-10);
    }

    #[test]
    fn test_deg2rad() {
        assert!((deg2rad(0.0) - 0.0).abs() < 1e-10);
        assert!((deg2rad(180.0) - std::f64::consts::PI).abs() < 1e-10);
        assert!((deg2rad(90.0) - std::f64::consts::PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_rad2deg() {
        assert!((rad2deg(0.0) - 0.0).abs() < 1e-10);
        assert!((rad2deg(std::f64::consts::PI) - 180.0).abs() < 1e-10);
        assert!((rad2deg(std::f64::consts::PI / 2.0) - 90.0).abs() < 1e-10);
    }

    #[test]
    fn test_sign() {
        assert_eq!(sign(5), 1);
        assert_eq!(sign(-5), -1);
        assert_eq!(sign(0), 1); // sign of 0 is 1 in MRPT
    }

    #[test]
    fn test_sign_with_zero() {
        assert_eq!(sign_with_zero(5), 1);
        assert_eq!(sign_with_zero(-5), -1);
        assert_eq!(sign_with_zero(0), 0);
    }

    #[test]
    fn test_lowest_positive() {
        assert_eq!(lowest_positive(5, 10), 5);
        assert_eq!(lowest_positive(10, 5), 5);
        assert_eq!(lowest_positive(-5, 10), 10);
        assert_eq!(lowest_positive(5, -10), 5);
    }

    #[test]
    fn test_min3() {
        assert_eq!(min3(1, 2, 3), 1);
        assert_eq!(min3(3, 1, 2), 1);
        assert_eq!(min3(2, 3, 1), 1);
    }

    #[test]
    fn test_max3() {
        assert_eq!(max3(1, 2, 3), 3);
        assert_eq!(max3(3, 1, 2), 3);
        assert_eq!(max3(2, 3, 1), 3);
    }

    #[test]
    fn test_fix() {
        assert_eq!(fix(3.7), 3);
        assert_eq!(fix(-3.7), -3);
        assert_eq!(fix(0.0), 0);
    }

    #[test]
    fn test_saturate() {
        let mut val = 15;
        saturate(&mut val, 0, 10);
        assert_eq!(val, 10);

        let mut val = -5;
        saturate(&mut val, 0, 10);
        assert_eq!(val, 0);

        let mut val = 5;
        saturate(&mut val, 0, 10);
        assert_eq!(val, 5);
    }

    #[test]
    fn test_saturate_val() {
        assert_eq!(saturate_val(15, 0, 10), 10);
        assert_eq!(saturate_val(-5, 0, 10), 0);
        assert_eq!(saturate_val(5, 0, 10), 5);
    }

    #[test]
    fn test_keep_min() {
        let mut val = 10;
        keep_min(&mut val, 5);
        assert_eq!(val, 5);

        let mut val = 10;
        keep_min(&mut val, 15);
        assert_eq!(val, 10);
    }

    #[test]
    fn test_keep_max() {
        let mut val = 10;
        keep_max(&mut val, 15);
        assert_eq!(val, 15);

        let mut val = 10;
        keep_max(&mut val, 5);
        assert_eq!(val, 10);
    }
}
