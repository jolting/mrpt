// Mobile Robot Programming Toolkit (MRPT)
// https://www.mrpt.org/
//
// Copyright (c) 2005-2024, Individual contributors, see AUTHORS file
// See: https://www.mrpt.org/Authors - All rights reserved.
// Released under BSD License. See: https://www.mrpt.org/License

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
}
