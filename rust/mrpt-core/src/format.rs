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

//! String formatting utilities for MRPT
//!
//! This module provides sprintf-like formatting functionality compatible with the C++ implementation.

use std::fmt;

/// Format a string using printf-style format specifiers (limited Rust implementation)
///
/// Note: This is a simplified version. For full printf compatibility, consider using
/// the `printf` crate or similar. This implementation uses Rust's formatting instead.
///
/// # Examples
///
/// ```
/// use mrpt_core::format_string;
///
/// let result = format_string!("Hello {}", "World");
/// assert_eq!(result, "Hello World");
/// ```
#[macro_export]
macro_rules! format_string {
    ($($arg:tt)*) => {
        format!($($arg)*)
    };
}

/// Extension trait for formatting values
pub trait Formattable {
    /// Format this value as a string
    fn to_formatted_string(&self) -> String;
}

impl<T: fmt::Display> Formattable for T {
    fn to_formatted_string(&self) -> String {
        format!("{}", self)
    }
}

/// Format with specific precision for floating point numbers
pub fn format_float(value: f64, precision: usize) -> String {
    format!("{:.prec$}", value, prec = precision)
}

/// Format a vector of values
pub fn format_vec<T: fmt::Display>(values: &[T], separator: &str) -> String {
    values
        .iter()
        .map(|v| format!("{}", v))
        .collect::<Vec<_>>()
        .join(separator)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_float() {
        let result = format_float(3.14159265, 2);
        assert_eq!(result, "3.14");

        let result = format_float(3.14159265, 4);
        assert_eq!(result, "3.1416");
    }

    #[test]
    fn test_format_vec() {
        let values = vec![1, 2, 3, 4, 5];
        let result = format_vec(&values, ", ");
        assert_eq!(result, "1, 2, 3, 4, 5");
    }

    #[test]
    fn test_format_string_macro() {
        let result = format_string!("Value: {}", 42);
        assert_eq!(result, "Value: 42");
    }
}
