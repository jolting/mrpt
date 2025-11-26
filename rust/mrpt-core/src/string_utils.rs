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

//! String manipulation utilities
//!
//! This module provides various string manipulation functions.

/// Convert string to lowercase
pub fn to_lower(s: &str) -> String {
    s.to_lowercase()
}

/// Convert string to uppercase
pub fn to_upper(s: &str) -> String {
    s.to_uppercase()
}

/// Trim whitespace from both ends
pub fn trim(s: &str) -> String {
    s.trim().to_string()
}

/// Trim whitespace from left
pub fn trim_left(s: &str) -> String {
    s.trim_start().to_string()
}

/// Trim whitespace from right
pub fn trim_right(s: &str) -> String {
    s.trim_end().to_string()
}

/// Tokenize string by delimiter
pub fn tokenize(s: &str, delim: &str) -> Vec<String> {
    s.split(delim)
        .map(|s| s.to_string())
        .collect()
}

/// Split string into lines
pub fn split_lines(s: &str) -> Vec<String> {
    s.lines()
        .map(|s| s.to_string())
        .collect()
}

/// Replace all occurrences of a pattern
pub fn replace_all(s: &str, from: &str, to: &str) -> String {
    s.replace(from, to)
}

/// Check if string starts with prefix
pub fn starts_with(s: &str, prefix: &str) -> bool {
    s.starts_with(prefix)
}

/// Check if string ends with suffix
pub fn ends_with(s: &str, suffix: &str) -> bool {
    s.ends_with(suffix)
}

/// Check if string contains substring
pub fn contains(s: &str, substring: &str) -> bool {
    s.contains(substring)
}

/// Parse a string into any type that implements FromStr
///
/// Returns the default value if parsing fails and throw_on_error is false.
/// Throws an error if parsing fails and throw_on_error is true.
///
/// # Examples
///
/// ```
/// use mrpt_core::string_utils::from_string;
///
/// let num: i32 = from_string("42", 0, false).unwrap();
/// assert_eq!(num, 42);
///
/// let invalid: i32 = from_string("not_a_number", 99, false).unwrap();
/// assert_eq!(invalid, 99); // Returns default value
/// ```
pub fn from_string<T>(s: &str, default_value: T, throw_on_error: bool) -> crate::exceptions::MrptResult<T>
where
    T: std::str::FromStr + Clone,
{
    match s.parse::<T>() {
        Ok(value) => Ok(value),
        Err(_) => {
            if throw_on_error {
                Err(crate::exceptions::MrptError::InvalidArgument(
                    format!("[from_string()] Cannot parse string: {}", s)
                ))
            } else {
                Ok(default_value)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_case_conversion() {
        assert_eq!(to_lower("HELLO"), "hello");
        assert_eq!(to_upper("hello"), "HELLO");
        assert_eq!(to_lower("MiXeD"), "mixed");
    }

    #[test]
    fn test_trim() {
        assert_eq!(trim("  hello  "), "hello");
        assert_eq!(trim_left("  hello  "), "hello  ");
        assert_eq!(trim_right("  hello  "), "  hello");
    }

    #[test]
    fn test_tokenize() {
        let tokens = tokenize("a,b,c", ",");
        assert_eq!(tokens, vec!["a", "b", "c"]);
        
        let tokens = tokenize("one two three", " ");
        assert_eq!(tokens, vec!["one", "two", "three"]);
    }

    #[test]
    fn test_split_lines() {
        let lines = split_lines("line1\nline2\nline3");
        assert_eq!(lines, vec!["line1", "line2", "line3"]);
    }

    #[test]
    fn test_replace_all() {
        assert_eq!(replace_all("hello world", "world", "rust"), "hello rust");
        assert_eq!(replace_all("aaa", "a", "b"), "bbb");
    }

    #[test]
    fn test_predicates() {
        assert!(starts_with("hello", "hel"));
        assert!(ends_with("hello", "llo"));
        assert!(contains("hello", "ell"));
        assert!(!starts_with("hello", "world"));
    }

    #[test]
    fn test_from_string() {
        // Test successful parsing
        let num: i32 = from_string("42", 0, false).unwrap();
        assert_eq!(num, 42);

        let float_val: f64 = from_string("3.14", 0.0, false).unwrap();
        assert!((float_val - 3.14).abs() < 1e-10);

        // Test with default value (throw_on_error = false)
        let invalid: i32 = from_string("not_a_number", 99, false).unwrap();
        assert_eq!(invalid, 99);

        // Test with error (throw_on_error = true)
        let result: Result<i32, _> = from_string("not_a_number", 0, true);
        assert!(result.is_err());

        // Test boolean parsing
        let bool_val: bool = from_string("true", false, false).unwrap();
        assert_eq!(bool_val, true);
    }
}
