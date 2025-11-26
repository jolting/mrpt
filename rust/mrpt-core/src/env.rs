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

//! Environment variable utilities
//!
//! This module provides functions for reading environment variables.

use crate::string_utils::from_string;

/// Read an environment variable and parse it to the desired type
///
/// Returns the default value if the variable is not set or cannot be parsed.
///
/// # Examples
///
/// ```
/// use mrpt_core::env::get_env;
///
/// // Get a string environment variable
/// std::env::set_var("TEST_VAR", "hello");
/// let value: String = get_env("TEST_VAR", "default".to_string());
/// assert_eq!(value, "hello");
///
/// // Get an integer with default
/// let port: i32 = get_env("PORT", 8080);
/// ```
pub fn get_env<T>(varname: &str, default_value: T) -> T
where
    T: std::str::FromStr + Clone,
{
    match std::env::var(varname) {
        Ok(value) => from_string(&value, default_value.clone(), false).unwrap_or(default_value),
        Err(_) => default_value,
    }
}

/// Read a boolean environment variable
///
/// Understands "true", "True", "TRUE", and any non-zero number as true.
/// Returns false for "false", "False", "FALSE", "0", or empty strings.
///
/// # Examples
///
/// ```
/// use mrpt_core::env::get_env_bool;
///
/// std::env::set_var("DEBUG", "true");
/// assert_eq!(get_env_bool("DEBUG", false), true);
///
/// std::env::set_var("VERBOSE", "1");
/// assert_eq!(get_env_bool("VERBOSE", false), true);
///
/// std::env::set_var("QUIET", "0");
/// assert_eq!(get_env_bool("QUIET", true), false);
/// ```
pub fn get_env_bool(varname: &str, default_value: bool) -> bool {
    match std::env::var(varname) {
        Ok(value) => {
            let v = value.trim();
            
            // Check for explicit true values
            if v == "true" || v == "True" || v == "TRUE" {
                return true;
            }
            
            // Check for explicit false values
            if v == "false" || v == "False" || v == "FALSE" {
                return false;
            }
            
            // Try to parse as integer - non-zero is true
            if let Ok(num) = v.parse::<i32>() {
                return num != 0;
            }
            
            default_value
        }
        Err(_) => default_value,
    }
}

/// Check if an environment variable is set (regardless of its value)
///
/// # Examples
///
/// ```
/// use mrpt_core::env::env_var_exists;
///
/// std::env::set_var("MY_VAR", "value");
/// assert!(env_var_exists("MY_VAR"));
/// assert!(!env_var_exists("NONEXISTENT_VAR"));
/// ```
pub fn env_var_exists(varname: &str) -> bool {
    std::env::var(varname).is_ok()
}

/// Set an environment variable
///
/// # Examples
///
/// ```
/// use mrpt_core::env::set_env;
///
/// set_env("MY_VAR", "my_value");
/// assert_eq!(std::env::var("MY_VAR").unwrap(), "my_value");
/// ```
pub fn set_env(varname: &str, value: &str) {
    std::env::set_var(varname, value);
}

/// Remove an environment variable
///
/// # Examples
///
/// ```
/// use mrpt_core::env::{set_env, remove_env, env_var_exists};
///
/// set_env("TEMP_VAR", "value");
/// assert!(env_var_exists("TEMP_VAR"));
///
/// remove_env("TEMP_VAR");
/// assert!(!env_var_exists("TEMP_VAR"));
/// ```
pub fn remove_env(varname: &str) {
    std::env::remove_var(varname);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_env_string() {
        std::env::set_var("TEST_STRING", "hello_world");
        let value: String = get_env("TEST_STRING", "default".to_string());
        assert_eq!(value, "hello_world");
        std::env::remove_var("TEST_STRING");
    }

    #[test]
    fn test_get_env_default() {
        std::env::remove_var("NONEXISTENT_VAR");
        let value: String = get_env("NONEXISTENT_VAR", "default".to_string());
        assert_eq!(value, "default");
    }

    #[test]
    fn test_get_env_int() {
        std::env::set_var("TEST_PORT", "8080");
        let port: i32 = get_env("TEST_PORT", 3000);
        assert_eq!(port, 8080);
        std::env::remove_var("TEST_PORT");
    }

    #[test]
    fn test_get_env_int_invalid() {
        std::env::set_var("TEST_INVALID", "not_a_number");
        let value: i32 = get_env("TEST_INVALID", 42);
        assert_eq!(value, 42); // Should return default
        std::env::remove_var("TEST_INVALID");
    }

    #[test]
    fn test_get_env_bool_true() {
        std::env::set_var("TEST_BOOL_TRUE1", "true");
        assert_eq!(get_env_bool("TEST_BOOL_TRUE1", false), true);

        std::env::set_var("TEST_BOOL_TRUE2", "True");
        assert_eq!(get_env_bool("TEST_BOOL_TRUE2", false), true);

        std::env::set_var("TEST_BOOL_TRUE3", "TRUE");
        assert_eq!(get_env_bool("TEST_BOOL_TRUE3", false), true);

        std::env::set_var("TEST_BOOL_TRUE4", "1");
        assert_eq!(get_env_bool("TEST_BOOL_TRUE4", false), true);

        std::env::set_var("TEST_BOOL_TRUE5", "42");
        assert_eq!(get_env_bool("TEST_BOOL_TRUE5", false), true);

        std::env::remove_var("TEST_BOOL_TRUE1");
        std::env::remove_var("TEST_BOOL_TRUE2");
        std::env::remove_var("TEST_BOOL_TRUE3");
        std::env::remove_var("TEST_BOOL_TRUE4");
        std::env::remove_var("TEST_BOOL_TRUE5");
    }

    #[test]
    fn test_get_env_bool_false() {
        std::env::set_var("TEST_BOOL_FALSE1", "false");
        assert_eq!(get_env_bool("TEST_BOOL_FALSE1", true), false);

        std::env::set_var("TEST_BOOL_FALSE2", "False");
        assert_eq!(get_env_bool("TEST_BOOL_FALSE2", true), false);

        std::env::set_var("TEST_BOOL_FALSE3", "0");
        assert_eq!(get_env_bool("TEST_BOOL_FALSE3", true), false);

        std::env::remove_var("TEST_BOOL_FALSE1");
        std::env::remove_var("TEST_BOOL_FALSE2");
        std::env::remove_var("TEST_BOOL_FALSE3");
    }

    #[test]
    fn test_get_env_bool_default() {
        std::env::remove_var("TEST_BOOL_NONEXIST");
        assert_eq!(get_env_bool("TEST_BOOL_NONEXIST", true), true);
        assert_eq!(get_env_bool("TEST_BOOL_NONEXIST", false), false);
    }

    #[test]
    fn test_env_var_exists() {
        std::env::set_var("TEST_EXISTS", "value");
        assert!(env_var_exists("TEST_EXISTS"));
        assert!(!env_var_exists("TEST_NOT_EXISTS"));
        std::env::remove_var("TEST_EXISTS");
    }

    #[test]
    fn test_set_and_remove_env() {
        set_env("TEST_SET_REMOVE", "test_value");
        assert!(env_var_exists("TEST_SET_REMOVE"));
        assert_eq!(std::env::var("TEST_SET_REMOVE").unwrap(), "test_value");

        remove_env("TEST_SET_REMOVE");
        assert!(!env_var_exists("TEST_SET_REMOVE"));
    }

    #[test]
    fn test_get_env_float() {
        std::env::set_var("TEST_FLOAT", "3.14159");
        let value: f64 = get_env("TEST_FLOAT", 0.0);
        assert!((value - 3.14159).abs() < 1e-5);
        std::env::remove_var("TEST_FLOAT");
    }
}
