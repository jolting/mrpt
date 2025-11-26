// Mobile Robot Programming Toolkit (MRPT)
// https://www.mrpt.org/
//
// Copyright (c) 2005-2024, Individual contributors, see AUTHORS file
// See: https://www.mrpt.org/Authors - All rights reserved.
// Released under BSD License. See: https://www.mrpt.org/License

//! Exception handling and error types for MRPT
//!
//! This module provides error types and exception handling compatible with MRPT's C++ implementation.
//! It includes call stack backtrace support and detailed error reporting.

use backtrace::Backtrace;
use thiserror::Error;

/// Maximum depth for exception call stack backtraces
pub const MAX_BACKTRACE_DEPTH: usize = 20;

/// Result type alias for MRPT operations
pub type MrptResult<T> = Result<T, MrptError>;

/// Main error type for MRPT operations
#[derive(Debug, Error)]
pub enum MrptError {
    /// Generic MRPT exception with message and optional backtrace
    #[error("MRPT Exception: {message}\n{location}")]
    Exception {
        /// Error message
        message: String,
        /// Source location (file:line)
        location: String,
        /// Function name where error occurred
        function: String,
        /// Call stack backtrace
        backtrace: Option<Backtrace>,
    },

    /// Invalid argument error
    #[error("Invalid argument: {0}")]
    InvalidArgument(String),

    /// Out of range error
    #[error("Out of range: {0}")]
    OutOfRange(String),

    /// IO error
    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    /// Logic error
    #[error("Logic error: {0}")]
    LogicError(String),

    /// Not implemented error
    #[error("Not implemented: {0}")]
    NotImplemented(String),

    /// Assertion failed error
    #[error("Assertion failed: {condition}\n{details}")]
    AssertionFailed {
        /// Condition that failed
        condition: String,
        /// Additional details
        details: String,
    },
}

impl MrptError {
    /// Create a new exception with full context
    pub fn new_exception(
        message: impl Into<String>,
        file: &str,
        line: u32,
        function: &str,
        with_backtrace: bool,
    ) -> Self {
        MrptError::Exception {
            message: message.into(),
            location: format!("{}:{}", file, line),
            function: function.to_string(),
            backtrace: if with_backtrace {
                Some(Backtrace::new())
            } else {
                None
            },
        }
    }

    /// Create an assertion failure error
    pub fn assertion_failed(condition: impl Into<String>, details: impl Into<String>) -> Self {
        MrptError::AssertionFailed {
            condition: condition.into(),
            details: details.into(),
        }
    }

    /// Get the backtrace if available
    pub fn backtrace(&self) -> Option<&Backtrace> {
        match self {
            MrptError::Exception { backtrace, .. } => backtrace.as_ref(),
            _ => None,
        }
    }

    /// Format error with full context including backtrace
    pub fn format_detailed(&self) -> String {
        let mut output = format!("{}", self);
        
        if let Some(bt) = self.backtrace() {
            output.push_str("\n\nCall stack backtrace:\n");
            output.push_str(&format!("{:?}", bt));
        }
        
        output
    }
}

/// Macro to throw an MRPT exception with context
#[macro_export]
macro_rules! mrpt_throw {
    ($msg:expr) => {
        return Err($crate::exceptions::MrptError::new_exception(
            $msg,
            file!(),
            line!(),
            std::any::type_name::<fn()>(),
            true,
        ))
    };
}

/// Macro to assert a condition and throw if false
#[macro_export]
macro_rules! mrpt_assert {
    ($cond:expr) => {
        if !$cond {
            return Err($crate::exceptions::MrptError::new_exception(
                format!("Assertion failed: {}", stringify!($cond)),
                file!(),
                line!(),
                std::any::type_name::<fn()>(),
                true,
            ));
        }
    };
    ($cond:expr, $msg:expr) => {
        if !$cond {
            return Err($crate::exceptions::MrptError::new_exception(
                format!("Assertion failed: {} - {}", stringify!($cond), $msg),
                file!(),
                line!(),
                std::any::type_name::<fn()>(),
                true,
            ));
        }
    };
}

/// Macro to assert equality
#[macro_export]
macro_rules! mrpt_assert_eq {
    ($left:expr, $right:expr) => {
        {
            let left_val = &$left;
            let right_val = &$right;
            if !(left_val == right_val) {
                return Err($crate::exceptions::MrptError::assertion_failed(
                    format!("{} == {}", stringify!($left), stringify!($right)),
                    format!("{:?} != {:?}", left_val, right_val),
                ));
            }
        }
    };
}

/// Macro to assert not equal
#[macro_export]
macro_rules! mrpt_assert_ne {
    ($left:expr, $right:expr) => {
        {
            let left_val = &$left;
            let right_val = &$right;
            if !(left_val != right_val) {
                return Err($crate::exceptions::MrptError::assertion_failed(
                    format!("{} != {}", stringify!($left), stringify!($right)),
                    format!("{:?} == {:?}", left_val, right_val),
                ));
            }
        }
    };
}

/// Macro to assert less than
#[macro_export]
macro_rules! mrpt_assert_lt {
    ($left:expr, $right:expr) => {
        {
            let left_val = &$left;
            let right_val = &$right;
            if !(left_val < right_val) {
                return Err($crate::exceptions::MrptError::assertion_failed(
                    format!("{} < {}", stringify!($left), stringify!($right)),
                    format!("{:?} >= {:?}", left_val, right_val),
                ));
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_exception_creation() {
        let err = MrptError::new_exception("Test error", "test.rs", 42, "test_fn", false);
        let msg = format!("{}", err);
        assert!(msg.contains("Test error"));
        assert!(msg.contains("test.rs:42"));
    }

    #[test]
    fn test_assertion_failed() {
        let err = MrptError::assertion_failed("x == y", "5 != 10");
        let msg = format!("{}", err);
        assert!(msg.contains("x == y"));
        assert!(msg.contains("5 != 10"));
    }

    fn test_throw_function() -> MrptResult<()> {
        mrpt_throw!("This is a test error");
    }

    #[test]
    fn test_mrpt_throw_macro() {
        let result = test_throw_function();
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(format!("{}", err).contains("This is a test error"));
    }

    fn test_assert_function(value: i32) -> MrptResult<()> {
        mrpt_assert!(value > 0, "Value must be positive");
        Ok(())
    }

    #[test]
    fn test_mrpt_assert_macro() {
        assert!(test_assert_function(5).is_ok());
        assert!(test_assert_function(-5).is_err());
    }

    fn test_assert_eq_function(a: i32, b: i32) -> MrptResult<()> {
        mrpt_assert_eq!(a, b);
        Ok(())
    }

    #[test]
    fn test_mrpt_assert_eq_macro() {
        assert!(test_assert_eq_function(5, 5).is_ok());
        assert!(test_assert_eq_function(5, 10).is_err());
    }
}
