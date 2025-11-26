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

//! Circular buffer implementation
//!
//! A fixed-size circular buffer with efficient wraparound behavior.

use crate::exceptions::{MrptError, MrptResult};

/// A circular buffer of fixed size
///
/// This is a FIFO buffer that wraps around when reaching capacity.
/// Elements are stored in a contiguous vector, with read and write
/// indices tracking the current positions.
///
/// # Examples
///
/// ```
/// use mrpt_core::containers::CircularBuffer;
///
/// let mut buffer = CircularBuffer::<i32>::new(5).unwrap();
/// buffer.push(1).unwrap();
/// buffer.push(2).unwrap();
/// assert_eq!(buffer.pop().unwrap(), 1);
/// assert_eq!(buffer.size(), 1);
/// ```
pub struct CircularBuffer<T> {
    data: Vec<T>,
    size: usize,
    next_read: usize,
    next_write: usize,
}

impl<T: Clone + Default> CircularBuffer<T> {
    /// Create a new circular buffer with the specified capacity
    ///
    /// # Arguments
    /// * `size` - Maximum number of elements the buffer can hold. Must be > 2.
    ///
    /// # Errors
    /// Returns an error if size <= 2
    pub fn new(size: usize) -> MrptResult<Self> {
        if size <= 2 {
            return Err(MrptError::InvalidArgument("size must be > 2".to_string()));
        }

        Ok(Self {
            data: vec![T::default(); size],
            size,
            next_read: 0,
            next_write: 0,
        })
    }

    /// Insert an element into the buffer
    ///
    /// # Errors
    /// Returns an error if the buffer is full
    pub fn push(&mut self, value: T) -> MrptResult<()> {
        let new_idx = if self.next_write + 1 == self.size {
            0
        } else {
            self.next_write + 1
        };

        if new_idx == self.next_read {
            return Err(MrptError::OutOfRange("push: circular_buffer is full".to_string()));
        }

        self.data[self.next_write] = value;
        self.next_write = new_idx;
        Ok(())
    }

    /// Insert multiple elements into the buffer
    ///
    /// # Errors
    /// Returns an error if the buffer runs out of space
    pub fn push_many(&mut self, elements: &[T]) -> MrptResult<()> {
        for elem in elements {
            self.push(elem.clone())?;
        }
        Ok(())
    }

    /// Remove and return the oldest element from the buffer
    ///
    /// # Errors
    /// Returns an error if the buffer is empty
    pub fn pop(&mut self) -> MrptResult<T> {
        if self.next_read == self.next_write {
            return Err(MrptError::OutOfRange("pop: circular_buffer is empty".to_string()));
        }

        let i = self.next_read;
        self.next_read += 1;
        if self.next_read == self.size {
            self.next_read = 0;
        }

        Ok(self.data[i].clone())
    }

    /// Remove multiple elements from the buffer
    ///
    /// # Errors
    /// Returns an error if the buffer has fewer elements than requested
    pub fn pop_many(&mut self, count: usize) -> MrptResult<Vec<T>> {
        let mut result = Vec::with_capacity(count);
        for _ in 0..count {
            result.push(self.pop()?);
        }
        Ok(result)
    }

    /// Peek at the next element without removing it
    ///
    /// # Errors
    /// Returns an error if the buffer is empty
    pub fn peek(&self) -> MrptResult<&T> {
        if self.next_read == self.next_write {
            return Err(MrptError::OutOfRange("peek: circular_buffer is empty".to_string()));
        }
        Ok(&self.data[self.next_read])
    }

    /// Peek at an element at a specific index ahead
    ///
    /// Index 0 means the immediate next element, index 1 the following one, etc.
    ///
    /// # Errors
    /// Returns an error if the index is beyond available elements
    pub fn peek_at(&self, index: usize) -> MrptResult<&T> {
        if index >= self.size() {
            return Err(MrptError::OutOfRange("peek: seek out of range".to_string()));
        }
        let actual_index = (self.next_read + index) % self.size;
        Ok(&self.data[actual_index])
    }

    /// Peek at multiple elements without removing them
    ///
    /// # Errors
    /// Returns an error if the buffer has fewer elements than requested
    pub fn peek_many(&self, count: usize) -> MrptResult<Vec<T>> {
        let mut result = Vec::with_capacity(count);
        let mut peek_read = self.next_read;

        for _ in 0..count {
            if peek_read == self.next_write {
                return Err(MrptError::OutOfRange("peek: circular_buffer is empty".to_string()));
            }

            result.push(self.data[peek_read].clone());
            peek_read += 1;
            if peek_read == self.size {
                peek_read = 0;
            }
        }

        Ok(result)
    }

    /// Return the number of elements available for reading
    pub fn size(&self) -> usize {
        if self.next_write >= self.next_read {
            self.next_write - self.next_read
        } else {
            self.next_write + (self.size - self.next_read)
        }
    }

    /// Return the maximum capacity of the buffer
    pub fn capacity(&self) -> usize {
        self.size
    }

    /// Return the number of elements that can be written without overflow
    pub fn available(&self) -> usize {
        self.capacity() - self.size() - 1
    }

    /// Check if the buffer is empty
    pub fn is_empty(&self) -> bool {
        self.next_read == self.next_write
    }

    /// Check if the buffer is full
    pub fn is_full(&self) -> bool {
        let new_idx = if self.next_write + 1 == self.size {
            0
        } else {
            self.next_write + 1
        };
        new_idx == self.next_read
    }

    /// Clear all elements from the buffer
    pub fn clear(&mut self) {
        self.next_write = 0;
        self.next_read = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_create() {
        let buffer = CircularBuffer::<i32>::new(5);
        assert!(buffer.is_ok());

        let buffer = CircularBuffer::<i32>::new(2);
        assert!(buffer.is_err());
    }

    #[test]
    fn test_push_pop() {
        let mut buffer = CircularBuffer::<i32>::new(5).unwrap();
        assert!(buffer.is_empty());

        buffer.push(1).unwrap();
        buffer.push(2).unwrap();
        buffer.push(3).unwrap();

        assert_eq!(buffer.size(), 3);
        assert_eq!(buffer.pop().unwrap(), 1);
        assert_eq!(buffer.pop().unwrap(), 2);
        assert_eq!(buffer.pop().unwrap(), 3);
        assert!(buffer.is_empty());
    }

    #[test]
    fn test_wraparound() {
        let mut buffer = CircularBuffer::<i32>::new(4).unwrap();

        // Fill to near capacity
        buffer.push(1).unwrap();
        buffer.push(2).unwrap();

        // Pop one
        assert_eq!(buffer.pop().unwrap(), 1);

        // Push more to wrap around
        buffer.push(3).unwrap();
        buffer.push(4).unwrap();

        assert_eq!(buffer.size(), 3);
        assert_eq!(buffer.pop().unwrap(), 2);
        assert_eq!(buffer.pop().unwrap(), 3);
        assert_eq!(buffer.pop().unwrap(), 4);
    }

    #[test]
    fn test_overflow() {
        let mut buffer = CircularBuffer::<i32>::new(4).unwrap();

        buffer.push(1).unwrap();
        buffer.push(2).unwrap();
        buffer.push(3).unwrap();

        // Should fail (capacity is size-1)
        let result = buffer.push(4);
        assert!(result.is_err());
    }

    #[test]
    fn test_underflow() {
        let mut buffer = CircularBuffer::<i32>::new(4).unwrap();
        let result = buffer.pop();
        assert!(result.is_err());
    }

    #[test]
    fn test_peek() {
        let mut buffer = CircularBuffer::<i32>::new(5).unwrap();
        buffer.push(1).unwrap();
        buffer.push(2).unwrap();
        buffer.push(3).unwrap();

        assert_eq!(*buffer.peek().unwrap(), 1);
        assert_eq!(*buffer.peek_at(0).unwrap(), 1);
        assert_eq!(*buffer.peek_at(1).unwrap(), 2);
        assert_eq!(*buffer.peek_at(2).unwrap(), 3);

        // Peek should not remove elements
        assert_eq!(buffer.size(), 3);
    }

    #[test]
    fn test_peek_many() {
        let mut buffer = CircularBuffer::<i32>::new(5).unwrap();
        buffer.push(1).unwrap();
        buffer.push(2).unwrap();
        buffer.push(3).unwrap();

        let peeked = buffer.peek_many(2).unwrap();
        assert_eq!(peeked, vec![1, 2]);

        // Should not have removed elements
        assert_eq!(buffer.size(), 3);
    }

    #[test]
    fn test_push_pop_many() {
        let mut buffer = CircularBuffer::<i32>::new(10).unwrap();

        buffer.push_many(&[1, 2, 3, 4, 5]).unwrap();
        assert_eq!(buffer.size(), 5);

        let popped = buffer.pop_many(3).unwrap();
        assert_eq!(popped, vec![1, 2, 3]);
        assert_eq!(buffer.size(), 2);
    }

    #[test]
    fn test_capacity_available() {
        let mut buffer = CircularBuffer::<i32>::new(5).unwrap();
        assert_eq!(buffer.capacity(), 5);
        assert_eq!(buffer.available(), 4); // capacity - size - 1

        buffer.push(1).unwrap();
        assert_eq!(buffer.available(), 3);

        buffer.push(2).unwrap();
        assert_eq!(buffer.available(), 2);
    }

    #[test]
    fn test_clear() {
        let mut buffer = CircularBuffer::<i32>::new(5).unwrap();
        buffer.push(1).unwrap();
        buffer.push(2).unwrap();
        buffer.push(3).unwrap();

        assert_eq!(buffer.size(), 3);
        buffer.clear();
        assert_eq!(buffer.size(), 0);
        assert!(buffer.is_empty());
    }

    #[test]
    fn test_is_full() {
        let mut buffer = CircularBuffer::<i32>::new(4).unwrap();
        assert!(!buffer.is_full());

        buffer.push(1).unwrap();
        buffer.push(2).unwrap();
        buffer.push(3).unwrap();

        assert!(buffer.is_full());
    }
}
