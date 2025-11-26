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

//! Thread-safe queue implementation
//!
//! A thread-safe queue for passing messages between threads.

use parking_lot::Mutex;
use std::collections::VecDeque;

/// A thread-safe queue for passing messages between threads
///
/// This queue is designed for producer-consumer patterns where one or more
/// threads push items and one or more threads retrieve them. All operations
/// are protected by a mutex for thread safety.
///
/// # Examples
///
/// ```
/// use mrpt_core::containers::ThreadSafeQueue;
/// use std::thread;
///
/// let queue = ThreadSafeQueue::<i32>::new();
/// let queue_clone = queue.clone();
///
/// // Producer thread
/// thread::spawn(move || {
///     queue_clone.push(42);
/// });
///
/// // Consumer thread
/// if let Some(value) = queue.get() {
///     assert_eq!(value, 42);
/// }
/// ```
#[derive(Clone)]
pub struct ThreadSafeQueue<T> {
    queue: std::sync::Arc<Mutex<VecDeque<T>>>,
}

impl<T> ThreadSafeQueue<T> {
    /// Create a new empty thread-safe queue
    pub fn new() -> Self {
        Self {
            queue: std::sync::Arc::new(Mutex::new(VecDeque::new())),
        }
    }

    /// Create a new thread-safe queue with a specific capacity
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            queue: std::sync::Arc::new(Mutex::new(VecDeque::with_capacity(capacity))),
        }
    }

    /// Push a message into the queue
    ///
    /// This operation is thread-safe and will never block.
    pub fn push(&self, msg: T) {
        self.queue.lock().push_back(msg);
    }

    /// Retrieve the next message from the queue
    ///
    /// Returns `None` if the queue is empty, otherwise returns the oldest
    /// message in the queue.
    pub fn get(&self) -> Option<T> {
        self.queue.lock().pop_front()
    }

    /// Get the most recent message, discarding all older messages
    ///
    /// This is useful when you only care about the latest state and can
    /// skip intermediate updates. Returns `None` if the queue is empty.
    pub fn get_latest_purge_old(&self) -> Option<T> {
        let mut queue = self.queue.lock();
        queue.pop_back().map(|last| {
            queue.clear();
            last
        })
    }

    /// Check if the queue is empty
    pub fn is_empty(&self) -> bool {
        self.queue.lock().is_empty()
    }

    /// Get the number of messages in the queue
    pub fn size(&self) -> usize {
        self.queue.lock().len()
    }

    /// Clear all messages from the queue
    pub fn clear(&self) {
        self.queue.lock().clear();
    }

    /// Try to get a message without blocking
    ///
    /// This is an alias for `get()` for consistency with std library naming.
    pub fn try_pop(&self) -> Option<T> {
        self.get()
    }

    /// Push multiple messages at once
    pub fn push_many(&self, messages: impl IntoIterator<Item = T>) {
        let mut queue = self.queue.lock();
        queue.extend(messages);
    }

    /// Pop up to `count` messages from the queue
    ///
    /// Returns a vector with all available messages up to `count`.
    /// If the queue has fewer than `count` messages, returns all available messages.
    pub fn pop_many(&self, count: usize) -> Vec<T> {
        let mut queue = self.queue.lock();
        let actual_count = count.min(queue.len());
        queue.drain(..actual_count).collect()
    }

    /// Peek at the next message without removing it
    ///
    /// Returns a reference to the oldest message in the queue, or `None`
    /// if the queue is empty.
    ///
    /// Note: This holds the lock while you're examining the item, so be quick!
    pub fn peek<F, R>(&self, f: F) -> Option<R>
    where
        F: FnOnce(&T) -> R,
    {
        let queue = self.queue.lock();
        queue.front().map(f)
    }
}

impl<T> Default for ThreadSafeQueue<T> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn test_basic_push_get() {
        let queue = ThreadSafeQueue::new();
        queue.push(1);
        queue.push(2);
        queue.push(3);

        assert_eq!(queue.get(), Some(1));
        assert_eq!(queue.get(), Some(2));
        assert_eq!(queue.get(), Some(3));
        assert_eq!(queue.get(), None);
    }

    #[test]
    fn test_empty() {
        let queue = ThreadSafeQueue::<i32>::new();
        assert!(queue.is_empty());

        queue.push(1);
        assert!(!queue.is_empty());

        queue.get();
        assert!(queue.is_empty());
    }

    #[test]
    fn test_size() {
        let queue = ThreadSafeQueue::new();
        assert_eq!(queue.size(), 0);

        queue.push(1);
        queue.push(2);
        assert_eq!(queue.size(), 2);

        queue.get();
        assert_eq!(queue.size(), 1);
    }

    #[test]
    fn test_clear() {
        let queue = ThreadSafeQueue::new();
        queue.push(1);
        queue.push(2);
        queue.push(3);

        assert_eq!(queue.size(), 3);
        queue.clear();
        assert_eq!(queue.size(), 0);
        assert!(queue.is_empty());
    }

    #[test]
    fn test_get_latest_purge_old() {
        let queue = ThreadSafeQueue::new();
        queue.push(1);
        queue.push(2);
        queue.push(3);

        // Should get the last item (3) and discard the rest
        assert_eq!(queue.get_latest_purge_old(), Some(3));
        assert!(queue.is_empty());

        // Should return None on empty queue
        assert_eq!(queue.get_latest_purge_old(), None);
    }

    #[test]
    fn test_multithreaded_producers_consumers() {
        let queue = ThreadSafeQueue::new();
        let num_producers = 4;
        let num_items_per_producer = 100;

        // Spawn producer threads
        let mut handles = vec![];
        for producer_id in 0..num_producers {
            let queue_clone = queue.clone();
            let handle = thread::spawn(move || {
                for i in 0..num_items_per_producer {
                    queue_clone.push((producer_id, i));
                    thread::sleep(Duration::from_micros(1));
                }
            });
            handles.push(handle);
        }

        // Wait for all producers to finish
        for handle in handles {
            handle.join().unwrap();
        }

        // Verify all items were produced
        assert_eq!(queue.size(), num_producers * num_items_per_producer);

        // Consume all items
        let mut consumed = 0;
        while queue.get().is_some() {
            consumed += 1;
        }

        assert_eq!(consumed, num_producers * num_items_per_producer);
        assert!(queue.is_empty());
    }

    #[test]
    fn test_push_many() {
        let queue = ThreadSafeQueue::new();
        queue.push_many(vec![1, 2, 3, 4, 5]);

        assert_eq!(queue.size(), 5);
        assert_eq!(queue.get(), Some(1));
        assert_eq!(queue.get(), Some(2));
    }

    #[test]
    fn test_pop_many() {
        let queue = ThreadSafeQueue::new();
        queue.push_many(vec![1, 2, 3, 4, 5]);

        let items = queue.pop_many(3);
        assert_eq!(items, vec![1, 2, 3]);
        assert_eq!(queue.size(), 2);

        // Pop more than available
        let remaining = queue.pop_many(10);
        assert_eq!(remaining, vec![4, 5]);
        assert!(queue.is_empty());
    }

    #[test]
    fn test_peek() {
        let queue = ThreadSafeQueue::new();
        queue.push(42);

        let value = queue.peek(|v| *v);
        assert_eq!(value, Some(42));

        // Peek shouldn't remove the item
        assert_eq!(queue.size(), 1);
        assert_eq!(queue.get(), Some(42));
    }

    #[test]
    fn test_with_capacity() {
        let queue = ThreadSafeQueue::<i32>::with_capacity(100);
        assert!(queue.is_empty());
        assert_eq!(queue.size(), 0);
    }

    #[test]
    fn test_concurrent_push_pop() {
        let queue = Arc::new(ThreadSafeQueue::new());
        let queue_producer = Arc::clone(&queue);
        let queue_consumer = Arc::clone(&queue);

        let producer = thread::spawn(move || {
            for i in 0..1000 {
                queue_producer.push(i);
            }
        });

        let consumer = thread::spawn(move || {
            let mut count = 0;
            for _ in 0..1000 {
                while queue_consumer.get().is_none() {
                    thread::yield_now();
                }
                count += 1;
            }
            count
        });

        producer.join().unwrap();
        let consumed = consumer.join().unwrap();

        assert_eq!(consumed, 1000);
        assert!(queue.is_empty());
    }
}
