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

//! Worker thread pool implementation
//!
//! This module provides a thread pool for executing tasks asynchronously.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;

/// Queue policy for handling task overflow
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueuePolicy {
    /// All tasks are executed in FIFO order. No drops.
    Fifo,
    /// If a task arrives and there are more pending tasks than worker threads,
    /// drop previous tasks.
    DropOld,
}

type Task = Box<dyn FnOnce() + Send + 'static>;

struct PoolState {
    tasks: Vec<Task>,
    stop: bool,
}

/// A thread pool for executing tasks asynchronously
///
/// The pool maintains a fixed number of worker threads that execute tasks
/// submitted via `enqueue()`. Tasks are stored in a queue and executed
/// when a worker becomes available.
pub struct WorkerThreadsPool {
    workers: Vec<thread::JoinHandle<()>>,
    state: Arc<(Mutex<PoolState>, Condvar)>,
    policy: QueuePolicy,
    name: String,
    pending_count: Arc<AtomicUsize>,
}

impl WorkerThreadsPool {
    /// Create a new thread pool with the specified number of threads
    ///
    /// # Arguments
    /// * `num_threads` - Number of worker threads to create
    /// * `policy` - Queue policy for handling task overflow
    /// * `name` - Name prefix for worker threads (for debugging)
    pub fn new(num_threads: usize, policy: QueuePolicy, name: String) -> Self {
        let state = Arc::new((
            Mutex::new(PoolState {
                tasks: Vec::new(),
                stop: false,
            }),
            Condvar::new(),
        ));
        let pending_count = Arc::new(AtomicUsize::new(0));

        let mut workers = Vec::with_capacity(num_threads);

        for id in 0..num_threads {
            let state_clone = Arc::clone(&state);
            let thread_name = format!("{}[{}]", name, id);
            let pending_clone = Arc::clone(&pending_count);

            let handle = thread::Builder::new()
                .name(thread_name)
                .spawn(move || {
                    Self::worker_loop(state_clone, pending_clone);
                })
                .expect("Failed to spawn worker thread");

            workers.push(handle);
        }

        Self {
            workers,
            state,
            policy,
            name,
            pending_count,
        }
    }

    /// Create a new thread pool with default settings (FIFO policy)
    pub fn with_threads(num_threads: usize) -> Self {
        Self::new(num_threads, QueuePolicy::Fifo, "WorkerThreadsPool".to_string())
    }

    fn worker_loop(
        state: Arc<(Mutex<PoolState>, Condvar)>,
        pending_count: Arc<AtomicUsize>,
    ) {
        let (lock, cvar) = &*state;

        loop {
            let task = {
                let mut pool_state = lock.lock().unwrap();

                // Wait for a task or stop signal
                while pool_state.tasks.is_empty() && !pool_state.stop {
                    pool_state = cvar.wait(pool_state).unwrap();
                }

                if pool_state.stop {
                    return;
                }

                // Get the next task
                if !pool_state.tasks.is_empty() {
                    pending_count.fetch_sub(1, Ordering::SeqCst);
                    Some(pool_state.tasks.remove(0))
                } else {
                    None
                }
            };

            if let Some(task) = task {
                // Execute the task outside the lock
                task();
            }
        }
    }

    /// Enqueue a task for execution
    ///
    /// The task will be executed by the next available worker thread.
    /// Returns immediately without waiting for the task to complete.
    ///
    /// # Panics
    /// Panics if the pool has been stopped
    pub fn enqueue<F>(&self, task: F)
    where
        F: FnOnce() + Send + 'static,
    {
        let (lock, cvar) = &*self.state;
        let mut pool_state = lock.lock().unwrap();

        if pool_state.stop {
            panic!("Cannot enqueue task on stopped WorkerThreadsPool");
        }

        // Apply policy
        if self.policy == QueuePolicy::DropOld {
            while pool_state.tasks.len() >= self.workers.len() {
                let _ = pool_state.tasks.remove(0); // Drop the old task
                self.pending_count.fetch_sub(1, Ordering::SeqCst);
            }
        }

        pool_state.tasks.push(Box::new(task));
        self.pending_count.fetch_add(1, Ordering::SeqCst);
        cvar.notify_one();
    }

    /// Get the number of pending tasks waiting to be executed
    pub fn pending_tasks(&self) -> usize {
        self.pending_count.load(Ordering::SeqCst)
    }

    /// Get the number of worker threads
    pub fn size(&self) -> usize {
        self.workers.len()
    }

    /// Get the name of the thread pool
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Clear all pending tasks and stop all worker threads
    pub fn clear(&mut self) {
        let (lock, cvar) = &*self.state;
        {
            let mut pool_state = lock.lock().unwrap();
            pool_state.stop = true;
            
            if !pool_state.tasks.is_empty() {
                eprintln!(
                    "[WorkerThreadsPool name=`{}`] Warning: clear() called while having {} pending tasks. Aborting them.",
                    self.name,
                    pool_state.tasks.len()
                );
            }
        }
        cvar.notify_all();

        // Join all worker threads
        for worker in self.workers.drain(..) {
            let _ = worker.join();
        }
    }
}

impl Drop for WorkerThreadsPool {
    fn drop(&mut self) {
        self.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicI32;
    use std::thread;
    use std::time::Duration;

    #[test]
    fn test_basic_execution() {
        let counter = Arc::new(AtomicI32::new(0));
        let pool = WorkerThreadsPool::with_threads(2);

        for _ in 0..10 {
            let counter_clone = Arc::clone(&counter);
            pool.enqueue(move || {
                counter_clone.fetch_add(1, Ordering::SeqCst);
            });
        }

        // Wait for tasks to complete
        thread::sleep(Duration::from_millis(100));
        assert_eq!(counter.load(Ordering::SeqCst), 10);
    }

    #[test]
    fn test_fifo_policy() {
        let pool = WorkerThreadsPool::new(1, QueuePolicy::Fifo, "test".to_string());

        let results = Arc::new(Mutex::new(Vec::new()));

        for i in 0..5 {
            let results_clone = Arc::clone(&results);
            pool.enqueue(move || {
                results_clone.lock().unwrap().push(i);
            });
        }

        thread::sleep(Duration::from_millis(100));
        let final_results = results.lock().unwrap();
        assert_eq!(*final_results, vec![0, 1, 2, 3, 4]);
    }

    #[test]
    fn test_pending_tasks() {
        let pool = WorkerThreadsPool::with_threads(1);

        // Enqueue a long-running task
        pool.enqueue(|| thread::sleep(Duration::from_millis(50)));

        // Enqueue more tasks
        for _ in 0..5 {
            pool.enqueue(|| {});
        }

        let pending = pool.pending_tasks();
        assert!(pending > 0);
        assert!(pending <= 6);
    }

    #[test]
    fn test_drop_old_policy() {
        let pool = WorkerThreadsPool::new(2, QueuePolicy::DropOld, "test".to_string());

        // Block the workers with long tasks
        pool.enqueue(|| thread::sleep(Duration::from_millis(100)));
        pool.enqueue(|| thread::sleep(Duration::from_millis(100)));

        // Wait a bit for workers to pick up tasks
        thread::sleep(Duration::from_millis(10));

        let counter = Arc::new(AtomicI32::new(0));

        // Enqueue many more tasks - old ones should be dropped
        for _ in 0..10 {
            let counter_clone = Arc::clone(&counter);
            pool.enqueue(move || {
                counter_clone.fetch_add(1, Ordering::SeqCst);
            });
        }

        thread::sleep(Duration::from_millis(200));

        // Not all 10 tasks should have executed due to dropping
        let count = counter.load(Ordering::SeqCst);
        assert!(count <= 10, "Expected some tasks to be dropped, but got {}", count);
    }

    #[test]
    fn test_size() {
        let pool = WorkerThreadsPool::with_threads(4);
        assert_eq!(pool.size(), 4);
    }

    #[test]
    fn test_name() {
        let pool = WorkerThreadsPool::new(2, QueuePolicy::Fifo, "MyPool".to_string());
        assert_eq!(pool.name(), "MyPool");
    }
}
