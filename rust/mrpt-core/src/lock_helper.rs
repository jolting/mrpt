/// RAII lock helper utilities.
///
/// This module provides RAII-based lock management similar to MRPT's `lock_helper.h`.

/// A RAII guard that automatically unlocks a mutex when dropped.
///
/// This is similar to `std::sync::MutexGuard` but works with any type
/// that has `lock()` and `unlock()` methods.
///
/// # Examples
///
/// ```
/// use std::sync::Mutex;
/// use mrpt_core::lock_helper::LockHelper;
///
/// let mutex = Mutex::new(42);
/// {
///     let guard = LockHelper::new(&mutex);
///     // Mutex is locked here
/// } // Mutex is automatically unlocked when guard goes out of scope
/// ```
pub struct LockHelper<'a, T: Lockable> {
    lock: Option<&'a T>,
}

impl<'a, T> LockHelper<'a, T>
where
    T: Lockable,
{
    /// Creates a new lock helper that immediately locks the given mutex.
    pub fn new(lock: &'a T) -> Self {
        lock.lock();
        Self { lock: Some(lock) }
    }

    /// Manually unlocks the mutex before the helper is dropped.
    ///
    /// This can be useful when you need to release the lock early.
    pub fn unlock(&mut self) {
        if let Some(lock) = self.lock.take() {
            lock.unlock();
        }
    }
}

impl<'a, T> Drop for LockHelper<'a, T>
where
    T: Lockable,
{
    fn drop(&mut self) {
        if let Some(lock) = self.lock.take() {
            lock.unlock();
        }
    }
}

/// Trait for types that can be locked and unlocked.
///
/// This trait is implemented for standard Rust mutexes.
pub trait Lockable {
    /// Lock the mutex.
    fn lock(&self);
    
    /// Unlock the mutex.
    fn unlock(&self);
}

/// Convenience function to create a lock helper.
///
/// This provides syntactic sugar similar to C++'s `lockHelper()`.
///
/// # Examples
///
/// ```
/// use std::sync::Mutex;
/// use mrpt_core::lock_helper::lock_helper;
///
/// let mutex = Mutex::new(42);
/// {
///     let _guard = lock_helper(&mutex);
///     // Mutex is locked here
/// } // Automatically unlocked
/// ```
#[inline]
pub fn lock_helper<T: Lockable>(lock: &T) -> LockHelper<'_, T> {
    LockHelper::new(lock)
}

// Note: We don't implement Lockable for std::sync::Mutex directly because
// Mutex::lock() returns a Result<MutexGuard, PoisonError> and doesn't have
// a separate unlock() method. Instead, users should use parking_lot::Mutex
// or implement their own Lockable types.

#[cfg(feature = "parking_lot")]
mod parking_lot_impl {
    use super::*;
    use parking_lot::Mutex;

    impl<T> Lockable for Mutex<T> {
        fn lock(&self) {
            let _ = self.lock();
        }

        fn unlock(&self) {
            unsafe {
                self.force_unlock();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicBool, Ordering};

    // Test implementation of Lockable for testing purposes
    struct TestLock {
        locked: AtomicBool,
    }

    impl TestLock {
        fn new() -> Self {
            Self {
                locked: AtomicBool::new(false),
            }
        }

        fn is_locked(&self) -> bool {
            self.locked.load(Ordering::SeqCst)
        }
    }

    impl Lockable for TestLock {
        fn lock(&self) {
            assert!(!self.locked.load(Ordering::SeqCst), "Already locked");
            self.locked.store(true, Ordering::SeqCst);
        }

        fn unlock(&self) {
            assert!(self.locked.load(Ordering::SeqCst), "Not locked");
            self.locked.store(false, Ordering::SeqCst);
        }
    }

    #[test]
    fn test_lock_helper_auto_unlock() {
        let lock = TestLock::new();
        assert!(!lock.is_locked());

        {
            let _guard = LockHelper::new(&lock);
            assert!(lock.is_locked());
        }

        assert!(!lock.is_locked());
    }

    #[test]
    fn test_lock_helper_manual_unlock() {
        let lock = TestLock::new();
        assert!(!lock.is_locked());

        {
            let mut guard = LockHelper::new(&lock);
            assert!(lock.is_locked());

            guard.unlock();
            assert!(!lock.is_locked());
        }

        assert!(!lock.is_locked());
    }

    #[test]
    fn test_lock_helper_function() {
        let lock = TestLock::new();
        assert!(!lock.is_locked());

        {
            let _guard = lock_helper(&lock);
            assert!(lock.is_locked());
        }

        assert!(!lock.is_locked());
    }

    #[test]
    fn test_multiple_unlock_is_safe() {
        let lock = TestLock::new();

        {
            let mut guard = LockHelper::new(&lock);
            assert!(lock.is_locked());

            guard.unlock();
            assert!(!lock.is_locked());

            // Second unlock should be a no-op
            guard.unlock();
            assert!(!lock.is_locked());
        }

        assert!(!lock.is_locked());
    }
}
