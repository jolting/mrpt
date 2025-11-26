# Testing CMake Toggle for Rust Core

This document describes how to test the CMake toggle between C++ and Rust implementations.

## Quick Test

### Test 1: Default (C++ Core)

```bash
cd /path/to/mrpt
mkdir build-cpp && cd build-cpp
cmake ..
cmake --build . --target core
```

Expected: C++ implementation of core is built.

### Test 2: Rust Core Enabled

```bash
cd /path/to/mrpt
mkdir build-rust && cd build-rust
cmake -DMRPT_USE_RUST_CORE=ON ..
cmake --build . --target mrpt_core_rust
```

Expected: Rust implementation is built and aliased as 'core'.

### Test 3: Rust Core Without Cargo (Should Fail)

```bash
cd /path/to/mrpt
mkdir build-test && cd build-test
# Temporarily hide cargo from PATH
cmake -DMRPT_USE_RUST_CORE=ON ..
```

Expected: CMake error message about missing Cargo.

## Verification

After building with Rust core enabled, verify:

1. **Library exists**:
   ```bash
   # Windows
   ls rust/target/release/mrpt_core.lib
   
   # Linux/Mac
   ls rust/target/release/libmrpt_core.a
   ```

2. **CMake reports Rust usage**:
   Look for this in CMake output:
   ```
   -- Using Rust implementation for mrpt-core
   -- Rust core library integration enabled
   ```

3. **FFI header is available**:
   ```bash
   ls rust/mrpt-core/include/mrpt_core_ffi.h
   ```

## Integration Testing

To test that dependent libraries work with Rust core:

```bash
cd build-rust
cmake --build . --target mrpt-system
cmake --build . --target mrpt-containers
```

Both should link successfully against the Rust core.

## Performance Comparison

Build both versions and compare:

```bash
# C++ version
cd build-cpp
cmake --build . --target core
time ./path/to/benchmark

# Rust version  
cd build-rust
cmake --build . --target mrpt_core_rust
time ./path/to/benchmark
```

## Troubleshooting

### "Cargo not found" error

Solution: Install Rust from https://rustup.rs/ and ensure `cargo` is in PATH.

### Linker errors with Rust core

Check that platform-specific system libraries are linked:
- Windows: ws2_32, userenv, advapi32, bcrypt, ntdll
- Linux: pthread, dl, m
- macOS: pthread, dl, m, Security framework, CoreFoundation framework

### CMake cache issues

If switching between C++ and Rust implementations causes problems:

```bash
rm -rf build
mkdir build && cd build
cmake -DMRPT_USE_RUST_CORE=ON ..  # or OFF
```

## CI/CD Integration

Add to your CI pipeline:

```yaml
# Test C++ version
- name: Build with C++ core
  run: |
    mkdir build-cpp && cd build-cpp
    cmake ..
    cmake --build .

# Test Rust version (if Rust is available)
- name: Build with Rust core
  run: |
    mkdir build-rust && cd build-rust
    cmake -DMRPT_USE_RUST_CORE=ON ..
    cmake --build .
```

## Migration Path

Recommended migration steps:

1. **Phase 1**: Test with `-DMRPT_USE_RUST_CORE=ON` in development
2. **Phase 2**: Enable in CI/CD for automated testing
3. **Phase 3**: Make Rust the default (change CMakeLists.txt option default)
4. **Phase 4**: Eventually deprecate C++ core implementation

Current status: **Phase 1** (Optional Rust implementation)
