# CMake Toggle for Rust Core Implementation

## Overview

MRPT now supports switching between C++ and Rust implementations of the core library using a CMake option.

## Usage

### Enable Rust Core

```bash
cmake -DMRPT_USE_RUST_CORE=ON ..
```

### Disable Rust Core (Default)

```bash
cmake -DMRPT_USE_RUST_CORE=OFF ..
```

## How It Works

### When `MRPT_USE_RUST_CORE=ON`

1. **Rust Build**: CMake invokes Cargo to build `rust/mrpt-core`
2. **Library Alias**: Creates `core` as an alias to `mrpt_core_rust`
3. **Dependency Replacement**: All MRPT libraries that depend on `mrpt-core` now link against the Rust version
4. **Installation**: Rust library and FFI header are installed with MRPT

### When `MRPT_USE_RUST_CORE=OFF` (Default)

1. **C++ Build**: Traditional C++ `mrpt-core` is built
2. **No Rust**: Rust code is not compiled
3. **Normal Behavior**: Everything works as before

## Architecture

```
MRPT_USE_RUST_CORE=ON:
┌─────────────────────┐
│  mrpt-system        │
│  mrpt-containers    │
│  other libs...      │
└──────────┬──────────┘
           │ links to
           ▼
    ┌──────────────┐
    │  core (alias)│
    └──────┬───────┘
           │
           ▼
    ┌──────────────┐
    │mrpt_core_rust│  ← Built from Rust
    └──────────────┘

MRPT_USE_RUST_CORE=OFF:
┌─────────────────────┐
│  mrpt-system        │
│  mrpt-containers    │
│  other libs...      │
└──────────┬──────────┘
           │ links to
           ▼
    ┌──────────────┐
    │     core     │  ← Built from C++
    └──────────────┘
```

## Benefits

### 1. Incremental Migration
- Test Rust implementation without breaking existing code
- Switch back to C++ if issues arise
- Gradual confidence building

### 2. Performance Testing
- Easy A/B comparison between implementations
- Benchmark both versions with identical workloads
- Validate performance claims

### 3. CI/CD Integration
- Test both implementations in parallel
- Catch regressions in either version
- Ensure API compatibility

### 4. User Choice
- Users can opt-in to Rust benefits
- Conservative users stay with C++
- Flexible deployment options

## Requirements

### For Rust Core (`MRPT_USE_RUST_CORE=ON`)

- **Rust**: Version 1.70 or later
- **Cargo**: Rust's package manager
- **System Libraries**: Same as C++ version plus Rust runtime dependencies

Installation:
```bash
# Windows
.\rust\install_rust.ps1

# Linux/macOS
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### For C++ Core (`MRPT_USE_RUST_CORE=OFF`)

- Standard C++17 compiler
- CMake 3.16+
- No Rust required

## Implementation Details

### Files Modified

1. **`CMakeLists.txt`** (root)
   - Added `MRPT_USE_RUST_CORE` option
   - Conditionally includes `rust/` subdirectory

2. **`libs/core/CMakeLists.txt`**
   - Checks `MRPT_USE_RUST_CORE` flag
   - Creates alias or builds C++ version
   - Handles installation for both cases

3. **`rust/CMakeLists.txt`**
   - Builds Rust library via Cargo
   - Creates imported target `mrpt_core_rust`
   - Links platform-specific system libraries

### CMake Targets

| Target | Description |
|--------|-------------|
| `core` | Main target (alias to Rust or C++ depending on option) |
| `mrpt_core_rust` | Imported target for Rust library (when enabled) |
| `mrpt_core_rust_build` | Custom target to trigger Cargo build |

### Installation Locations

**With Rust Core:**
- Library: `${CMAKE_INSTALL_LIBDIR}/libmrpt_core.a` (or `.lib` on Windows)
- Header: `${CMAKE_INSTALL_INCLUDEDIR}/mrpt/core/mrpt_core_ffi.h`

**With C++ Core:**
- Library: Standard MRPT installation paths
- Headers: Standard MRPT header locations

## Testing

### Manual Test

```bash
# Test C++
mkdir build-cpp && cd build-cpp
cmake -DMRPT_USE_RUST_CORE=OFF ..
cmake --build . --target core

# Test Rust
cd ..
mkdir build-rust && cd build-rust
cmake -DMRPT_USE_RUST_CORE=ON ..
cmake --build . --target core
```

### Automated Test

See `TESTING_CMAKE_TOGGLE.md` for comprehensive testing procedures.

## Troubleshooting

### Error: "Cargo not found"

**Cause**: `MRPT_USE_RUST_CORE=ON` but Rust is not installed.

**Solution**:
```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source $HOME/.cargo/env

# Or disable Rust core
cmake -DMRPT_USE_RUST_CORE=OFF ..
```

### Error: Linker errors with Rust core

**Cause**: Missing platform-specific system libraries.

**Solution**: Rust CMakeLists.txt should handle this automatically. If not:
- Windows: Install Windows SDK
- Linux: Install build-essential
- macOS: Install Xcode Command Line Tools

### Cache Issues

If switching between implementations causes problems:

```bash
# Clean build directory
rm -rf build
mkdir build && cd build
cmake -DMRPT_USE_RUST_CORE=<ON|OFF> ..
```

## Future Plans

### Short Term
- Document API compatibility between versions
- Add integration tests
- Performance benchmarking suite

### Medium Term
- Make Rust the default (`MRPT_USE_RUST_CORE=ON` by default)
- Deprecation warnings for C++ core

### Long Term
- Remove C++ core implementation
- Rust becomes the only implementation

## Current Status

- **Phase**: Optional/Experimental
- **Default**: C++ implementation (`OFF`)
- **Stability**: Rust implementation passes all tests
- **Recommendation**: Test with `ON` in development environments

## Examples

### Example 1: Development with Rust

```bash
cd mrpt
mkdir build && cd build
cmake -DMRPT_USE_RUST_CORE=ON -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
ctest  # Run tests
```

### Example 2: Production with C++

```bash
cd mrpt
mkdir build && cd build
cmake -DMRPT_USE_RUST_CORE=OFF -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
sudo cmake --install .
```

### Example 3: CI/CD Matrix

```yaml
strategy:
  matrix:
    core_impl: [cpp, rust]
    
steps:
  - name: Configure
    run: |
      USE_RUST=${{ matrix.core_impl == 'rust' && 'ON' || 'OFF' }}
      cmake -DMRPT_USE_RUST_CORE=$USE_RUST ..
      
  - name: Build
    run: cmake --build .
    
  - name: Test
    run: ctest --output-on-failure
```

## Documentation

- **Quick Start**: See `QUICKSTART.md`
- **Testing Guide**: See `TESTING_CMAKE_TOGGLE.md`
- **Migration Plan**: See `MIGRATION_PLAN.md`
- **Full Documentation**: See `README.md`

## Support

For issues or questions:
- GitHub Issues: https://github.com/MRPT/mrpt/issues
- Forum: https://www.mrpt.org/forum
- Documentation: https://docs.mrpt.org/

## License

Both implementations (C++ and Rust) are released under the BSD License.
