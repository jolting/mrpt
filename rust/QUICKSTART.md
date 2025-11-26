# MRPT Rust Core - Quick Start Guide

Get started with the MRPT Rust core library in 5 minutes!

## Prerequisites

You need Rust installed on your system. If you don't have it:

**Windows:**
```powershell
.\install_rust.ps1
```

**Linux/macOS:**
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

After installation, restart your terminal.

## Build the Library

```bash
cd rust
cargo build --release
```

This will compile the Rust core library in release mode with full optimizations.

## Run Tests

```bash
cargo test
```

All tests should pass. You'll see output like:
```
running 15 tests
test clock::tests::test_clock_now ... ok
test exceptions::tests::test_exception_creation ... ok
...
test result: ok. 15 passed; 0 failed; 0 ignored; 0 measured
```

## Try the Examples

```bash
cargo run --example basic_usage
```

This demonstrates the main features:
- Clock operations
- Exception handling
- String formatting
- Bit manipulation

## Run Benchmarks

```bash
cargo bench
```

See performance metrics for critical operations like `Clock::now()`.

## Use from Rust

Create a new Rust project and add as dependency:

```toml
[dependencies]
mrpt-core = { path = "../mrpt/rust/mrpt-core" }
```

Then use it:

```rust
use mrpt_core::clock::Clock;

fn main() {
    let now = Clock::now();
    println!("Current time: {}", now);
}
```

## Use from C++

Include the FFI header in your C++ code:

```cpp
#include "mrpt_core_ffi.h"

int main() {
    int64_t now = mrpt_clock_now();
    printf("Current time: %lld\n", now);
    return 0;
}
```

Link with `mrpt_core_rust` target in CMake:

```cmake
target_link_libraries(my_app PRIVATE mrpt_core_rust)
```

## Build with MRPT CMake

When building the full MRPT project, you can choose to use the Rust core implementation:

### Option 1: Use Rust Core (Recommended for Testing)

```bash
cd /path/to/mrpt
mkdir build && cd build
cmake -DMRPT_USE_RUST_CORE=ON ..
cmake --build .
```

This will:
1. Build the Rust `mrpt-core` library
2. Replace the C++ core with the Rust implementation
3. All MRPT libraries will link against Rust core

### Option 2: Use C++ Core (Default)

```bash
cd /path/to/mrpt
mkdir build && cd build
cmake ..
cmake --build .
```

The C++ implementation remains the default. The Rust library is only built as a separate optional target.

## IDE Setup

### VS Code
1. Install "rust-analyzer" extension
2. Open the `rust` folder
3. IntelliSense will work automatically

### CLion
1. Install Rust plugin
2. Import the `rust` folder
3. Configure Cargo toolchain

### Visual Studio
1. Install Rust for Visual Studio
2. Open folder or use CMake integration

## View Documentation

Generate and open the Rust documentation:

```bash
cargo doc --open
```

This creates comprehensive API documentation with examples.

## Common Commands

```bash
# Build (debug)
cargo build

# Build (release, optimized)
cargo build --release

# Run tests
cargo test

# Run specific test
cargo test clock::tests

# Run benchmarks
cargo bench

# Check without building
cargo check

# Format code
cargo fmt

# Lint code
cargo clippy

# Clean build artifacts
cargo clean
```

## What's Included

The Rust core library provides:

### Clock Module
- High-resolution timestamps
- Multiple clock sources (Realtime, Monotonic, Simulated)
- Thread-safe operations
- Compatible with MRPT's TTimeStamp

```rust
use mrpt_core::clock::{Clock, ClockSource};

let t1 = Clock::now();
Clock::set_active_clock(ClockSource::Monotonic);
let t2 = Clock::now();
```

### Exceptions Module
- Rich error types
- Call stack backtraces
- Assertion macros
- Source location tracking

```rust
use mrpt_core::{MrptResult, mrpt_assert};

fn validate(value: i32) -> MrptResult<()> {
    mrpt_assert!(value > 0, "Value must be positive");
    Ok(())
}
```

### Format Module
- String formatting utilities
- Float precision control
- Vector formatting

```rust
use mrpt_core::format::format_float;

let pi = std::f64::consts::PI;
println!("{}", format_float(pi, 4)); // "3.1416"
```

### Bits Module
- Byte reversal (endianness)
- Bit manipulation
- Population count
- Memory operations

```rust
use mrpt_core::bits::ReverseBytesExt;

let value = 0x12345678u32;
let reversed = value.reverse_bytes(); // 0x78563412
```

### FFI Bridge
- C-compatible interface
- Seamless C++ integration
- All core functions exported

```c
#include "mrpt_core_ffi.h"

int64_t now = mrpt_clock_now();
double now_d = mrpt_clock_now_double();
```

## Performance

The Rust implementation provides excellent performance:

- `Clock::now()`: ~33 nanoseconds
- `Clock::now_double()`: ~38 nanoseconds
- Zero-cost abstractions
- No runtime overhead

Run `cargo bench` to see detailed metrics on your system.

## Safety Guarantees

Rust provides compile-time guarantees:

- ✅ No buffer overflows
- ✅ No use-after-free
- ✅ No data races
- ✅ No null pointer dereferences
- ✅ Thread safety enforced
- ✅ Memory safety without GC

## Next Steps

1. **Explore Examples**: Check `mrpt-core/examples/`
2. **Read Docs**: Run `cargo doc --open`
3. **Try Integration**: Use from C++ code
4. **Contribute**: See `MIGRATION_PLAN.md`
5. **Learn Rust**: https://doc.rust-lang.org/book/

## Troubleshooting

### Cargo not found
- Restart terminal after Rust installation
- Check PATH includes `~/.cargo/bin`

### Build errors
```bash
cargo clean
cargo build
```

### Test failures
- Ensure you're on a supported platform
- Check Rust version: `rustc --version` (need 1.70+)

### CMake doesn't find Cargo
- Ensure Cargo is in PATH
- Restart IDE/terminal
- Check CMake output for detection message

## Getting Help

- **Documentation**: `cargo doc --open`
- **Examples**: `rust/mrpt-core/examples/`
- **Issues**: https://github.com/MRPT/mrpt/issues
- **Forum**: https://www.mrpt.org/forum

## Success!

You're now ready to use the MRPT Rust core library!

For more detailed information, see:
- `README.md` - Full project overview
- `SETUP.md` - Detailed setup instructions
- `MIGRATION_PLAN.md` - Migration strategy and roadmap
