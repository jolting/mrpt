# MRPT Rust Setup Guide

This guide will help you set up the Rust toolchain to work with the MRPT Rust core library.

## Installing Rust

### Windows

1. Download and run rustup-init.exe from: https://rustup.rs/
2. Follow the on-screen instructions
3. Restart your terminal/PowerShell

Or use the PowerShell script in this directory:
```powershell
.\install_rust.ps1
```

### Linux/macOS

Run this command in your terminal:
```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Then restart your terminal or run:
```bash
source $HOME/.cargo/env
```

## Verifying Installation

After installation, verify Rust is properly installed:

```bash
cargo --version
rustc --version
```

You should see output like:
```
cargo 1.70.0 (...)
rustc 1.70.0 (...)
```

## Building the MRPT Rust Core Library

Once Rust is installed, you can build the library:

### Standalone Build

```bash
cd rust
cargo build --release
```

### Run Tests

```bash
cargo test
```

### Run Examples

```bash
cargo run --example basic_usage
```

### Run Benchmarks

```bash
cargo bench
```

### Build with CMake

When building the entire MRPT project, CMake will automatically detect Cargo and build the Rust library:

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

If Cargo is not found, MRPT will build without the Rust core library (using the C++ version instead).

## IDE Setup

### Visual Studio Code

Install these extensions for Rust development:
- rust-analyzer (provides IntelliSense, code completion, etc.)
- CodeLLDB (for debugging)
- crates (helps manage dependencies)

### CLion / IntelliJ IDEA

Install the Rust plugin from the marketplace.

### Visual Studio

Install the Rust for Visual Studio extension.

## Troubleshooting

### Cargo not found after installation

On Windows, you may need to restart your terminal or IDE after installing Rust.
You can also manually add Rust to your PATH:
- Default location: `%USERPROFILE%\.cargo\bin`

### Build errors

If you encounter build errors, try:
```bash
cargo clean
cargo build
```

### Linker errors on Linux

Install the development tools:
```bash
# Ubuntu/Debian
sudo apt-get install build-essential

# Fedora/RHEL
sudo dnf install gcc gcc-c++
```

### OpenSSL errors

Some dependencies may require OpenSSL development libraries:
```bash
# Ubuntu/Debian
sudo apt-get install libssl-dev pkg-config

# Fedora/RHEL
sudo dnf install openssl-devel
```

## Next Steps

After setting up Rust:

1. Build the library: `cargo build --release`
2. Run tests to verify: `cargo test`
3. Try the examples: `cargo run --example basic_usage`
4. Read the documentation: `cargo doc --open`
5. Start contributing!

## Resources

- Rust Book: https://doc.rust-lang.org/book/
- Rust by Example: https://doc.rust-lang.org/rust-by-example/
- Cargo Book: https://doc.rust-lang.org/cargo/
- MRPT Documentation: https://docs.mrpt.org/
