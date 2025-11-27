# MRPT Math Library - Rust FFI Integration Guide

## Overview

The mrpt-math Rust library now provides a complete C/C++ FFI (Foreign Function Interface) bridge, allowing the MRPT C++ codebase to call Rust implementations of mathematical operations.

## Architecture

```
┌─────────────────────┐
│  C++ MRPT Types     │
│  (TPoint3D, etc.)   │
└──────────┬──────────┘
           │
           │ uses
           ▼
┌─────────────────────┐
│  C++ Adapter Layer  │
│  (conversions)      │
└──────────┬──────────┘
           │
           │ calls
           ▼
┌─────────────────────┐
│  C FFI Layer        │
│  (extern "C")       │
└──────────┬──────────┘
           │
           │ invokes
           ▼
┌─────────────────────┐
│  Rust Library       │
│  (mrpt-math)        │
└─────────────────────┘
```

## Building

### Build Rust Library with FFI

```bash
cd rust/mrpt-math
cargo build --features ffi --release
```

This produces a static library: `target/release/libmrpt_math.a` (or `.lib` on Windows)

### Build C++ Test Program

The CMakeLists.txt in `rust/mrpt-math/` provides integration:

```bash
# From MRPT build directory
cmake -DMRPT_USE_RUST_MATH=ON ..
cmake --build . --target test-rust-math
./test-rust-math
```

## Using in C++ Code

### 1. Include Headers

```cpp
#include "mrpt_math_rust_adapter.h"
```

### 2. Convert Types

Use the `mrpt::math::rust` namespace for conversions:

```cpp
using namespace mrpt::math;
using namespace mrpt::math::rust;

// C++ to Rust
TPoint3D cpp_point{1.0, 2.0, 3.0};
TPoint3D_Rust rust_point = toRust(cpp_point);

// Rust to C++
TPoint3D back_to_cpp = fromRust(rust_point);
```

### 3. Call Rust Functions

#### Direct FFI Calls

```cpp
#include "mrpt_math_rust.h"

TPoint3D_Rust p1 = mrpt_math_point3d_new(1.0, 0.0, 0.0);
TPoint3D_Rust p2 = mrpt_math_point3d_new(0.0, 1.0, 0.0);

double distance = mrpt_math_point3d_distance(&p1, &p2);
```

#### Using Adapter Wrappers

```cpp
#include "mrpt_math_rust_adapter.h"

using namespace mrpt::math::rust;

TPoint3D p1{1.0, 0.0, 0.0};
TPoint3D p2{0.0, 1.0, 0.0};

// Calls Rust implementation
double dist = point3d_distance_rust(p1, p2);
```

## Available Functions

### Point Operations

- **Point2D**: `new`, `distance`, `norm`, `dot`
- **Point3D**: `new`, `distance`, `norm`, `dot`, `cross`

### Pose Operations

- **Pose2D**: `new`, `compose`, `inverse`, `distance`
- **Pose3D**: `new`, `distance`

### Line Operations

- **Line2D**: `new`, `from_two_points`, `distance`, `signed_distance`, `contains`, `unitarize`
- **Line3D**: `new`, `from_two_points`, `distance`, `contains`, `closest_point`, `unitarize`

### Plane Operations

- **Plane**: `new`, `from_three_points`, `from_point_and_normal`, `distance`, `signed_distance`, `contains`, `contains_line`, `normal_vector`, `unitarize`

### Utility Functions

- **Epsilon**: `get_epsilon`, `set_epsilon`

## Example: Complete Integration

```cpp
#include "mrpt_math_rust_adapter.h"
#include <iostream>

using namespace mrpt::math;
using namespace mrpt::math::rust;

void compute_plane_distance()
{
    // Create XY plane using Rust
    TPoint3D_Rust p1_r = mrpt_math_point3d_new(0.0, 0.0, 0.0);
    TPoint3D_Rust p2_r = mrpt_math_point3d_new(1.0, 0.0, 0.0);
    TPoint3D_Rust p3_r = mrpt_math_point3d_new(0.0, 1.0, 0.0);
    
    TPlane_Rust plane_r;
    mrpt_math_plane_from_three_points(&p1_r, &p2_r, &p3_r, &plane_r);
    
    // Test point above plane
    TPoint3D_Rust test_point = mrpt_math_point3d_new(5.0, 5.0, 3.0);
    
    double distance = mrpt_math_plane_distance(&plane_r, &test_point);
    std::cout << "Distance to plane: " << distance << std::endl; // 3.0
}

// Or using C++ types with adapter
void compute_with_cpp_types()
{
    TPlane plane{0.0, 0.0, 1.0, 0.0};  // XY plane
    TPoint3D point{5.0, 5.0, 3.0};
    
    double dist = plane_distance_rust(plane, point);
    std::cout << "Distance: " << dist << std::endl; // 3.0
}
```

## Integration into MRPT C++ Modules

### Option 1: Runtime Toggle

Allow users to choose implementation at runtime:

```cpp
namespace mrpt::math
{
enum class Implementation { CPP, RUST };

Implementation g_implementation = Implementation::CPP;

double TPoint3D::distanceTo(const TPoint3D& other) const
{
    if (g_implementation == Implementation::RUST)
        return rust::point3d_distance_rust(*this, other);
    else
        return /* existing C++ implementation */;
}
}
```

### Option 2: Compile-Time Toggle

Use CMake options to switch implementations:

```cpp
#ifdef MRPT_USE_RUST_MATH
    return mrpt::math::rust::point3d_distance_rust(*this, other);
#else
    // Original C++ implementation
#endif
```

### Option 3: Separate Namespace

Keep both implementations available:

```cpp
// C++ implementation
namespace mrpt::math { /* ... */ }

// Rust implementation accessible via
namespace mrpt::math::rust { /* ... */ }
```

## Performance Considerations

- **FFI Overhead**: Minimal for computational functions (nanoseconds)
- **Memory Layout**: `#[repr(C)]` ensures zero-cost type conversion
- **Optimization**: Rust release builds enable full optimizations
- **Inlining**: LTO (Link Time Optimization) can inline across FFI boundary

## Testing

Run the C++ test suite:

```bash
cd rust/mrpt-math
cmake -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build
```

Or run the test executable directly:

```bash
./build/test-rust-math
```

Expected output:
```
========================================
MRPT Rust Math Library FFI Test
========================================

Testing Point2D...
  Distance: 5 (expected 5.0)
  ✓ Point2D tests passed
...
========================================
All tests passed! ✓
========================================
```

## Troubleshooting

### Linking Errors

**Windows**: Ensure `ws2_32.lib`, `userenv.lib`, `bcrypt.lib`, `ntdll.lib` are linked  
**Linux**: Ensure `pthread` and `dl` are linked  
**macOS**: Ensure `Security` and `CoreFoundation` frameworks are linked

### Type Mismatches

Ensure Rust struct definitions match C header exactly:
- Same field order
- Same data types (f64 = double)
- `#[repr(C)]` on all FFI-exposed structs

### Feature Flag

Always build with `--features ffi` to enable the FFI module:

```bash
cargo build --features ffi
```

## Future Enhancements

- [ ] Add more geometric types (segments, polygons)
- [ ] Matrix operations via FFI
- [ ] Quaternion operations
- [ ] Transformation pipelines
- [ ] SIMD-accelerated variants
- [ ] GPU computation backends

## License

BSD-3-Clause (same as MRPT)
