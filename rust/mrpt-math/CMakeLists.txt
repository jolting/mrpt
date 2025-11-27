# Build Rust mrpt-math library
set(MRPT_MATH_RUST_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(MRPT_MATH_RUST_LIB "${MRPT_MATH_RUST_DIR}/target/$<IF:$<CONFIG:Debug>,debug,release>/mrpt_math${CMAKE_STATIC_LIBRARY_SUFFIX}")

# Find Cargo
find_program(CARGO_EXECUTABLE cargo REQUIRED)

# Determine build type flag
set(CARGO_BUILD_TYPE_FLAG "")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CARGO_BUILD_TYPE_FLAG "--release")
endif()

# Custom command to build the Rust library
add_custom_command(
    OUTPUT ${MRPT_MATH_RUST_LIB}
    COMMAND ${CARGO_EXECUTABLE} build ${CARGO_BUILD_TYPE_FLAG} --features ffi
    WORKING_DIRECTORY ${MRPT_MATH_RUST_DIR}
    COMMENT "Building mrpt-math Rust library with FFI support"
    VERBATIM
)

# Custom target to trigger Rust build
add_custom_target(mrpt_math_rust_build ALL
    DEPENDS ${MRPT_MATH_RUST_LIB}
)

# Create an imported library that C++ can link against
add_library(mrpt_math_rust STATIC IMPORTED GLOBAL)
set_target_properties(mrpt_math_rust PROPERTIES
    IMPORTED_LOCATION ${MRPT_MATH_RUST_LIB}
)

# Make sure the Rust library is built before trying to import it
add_dependencies(mrpt_math_rust mrpt_math_rust_build)

# Export the include directory for the C header
target_include_directories(mrpt_math_rust INTERFACE
    ${MRPT_MATH_RUST_DIR}/include
)

# Platform-specific linking requirements
if(WIN32)
    # Windows needs additional system libraries
    target_link_libraries(mrpt_math_rust INTERFACE
        ws2_32
        userenv
        bcrypt
        ntdll
    )
elseif(UNIX AND NOT APPLE)
    # Linux needs pthreads and dl
    target_link_libraries(mrpt_math_rust INTERFACE
        pthread
        dl
    )
elseif(APPLE)
    # macOS needs framework libraries
    target_link_libraries(mrpt_math_rust INTERFACE
        "-framework Security"
        "-framework CoreFoundation"
    )
endif()

# Make the target available for other parts of MRPT to use
set(MRPT_MATH_RUST_AVAILABLE TRUE PARENT_SCOPE)

# Build test executable if testing is enabled
if(BUILD_TESTING)
    add_executable(test-rust-math tests/test_rust_math.cpp)
    target_link_libraries(test-rust-math PRIVATE mrpt_math_rust)
    target_include_directories(test-rust-math PRIVATE ${MRPT_MATH_RUST_DIR}/include)
    
    # Add test to CTest
    add_test(NAME test-rust-math COMMAND test-rust-math)
endif()
