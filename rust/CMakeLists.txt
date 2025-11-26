# CMake configuration for Rust core library integration

cmake_minimum_required(VERSION 3.16)

# This file is only included when MRPT_USE_RUST_CORE is ON

# Check if Rust/Cargo is available
find_program(CARGO_EXECUTABLE cargo)
if(NOT CARGO_EXECUTABLE)
    message(FATAL_ERROR 
        "Cargo not found but MRPT_USE_RUST_CORE is enabled. "
        "Please install Rust from https://rustup.rs/ or disable MRPT_USE_RUST_CORE.")
endif()

message(STATUS "Rust core library will be built")
set(MRPT_RUST_CORE_AVAILABLE TRUE CACHE BOOL "Rust core library available" FORCE)

# Determine build type for Cargo
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CARGO_BUILD_TYPE "debug")
    set(CARGO_BUILD_FLAG "")
else()
    set(CARGO_BUILD_TYPE "release")
    set(CARGO_BUILD_FLAG "--release")
endif()

# Paths
set(RUST_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(RUST_TARGET_DIR "${RUST_ROOT_DIR}/target")
set(RUST_OUTPUT_DIR "${RUST_TARGET_DIR}/${CARGO_BUILD_TYPE}")

# Determine library extension
if(WIN32)
    set(RUST_LIB_PREFIX "")
    set(RUST_STATIC_EXT ".lib")
    set(RUST_DYNAMIC_EXT ".dll")
elseif(APPLE)
    set(RUST_LIB_PREFIX "lib")
    set(RUST_STATIC_EXT ".a")
    set(RUST_DYNAMIC_EXT ".dylib")
else()
    set(RUST_LIB_PREFIX "lib")
    set(RUST_STATIC_EXT ".a")
    set(RUST_DYNAMIC_EXT ".so")
endif()

set(RUST_CORE_STATIC_LIB "${RUST_OUTPUT_DIR}/${RUST_LIB_PREFIX}mrpt_core${RUST_STATIC_EXT}")
set(RUST_CORE_DYNAMIC_LIB "${RUST_OUTPUT_DIR}/${RUST_LIB_PREFIX}mrpt_core${RUST_DYNAMIC_EXT}")

# Custom command to build Rust library
add_custom_command(
    OUTPUT ${RUST_CORE_STATIC_LIB}
    COMMAND ${CARGO_EXECUTABLE} build ${CARGO_BUILD_FLAG}
    WORKING_DIRECTORY ${RUST_ROOT_DIR}
    COMMENT "Building Rust mrpt-core library (${CARGO_BUILD_TYPE})"
    VERBATIM
)

# Custom target for Rust build
add_custom_target(mrpt_core_rust_build
    DEPENDS ${RUST_CORE_STATIC_LIB}
)

# Create imported library target
add_library(mrpt_core_rust STATIC IMPORTED GLOBAL)
set_target_properties(mrpt_core_rust PROPERTIES
    IMPORTED_LOCATION ${RUST_CORE_STATIC_LIB}
    INTERFACE_INCLUDE_DIRECTORIES "${RUST_ROOT_DIR}/mrpt-core/include"
)

# Add dependency on the build target
add_dependencies(mrpt_core_rust mrpt_core_rust_build)

# Add platform-specific system libraries that Rust needs
if(WIN32)
    target_link_libraries(mrpt_core_rust INTERFACE
        ws2_32 userenv advapi32 bcrypt ntdll
    )
elseif(UNIX AND NOT APPLE)
    target_link_libraries(mrpt_core_rust INTERFACE
        pthread dl m
    )
elseif(APPLE)
    target_link_libraries(mrpt_core_rust INTERFACE
        pthread dl m
        "-framework Security"
        "-framework CoreFoundation"
    )
endif()

message(STATUS "Rust core library integration enabled")
message(STATUS "  Cargo: ${CARGO_EXECUTABLE}")
message(STATUS "  Build type: ${CARGO_BUILD_TYPE}")
message(STATUS "  Output: ${RUST_CORE_STATIC_LIB}")
