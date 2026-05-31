cmake_minimum_required(VERSION 3.16)

project(mks_servo
    VERSION 0.3.0
    DESCRIPTION "C++17 library for MKS SERVO42D RS485 stepper drivers"
    HOMEPAGE_URL "https://github.com/riccardi-samuele/mks-servo-cpp"
    LANGUAGES CXX
)

# ─── options ─────────────────────────────────────────────────────────
option(MKS_SERVO_BUILD_TESTS    "Build the unit tests"        ON)
option(MKS_SERVO_BUILD_EXAMPLES "Build example programs"      ON)
option(MKS_SERVO_WERROR         "Treat warnings as errors"    OFF)

# When consumed as a subdirectory (FetchContent / add_subdirectory),
# disable tests by default — only the top-level project should build them.
if(NOT PROJECT_IS_TOP_LEVEL)
    set(MKS_SERVO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
endif()

# ─── the library: header-only INTERFACE target ───────────────────────
add_library(mks_servo INTERFACE)
add_library(mks_servo::mks_servo ALIAS mks_servo)

target_include_directories(mks_servo INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(mks_servo INTERFACE cxx_std_17)

# ─── tests ───────────────────────────────────────────────────────────
if(MKS_SERVO_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(MKS_SERVO_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ─── install + find_package support ──────────────────────────────────
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS mks_servo
    EXPORT mks_servoTargets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT mks_servoTargets
    FILE        mks_servoTargets.cmake
    NAMESPACE   mks_servo::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mks_servo
)

configure_package_config_file(
    cmake/mks_servoConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/mks_servoConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mks_servo
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/mks_servoConfigVersion.cmake"
    VERSION       ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/mks_servoConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/mks_servoConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mks_servo
)
