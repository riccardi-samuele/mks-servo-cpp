include(FetchContent)
FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG        v2.4.11
    GIT_SHALLOW    TRUE
)
# doctest v2.4.11 still uses `cmake_minimum_required(VERSION 2.x)` which
# CMake 4.x refuses to evaluate. This cache variable downgrades the policy
# version that the fetched project sees, without changing our own minimum.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)
FetchContent_MakeAvailable(doctest)
unset(CMAKE_POLICY_VERSION_MINIMUM CACHE)

find_package(Threads REQUIRED)

# Strict warnings for our own code only.
function(mks_servo_strict tgt)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
        target_compile_options(${tgt} PRIVATE
            -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion
        )
        if(MKS_SERVO_WERROR)
            target_compile_options(${tgt} PRIVATE -Werror)
        endif()
    elseif(MSVC)
        target_compile_options(${tgt} PRIVATE /W4 /permissive-)
        if(MKS_SERVO_WERROR)
            target_compile_options(${tgt} PRIVATE /WX)
        endif()
    endif()
endfunction()

add_executable(test_protocol test_protocol.cpp)
target_link_libraries(test_protocol PRIVATE mks_servo::mks_servo doctest::doctest)
mks_servo_strict(test_protocol)
add_test(NAME test_protocol COMMAND test_protocol)

# Transport-dependent tests are Linux-only (the header is, too).
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(test_transport_compile test_transport_compile.cpp)
    target_link_libraries(test_transport_compile PRIVATE mks_servo::mks_servo doctest::doctest)
    target_compile_definitions(test_transport_compile PRIVATE DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN)
    mks_servo_strict(test_transport_compile)
    add_test(NAME test_transport_compile COMMAND test_transport_compile)

    add_executable(test_raw_driver_mock test_raw_driver_mock.cpp)
    target_link_libraries(test_raw_driver_mock PRIVATE mks_servo::mks_servo doctest::doctest Threads::Threads)
    target_compile_definitions(test_raw_driver_mock PRIVATE DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN)
    mks_servo_strict(test_raw_driver_mock)
    add_test(NAME test_raw_driver_mock COMMAND test_raw_driver_mock)

    add_executable(test_motor test_motor.cpp)
    target_link_libraries(test_motor PRIVATE mks_servo::mks_servo doctest::doctest Threads::Threads)
    target_compile_definitions(test_motor PRIVATE DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN)
    mks_servo_strict(test_motor)
    add_test(NAME test_motor COMMAND test_motor)

    add_executable(test_motor_group test_motor_group.cpp)
    target_link_libraries(test_motor_group PRIVATE mks_servo::mks_servo doctest::doctest Threads::Threads)
    target_compile_definitions(test_motor_group PRIVATE DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN)
    mks_servo_strict(test_motor_group)
    add_test(NAME test_motor_group COMMAND test_motor_group)

    add_executable(test_rt test_rt.cpp)
    target_link_libraries(test_rt PRIVATE mks_servo::mks_servo doctest::doctest Threads::Threads)
    target_compile_definitions(test_rt PRIVATE DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN)
    mks_servo_strict(test_rt)
    add_test(NAME test_rt COMMAND test_rt)
endif()
