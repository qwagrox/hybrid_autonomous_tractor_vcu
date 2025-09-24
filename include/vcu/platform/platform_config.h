// Copyright 2025 Manus AI

#ifndef VCU_PLATFORM_PLATFORM_CONFIG_H_
#define VCU_PLATFORM_PLATFORM_CONFIG_H_

/**
 * @file platform_config.h
 * @brief 平台配置和特性检测
 * 
 * 这个文件定义了不同平台的配置选项和特性检测宏
 */

// Platform detection
#if defined(PLATFORM_LINUX)
    #define VCU_PLATFORM_NAME "Linux"
    #define VCU_PLATFORM_LINUX 1
    #define VCU_PLATFORM_NUTTX 0
#elif defined(PLATFORM_NUTTX)
    #define VCU_PLATFORM_NAME "NuttX"
    #define VCU_PLATFORM_LINUX 0
    #define VCU_PLATFORM_NUTTX 1
#else
    #error "Unsupported platform. Please define PLATFORM_LINUX or PLATFORM_NUTTX"
#endif

// POSIX feature detection
#if defined(_POSIX_C_SOURCE) && _POSIX_C_SOURCE >= 200809L
    #define VCU_HAS_POSIX_2008 1
#else
    #define VCU_HAS_POSIX_2008 0
#endif

#if defined(_POSIX_THREADS) && _POSIX_THREADS > 0
    #define VCU_HAS_PTHREAD 1
#else
    #define VCU_HAS_PTHREAD 0
#endif

// Platform-specific features
#if VCU_PLATFORM_LINUX
    #define VCU_HAS_SOCKETCAN 1
    #define VCU_HAS_FILESYSTEM 1
    #define VCU_HAS_DYNAMIC_LOADING 1
    #define VCU_HAS_SIGNALS 1
#elif VCU_PLATFORM_NUTTX
    #define VCU_HAS_SOCKETCAN 0
    #define VCU_HAS_FILESYSTEM 1
    #define VCU_HAS_DYNAMIC_LOADING 0
    #define VCU_HAS_SIGNALS 1
#endif

// Memory management
#if VCU_PLATFORM_LINUX
    #define VCU_MEMORY_ALIGNMENT 8
    #define VCU_HAS_MALLOC 1
#elif VCU_PLATFORM_NUTTX
    #define VCU_MEMORY_ALIGNMENT 4
    #define VCU_HAS_MALLOC 1
#endif

// Timing and scheduling
#if VCU_PLATFORM_LINUX
    #define VCU_HAS_CLOCK_MONOTONIC 1
    #define VCU_HAS_NANOSLEEP 1
    #define VCU_HAS_TIMER_CREATE 1
#elif VCU_PLATFORM_NUTTX
    #define VCU_HAS_CLOCK_MONOTONIC 1
    #define VCU_HAS_NANOSLEEP 1
    #define VCU_HAS_TIMER_CREATE 1
#endif

// Networking
#if VCU_PLATFORM_LINUX
    #define VCU_HAS_SOCKETS 1
    #define VCU_HAS_UNIX_SOCKETS 1
#elif VCU_PLATFORM_NUTTX
    #define VCU_HAS_SOCKETS 1
    #define VCU_HAS_UNIX_SOCKETS 0
#endif

// Debugging and diagnostics
#if VCU_PLATFORM_LINUX
    #define VCU_HAS_BACKTRACE 1
    #define VCU_HAS_SYSLOG 1
#elif VCU_PLATFORM_NUTTX
    #define VCU_HAS_BACKTRACE 0
    #define VCU_HAS_SYSLOG 1
#endif

// Compiler and language features
#if defined(__GNUC__)
    #define VCU_COMPILER_GCC 1
    #define VCU_COMPILER_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#else
    #define VCU_COMPILER_GCC 0
    #define VCU_COMPILER_VERSION 0
#endif

// C++ standard library features
#if __cplusplus >= 201703L
    #define VCU_HAS_CPP17 1
#else
    #define VCU_HAS_CPP17 0
#endif

// Platform-specific includes
#if VCU_PLATFORM_LINUX
    #include <unistd.h>
    #include <pthread.h>
    #include <time.h>
#elif VCU_PLATFORM_NUTTX
    #include <unistd.h>
    #include <pthread.h>
    #include <time.h>
    #include <nuttx/config.h>
#endif

// Utility macros
#define VCU_UNUSED(x) ((void)(x))
#define VCU_LIKELY(x) __builtin_expect(!!(x), 1)
#define VCU_UNLIKELY(x) __builtin_expect(!!(x), 0)

#if VCU_COMPILER_GCC
    #define VCU_FORCE_INLINE __attribute__((always_inline)) inline
    #define VCU_NO_INLINE __attribute__((noinline))
    #define VCU_PACKED __attribute__((packed))
    #define VCU_ALIGNED(n) __attribute__((aligned(n)))
#else
    #define VCU_FORCE_INLINE inline
    #define VCU_NO_INLINE
    #define VCU_PACKED
    #define VCU_ALIGNED(n)
#endif

// Static assertions for platform requirements
#if !VCU_HAS_PTHREAD
    #error "Platform must support POSIX threads"
#endif

#if !VCU_HAS_POSIX_2008
    #error "Platform must support POSIX.1-2008 or later"
#endif

#endif // VCU_PLATFORM_PLATFORM_CONFIG_H_
