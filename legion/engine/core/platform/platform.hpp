#pragma once
/**
 * @file platform.hpp
 */

#if !defined(PROJECT_NAME)
#define PROJECT_NAME user_project
#endif

#define CONCAT(A, B) A ## B

#define CONCAT_DEFINE(A, B) CONCAT(A, B)

/**@def LEGION_CPP17V
 * @brief the version number of c++17 as long
 */
#define LEGION_CPP17V 201703L

#define LEGION_DEBUG_VALUE 1
#define LEGION_RELEASE_VALUE 2

#if defined(_DEBUG) || defined(DEBUG)
    /**@def LEGION_DEBUG
     * @brief Defined in debug mode.
     */
    #define LEGION_DEBUG
    #define LEGION_CONFIGURATION LEGION_DEBUG_VALUE
#else
    /**@def LEGION_RELEASE
     * @brief Defined in release mode.
     */
    #define LEGION_RELEASE 
    #define LEGION_CONFIGURATION LEGION_RELEASE_VALUE
#endif

#if (!defined(LEGION_LOW_POWER) && !defined(LEGION_HIGH_PERFORMANCE))
    /**@def LEGION_HIGH_PERFORMANCE
     * @brief Automatically defined if LEGION_LOW_POWER was not defined. It makes Legion ask the hardware's full attention to run as fast as possible.
     * @note Define LEGION_LOW_POWER to run Legion with minimal resources instead.
     */
    #define LEGION_HIGH_PERFORMANCE
#endif


#if defined(_WIN64)
    /**@def LEGION_WINDOWS
     * @brief Defined when compiling for Windows.
     */
    #define LEGION_WINDOWS

    #define WIN32_LEAN_AND_MEAN
    #define VC_EXTRALEAN
    #define NOMINMAX
    #include <Windows.h>
    #include <shellapi.h>
    #include <shlobj.h>
    #include <shlwapi.h>
    #include <objbase.h>
#elif defined(__linux__)
    /**@def LEGION_LINUX
     * @brief Defined when compiling for Linux.
     */
    #define LEGION_LINUX

    #include <sys/resource.h>
    #include <sys/types.h>
    #include <sys/wait.h>
    #include <unistd.h>
    #include <sched.h>
    #include <errno.h>
#endif

#ifndef __FUNC__
    #define __FUNC__ __func__ 
#endif

#ifndef __FULL_FUNC__
    #if defined(LEGION_WINDOWS)
        #define __FULL_FUNC__ __FUNCSIG__
    #elif defined(__linux__)
        #define __FULL_FUNC__ __PRETTY_FUNCTION__
    #else
        #define __FULL_FUNC__ __func__
    #endif
#endif

#if defined(__clang__)
    // clang
#define LEGION_CLANG

#if defined(__GNUG__) || (defined(__GNUC__) && defined(__cplusplus))
#define LEGION_CLANG_GCC
#elif defined(_MSC_VER)
#define LEGION_CLANG_MSVC
#endif

#define L_PAUSE_INSTRUCTION __builtin_ia32_pause
#elif defined(__GNUG__) || (defined(__GNUC__) && defined(__cplusplus))
    // gcc
#define LEGION_GCC
#define L_PAUSE_INSTRUCTION __builtin_ia32_pause
#elif defined(_MSC_VER)
    // msvc
#define LEGION_MSVC
#define L_PAUSE_INSTRUCTION _mm_pause
#endif

#if (defined(LEGION_WINDOWS) && !defined(LEGION_WINDOWS_USE_CDECL)) || defined (DOXY_INCLUDE)
    /**@def LEGION_CCONV
     * @brief the calling convention exported functions will use in the args engine
     */
    #define LEGION_CCONV __fastcall
#else
    #define LEGION_CCONV __cdecl
#endif

/**@def NO_MANGLING
 * @brief exports functions with C style names instead of C++ mangled names
 */
#define NO_MANGLING extern "C"

/**@def LEGION_FUNC
 * @brief export setting + calling convention used by the engine
 */
#define LEGION_FUNC LEGION_CCONV

/**@def LEGION_INTERFACE
 * @brief un-mangled function name +  export setting + calling convention used by the engine
 */
#define LEGION_INTERFACE NO_MANGLING LEGION_CCONV 

#if defined(__has_cpp_attribute)|| defined(DOXY_INCLUDE) 
/**@def L_HASCPPATTRIB
 * @brief checks if a certain attribute exists in this version of c++
 * @param x attribute you want to test for
 * @return true if attribute exists
 */
#  define L_HASCPPATTRIB(x) __has_cpp_attribute(x)
#else
#  define L_HASCPPATTRIB(x) 0
#endif

#if L_HASCPPATTRIB(fallthrough)
#define L_FALLTHROUGH [[fallthrough]]
#else
#define L_FALLTHROUGH
#endif

#if __cplusplus >= LEGION_CPP17V || L_HASCPPATTRIB(nodiscard) || defined(DOXY_INCLUDE)

/**@def L_NODISCARD
 * @brief Marks a function as "nodiscard" meaning that result must be captured and should not be discarded.
 */
#define L_NODISCARD [[nodiscard]]
#else
#define L_NODISCARD
#endif

#if __cplusplus > LEGION_CPP17V || L_HASCPPATTRIB(noreturn) || defined(DOXY_INCLUDE)
/**@def L_NORETURN
 * @brief Marks a function as "noreturn" meaning that the function will never finish, or terminate the application
 */
#define L_NORETURN [[noreturn]]
#else
#define L_NORETURN
#endif

/**@def LEGION_PURE
 * @brief Marks a function as pure virtual.
 */
#define LEGION_PURE =0

/**@def LEGION_IMPURE
 * @brief Marks a function as overridable but default implemented.
 */
#define LEGION_IMPURE {}

/**@def LEGION_IMPURE_RETURN
 * @brief Marks a function as overridable but default implemented with certain default return value.
 * @param x value the function should return.
 */
#define LEGION_IMPURE_RETURN(x) { return (x); }

#if !defined(LEGION_MIN_THREADS)
#define LEGION_MIN_THREADS 5
#endif
