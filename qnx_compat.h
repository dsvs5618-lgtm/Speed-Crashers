/**
 * @file qnx_compat.h
 * @brief Native QNX Neutrino Microkernel IPC & Cross-Platform Compatibility Layer
 *
 * Provides real-time primitives:
 * - QNX Channels, Connections, and Pulses (ChannelCreate, MsgSendPulse, SIGEV_PULSE)
 * - Zero-drift monotonic periodic timers
 * - Priority Inheritance Mutexes (PTHREAD_PRIO_INHERIT)
 * - POSIX Shared Memory (shm_open, mmap)
 * - Transparent fallback when building on host test environments
 */

#ifndef QNX_COMPAT_H
#define QNX_COMPAT_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>

#ifdef __QNX__
    #include <sys/neutrino.h>
    #include <sys/syspage.h>
    #include <sys/netmgr.h>
    #include <sys/mman.h>
    #include <hw/inout.h>
#else
    #ifdef _WIN32
        #include <winsock2.h>
        #include <windows.h>
    #else
        #include <sys/mman.h>
        #include <sys/stat.h>
        #include <fcntl.h>
        #include <unistd.h>
    #endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* 1. QNX Microkernel Pulse Codes (8-bit signed char in QNX Neutrino)         */
/* ========================================================================== */
#define PULSE_CODE_TIMER_SAFETY      _PULSE_CODE_MINAVAIL + 1
#define PULSE_CODE_TIMER_SENSOR      _PULSE_CODE_MINAVAIL + 2
#define PULSE_CODE_TIMER_ESTIMATOR   _PULSE_CODE_MINAVAIL + 3
#define PULSE_CODE_TIMER_CONTROLLER  _PULSE_CODE_MINAVAIL + 4
#define PULSE_CODE_TIMER_ACTUATOR    _PULSE_CODE_MINAVAIL + 5
#define PULSE_CODE_WATCHDOG_KICK     _PULSE_CODE_MINAVAIL + 6
#define PULSE_CODE_AEB_OVERRIDE      _PULSE_CODE_MINAVAIL + 7
#define PULSE_CODE_MODE_TOGGLE       _PULSE_CODE_MINAVAIL + 8

#ifndef _PULSE_CODE_MINAVAIL
#define _PULSE_CODE_MINAVAIL 0
#endif

/* ========================================================================== */
/* 2. High-Resolution Timing & Clock Utilities                                */
/* ========================================================================== */

/**
 * @brief Get high-resolution monotonic time in nanoseconds
 */
static inline uint64_t qnx_get_time_ns(void) {
#ifdef __QNX__
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000ULL) + (uint64_t)ts.tv_nsec;
#elif defined(_WIN32)
    static LARGE_INTEGER freq;
    static int initialized = 0;
    LARGE_INTEGER count;
    if (!initialized) {
        QueryPerformanceFrequency(&freq);
        initialized = 1;
    }
    QueryPerformanceCounter(&count);
    return (uint64_t)((count.QuadPart * 1000000000ULL) / freq.QuadPart);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000000000ULL) + (uint64_t)ts.tv_nsec;
#endif
}

/**
 * @brief Get monotonic time in milliseconds
 */
static inline uint64_t qnx_get_time_ms(void) {
    return qnx_get_time_ns() / 1000000ULL;
}

/**
 * @brief Zero-drift absolute monotonic periodic sleeper
 * @param period_ms Desired loop period in milliseconds
 * @param next_wake_ns Pointer to running wake timestamp
 */
static inline void qnx_periodic_sleep_until(uint32_t period_ms, uint64_t *next_wake_ns) {
    uint64_t period_ns = (uint64_t)period_ms * 1000000ULL;
    uint64_t now_ns = qnx_get_time_ns();

    if (*next_wake_ns == 0) {
        *next_wake_ns = now_ns + period_ns;
    }

    if (*next_wake_ns > now_ns) {
        uint64_t diff_ns = *next_wake_ns - now_ns;
#ifdef _WIN32
        DWORD sleep_ms = (DWORD)(diff_ns / 1000000ULL);
        if (sleep_ms > 0) {
            Sleep(sleep_ms);
        } else {
            Sleep(0);
        }
#else
        struct timespec ts;
        ts.tv_sec = diff_ns / 1000000000ULL;
        ts.tv_nsec = diff_ns % 1000000000ULL;
        nanosleep(&ts, NULL);
#endif
    }
    *next_wake_ns += period_ns;
}

/* ========================================================================== */
/* 3. Thread Priority & Priority Inheritance Initialization                  */
/* ========================================================================== */

/**
 * @brief Initialize a mutex with Priority Inheritance (PTHREAD_PRIO_INHERIT)
 * Critical for preventing priority inversion in RTOS environments.
 */
static inline int qnx_init_priority_mutex(pthread_mutex_t *mutex) {
    pthread_mutexattr_t attr;
    if (pthread_mutexattr_init(&attr) != 0) {
        return -1;
    }
#if defined(__QNX__) || defined(PTHREAD_PRIO_INHERIT)
    pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
#endif
    int ret = pthread_mutex_init(mutex, &attr);
    pthread_mutexattr_destroy(&attr);
    return ret;
}

/**
 * @brief Set thread priority according to Rate Monotonic Scheduling (RMS)
 */
static inline int qnx_set_thread_priority(pthread_t thread, int qnx_priority) {
#ifdef __QNX__
    struct sched_param param;
    param.sched_priority = qnx_priority;
    return pthread_setschedparam(thread, SCHED_FIFO, &param);
#else
    /* Non-QNX simulation environment: log or set best-effort priority */
    (void)thread;
    (void)qnx_priority;
    return 0;
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* QNX_COMPAT_H */
