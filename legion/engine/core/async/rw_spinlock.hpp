#pragma once
#include <atomic>
#include <tuple>
#include <unordered_map>
#include <array>
#include <thread>
#include <functional>
#include <core/types/primitives.hpp>
#include <core/platform/platform.hpp>
#include <core/containers/sparse_set.hpp>
#include <core/detail/internals.hpp>

/**
 * @file rw_spinlock.hpp
 */

namespace legion::core::async
{
    enum struct lock_state : int { idle = 0, read = 1, write = -1 };

    enum struct lock_priority : int { normal, wait, real_time };

    inline constexpr lock_state lock_state_idle = lock_state::idle;
    inline constexpr lock_state lock_state_read = lock_state::read;
    inline constexpr lock_state lock_state_write = lock_state::write;

    inline constexpr lock_priority lock_priority_normal = lock_priority::normal;
    inline constexpr lock_priority lock_priority_wait = lock_priority::wait;
    inline constexpr lock_priority lock_priority_real_time = lock_priority::real_time;

    /**@class rw_spinlock
     * @brief Lock used with ::async::readonly_guard and ::async::readwrite_guard.
     * @note Read-only operations can happen simultaneously without waiting for each other.
     *		 Read-only operations will only wait for Read-Write operations to be finished.
     * @note Read-Write operations cannot happen simultaneously and will wait for each other.
     *		 Read-Write operations will also wait for any Read-only operations to be finished.
     * @ref legion::core::async::readonly_guard
     * @ref legion::core::async::readwrite_guard
     * @ref legion::core::async::readonly_multiguard
     * @ref legion::core::async::readwrite_multiguard
     * @ref legion::core::async::mixed_multiguard
     */
    struct rw_spinlock final
    {
    private:
        static bool m_forceRelease;
        static std::atomic_uint m_lastId;

        static thread_local std::unordered_map<uint, int> m_localWriters;
        static thread_local std::unordered_map<uint, int> m_localReaders;
        static thread_local std::unordered_map<uint, lock_state> m_localState;

        uint m_id = m_lastId.fetch_add(1, std::memory_order_relaxed);
        // State of the lock. -1 means that a thread has write permission. 0 means that the lock is unlocked. 1+ means that there are N amount of readers.
        mutable  std::atomic_int m_lockState = { 0 };
        mutable std::thread::id m_writer;

        void read_lock(lock_priority priority = lock_priority::real_time) const;

        bool read_try_lock() const;

        void write_lock(lock_priority priority = lock_priority::real_time) const;

        bool write_try_lock() const;

        void read_unlock() const;

        void write_unlock() const;

    public:
        static void force_release(bool release = true);

        rw_spinlock() = default;

        rw_spinlock(rw_spinlock&& source) noexcept;
        rw_spinlock& operator=(rw_spinlock&& source) noexcept;

        rw_spinlock(const rw_spinlock&) = delete;
        rw_spinlock& operator=(const rw_spinlock&) = delete;

        /**@brief Lock for a certain permission level. (locking for idle does nothing)
         * @note Locking stacks, locking for readonly multiple times will remain readonly.
         *		 Locking for write after already being locked for readonly in the same thread
         *       will attempt to elevate lock permission of this thread to write.
         *		 Locking for write multiple times will remain in write.
         * @param permissionLevel
         */
        void lock(lock_state permissionLevel = lock_state::write, lock_priority priority = lock_priority::real_time) const;

        /**@brief Try to lock for a certain permission level. If it fails it will return false otherwise true. (locking for idle does nothing)
         * @note Locking stacks, locking for readonly multiple times will remain readonly.
         *		 Locking for write after already being locked for readonly in the same thread will attempt to elevate lock permission of this thread to write.
         *		 Locking for write multiple times will remain in write.
         * @param permissionLevel
         * @return bool True when locked.
         */
        bool try_lock(lock_state permissionLevel = lock_state::write) const;

        /**@brief Unlock from a certain permission level.
         * @note If both read and write locks have been requested before and write is unlocked then the lock will return to readonly state.
         * @param permissionLevel
         */
        void unlock(lock_state permissionLevel = lock_state::write) const;

        /** @brief Locks the rw_spinlockfor shared ownership, blocks if the rw_spinlockis not available
         */
        void lock_shared() const;

        /** @brief Tries to lock the rw_spinlockfor shared ownership, returns if the rw_spinlockis not available
         */
        bool try_lock_shared() const;

        /** @brief Unlocks the mutex (shared ownership)
         */
        void unlock_shared() const;

        /**@brief Execute a function inside a critical section locked by a certain guard.
         * @tparam Guard Guard type to lock the lock with.
         * @param func Function to execute.
         * @return Return value of func.
         */
        template<typename Guard, typename Func>
        auto critical_section(const Func& func) const -> decltype(auto)
        {
            Guard guard(*this);
            return std::invoke(func);
        }
    };

    /**@class readonly_guard
     * @brief RAII guard that uses ::async::rw_spinlock to lock for read-only.
     * @note Read-only operations can happen simultaneously without waiting for each other.
     *		 Read-only operations will only wait for Read-Write operations to be finished.
     * @ref legion::core::async::rw_spinlock
     */
    class readonly_guard final
    {
    private:
        const rw_spinlock& m_lock;

    public:
        /**@brief Creates readonly guard and locks for Read-only.
         */
        readonly_guard(const rw_spinlock& lock, lock_priority priority = lock_priority::real_time) : m_lock(lock)
        {
            m_lock.lock(lock_state::read, priority);
        }

        readonly_guard(const readonly_guard&) = delete;

        /**@brief RAII style unlocks lock from Read-only.
         */
        ~readonly_guard()
        {
            m_lock.unlock(lock_state::read);
        }

        readonly_guard& operator=(readonly_guard&&) = delete;
    };

    /**@class readonly_multiguard
     * @brief RAII guard that uses multiple ::async::readonly_rw_spinlocks to lock them all for read-only. (similar to std::lock)
     * @note Read-only operations can happen simultaneously without waiting for each other.
     *		 Read-only operations will only wait for Read-Write operations to be finished.
     * @ref legion::core::async::rw_spinlock
     */
    template<size_type S>
    class readonly_multiguard final
    {
    private:
        std::array<const rw_spinlock*, S> m_locks;

    public:
        /**@brief Creates readonly multi-guard and locks for Read-only.
         */
        template<typename lock_type1 = rw_spinlock, typename lock_type2 = rw_spinlock, typename... lock_typesN>
        readonly_multiguard(const lock_type1& lock1, const lock_type2& lock2, const lock_typesN&... locks) : m_locks{ {&lock1, &lock2, &locks...} }
        {
            int lastLocked = -1; // Index to the last locked lock.

            bool locked = true;
            do
            {
                for (int i = 0; i <= lastLocked; i++) // If we failed to lock all locks we need to unlock the ones we did lock.
                    m_locks[i]->unlock(lock_state::read);

                // Reset variables
                locked = true;
                lastLocked = -1;

                // Try to lock all locks.
                for (int i = 0; i < m_locks.size(); i++)
                {
                    if (m_locks[i]->try_lock(lock_state::read))
                    {
                        lastLocked = i;
                    }
                    else
                    {
                        locked = false;
                        break;
                    }
                }
            } while (!locked);
        }

        readonly_multiguard(const readonly_multiguard&) = delete;

        /**@brief RAII style unlocks lock from Read-only.
         */
        ~readonly_multiguard()
        {
            for (auto* lock : m_locks)
                lock->unlock(lock_state::read);
        }

        readonly_multiguard& operator=(readonly_multiguard&&) = delete;
    };

#if !defined(DOXY_EXCLUDE)
    template<typename... types>
    readonly_multiguard(types...)->readonly_multiguard<sizeof...(types)>;
#endif
    /**@class readwrite_guard
     * @brief RAII guard that uses ::async::rw_spinlock to lock for read-write.
     * @note Read-Write operations cannot happen simultaneously and will wait for each other.
     *		 Read-Write operations will also wait for any Read-only operations to be finished.
     * @ref legion::core::async::rw_spinlock
     */
    class readwrite_guard final
    {
    private:
        const rw_spinlock& m_lock;

    public:
        /**@brief Creates read-write guard and locks for Read-Write.
         */
        readwrite_guard(const rw_spinlock& lock, lock_priority priority = lock_priority::real_time) : m_lock(lock)
        {
            m_lock.lock(lock_state::write, priority);
        }

        readwrite_guard(const readwrite_guard&) = delete;

        /**@brief RAII style unlocks lock from Read-Write.
         */
        ~readwrite_guard()
        {
            m_lock.unlock(lock_state::write);
        }

        readwrite_guard& operator=(readwrite_guard&&) = delete;
    };


    /**@class readwrite_multiguard
     * @brief RAII guard that uses multiple ::async::readonly_rw_spinlocks to lock them all for read-write. (similar to std::lock)
     * @note Read-Write operations cannot happen simultaneously and will wait for each other.
     *		 Read-Write operations will also wait for any Read-only operations to be finished.
     * @ref legion::core::async::rw_spinlock
     */
    template<size_type S>
    class readwrite_multiguard final
    {
    private:
        std::array<const rw_spinlock*, S> m_locks;

    public:
        /**@brief Creates read-write multi-guard and locks for Read-Write.
         */
        template<typename lock_type1, typename lock_type2, typename... lock_typesN>
        readwrite_multiguard(const lock_type1& lock1, const lock_type2& lock2, const lock_typesN&... locks) : m_locks{ {&lock1, &lock2, &locks...} }
        {
            int lastLocked = -1; // Index to the last locked lock.

            bool locked = true;
            do
            {
                for (int i = 0; i <= lastLocked; i++) // If we failed to lock all locks we need to unlock the ones we did lock.
                    m_locks[i]->unlock(lock_state::write);

                // Reset variables
                locked = true;
                lastLocked = -1;

                // Try to lock all locks.
                for (int i = 0; i < m_locks.size(); i++)
                {
                    if (m_locks[i]->try_lock(lock_state::write))
                    {
                        lastLocked = i;
                    }
                    else
                    {
                        locked = false;
                        break;
                    }
                }
            } while (!locked);
        }

        readwrite_multiguard(const readwrite_multiguard&) = delete;

        /**@brief RAII style unlocks lock from Read-Write.
         */
        ~readwrite_multiguard()
        {
            for (auto* lock : m_locks)
                lock->unlock(lock_state::write);
        }

        readwrite_multiguard& operator=(readwrite_multiguard&&) = delete;
    };

    template<typename... types>
    readwrite_multiguard(types...)->readwrite_multiguard<sizeof...(types)>;

    /**@class mixed_multiguard
     * @brief RAII guard that uses multiple ::async::readonly_rw_spinlocks to lock them all for user specified permissions. (similar to std::lock)
     * @note Read-only operations can happen simultaneously without waiting for each other.
     *		 Read-only operations will only wait for Read-Write operations to be finished.
     * @note Read-Write operations cannot happen simultaneously and will wait for each other.
     *		 Read-Write operations will also wait for any Read-only operations to be finished.
     * @ref legion::core::async::rw_spinlock
     */
    template<size_type S>
    class mixed_multiguard final
    {
    private:
        std::array<const rw_spinlock*, S / 2> m_locks;
        std::array<lock_state, S / 2> m_states;

        // Recursive function for filling the arrays with the neccessary data from the template arguments.
        template<size_type I, typename... types>
        void fill(const rw_spinlock& lock, lock_state state, types&&... args)
        {
            if constexpr (I > 2)
            {
                fill<I - 2>(args...);
            }

            m_locks[(I / 2) - 1] = &lock;
            m_states[(I / 2) - 1] = state;
        }

    public:
        /**@brief Creates readonly multi-guard and locks for specified permissions.
         * @note Argument order should be as follows: (rw_spinlock&, lock_state, rw_spinlock&, lock_state, ...)
         */
        template<typename... types>
        explicit mixed_multiguard(types&&... arguments)
        {
            static_assert(sizeof...(types) % 2 == 0, "Argument order should be (lock, lock-state, lock, lock-state). Argument count should thus be even.");

            fill<sizeof...(types)>(arguments...);

            int lastLocked = -1; // Index to the last locked lock.

            bool locked = true;
            do
            {
                for (int i = 0; i <= lastLocked; i++) // If we failed to lock all locks we need to unlock the ones we did lock.
                    m_locks[i]->unlock(m_states[i]);

                // Reset variables
                locked = true;
                lastLocked = -1;

                // Try to lock all locks.
                for (int i = 0; i < m_locks.size(); i++)
                {
                    if (m_locks[i]->try_lock(m_states[i]))
                    {
                        lastLocked = i;
                    }
                    else
                    {
                        locked = false;
                        break;
                    }
                }
            } while (!locked);
        }

        mixed_multiguard(const mixed_multiguard&) = delete;
        mixed_multiguard(mixed_multiguard&&) = delete;

        /**@brief RAII style unlocks lock from specified permissions.
         */
        ~mixed_multiguard()
        {
            for (int i = 0; i < m_locks.size(); i++)
                m_locks[i]->unlock(m_states[i]);
        }

        mixed_multiguard& operator=(mixed_multiguard&&) = delete;
        mixed_multiguard& operator=(const mixed_multiguard&) = delete;
    };

    // CTAD so you don't need to input the size of the guard parameters.
    template<typename... types>
    mixed_multiguard(types...)->mixed_multiguard<sizeof...(types)>;
}
