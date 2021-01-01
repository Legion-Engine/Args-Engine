#include <core/async/rw_spinlock.hpp>
#include <Optick/optick.h>
#include <sstream>

namespace legion::core::async
{
    bool rw_spinlock::m_forceRelease = false;
    std::atomic_uint rw_spinlock::m_lastId = { 1 };

    thread_local std::unordered_map<uint, int> rw_spinlock::m_localWriters;
    thread_local std::unordered_map<uint, int> rw_spinlock::m_localReaders;
    thread_local std::unordered_map<uint, lock_state> rw_spinlock::m_localState;

    void rw_spinlock::read_lock(lock_priority priority) const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return;

        if (m_localState.at(m_id) != lock_state::idle) // If we're either already reading or writing then the lock doesn't need to be reacquired.
        {
            // Report another local reader to the lock.
            m_localReaders.at(m_id)++;
            return;
        }

        int state;

        while (true)
        {
            // Read the current value and continue waiting until we're in a lockable state.
            while ((state = m_lockState.load(std::memory_order_relaxed)) == (int)lock_state::write)
            {
                OPTICK_EVENT("Acquire read lock");

                switch (priority)
                {
                case lock_priority::wait:
                    std::this_thread::sleep_for(std::chrono::microseconds(1));
                    break;
                case lock_priority::normal:
                    std::this_thread::yield();
                    break;
                case lock_priority::real_time:
                default:
                    L_PAUSE_INSTRUCTION();
                    break;
                }
            }

            // Try to add a reader to the lock state. If the lock succeeded then we can continue.
            if (m_lockState.compare_exchange_weak(state, state + (int)lock_state::read, std::memory_order_acquire, std::memory_order_relaxed))
                break;
        }

        // Report another reader to the lock.
        m_localReaders.at(m_id)++;
        m_localState.at(m_id) = lock_state::read; // Set thread_local state to read.
    }

    bool rw_spinlock::read_try_lock() const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return true;

        if (m_localState.at(m_id) != lock_state::idle) // If we're either already reading or writing then the lock doesn't need to be reacquired.
        {
            // Report another local reader to the lock.
            m_localReaders.at(m_id)++;
            return true;
        }

        // Expect idle as default.
        int state;

        if ((state = m_lockState.load(std::memory_order_relaxed)) == (int)lock_state::write || // Check if we can lock at all first to reduce LSU abuse on SMT CPUs if this occurs in a try_lock loop.
            !m_lockState.compare_exchange_strong(state, state + (int)lock_state::read, std::memory_order_acquire, std::memory_order_relaxed)) // Try to add a reader to the lock state.
            return false;

        // Report another reader to the lock.
        m_localReaders.at(m_id)++;
        m_localState.at(m_id) = lock_state::read; // Set thread_local state to read.
        return true;
    }

    void rw_spinlock::write_lock(lock_priority priority) const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return;

        if (m_localState.at(m_id) == lock_state::read) // If we're currently only acquired for read we need to stop reading before requesting rw.
        {
            read_unlock();
            m_localReaders.at(m_id)++;
        }
        else if (m_localState.at(m_id) == lock_state::write) // If we're already writing then we don't need to reacquire the lock.
        {
            m_localWriters.at(m_id)++;
            return;
        }

        int state;

        while (true)
        {
            // Read the current value and continue waiting until we're in a lockable state.
            while ((state = m_lockState.load(std::memory_order_relaxed)) != (int)lock_state::idle)
            {
                OPTICK_EVENT("Acquire write lock");

                switch (priority)
                {
                case lock_priority::wait:
                    std::this_thread::sleep_for(std::chrono::microseconds(1));
                    break;
                case lock_priority::normal:
                    std::this_thread::yield();
                    break;
                case lock_priority::real_time:
                default:
                    L_PAUSE_INSTRUCTION();
                    break;
                }
            }

            // Try to set the lock state to write. If the lock succeeded then we can continue.
            if (m_lockState.compare_exchange_weak(state, (int)lock_state::write, std::memory_order_acquire, std::memory_order_relaxed))
                break;
        }


        m_writer = std::this_thread::get_id();
        m_localWriters.at(m_id)++;
        m_localState.at(m_id) = lock_state::write; // Set thread_local state to write.
    }

    bool rw_spinlock::write_try_lock() const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return true;

        bool relock = false;
        if (m_localState.at(m_id) == lock_state::read) // If we're currently only acquired for read we need to stop reading before requesting rw.
        {
            relock = true;
            read_unlock();
            m_localReaders.at(m_id)++;
        }
        else if (m_localState.at(m_id) == lock_state::write) // If we're already writing then we don't need to reacquire the lock.
        {
            m_localWriters.at(m_id)++;
            return true;
        }

        // Expect idle as default.
        int state = (int)lock_state::idle;

        if ((state = m_lockState.load(std::memory_order_relaxed)) != (int)lock_state::idle || // Check if we can lock at all first to reduce LSU abuse on SMT CPUs if this occurs in a try_lock loop.
            !m_lockState.compare_exchange_strong(state, (int)lock_state::write, std::memory_order_acquire, std::memory_order_relaxed)) // Try to set the lock state to write.
        {
            if (relock)
            {
                m_localReaders.at(m_id)--;
                read_lock();
            }
            return false;
        }

        m_writer = std::this_thread::get_id();
        m_localWriters.at(m_id)++;
        m_localState.at(m_id) = lock_state::write; // Set thread_local state to write.
        return true;
    }

    void rw_spinlock::read_unlock() const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return;

        auto& localReaders = m_localReaders.at(m_id);
        localReaders--;

        if (localReaders > 0 || m_localWriters.at(m_id) > 0) // Another local guard is still alive that will unlock the lock for this thread.
        {
            return;
        }

        m_lockState.fetch_sub((int)lock_state::read, std::memory_order_release);

        m_localState.at(m_id) = lock_state::idle; // Set thread_local state to idle.
    }

    void rw_spinlock::write_unlock() const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return;

        auto& localWriters = m_localWriters.at(m_id);
        localWriters--;

        if (localWriters > 0) // Another local guard is still alive that will unlock the lock for this thread.
        {
            return;
        }
        else if (m_localReaders.at(m_id) > 0)
        {
            m_lockState.store((int)lock_state::read, std::memory_order_release);
            m_localState.at(m_id) = lock_state::read; // Set thread_local state to idle.
            return;
        }

        m_lockState.store((int)lock_state::idle, std::memory_order_release);
        m_localState.at(m_id) = lock_state::idle; // Set thread_local state to idle.
    }

    void rw_spinlock::force_release(bool release)
    {
        m_forceRelease = release;
    }

    rw_spinlock::rw_spinlock(rw_spinlock&& source) noexcept
    {
        if (m_forceRelease)
            return;

        assert_msg("Attempted to move a rw_spinlock that was locked.", source.m_lockState.load(std::memory_order_relaxed) == (int)lock_state::idle);
        m_id = source.m_id;
    }

    rw_spinlock& rw_spinlock::operator=(rw_spinlock&& source) noexcept
    {
        if (m_forceRelease)
            return *this;

        assert_msg("Attempted to move a rw_spinlock that was locked.", source.m_lockState.load(std::memory_order_relaxed) == (int)lock_state::idle);
        m_id = source.m_id;
        return *this;
    }

    void rw_spinlock::lock(lock_state permissionLevel, lock_priority priority) const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return;

        if (!m_localState.count(m_id))
        {
            m_localWriters.emplace(m_id, 0);
            m_localReaders.emplace(m_id, 0);
            m_localState.emplace(m_id, lock_state_idle);
        }

        switch (permissionLevel)
        {
        case lock_state::read:
            return read_lock(priority);
        case lock_state::write:
            return write_lock(priority);
        default:
            return;
        }
    }

    bool rw_spinlock::try_lock(lock_state permissionLevel) const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return true;

        if (!m_localState.count(m_id))
        {
            m_localWriters.emplace(m_id, 0);
            m_localReaders.emplace(m_id, 0);
            m_localState.emplace(m_id, lock_state_idle);
        }

        switch (permissionLevel)
        {
        case lock_state::read:
            return read_try_lock();
        case lock_state::write:
            return write_try_lock();
        default:
            return false;
        }
    }

    void rw_spinlock::unlock(lock_state permissionLevel) const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return;

        switch (permissionLevel)
        {
        case legion::core::async::lock_state::read:
            return read_unlock();
        case legion::core::async::lock_state::write:
            return write_unlock();
        default:
            return;
        }
    }

    void rw_spinlock::lock_shared() const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return;

        return read_lock();
    }

    bool rw_spinlock::try_lock_shared() const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return true;

        return read_try_lock();
    }

    void rw_spinlock::unlock_shared() const
    {
        OPTICK_EVENT();
        if (m_forceRelease)
            return;

        return read_unlock();
    }
}
