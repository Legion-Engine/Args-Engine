#include <core/scheduling/scheduler.hpp>
#include <core/logging/logging.hpp>

namespace legion::core::scheduling
{
    constexpr size_type reserved_threads = 1; // OS, this, OpenAL, Drivers

    async::rw_spinlock Scheduler::m_threadsLock;
    sparse_map<std::thread::id, std::unique_ptr<std::thread>> Scheduler::m_threads;
    std::queue<std::thread::id> Scheduler::m_unreservedThreads;
    const uint Scheduler::m_maxThreadCount = (static_cast<int>(std::thread::hardware_concurrency()) - reserved_threads) <= 0 ? reserved_threads : (std::thread::hardware_concurrency());
    async::rw_spinlock Scheduler::m_availabilityLock;
    uint Scheduler::m_availableThreads = static_cast<uint>(math::ceil(((m_maxThreadCount * 0.5f) - reserved_threads)) + math::epsilon<float>()); // subtract OS and this_thread, and then leave some extra for miscellaneous processes.

    async::rw_spinlock Scheduler::m_jobQueueLock;
    std::queue<Scheduler::runnable> Scheduler::m_jobs;
    sparse_map<std::thread::id, async::rw_spinlock> Scheduler::m_commandLocks;
    sparse_map<std::thread::id, std::queue<Scheduler::runnable>> Scheduler::m_commands;

    void Scheduler::threadMain(bool* exit, bool* start, bool lowPower)
    {
        std::thread::id id = std::this_thread::get_id();

        while (!(*start))
        {
            if (lowPower)
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            else
                std::this_thread::yield();
        }

        while (!(*exit))
        {
            OPTICK_EVENT();
            Scheduler::runnable instruction{};

            {
                async::readwrite_guard guard(m_commandLocks[id], async::lock_priority_normal);
                if (!m_commands[id].empty())
                {
                    instruction = m_commands[id].front();
                    m_commands[id].pop();
                }
            }

            instruction();
            instruction = {};

            static bool empty;
            {
                async::readwrite_guard guard(m_jobQueueLock, async::lock_priority_normal);
                empty = m_jobs.empty();

                if (!empty)
                {
                    instruction = m_jobs.front();
                    m_jobs.pop();
                    empty = m_jobs.empty();
                }
            }

            instruction();

            if (empty)
            {
                OPTICK_CATEGORY("Relieve LSU contention", Optick::Category::Wait);
                if (lowPower)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                else
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(1));
                }
            }
        }
    }

    Scheduler::Scheduler(events::EventBus* eventBus, bool lowPower, uint minThreads) : m_eventBus(eventBus), m_lowPower(lowPower)
    {
        legion::core::log::impl::thread_names[std::this_thread::get_id()] = "Initialization";

        if (m_availableThreads < minThreads)
        {
            m_lowPower = true;
            m_availableThreads = minThreads;
        }

        async::rw_spinlock::force_release(false);
        async::spinlock::force_release(false);

        std::thread::id id;
        while ((id = createThread(threadMain, &m_threadsShouldTerminate, &m_threadsShouldStart, m_lowPower)) != invalid_thread_id)
        {
            m_commands[id];
            m_commandLocks[id];
        }

        m_threadsShouldStart = true;

        addProcessChain("Update");
    }

    Scheduler::~Scheduler()
    {
        for (auto [_, processChain] : m_processChains)
            processChain.exit();

        m_threadsShouldTerminate = true;

        for (auto [_, thread] : m_threads)
            thread->join();
    }

    void Scheduler::run()
    {
        {
            auto unreserved = m_unreservedThreads;
            uint i = 0;
            while (!unreserved.empty())
            {
                auto id = unreserved.front();
                unreserved.pop();
                legion::core::log::impl::thread_names[id] = std::string("Worker ") + std::to_string(i++);
#if USE_OPTICK
                sendCommand(id, [&](void* param)
                    {
                        (void)param;
                        log::info("Thread {} assigned.", std::this_thread::get_id());
                        std::lock_guard guard(m_threadScopesLock);
                        m_threadScopes.push_back(std::make_unique<Optick::ThreadScope>(legion::core::log::impl::thread_names[std::this_thread::get_id()].c_str()));
                        OPTICK_UNUSED(*m_threadScopes[m_threadScopes.size() - 1]);
                    });
#else
                sendCommand(id, [](void* param)
                    {
                        (void)param;
                        log::info("Thread {} assigned.", std::this_thread::get_id());
                    });
#endif
            }
        }

        { // Start threads of all the other chains.
            async::readonly_guard guard(m_processChainsLock);
            for (auto [_, chain] : m_processChains)
                chain.run(m_lowPower);
        }

        legion::core::log::impl::thread_names[std::this_thread::get_id()] = "Update";
#if USE_OPTICK
        {
            std::lock_guard guard(m_threadScopesLock);
            m_threadScopes.push_back(std::make_unique<Optick::ThreadScope>(legion::core::log::impl::thread_names[std::this_thread::get_id()].c_str()));
            OPTICK_UNUSED(*m_threadScopes[m_threadScopes.size() - 1]);
        }
#endif

        while (!m_eventBus->checkEvent<events::exit>()) // Check for engine exit flag.
        {
            OPTICK_EVENT("Mainthread frame");
            {
                OPTICK_EVENT("Destroy exited threads");

                async::readwrite_guard guard(m_exitsLock); // Check for any intentionally exited threads and clean them up.

                if (m_exits.size())
                {
                    for (auto& id : m_exits)
                    {
                        destroyThread(id);
                    }

                    m_exits.clear();
                }
            }

            {
                OPTICK_EVENT("Destroy crashed threads");
                async::readwrite_guard guard(m_errorsLock); // Check for any unintentionally exited threads and clean them up.

                if (m_errors.size())
                {
                    for (thread_error& error : m_errors)
                    {
                        log::error("{}", error.message);
                        destroyThread(error.threadId);
                    }

                    throw std::logic_error(m_errors[0].message); // Re-throw an empty error so that the normal error handling system can take care of the rest.
                    m_errors.clear();
                }
            }

            {
                OPTICK_EVENT("Erase destroyed process chains");
                std::vector<id_type> toRemove; // Check for all the chains that have exited their threads and remove them from the chain list.

                {
                    async::readonly_multiguard rmguard(m_processChainsLock, m_threadsLock);
                    for (auto [id, chain] : m_processChains)
                        if (chain.threadId() != std::thread::id() && !m_threads.contains(chain.threadId()))
                            toRemove.push_back(id);
                }

                async::readwrite_guard wguard(m_processChainsLock);
                for (id_type& id : toRemove)
                    m_processChains.erase(id);
            }

            if (m_localChain.id()) // If the local chain is valid run an iteration.
                m_localChain.runInCurrentThread();

            if (syncRequested()) // If a major engine sync was requested halt thread until all threads have reached a sync point and let them all continue.
                waitForProcessSync();
        }

        for (auto [_, processChain] : m_processChains)
            processChain.exit();

        m_threadsShouldTerminate = true;
        m_syncLock.force_release();

        size_type exits;
        size_type chains;

        {
            async::readonly_multiguard rmguard(m_exitsLock, m_processChainsLock);
            exits = m_exits.size();
            chains = m_processChains.size();
        }

        size_type prevExits = 0;
        size_type prevChains = 0;

        {
            OPTICK_EVENT("Waiting for exits");
            while (exits < chains)
            {
                std::this_thread::yield();
                async::readonly_multiguard rmguard(m_exitsLock, m_processChainsLock);

                if (prevExits != exits || prevChains != chains)
                {
                    prevExits = exits;
                    prevChains = chains;
                    log::info("waiting for threads to end. {} threads left", chains - exits);
                }

                exits = m_exits.size();
                chains = m_processChains.size();
            }
        }

        for (auto& id : m_exits)
        {
            destroyThread(id);
        }

        m_threadScopes.clear();
        OPTICK_SHUTDOWN();

        async::rw_spinlock::force_release(true);
        async::spinlock::force_release(true);

        m_exits.clear();
    }

    void Scheduler::sendCommand(std::thread::id id, delegate<void(void*)> command, void* parameter)
    {
        OPTICK_EVENT();
        async::readwrite_guard guard(m_commandLocks[id]);
        m_commands[id].push({ command, parameter });
    }

    void Scheduler::queueJob(delegate<void(void*)> job, void* parameter)
    {
        OPTICK_EVENT();
        async::readwrite_guard guard(m_jobQueueLock);
        m_jobs.push({ job, parameter });
    }

    void Scheduler::destroyThread(std::thread::id id)
    {
        OPTICK_EVENT();
        async::readwrite_multiguard guard(m_availabilityLock, m_threadsLock);

        if (m_threads.contains(id)) // Check if thread exists.
        {
            m_availableThreads++;
            if (m_threads[id]->joinable()) // If the thread is still running then we need to wait for it to finish.
                m_threads[id]->join();
            m_threads.erase(id); // Remove the thread.
        }

    }

    void Scheduler::reportExit(const std::thread::id& id)
    {
        async::readwrite_guard guard(m_exitsLock);
        m_exits.push_back(id);
    }

    void Scheduler::reportExitWithError(const std::string& name, const std::thread::id& id, const legion::core::exception& exc)
    {
        async::readwrite_guard guard(m_errorsLock);
        std::stringstream ss;

        ss << "Encountered cross thread exception:"
            << "\n  thread id:\t" << id
            << "\n  thread name:\t" << name
            << "\n  message: \t" << exc.what()
            << "\n  file:    \t" << exc.file()
            << "\n  line:    \t" << exc.line()
            << "\n  function:\t" << exc.func() << '\n';

        m_errors.push_back({ ss.str(), id });
    }

    void Scheduler::reportExitWithError(const std::thread::id& id, const legion::core::exception& exc)
    {
        async::readwrite_guard guard(m_errorsLock);
        std::stringstream ss;

        ss << "Encountered cross thread exception:"
            << "\n  thread id:\t" << id
            << "\n  message: \t" << exc.what()
            << "\n  file:    \t" << exc.file()
            << "\n  line:    \t" << exc.line()
            << "\n  function:\t" << exc.func() << '\n';

        m_errors.push_back({ ss.str(), id });
    }

    void Scheduler::reportExitWithError(const std::string& name, const std::thread::id& id, const std::exception& exc)
    {
        async::readwrite_guard guard(m_errorsLock);
        std::stringstream ss;

        ss << "Encountered cross thread exception:"
            << "\n  thread id:\t" << id
            << "\n  thread name:\t" << name
            << "\n  message: \t" << exc.what() << '\n';

        m_errors.push_back({ ss.str(), id });
    }

    void Scheduler::reportExitWithError(const std::thread::id& id, const std::exception& exc)
    {
        async::readwrite_guard guard(m_errorsLock);
        std::stringstream ss;

        ss << "Encountered cross thread exception:"
            << "\n  thread id:\t" << id
            << "\n  message: \t" << exc.what() << '\n';

        m_errors.push_back({ ss.str(), id });
    }

    void Scheduler::waitForProcessSync()
    {
        OPTICK_EVENT();
        //log::debug("synchronizing thread: {}", log::impl::thread_names[std::this_thread::get_id()]);
        if (std::this_thread::get_id() != m_syncLock.ownerThread()) // Check if this is the main thread or not.
        {
            m_requestSync.store(true, std::memory_order_relaxed); // Request a synchronization.
            m_syncLock.sync(); // Wait for synchronization moment.
        }
        else
        {
            {
                async::readonly_guard guard(m_processChainsLock);
                while (m_syncLock.waiterCount() != m_processChains.size()) // Wait until all other threads have reached the synchronization moment.
                    std::this_thread::yield();
            }

            m_requestSync.store(false, std::memory_order_release);
            m_syncLock.sync(); // Release sync lock.
        }
    }

    bool Scheduler::hookProcess(cstring chainName, Process* process)
    {
        OPTICK_EVENT();
        id_type chainId = nameHash(chainName);
        async::readonly_guard guard(m_processChainsLock);
        if (m_processChains.contains(chainId))
        {
            m_processChains[chainId].addProcess(process);
            return true;
        }
        else if (m_localChain.id() == chainId)
        {
            m_localChain.addProcess(process);
            return true;
        }

        return false;
    }

    bool Scheduler::unhookProcess(cstring chainName, Process* process)
    {
        OPTICK_EVENT();
        id_type chainId = nameHash(chainName);
        async::readonly_guard guard(m_processChainsLock);
        if (m_processChains.contains(chainId))
        {
            m_processChains[chainId].removeProcess(process);
            return true;
        }
        else if (m_localChain.id() == chainId)
        {
            m_localChain.removeProcess(process);
            return true;
        }

        return false;
    }

}
