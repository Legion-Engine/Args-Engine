#include <core/scheduling/processchain.hpp>
#include <core/scheduling/process.hpp>
#include <core/scheduling/scheduler.hpp>
#include <core/common/exception.hpp>
#include <thread>

#include <core/logging/logging.hpp>

namespace legion::core::scheduling
{
    async::rw_spinlock ProcessChain::m_callbackLock;
    multicast_delegate<void()> ProcessChain::m_onFrameStart;
    multicast_delegate<void()> ProcessChain::m_onFrameEnd;

    void ProcessChain::threadedRun(ProcessChain* chain)
    {
        log::info("Chain started.");
        chain->m_scheduler->subscribeToSync();
       /* try
        {*/
            while (!chain->m_exit->load(std::memory_order_acquire)) // Check for exit flag.
            {
                OPTICK_EVENT();
                chain->runInCurrentThread(); // Execute all processes.

                if (chain->m_scheduler->syncRequested()) // Sync if requested.
                    chain->m_scheduler->waitForProcessSync();

                {
                    OPTICK_CATEGORY("Relieve LSU contention", Optick::Category::Wait);
                    L_PAUSE_INSTRUCTION();
                    std::this_thread::yield();
                }
            }
            chain->m_scheduler->unsubscribeFromSync();
            chain->m_scheduler->reportExit(chain->m_threadId); // Mark Exit.
       ///* }
       // catch (const legion::core::exception& e)
       // {*/
       //     chain->m_scheduler->unsubscribeFromSync();
       //     chain->m_scheduler->reportExitWithError(chain->m_name, std::this_thread::get_id(), e); // Mark error.
       ///* }
       // catch (const std::exception& e)
       // {*/
       //     chain->m_scheduler->unsubscribeFromSync();
       //     chain->m_scheduler->reportExitWithError(chain->m_name, std::this_thread::get_id(), e); // Mark error.
       // //}
    }

    bool ProcessChain::run(bool low_power)
    {
        m_low_power = low_power;
        m_exit->store(false, std::memory_order_release);
        std::thread::id threadId = m_scheduler->getChainThreadId(m_nameHash);
        if (threadId != std::thread::id())
        {
            m_threadId = threadId;

            m_scheduler->sendCommand(threadId, [](void* param) { ProcessChain::threadedRun(reinterpret_cast<ProcessChain*>(param)); }, this);

            return true;
        }
        return false;
    }

    void ProcessChain::exit()
    {
        m_exit->store(true, std::memory_order_release);
    }

    void ProcessChain::runInCurrentThread()
    {
        OPTICK_EVENT("Run process chain");
        OPTICK_TAG("Process chain", m_name.c_str());

        {
            async::readonly_guard guard(m_callbackLock);
            m_onFrameStart();
        }

        hashed_sparse_set<id_type> finishedProcesses;
        async::readonly_guard guard(m_processesLock); // Hooking more processes whilst executing isn't allowed.
        do
        {
            for (auto [id, process] : m_processes)
                if (!finishedProcesses.contains(id))
                    if (process->execute(m_scheduler->getTimeScale())) // If the process wasn't finished then execute it and check if it's finished now.
                        finishedProcesses.insert(id);

        } while (finishedProcesses.size() != m_processes.size() && !m_exit->load(std::memory_order_acquire));

        {
            async::readonly_guard guard(m_callbackLock);
            m_onFrameEnd();
        }
    }

    void ProcessChain::addProcess(Process* process)
    {
        OPTICK_EVENT();
        async::readwrite_guard guard(m_processesLock);
        if (m_processes.insert(process->id(), process).second)
            process->m_hooks.insert(m_nameHash);
    }

    void ProcessChain::removeProcess(Process* process)
    {
        OPTICK_EVENT();
        async::readwrite_guard guard(m_processesLock);
        if (m_processes.erase(process->id()))
            process->m_hooks.erase(m_nameHash);
    }
}
