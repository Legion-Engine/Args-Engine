#define LEGION_ENTRY
#if defined(NDEBUG)
    #define LEGION_KEEP_CONSOLE
#endif
//#define LEGION_LOW_POWER
//#define SUPER_LOW_POWER
#if defined(SUPER_LOW_POWER)
#define LEGION_MIN_THREADS 3 // Update, Rendering, Input, excluding audio and physics
#define LEGION_LOW_POWER
#else
#define LEGION_MIN_THREADS 5 // Update, Rendering, Input, Audio, Physics
#endif

#include <core/core.hpp>
#include <application/application.hpp>
#include <rendering/rendering.hpp>
#define AUDIO_EXIT_ON_FAIL
#include <audio/audio.hpp>
#include <physics/physics.hpp>
#include <rendering/module/renderingmodule.hpp>


#include "module/testmodule.hpp"

using namespace legion;

void LEGION_CCONV reportModules(Engine* engine)
{
    log::filter(log::severity::debug);

    engine->reportModule<TestModule>();
    engine->reportModule<app::ApplicationModule>();
    engine->reportModule<rendering::RenderingModule>();

#if !defined(SUPER_LOW_POWER)
    engine->reportModule<physics::PhysicsModule>();
    engine->reportModule<audio::AudioModule>();
#endif
}
