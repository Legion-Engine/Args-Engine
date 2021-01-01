#pragma once
#include <application/events/inputevents.hpp>
#include <application/events/windowinputevents.hpp>
#include <numeric>

namespace legion::application
{



    class InputSystem : public core::System<InputSystem>
    {

        using action_callback = delegate<void(InputSystem*, bool, inputmap::modifier_keys, inputmap::method, float, float)>;

        struct action_data
        {
            action_callback callback;

            float trigger_value;

            inputmap::method last_method;
            inputmap::modifier_keys last_mods;
            bool last_state;
            bool repeat;
        };

        using axis_callback = delegate<void(InputSystem*, float, inputmap::modifier_keys, inputmap::method, float)>;

        struct axis_data
        {
            axis_callback callback;

            inputmap::method last_method;
            inputmap::modifier_keys last_mods;
            float last_value;
        };

        struct axis_command_queue
        {
            std::vector<float> values;
            std::vector<inputmap::modifier_keys> mods;
            std::vector<inputmap::method> methods;
            delegate<void()> invoke;
        };

    public:
        void setup() override
        {
            //subscribe to the raw events emitted from the window system
            bindToEvent<key_input, &InputSystem::onKey>();
            bindToEvent<mouse_moved, &InputSystem::onMouseMove>();
            bindToEvent<mouse_button, &InputSystem::onMouseButton>();
            bindToEvent<mouse_scrolled, &InputSystem::onMouseScrolled>();

            //create Update Process
            createProcess<&InputSystem::onUpdate>("Input", 1.f/300.f);

            //make sure we get the joystick-callback on initialization of GLFW
            ContextHelper::addOnInitCallback(delegate<void()>::create([]
                {
                    ContextHelper::setJoystickCallback(&InputSystem::onCheckGamepadPresence);

                    //also enumerate Joysticks at the beginning of the Engine, the callback is not called on Joysticks that
                    //that are already connected

                    //note that GLFW only supports 16 gamepads!
                    for (size_type i = 0; i < inputmap::modifier_keys::MAX_SIZE - inputmap::modifier_keys::JOYSTICK0; ++i)
                    {
                        if (ContextHelper::joystickPresent(i))
                        {
                            m_presentGamepads.insert(i);
                        }
                    }
                }));
            //make sure the mappings match here!
            //ContextHelper::updateGamepadMappings("assets/conf/gamepad.conf");

        }

        inline static math::dvec2 getMousePosition()
        {
            return m_mousePos;
        }

        inline static math::dvec2 getMouseDelta()
        {
            return m_mouseDelta;
        }

        /**
         * @brief Creates a Binding of a Key /Axis to the emission of an event in the event bus.
         *
         * @tparam Event The type of event you want to be emitted on the appearance of the action/axis.
         * @param k The axis/action you want to listen to. @see inputmap::method for options
         * @param value The value you want to map to if your Event is an Axis but the event is a button/key.
         *         For instance if you want to map one axis to 'W/S' you can make 'W' emit 1 and 'S' -1.
         */
        template <class Event>
        static void createBinding(inputmap::method k, float value = 1) {
            static_assert(std::is_base_of_v<input_action<Event>, Event> ||
                std::is_base_of_v<input_axis<Event>, Event>,
                "Event needs to either be an input_action or an input_axis");

            if constexpr (std::is_base_of_v<input_action<Event>, Event>)
            {
                //are we dealing with a family of methods ?
                if (inputmap::is_family(k))
                {
                    for (inputmap::method m : inputmap::get_family(k))
                    {
                        if (inputmap::is_key(m))
                        {
                            bindKeyToAction<Event>(m);
                        }
                        if (inputmap::is_axis(m))
                        {
                            bindAxisToAction<Event>(m, value);
                        }
                    }
                }
                else
                {
                    if (inputmap::is_key(k))
                    {
                        bindKeyToAction<Event>(k);
                    }
                    if (inputmap::is_axis(k))
                    {
                        bindAxisToAction<Event>(k, value);
                    }
                }
            }
            else
            {
                if (inputmap::is_family(k))
                {
                    for (inputmap::method m : inputmap::get_family(k))
                    {
                        if (inputmap::is_key(m))
                        {
                            bindKeyToAxis<Event>(m, value);
                        }
                        if (inputmap::is_axis(m))
                        {
                            bindAxisToAxis<Event>(m, value);
                        }
                    }
                }
                else
                {
                    if (inputmap::is_key(k))
                    {
                        bindKeyToAxis<Event>(k, value);
                    }
                    if (inputmap::is_axis(k))
                    {
                        bindAxisToAxis<Event>(k, value);
                    }
                }
            }
        }

        /** @brief removes a Binding from a method
         *  @tparam Event the Event you want to unbind
         *  @param met the method you want to unbind from
         */
        template<class Event>
        static void removeBinding(inputmap::method met)
        {
            static_assert(std::is_base_of_v<input_action<Event>, Event> ||
                std::is_base_of_v<input_axis<Event>, Event>,
                "Event needs to either be an input_action or an input_axis");

            if constexpr (std::is_base_of_v<input_action<Event>, Event>)
            {

                //are we dealing with a family of methods ?
                if (inputmap::is_family(met))
                {
                    for (inputmap::method member : inputmap::get_family(met))
                    {
                        if (inputmap::is_key(member))
                        {
                            m_actions[member][typeHash<Event>()].callback.clear();
                        }
                        if (inputmap::is_axis(member))
                        {
                            m_axes[member][typeHash<Event>()].callback.clear();
                        }
                    }
                }
                else
                {
                    if (inputmap::is_key(met))
                    {
                        m_actions[met][typeHash<Event>()].callback.clear();
                    }
                    if (inputmap::is_axis(met))
                    {
                        m_axes[met][typeHash<Event>()].callback.clear();
                    }
                }
            }
            else
            {
                if (inputmap::is_family(met))
                {
                    for (inputmap::method member : inputmap::get_family(met))
                    {
                        if (inputmap::is_key(member))
                        {
                            m_actions[member][typeHash<Event>()].callback.clear();
                        }
                        if (inputmap::is_axis(member))
                        {
                            m_axes[member][typeHash<Event>()].callback.clear();
                        }
                    }
                }
                else
                {
                    if (inputmap::is_key(met))
                    {
                        m_actions[met][typeHash<Event>()].callback.clear();
                    }
                    if (inputmap::is_axis(met))
                    {
                        m_axes[met][typeHash<Event>()].callback.clear();
                    }
                }
            }
        }

    private:

        template<class Event>
        static void bindKeyToAction(inputmap::method m)
        {
            //creates a tuple with default value 0
            auto& data = m_actions[m][typeHash<Event>()];

            data.callback = action_callback::create(
                [](InputSystem* self, bool state, inputmap::modifier_keys mods, inputmap::method method, float def, float delta)
                {
                    OPTICK_EVENT("Key to action callback");
                    (void)def;
                    Event e;
                    e.input_delta = delta;
                    e.set(state, mods, method);
                    self->raiseEvent<Event>(e);
                }
            );
            data.last_method = m;
            data.last_mods = inputmap::modifier_keys::NONE;
            data.repeat = false;
        }

        template<class Event>
        static void bindKeyToAxis(inputmap::method m, float value)
        {
            auto& data = m_actions[m][typeHash<Event>()];

            data.callback = action_callback::create(
                [](InputSystem* self, bool state, inputmap::modifier_keys mods, inputmap::method method, float def, float delta)
                {
                    self->pushCommand<Event>(state ? def : 0.0f,mods,method);
                }
            );
            data.trigger_value = value;
            data.last_method = m;
            data.last_mods = inputmap::modifier_keys::NONE;
            data.last_state = false;
            data.repeat = true;
        }

        template<class Event>
        static void bindAxisToAction(inputmap::method m, float value)
        {

            auto& data = m_axes[m][typeHash<Event>()];

            data.callback = axis_callback::create(
                [](InputSystem* self, float value, inputmap::modifier_keys mods, inputmap::method method, float delta)
                {
                    OPTICK_EVENT("Axis to action callback");
                    Event e;
                    e.input_delta = delta;
                    e.set(value > 0.05f || value < -0.05f, mods, method); //convert float range 0-1 to key state false:true
                    self->raiseEvent<Event>(e);
                }
            );

            data.last_value = value;
            data.last_method = m;
            data.last_mods = inputmap::modifier_keys::NONE;
        }

        template<class Event>
        static void bindAxisToAxis(inputmap::method m, float value)
        {

            auto& data = m_axes[m][typeHash<Event>()];

            data.callback = axis_callback::create(
                [](InputSystem* self, float value, inputmap::modifier_keys mods, inputmap::method method, float delta)
                {
                    self->pushCommand<Event>(value,mods,method);
                }
            );

            data.last_value = value;
            data.last_method = m;
            data.last_mods = inputmap::modifier_keys::NONE;
        }

        //joystick (dis)connect callback
        static void onCheckGamepadPresence(int jid, int event)
        {
            if (event == GLFW_CONNECTED)
                m_presentGamepads.insert(jid);
            else if (event == GLFW_DISCONNECTED)
                m_presentGamepads.erase(jid);
        }

        void onUpdate(time::time_span<fast_time> deltaTime)
        {
            OPTICK_EVENT();
            onJoystick(deltaTime);

            {
                OPTICK_EVENT("Update axes");
                //update all axis with their current values
                for (auto [_, inner_map] : m_axes)
                {
                    for (auto [_, axis] : inner_map)
                    {
                        axis.callback(this, axis.last_value, axis.last_mods, axis.last_method, deltaTime);
                    }
                }
            }

            {
                OPTICK_EVENT("Action repeating callbacks");
                for (auto [_, inner_map] : m_actions)
                {
                    for (auto [_, action] : inner_map)
                    {
                        if (action.repeat)
                            action.callback(this, action.last_state, action.last_mods, action.last_method, action.trigger_value, deltaTime);
                    }
                }
            }

            raiseCommandQueues(deltaTime);

            onMouseReset();
        }

        void matchGLFWAxisWithSignalAxis(const GLFWgamepadstate& state, inputmap::modifier_keys joystick,
            const size_type glfw, inputmap::method m)
        {
            OPTICK_EVENT();
            const float value = state.axes[glfw];
            for (auto [_, axis] : m_axes[m])
            {
                axis.last_value = value;
                axis.last_method = m;
                axis.last_mods = joystick;
            }
        }


        void onMouseReset()
        {
            for (auto [_, axis] : m_axes[inputmap::method::MOUSE_X])
            {
                axis.last_value = 0.0f;
                axis.last_mods = inputmap::modifier_keys::NONE;
                axis.last_method = inputmap::method::MOUSE_X;
            }
            for (auto [_, axis] : m_axes[inputmap::method::MOUSE_Y])
            {
                axis.last_value = 0.0f;
                axis.last_mods = inputmap::modifier_keys::NONE;
                axis.last_method = inputmap::method::MOUSE_Y;
            }
        }

        void onJoystick(float dt)
        {
            OPTICK_EVENT();
            for (int glfw_joystick_id : m_presentGamepads)
            {
                using mods = inputmap::modifier_keys;
                using method = inputmap::method;

                GLFWgamepadstate state;
                if (!ContextHelper::getGamepadSate(glfw_joystick_id, &state)) continue;

                const auto joystick = mods::JOYSTICK0 + glfw_joystick_id;

                for (auto [_, action] : m_actions[method::GAMEPAD_A])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_A], joystick, method::GAMEPAD_A, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_B])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_B], joystick, method::GAMEPAD_B, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_X])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_X], joystick, method::GAMEPAD_X, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_Y])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_Y], joystick, method::GAMEPAD_Y, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_LEFT_BUMPER])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_LEFT_BUMPER], joystick, method::GAMEPAD_LEFT_BUMPER, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_RIGHT_BUMPER])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_RIGHT_BUMPER], joystick, method::GAMEPAD_RIGHT_BUMPER, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_BACK])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_BACK], joystick, method::GAMEPAD_BACK, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_START])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_START], joystick, method::GAMEPAD_START, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_DPAD_UP])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_UP], joystick, method::GAMEPAD_DPAD_UP, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_DPAD_RIGHT])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_RIGHT], joystick, method::GAMEPAD_DPAD_RIGHT, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_DPAD_LEFT])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_LEFT], joystick, method::GAMEPAD_DPAD_LEFT, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_DPAD_DOWN])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_DPAD_DOWN], joystick, method::GAMEPAD_DPAD_DOWN, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_LEFT_THUMB])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_LEFT_THUMB], joystick, method::GAMEPAD_LEFT_THUMB, action.trigger_value, dt);
                for (auto [_, action] : m_actions[method::GAMEPAD_RIGHT_THUMB])
                    action.callback(this, state.buttons[GLFW_GAMEPAD_BUTTON_RIGHT_THUMB], joystick, method::GAMEPAD_RIGHT_THUMB, action.trigger_value, dt);

                matchGLFWAxisWithSignalAxis(state, joystick, GLFW_GAMEPAD_AXIS_LEFT_X, method::GAMEPAD_LEFT_X);
                matchGLFWAxisWithSignalAxis(state, joystick, GLFW_GAMEPAD_AXIS_LEFT_Y, method::GAMEPAD_LEFT_Y);
                matchGLFWAxisWithSignalAxis(state, joystick, GLFW_GAMEPAD_AXIS_LEFT_TRIGGER, method::GAMEPAD_LEFT_TRIGGER);
                matchGLFWAxisWithSignalAxis(state, joystick, GLFW_GAMEPAD_AXIS_RIGHT_X, method::GAMEPAD_RIGHT_X);
                matchGLFWAxisWithSignalAxis(state, joystick, GLFW_GAMEPAD_AXIS_RIGHT_Y, method::GAMEPAD_RIGHT_Y);
                matchGLFWAxisWithSignalAxis(state, joystick, GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER, method::GAMEPAD_RIGHT_TRIGGER);
            }
        }

        static inputmap::modifier_keys translateModifierKeys(int glfw_mods)
        {
            using mods = inputmap::modifier_keys;
            mods result = mods::NONE;

            if (glfw_mods & GLFW_MOD_ALT)
                result = static_cast<mods>(result & inputmap::modifier_keys::ALT);

            if (glfw_mods & GLFW_MOD_SHIFT)
                result = static_cast<mods>(result & inputmap::modifier_keys::SHIFT);

            if (glfw_mods & GLFW_MOD_CONTROL)
                result = static_cast<mods>(result & inputmap::modifier_keys::CTRL);

            return result;
        }

        void onKey(key_input* window_key_event)
        {
            const auto m = static_cast<inputmap::method>(window_key_event->key);
            for (auto [_, action] : m_actions[m])
            {
                action.last_state = window_key_event->action != GLFW_RELEASE;
                action.last_mods = translateModifierKeys(window_key_event->mods);
                action.last_method = m;
                if (!action.repeat)
                    action.callback(this, action.last_state, action.last_mods, action.last_method, action.trigger_value, 0.0f);

            }
        }

        void onMouseMove(mouse_moved* window_mouse_event)
        {
            m_mouseDelta = window_mouse_event->position - m_mousePos;
            m_mousePos = window_mouse_event->position;
            if (math::abs(m_mouseDelta.x) < 0.0001)
                m_mouseDelta.x = 0.0;

            if (math::abs(m_mouseDelta.y) < 0.0001)
                m_mouseDelta.y = 0.0;

            for (auto [_, axis] : m_axes[inputmap::method::MOUSE_X])
            {
                axis.last_value = static_cast<float>(m_mouseDelta.x);
                axis.last_mods = inputmap::modifier_keys::NONE;
                axis.last_method = inputmap::method::MOUSE_X;
            }
            for (auto [_, axis] : m_axes[inputmap::method::MOUSE_Y])
            {
                axis.last_value = static_cast<float>(m_mouseDelta.y);
                axis.last_mods = inputmap::modifier_keys::NONE;
                axis.last_method = inputmap::method::MOUSE_Y;
            }
        }

        void onMouseButton(mouse_button* window_mouse_event)
        {
            switch (window_mouse_event->button)
            {
            case GLFW_MOUSE_BUTTON_LEFT: {
                for (auto [_, action] : m_actions[inputmap::method::MOUSE_LEFT])
                {
                    action.last_state = window_mouse_event->action != GLFW_RELEASE;
                    action.last_mods = translateModifierKeys(window_mouse_event->mods);
                    action.last_method = inputmap::method::MOUSE_LEFT;
                    if (!action.repeat)
                        action.callback(this, action.last_state, action.last_mods, action.last_method, action.trigger_value, 0.0f);
                }
                break;
            }
            case GLFW_MOUSE_BUTTON_MIDDLE: {
                for (auto [_, action] : m_actions[inputmap::method::MOUSE_MIDDLE])
                {
                    action.last_state = window_mouse_event->action != GLFW_RELEASE;
                    action.last_mods = translateModifierKeys(window_mouse_event->mods);
                    action.last_method = inputmap::method::MOUSE_MIDDLE;
                    if (!action.repeat)
                        action.callback(this, action.last_state, action.last_mods, action.last_method, action.trigger_value, 0.0f);
                }
                break;
            }
            case GLFW_MOUSE_BUTTON_RIGHT: {
                for (auto [_, action] : m_actions[inputmap::method::MOUSE_RIGHT])
                {
                    action.last_state = window_mouse_event->action != GLFW_RELEASE;
                    action.last_mods = translateModifierKeys(window_mouse_event->mods);
                    action.last_method = inputmap::method::MOUSE_RIGHT;
                    if (!action.repeat)
                        action.callback(this, action.last_state, action.last_mods, action.last_method, action.trigger_value, 0.0f);
                }
                break;
            }
            }
        }

        void onMouseScrolled(mouse_scrolled* window_mouse_event)
        {
            const auto pos = window_mouse_event->offset;
            for (auto [_, axis] : m_axes[inputmap::method::HSCROLL])
            {
                axis.last_value += static_cast<float>(pos.x);
                axis.last_mods = inputmap::modifier_keys::NONE;
                axis.last_method = inputmap::method::HSCROLL;
            }

            for (auto [_, axis] : m_axes[inputmap::method::VSCROLL])
            {
                axis.last_value += static_cast<float>(pos.y);
                axis.last_mods = inputmap::modifier_keys::NONE;
                axis.last_method = inputmap::method::VSCROLL;
            }
        }

        template <class Event>
        void pushCommand(float value,inputmap::modifier_keys mods, inputmap::method method)
        {
            auto& cq = m_axes_command_queues[Event::id];
            cq.values.push_back(value);
            cq.mods.push_back(mods);
            cq.methods.push_back(method);
        }

        void raiseCommandQueues(float delta)
        {
            OPTICK_EVENT();
            for(auto  [key,value] : m_axes_command_queues){
                auto axis = std::make_unique<input_axis<std::nullptr_t>>();
                axis->value_parts = value.values;
                axis->mods_parts = value.mods;
                axis->identifier_parts = value.methods;

                axis->input_delta = delta;
                axis->value = std::accumulate(value.values.begin(),value.values.end(),0.0f);

                raiseEventUnsafe(std::move(axis),key);
                value.mods.clear();
                value.values.clear();
                value.methods.clear();
            }
        }

        static math::dvec2 m_mousePos;
        static math::dvec2 m_mouseDelta;

        static std::set<int> m_presentGamepads;
        static sparse_map<inputmap::method, sparse_map<id_type, action_data>> m_actions;

        static sparse_map<inputmap::method, sparse_map<id_type, axis_data>>  m_axes;

        static sparse_map<id_type,axis_command_queue> m_axes_command_queues;

    };
}
