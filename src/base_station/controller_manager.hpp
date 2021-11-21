#ifndef CONTROLLER_MANAGER
#define CONTROLLER_MANAGER

//#include "../simple_config/simpleconfig.h"
#include "controller.hpp"

#include <GLFW/glfw3.h>
#include <array>
#include <vector>
#include <functional>


class ControllerManager {
    public:
        ControllerManager();
        void init(/*sc::SimpleConfig& cfg*/);
        void update_controls();
        void scan_devices();

        static void glfw_joystick_callback(int joystick_id, int event);

        // GLFW supports up to 16 joystick devices
        std::array<Controller, GLFW_JOYSTICK_LAST + 1> devices;
    private:
        static ControllerManager* main_input_manager;

};

#endif
