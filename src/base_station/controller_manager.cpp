#include "controller_manager.hpp"

#include <iostream>

ControllerManager* ControllerManager::main_input_manager = nullptr;

ControllerManager::ControllerManager() {
    if (main_input_manager != nullptr) {
        std::cerr << "WARNING! Session has duplicate instances of ControllerManager! This behavior is undefined!\n";
    }
    main_input_manager = this;
    for (unsigned int i = 0; i < devices.size(); i++) {
        devices[i].set_joystick_id(i);
    }
}

void ControllerManager::init(/*sc::SimpleConfig& cfg*/) {

    for (Controller& c : devices) {
        c.update_device();
    }

    glfwSetJoystickCallback(glfw_joystick_callback);
}

void ControllerManager::scan_devices() {
    for (Controller& c : devices) {
        c.update_device();
    }
}

void ControllerManager::update_controls() {
    for (Controller& c : devices) {
        if (c.present()) {
            c.update_axes();
        }
    }
}

void ControllerManager::glfw_joystick_callback(int joystick_id, int event) {
    main_input_manager->devices[joystick_id].update_device();
}
