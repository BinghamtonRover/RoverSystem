#include "controller.hpp"

#include <GLFW/glfw3.h>
#include <limits>
#include <cmath>

void JoystickAxis::set_action(std::function<void(float)>& new_action) {
	if (calibrating) {
		displaced_action = new_action;
	} else {
		action = new_action;
	}
}

void JoystickAxis::begin_calibration() {
	// Floating point math for using infinity, division by 0, etc. is well defined.
	// We can use these values during calculation but must check they were updated resuming regular actions

	if (!calibrating) {
		calibrating = true;

		// Save the user action so we can restore it after calibration
		displaced_action = action;
		minimum = std::numeric_limits<float>::infinity();
		maximum = -std::numeric_limits<float>::infinity();
		left_scale = AXIS_RANGE / (2 * (maximum - minimum));
		right_scale = left_scale;
		action = std::bind(&JoystickAxis::calibrate_action, this, std::placeholders::_1);
	}
}

void JoystickAxis::recenter(float center, float dead_zone) {
	// Cap dead zone at a reasonable 50% to prevent division by 0
	this->center = center;
	this->dead_zone = dead_zone;
	if (center > AXIS_MIN && center < AXIS_MAX && dead_zone < AXIS_RANGE / 4) {
		left_scale = (AXIS_RANGE / 2.0F) / ((center - dead_zone) - minimum);
		right_scale = (AXIS_RANGE / 2.0F) / (maximum - (center + dead_zone));
	}
}

void JoystickAxis::finish_calibration() {
	if (std::isnan(left_scale)) {
		left_scale = 1.0F;
	}
	if (std::isnan(right_scale)) {
		right_scale = 1.0F;
	}
	action = displaced_action;
	calibrating = false;
}

void JoystickAxis::calibrate_action(float x) {
	if (calibrating) {
		if (x < minimum) minimum = x;
		if (x > maximum) maximum = x;
		left_scale = (AXIS_RANGE / 2.0F) / ((center - dead_zone) - minimum);
		right_scale = (AXIS_RANGE / 2.0F) / (maximum - (center + dead_zone));
	}
}

float JoystickAxis::translate(float x) {
	if (x < center - dead_zone) {
		x = left_scale * (x - minimum) + AXIS_MIN;
	} else if (x > center + dead_zone) {
		x = right_scale * (x - center - dead_zone);
	} else {
		x = 0.0F;
	}

	//return scale * (x - minimum) + AXIS_MIN;
	if (x > AXIS_MAX) x = AXIS_MAX;
	else if (x < AXIS_MIN) x = AXIS_MIN;

	return x;
}

int Controller::count_axes() const {
	return static_cast<int>(axes.size());
}

JoystickAxis& Controller::operator[](std::size_t index) {
	return axes[index];
}

void Controller::set_joystick_id(int id) {
    joystick_id = id;
}

int Controller::get_joystick_id() const {
    return joystick_id;
}

bool Controller::present() const {
    return _present;
}

void Controller::update_device() {
    _present = glfwJoystickPresent(joystick_id);
    if (_present) {
        int axis_count;
        glfwGetJoystickAxes(joystick_id, &axis_count);

        if (axes.size() != axis_count) {
            axes.resize(axis_count);
        }

    }
}

void Controller::update_axes() {
    int count;
    const float* values = glfwGetJoystickAxes(joystick_id, &count);
    if (count != axes.size()) {
        update_device();
    }

    while (count) {
        count--;
        axes[count].update(values[count]);
    }
}
