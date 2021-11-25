#ifndef CONTROLLER
#define CONTROLLER

#include <vector>
#include <functional>

class JoystickAxis {
    public:

        constexpr static float AXIS_MIN = -1.0F;
        constexpr static float AXIS_MAX = 1.0F;
        constexpr static float AXIS_RANGE = AXIS_MAX - AXIS_MIN;

        void begin_calibration();
        void finish_calibration();
        void set_action(std::function<void(float)>& action);

        // Calibrate the joystick around a center value with a dead zone
        void recenter(float center, float dead_zone);

        inline void update(float x) { if (!calibrating) x = translate(x); if (action) action(x); }

        // Scale the raw controller input to the calibrated range for this joystick
        float translate(float);
    private:
        // Joystick calibration:
        // Min and max show the range achievable by this axis. Typically all axes cover the entire [-1, 1]
        float minimum = AXIS_MIN;
        float maximum = AXIS_MAX;
        // Two-sided scaling: joysticks usually don't center exactly at 0, so scale around a calibrated center
        // There are two separate scales since one side will cover a larger range of values
        float left_scale = 1.0F;
        float right_scale = 1.0F;
        float center = 0.0F;
        float dead_zone = 0.0F;

        bool calibrating = false;

        void calibrate_action(float);
        std::function<void(float)> action;
        std::function<void(float)> displaced_action;
};

class Controller {
    public:
        void set_joystick_id(int id);
        int get_joystick_id() const;
        bool present() const;
        void update_device();
        void update_axes();
        JoystickAxis& operator[](std::size_t index);
        int count_axes() const;
    private:
        int joystick_id;
        std::vector<JoystickAxis> axes;
        bool _present = false;
};

#endif
