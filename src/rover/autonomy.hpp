#include <vector>

#include "suspension.hpp"
#include "../autonomy/autonomy.hpp"
#include "../network/network.hpp"

namespace autonomy {


void update(float latitude, float longitude, float heading, std::vector<long> lidar_points);

network::AutonomyStatusMessage::Status get_status();

StepResult step();

struct SuspensionInstructions {
    suspension::Direction left_direction;
    suspension::Direction right_direction;

    uint8_t left_speed;
    uint8_t right_speed;
};

void get_suspension_instructions(SuspensionInstructions* out_instructions);

} // namespace autonomy
