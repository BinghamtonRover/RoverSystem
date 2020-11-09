#include "../network/network.hpp"
#include "../simple_config/simpleconfig.h"
#include "../util/util.hpp"
#include "../logger/logger.hpp"

#include <vector>


const uint8_t SUSPENSION_I2C_ADDR = 0x01;
const uint8_t ARM_I2C_ADDR = 0x02;
const uint8_t GRIPPER_I2C_ADDR = 0x03;

const int SUSPENSION_UPDATE_INTERVAL = 1000 / 15;

const int SUSPENSION_CONNECT_TRIES = 5;

const int NETWORK_UPDATE_INTERVAL = 1000 / 2;

const int SUBSYSTEM_SEND_INTERVAL = 1000 * 5;

const int LOCATION_SEND_INTERVAL = 1000;

const int LIDAR_UPDATE_INTERVAL = 1000 / 15;

const int TICK_INTERVAL = 1000;

struct Config
{
    int base_station_port;
    int rover_port;
    int video_port;

    char base_station_multicast_group[16];
    char rover_multicast_group[16];
    char video_multicast_group[16];
    char interface[16];

    // For now, this is dynamically-sized.
    char* gps_serial_id;
};

class Session{
private:
public:
    network::ModeMessage::Mode mode;
    util::Clock global_clock;
    Config config;

    bool suspension_inited;
    bool arm_inited;
    bool gripper_inited;

    network::Feed r_feed;
    network::Feed bs_feed;
    network::Feed v_feed;

    util::Timer location_send_timer;
    util::Timer tick_timer;
    util::Timer network_update_timer;
    util::Timer subsystem_send_timer;
    util::Timer suspension_update_timer;
    util::Timer lidar_update_timer;

    uint32_t ticks;

    network::MovementMessage last_movement_message = { 0, 0 };

    std::vector<long> lidar_points;

    Session();
    ~Session();

    void load_config(const char* filename);
    //void stderr_handler(logger::Level level, std::string message);
};