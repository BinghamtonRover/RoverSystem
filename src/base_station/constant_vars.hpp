#ifndef CONST_VARS
#define CONST_VARS

// Default angular resolution (vertices / radian) to use when drawing circles.
constexpr float ANGULAR_RES = 10.0f;

// We know that our base station will have this resolution.
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;

// For control smoothing.
const float CONTROL_ALPHA = 30;

// Speed for the DPAD up/down.
const int16_t JOINT_DRIVE_SPEED = 100;

// Send movement updates x times per second.
const int MOVEMENT_SEND_INTERVAL = 1000 / 15;

// Update network statistics once per second.
const int NETWORK_STATS_INTERVAL = 1000;

const int ARM_SEND_INTERVAL = 1000 / 9;

const int LOG_VIEW_WIDTH = 572;
const int LOG_VIEW_HEIGHT = 458;

const int PRIMARY_FEED_WIDTH = 1298;
const int PRIMARY_FEED_HEIGHT = 730;

const int SECONDARY_FEED_WIDTH = 533;
const int SECONDARY_FEED_HEIGHT = 300;


#endif