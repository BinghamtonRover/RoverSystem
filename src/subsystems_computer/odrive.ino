// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Below are some sample configurations.
// You can comment out the default Teensy one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive GND to Arduino GND.

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
//HardwareSerial& odrive_serial = Serial1;

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
// HardwareSerial& odrive_serial = Serial1;

// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX
 SoftwareSerial odrive_serial(8, 9);
 //SoftwareSerial odrive_serial2(10, 11);


// ODrive object
ODriveArduino odrive(odrive_serial);
ODriveArduino odrive2(odrive_serial);

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters

  int axis = 1;
  
  odrive_serial << "w axis1.controller.config.vel_gain 0.01\n";
  odrive_serial << "w axis1.controller.config.vel_integrator_gain 0.05\n";
  odrive_serial << "w axis1.controller.config.control_mode 2\n";
  odrive_serial << "w axis1.controller.input_vel 0\n";
  odrive_serial << "w axis1.controller.config.vel_limit 100\n";
  odrive_serial << "r axis1.config.sensorless_ramp.current\n";  
  float sensorless_ramp_current = odrive.readFloat();
  sensorless_ramp_current *= 2;
  odrive_serial << "w axis1.motor.config.current_lim " << sensorless_ramp_current << "\n"; 
  odrive_serial << "w axis1.motor.config.direction 1\n";
  float pm_flux_linkage = 5.51328895422f / (40.0f * 100.0f);
  odrive_serial << "w axis1.sensorless_estimator.config.pm_flux_linkage " << pm_flux_linkage << "\n";

    odrive_serial << "r axis1.config.sensorless_ramp.current\n";
  Serial << odrive.readFloat() << "\n";
  
  odrive_serial << "w axis1.requested_state AXIS_STATE_SENSORLESS_CONTROL\n";
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();

    // Run calibration sequence
    if (c == '0' || c == '1') {
    }

    // Sinusoidal test move
    if (c == 's') {
      odrive.SetVelocity(1, 100.0f);
      Serial << "Velocity " << odrive.GetVelocity(1) << "\n";
    }

    // Read bus voltage
    if (c == 'b') {
      odrive_serial << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    // print motor positions in a 10s loop
    if (c == 'p') {
      static const unsigned long duration = 10000;
      unsigned long start = millis();
      while(millis() - start < duration) {
        for (int motor = 0; motor < 2; ++motor) {
          odrive_serial << "r axis" << motor << ".encoder.pos_estimate\n";
          Serial << odrive.readFloat() << '\t';
        }
        Serial << '\n';
      }
    }
  }
}