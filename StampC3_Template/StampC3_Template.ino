#include <Wire.h>

#include "RobotWifiAP.h"

/** @brief WiFi access-point and TCP server used for optional telemetry. */
RobotWifiAP_c server("myAP", "myPassword", 80);

#include "Robot.h"
#include "RobotIMU.h"
#include "Controller.h"


/** @brief Hardware-facing interface to the Pololu robot. */
Robot_c robot;
/** @brief Direct interface to the 3Pi+ accelerometer, gyro, and magnetometer. */
RobotIMU_c imu;

// Select the individual motor experiment in Controller_c::update().
Controller_c controller;


/** @brief Initialises serial, WiFi, IMU, and the robot hardware before button start. */
void setup() {

  Serial.begin(115200);
  server.begin();

  robot.initialise();
//  robot.initialise("team name");



  // Robot_c starts the shared I2C bus before RobotIMU_c uses it. IMU readings
  // come directly from the sensor ICs and do not pass through the middleware.
  if (!imu.initialise() ) {
    Serial.println("Warning: LSM6DS33 IMU was not detected.");
    robot.setLED( 90, 50, 0, 100 ); // orange warning
  }

}


/** @brief Maintains telemetry and runs the button-started motor experiment. */
void loop() {
  server.update();
  controller.update(robot, server);
}
