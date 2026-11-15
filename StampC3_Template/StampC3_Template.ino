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

// 动作参数和顺序在Controller_c::updateMotion()中，通过runArc()直接调用。
// 按实测拟合公式|PWM|=(|v|+28.861902)/3.722106换算；实测轮速仅用于观测。
// 原圆弧内轮目标66 mm/s映射到PWM25，会被下述死区保护拒绝执行。
// 死区PWM30单独检查，非零目标映射到|PWM|<=30时拒绝配置。
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


/** @brief 按键启动Controller中的动作序列；每段完成后停车观察2秒。 */
void loop() {
  server.update();
  controller.update(robot, server);
}
