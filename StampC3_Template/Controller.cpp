#include "Controller.h"

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const unsigned long Controller_c::RECOVERY_TIMEOUT_MS;
const uint16_t Controller_c::LINE_THRESHOLD;
constexpr float Controller_c::BASE_PWM;
constexpr float Controller_c::LINE_FOLLOW_GAIN;
constexpr float Controller_c::RECOVERY_LEFT_PWM;
constexpr float Controller_c::RECOVERY_RIGHT_PWM;

Controller_c::Controller_c()
  : deadband_timer(2000),
    saturation_timer(3000),
    straight_timer(2000),
    straight_trial(0),
    step_timer(2000),
    step_trial(0),
    update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS),
    ramp_timer(1000) {
  reset();
}

void Controller_c::reset() {
  signal = 0;
  mode = WAITING;
  recovery_start_ms = 0;
  call_count = 0;
  ramp_pwm = 30;
  // ramp_count = 0;
  ramp_up = true;
  // Measuring Wheel Speed: Encoders
  last_time_ms = 0;
  last_count_left = 0;
  last_count_right = 0;
  speed_left = 0.0f;
  speed_right = 0.0f;
  speed_initialised = false;
  // Deadband: find where motion begins
  deadband_phase = DB_IDLE;
  test_pwm = DB_PWM_MIN;
  deadband_trial = 1;
  deadband_sample = 0;
  deadband_run_start_ms = 0;
  saturation_phase = SAT_IDLE;
  saturation_pwm = SAT_PWM_MIN;
  saturation_trial = 1;
  saturation_sample = 0;
  saturation_phase_start_ms = 0;
  straight_phase = STR_IDLE;
  straight_left_pwm = 0;
  straight_right_pwm = 0;
  straight_sample = 0;
  straight_run_start_ms = 0;
  step_phase = STEP_IDLE;
  step_phase_start_ms = 0;


}

uint8_t Controller_c::getSignal() const {
  return signal;
}

void Controller_c::setSignal(uint8_t requested_signal) {
  if (requested_signal == 0) {
    reset();
    return;
  }

  if (signal == 0) {
    signal = 1;
    mode = FOLLOWING_LINE;
    recovery_start_ms = 0;
    ramp_timer.resetTimer();  //从本次启动开始计时
  }
}

void Controller_c::update(Robot_c &robot, RobotWifiAP_c &server) {

  // Get the current count of milliseconds since the StampC3 was powered on.
  unsigned long now = robot.getMillis();

  // Check if our TaskTimer_c for the general controller update indicates
  // that it is the correct time to operate.  Note, we give the TaskTimer_c
  // the current count in milliseconds.
  // We do this because we don't want to query the robot faster than at 10ms
  // intervals.
  if (!update_timer.isReady(now)) {

    // TaskTimer_c says it is not ready, so we simply end this call to
    // the controller here by calling return.
    return;
  }

  // The TaskTimer_c was ready, so we now reset the TaskTimer_c so it
  // begins counting again.  We now run the remaining controller code.
  update_timer.resetTimer(now);

  // Ask the robot for latest information on surface reflectance sensors.
  robot.getSurfaceSensors();

  // Ask the robot for the latest encoder information, and then use this
  // to update the odometry on board the StampC3 (see OdometryModel.h).
  if (robot.getEncoders()) {
    const unsigned long sample_time_ms = robot.getMillis();
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
    measureWheelSpeed(robot, sample_time_ms, &server);
  }

  // Pose estimation from the Pololu 3Pi robot itself.
  robot.getPose();

  // While waiting, a button press sets signal to 1 and starts the configured
  // physical behaviour, even without Processing connected. The transmitted
  // signal also starts an armed Digital Twin live comparison. This is a start
  // control, not a start/stop toggle; further presses are ignored while signal is 1.
  if (signal == 0) {
    if (robot.isButtonPressed()) {
      setSignal(1);
    }
  }

  // Enable one experiment only. Its header setting selects one wheel per run.
  if (signal == 1) {
    testStepResponse(robot, 60, 55, 1000, 1000, &server);
    // runStraightPWM(robot, 60, 60, 1000, &server);
    // runStraightPWM(robot, 62, 58, 1000, &server);
    // testSaturation(robot);
    // testDeadband(robot);
    // motorStartStopTest(robot, now);
  }

  // Use another TaskTimer_c to limit how often we transmit telemetry data
  // on WiFi and Serial.
  // if (telemetry_timer.isReady(now)) {
  //   telemetry_timer.resetTimer(now);
  //   publishTelemetry(robot, server, now);

  // }
}



bool Controller_c::lineDetected(const Robot_c &robot) const {
  // Note: only using the central sensor to detect a line.
  if (robot.surface.reading[2] >= LINE_THRESHOLD) {
    return true;
  }

  return false;
}

void Controller_c::publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms) {
  server.printf(
    "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
    timestamp_ms,
    odometry.pose.x,
    odometry.pose.y,
    odometry.pose.theta,
    robot.getLeftMotorPWM(),
    robot.getRightMotorPWM(),
    (long)robot.getLeftEncoderCount(),
    (long)robot.getRightEncoderCount(),
    robot.surface.reading[0],
    robot.surface.reading[1],
    robot.surface.reading[2],
    robot.surface.reading[3],
    robot.surface.reading[4],
    signal
  );
  // if ( Serial ) { // If serial is connected
  //   Serial.printf(
  //     "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
  //     timestamp_ms,
  //     odometry.pose.x,
  //     odometry.pose.y,
  //     odometry.pose.theta,
  //     robot.getLeftMotorPWM(),
  //     robot.getRightMotorPWM(),
  //     (long)robot.getLeftEncoderCount(),
  //     (long)robot.getRightEncoderCount(),
  //     robot.surface.reading[0],
  //     robot.surface.reading[1],
  //     robot.surface.reading[2],
  //     robot.surface.reading[3],
  //     robot.surface.reading[4],
  //     signal
  //   );
  // }
}

void Controller_c::runLineFollower(Robot_c &robot, unsigned long now) {
  bool line_detected = lineDetected(robot);

  if (!line_detected && mode == FOLLOWING_LINE) {
    mode = SEARCHING_FOR_LINE;
    recovery_start_ms = now;
  }

  if (mode == SEARCHING_FOR_LINE) {
    if (line_detected) {
      mode = FOLLOWING_LINE;
    } else if (now - recovery_start_ms >= RECOVERY_TIMEOUT_MS) {
      mode = STOPPED;
    }
  }

  if (mode == STOPPED) {
    robot.setMotorPWM(0, 0);
    return;
  }

  if (mode == SEARCHING_FOR_LINE) {
    robot.setMotorPWM(RECOVERY_LEFT_PWM, RECOVERY_RIGHT_PWM);
    return;
  }

  float error = (float)robot.surface.reading[1] - (float)robot.surface.reading[3];
  float turn = error * LINE_FOLLOW_GAIN;
  robot.setMotorPWM(BASE_PWM - turn, BASE_PWM + turn);
}
void Controller_c::motorStartStopTest(Robot_c &robot, unsigned long now) {
  (void)now;
  // This is a simple test to start and stop the motors.  It is not a line
  // following controller.  It is used to test the motors and the motor
  // Call count has exceeded maximum
  const int max_pwm = 80;
  const int step_pwm = 10;
  if(!ramp_timer.isReady()) {
    return;
  }
  // Reset the ramp timer
  ramp_timer.resetTimer();
  if (ramp_up) {
    ramp_pwm += step_pwm;
    if (ramp_pwm >= max_pwm) {
      ramp_pwm = max_pwm;
      ramp_up = false;
    }
  } else {
    ramp_pwm -= step_pwm;
    if (ramp_pwm <= 0) {
      robot.setMotorPWM(0, 0);
      setSignal(0); // Reset signal to 0 to stop the test
      return;
    }
  }
  robot.setMotorPWM(ramp_pwm, ramp_pwm);
}

void Controller_c::measureWheelSpeed(Robot_c &robot, unsigned long now, RobotWifiAP_c *server) {

  const int32_t count_left = robot.getLeftEncoderCount();
  const int32_t count_right = robot.getRightEncoderCount();

  // 第一次成功采集：只保存基准，不计算速度。
  if (!speed_initialised) {
    last_count_left = count_left;
    last_count_right = count_right;
    last_time_ms = now;
    speed_initialised = true;
    return;
  }

  const uint32_t dt_ms = uint32_t(now) - uint32_t(last_time_ms);

  if (dt_ms < 100) {
    return;
  }

  // 用更宽的类型计算差值，避免普通有符号减法溢出。
  int64_t delta_left =
      (int64_t)count_left - (int64_t)last_count_left;
  int64_t delta_right =
      (int64_t)count_right - (int64_t)last_count_right;
  // 编码器跨过32位有符号边界时仍保留正确的小增量。
  const int64_t counter_range = int64_t(1) << 32;
  if (delta_left > INT32_MAX) delta_left -= counter_range;
  if (delta_left < INT32_MIN) delta_left += counter_range;
  if (delta_right > INT32_MAX) delta_right -= counter_range;
  if (delta_right < INT32_MIN) delta_right += counter_range;

  // Share the odometry model's geometry; use each wheel's own radius.
  const float radians_per_count = TWO_PI / odometry.encoderCountsPerRevolution;
  const float counts_per_second_scale = 1000.0f / (float)dt_ms;
  speed_left = (float)delta_left * radians_per_count
      * odometry.leftWheelRadiusMm * counts_per_second_scale;
  speed_right = (float)delta_right * radians_per_count
      * odometry.rightWheelRadiusMm * counts_per_second_scale;

  // Compact CSV: wheel,pwm,trial,phase,phase_ms,speed_mm_s.
  if (signal == 1 && deadband_phase == DB_RUN) {
    deadband_sample++;
    Serial.printf("%c,%d,%d,RUN,%lu,%.3f\n",
                  DB_TEST_RIGHT_WHEEL ? 'R' : 'L', test_pwm, deadband_trial,
                  now - deadband_run_start_ms,
                  DB_TEST_RIGHT_WHEEL ? speed_right : speed_left);
  }

  // Keep settling rows for checking the transient, but average MEASURE only.
  if (signal == 1 &&
      (saturation_phase == SAT_SETTLE || saturation_phase == SAT_MEASURE)) {
    saturation_sample++;
    Serial.printf("%c,%d,%d,%s,%lu,%.3f\n",
                  SAT_TEST_RIGHT_WHEEL ? 'R' : 'L', saturation_pwm, saturation_trial,
                  saturation_phase == SAT_SETTLE ? "SETTLE" : "MEASURE",
                  now - saturation_phase_start_ms,
                  SAT_TEST_RIGHT_WHEEL ? speed_right : speed_left);
  }

  if (signal == 1 && straight_phase == STR_RUN) {
    straight_sample++;
    const unsigned long run_ms = now - straight_run_start_ms;
    const char* phase_label = (run_ms <= 500UL) ? "SETTLE" : "MEASURE";
    const float theta_deg = odometry.pose.theta * (360.0f / (float)TWO_PI);
    Serial.printf("STRAIGHT,%d,%d,%d,%s,%lu,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                  straight_trial, straight_left_pwm, straight_right_pwm,
                  phase_label, run_ms, speed_left, speed_right,
                  odometry.pose.x, odometry.pose.y, theta_deg);
    if (server != nullptr) {
      server->printf("STRAIGHT,%d,%d,%d,%s,%lu,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                     straight_trial, straight_left_pwm, straight_right_pwm,
                     phase_label, run_ms, speed_left, speed_right,
                     odometry.pose.x, odometry.pose.y, theta_deg);
    }
  }

  // 时间与计数必须一起更新。
  last_count_left = count_left;
  last_count_right = count_right;
  last_time_ms = now;
}
void Controller_c::testDeadband(Robot_c &robot) {
  // 第一次进入：先停车，再开始休息计时
  switch (deadband_phase) {
    case DB_IDLE:
      if(!robot.setMotorPWM(0, 0)) {
        return; // Wait until the stop command is sent successfully
      }
      deadband_timer.setIntervalMS(2000); // Set rest interval to 2 seconds
      deadband_timer.resetTimer();
      deadband_phase = DB_REST;
      speed_initialised = false; // Reset speed measurement for the next run
      break;
    // 每次实验前，保持停车休息2秒
    case DB_REST:
      if (deadband_timer.isReady()) {
        if(!robot.setMotorPWM(DB_TEST_RIGHT_WHEEL ? 0 : test_pwm,
                              DB_TEST_RIGHT_WHEEL ? test_pwm : 0)) {
          deadband_phase = DB_IDLE; // Stay in rest phase until the command is sent successfully
          return; // Wait until the start command is sent successfully
        }
        deadband_sample = 0;
        speed_initialised = false; // Reset speed measurement for the next run
        deadband_phase = DB_RUN;
        deadband_run_start_ms = robot.getMillis();
        deadband_timer.setIntervalMS(3000); // Set run interval to 3 second
        deadband_timer.resetTimer();
      }
      break;
    case DB_RUN:
      if(!deadband_timer.isReady()) {
        return; // Wait until the run interval is complete
      }
      // Stop the robot after the run interval
      if(!robot.setMotorPWM(0, 0)) {
        return; // Wait until the stop command is sent successfully
      }
      deadband_trial++;
      // If all trials for the current PWM are done, move to the next PWM
      if (deadband_trial > DB_REPEATS) {
        deadband_trial = 1;
        test_pwm += DB_PWM_STEP;
      }
      // If all PWM values have been tested, reset to idle
      if (test_pwm > DB_PWM_MAX) {
        setSignal(0); // Reset signal to 0 to stop the test
        return;
      }
      deadband_sample=0;
      speed_initialised = false; // Reset speed measurement for the next run
      deadband_phase = DB_REST;
      deadband_timer.setIntervalMS(2000); // Set rest interval to 2 seconds
      deadband_timer.resetTimer();
      break;
  }
}

void Controller_c::testSaturation(Robot_c &robot) {
  switch (saturation_phase) {
    case SAT_IDLE:
      if (!robot.setMotorPWM(0, 0)) return;
      saturation_timer.setIntervalMS(3000); // Stopped rest between powered trials.
      saturation_timer.resetTimer();
      saturation_phase = SAT_REST;
      speed_initialised = false;
      break;

    case SAT_REST:
      if (!saturation_timer.isReady()) return;
      if (!robot.setMotorPWM(SAT_TEST_RIGHT_WHEEL ? 0 : saturation_pwm,
                             SAT_TEST_RIGHT_WHEEL ? saturation_pwm : 0)) {
        saturation_phase = SAT_IDLE; // Retry stopping before another start.
        return;
      }
      saturation_sample = 0;
      speed_initialised = false;
      saturation_phase_start_ms = robot.getMillis();
      saturation_timer.setIntervalMS(500);
      saturation_timer.resetTimer();
      saturation_phase = SAT_SETTLE;
      break;

    case SAT_SETTLE:
      if (!saturation_timer.isReady()) return;
      saturation_sample = 0;
      // A fresh baseline excludes settling from the first measurement window.
      speed_initialised = false;
      saturation_phase_start_ms = robot.getMillis();
      saturation_timer.setIntervalMS(1500);
      saturation_timer.resetTimer();
      saturation_phase = SAT_MEASURE;
      break;

    case SAT_MEASURE:
      if (!saturation_timer.isReady()) return;
      if (!robot.setMotorPWM(0, 0)) return; // Retry stop on the next cycle.
      saturation_trial++;
      if (saturation_trial > SAT_REPEATS) {
        saturation_trial = 1;
        saturation_pwm += SAT_PWM_STEP;
      }
      if (saturation_pwm > SAT_PWM_MAX) {
        setSignal(0);
        return;
      }
      saturation_sample = 0;
      speed_initialised = false;
      saturation_timer.setIntervalMS(3000);
      saturation_timer.resetTimer();
      saturation_phase = SAT_REST;
      break;
  }
}

void Controller_c::runStraightPWM(Robot_c &robot, int leftPwm, int rightPwm, unsigned long durationMs,
                                  RobotWifiAP_c *server) {
  switch (straight_phase) {
    case STR_IDLE:
      if (!robot.setMotorPWM(0, 0)) {
        return;
      }
      straight_trial++;
      straight_left_pwm = leftPwm;
      straight_right_pwm = rightPwm;
      straight_sample = 0;
      speed_initialised = false;
      straight_timer.setIntervalMS(2000);
      straight_timer.resetTimer();
      straight_phase = STR_REST;
      break;

    case STR_REST:
      if (!straight_timer.isReady()) {
        return;
      }
      if (!robot.setMotorPWM((float)leftPwm, (float)rightPwm)) {
        return;
      }
      odometry.reset();
      odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
      straight_left_pwm = leftPwm;
      straight_right_pwm = rightPwm;
      straight_sample = 0;
      speed_initialised = false;
      straight_run_start_ms = robot.getMillis();
      straight_timer.setIntervalMS(durationMs);
      straight_timer.resetTimer();
      straight_phase = STR_RUN;
      break;

    case STR_RUN:
      if (!straight_timer.isReady()) {
        return;
      }
      if (!robot.setMotorPWM(0, 0)) {
        return;
      }
      {
        const float theta_deg = odometry.pose.theta * (360.0f / (float)TWO_PI);
        const unsigned long run_ms = robot.getMillis() - straight_run_start_ms;
        Serial.printf("SUMMARY,%d,%d,%d,END,%lu,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                      straight_trial, straight_left_pwm, straight_right_pwm,
                      run_ms, speed_left, speed_right,
                      odometry.pose.x, odometry.pose.y, theta_deg);
        if (server != nullptr) {
          server->printf("SUMMARY,%d,%d,%d,END,%lu,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                         straight_trial, straight_left_pwm, straight_right_pwm,
                         run_ms, speed_left, speed_right,
                         odometry.pose.x, odometry.pose.y, theta_deg);
        }
      }
      setSignal(0);
      break;
  }
}

void Controller_c::testStepResponse(Robot_c &robot, int leftPwm, int rightPwm,
                                    unsigned long runMs, unsigned long coastMs,
                                    RobotWifiAP_c *server) {
  const unsigned long now = robot.getMillis();
  const long count_l = (long)robot.getLeftEncoderCount();
  const long count_r = (long)robot.getRightEncoderCount();

  switch (step_phase) {
    case STEP_IDLE:
      robot.setMotorPWM(0, 0);
      step_trial++;
      step_timer.setIntervalMS(2000);
      step_timer.resetTimer(now);
      step_phase = STEP_REST;
      break;

    case STEP_REST:
      if (!step_timer.isReady(now)) return;
      robot.setMotorPWM((float)leftPwm, (float)rightPwm);
      step_phase_start_ms = now;
      step_timer.setIntervalMS(runMs);
      step_timer.resetTimer(now);
      step_phase = STEP_UP;
      Serial.printf("STEP,%d,UP,0,%d,%d,%ld,%ld\n",
                    step_trial, leftPwm, rightPwm, count_l, count_r);
      if (server != nullptr) {
        server->printf("STEP,%d,UP,0,%d,%d,%ld,%ld\n",
                       step_trial, leftPwm, rightPwm, count_l, count_r);
      }
      break;

    case STEP_UP:
      Serial.printf("STEP,%d,UP,%lu,%d,%d,%ld,%ld\n",
                    step_trial, now - step_phase_start_ms,
                    leftPwm, rightPwm, count_l, count_r);
      if (server != nullptr) {
        server->printf("STEP,%d,UP,%lu,%d,%d,%ld,%ld\n",
                       step_trial, now - step_phase_start_ms,
                       leftPwm, rightPwm, count_l, count_r);
      }
      if (step_timer.isReady(now)) {
        robot.setMotorPWM(0, 0);
        step_phase_start_ms = now;
        step_timer.setIntervalMS(coastMs);
        step_timer.resetTimer(now);
        step_phase = STEP_COAST;
        Serial.printf("STEP,%d,COAST,0,0,0,%ld,%ld\n",
                      step_trial, count_l, count_r);
        if (server != nullptr) {
          server->printf("STEP,%d,COAST,0,0,0,%ld,%ld\n",
                         step_trial, count_l, count_r);
        }
      }
      break;

    case STEP_COAST:
      Serial.printf("STEP,%d,COAST,%lu,0,0,%ld,%ld\n",
                    step_trial, now - step_phase_start_ms, count_l, count_r);
      if (server != nullptr) {
        server->printf("STEP,%d,COAST,%lu,0,0,%ld,%ld\n",
                       step_trial, now - step_phase_start_ms, count_l, count_r);
      }
      if (step_timer.isReady(now)) {
        robot.setMotorPWM(0, 0);
        setSignal(0);
      }
      break;
  }
}
