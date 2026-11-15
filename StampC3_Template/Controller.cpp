#include "Controller.h"
#include <math.h>
#include <float.h>

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const unsigned long Controller_c::RECOVERY_TIMEOUT_MS;
const uint16_t Controller_c::LINE_THRESHOLD;
constexpr float Controller_c::BASE_PWM;
constexpr float Controller_c::LINE_FOLLOW_GAIN;
constexpr float Controller_c::RECOVERY_LEFT_PWM;
constexpr float Controller_c::RECOVERY_RIGHT_PWM;
constexpr float Controller_c::PWM_SPEED_GAIN;
constexpr float Controller_c::PWM_SPEED_INTERCEPT;
const int16_t Controller_c::PWM_DEADBAND;
const int16_t Controller_c::MAX_PWM;
const uint32_t Controller_c::ENCODER_TIMEOUT_MS;

Controller_c::Controller_c()
  : deadband_timer(2000),
    saturation_timer(3000),
    update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS),
    ramp_timer(1000) {
  reset();
}

void Controller_c::reset() {
  const bool actionActive = experimentPhase == ExperimentPhase::RUNNING ||
                            experimentPhase == ExperimentPhase::SETTLING;
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
  encoder_sample_updated = false;
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
  if (!actionActive) {
    motionStep = MotionStep::ARC_1;
    experimentPhase = ExperimentPhase::WAITING;
    motionStartMs = settlingStartMs = lastEncoderMs = 0;
    motionBaselineInitialized = false;
  }

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
  encoder_sample_updated = robot.getEncoders();
  if (encoder_sample_updated) {
    const unsigned long sample_time_ms = robot.getMillis();
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
    measureWheelSpeed(robot, sample_time_ms);
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

  // Dispatch the configured motion after the start signal.
  if (signal == 1) {
    if (!motionBaselineInitialized && experimentPhase == ExperimentPhase::WAITING &&
        encoder_sample_updated) {
      // 整个动作序列仅在这里建立一次基线，动作切换不清零位姿。
      odometry.reset();
      odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
      last_count_left = robot.getLeftEncoderCount();
      last_count_right = robot.getRightEncoderCount();
      last_time_ms = now;
      speed_left = speed_right = 0.0f;
      speed_initialised = true;
      motionBaselineInitialized = true;
    }
    updateMotion(robot, now);
    // testDeadband(robot);
    // testSaturation(robot);

  }

  // Use another TaskTimer_c to limit how often we transmit telemetry data
  // on WiFi and Serial.
  if (telemetry_timer.isReady(now)) {
    telemetry_timer.resetTimer(now);
    publishTelemetry(robot, server, now);

  }
}



Controller_c::ExperimentPhase Controller_c::getExperimentPhase() const {
  return experimentPhase;
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

void Controller_c::updateMotion(Robot_c &robot, unsigned long now) {
  // 完成/故障由执行状态表示；motionStep只选择当前的具体动作。
  if (experimentPhase == ExperimentPhase::DONE || experimentPhase == ExperimentPhase::FAULT) {
    stopMotion(robot);
    return;
  }
  switch (motionStep) {
    case MotionStep::ARC_1:
      if (runArc(robot, uint32_t(now), 200.0f, -float(TWO_PI) / 4.0f, 180.0f)
          == ExperimentPhase::DONE) {
        // 当前直线未实现，保持已有路线；接入后可改为STRAIGHT_1。
        // motionStep = MotionStep::ARC_2;
        // experimentPhase = ExperimentPhase::WAITING;
      }
      break;
    case MotionStep::STRAIGHT_1:
      // 该步尚未实现：不能跳过它，也不能提前执行后面的圆弧。
      // 实现runStraight后，在完成时切换至ARC_2并将阶段置回WAITING。
      experimentPhase = ExperimentPhase::FAULT;
      stopMotion(robot);
      return;
    // case MotionStep::ARC_2:
    //   runArc(robot, uint32_t(now), 100.0f, -float(TWO_PI) / 4.0f, 180.0f);
    //   break;
  }
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

void Controller_c::measureWheelSpeed(Robot_c &robot, unsigned long now) {

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

  // time_ms,left_count,right_count,dt_ms,left_mm_s,right_mm_s
  // Serial.printf("%lu,%ld,%ld,%lu,%.3f,%.3f\n",
  //               now, (long)count_left, (long)count_right,
  //               dt_ms, speed_left, speed_right);
  if (signal == 1 && deadband_phase == DB_RUN) {
    deadband_sample++;
    const unsigned long run_ms = now - deadband_run_start_ms;
    Serial.printf(
      "%lu,L,%d,%d,%d,%lu,%ld,%ld,%lu,%.3f,%.3f\n",
      now,
      test_pwm,
      deadband_trial,
      deadband_sample,
      run_ms,
      (long)count_left,
      (long)count_right,
      dt_ms,
      speed_left,
      speed_right);
  }

  // Keep settling rows for checking the transient, but average MEASURE only.
  if (signal == 1 &&
      (saturation_phase == SAT_SETTLE || saturation_phase == SAT_MEASURE)) {
    saturation_sample++;
    Serial.printf("%lu,L,%d,%d,%d,%s,%lu,%ld,%ld,%lu,%.3f,%.3f\n",
                  now, saturation_pwm, saturation_trial, saturation_sample,
                  saturation_phase == SAT_SETTLE ? "SETTLE" : "MEASURE",
                  now - saturation_phase_start_ms,
                  (long)count_left, (long)count_right, (unsigned long)dt_ms,
                  speed_left, speed_right);
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
        if(!robot.setMotorPWM(test_pwm, 0)) {
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
      if (!robot.setMotorPWM(saturation_pwm, 0)) {
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

bool Controller_c::calculateArcMotion(
    float radiusMm, float angleRad, float centreSpeedMmPerSec, float wheelSeparationMm,
    float& leftSpeed, float& rightSpeed, uint32_t& durationMs) {
  if (!isfinite(radiusMm) || radiusMm <= 0 || !isfinite(angleRad) || angleRad == 0 ||
      !isfinite(centreSpeedMmPerSec) || centreSpeedMmPerSec <= 0 ||
      !isfinite(wheelSeparationMm) || wheelSeparationMm <= 0) return false;
  const double ratio = copysign(double(wheelSeparationMm) / (2.0 * radiusMm), double(angleRad));
  const double left = centreSpeedMmPerSec * (1.0 - ratio);
  const double right = centreSpeedMmPerSec * (1.0 + ratio);
  const double duration = floor(fabs(double(angleRad)) * radiusMm
                                / centreSpeedMmPerSec * 1000.0 + 0.5);
  if (!isfinite(left) || !isfinite(right) || fabs(left) > FLT_MAX || fabs(right) > FLT_MAX ||
      !isfinite(duration) || duration < 1 || duration > 2147483647.0) return false;
  leftSpeed = float(left);
  rightSpeed = float(right);
  durationMs = uint32_t(duration);
  return true;
}

bool Controller_c::stopMotion(Robot_c& robot) {
  if (robot.setMotorPWM(0, 0)) return true;
  experimentPhase = ExperimentPhase::FAULT;
  return robot.setMotorPWM(0, 0);
}

Controller_c::ExperimentPhase Controller_c::runArc(
    Robot_c& robot, uint32_t nowMs, float radiusMm, float angleRad, float centreSpeedMmPerSec) {
  float leftSpeed = 0, rightSpeed = 0;
  uint32_t durationMs = 0;
  if (experimentPhase == ExperimentPhase::WAITING || experimentPhase == ExperimentPhase::RUNNING) {
    int16_t leftPwm, rightPwm;
    if (!calculateArcMotion(radiusMm, angleRad, centreSpeedMmPerSec, odometry.wheelSeparationMm,
                            leftSpeed, rightSpeed, durationMs) ||
        !speedToPwm(leftSpeed, leftPwm) || !speedToPwm(rightSpeed, rightPwm)) {
      experimentPhase = ExperimentPhase::FAULT;
    }
  }
  if (experimentPhase == ExperimentPhase::WAITING) {
    if (!encoder_sample_updated) {
      experimentPhase = ExperimentPhase::FAULT;
    } else {
      motionStartMs = lastEncoderMs = nowMs;
      experimentPhase = ExperimentPhase::RUNNING;
    }
  }
  if (experimentPhase == ExperimentPhase::RUNNING) {
    if (uint32_t(nowMs - motionStartMs) >= durationMs) {
      experimentPhase = ExperimentPhase::SETTLING;
      settlingStartMs = nowMs;
    } else if (uint32_t(nowMs - lastEncoderMs) > ENCODER_TIMEOUT_MS) {
      experimentPhase = ExperimentPhase::FAULT;
    } else {
      if (encoder_sample_updated) lastEncoderMs = nowMs;
      if (!setWheelSpeeds(robot, leftSpeed, rightSpeed)) experimentPhase = ExperimentPhase::FAULT;
    }
  }
  if (experimentPhase != ExperimentPhase::RUNNING) {
    if (!stopMotion(robot)) return experimentPhase;
    if (experimentPhase == ExperimentPhase::SETTLING &&
        uint32_t(nowMs - settlingStartMs) >= 2000UL) experimentPhase = ExperimentPhase::DONE;
  }
  return experimentPhase;
}

// 公共电机工具：供圆弧、直线和其他运动控制复用。
bool Controller_c::speedToPwm(float speedMmPerSec, int16_t& pwm) {
  if (!isfinite(speedMmPerSec)) return false;
  if (speedMmPerSec == 0) { pwm = 0; return true; }
  // Match the float input's representable endpoint at PWM 200.
  const float maximum = float(double(PWM_SPEED_GAIN) * MAX_PWM + PWM_SPEED_INTERCEPT);
  if (fabs(double(speedMmPerSec)) > maximum) return false;
  // |PWM| = (|v| + 28.861902) / 3.722106; reverse shares the magnitude model.
  // No additional deadband offset. Only absorb rounding at the float endpoint.
  const double magnitude = fmin(double(MAX_PWM),
      (fabs(double(speedMmPerSec)) - PWM_SPEED_INTERCEPT) / PWM_SPEED_GAIN);
  if (!isfinite(magnitude)) return false;
  const int16_t rounded = int16_t(floor(magnitude + 0.5));
  // 不抬高PWM改变圆弧比例；低于可用驱动范围时拒绝执行。
  if (rounded <= PWM_DEADBAND) return false;
  pwm = speedMmPerSec > 0 ? rounded : -rounded;
  return true;
}

bool Controller_c::setWheelSpeeds(
    Robot_c& robot, float leftSpeedMmPerSec, float rightSpeedMmPerSec) {
  int16_t leftPwm = 0, rightPwm = 0;
  if (!speedToPwm(leftSpeedMmPerSec, leftPwm) ||
      !speedToPwm(rightSpeedMmPerSec, rightPwm)) return false;
  return robot.setMotorPWM(leftPwm, rightPwm);
}
