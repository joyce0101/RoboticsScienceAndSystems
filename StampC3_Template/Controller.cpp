#include "Controller.h"
#include <stdio.h>

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;

Controller_c::Controller_c()
  : update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS) {
  reset();
}

void Controller_c::reset() {
  const bool actionActive = experimentPhase == ExperimentPhase::RUNNING ||
                            experimentPhase == ExperimentPhase::SETTLING;
  signal = 0;
  // Measuring Wheel Speed: Encoders
  last_time_ms = 0;
  last_count_left = 0;
  last_count_right = 0;
  speed_left = 0.0f;
  speed_right = 0.0f;
  speed_initialised = false;
  if (!actionActive) {
    // 每次试验只改这一行，选这次跑哪个动作
    // motionStep = MotionStep::STRAIGHT_FWD;
    // motionStep = MotionStep::TURN_LEFT;
    // motionStep = MotionStep::TURN_RIGHT;
    // motionStep = MotionStep::ARC_LEFT;
    // motionStep = MotionStep::ARC_RIGHT;
    motionStep = MotionStep::ROUTE_E5_DUAL_ARC;
    experimentPhase = ExperimentPhase::WAITING;
    motionStartMs = settlingStartMs = routeStartMs = 0;
    routeSegmentIndex = 0;
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
  }
}

void Controller_c::update(Robot_c &robot, RobotWifiAP_c &server) {
  unsigned long now = robot.getMillis();

  if (!update_timer.isReady(now)) {
    return;
  }
  update_timer.resetTimer(now);

  robot.getSurfaceSensors();

  const bool encoder_sample_updated = robot.getEncoders();
  if (encoder_sample_updated) {
    const unsigned long sample_time_ms = robot.getMillis();
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
    measureWheelSpeed(robot, sample_time_ms);
  }

  robot.getPose();

  // While waiting, a button press sets signal to 1 and starts the configured
  // physical behaviour, even without Processing connected.
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
      routeStartMs = now;
    }
    updateMotion(robot, now);
  }

  if (telemetry_timer.isReady(now)) {
    telemetry_timer.resetTimer(now);
    publishTelemetry(robot, server, now);
  }
}

Controller_c::ExperimentPhase Controller_c::getExperimentPhase() const {
  return experimentPhase;
}

void Controller_c::publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms) {
  char line[192];
  const int len = snprintf(
    line,
    sizeof(line),
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
  if (len <= 0) {
    return;
  }
  server.send(line);
  if (Serial) {
    Serial.print(line);
  }
}

void Controller_c::updateMotion(Robot_c &robot, unsigned long now) {
  if (experimentPhase == ExperimentPhase::DONE) {
    robot.setMotorPWM(0, 0);
    return;
  }
  // 按下按钮后先静止1 s，记录静止时的起点
  if (uint32_t(now) - routeStartMs < 1000) {
    robot.setMotorPWM(0, 0);
    return;
  }
  switch (motionStep) {
    // 按 data/forward_steady 与 data/reverse_steady 的 counts/s–PWM 拟合 + r=16mm, B=90mm 计算
    case MotionStep::STRAIGHT_FWD:
      runTimedPwm(robot, uint32_t(now), 58, 57, 1940);
      break;
    case MotionStep::TURN_LEFT:
      // runTimedPwm(robot, uint32_t(now), -41, 40, 1250);  // 180° 第1组 B=90 r=16
      // runTimedPwm(robot, uint32_t(now), -41, 40, 1160);  // 180° 第2组 B=85.8 r=16.36
      runTimedPwm(robot, uint32_t(now), -41, 40, 1220);  // 180° 第3组 B=85.8 r=16.36 +阶跃响应
      break;
    case MotionStep::TURN_RIGHT:
      // runTimedPwm(robot, uint32_t(now), 40, -39, 1270);  // 180° 第1组 B=90 r=16
      // runTimedPwm(robot, uint32_t(now), 40, -39, 1180);  // 180° 第2组 B=85.8 r=16.36
      runTimedPwm(robot, uint32_t(now), 40, -39, 1240);  // 180° 第3组 B=85.8 r=16.36 +阶跃响应
      break;
    case MotionStep::ARC_LEFT:
      // runTimedPwm(robot, uint32_t(now), 48, 68, 1730);  // 第1组 B=90 r=16
      // runTimedPwm(robot, uint32_t(now), 50, 70, 1610);  // 第2组 B=85.8 r=16.36
      runTimedPwm(robot, uint32_t(now), 50, 70, 1660);     // 第3组 B=85.8 r=16.36 +阶跃响应（转角补偿）
      break;
    case MotionStep::ARC_RIGHT:
      // runTimedPwm(robot, uint32_t(now), 69, 47, 1730);  // 第1组 B=90 r=16
      // runTimedPwm(robot, uint32_t(now), 71, 49, 1610);  // 第2组 B=85.8 r=16.36
      runTimedPwm(robot, uint32_t(now), 71, 49, 1690);     // 第3组 B=85.8 r=16.36 +阶跃响应（转角补偿）
      break;
    case MotionStep::ROUTE_E5_DUAL_ARC: {
      // E5 5段双弧拱门路径（包含左弧+右弧+原地转+直行，完全落在 X∈[0,400], Y∈[0,200]）：
      // (0,0,0°) -> S1左弧 -> (200,200,+90°) -> S2右转 -> (200,200,0°)
      //          -> S3右弧 -> (400,0,-90°)   -> S4右转 -> (400,0,-180°) -> S5直行200 -> (200,0,-180°)
      // --- 第1组：名义参数 B=90, r=16 ---
      // static const TimedSegment kRouteDualArc[] = {
      //   { 48,  68, 1730 },  // S1 左弧 R=200, +90°
      //   { 40, -39,  640 },  // S2 原地右转 -90°
      //   { 69,  47, 1730 },  // S3 右弧 R=200, -90°
      //   { 40, -39,  640 },  // S4 原地右转 -90°
      //   { 58,  57, 1110 },  // S5 直行 200 mm
      // };
      // --- 第2组：E0 几何校准 B=85.8, r=16.36 ---
      // static const TimedSegment kRouteDualArc[] = {
      //   { 50,  70, 1610 },  // S1 左弧 R=200, +90°
      //   { 40, -39,  590 },  // S2 原地右转 -90°
      //   { 71,  49, 1610 },  // S3 右弧 R=200, -90°
      //   { 40, -39,  590 },  // S4 原地右转 -90°
      //   { 57,  56, 1110 },  // S5 直行 200 mm
      // };
      // --- 第3组：E0 几何校准 + 阶跃响应补偿（默认启用，段间停顿 3000 ms）---
      static const TimedSegment kRouteDualArc[] = {
        { 50,  70, 1660 },  // S1 左弧 R=200, +90° (+46.6 ms 转角补偿)
        { 40, -39,  650 },  // S2 原地右转 -90° (+63 ms 转角补偿)
        { 71,  49, 1690 },  // S3 右弧 R=200, -90° (+79.4 ms 转角补偿)
        { 40, -39,  650 },  // S4 原地右转 -90°
        { 57,  56, 1170 },  // S5 直行 200 mm (+63 ms 路程补偿)
      };
      runRouteSequence(robot, uint32_t(now), kRouteDualArc, 5, 3000, 2000);
      break;
    }
  }
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
  int64_t delta_left = (int64_t)count_left - (int64_t)last_count_left;
  int64_t delta_right = (int64_t)count_right - (int64_t)last_count_right;
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

  // 时间与计数必须一起更新。
  last_count_left = count_left;
  last_count_right = count_right;
  last_time_ms = now;
}

Controller_c::ExperimentPhase Controller_c::runTimedPwm(
    Robot_c& robot, uint32_t nowMs, int16_t leftPwm, int16_t rightPwm,
    uint32_t durationMs, uint32_t settleMs) {
  // 刚开始：记下开始时间
  if (experimentPhase == ExperimentPhase::WAITING) {
    motionStartMs = nowMs;
    experimentPhase = ExperimentPhase::RUNNING;
  }
  // 没到时间就一直发PWM，到时间就停车
  if (experimentPhase == ExperimentPhase::RUNNING) {
    if (nowMs - motionStartMs >= durationMs) {
      experimentPhase = ExperimentPhase::SETTLING;
      settlingStartMs = nowMs;
    } else {
      robot.setMotorPWM(leftPwm, rightPwm);
    }
  }
  // 停车后继续记录settleMs（单动作默认2000 ms，组合路径段间停顿3000 ms供打点）
  if (experimentPhase == ExperimentPhase::SETTLING) {
    robot.setMotorPWM(0, 0);
    if (nowMs - settlingStartMs >= settleMs) {
      experimentPhase = ExperimentPhase::DONE;
    }
  }
  return experimentPhase;
}

void Controller_c::runRouteSequence(
    Robot_c& robot, uint32_t nowMs,
    const TimedSegment* segments, uint8_t segmentCount,
    uint32_t pauseMs, uint32_t finalSettleMs) {
  if (routeSegmentIndex >= segmentCount) {
    experimentPhase = ExperimentPhase::DONE;
    robot.setMotorPWM(0, 0);
    return;
  }
  const TimedSegment& seg = segments[routeSegmentIndex];
  const bool isLast = (routeSegmentIndex + 1 >= segmentCount);
  const uint32_t settleMs = isLast ? finalSettleMs : pauseMs;
  if (runTimedPwm(robot, nowMs, seg.leftPwm, seg.rightPwm, seg.durationMs, settleMs)
      == ExperimentPhase::DONE) {
    if (!isLast) {
      routeSegmentIndex++;
      experimentPhase = ExperimentPhase::WAITING;
    }
  }
}
