/**
 * @brief Timed-PWM motion experiment controller shared conceptually with StampC3.
 * @details The controller reads cached robot values at a fixed interval and
 * commands requested PWM values. It is intentionally separate from the
 * physical-response models so those models can be revised independently.
 */
class Controller_c {
  /** @brief Minimum interval between control/acquisition updates, in ms. */
  final int UPDATE_INTERVAL_MS = 10;
  /** @brief Interval reserved for telemetry work, in ms. */
  final int TELEMETRY_INTERVAL_MS = 50;
  int signal = 0;
  // 单动作试验：执行状态和动作编号，和StampC3那边对应
  final int PHASE_WAITING = 0, PHASE_RUNNING = 1, PHASE_SETTLING = 2, PHASE_DONE = 3;
  final int STRAIGHT_FWD = 0, TURN_LEFT = 1, TURN_RIGHT = 2, ARC_LEFT = 3, ARC_RIGHT = 4;
  int phase = PHASE_WAITING;
  int motionStep = ARC_RIGHT;
  boolean routeStarted = false;
  long routeStartMs = 0, motionStartMs = 0, settlingStartMs = 0;

  /** @brief TaskTimer_c to keep aligned with code from Arduino platform */
  TaskTimer_c update_timer = new TaskTimer_c( UPDATE_INTERVAL_MS );
  TaskTimer_c telemetry_timer = new TaskTimer_c( TELEMETRY_INTERVAL_MS );

  // 和StampC3一样，创建时先调用reset()，让reset()里选的动作生效
  Controller_c() {
    reset();
  }
  
  /** @brief Returns the controller to its inactive waiting state. */
  void reset() {
    signal = 0;
    // 每次试验只改这一行，选这次跑哪个动作
    // motionStep = STRAIGHT_FWD;
    // motionStep = TURN_LEFT;
    // motionStep = TURN_RIGHT;
    // motionStep = ARC_LEFT;
    motionStep = ARC_RIGHT;
    phase = PHASE_WAITING;
    routeStarted = false;
    routeStartMs = 0;
    motionStartMs = 0;
    settlingStartMs = 0;
  }

  /** @return The latched start signal: 0 while waiting, 1 once started. */
  int getSignal() {
    return signal;
  }

  /**
   * @brief Updates the latched start signal.
   * @param requested Zero resets the controller; a non-zero value starts it.
   */
  void setSignal(int requested) {
    if (requested == 0) {
      reset();
      return;
    }
    if (signal == 0) {
      signal = 1;
    }
  }

  /**
   * @brief Runs at most one periodic controller update.
   * @param robot Simulated robot supplying cached readings and accepting PWM.
   * @note No update occurs until UPDATE_INTERVAL_MS has elapsed in simulation
   * time.
   */
  void update(Robot_c robot) {
    long now = robot.getMillis();

    if ( !update_timer.isReady(now) ) {
      return;
    }

    update_timer.resetTimer( now );

    robot.getSurfaceSensors();
    robot.getEncoders();
    robot.getPose();

    if (signal == 1) {
      if (!routeStarted) {
        routeStarted = true;
        routeStartMs = now;
        println("START," + now);
      }
      updateMotion(robot, now);
    }

    if ( telemetry_timer.isReady(now) ) {
      telemetry_timer.resetTimer(now);
      publishTelemetry(robot); // Note: in simulation this is a dummy call
    }
  }

  /** @brief 每周期执行当前动作，跑完后一直停车。 */
  void updateMotion(Robot_c robot, long now) {
    if (phase == PHASE_DONE) {
      robot.setMotorPWM(0, 0);
      return;
    }
    // 开始后先静止1 s
    if (now - routeStartMs < 1000) {
      robot.setMotorPWM(0, 0);
      return;
    }
    // 按 data/forward_steady 与 data/reverse_steady 的 counts/s–PWM 拟合 + r=16mm, B=90mm 计算，和Controller.cpp保持一致
    switch (motionStep) {
      case STRAIGHT_FWD:
        runTimedPwm(robot, now, 58, 57, 1940);
        break;
      case TURN_LEFT:
        runTimedPwm(robot, now, -41, 40, 1250);  // 180° 第1组 B=90 r=16
        break;
      case TURN_RIGHT:
        runTimedPwm(robot, now, 40, -39, 1270);  // 180° 第1组 B=90 r=16
        break;
      case ARC_LEFT:
        // runTimedPwm(robot, now, 48, 68, 1730);  // 第1组 B=90 r=16
        // runTimedPwm(robot, now, 50, 70, 1610);  // 第2组 B=85.8 r=16.36
        runTimedPwm(robot, now, 50, 70, 1660);     // 第3组 B=85.8 r=16.36 +阶跃响应（转角补偿）
        break;
      case ARC_RIGHT:
        // runTimedPwm(robot, now, 69, 47, 1730);  // 第1组 B=90 r=16
        // runTimedPwm(robot, now, 71, 49, 1610);  // 第2组 B=85.8 r=16.36
        runTimedPwm(robot, now, 71, 49, 1690);     // 第3组 B=85.8 r=16.36 +阶跃响应（转角补偿）
        break;
    }
  }

  /** @brief 按给定左右PWM跑durationMs，再停车记录2000 ms。 */
  void runTimedPwm(Robot_c robot, long nowMs, float leftPwm, float rightPwm, long durationMs) {
    if (phase == PHASE_WAITING) {
      motionStartMs = nowMs;
      phase = PHASE_RUNNING;
      println("RUNNING," + nowMs);
    }
    if (phase == PHASE_RUNNING) {
      if (nowMs - motionStartMs >= durationMs) {
        phase = PHASE_SETTLING;
        settlingStartMs = nowMs;
        println("SETTLING," + nowMs);
      } else {
        robot.setMotorPWM(leftPwm, rightPwm);
      }
    }
    if (phase == PHASE_SETTLING) {
      robot.setMotorPWM(0, 0);
      if (nowMs - settlingStartMs >= 2000) {
        phase = PHASE_DONE;
        println("DONE," + nowMs);
      }
    }
  }

  /**
   * @brief Placeholder matching the StampC3 controller's telemetry task.
   * @note Simulator owns TCP reception and graphing. This empty method keeps
   * the 50 ms controller task visible without creating a second transport.
   */
  void publishTelemetry(Robot_c robot) {
  }
}
