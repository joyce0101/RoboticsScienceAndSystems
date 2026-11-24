/**
 * @brief Nominal line-following controller shared conceptually with StampC3.
 * @details The controller reads cached robot values at a fixed interval and
 * commands requested PWM values. It is intentionally separate from the
 * physical-response models so those models can be revised independently.
 */
class Controller_c {
  /** @brief Minimum interval between control/acquisition updates, in ms. */
  final int UPDATE_INTERVAL_MS = 10;
  /** @brief Interval reserved for telemetry work, in ms. */
  final int TELEMETRY_INTERVAL_MS = 50;
  final int RECOVERY_TIMEOUT_MS = 5000;
  final float LINE_THRESHOLD = 1400;
  // Keep these values aligned with StampC3_Template/Controller.h.
  final float BASE_PWM = 45;
  final float LINE_FOLLOW_GAIN = 0.01;
  final float RECOVERY_LEFT_PWM = -35.0;
  final float RECOVERY_RIGHT_PWM = 35.0;
  final int WAITING = 0, FOLLOWING = 1, SEARCHING = 2, STOPPED = 3;
  int signal = 0, mode = WAITING;
  long recoveryStartMs = 0;
  // 单动作试验：执行状态和动作编号，和StampC3那边对应
  final int PHASE_WAITING = 0, PHASE_RUNNING = 1, PHASE_SETTLING = 2, PHASE_DONE = 3;
  final int STRAIGHT_FWD = 0, TURN_LEFT = 1, TURN_RIGHT = 2, ARC_LEFT = 3, ARC_RIGHT = 4;
  int phase = PHASE_WAITING;
  int motionStep = STRAIGHT_FWD;
  boolean routeStarted = false;
  long routeStartMs = 0, motionStartMs = 0, settlingStartMs = 0;

  /** @brief TaskTimer_c to keep aligned with code from Arduino platform */
  TaskTimer_c update_timer = new TaskTimer_c( UPDATE_INTERVAL_MS );
  TaskTimer_c telemetry_timer = new TaskTimer_c( TELEMETRY_INTERVAL_MS );
  

  /** @brief Returns the controller to its inactive waiting state. */
  void reset() {
    signal = 0;
    mode = WAITING;
    recoveryStartMs = 0;
    // 每次试验只改这一行，选这次跑哪个动作
    motionStep = STRAIGHT_FWD;
    phase = PHASE_WAITING;
    routeStarted = false;
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
      mode = FOLLOWING;
      recoveryStartMs = 0;
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
    
    if( !update_timer.isReady(now) ) {
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
    
    if( telemetry_timer.isReady(now) ) {
      telemetry_timer.resetTimer(now);
      publishTelemetry(robot); // Note: in simulation this is a dummy call
    }
  }
  
  
  
  /** @brief Tests whether any surface channel is at or above the line threshold. */
  boolean lineDetected(Robot_c robot) {
    if (robot.surface.reading[2] >= LINE_THRESHOLD) return true;
    return false;
  }
  
  
  /** @brief Applies following, search, or stopped motor commands. */
  void runLineFollower(Robot_c robot, long now) {
    boolean detected = lineDetected(robot);
    if (!detected && mode == FOLLOWING) {
      mode = SEARCHING;
      recoveryStartMs = now;
    }
    
    if (mode == SEARCHING && detected) {
      mode = FOLLOWING;
    }
    if (mode == SEARCHING && now - recoveryStartMs >= RECOVERY_TIMEOUT_MS) {
      mode = STOPPED;
    }

    if (mode == STOPPED) {
      robot.setMotorPWM(0, 0);
      return;
    }
    if (mode == SEARCHING) {
      robot.setMotorPWM(RECOVERY_LEFT_PWM, RECOVERY_RIGHT_PWM);
      return;
    }
    
    float turn = (robot.surface.reading[1] - robot.surface.reading[3]) * LINE_FOLLOW_GAIN;
    robot.setMotorPWM(BASE_PWM - turn, BASE_PWM + turn);
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
    // 参数暂定，标定仿真后再改，和Controller.cpp保持一致
    switch (motionStep) {
      case STRAIGHT_FWD:
        runTimedPwm(robot, now, 80, 80, 3500);
        break;
      case TURN_LEFT:
        runTimedPwm(robot, now, -40, 40, 1414);
        break;
      case TURN_RIGHT:
        runTimedPwm(robot, now, 40, -40, 1414);
        break;
      case ARC_LEFT:
        runTimedPwm(robot, now, 62, 98, 3142);
        break;
      case ARC_RIGHT:
        runTimedPwm(robot, now, 98, 62, 3142);
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
