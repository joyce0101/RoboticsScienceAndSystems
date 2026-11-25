/**
 * @brief Paired motor-response model for the simulated robot.
 * @note Uses per-wheel forward and reverse counts/s vs PWM linear fits
 * from data/forward_steady/fit.json and data/reverse_steady/fit.json.
 */
class MotorsModel_c {
  // Deliberately simple starting assumption kept for baseline reference.
  final float maximumWheelSpeedMmPerMs = 0.25;
  MotorModel_c left = new MotorModel_c();
  MotorModel_c right = new MotorModel_c();

  /** @brief Stores requested left and right PWM values after limiting them. */
  void setPWM(float leftPwm, float rightPwm) {
    left.setPWM(leftPwm);
    right.setPWM(rightPwm);
  }

  /** @brief Updates both wheel-speed estimates using odometry mmPerEncoderCount. */
  void update(float mmPerEncoderCount) {
    left.update(mmPerEncoderCount, maximumWheelSpeedMmPerMs);
    right.update(mmPerEncoderCount, maximumWheelSpeedMmPerMs);
  }
}

/** @brief Response model for one motor (counts/s = k * (|PWM| - p0)). */
class MotorModel_c {
  float pwm = 0;
  float speedMmPerMs = 0;
  // 公用小车稳态拟合参数（counts/s），在DigitalTwin.pde的setup()中分别设置
  float fwdCountsPerPwm = 0;
  float fwdDeadZonePwm = 0;
  float revCountsPerPwm = 0;
  float revDeadZonePwm = 0;

  /** @brief Limits and stores a requested PWM value in [-200, +200]. */
  void setPWM(float value) { pwm = constrain(value, -200, 200); }

  /** @brief Maps the requested PWM to wheel speed in mm/ms via counts/s fit. */
  void update(float mmPerEncoderCount, float maximumSpeed) {
    // 原来的线性映射，做基线对比时改回这一行
    // speedMmPerMs = map(pwm, -200, 200, -maximumSpeed, maximumSpeed);
    if (pwm > 0) {
      float countsPerSec = max(0.0, fwdCountsPerPwm * (pwm - fwdDeadZonePwm));
      speedMmPerMs = countsPerSec * mmPerEncoderCount / 1000.0;
    } else if (pwm < 0) {
      float countsPerSec = max(0.0, revCountsPerPwm * (-pwm - revDeadZonePwm));
      speedMmPerMs = -countsPerSec * mmPerEncoderCount / 1000.0;
    } else {
      speedMmPerMs = 0;
    }
  }
}
