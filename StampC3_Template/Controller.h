#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"

/**
 * @brief Runs the button-started motion sequence and shared motor/sensor tools.
 *
 * Controller_c owns the controller state, while Robot_c owns the latest hardware
 * snapshot and RobotWifiAP_c owns optional telemetry transport. Students may read
 * or replace this controller, but should treat its constants as a demonstrator
 * baseline rather than a calibration of every physical robot.
 */
class Controller_c {
  
  public:
    /** @brief Editable local encoder-to-pose model; inspect odometry.pose.
     * @note Controller mode resets do not reset this estimate. Use
     * odometry.reset(...) explicitly to start a new physical trial.
     */
    OdometryModel_c odometry;
    unsigned long last_time_ms;
    // Measuring Wheel Speed: Encoders
    int32_t last_count_left;
    int32_t last_count_right;
    float speed_left;   // Left wheel rim speed, mm/s; positive counts are forward.
    float speed_right;  // Right wheel rim speed, mm/s.
    bool speed_initialised;

    /** @brief Fastest interval between controller and I2C acquisition cycles. */
    static const unsigned long UPDATE_INTERVAL_MS = 10;
    /** @brief Interval between telemetry records in milliseconds. */
    static const unsigned long TELEMETRY_INTERVAL_MS = 50;
    /** @brief Maximum line-search duration before the controller stops. */
    static const unsigned long RECOVERY_TIMEOUT_MS = 5000;
    /** @brief Surface-reading threshold used to decide whether a line is present. */
    static const uint16_t LINE_THRESHOLD = 1400;
    /** @brief Nominal forward PWM bias for line following. */
    static constexpr float BASE_PWM = 45.0f;
    /** @brief Proportional gain applied to the DN2-minus-DN4 surface error. */
    static constexpr float LINE_FOLLOW_GAIN = 0.01f;
    /** @brief Left PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_LEFT_PWM = -35.0f;
    /** @brief Right PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_RIGHT_PWM = 35.0f;

    /** @brief Creates a controller in its waiting state. */
    Controller_c();

    enum class ExperimentPhase { WAITING, RUNNING, SETTLING, DONE, FAULT };
    ExperimentPhase getExperimentPhase() const;
    static const uint32_t ENCODER_TIMEOUT_MS = 300;

    // 纯计算：半径mm、转角rad、中心速度mm/s、轮距mm -> 左右轮速和时长。
    // 无效参数不修改输出；PWM可执行范围另外由speedToPwm检查。
    static bool calculateArcMotion(float radiusMm, float angleRad,
                                   float centreSpeedMmPerSec, float wheelSeparationMm,
                                   float& leftSpeed, float& rightSpeed, uint32_t& durationMs);

    // 空载左轮正向PWM40～200拟合：v = 3.722106*PWM - 28.861902 (mm/s)。
    // 右轮/反向暂共用幅值模型，尚未独立标定；截距不是PWM死区。
    static constexpr float PWM_SPEED_GAIN = 3.722106f;
    static constexpr float PWM_SPEED_INTERCEPT = -28.861902f;
    // 独立于速度截距：非零目标映射后的PWM幅值必须超过30。
    static const int16_t PWM_DEADBAND = 30;
    static const int16_t MAX_PWM = 200;

    /** @brief Convert mm/s to signed PWM using the shared motor model.
     * @return False for non-finite/out-of-range speeds or nonzero commands in
     * the deadband. On failure pwm is unchanged. This sends no motor command.
     */
    static bool speedToPwm(float speedMmPerSec, int16_t& pwm);

    /** @brief Validate both target speeds, then send one complete motor command.
     * @return False for invalid speeds or a failed motor write. Invalid speeds
     * send no command; the caller must handle failure and stopping explicitly.
     */
    static bool setWheelSpeeds(Robot_c& robot, float leftSpeedMmPerSec,
                               float rightSpeedMmPerSec);

    /** @brief Clears signal and mode state, returning the controller to waiting.
     * @note Task timers and odometry are retained. An active action is retained;
     * idle, completed or faulted motion is rearmed to the first step.
     * It sends no motor command: returning to waiting does not stop the motors.
     * Send robot.setMotorPWM(0, 0) separately and check the command result.
     */
    void reset();

    /**
     * @brief Returns whether the controller has received its start signal.
     * @return Zero while waiting; one after the button or caller starts it.
     */
    uint8_t getSignal() const;

    /**
     * @brief Starts the controller or resets it to waiting.
     * @param requested_signal Zero resets to waiting without sending a motor stop;
     * a non-zero value starts motion only if the current signal is zero.
     * @note The signal remains one after a line-search timeout. Reset to zero
     * before requesting another start; a repeated non-zero value has no effect.
     */
    void setSignal(uint8_t requested_signal);

    /**
     * @brief Runs one time-controlled acquisition, control, and telemetry cycle.
     * @param robot Hardware-facing interface and cached measurement snapshot.
     * @param server Optional WiFi/TCP telemetry transport.
     * @note Each accepted update cycle attempts to refresh sensors, encoders,
     * and the middleware pose. Failed reads retain their previous cached values;
     * local odometry is updated only after a successful encoder read. The method
     * returns early between update intervals.
     */
    void update(Robot_c &robot, RobotWifiAP_c &server);
    protected:
    /** 每周期调用当前动作方法，完成后推进下一步。 */
    virtual void updateMotion(Robot_c &robot, unsigned long now);
    // 在updateMotion中使用，读取update()本周期已采集的编码器有效标志。
    ExperimentPhase runArc(Robot_c& robot, uint32_t nowMs,
                           float radiusMm, float angleRad, float centreSpeedMmPerSec);
    // 按给定左右PWM跑durationMs，再停车记录2000 ms；直线、原地转和圆弧共用。
    ExperimentPhase runTimedPwm(Robot_c& robot, uint32_t nowMs,
                                int16_t leftPwm, int16_t rightPwm, uint32_t durationMs);

  private:
    /** @brief Internal modes used by the nominal line-following demonstration. */
    enum Mode {
      WAITING,
      FOLLOWING_LINE,
      SEARCHING_FOR_LINE,
      STOPPED
    };

    uint8_t signal;
    Mode mode;
    TaskTimer_c update_timer;
    TaskTimer_c telemetry_timer;
    unsigned long recovery_start_ms;


    // 每个枚举值是路线中的一个具体动作，同类型动作也各占一步。
    enum class MotionStep { ARC_1, STRAIGHT_1, STRAIGHT_FWD, TURN_LEFT, TURN_RIGHT, ARC_LEFT, ARC_RIGHT };
    MotionStep motionStep = MotionStep::ARC_1;
    ExperimentPhase experimentPhase = ExperimentPhase::WAITING;
    uint32_t motionStartMs = 0;
    uint32_t settlingStartMs = 0;
    uint32_t routeStartMs = 0;
    uint32_t lastEncoderMs = 0;
    bool motionBaselineInitialized = false;
    bool encoder_sample_updated = false;
    bool stopMotion(Robot_c& robot);

    /** @brief Returns whether cached DN3 is at or above LINE_THRESHOLD. */
    bool lineDetected(const Robot_c &robot) const;

    /** @brief Applies the nominal controller behaviour for the current mode. */
    void runLineFollower(Robot_c &robot, unsigned long now);
    void measureWheelSpeed(Robot_c &robot, unsigned long now);

    /**
     * @brief Sends the current controller and robot snapshot as one CSV record.
     * @note Controller.cpp writes these 14 columns, in order: timestamp_ms,
     * x, y, theta, left_pwm, right_pwm, left_count, right_count,
     * DN1, DN2, DN3, DN4, DN5, signal. These labels are not sent as a header.
     * x and y are in mm and theta is in radians, from the local Euler estimate,
     * not robot.pose from I2C. PWM values are cached requests, not measured output.
     * The timestamp is the controller-cycle start time; it does not guarantee
     * that every cached reading was successfully refreshed in that cycle.
     */
    void publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms);
};

#endif
