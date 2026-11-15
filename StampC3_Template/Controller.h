#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"

/**
 * @brief Runs button-started motor experiments and encoder speed measurement.
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
    int call_count;
    int ramp_pwm;
    // int ramp_count;
    bool ramp_up;
    unsigned long last_time_ms;
    // Measuring Wheel Speed: Encoders
    int32_t last_count_left;
    int32_t last_count_right;
    float speed_left;   // Left wheel rim speed, mm/s; positive counts are forward.
    float speed_right;  // Right wheel rim speed, mm/s.
    bool speed_initialised;
    // Deadband: find where motion begins
    // 首次只测试 PWM 25 一次，验证流程。
    // 跑通后再改为 0～60、步长 5、每档重复 3 次。
    static const int DB_PWM_MIN = 25;
    static const int DB_PWM_MAX = 30;
    static const int DB_PWM_STEP = 1;
    static const int DB_REPEATS = 3;
    enum DeadbandPhase {
      DB_IDLE,
      DB_REST,
      DB_RUN
    };

    DeadbandPhase deadband_phase;
    TaskTimer_c deadband_timer;
    int test_pwm;
    int deadband_trial;
    int deadband_sample;

unsigned long deadband_run_start_ms;

    // Saturation pilot: expand MAX to 100 and REPEATS to 3 after checking stops.
    static const int SAT_PWM_MIN = 100;
    static const int SAT_PWM_MAX = 130;
    static const int SAT_PWM_STEP = 30;
    static const int SAT_REPEATS = 3;
    enum SaturationPhase { SAT_IDLE, SAT_REST, SAT_SETTLE, SAT_MEASURE };
    SaturationPhase saturation_phase;
    TaskTimer_c saturation_timer;
    int saturation_pwm;
    int saturation_trial;
    int saturation_sample;
    unsigned long saturation_phase_start_ms;

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

    /** @brief Clears signal and mode state, returning the controller to waiting.
     * @note Task timers and odometry are retained; motor experiments are rearmed.
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
     * a non-zero value starts the selected experiment if the signal is zero.
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
    TaskTimer_c ramp_timer;   // 控制pwm更新时间
    unsigned long recovery_start_ms;


    /** @brief Returns whether cached DN3 is at or above LINE_THRESHOLD. */
    bool lineDetected(const Robot_c &robot) const;

    /** @brief Applies the nominal controller behaviour for the current mode. */
    void runLineFollower(Robot_c &robot, unsigned long now);
    void motorStartStopTest(Robot_c &robot, unsigned long now);
    void measureWheelSpeed(Robot_c &robot, unsigned long now);
    void testDeadband(Robot_c &robot);
    void testSaturation(Robot_c &robot);

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
