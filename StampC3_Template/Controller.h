#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"

/**
 * @brief Runs the button-started timed-PWM motion experiments and telemetry.
 *
 * Controller_c owns the controller state, while Robot_c owns the latest hardware
 * snapshot and RobotWifiAP_c owns optional telemetry transport.
 */
class Controller_c {

  public:
    /** @brief Editable local encoder-to-pose model; inspect odometry.pose.
     * @note Controller resets do not reset this estimate. Use
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

    /** @brief Creates a controller in its waiting state. */
    Controller_c();

    enum class ExperimentPhase { WAITING, RUNNING, SETTLING, DONE };
    ExperimentPhase getExperimentPhase() const;

    /** @brief Clears signal state, returning the controller to waiting.
     * @note Task timers and odometry are retained. An active action is retained;
     * idle or completed motion is rearmed to the default step.
     * It sends no motor command: returning to waiting does not stop the motors.
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
     */
    void setSignal(uint8_t requested_signal);

    /**
     * @brief Runs one time-controlled acquisition, control, and telemetry cycle.
     * @param robot Hardware-facing interface and cached measurement snapshot.
     * @param server Optional WiFi/TCP telemetry transport.
     */
    void update(Robot_c &robot, RobotWifiAP_c &server);

  protected:
    /** 每周期调用当前动作方法，跑完后保持停车。 */
    virtual void updateMotion(Robot_c &robot, unsigned long now);
    // 按给定左右PWM跑durationMs，再停车记录settleMs；直线、原地转、圆弧和组合路径共用。
    ExperimentPhase runTimedPwm(Robot_c& robot, uint32_t nowMs,
                                int16_t leftPwm, int16_t rightPwm,
                                uint32_t durationMs, uint32_t settleMs = 2000);

  private:
    struct TimedSegment {
      int16_t leftPwm;
      int16_t rightPwm;
      uint32_t durationMs;
    };

    uint8_t signal;
    TaskTimer_c update_timer;
    TaskTimer_c telemetry_timer;

    // 每个枚举值是试验中的一个具体动作（含 E5 5段双弧拱门组合路径）。
    enum class MotionStep {
      STRAIGHT_FWD, TURN_LEFT, TURN_RIGHT, ARC_LEFT, ARC_RIGHT,
      ROUTE_E5_DUAL_ARC
    };
    MotionStep motionStep = MotionStep::ROUTE_E5_DUAL_ARC;
    ExperimentPhase experimentPhase = ExperimentPhase::WAITING;
    uint32_t motionStartMs = 0;
    uint32_t settlingStartMs = 0;
    uint32_t routeStartMs = 0;
    uint8_t routeSegmentIndex = 0;
    bool motionBaselineInitialized = false;

    void runRouteSequence(Robot_c& robot, uint32_t nowMs,
                          const TimedSegment* segments, uint8_t segmentCount,
                          uint32_t pauseMs = 3000, uint32_t finalSettleMs = 2000);

    void measureWheelSpeed(Robot_c &robot, unsigned long now);

    /**
     * @brief Sends the current controller and robot snapshot as one CSV record.
     * @note Controller.cpp writes these 14 columns, in order: timestamp_ms,
     * x, y, theta, left_pwm, right_pwm, left_count, right_count,
     * DN1, DN2, DN3, DN4, DN5, signal. These labels are not sent as a header.
     */
    void publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms);
};

#endif
