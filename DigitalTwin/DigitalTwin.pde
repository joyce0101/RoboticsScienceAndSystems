import processing.net.*;
import processing.core.PApplet;
import java.net.*;

/** @brief Initial application-window width in screen pixels. */
final int WINDOW_WIDTH = 1000;
/** @brief Initial application-window height in screen pixels. */
final int WINDOW_HEIGHT = 800;
/** @brief Initial surface-image x offset from the world origin, in mm. */
final float SURFACE_OFFSET_X_MM = 135.0;
/** @brief Initial surface-image y offset from the world origin, in mm. */
final float SURFACE_OFFSET_Y_MM = 0.0;

/** @brief Application-level owner of the simulated robot and UI. */
Simulator simulator;

/** @brief Configures Processing's initial drawing surface. */
void settings() {
  size(WINDOW_WIDTH, WINDOW_HEIGHT);
}

/** @brief Creates the resizable simulator window and its application state. */
void setup() {
  
  // Keep a normal window (rather than fullscreen) so students can resize it
  // or place it beside their controller code. Starting at the display size
  // gives the same practical benefit as maximising it at launch.
  surface.setResizable(true);
  
  //surface.setSize(displayWidth, displayHeight);
  //surface.setLocation(0, 0);
  surface.setTitle("CW2026 Digital Twin");
  simulator = new Simulator(this, SURFACE_OFFSET_X_MM, SURFACE_OFFSET_Y_MM);
  // 左右轮正/反转稳态拟合（counts/s = k * (|PWM| - p0)），来自 data/forward_steady 与 data/reverse_steady
  // 实测有效区间 |PWM| 35–80，轮径由 Robot_c(16.0, ...) 通过 odometry.mmPerEncoderCount 统一换算
  simulator.robot.motors.left.fwdCountsPerPwm = 13.8064;
  simulator.robot.motors.left.fwdDeadZonePwm = 11.43;
  simulator.robot.motors.right.fwdCountsPerPwm = 13.9665;
  simulator.robot.motors.right.fwdDeadZonePwm = 10.96;
  simulator.robot.motors.left.revCountsPerPwm = 13.8813;
  simulator.robot.motors.left.revDeadZonePwm = 11.99;
  simulator.robot.motors.right.revCountsPerPwm = 13.9134;
  simulator.robot.motors.right.revDeadZonePwm = 10.34;
  
}

/** @brief Advances the application state and renders one frame. */
void draw() {
  simulator.update();
  simulator.draw();
}

/** @brief Forwards keyboard input to the simulator controls.
 * @see Simulator#handleKey(char, int)
 */
void keyPressed() {
  simulator.handleKey(key, keyCode);
}
