#ifndef HUMANOID_MOTOR_CONTROL__CYTRON_CONTROLLER_HPP_
#define HUMANOID_MOTOR_CONTROL__CYTRON_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <cstdint>

namespace humanoid_motor_control
{

/**
 * @brief Controller for Cytron MDDRC10 Motor Driver
 * 
 * Uses RC/Servo PWM signals for control:
 * - RC1 channel controls left motors (motors 0, 2)
 * - RC2 channel controls right motors (motors 1, 3)
 * - PWM pulse width: 1000-2000 microseconds
 * - 1500μs = neutral (motor stop)
 * - <1500μs = counter-clockwise rotation
 * - >1500μs = clockwise rotation
 * - Speed increases as value moves away from 1500μs
 */
class CytronController
{
public:
  CytronController();
  ~CytronController();

  /**
   * @brief Initialize PWM for Cytron MDDRC10 RC inputs
   * @param rc1_pin GPIO pin for RC1 (left motors)
   * @param rc2_pin GPIO pin for RC2 (right motors)
   * @return true if successful
   */
  bool init(int rc1_pin, int rc2_pin);

  /**
   * @brief Set motor speed and direction
   * @param motor_id Motor ID (0=FL, 1=FR, 2=RL, 3=RR)
   * @param speed Speed value (-100 to 100, negative = reverse)
   */
  void setMotorSpeed(uint8_t motor_id, int16_t speed);

  /**
   * @brief Set left and right motor speeds directly
   * @param left_speed Speed for left motors (-100 to 100)
   * @param right_speed Speed for right motors (-100 to 100)
   */
  void setLeftRight(int16_t left_speed, int16_t right_speed);

  /**
   * @brief Set all motors at once
   * @param speeds Vector of 4 speeds (-100 to 100)
   */
  void setAllMotors(const std::vector<int16_t> & speeds);

  /**
   * @brief Emergency stop - set all motors to neutral (1500μs)
   */
  void emergencyStop();

  /**
   * @brief Check if controller is initialized
   */
  bool isConnected() const { return is_initialized_; }

  /**
   * @brief Cleanup GPIO
   */
  void close();

private:
  int rc1_pin_;  // GPIO pin for RC1 (left motors)
  int rc2_pin_;  // GPIO pin for RC2 (right motors)
  bool is_initialized_;
  int pigpio_handle_ = -1;

  // PWM constants (microseconds)
  static constexpr int PWM_MIN = 1000;
  static constexpr int PWM_NEUTRAL = 1500;
  static constexpr int PWM_MAX = 2000;
  static constexpr int PWM_RANGE = 500;  // Range from neutral to min/max

  /**
   * @brief Convert speed (-100 to 100) to PWM pulse width (1000-2000μs)
   * @param speed Speed percentage
   * @return PWM pulse width in microseconds
   */
  int speedToPWM(int16_t speed);

  /**
   * @brief Send RC PWM signal to GPIO pin
   * @param pin GPIO pin number
   * @param pulse_width_us Pulse width in microseconds
   */
  void sendPWM(int pin, int pulse_width_us);

  /**
   * @brief Clamp speed to valid range (-100 to 100)
   */
  int16_t clampSpeed(int16_t speed);
};

}  // namespace humanoid_motor_control

#endif  // HUMANOID_MOTOR_CONTROL__CYTRON_CONTROLLER_HPP_