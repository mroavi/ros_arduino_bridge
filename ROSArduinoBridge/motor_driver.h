/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#endif

#if defined(ARDUINO_MOTOR_SHIELD_REV3) || defined(KS_L298P_SHIELD)
  // Pin mapping for Arduino Motor Shield Rev3 / KS0007 (L298P)
  #define LEFT_MOTOR_SPEED   3    // PWM
  #define LEFT_MOTOR_DIR     12   // Direction
  #define RIGHT_MOTOR_SPEED  11   // PWM
  #define RIGHT_MOTOR_DIR    13   // Direction
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
