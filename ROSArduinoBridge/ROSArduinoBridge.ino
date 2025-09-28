#include <Wire.h>

/*********************************************************************
 *  ROSArduinoBridge (I2C Version, variable-length replies)
 *********************************************************************/

#define USE_BASE
#define L298P_ENC_COUNTER
#define KS_L298P_SHIELD

#undef USE_SERVOS

#define BAUDRATE 57600 // kept for compatibility
#define MAX_PWM 255

#include "Arduino.h"
#include "commands.h"
#include "sensors.h"

#ifdef USE_SERVOS
#include "servos.h"
#include <Servo.h>
#endif

#ifdef USE_BASE
#include "motor_driver.h"
#include "encoder_driver.h"
#include "diff_controller.h"

#define PID_RATE 30
const int PID_INTERVAL = 1000 / PID_RATE;
unsigned long nextPID = PID_INTERVAL;

#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Command parsing variables */
char cmd;
char argv1[16];
char argv2[16];
long arg1;
long arg2;

char replyBuffer[32]; // reply buffer

/* I2C buffering */
#define CMD_BUFFER_SIZE 32
volatile char i2cBuffer[CMD_BUFFER_SIZE];
volatile size_t i2cIndex = 0;
volatile bool commandReady = false;

/* Helpers */
void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
}

void writeResponse(const char *s) {
  memset(replyBuffer, 0, sizeof(replyBuffer));
  strncpy(replyBuffer, s, sizeof(replyBuffer) - 1);
  replyBuffer[sizeof(replyBuffer) - 1] = '\0'; // ensure termination
}

/* Run a command */
void runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
  case GET_BAUDRATE:
    writeResponse(String(BAUDRATE).c_str());
    break;
  case ANALOG_READ:
    writeResponse(String(analogRead(arg1)).c_str());
    break;
  case DIGITAL_READ:
    writeResponse(String(digitalRead(arg1)).c_str());
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    writeResponse("OK");
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0)
      digitalWrite(arg1, LOW);
    else if (arg2 == 1)
      digitalWrite(arg1, HIGH);
    writeResponse("OK");
    break;
  case PIN_MODE:
    if (arg2 == 0)
      pinMode(arg1, INPUT);
    else if (arg2 == 1)
      pinMode(arg1, OUTPUT);
    writeResponse("OK");
    break;
  case PING:
    writeResponse(String(Ping(arg1)).c_str());
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    writeResponse("OK");
    break;
  case SERVO_READ:
    writeResponse(String(servos[arg1].getServo().read()).c_str());
    break;
#endif
#ifdef USE_BASE
  case READ_ENCODERS:
    writeResponse(
        (String(readEncoder(LEFT)) + " " + String(readEncoder(RIGHT))).c_str());
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    writeResponse("OK");
    break;
  case MOTOR_SPEEDS:
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    } else
      moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    writeResponse("OK");
    break;
  case MOTOR_RAW_PWM:
    lastMotorCommand = millis();
    resetPID();
    moving = 0;
    setMotorSpeeds(arg1, arg2);
    writeResponse("OK");
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != NULL) {
      pid_args[i] = atoi(str);
      i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    writeResponse("OK");
    break;
#endif
  default:
    writeResponse("Invalid");
    break;
  }
}

/* I2C receive callback (ISR-safe: only buffer data) */
void receiveEvent(int howMany) {
  while (Wire.available() && i2cIndex < CMD_BUFFER_SIZE - 1) {
    char c = Wire.read();
    if (c == '\r') {
      i2cBuffer[i2cIndex] = '\0';
      commandReady = true;
      i2cIndex = 0;
    } else {
      i2cBuffer[i2cIndex++] = c;
    }
  }
}

/* I2C request callback */
void requestEvent() {
  if (replyBuffer[0] == '\0') {
    strcpy(replyBuffer, "OK"); // fallback
  }
  size_t len = strnlen(replyBuffer, sizeof(replyBuffer));
  if (len == 0) {
    replyBuffer[0] = 'O'; replyBuffer[1] = 'K'; replyBuffer[2] = '\0';
    len = 2;
  }
  Wire.write((uint8_t *)replyBuffer, len);
}

/* Setup */
void setup() {
  Wire.begin(0x08);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

#ifdef USE_BASE
#ifdef L298P_ENC_COUNTER
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), isrLeftA, CHANGE);
  attachPCINT(digitalPinToPCINT(LEFT_ENC_PIN_B), isrLeftB, CHANGE);
  attachPCINT(digitalPinToPCINT(RIGHT_ENC_PIN_A), isrRightA, CHANGE);
  attachPCINT(digitalPinToPCINT(RIGHT_ENC_PIN_B), isrRightB, CHANGE);
#endif
  initMotorController();
  resetPID();
#endif
}

/* Loop */
void loop() {
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif

  // Process commands outside ISR
  if (commandReady) {
    commandReady = false;

    resetCommand();

    // Parse command string from buffer
    char *p = (char *)i2cBuffer;
    cmd = *p++;
    if (*p) {
      char *space = strchr(p, ' ');
      if (space) {
        *space = '\0';
        strncpy(argv1, p, sizeof(argv1) - 1);
        strncpy(argv2, space + 1, sizeof(argv2) - 1);
      } else {
        strncpy(argv1, p, sizeof(argv1) - 1);
      }
    }

    runCommand();
  }

#ifdef USE_SERVOS
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}
