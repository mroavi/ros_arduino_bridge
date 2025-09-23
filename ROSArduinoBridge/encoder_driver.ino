/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  /* Interrupt routine for LEFT encoder, taking care of actual counting */
  ISR (PCINT2_vect){
  	static uint8_t enc_last=0;
        
	enc_last <<=2; //shift previous state two places
	enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits
  
  	left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Interrupt routine for RIGHT encoder, taking care of actual counting */
  ISR (PCINT1_vect){
        static uint8_t enc_last=0;
          	
	enc_last <<=2; //shift previous state two places
	enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits
  
  	right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
  }
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }
#elif defined(ARDUINO_MOTOR_SHIELD_REV3) || defined(KS_L298P_SHIELD)
  // Encoder implementation for Rev3/KS0007 using PinChangeInterrupt library
  #include <PinChangeInterrupt.h>
  #include <util/atomic.h>

  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;

  // Fast read macros
  #define readLeftA   bitRead(PIND, LEFT_ENC_PIN_A)
  #define readLeftB   bitRead(PIND, LEFT_ENC_PIN_B)
  #define readRightA  bitRead(PIND, RIGHT_ENC_PIN_A)
  #define readRightB  bitRead(PIND, RIGHT_ENC_PIN_B)

  void isrLeftA()  { (readLeftB  != readLeftA)  ? left_enc_pos++  : left_enc_pos--; }
  void isrLeftB()  { (readLeftA  == readLeftB)  ? left_enc_pos++  : left_enc_pos--; }
  void isrRightA() { (readRightB != readRightA) ? right_enc_pos++ : right_enc_pos--; }
  void isrRightB() { (readRightA == readRightB) ? right_enc_pos++ : right_enc_pos--; }

  void initEncoders() {
    pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), isrLeftA, CHANGE);
    attachPCINT(digitalPinToPCINT(LEFT_ENC_PIN_B), isrLeftB, CHANGE);
    attachPCINT(digitalPinToPCINT(RIGHT_ENC_PIN_A), isrRightA, CHANGE);
    attachPCINT(digitalPinToPCINT(RIGHT_ENC_PIN_B), isrRightB, CHANGE);
  }

  long readEncoder(int i) {
    long val;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      val = (i == LEFT) ? left_enc_pos : right_enc_pos;
    }
    return val;
  }

  void resetEncoder(int i) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if (i == LEFT) left_enc_pos = 0L;
      else right_enc_pos = 0L;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif

