/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5

#elif defined(L298P_ENC_COUNTER)
  #include <PinChangeInterrupt.h>
  // Encoder wiring for L298P shields (Rev3 and KS0007)
  // Uses pins D2, D4, D5, D6 (since 3, 11, 12, 13 are occupied)
  #define LEFT_ENC_PIN_A   2   // hardware interrupt INT0
  #define LEFT_ENC_PIN_B   4   // PCINT
  #define RIGHT_ENC_PIN_A  5   // PCINT
  #define RIGHT_ENC_PIN_B  6   // PCINT
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

