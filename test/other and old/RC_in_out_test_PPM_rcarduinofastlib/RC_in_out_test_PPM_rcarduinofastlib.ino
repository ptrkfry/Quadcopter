#include <RCArduinoFastLib.h>
// MultiChannels
//
// rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// rcarduino.blogspot.com
//
//!!!!!!!!!!!!uses interrupt pin 0 (=digital pin 2) for PPM input (uses pinchangeinterupt library too?)
//!!!!!!!!!!!! change RC_CHANNEL_IN_COUNT in RCArduinoFastLib.h to read more channels from PPM

//  The library is able to generate higher refresh rate signals than the standard Servo library which forces a 50Hz refresh rate. 
//  To set the refresh rate we create an additional entry in the servo array (by setting RC_CHANNEL_OUT_COUNT  to one more servo than we need)(no need to "attach")
//  We use the 'setFrameSpace' function on this last entry to set the frame space which sets the refresh rate. 
//  While this is initially an awkward additional step, it also gives us a lot more flexibility, 
//  for example we can use the library to drive high refresh rates upto 500Hz

//Define indices of Outputs (ESC's and frame space)
#define ESC_1_OUT_index 0
#define ESC_2_OUT_index 1
#define ESC_3_OUT_index 2
#define ESC_4_OUT_index 3
#define FRAME_SPACE_index 4

// Assign channel out pins
#define ESC_1_OUT_PIN 8 
#define ESC_2_OUT_PIN 9 
#define ESC_3_OUT_PIN 10 
#define ESC_4_OUT_PIN 11

volatile uint32_t ulCounter = 0;

void setup()
{

  Serial.begin(115200);
  //ESC Output
  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  CRCArduinoFastServos::attach(ESC_1_OUT_index,ESC_1_OUT_PIN);
  CRCArduinoFastServos::attach(ESC_2_OUT_index,ESC_2_OUT_PIN);
  CRCArduinoFastServos::attach(ESC_3_OUT_index,ESC_3_OUT_PIN);
  CRCArduinoFastServos::attach(ESC_4_OUT_index,ESC_4_OUT_PIN);

  // The setFrameSpace function is provided for you to
  // add a pause before the library begins its next run through the servos
  // for 50 hz, the pause should be to (20,000 - ((RC_CHANNEL_OUT_COUNT-1) * 2000)) 
  CRCArduinoFastServos::setFrameSpaceA(FRAME_SPACE_index,12000); //FrameSpace A is for first ten servos, (B for second ten servos)
  //start outputting to servos/ESC's
  CRCArduinoFastServos::begin();

  //start PPM reading
  CRCArduinoPPMChannels::begin();


}

void loop()
{
  // Pass the signals straight through 
  uint16_t length_FirstPulse = CRCArduinoPPMChannels::getChannel(ESC_1_OUT_index); //get signal out of PPM, number = index of signal
  Serial.println("First pulse: "+length_FirstPulse);

  
  if(length_FirstPulse)
  {
    CRCArduinoFastServos::writeMicroseconds(ESC_1_OUT_index,length_FirstPulse); //write to output pin (to Servo/ESC with index 0)
  }
}



