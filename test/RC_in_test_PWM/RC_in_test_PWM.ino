//assume that pin 32 is receiving PWM input
#define CHANNEL_1_PIN 32

//micros when the pin goes HIGH
volatile unsigned long timer_start;

//difference between timer_start and micros() is the length of time that the pin
//was HIGH - the PWM pulse length. 
volatile int pulse_time;

//this is the time that the last interrupt occurred.
//you can use this to determine if your receiver has a signal or not.
volatile int last_interrupt_time;

void setup()
{
  timer_start = 0;
  attachInterrupt(CHANNEL_1_PIN, calcSignal, CHANGE);
  Serial.begin(115200);
}
void loop()
{
  Serial.println(pulse_time);
  delay(20);
}

//ISR routine
void calcSignal()
{
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  last_interrupt_time = micros();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if(digitalRead(CHANNEL_1_PIN) == HIGH)
  {
    timer_start = micros();
  }
  //otherwise, the pin has gone LOW
  else
  {
    //only worry about this if the timer has actually started
    if(timer_start != 0)
    {
      //record the pulse time
      pulse_time = ((volatile int)micros() - timer_start);
      //restart the timer
      timer_start = 0;
    }
  }
}
