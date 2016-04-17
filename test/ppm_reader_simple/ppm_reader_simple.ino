const int numberOfChannels=6;
volatile long currentTime;
volatile long delta;
volatile long lastPulse=0;
volatile int pulseLengths[numberOfChannels+1]; //framespace uses one space too
volatile int channel=0;
volatile bool start=false;

void setup() 
{
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(2),pulseDetected,RISING);
}

void loop() 
{

  for(int i=1;i<numberOfChannels+1;i++)
  {
   Serial.print(pulseLengths[i]);
   if(i<numberOfChannels) //no forward slash at end
   {
    Serial.print(" / ");
   }
  }
  Serial.print("\n"); 

  delay(200);
}

//ISR
void  pulseDetected()
{
  currentTime=micros();
  delta=currentTime-lastPulse;
  lastPulse=currentTime;
  if(delta>10000)
  {
    //Serial.println(" ");
    channel=0;
    start=true;
  }
  else
  {
    channel=channel+1;
  }
  
  if(start)
  {
    //Serial.println(delta);
    pulseLengths[channel]=delta;
  }
}


