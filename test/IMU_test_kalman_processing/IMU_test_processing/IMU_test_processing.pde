import processing.serial.*;

Serial myPort;        // The serial port

float angleX = 0, angleY=0, angleZ=0;
int index=0;

void setup () 
{
  // set the window size:
  size(500, 500,P3D);
  background(0);

  // List all the available serial ports
  // if using Processing 2.1 or later, use Serial.printArray()
  println(Serial.list());

  // I know that the first port in the serial list on my mac
  // is always my  Arduino, so I open Serial.list()[0].
  // Open whatever port is the one you're using.
  myPort = new Serial(this, Serial.list()[0], 115200);

  // don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil('\n');

}
void draw ()
{
  background(0);
  //change point of view (camera)
  //camera(width/2,height/2,(height/2) / tan(PI/6),width/2, height/2, 0,0,1,0); //camera(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, upX, upY, upZ); upX, upY and upZ determine which axis points up
  //camera(width/2,0,(height/2) / tan(PI/6),width/2, height/2, 0,0,1,0); //moved eye higher up
  
  //pushMatrix();
  translate(width/2, height/2, 0); //move to center of screen
  rotateX(-angleX*PI/180.0);
  rotateY(0);
  rotateZ(angleY*PI/180.0);
  box(20,10,300); //box(size in x dir, size in y dir, size in z dir);
  box(300,10,20);
  //popMatrix();
}


void serialEvent (Serial myPort) {
  
  try{
              index++;
    // get the ASCII string:
    String inString = myPort.readStringUntil('\n');
    // trim off any whitespace:
    inString = trim(inString);
    //println(inString);
  
    if (inString != null) {
      if(inString.equals("start")) //compare strings
      {
        index=0;
        //println("start detected");
      }
      else
      {
        //check sign
        int sign=1;
        if(inString.substring(0, 1).equals("-"))
        {
          sign=-1;
          inString=inString.substring(1); //get string from second character to end
        }
        //println(inString);
        
 
        // convert to float
        float angle= sign*float(inString);
        if(Float.isNaN(angle)){println("nan");index++;return;}

        switch(index)
        {
        case 1:
          angleX=angle;
          //print("x=");
          //println(angleX);
          break;
        case 2:
          angleY=angle;
          //print("y=");
          //println(angleY);
          break;
        case 3:
          angleZ=angle;
          //print("z=");
          //println(angleZ);
          break;
        default:
          break;
        }
      }
    }
  }
  catch(RuntimeException e){ //to avoid crashing if an error occurs
    println("runtime exception");
  }
}