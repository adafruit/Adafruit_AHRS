import processing.serial.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;
import processing.opengl.*;
import saito.objloader.*;

float roll  = 0.0F; // Drawn along X axis in Processing
float pitch = 0.0F; // Drawn along Z axis in Processing
float yaw   = 0.0F; // Drawn along Y axis in Processing

OBJModel model;
Serial   port;
String   buffer = "";

void setup()
{
  size( 400, 400, OPENGL);
  frameRate(30);
  model = new OBJModel(this);
  model.load("airplane.obj");
  model.scale(3.5);
  
  // ToDo: Check for errors, this will fail with no serial device present
  String ttyPort = Serial.list()[0];
  port = new Serial(this, ttyPort, 115200);
  port.bufferUntil('\n');
}
 
void draw()
{
  background(128, 128, 128);

  // Set a new co-ordinate space
  pushMatrix();
 
  // Get some light in here
  lights();
  
  // Displace objects from 0,0
  translate(200, 200, 0);
  
  // Rotate shapes around the X/Y/Z axis (values in radians, 0..Pi*2)
  rotateX(radians(roll));
  rotateZ(radians(pitch));
  rotateY(radians(yaw));

  pushMatrix();
  rotateY(PI/2);
  noStroke();
  model.draw();
  popMatrix();
  
  // Roll Axis (X in Processing)
  stroke(255, 0, 0);
  box(250, 1, 1);
  text("Roll (X)", 85, -10, 0);
  
  // Pitch Axis (Z in Processing)
  stroke(0, 255, 0);
  box(1, 1, 200);
  text("Pitch (Y)", -25, -10, 100);
  
  // Yaw Axis (Y in Processing)
  stroke(0, 0, 255);
  box(1, 50, 1);
  text("Yaw (Z)", -25, -35, 0);

  // Render legend in normal XYZ space
  popMatrix();
  stroke(255);
  text("Roll (X):", 10, 20);
  text(roll, 75, 20);
  text("Pitch (Y):", 10, 35);
  text(pitch, 75, 35);
  text("Yaw (Z):", 10, 50);
  text(yaw, 75, 50);

  // Render incoming text from the Arduino sketch
  text("Incoming Data: " + buffer, 10, 385);  
}

void serialEvent(Serial p) 
{
  String incoming = p.readString();
  if ((incoming.length() > 8))
  {
    String[] list = split(incoming, " ");
    if ( (list.length > 0) && (list[0].equals("euler:")) ) 
    {
      roll  = float(list[1]);
      pitch = float(list[2]);
      yaw   = float(list[3]);
      buffer = incoming;
    }
  }
}

