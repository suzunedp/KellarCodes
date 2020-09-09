
import processing.serial.*;

Serial myPort;

int x;

void setup(){
  size(1028, 768);
  strokeWeight(64);
  fill(#eb4034);
  myPort = new Serial(this, "/dev/cu.usbmodem141201", 9600);
}

void draw(){
  background(255);
  translate(width/2,height/2);
  map(x, 0, 255, 0, 360);
  rotate(-(x-180)*PI/180);
  ellipse(0,0,512,512);
  line(0, -256, 0, 256);
}

void serialEvent(Serial p){
  x = p.read();
}
