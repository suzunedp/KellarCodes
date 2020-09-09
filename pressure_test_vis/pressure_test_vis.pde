import processing.serial.*;

Serial myPort;

int x = 0;
int alpha = 0;

void setup(){
  size(1028, 768);
  noStroke();
  myPort = new Serial(this, "/dev/cu.usbmodem141201", 9600);
  
}

void draw(){
  background(255);
  alpha = 100 - (int)map(x, 0, 255, 0, 80);
  fill(#eb4034, alpha);
  ellipse(width/2,height/2,512,512);
}

void serialEvent(Serial p){
  x = p.read();
  println(x);
}
