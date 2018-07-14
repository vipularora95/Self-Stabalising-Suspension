import processing.serial.*;

int lf = 10;    // Linefeed in ASCII
String myString = null;
Serial myPort;  // Serial port you are using
float num,pan,tilt;
int temp,temp1;
float[] y,x;

void setup() {
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.clear();
  size(300, 700);
  y = new float[width-100];
  x = new float[width-100];
}

void draw() {
  if (mousePressed) {
    myPort.write("H");
  }
  while (myPort.available() > 0) {
    myString = myPort.readStringUntil(lf);
    if (myString != null) {
  //print(myString);  // Prints String
  num=float(myString);  // Converts and prints float
  //println(num);
  
  temp=int(num);
  temp1=temp/1000;
  
  //println(temp1);
  if (temp1>0)
  {
    pan=num-2000;
    print("Roll= ");
    println(pan);
  }
  else
  {
    tilt=num;
    print("Pitch= ");
    println(tilt);
  }//print it out in the console
  background(204); // Read the array from the end to the
  fill(255);
  rect(50,0+15,200,200);
  rect(50,225+15,200,200);
  rect(50,450+15,200,200);
  stroke(255,0,0);
  line(50,100+15,250,100+15);
  line(50,100+15+225,250,100+15+225);
  line(50,100+15+450,250,100+15+450);
  line(150,450+15,150,650+15);
  stroke(0);
  fill(0);
  textSize(18);
  textAlign(CENTER);
  text("PITCH",150,218+13);
  text("ROLL",150,218+13+225);
  text("PLOT",150,218+13+450);
  textSize(12);
  text("0°",35,100+6+15);
  text("0°",35,100+6+15+225);
  text("0°",35,100+6+15+450);
  strokeWeight(5);
  point(150-pan,100+15+450+tilt );
  strokeWeight(1);
  // beginning to avoid overwriting the data
  for (int i = y.length-1; i > 0; i--) {
    y[i] = y[i-1];
  }
  // Add new values to the beginning
  y[0] = -tilt+100+15;
  // Display each pair of values as a line
  for (int i = 1; i < y.length; i++) {
    line(50+i, y[i], 50+i-1, y[i-1]);
  }
  
  
  for (int i = x.length-1; i > 0; i--) {
    x[i] = x[i-1];
  }
  // Add new values to the beginning
  x[0] = -pan+100+15+225;
  // Display each pair of values as a line
  for (int i = 1; i < x.length; i++) {
    line(50+i, x[i], 50+i-1, x[i-1]);
  }
  
    }
  }
  myPort.clear();
} 