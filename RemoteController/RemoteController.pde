import processing.serial.*;

Serial port;

String portStream;
float throttle = 0;
PFont f;

//Settings

int WINDOW_SIZE_X = 1120;
int WINDOW_SIZE_Y = 500;

//RectBarSize:

int RECT_BAR_SIZE_X = 100;
int RECT_BAR_SIZE_Y = 300;

FloatList graph1;
FloatList graph2;
FloatList graph3;

void setup() {
  size(1120, 750);
  
  graph1 = new FloatList();
  graph2 = new FloatList();
  graph3 = new FloatList();

  drawBackground();

  fill(0, 0, 0);

  f = createFont("Arial", 50, true); 
  textFont(f, 50);
  textAlign(CENTER);

  useColor(1);
  text("Loading...", 560, 315);

  printArray(Serial.list());
  port = new Serial(this, "COM3", 115200);
  port.bufferUntil('\n');

  println("Starting controller");
}

void draw() {
  if (portStream != null && portStream.charAt(0) == 'S' && portStream.charAt(portStream.length() - 3) == 'E') {

    String[] data = split(portStream.substring(1, portStream.length() - 3), '|');

    //println("Message: " + portStream);

    float angle = float(data[0]);
    float throttle = float(data[1]);
    float angle_desired = float(data[2]);
    float pid_total = float(data[3]);
    float pid_p = float(data[4]);
    float pid_d = float(data[5]);
    float gain_p = float(data[6]);
    float gain_d = float(data[7]);
    float time_elapsed = float(data[8]);
    float filter = float(data[9]);
    float gyro = float(data[10]);
    float acc = float(data[11]);
    
    println(portStream);

    drawBackground();
    
    //Draw display
    translate(0, -150);
    
    displayRotation(200, 330, 1, angle, angle_desired);
    displayCircle(920, 330, "Speed", 1, int(throttle), 1000, 1800);
    
    displayCircle(390, 500, "Prop.", 0.5, gain_p, 0, 5.0);
    displayCircle(560, 500, "Integral", 0.5, 0, 0, 5.0);
    displayCircle(730, 500, "Derivitive", 0.5, gain_d, 0, 5.0);
    
    displayBar(430, 200, "Error", 1, pid_total, 0, 150);
    displayBar(560, 200, "P Error", 1, pid_p, 0, 150);
    displayBar(690, 200, "D Error", 1, pid_d, 0, 150);
    
    displayCircle(250, 550, "Calculations", 0.4, 1 / time_elapsed, 0, 3000);
    displayCircle(870, 550, "Filter", 0.4, filter, 0.5, 1);
    
    translate(0, 150);
    
    //Draw graph
    displayGraph(gyro, acc, angle);
    
  } else if (portStream != null) {
    println("Undefined data: '" + portStream + "'");
  } else {
    //println("No data available");
  }
}

void displayGraph(float graph1New, float graph2New, float graph3New) {  
  translate(100, 600);
  
  // Draw grid
  useColor(3);
  strokeWeight(1);
  
  for(int i = -90; i <= 90; i += 10) {
    line(0, i, 920, i);
  }
  
  for(int i = 0; i <= 920; i += 20) {
    line(i, 90, i, -90);
  }
  
  stroke(0, 0, 0);
  line(0, 0, 920, 0);
  
  int stepSize = 2;
  
  
  // Graph 1
  if(graph1.size() > 460) {
    graph1.remove(0);
  }
  
  graph1.append(graph1New);
  
  strokeWeight(2);
  
  // Draw line
  stroke(255, 0, 0);
  for(int i = 0; i < graph1.size() - 1; i++) {
    line(i * stepSize, graph1.get(i), (i + 1) * stepSize, graph1.get(i + 1));
  }
  
  
  // Graph 2
  if(graph2.size() > 460) {
    graph2.remove(0);
  }
  
  graph2.append(graph2New);
  
  strokeWeight(2);
  
  // Draw line
  useColor(1);
  for(int i = 0; i < graph2.size() - 1; i++) {
    line(i * stepSize, graph2.get(i), (i + 1) * stepSize, graph2.get(i + 1));
  }
  
  
  // Graph 3
  if(graph3.size() > 460) {
    graph3.remove(0);
  }
  
  graph3.append(graph3New);
  
  strokeWeight(2);
  
  // Draw line
  useColor(2);
  for(int i = 0; i < graph3.size() - 1; i++) {
    line(i * stepSize, graph3.get(i), (i + 1) * stepSize, graph3.get(i + 1));
  }
  
  translate(-100, -600);
}

// Display for the rotation
void displayRotation(int posX, int posY, float scale, float value, float desiredValue) {
  // Display: Angle
  pushMatrix();
  
  translate(posX, posY);
  scale(scale);
  
  useColor(3);
  ellipse(0, 0, 300, 300);

  useColor(0);
  ellipse(0, 0, 280, 280);

  useColor(3);
  rectRotatedMirrored(0, 0, 250, 10, desiredValue);
  
  useColor(1);
  rectRotatedMirrored(0, 0, 250, 10, value);

  useColor(0);
  rectRotatedMirrored(0, 150, 140, 60, 0);

  useColor(1);
  textFont(f, 30);
  text("Rotation", 0, 140);
  
  popMatrix();
}

void displayMotorSpeeds(int posX, int posY, float scale, float value1, float value2) {
  // Display: Angle
  pushMatrix();
  
  translate(posX, posY);
  scale(scale);
  
  useColor(3);
  ellipse(0, 0, 300, 300);

  useColor(0);
  ellipse(0, 0, 280, 280);

  useColor(1);
  rectRotated(0, 0, 125, 10, value1);
  
  useColor(1);
  rectRotated(0, 0, 125, 10, value2);

  useColor(0);
  rectRotatedMirrored(0, 150, 140, 60, 0);

  useColor(1);
  textFont(f, 30);
  text("Rotation", 0, 140);
  
  popMatrix();
}

void displayBar(int posX, int posY, String text, float scale, float value, float min, float max) {
  pushMatrix();
  
  value = abs(value);
  
  translate(posX - 40, posY);
  scale(scale);
  
  useColor(3);
  rect(0, 0, 80, 150);
  
  useColor(2);
  rect(0, 150, 80, - (value - min) / (max - min) * 150);
  
  useColor(0);
  textFont(f, 30);
  text(int(value), 40, 100);
  
  useColor(2);
  textAlign(CENTER);
  textFont(f, 20);
  text(text, 40, 180);
  
  popMatrix();
}

void displayCircle(int posX, int posY, String text, float scale, float value, float min, float max) {
  // Display: Angle
  pushMatrix();
  
  translate(posX, posY);
  scale(scale);
  
  useColor(3);
  ellipse(0, 0, 300, 300);
  
  useColor(1);
  rectRotated(0, 0, 150, 10, 120 + ((value - min) / (max - min)) * 300);

  useColor(0);
  ellipse(0, 0, 280, 280);

  useColor(1);
  rectRotated(0, 0, 120, 10, 120 + ((value - min) / (max - min)) * 300);

  useColor(0);
  rectRotatedMirrored(0, 150, 140, 80, 0);

  useColor(1);
  textFont(f, 30);
  
  text(value, 0, 70);
  text(text, 0, 140);
  
  popMatrix();
}

// Draws a rect rotated mirrored around it's center
void rectRotatedMirrored(int posX, int posY, int sizeX, int sizeY, float angle) {
  // Isolate rotation
  pushMatrix();

  // Create and translate
  translate(posX, posY);
  rotate(radians(angle));
  rect(0 - sizeX / 2, 0 - sizeY / 2, sizeX, sizeY); 

  // Isolate
  popMatrix();
}

// Draw a rect rotated around it's center
void rectRotated(int posX, int posY, int sizeX, int sizeY, float angle) {
  // Isolate rotation
  pushMatrix();

  // Create and translate
  translate(posX, posY);
  rotate(radians(angle));
  rect(0, 0, sizeX, sizeY); 

  // Isolate
  popMatrix();
}

void keyPressed() {
  if (keyCode == UP) {
    println("Increasing throttle");
    port.write('u');
  } else if (keyCode == DOWN) {
    println("Descresing throttle");
    port.write('d');
  } else if (keyCode == LEFT) {
    println("Turning left");
    port.write('l');
  } else if (keyCode == RIGHT) {
    println("Turning right");
    port.write('r');
  } else if (key == 'q') {
    println("Incresing p");
    port.write('q');
  } else if (key == 'a') {
    println("Decreasing p");
    port.write('a');
  } else if (key == 'w') {
    println("Increasing d");
    port.write('w');
  } else if (key == 's') {
    println("Decreasing d");
    port.write('s');
  } else if (key == 'e') {
    println("Increasing filter");
    port.write('t');
  } else if (key == 'd') {
    println("Decreasing filter");
    port.write('g');
  } else if (key == '1') {
    println("Mode 1");
    port.write('0');
  } else if (key == '2') {
    println("Mode 2");
    port.write('1');
  } else if (key == '3') {
    println("Mode 3");
    port.write('2');
  } else {
    println("BREAK");
    port.write('b');
  }
}

//Draws the background. This is used to "delete" the current drawing
void drawBackground() {
  useColor(0);
  rect(0, 0, WINDOW_SIZE_X, 750);
}

// Changes the color to one of the in the pattern included ones
void useColor(int index) {
  // White for background
  if (index == 0) {
    stroke(255, 255, 255);
    fill(255, 255, 255);
  }

  // Blue for main displays
  if (index == 1) {
    stroke(0, 204, 204);
    fill(0, 204, 204);
  }

  // Orange for secondary displays
  if (index == 2) {
    stroke(231, 118, 50);
    fill(231, 118, 50);
  }

  // Gray for display backgrounds
  if (index == 3) {
    stroke(208, 208, 208);
    fill(208, 208, 208);
  }

  // Blue for header (?)
  if (index == 4) {
    stroke(89, 118, 148);
    fill(89, 118, 148);
  }
}

void serialEvent(Serial myPort) {
  portStream = myPort.readString();
}
