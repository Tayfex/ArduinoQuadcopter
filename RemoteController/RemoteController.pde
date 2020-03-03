import net.java.games.input.*;
import org.gamecontrolplus.*;
import org.gamecontrolplus.gui.*;

import processing.serial.*;

// --- SETTINGS ---

int WINDOW_SIZE_X = 1120;               // Window width
int WINDOW_SIZE_Y = 500;                // Window height

int RECT_BAR_SIZE_X = 100;              // Rect bar width
int RECT_BAR_SIZE_Y = 300;              // Rect bar height

// --- VARIABLES ---

float angle = 0;                        // Current angle
float throttle = 0;                     // Current throttle
float angle_desired = 0;                // Desired angle
float pid_total = 0;                    // Total PID value
float pid_p = 0;                        // PID P value
float pid_d = 0;                        // PID D value
float gain_p = 0;                       // PID P gain
float gain_d = 0;                       // PID D gain
float time_elapsed = 0;                 // Elapsed time
float filter = 0;                       // Filter strength
float gyro = 0;                         // Gyro value
float acc = 0;                          // Accelerometer value

int throttleGamepad = 0;

FloatList graph1;                       // Gyro history
FloatList graph2;                       // Acc history
FloatList graph3;                       // Angle history

Serial port;                            // Port for serial communication
String portStream;                      // Port input stream

PFont f;                                // GUI font

boolean setupFinished = false;          // True as soon as loading has finished

ControlDevice controller;               // Gamepad controller
ControlIO control;

void setup() {
  // Initialize lists
  graph1 = new FloatList();
  graph2 = new FloatList();
  graph3 = new FloatList();

  // Load font
  f = createFont("Arial", 50, true); 
  textFont(f, 50);
  textAlign(CENTER);

  // Prepare window
  size(1120, 750);
  fill(0, 0, 0);
  drawDisplay();

  // Display loading screen
  /*useColor(1);
   text("Loading...", 560, 315);*/
  drawInfoBox("Loading");

  // Start communication with Arduino
  printArray(Serial.list());
  port = new Serial(this, "COM3", 115200);

  // Start communication with gamepad
  control = ControlIO.getInstance(this);
  controller = control.filter(GCP.GAMEPAD).getMatchedDevice("xboxController");

  // Check if gamepad has been found
  if (controller == null) {
    println("Gamepad not found");
    System.exit(-1);
  }
}

void draw() {
  
  // Redraw display whenever recieving new data
  if (port.available() > 0) {

    // Read port stream
    portStream = port.readStringUntil('\n');

    // ----- Drone in FLY MODE -----
    if (setupFinished && portStream != null) {

      // Send gamepad data
      sendGamepadInput();

      // Read again if recieve data are invalid because this might be due to the cleaning of the buffer
      if (!(portStream.length() > 3 && portStream.charAt(0) == 'B' && portStream.charAt(portStream.length() - 3) == 'E')) {
        portStream = port.readStringUntil('\n');
      }

      // Check if recieved data matches protocol
      if (portStream.length() > 3 && portStream.charAt(0) == 'B' && portStream.charAt(portStream.length() - 3) == 'E') {

        // Save recieved data
        String[] data = split(portStream.substring(1, portStream.length() - 3), '|');

        // Check if recieved data are complete
        if (data.length > 12) {

          // Unpack data
          angle = float(data[0]);
          throttle = float(data[1]);
          angle_desired = float(data[2]);
          pid_total = float(data[3]);
          pid_p = float(data[4]);
          pid_d = float(data[5]);
          gain_p = float(data[6]);
          gain_d = float(data[7]);
          time_elapsed = float(data[8]);
          filter = float(data[9]);
          gyro = float(data[10]);
          acc = float(data[11]);

          println(portStream);

          // Redraw display
          drawDisplay();
        } else {

          // Print error
          println("ERROR during runtime | Data incomplete: " + portStream);
        }
      } else {

        // Print error
        println("ERROR during runtime | Invalid data: " + portStream);
      }

      // Clear buffer after reading to avoid delay due to buffer overflow
      if (port.available() > 500) {

        // Clear the buffer except the last 300 bytes
        port.readBytes(port.available() - 300);
      }
    } else if (portStream != null) {
      // ----- Drone in SETUP MODE -----

      // Read port's full stream!
      // Because the draw() loop is way slower then the arduino sending data the whole buffer has to be made empty within one draw() loop
      // Otherwise the buffer is overflowing over time which is causing a huge delay! 
      // At the same time it has to be made sure that none of the messages is accidentially skipped during SETUP MODE (reliable communication)
      // In FLY MODE this isn't necessary anymore (unreliable communication)
      while (portStream != null) {

        // Check if data is setup message
        if (portStream.length() > 5 && portStream.indexOf("SETUP") != -1) {

          // Print message on display
          drawInfoBox(portStream.substring(portStream.indexOf("SETUP") + 7));

          // Check if setup is finished
          if (portStream.length() > 15 && portStream.substring(0, 15).equals("SETUP: Finished")) {
            setupFinished = true;
          }
        } else {

          // Print error
          println("ERROR during setup | Invalid  data: " + portStream);
        }

        // Read the next complete message to make sure that the buffer is emtpy except the last bytes
        // which might be the first bytes of the next not yet full recieved message
        portStream = port.readStringUntil('\n');
      }
    }
  }
}

void sendGamepadInput() {
  // Read gamepad
  int throttleGamepadNew = (int) map(controller.getSlider("throttle").getValue(), 0, -1, 1000, 1300);

  if (throttleGamepadNew != throttleGamepad) {
    throttleGamepad = throttleGamepadNew;

    if (throttleGamepad < 1000) {
      throttleGamepad = 1000;
    }

    // Send gamepad throttle
    port.write(".t" + str(throttleGamepad) + ";");
  }
}

void drawDisplay() {
  // --- Draw display ---
  drawBackground();
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
}

void displayGraph(float graph1New, float graph2New, float graph3New) {  
  translate(100, 600);

  // Draw grid
  useColor(3);
  strokeWeight(1);

  for (int i = -90; i <= 90; i += 10) {
    line(0, i, 920, i);
  }

  for (int i = 0; i <= 920; i += 20) {
    line(i, 90, i, -90);
  }

  stroke(0, 0, 0);
  line(0, 0, 920, 0);

  int stepSize = 2;


  // Graph 1
  if (graph1.size() > 460) {
    graph1.remove(0);
  }

  graph1.append(graph1New);

  strokeWeight(2);

  // Draw line
  stroke(255, 0, 0);
  for (int i = 0; i < graph1.size() - 1; i++) {
    line(i * stepSize, graph1.get(i), (i + 1) * stepSize, graph1.get(i + 1));
  }


  // Graph 2
  if (graph2.size() > 460) {
    graph2.remove(0);
  }

  graph2.append(graph2New);

  strokeWeight(2);

  // Draw line
  useColor(1);
  for (int i = 0; i < graph2.size() - 1; i++) {
    line(i * stepSize, graph2.get(i), (i + 1) * stepSize, graph2.get(i + 1));
  }


  // Graph 3
  if (graph3.size() > 460) {
    graph3.remove(0);
  }

  graph3.append(graph3New);

  strokeWeight(2);

  // Draw line
  useColor(2);
  for (int i = 0; i < graph3.size() - 1; i++) {
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

void drawInfoBox(String text) {
  translate(0, 100);

  float width = WINDOW_SIZE_X * 0.5;
  float height = WINDOW_SIZE_Y * 0.4;

  useColor(1);
  rect(WINDOW_SIZE_X / 2 - width / 2, WINDOW_SIZE_Y / 2 - height / 2, width, height, 7);

  width -= 10;
  height -= 10;

  useColor(0);
  rect(WINDOW_SIZE_X / 2 - width / 2, WINDOW_SIZE_Y / 2 - height / 2, width, height, 7);

  width -= 10;
  height -= 10;

  useColor(1);
  rect(WINDOW_SIZE_X / 2 - width / 2, WINDOW_SIZE_Y / 2 - height / 2, width, height, 7);

  useColor(0);
  textSize(50);
  text(text, WINDOW_SIZE_X / 2, WINDOW_SIZE_Y / 2);

  translate(0, -100);
}

void keyPressed() {
  if (keyCode == UP) {
    println("Increasing throttle");
    port.write("throttle+;");
  } else if (keyCode == DOWN) {
    println("Descresing throttle");
    port.write("throttle-;");
  } else if (keyCode == LEFT) {
    println("Turning left");
    port.write("left;");
  } else if (keyCode == RIGHT) {
    println("Turning right");
    port.write("right;");
  } else if (key == 'q') {
    println("Incresing p");
    port.write("gainP+;");
  } else if (key == 'a') {
    println("Decreasing p");
    port.write("gainP-;");
  } else if (key == 'w') {
    println("Increasing d");
    port.write("gainD+;");
  } else if (key == 's') {
    println("Decreasing d");
    port.write("gainD-;");
  } else if (key == 'e') {
    println("Increasing filter");
    port.write("filter+;");
  } else if (key == 'd') {
    println("Decreasing filter");
    port.write("filter-;");
  } else if (key == '1') {
    println("Mode 1");
    port.write("mode0;");
  } else if (key == '2') {
    println("Mode 2");
    port.write("mode1;");
  } else if (key == '3') {
    println("Mode 3");
    port.write("mode2;");
  } else {
    println("STOP");
    port.write("stop;");
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
