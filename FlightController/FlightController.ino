#include <Wire.h>
#include <Servo.h>

/* --- SETTINGS --- */

int THROTTLE_MINIMUM = 1000;                        // Minimum throttle of a motor
int THROTTLE_MAXIMUM = 1800;                        // Maximum throttle of a motor

float COMPLEMENTARY_ANGLE = 0.98;                   // Complementary filter for combining acc and gyro

double throttle = 1000;                             // Desired throttle
float angle_desired[3] = {0, 0, 0};                 // Desired angle

float gain_p[3] = {1.5, 1.5, 0};                    // Gain proportional
float gain_i[3] = {0, 0, 0};                        // Gain integral
float gain_d[3] = {0.4, 0.4, 0};                // Gain derivetive

float filter = 0.9;                                // Complementary filter for pid

int mode = 0;                                       // Mode for testing purpose: 0 = all motors | 1 = motor 1 & 3 | 2 = motor 2 & 4

/* --- CONSTANTS --- */

#define PITCH 0                                     // Rotation forward/backward
#define ROLL 1                                      // Rotation left/right
#define YAW 2                                       // Rotation around center

#define MPU_ADDRESS 0x68

/* --- VARIABLES --- */

float error_current[3] = {0, 0, 0};                 // Current error
float error_prev[3] = {0, 0, 0};                    // Previous error

float pid_current[3] = {0, 0, 0};                   // PID weighted (!) sum of proportional, integral and derivitive error
float pid_p[3] = {0, 0, 0};                         // PID proportional error     
float pid_i[3] = {0, 0, 0};                         // PID integral error
float pid_d[3] = {0, 0, 0};                         // PID derivitive error

float angle_current[3];                             // Angle measured after filtering
float angle_acc[3];                                 // Angle measured using accelerometer
float angle_gyro[3];                                // Angle measured using gyro

int16_t angle_acc_raw[3];                           // Accelerator raw data
int16_t angle_gyro_raw[3];                          // Gyro raw data

float time_current;                                 // Current time
float time_prev;                                    // Previous time
double time_elapsed;                                // Elapsed time during the last loop

Servo motor_1;                                      // Motor front right
Servo motor_2;                                      // Motor front left
Servo motor_3;                                      // Motor back left
Servo motor_4;                                      // Motor back right

float rad_to_deg = 180/3.141592654;                 // Constant for convert radian to degrees

void setup() {
  /* Begin serial communication for remote control */
  Serial.begin(115200);
  
  Serial.println("Start");

  /* Begin the wire communication with the gyro */
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.println("Communication with gyro started");

  /* Set gyro's digital low pass filter to ~43Hz */
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  Serial.println("Gyro's low pass filter set");

  time_current = millis();

  /* Attach the motors */
  motor_1.attach(13);                                
  motor_2.attach(11);
  motor_3.attach(9);
  motor_4.attach(8);

  Serial.println("Motors attached");

  /* Calibrate the motors */
  calibrateMotors();

  Serial.println("Motors calibrated");
  Serial.println("----------- Starting -----------");
}


void loop() {
  /* Calculate elapsed time */
  time_prev = time_current;
  time_current = millis();
  time_elapsed = (time_current - time_prev) / 1000;

  /* Get gyro's data */
  readGyro();
  readAccelerometer();

  /* Filter the data to reduce noise */
  filterAngle();

  /* Recieve the remote controller's commands */
  recieveControl();

  /* Calculate PID */
  calculatePid();

  if(throttle > 1010) {
    /* Apply PID to all motors */
    setMotorPids();
  } else {
    /* Turn off all motors */
    setSpeedForAllMotors(THROTTLE_MINIMUM);
  }

  /* Send calculated and measured data to the remote controller */
  sendData(ROLL);
}


/**
 * Calculates all PID values
 */
void calculatePid() {
  /* Save previous errors */
  error_prev[PITCH] = error_current[PITCH];
  error_prev[ROLL] = error_current[ROLL];

  /* Calculate current error */
  error_current[PITCH] = angle_current[PITCH] - angle_desired[PITCH];
  error_current[ROLL] = angle_current[ROLL] - angle_desired[ROLL];

  /* Calculate weighted proportional error */
  pid_p[PITCH] = gain_p[PITCH] * error_current[PITCH];
  pid_p[ROLL] = gain_p[ROLL] * error_current[ROLL];

  /* Calculated weighted derivitive error */
  float pid_d_new[3];

  pid_d_new[PITCH] = gain_d[PITCH] * (error_current[PITCH] - error_prev[PITCH]) / time_elapsed;
  pid_d_new[ROLL] = gain_d[ROLL] * (error_current[ROLL] - error_prev[ROLL]) / time_elapsed;

  pid_d[PITCH] = filter * pid_d[PITCH] + (1 - filter) * pid_d_new[PITCH];
  pid_d[ROLL] = filter * pid_d[ROLL] + (1 - filter) * pid_d_new[ROLL];

  /* Calculate weighted sum of the PID */
  pid_current[PITCH] = pid_p[PITCH] + pid_i[PITCH] + pid_d[PITCH];
  pid_current[ROLL] = pid_p[ROLL] + pid_i[ROLL] + pid_d[ROLL];
}


/* 
* Sets the PID values to motors 
* THERE MIGHT BE SOME ERRORS WITH THE SIGNS of the pid variables
* TODO: Write down the orientation 
*/
void setMotorPids() {
  motor_1.writeMicroseconds(throttle + pid_current[PITCH] + pid_current[ROLL] );      // Set PID for front right motor
  motor_2.writeMicroseconds(throttle + pid_current[PITCH] - pid_current[ROLL] );      // Set PID for front left motor
  motor_3.writeMicroseconds(throttle - pid_current[PITCH] - pid_current[ROLL] );      // Set PID for back left motor
  motor_4.writeMicroseconds(throttle - pid_current[PITCH] + pid_current[ROLL] );      // Set PID for back right motor
  
}


/**
 * Calibrates all 4 motors
 */
void calibrateMotors() {
  setSpeedForAllMotors(THROTTLE_MINIMUM);
  delay(7000);
}


/**
 * Sets the given speed for all motors
 */
void setSpeedForAllMotors(double speed) {
  motor_1.writeMicroseconds(speed);
  motor_2.writeMicroseconds(speed);
  motor_3.writeMicroseconds(speed);
  motor_4.writeMicroseconds(speed);
}

/**
 * Low pass filter the gyro's data using a complementary filter 
 */
void filterAngle() {
  float angle_new[3];

  angle_new[PITCH] = -(COMPLEMENTARY_ANGLE * (-angle_current[PITCH] + angle_gyro[PITCH] * time_elapsed) + (1 - COMPLEMENTARY_ANGLE) * angle_acc[PITCH]);    // Positive angle -> forward
  angle_new[ROLL] = COMPLEMENTARY_ANGLE * (angle_current[ROLL] + angle_gyro[ROLL] * time_elapsed) + (1 - COMPLEMENTARY_ANGLE) * angle_acc[ROLL];            // Positive angle -> right
  angle_new[YAW] = COMPLEMENTARY_ANGLE * (angle_current[YAW] + angle_gyro[YAW] * time_elapsed) + (1 - COMPLEMENTARY_ANGLE) * angle_acc[YAW];                // Calculated by chris

  float value = 0.5;

  angle_current[PITCH] = value * angle_current[PITCH] + (1 - value) * angle_new[PITCH];
  angle_current[ROLL] = value * angle_current[ROLL] + (1 - value) * angle_new[ROLL];
  angle_current[YAW] = value * angle_current[YAW] + (1 - value) * angle_new[YAW];
}


/**
 * Reads the gyro and saves the values
 */
void readGyro() {
  /* Ask gyro for gyro data */
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true);

  /* Save recieved answer */
  angle_gyro_raw[PITCH] = Wire.read()<<8|Wire.read();
  angle_gyro_raw[ROLL] = Wire.read()<<8|Wire.read();
  angle_gyro_raw[YAW] = Wire.read()<<8|Wire.read();         // Added, did not check if that works

  /* Convert the data to degrees */
  angle_gyro[PITCH] = angle_gyro_raw[PITCH] / 131.0;
  angle_gyro[ROLL] = angle_gyro_raw[ROLL] / 131.0;
  angle_gyro[YAW] = angle_gyro_raw[YAW] / 131.0;              // Added, did not check if that works

  /* Adjust offsets */
  angle_gyro[0] = angle_gyro[0] + 2.5;
}


/**
 *  Reads the accelerometer and saves the values
 */
void readAccelerometer() {
  /* Ask gyro for acceleration data */
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true);

  /* Save recieved answer */
  angle_acc_raw[PITCH] = Wire.read()<<8|Wire.read();
  angle_acc_raw[ROLL] = Wire.read()<<8|Wire.read();
  angle_acc_raw[YAW] = Wire.read()<<8|Wire.read();

  /* Convert the data to g */
  angle_acc[PITCH] = atan((angle_acc_raw[ROLL] / 16384.0) / sqrt(pow((angle_acc_raw[PITCH] / 16384.0), 2) + pow((angle_acc_raw[YAW] / 16384.0), 2))) * rad_to_deg;
  angle_acc[ROLL] = atan(-1 * (angle_acc_raw[PITCH] / 16384.0) / sqrt(pow((angle_acc_raw[ROLL] / 16384.0), 2) + pow((angle_acc_raw[YAW] / 16384.0), 2))) * rad_to_deg;
  angle_acc[YAW] = angle_acc_raw[YAW] / 16384.0;                   // Calculated by my own, don't know if its correct...
  
}


/**
 * Recieve remote control's command
 */
void recieveControl() {
  if(Serial.available()) {
    int command = Serial.read();

    if(command == 'u') {
      throttle += 50;                                             // Increase throttle

      if(throttle >= THROTTLE_MAXIMUM) {
        throttle = THROTTLE_MAXIMUM;
      }
    } else if(command == 'd') {
      throttle -= 50;                                             // Decrease throttle

      if(throttle <= THROTTLE_MINIMUM) {
        throttle = THROTTLE_MINIMUM;
      }
    } else if(command == 'b') {
      throttle = THROTTLE_MINIMUM;                                // Turn off all motors
    } else if(command == 'q') {
      gain_p[PITCH] = gain_p[PITCH] + 0.1;                        // Increase P gain for pitch
      gain_p[ROLL] = gain_p[ROLL] + 0.1;                          // Increase P gain for roll
    } else if(command == 'a') {
      gain_p[PITCH] = gain_p[PITCH] - 0.1;                        // Decrease P gain for pitch
      gain_p[ROLL] = gain_p[ROLL] - 0.1;                          // Decrease P gain for roll
    } else if(command == 'w') {
      gain_d[PITCH] = gain_d[PITCH] + 0.05;                       // Increase D gain for pitch
      gain_d[ROLL] = gain_d[ROLL] + 0.05;                         // Increase D gain for roll
    } else if(command == 's') {
      gain_d[PITCH] = gain_d[PITCH] - 0.05;                       // Decrease D gain for pitch
      gain_d[ROLL] = gain_d[ROLL] - 0.05;                         // Decrease D gain for roll
    } else if(command == 'r') {
      angle_desired[ROLL] = angle_desired[ROLL] + 2;              // Move right
    } else if(command == 'l') {
      angle_desired[ROLL] = angle_desired[ROLL] - 2;              // Move left
    } else if(command == 't') {
      filter = filter + 0.005;                                    // Increase complementary filter
    } else if(command == 'g') {
      filter = filter - 0.005;                                    // Decrease complementary filter
    } else if(command == '0') {
      mode = 0;                                                   // Activate all motors
    } else if(command == '1') {
      mode = 1;                                                   // Activate motor 1 & 3
    } else if(command == '2') {
      mode = 2;                                                   // Activate motor 2 & 4
    }
  }
}


/**
 * Sends the controller's settings and measured data to the remote controller
 */
void sendData(int angleType) {
  Serial.println("S" + 
    String(angle_current[angleType]) + "|" + 
    String(throttle) + "|" + 
    String(angle_desired[angleType]) + "|" + 
    String(pid_current[angleType]) + "|" + 
    String(pid_p[angleType]) + "|" + 
    String(pid_d[angleType]) + "|" + 
    String(gain_p[angleType]) + "|" + 
    String(gain_d[angleType], 3) + "|" + 
    String(time_elapsed, 6) + "|" + 
    String(filter, 3) + "|" + 
    String(angle_gyro[angleType], 6) + "|" + 
    String(angle_acc[angleType], 6) + "|" + 
    String(mode) + "E");
}