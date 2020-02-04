#include <Wire.h>
#include <Servo.h>

/* --- SETTINGS --- */

int THROTTLE_MINIMUM = 1000;                        // Minimum throttle of a motor
int THROTTLE_MAXIMUM = 1800;                        // Maximum throttle of a motor

double throttle = 1000;                             // Desired throttle
float angle_desired[3] = {0, 0, 0};                 // Desired angle

float gain_p[3] = {1.5, 0, 0};                      // Gain proportional
float gain_i[3] = {0, 0, 0};                        // Gain integral
float gain_d[3] = {0.415, 0, 0};                    // Gain derivetive

float filter = 0.98;                                // Complementary filter

int mode = 0;                                       // Mode for testing purpose: 0 = all motors | 1 = motor 1 & 3 | 2 = motor 2 & 4

/* --- CONSTANTS --- */

#define PITCH 0                                     // Rotation forward/backward
#define ROLL 1                                      // Rotation left/right
#define YAW 2                                       // Rotation left/right

#define X 0
#define Y 1
#define Z 2

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
Servo motor_3;                                      // Motor back right
Servo motor_4;                                      // Motor back left

float rad_to_deg = 180/3.141592654;                 // Constant for convert radian to degrees

void setup() {
  /* Begin the wire communication with the gyro */
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  /* Set gyro's digital low pass filter to ~43Hz */
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  /* Begin serial communication for remote control */
  Serial.begin(115200);

  time_current = millis();

  /* Attach the motors */
  motor_1.attach(8);
  motor_2.attach(9);
  motor_3.attach(10);
  motor_4.attach(11);

  /* Calibrate the motors */
  calibrateMotors();
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
  filterData();

  /* Recieve the remote controller's commands */
  recieveControl();

  if(throttle > 1010) {
    /* Calculate PID */
    calculatePid();

    /* Apply PID to all motors */
    setMotorPids();
  } else {
    /* Turn off all motors */
    setSpeedForAllMotors(THROTTLE_MINIMUM);
  }

  /* Send calculated and measured data to the remote controller */
  sendData();
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
  pid_d[PITCH] = gain_d[PITCH] * (error_current[PITCH] - error_prev[PITCH]) / time_elapsed;
  pid_d[ROLL] = gain_d[ROLL] * (error_current[ROLL] - error_prev[ROLL]) / time_elapsed;

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
  motor_3.writeMicroseconds(throttle - pid_current[PITCH] + pid_current[ROLL] );      // Set PID for back right motor
  motor_4.writeMicroseconds(throttle - pid_current[PITCH] - pid_current[ROLL] );      // Set PID for back left motor
  
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
void filterData() {
  angle_current[X] = filter * (angle_current[X] + angle_gyro[X] * time_elapsed) + (1 - filter) * angle_acc[X];
  angle_current[Y] = filter * (angle_current[Y] + angle_gyro[Y] * time_elapsed) + (1 - filter) * angle_acc[Y];
  angle_current[Z] = filter * (angle_current[Z] + angle_gyro[Z] * time_elapsed) + (1 - filter) * angle_acc[Z];        // Calculated by chris
}


/**
 * Reads the gyro and saves the values 
 */
void readGyro() {
  /* Ask gyro for gyro data */
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 4, true);

  /* Save recieved answer */
  angle_gyro_raw[X] = Wire.read()<<8|Wire.read();
  angle_gyro_raw[Y] = Wire.read()<<8|Wire.read();
  angle_gyro_raw[Z] = Wire.read()<<8|Wire.read();         // Added, did not check if that works

  /* Convert the data to degrees */
  angle_gyro[X] = angle_gyro_raw[X] / 131.0;
  angle_gyro[Y] = angle_gyro_raw[Y] / 131.0;
  angle_gyro[Z] = angle_gyro_raw[Z] / 131.0;              // Added, did not check if that works

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
  angle_acc_raw[X] = Wire.read()<<8|Wire.read();
  angle_acc_raw[Y] = Wire.read()<<8|Wire.read();
  angle_acc_raw[Z] = Wire.read()<<8|Wire.read();

  /* Convert the data to g */
  angle_acc[X] = atan((angle_acc_raw[Y] / 16384.0) / sqrt(pow((angle_acc_raw[X] / 16384.0), 2) + pow((angle_acc_raw[Z] / 16384.0), 2))) * rad_to_deg;
  angle_acc[Y] = atan(-1 * (angle_acc_raw[X] / 16384.0) / sqrt(pow((angle_acc_raw[Y] / 16384.0), 2) + pow((angle_acc_raw[Z] / 16384.0), 2))) * rad_to_deg;
  angle_acc[Z] = angle_acc_raw[Z] / 16384.0;                   // Calculated by my own, don't know if its correct...
  
}


/**
 * Sends the controller's settings and measured data to the remote controller
 */
void sendData() {
  Serial.println("S" + 
    String(angle_current[1]) + "|" + 
    String(throttle) + "|" + 
    String(angle_desired[0]) + "|" + 
    String(pid_current[0]) + "|" + 
    String(pid_p[0]) + "|" + 
    String(pid_d[0]) + "|" + 
    String(gain_p[0]) + "|" + 
    String(gain_d[0], 3) + "|" + 
    String(time_elapsed, 6) + "|" + 
    String(filter, 3) + "|" + 
    String(angle_gyro[0], 6) + "|" + 
    String(angle_acc[0], 6) + "|" + 
    String(mode) + "E");
}