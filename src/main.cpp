#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Global Variables
bool imu_started = false;
bool gyro_calibrated = false;
long loop_timer;

// Define MPU Variables
const int mpu_address = 0x68;
int gyro_cal_int; // Gyroscope calibration int counter
float gyro_x_offset, gyro_y_offset, gyro_z_offset; // Gyroscope roll, pitch, yaw calibration offsets
float AccX, AccY, AccZ, temperature, GyroX, GyroY, GyroZ; // Raw MPU data
float total_vector_acc;
float roll_angle_acc, pitch_angle_acc;
float roll_angle, pitch_angle, yaw_angle;
float roll_level_adjust, pitch_level_adjust;

// Define Quadcopter Inputs
int throttle;
int reciever_roll_input, reciever_pitch_input, reciever_yaw_input;

// Define PID Controllers
float roll_Kp = 1.0; // Roll p gain
float roll_Ki = 0.0; // Roll i gain
float roll_Kd = 0.0; // Roll d gain
float roll_lim = 400.0; // Roll limit +/-
float gyro_roll_input, roll_setpoint, roll_error, roll_previous_error, roll_int_error, roll_output; // Input from gyroscope

float pitch_Kp = 1.0; // Pitch p gain
float pitch_Ki = 0.0; // Pitch i gain
float pitch_Kd = 0.0; // Pitch d gain
float pitch_lim = 400.0; // Pitch limit +/-
float gyro_pitch_input, pitch_setpoint, pitch_error, pitch_previous_error, pitch_int_error, pitch_output; // Input from gyroscope

float yaw_Kp = 1.0; // Yaw p gain
float yaw_Ki = 0.0; // Yaw i gain
float yaw_Kd = 0.0; // Yaw d gain
float yaw_lim = 400.0; // Yaw limit +/-
float gyro_yaw_input, yaw_setpoint, yaw_error, yaw_previous_error, yaw_int_error, yaw_output; // Input from gyroscope

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Radio
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Define data struct for recieving
// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package
{
  byte j1x_VAL;
  byte j1y_VAL;
  byte j1b_VAL;
  byte j2x_VAL;
  byte j2y_VAL;
  byte j2b_VAL;
  byte pot1_VAL;
  byte pot2_VAL;
  byte t1_VAL;
  byte t2_VAL;
  byte pb1_VAL;
  byte pb2_VAL;
  byte pb3_VAL;
  byte pb4_VAL;
};

Data_Package data; // Create a variable with the above structure

// Define RF
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Servos
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Define Servos
bool esc_armed = false;
int esc_armed_int = 0;
#define bm1_PIN 2 // Brushless motor 1
#define bm2_PIN 3 // Brushless motor 2
#define bm3_PIN 4 // Brushless motor 3
#define bm4_PIN 5 // Brushless motor 4
Servo BM1;
Servo BM2;
Servo BM3;
Servo BM4;
int bm1, bm2, bm3, bm4; // ESC input FL, FR, RL, RR

////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////

void resetData()
{
  data.j1x_VAL = 127;
  data.j1y_VAL = 127;
  data.j2x_VAL = 127;
  data.j2y_VAL = 127;
  data.j1b_VAL = 1;
  data.j2b_VAL = 1;
  data.pot1_VAL = 0;
  data.pot2_VAL = 0;
  data.t1_VAL = 0;
  data.t2_VAL = 0;
  data.pb1_VAL = 0;
  data.pb2_VAL = 0;
  data.pb3_VAL = 0;
  data.pb4_VAL = 0;
}

void getRCtransmission()
{
  if (radio.available()) // If data is available read it
  {
    radio.read(&data, sizeof(Data_Package));
    lastReceiveTime = millis(); // Get lastRecievedTime
  }

  currentTime = millis(); // Get currentTime
  
  if (currentTime - lastReceiveTime > 1000) // If data hasn't been read for over 1 second reset data
  {
    resetData();
  }
}

void setupMPUregisters()
{
  // Activate the MPU-6050
  Wire.beginTransmission(mpu_address); // Start communicating with the MPU-6050
  Wire.write(0x6B); // Send the requested starting register
  Wire.write(0x00); // Set the requested starting register
  Wire.endTransmission(); // End the transmission
  //Configure the accelerometer (+/- 8g)
  Wire.beginTransmission(mpu_address); // Start communicating with the MPU-6050
  Wire.write(0x1C); // Send the requested starting register
  Wire.write(0x10); // Set the requested starting register
  Wire.endTransmission(); // End the transmission
  // Configure the gyroscope (500 deg/s full scale)
  Wire.beginTransmission(mpu_address); // Start communicating with the MPU-6050
  Wire.write(0x1B); // Send the requested starting register
  Wire.write(0x08); // Set the requested starting register
  Wire.endTransmission(); // End the transmission
}

void readMPUdata()
{
  // Read raw gyroscope and accelerometer data
  Wire.beginTransmission(mpu_address); // Start communicating with the MPU-6050
  Wire.write(0x3B); // Send the requested starting register
  Wire.endTransmission(); // End the transmission
  Wire.requestFrom(mpu_address, 14); //Request 14 bytes from the MPU-6050
  //Wire.requestFrom(mpu_address, 14, true); //Request 14 bytes from the MPU-6050
  //while(Wire.available() < 14); // Wait until all the bytes are received ### ISSUE ON THIS LINE ###
  AccX = -(Wire.read() << 8 | Wire.read());
  AccY = -(Wire.read() << 8 | Wire.read());
  AccZ = (Wire.read() << 8 | Wire.read());
  temperature = Wire.read() << 8 | Wire.read();
  GyroX = -(Wire.read() << 8 | Wire.read());
  GyroY = -(Wire.read() << 8 | Wire.read());
  GyroZ = (Wire.read() << 8 | Wire.read());
}

void getRollPitch()
{
  // Step 1: Get accelerometer and gyroscope data
  readMPUdata();

  // Step 2: Subtract gyroscope offsets
  if (gyro_calibrated == true)
  {
    GyroX -= gyro_x_offset;
    GyroY -= gyro_y_offset;
    GyroZ -= gyro_z_offset;
  }

  // Step 3: Gyroscope angle calculations
  // 0.0000611 = dt / 65.5, where dt = 0.0004
  roll_angle += GyroX * 0.0000611;
  pitch_angle += GyroY * 0.0000611;
  
  // Step 4: Correct roll and pitch for IMU yawing
  // 0.000001066 = 0.0000611 * (PI / 180) -> sin function uses radians
  roll_angle -= pitch_angle * sin(GyroZ * 0.000001066); // If IMU has yawed transfer the pitch angle to the roll angel
  pitch_angle += roll_angle * sin(GyroZ * 0.000001066); // If IMU has yawed transfer the roll angle to the pitch angel
  
  // Step 5: Accelerometer angle calculation
  // 57.296 = 180 / PI -> asin function uses radians
  total_vector_acc = sqrt((AccX * AccX) + (AccY * AccY) + (AccZ * AccZ)); // Calculate the total accelerometer vector
  roll_angle_acc = asin((float)AccY / total_vector_acc) * -57.296;
  pitch_angle_acc = asin((float)AccX / total_vector_acc) * 57.296;

  // Step 6: Correct for roll_angle_acc and pitch_angle_acc offsets (found manually)
  roll_angle_acc -= 4.17;
  pitch_angle_acc -= 2.81;

  // Step 7: Set roll and pitch angle depending on if IMU has already started or not
  if(imu_started) // If the IMU is already started
  {
    roll_angle = roll_angle * 0.95 + roll_angle_acc * 0.05; // Correct the drift of the gyro roll angle with the accelerometer roll angle
    pitch_angle = pitch_angle * 0.95 + pitch_angle_acc * 0.05; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  }
  else // On first start
  {
    roll_angle = roll_angle_acc;
    pitch_angle = pitch_angle_acc;
    imu_started = true; // Set the IMU started flag
  }
  roll_level_adjust = roll_angle * 15; // Calculate the roll angle correction
  pitch_level_adjust = pitch_angle * 15; // Calculate the pitch angle correction
}

void getPIDoutput() // Get PID output
{
  // Roll
  gyro_roll_input = (gyro_roll_input * 0.7) + ((GyroX / 65.5) * 0.3); // 65.5 = 1 deg/s
  roll_error = gyro_roll_input - roll_setpoint;

  roll_int_error += roll_Ki * roll_error;
  if (roll_int_error > roll_lim) roll_int_error = roll_lim; // Deal with integral wind up
  else if (roll_int_error < -1 * roll_lim) roll_int_error = -1 * roll_lim;

  roll_output = (roll_Kp * roll_error) + roll_int_error + (roll_Kd * (roll_error - roll_previous_error));
  if(roll_output > roll_lim) roll_output = roll_lim; // Limit roll output
  else if(roll_output < roll_lim * -1) roll_output = roll_lim * -1;

  roll_previous_error = roll_error;

  // Pitch
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((GyroY / 65.5) * 0.3); // 65.5 = 1 deg/s
  pitch_error = gyro_pitch_input - pitch_setpoint;

  pitch_int_error += pitch_Ki * pitch_error;
  if (pitch_int_error > pitch_lim) pitch_int_error = pitch_lim; // Deal with integral wind up
  else if (pitch_int_error < -1 * pitch_lim) pitch_int_error = -1 * pitch_lim;

  pitch_output = (pitch_Kp * pitch_error) + pitch_int_error + (pitch_Kd * (pitch_error - pitch_previous_error));
  if(pitch_output > pitch_lim) pitch_output = pitch_lim; // Limit pitch output
  else if(pitch_output < pitch_lim * -1) pitch_output = pitch_lim * -1;

  pitch_previous_error = pitch_error;

  // Yaw
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((GyroZ / 65.5) * 0.3); // 65.5 = 1 deg/s
  yaw_error = gyro_yaw_input - yaw_setpoint;

  yaw_int_error += yaw_Ki * yaw_error;
  if (yaw_int_error > yaw_lim) yaw_int_error = yaw_lim; // Deal with integral wind up
  else if (yaw_int_error < -1 * yaw_lim) yaw_int_error = -1 * yaw_lim;

  yaw_output = (yaw_Kp * yaw_error) + yaw_int_error + (yaw_Kd * (yaw_error - yaw_previous_error));
  if(yaw_output > yaw_lim) yaw_output = yaw_lim; // Limit yaw output
  else if(yaw_output < yaw_lim * -1) yaw_output = yaw_lim * -1;

  yaw_previous_error = yaw_error;
}

void setup()
{
  //Serial.begin(57600);

  // Define radio communication
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  // Set default values
  resetData();

  // MPU setup (reset the sensor through the power management register)
  setupMPUregisters();

  // Servos setup
  BM1.attach(bm1_PIN, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds)
  BM2.attach(bm2_PIN, 1000, 2000);
  BM3.attach(bm3_PIN, 1000, 2000);
  BM4.attach(bm4_PIN, 1000, 2000);

  loop_timer = micros(); // Reset the loop timer
}

void loop()
{
  // Step 1: Get MPU data
  getRollPitch();
  //readMPUdata();
  
  gyro_roll_input = (gyro_roll_input * 0.7) + ((GyroX / 65.5) * 0.3); // 65.5 = 1 deg/s
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((GyroY / 65.5) * 0.3); // 65.5 = 1 deg/s
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((GyroZ / 65.5) * 0.3); // 65.5 = 1 deg/s
  /*
  gyro_roll_input = 0.0;
  gyro_pitch_input = 0.0;
  gyro_yaw_input = 0.0;
  */

  // Step 2: Get transmission from RC controller
  getRCtransmission();

  // Step 3: Arm esc's -> calibrate gyro -> run
  if ((esc_armed == false) && (gyro_calibrated == false))
  {
    bm1 = map(data.pot1_VAL, 0, 255, 1000, 2000);
    bm2 = bm1;
    bm3 = bm1;
    bm4 = bm1;
    if (esc_armed_int < 7500)
    {
      esc_armed_int += 1;
    }
    else if (esc_armed_int == 7500)
    {
      esc_armed = true;
      esc_armed_int += 1;
    }
  }
  else if ((esc_armed == true) && (gyro_calibrated == false))
  {
    bm1 = map(data.pot1_VAL, 0, 255, 1000, 2000);
    bm2 = bm1;
    bm3 = bm1;
    bm4 = bm1;
    if (gyro_cal_int < 2000)
    {
      gyro_x_offset += GyroX;
      gyro_y_offset += GyroY;
      gyro_z_offset += GyroZ;
      gyro_cal_int += 1;
    }
    else if (gyro_cal_int == 2000)
    {
      gyro_x_offset /= 2000;
      gyro_y_offset /= 2000;
      gyro_z_offset /= 2000;
      gyro_calibrated = true;
      gyro_cal_int += 1;
    }
  }
  else if ((esc_armed == true) && (gyro_calibrated == true))
  {
    throttle = map(data.pot1_VAL, 0, 255, 1000, 2000);
    reciever_roll_input = map(data.j2x_VAL, 0, 255, 1000, 2000);
    reciever_pitch_input = map(data.j2y_VAL, 0, 255, 1000, 2000);
    reciever_yaw_input = map(data.j1x_VAL, 0, 255, 1000, 2000);

    // Step 3: Calculate setpoints
    roll_setpoint = 0;
    if(reciever_roll_input > 1520) roll_setpoint = reciever_roll_input - 1520;
    else if(reciever_roll_input < 1480) roll_setpoint = reciever_roll_input - 1480;
    roll_setpoint -= roll_level_adjust; // Subtract roll angle correction from the standardized receiver roll input value
    roll_setpoint /= 3; // Divide roll setpoint for the PID roll controller by 3 to get angles in degrees

    pitch_setpoint = 0;
    if(reciever_pitch_input > 1520) pitch_setpoint = reciever_pitch_input - 1520;
    else if(reciever_pitch_input < 1480) pitch_setpoint = reciever_pitch_input - 1480;
    pitch_setpoint -= pitch_level_adjust; // Subtract pitch angle correction from the standardized receiver pitch input value
    pitch_setpoint /= 3; // Divide pitch setpoint for the PID pitch controller by 3 to get angles in degrees

    yaw_setpoint = 0;
    if(throttle > 1050) // Do not yaw when turning off the motors.
    {
      if(reciever_yaw_input > 1520) yaw_setpoint = reciever_yaw_input - 1520;
      else if(reciever_yaw_input < 1480) yaw_setpoint = reciever_yaw_input - 1480;
      yaw_setpoint /= 3; // Divide yaw setpoint for the PID yaw controller by 3 to get angles in degrees
    }

    // Step 4: Get PID output
    getPIDoutput();

    // Step 5: Calculate esc input
    if (throttle > 1800) throttle = 1800; // We need some room to keep full control at full throttle.
    bm1 = throttle - roll_output - pitch_output + yaw_output; // Calculate the pulse for bm1 (front-left - CW)
    bm2 = throttle + roll_output - pitch_output - yaw_output; // Calculate the pulse for bm2 (front-right - CCW)
    bm3 = throttle - roll_output + pitch_output - yaw_output; // Calculate the pulse for bm3 (rear-left - CCW)
    bm4 = throttle + roll_output + pitch_output + yaw_output; // Calculate the pulse for bm4 (rear-right - CW)
    // Keep the motors running
    if (bm1 < 1000) bm1 = 1000;
    if (bm2 < 1000) bm2 = 1000;
    if (bm3 < 1000) bm3 = 1000;
    if (bm4 < 1000) bm4 = 1000;
    // Limit the bm pulse to 2000us
    if(bm1 > 2000) bm1 = 2000;
    if(bm2 > 2000) bm2 = 2000;
    if(bm3 > 2000) bm3 = 2000;
    if(bm4 > 2000) bm4 = 2000;
  }

  BM1.writeMicroseconds(bm1);
  BM2.writeMicroseconds(bm2);
  BM3.writeMicroseconds(bm3);
  BM4.writeMicroseconds(bm4);
  
  while(micros() - loop_timer < 4000); // Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();
}