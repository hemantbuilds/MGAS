#include <Wire.h> // Include the Wire library
#include <MPU6050.h> // Include the MPU6050 library#include <Adafruit_Sensor.h>
//#include <utility/imumaths.h>
#include <math.h>
#include <Servo.h>
#define DT 0.01
#define GYRO_SENSITIVITY 131
#define ACC_SENSITIVITY 17000 
#define GRAVITATIONAL_ACCELERATION 9.81
#define TIME_INTERVAL 0.1
#define PI 3.14159265

MPU6050 mpu; // Create a MPU6050 object

float gyro_range = 100;

float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

Servo pitchServo;
Servo rollServo;
//relay pin assignment
//int relayPin = 4; 
float pitchServoVal = 90;
float rollServoVal = 90;
int milliOld;
int milliNew;
int dt;
int in1=3,in2=4,EN=5;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;


// raw sensor data
int16_t ax, ay, az; // Variables to store accelerometer data
int16_t gx, gy, gz; // Variables to store gyroscope data

// variables to store calibration data
float AcX_cal, AcY_cal, AcZ_cal;
float GyX_cal, GyY_cal, GyZ_cal; 

// offset values
float AcX_offset, AcY_offset, AcZ_offset;
float GyX_offset, GyY_offset, GyZ_offset;

// LowPass filter variables
float gx_filtered, gy_filtered, gz_filtered = 0.0;
float ax_filtered, ay_filtered, az_filtered = 0.0;

// number of samples to use for calibration
int num_samples = 2000;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

float roll_deg, pitch_deg, yaw_deg = 0.0;

void setup() {
  // for printing on serial monitor
  rollServo.attach(12);
  pitchServo.attach(13);
  //pinMode(relayPin, OUTPUT);
  //digitalWrite(relayPin, HIGH);
  Serial.begin(19200); // Initialize serial communication at 9600 baud
  while (!Serial); // Wait for serial port to be available

  Wire.begin(); // Initialize the Wire library
  mpu.initialize(); // Initialize the MPU6050

  // Check if the MPU6050 is connected
  if (!mpu.testConnection()) {
    Serial.println("Error: MPU6050 not connected");
    while (1); // Stop the program if the MPU6050 is not connected
  }

  // sum of samples
  int16_t AcX_sum, AcY_sum, AcZ_sum = 0;
  int16_t GyX_sum, GyY_sum, GyZ_sum = 0;

  for(int i = 0; i < num_samples; i++) {
    // reading raw sample data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // adding to accelerometer sums
    AcX_sum += ax;
    AcY_sum += ay;
    AcZ_sum += az;

    // adding to gyro sums
    GyX_sum += gx;
    GyY_sum += gy;
    GyZ_sum += gz;
  }
  
  AcX_offset = AcX_sum / num_samples;
  AcY_offset = AcY_sum / num_samples;
  AcZ_offset = AcZ_sum / num_samples;
  
  GyX_offset = GyX_sum / num_samples;
  GyY_offset = GyY_sum / num_samples;
  GyZ_offset = GyZ_sum / num_samples;

  Serial.print("Gyro offsets : ");
  Serial.print(GyX_offset);
  Serial.print(" ");
  Serial.print(GyY_offset);
  Serial.print(" ");
  Serial.println(GyZ_offset);

  Serial.print("Accelerometer offsets : ");
  Serial.print(AcX_offset);
  Serial.print(" ");
  Serial.print(AcY_offset);
  Serial.print(" ");
  Serial.println(AcZ_offset);
  
  // indicate on pin 13 that calibration is done
  pinMode(11, OUTPUT);

  for(int i = 0; i < 10; i++) {
    digitalWrite(11, HIGH);
    delay(250);
    digitalWrite(11, LOW);
    delay(250);
  }
  rollServo.write(rollServoVal);
  delay(20);
  pitchServo.write(pitchServoVal);
  delay(20);
  

  Serial.println("Roll:, Pitch:, Yaw:");
}

void loop() {
  // Read the accelerometer and gyroscope data
  imu_read();

  milliOld = milliNew;
  milliNew = millis();
  dt = milliNew - milliOld;
  /*if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '0') {
      digitalWrite(relayPin, HIGH); 
    } else if (command == '1') {
      digitalWrite(relayPin, LOW); 
    }
  }*/
  }
int imu_read() {
  digitalWrite(11, HIGH);
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // calculate calibrated readings
  // convert accelerometer readings to g's
  AcX_cal = ((ax - AcX_offset) / 17000) * GRAVITATIONAL_ACCELERATION;
  AcY_cal = ((ay - AcY_offset) / 17000) * GRAVITATIONAL_ACCELERATION;
  AcZ_cal = ((az - AcZ_offset) / 17000) * GRAVITATIONAL_ACCELERATION;
  // convert gyro readings to radians per second
  GyX_cal = ((gx - GyX_offset) / GYRO_SENSITIVITY) * (PI / 180);
  GyY_cal = ((gy - GyY_offset) / GYRO_SENSITIVITY) * (PI / 180);
  GyZ_cal = ((gz - GyZ_offset) / GYRO_SENSITIVITY) * (PI / 180);

  // calculate roll, pitch and yaw using accelerometer
  float roll_acc = atan2(AcY_cal, AcZ_cal);
  float pitch_acc = atan2(-AcX_cal, sqrt(AcY_cal * AcY_cal + AcZ_cal * AcZ_cal));
  float yaw_acc = atan2(AcX_cal, sqrt(AcY_cal * AcY_cal + AcZ_cal * AcZ_cal));

  // calculate angular velocity
  float gyro_roll = GyX_cal * TIME_INTERVAL;
  float gyro_pitch = GyY_cal * TIME_INTERVAL;
  float gyro_yaw = GyZ_cal * TIME_INTERVAL;

  applyComplementaryFilter(roll_acc, pitch_acc, yaw_acc, gyro_roll, gyro_pitch, gyro_yaw);  
  float rollval = (roll * 90 / 1.5)+0.45+2;
  float pitchval = (pitch * 90 / 1.5)+6.5+0.9;
  //0yaw = yaw * 90 / 1.5;
  //float ax,ay,az;
  float ay1=  ay;
  float ax1= ax;
  float rawroll=(ay1/17000)*(90/1.5);
  float rawpitch=(ax1/17000)*(90/1.5)*(-1);
        
  // Serial.print(gyro_roll);
  // Serial.print(",");
  // Serial.print(gyro_pitch);
  // Serial.print(",");
  // Serial.println(gyro_yaw);

  // Serial.print(roll_acc);
  // Serial.print(",");
  // Serial.print(pitch_acc);
  // Serial.print(",");
  // Serial.println(yaw_acc);

  rollServoVal = map(rollval, -90, 90, 200, -20);

  pitchServoVal = map(pitchval, -90, 90, 200, -20);
  float pitchWrite=pitchServoVal+rollServoVal-90;
  float rollWrite=rollServoVal;
  pitchServo.write(pitchWrite);
  rollServo.write(rollWrite);
    digitalWrite(in1,HIGH);
   digitalWrite(in2,LOW);
   float mSpeed=150.+(105/10)*sqrt(AcX_cal*AcX_cal+AcY_cal*AcY_cal+AcZ_cal*AcZ_cal);
  

  for (int i = 0; i < mSpeed; i++) {
    EN=mSpeed;
    analogWrite(EN, i);}

  Serial.print(rollval);
  Serial.print(","); 
  Serial.print(rawroll);
  Serial.print(",");
  Serial.print(rollWrite);
  Serial.print(",");
  Serial.print(pitchval);
  Serial.print(",");
  Serial.print(rawpitch);
  Serial.print(",");
  Serial.println(pitchWrite);
  
  //Serial.print(yaw);
/*Serial.print("  gyroX=");
 Serial.print(GyX_cal);
 Serial.print("  gyroY="); 
 Serial.print(GyY_cal);
 Serial.print("  gyroZ=");
 Serial.print(GyZ_cal);
 Serial.print("  AccX=");
 Serial.print(AcX_cal);
 Serial.print("  AccY="); 
 Serial.print(AcY_cal);
 Serial.print("  AccZ=");
 Serial.println(AcZ_cal);*/


 

  delay(100);
}

void applyLowPassFilter() {
  float alpha = 0.5;
  // apply lowpass on gyro
   gx_filtered = alpha * gx_filtered + (1.0 - alpha) * GyX_cal;
   gy_filtered = alpha * gy_filtered + (1.0 - alpha) * GyY_cal;
   gz_filtered = alpha * gz_filtered + (1.0 - alpha) * GyZ_cal;
  // // apply lowpass on accelerometer
   ax_filtered = alpha * ax_filtered + (1.0 - alpha) * AcX_cal;
   ay_filtered = alpha * ay_filtered + (1.0 - alpha) * AcY_cal;
   az_filtered = alpha * az_filtered + (1.0 - alpha) * AcZ_cal;
    
}

void applyComplementaryFilter(float roll_acc, float pitch_acc, float yaw_acc, float gyro_roll, float gyro_pitch, float gyro_yaw) {
  float ALPHA = 0.5;

  roll = ALPHA * (roll + gyro_roll) + (1.0 - ALPHA) * roll_acc;
  pitch = ALPHA * (pitch + gyro_pitch) + (1.0 - ALPHA) * pitch_acc;
  yaw = ALPHA * (yaw + gyro_yaw) + (1.0 - ALPHA) * yaw_acc;
}
void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}
