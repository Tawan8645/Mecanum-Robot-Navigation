// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ESP32S3_CONFIG_H
#define ESP32S3_CONFIG_H
#define ESP32S3
#define LED_PIN 48 //used for debugging status

//uncomment the base you're building
//#define ROBOT_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
// #define ROBOT_BASE SKID_STEER            // 4WD robot
#define ROBOT_BASE MECANUM               // Mecanum drive robot

//uncomment the motor driver you're using
// #define USE_GENERIC_2_IN_MOTOR_DRIVER      // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
// #define USE_GENERIC_1_IN_MOTOR_DRIVER   // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
#define USE_BTS7960_MOTOR_DRIVER        // BTS7970 Motor Driver using A4950 (<40V) module or DRV8833 (<10V)
// #define USE_ESC_MOTOR_DRIVER            // Motor ESC for brushless motors

//uncomment the IMU you're using
// #define USE_GY85_IMU
#define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU
// #define USE_QMI8658_IMU
// #define USE_HMC5883L_MAG
// #define USE_AK8963_MAG
// #define USE_AK8975_MAG
// #define USE_AK09918_MAG
// #define USE_QMC5883L_MAG
// #define MAG_BIAS { 0, 0, 0 }
// #define IMU_TWEAK {}
// #define MAG_TWEAK {}

//--------------------- I2C BUS 0 : IMU (default) ---------------------
#define SDA_PIN 2   // ใช้โดย Wire (IMU) — main.cpp เรียก Wire.begin(SDA_PIN, SCL_PIN) อยู่แล้ว
#define SCL_PIN 1

// ==================== I2C Pins สำหรับ GY219 (INA219) — บัสแยกจาก IMU ====================
//#define USE_INA219

//#define BATTERY_SDA_PIN 41
//#define BATTERY_SCL_PIN 40

// ปรับตามแบตเตอรี่ของคุณ
//#define BATTERY_MIN 9.0f
//#define BATTERY_MAX 12.6f

#define K_P1 0.10   //0.15
#define K_I1 0.05   //0.15                      
#define K_D1 0       //0.15   

#define K_P2 0.10  //0.15                     
#define K_I2 0.05      //0.15                      
#define K_D2 0       //0.15  

#define K_P3 0.10    //0.15                      
#define K_I3 0.05   //0.15                      
#define K_D3 0       //0.15  

#define K_P4 0.10 //0.15                      
#define K_I4 0.05 //0.15                      
#define K_D4 0       //0.15                     
/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

#define MOTOR_MAX_RPM 120
#define MAX_RPM_RATIO 0.8
#define MOTOR_OPERATING_VOLTAGE 12
#define MOTOR_POWER_MAX_VOLTAGE 12
#define MOTOR_POWER_MEASURED_VOLTAGE 12
#define COUNTS_PER_REV1 1224
#define COUNTS_PER_REV2 1200
#define COUNTS_PER_REV3 1225
#define COUNTS_PER_REV4 1146
#define WHEEL_DIAMETER 0.065
#define LR_WHEELS_DISTANCE 0.1975
#define PWM_BITS 8
#define PWM_FREQUENCY 8000

// Fixed pin numbers for ESP32-WROOM-32D 38 PIN VERSION
/// ENCODER PINS
#define MOTOR1_ENCODER_A 13
#define MOTOR1_ENCODER_B 12
#define MOTOR1_ENCODER_INV true  

#define MOTOR2_ENCODER_A 11
#define MOTOR2_ENCODER_B 10
#define MOTOR2_ENCODER_INV false 

#define MOTOR3_ENCODER_A 9 
#define MOTOR3_ENCODER_B 8
#define MOTOR3_ENCODER_INV true 

#define MOTOR4_ENCODER_A  14
#define MOTOR4_ENCODER_B  21
#define MOTOR4_ENCODER_INV false 

// Motor Pins
  #define MOTOR1_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 18
  #define MOTOR1_IN_B 17
  #define MOTOR1_INV false

  #define MOTOR2_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 16
  #define MOTOR2_IN_B 15
  #define MOTOR2_INV true

  #define MOTOR3_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 7
  #define MOTOR3_IN_B 6
  #define MOTOR3_INV true

  #define MOTOR4_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 4
  #define MOTOR4_IN_B 5
  #define MOTOR4_INV true

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -(pow(2, PWM_BITS) - 1)
#endif
