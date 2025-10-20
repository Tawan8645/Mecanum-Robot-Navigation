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

#ifndef MYROBOT_CONFIG_H
#define MYROBOT_CONFIG_H

#define LED_PIN 2
#define ESP32
//#define SOC_PCNT_SUPPORTED
#define BAUDRATE 115200
#define LINO_BASE MECANUM 

#define USE_BTS7960_MOTOR_DRIVER 
#define USE_BNO055_IMU
//#define USE_MPU9250_IMU
#define SDA_PIN 21 // specify I2C pins for esp32
#define SCL_PIN 22

#define K_P 0.1                             
#define K_I 0.05                             
#define K_D 0.05                            
/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/
#define MOTOR_MAX_RPM 60       
#define MAX_RPM_RATIO 0.8
#define MOTOR_OPERATING_VOLTAGE 12
#define MOTOR_POWER_MAX_VOLTAGE 12
#define MOTOR_POWER_MEASURED_VOLTAGE 12            
#define COUNTS_PER_REV1 1125 //960
#define COUNTS_PER_REV2 1074 //960
#define COUNTS_PER_REV3 1079
#define COUNTS_PER_REV4 1084
#define WHEEL_DIAMETER 0.055               
#define LR_WHEELS_DISTANCE 0.20           
#define PWM_BITS 8                         
#define PWM_FREQUENCY 8000

// Fixed pin numbers for ESP32-WROOM-32D 38 PIN VERSION
/// ENCODER PINS
#define MOTOR1_ENCODER_A 15
#define MOTOR1_ENCODER_B 2 
#define MOTOR1_ENCODER_INV true  

#define MOTOR2_ENCODER_A 4
#define MOTOR2_ENCODER_B 16
#define MOTOR2_ENCODER_INV false 

#define MOTOR3_ENCODER_A 17 
#define MOTOR3_ENCODER_B 5
#define MOTOR3_ENCODER_INV true 

#define MOTOR4_ENCODER_A  18
#define MOTOR4_ENCODER_B  19
#define MOTOR4_ENCODER_INV false 

// Motor Pins
  #define MOTOR1_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 13
  #define MOTOR1_IN_B 12
  #define MOTOR1_INV false

  #define MOTOR2_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 14
  #define MOTOR2_IN_B 27
  #define MOTOR2_INV true

  #define MOTOR3_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 26
  #define MOTOR3_IN_B 25
  #define MOTOR3_INV true

  #define MOTOR4_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 33
  #define MOTOR4_IN_B 32
  #define MOTOR4_INV false

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -(pow(2, PWM_BITS) - 1)
#endif
