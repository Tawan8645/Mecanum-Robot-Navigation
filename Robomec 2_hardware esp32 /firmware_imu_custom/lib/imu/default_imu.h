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

#ifndef DEFAULT_IMU
#define DEFAULT_IMU

//include IMU base interface
#include "imu_interface.h"

//include sensor API headers
#include "I2Cdev.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"



#include "Arduino_NineAxesMotion.h"


class BNO055IMU: public IMUInterface
{
    private:
        const float accel_scale_ = 1 / 16384.0;
        const float gyro_scale_ = 1 / 131.0;
        const float mag_scale_ = 0.3;
        const float quat_scale_ = 1 / 16384.0;

        NineAxesMotion accelerometer_;
        NineAxesMotion gyroscope_;
        NineAxesMotion magnetometer_;
        NineAxesMotion quaternion_;


        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;
        geometry_msgs__msg__Vector3 mag_;
        geometry_msgs__msg__Quaternion quad_;

    public:
        BNO055IMU()
        {
        }

        bool startSensor() override
        {
            Wire.begin();
            bool ret;
            //accelerometer_.initialize();
            accelerometer_.initSensor(0x28);
            //ret = accelerometer_.testConnection();
            ret = true;
            accelerometer_.setOperationMode(OPERATION_MODE_NDOF);
            accelerometer_.setUpdateMode(MANUAL);
            accelerometer_.updateAccelConfig();
            //use system status ? 
            if(!ret)
                return false;

            //gyroscope_.initialize();
            //ret = gyroscope_.testConnection();
            //if(!ret)
            //    return false;

            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            //int16_t ax, ay, az;

            accelerometer_.updateAccel();
            accel_.x = accelerometer_.readAccelX();
            accel_.y = accelerometer_.readAccelY();
            accel_.z = accelerometer_.readAccelZ();


            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            int16_t gx, gy, gz;

            //gyroscope_.getRotation(&gx, &gy, &gz);
            gyroscope_.updateGyro();
            gx = gyroscope_.readGyroX();
            gy = gyroscope_.readGyroY();
            gz = gyroscope_.readGyroZ();
            gyro_.x = gx * DEG_TO_RAD;
            gyro_.y = gy * DEG_TO_RAD;
            gyro_.z = gz * DEG_TO_RAD;

            return gyro_;
        }

        geometry_msgs__msg__Quaternion readQuaternion() override
        {
            //int16_t qw, qx, qy, qz;

            quaternion_.updateQuat();
            //qw = quaternion_.readQuatW();
            //qx = quaternion_.readQuatX();
            //qy = quaternion_.readQuatY();
            //qz = quaternion_.readQuatZ();

            //quad_.x = qx * quat_scale_;
            //quad_.y = qy * quat_scale_;
            //quad_.z = qz * quat_scale_;
            //quad_.w = qw * quat_scale_;

            quad_.x = quaternion_.readQuatX() * quat_scale_;
            quad_.y = quaternion_.readQuatY() * quat_scale_;
            quad_.z = quaternion_.readQuatZ() * quat_scale_;
            quad_.w = quaternion_.readQuatW() * quat_scale_;

            return quad_;

        }


};

class FakeIMU: public IMUInterface 
{
    private:
        geometry_msgs__msg__Vector3 accel_;
        geometry_msgs__msg__Vector3 gyro_;

    public:
        FakeIMU()
        {
        }

        bool startSensor() override
        {
            return true;
        }

        geometry_msgs__msg__Vector3 readAccelerometer() override
        {
            return accel_;
        }

        geometry_msgs__msg__Vector3 readGyroscope() override
        {
            return gyro_;
        }
};

#endif
//ADXL345 https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
//HMC8553L https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
//ITG320 https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf


//MPU9150 https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
//MPU9250 https://www.invensense.com/wp-content/uploads/2015/02/MPU-9150-Datasheet.pdf
//MPU6050 https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf

//http://www.sureshjoshi.com/embedded/invensense-imus-what-to-know/
//https://stackoverflow.com/questions/19161872/meaning-of-lsb-unit-and-unit-lsb
