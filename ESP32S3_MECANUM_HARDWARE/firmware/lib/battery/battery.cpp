#include <Arduino.h>
#include <sensor_msgs/msg/battery_state.h>
#include "config.h"

#ifdef USE_INA219

#include <Wire.h>
#include <INA219_WE.h>

#ifndef INA219_I2C_ADDRESS
#define INA219_I2C_ADDRESS 0x40
#endif

// ใช้ I2C Bus ตัวที่ 2 (Wire1)
INA219_WE ina219(&Wire1, INA219_I2C_ADDRESS);

// Battery Variables
float shuntVoltage_mV = 0.0f;
float busVoltage_V = 0.0f;
float loadVoltage_V = 0.0f;
float current_mA = 0.0f;
float power_mW = 0.0f;
bool overflow = false;

void initBattery()
{
    // GPIO40 = SDA, GPIO41 = SCL
    Wire1.begin(BATTERY_SDA_PIN, BATTERY_SCL_PIN, 400000);

    if (!ina219.init())
    {
        Serial.println("INA219 NOT FOUND!");
        return;
    }

    Serial.println("INA219 READY");

    ina219.setADCMode(INA219_BIT_MODE_12);
    ina219.setPGain(INA219_PG_320);
    ina219.setBusRange(INA219_BRNG_16);
    ina219.setShuntSizeInOhms(0.1);
    ina219.setCorrectionFactor(1.0f);
}

void updateBattery()
{
    busVoltage_V = ina219.getBusVoltage_V();
    shuntVoltage_mV = ina219.getShuntVoltage_mV();
    current_mA = ina219.getCurrent_mA();

    Serial.printf("Bus = %.2fV\n", busVoltage_V);
    Serial.printf("Shunt = %.2fmV\n", shuntVoltage_mV);
    Serial.printf("Current = %.2fmA\n", current_mA);
    Serial.print("Bus Voltage : ");
    Serial.println(busVoltage_V);

    Serial.print("Shunt Voltage : ");
    Serial.println(shuntVoltage_mV);

    Serial.print("Load Voltage : ");
    Serial.println(loadVoltage_V);

    Serial.print("Current : ");
    Serial.println(current_mA);

    Serial.print("Overflow : ");
    Serial.println(overflow);
}

float getBatteryPercentage(float voltage)
{
    float percentage = (voltage - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN);

    if (percentage > 1.0f)
        percentage = 1.0f;

    if (percentage < 0.0f)
        percentage = 0.0f;

    return percentage;
}

#else

void initBattery()
{
}

#endif

sensor_msgs__msg__BatteryState battery_msg_;

sensor_msgs__msg__BatteryState getBattery()
{
#ifdef USE_INA219

    updateBattery();

    battery_msg_.voltage = loadVoltage_V;
    battery_msg_.current = -(current_mA / 1000.0f);      // A
    battery_msg_.percentage = getBatteryPercentage(loadVoltage_V);
    battery_msg_.present = true;

    if (current_mA > 50.0f)
    {
        battery_msg_.power_supply_status =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_CHARGING;
    }
    else if (current_mA < -50.0f)
    {
        battery_msg_.power_supply_status =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    }
    else
    {
        battery_msg_.power_supply_status =
            sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

    battery_msg_.power_supply_health =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD;

    battery_msg_.power_supply_technology =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;

#endif

    return battery_msg_;
}