#pragma once

#include <string>


class Object {
public:
    float height;       
    float max_width;
    float min_width;
    float weight;

    Object(float h = 0.0f, float max_w = 0.0f, float min_w = 0.0f, float w = 0.1f) : height(h), max_width(max_w), min_width(min_w), weight(w) {}
};


class Robot {
public:
    float battery_voltage;   // V
    float battery_mah;       // mAh
    float motor_power;       // W

    Robot(float v = 0.0f, float mah = 0.0f, float w = 0.0f) : battery_voltage(v), battery_mah(mah), motor_power(w) {}

    float battery_joules()    const { return battery_voltage * battery_mah * 3.6f; }
    float motor_current()     const { return motor_power / battery_voltage; }
    float autonomy_seconds()  const { return battery_joules() / motor_power; }
};