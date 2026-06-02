#pragma once

#include <string>


struct Object {
    float height;       
    float max_width;
    float min_width;
    float weight;

    Object(float h = 0.0f, float max_w = 0.0f, float min_w = 0.0f, float w = 0.1f) : height(h), max_width(max_w), min_width(min_w), weight(w) {}
};
