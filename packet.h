#pragma once
#include <cstdint>

struct Packet { float KPX;
                    float KIX;
                    float KDX;
                    float KPY;
                    float KIY;
                    float KDY;
                    float X;
                    float Y;
                    float Z;
                    bool pid_x_enabled = true;
                    bool pid_y_enabled = true;
                    int32_t nb_kp_ref;
                    int32_t nb_kp_cur;
                    int32_t nb_good;
            };