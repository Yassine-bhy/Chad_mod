#pragma once
#include <cstdint>

struct __attribute__((packed)) Packet {
                    float KPX;
                    float KIX;
                    float KDX;
                    float KPY;
                    float KIY;
                    float KDY;
                    float KPZ;
                    float KIZ;
                    float KDZ;
                    float X;
                    float Y;
                    float Z;
                    bool pid_x_enabled = true;
                    bool pid_y_enabled = true;
                    bool pid_z_enabled = true;
                    int32_t nb_kp_ref;
                    int32_t nb_kp_cur;
                    int32_t nb_good;
            };