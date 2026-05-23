#include "keyboard.h"
#include <webots/keyboard.h>
#include <stdbool.h>

void Keyboard_Init(int time_step) {
    wb_keyboard_enable(time_step);
}

void Keyboard_Update(float *target_v, float *target_L0, float *target_turn) {
    int key;
    bool w_pressed = false;
    bool s_pressed = false;
    bool a_pressed = false;
    bool d_pressed = false;

    while ((key = wb_keyboard_get_key()) > 0) {
        switch (key) {
            case 'W':
            case 'w':
                w_pressed = true;
                break;
            case 'S':
            case 's':
                s_pressed = true;
                break;
            case 'A':
            case 'a':
                a_pressed = true;
                break;
            case 'D':
            case 'd':
                d_pressed = true;
                break;
            case 'Q':
            case 'q':
                *target_L0 += 0.001f;
                if (*target_L0 > 1.2f) *target_L0 = 1.2f;
                break;
            case 'E':
            case 'e':
                *target_L0 -= 0.001f;
                if (*target_L0 < 0.5f) *target_L0 = 0.6f;
                break;
            default:
                break;
        }
    }

    if (w_pressed) {
        *target_v = 1.0f; // 前进定值速度
    } else if (s_pressed) {
        *target_v = -1.0f; // 后退定值速度
    } else {
        *target_v = 0.0f; // 松开时减速到0
    }

    float turn_speed = 0.001f;
    if (a_pressed) {
        *target_turn += turn_speed; // 左转
    } else if (d_pressed) {
        *target_turn -= turn_speed; // 右转
    }

    while (*target_turn > 3.14159f) {
        *target_turn -= 2.0f * 3.14159f;
    }
    while (*target_turn < -3.14159f) {
        *target_turn += 2.0f * 3.14159f;
    }
}
