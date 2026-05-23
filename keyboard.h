#ifndef WHEELLEG_CONTROLLER_KEYBOARD_H
#define WHEELLEG_CONTROLLER_KEYBOARD_H

void Keyboard_Init(int time_step);
void Keyboard_Update(float *target_v, float *target_L0, float *target_turn);

#endif //WHEELLEG_CONTROLLER_KEYBOARD_H