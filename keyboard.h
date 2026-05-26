#ifndef WHEELLEG_CONTROLLER_KEYBOARD_H
#define WHEELLEG_CONTROLLER_KEYBOARD_H

void Keyboard_Init(int time_step, float v, float w, float l_delta);
void Keyboard_Update(float *target_v, float *target_L0, float *target_turn, int *jump_trigger);

#endif //WHEELLEG_CONTROLLER_KEYBOARD_H