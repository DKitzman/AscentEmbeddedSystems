#ifndef MOTOR16_H
#define MOTOR16_H

#define STOP_SMOOTH 1
#define STOP_BRAKE 2

void motor_init();
void motor0_set(int16_t value,  uint8_t stopBehavior);
void motor1_set(int16_t value,  uint8_t stopBehavior);

#endif