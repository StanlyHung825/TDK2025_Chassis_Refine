#include "motor_monitor.hpp"
#include "motor_ctrl.hpp"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

double test = 0;
double monitor_speed = 0;

MotorController Motor1(&htim1, &htim2, TIM_CHANNEL_1, GPIOA, GPIO_PIN_5, 10.0, 0.5, 0.01);

double Vgoal = 0.0;

void motor1_monitor(void) {
    Motor1.setSpeed(Vgoal);
    monitor_speed = Motor1.getSpeed();
}
