/*stm32 include*/
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
// #include "motor_ctrl.hpp"
#include "timers.h"
#include "motor_monitor.hpp"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

uint16_t CNT = 0;

TimerHandle_t xTimer;
void motorTimerCallback(TimerHandle_t xTimer);

void StartDefaultTask(void *argument)
{
    xTimer = xTimerCreate("MotorTimer", pdMS_TO_TICKS(100), pdTRUE, (void *)0, motorTimerCallback);
    xTimerStart(xTimer, 0);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

    for(;;)
    {
        
        osDelay(100);
    }
}

void motorTimerCallback(TimerHandle_t xTimer)
{
    CNT = __HAL_TIM_GET_COUNTER(&htim1);
    motor1_monitor();
}