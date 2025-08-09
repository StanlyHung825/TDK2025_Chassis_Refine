#include "trace.hpp"


/*-----Variable for computing the offset from black line-----*/
double weight_err = 0, weight_change = 0, weight_lastTime = 0;
double cmd_Vx = 0, cmd_Vy = 0, cmd_W = 0;

void trace_init(){
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcRead, 7);
}

double trace_straight_line(){
    weight_err = ((double)(-3*adcRead[0] - adcRead[1] + adcRead[3] + 3*adcRead[4])
    				/(adcRead[0] + adcRead[1] + adcRead[2] + adcRead[3] + adcRead[4]));
    weight_change = weight_err - weight_lastTime;
    weight_lastTime = weight_err;

    cmd_W = (weight_err * w_kp + weight_change * w_kd); // Robtion correction//    temp = (weight_err * w_kp + weight_change * w_kd); // Rotation correction
    return cmd_W;
}