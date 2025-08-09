#include "trace.hpp"


void trace_init(){
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcRead, 7);
}

double trace_straight_line(){
    weight_err = ((float)(-3*adcRead[0] - adcRead[1] + adcRead[3] + 3*adcRead[4])
    				/(adcRead[0] + adcRead[1] + adcRead[2] + adcRead[3] + adcRead[4]));
    weight_change = weight_err - weight_lastTime;
    weight_lastTime = weight_err;

    cmd_W = (weight_err * w_kp + weight_change * w_kd); // Robtion correction//    temp = (weight_err * w_kp + weight_change * w_kd); // Rotation correction
    return cmd_W;
}