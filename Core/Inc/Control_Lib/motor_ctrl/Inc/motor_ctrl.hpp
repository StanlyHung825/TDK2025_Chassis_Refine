#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f446xx.h"

#include "math.h"
#include "motor_config.h"

class MotorController {
public:
    MotorController(TIM_HandleTypeDef* enc, TIM_HandleTypeDef* pwm, uint32_t channel, GPIO_TypeDef* dirGPIO, uint16_t dirPin, double kp, double ki, double kd):
        _enc(enc), _pwm(pwm), _channel(channel), _dirGPIO(dirGPIO), _dirPin(dirPin), _kp(kp), _ki(ki), _kd(kd){}
    ~MotorController() = default;
    
    void init();                        // Initialize the basic parameter
    void setSpeed(double speed);        // Set motor speed in RPS
    double getSpeed();                  // Return the current speed (Need to enable setSpeed())

private:
    TIM_HandleTypeDef* _enc;
    TIM_HandleTypeDef* _pwm;
    uint32_t _channel;
    GPIO_TypeDef* _dirGPIO;
    uint16_t _dirPin;

    double _kp;
    double _ki;
    double _kd;

    bool _isCountingDown = false;
    uint32_t _last_cnt = 0;
    uint32_t _pwmValue = 0;
    double _pidOutput = 0.0;
    double _currentSpeed = 0;
    double _targetSpeed = 0;
    double _error = 0;
    double _lastError = 0;
    double _integral = 0;
    double _dt = 100;

    double ComputePID();                // PID controller
    double updateSpeed();               // Update the motor current speed
};

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CTRL_H */