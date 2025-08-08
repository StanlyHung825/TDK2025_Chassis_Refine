#include "motor_ctrl.hpp"


void MotorController::init() {    
    HAL_TIM_Encoder_Start(_enc, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(_pwm, _channel);
}

void MotorController::setSpeed(double speed) {
    _targetSpeed = speed;
    ComputePID();

    if (_pidOutput > 80.0) _pidOutput = 80.0;
    if (_pidOutput < -80.0) _pidOutput = -80.0;

    HAL_GPIO_WritePin(_dirGPIO, _dirPin, _pidOutput >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    _pwmValue = (uint32_t)(fabs(_pidOutput) * PWM_ARR / 100.0);
    __HAL_TIM_SET_COMPARE(_pwm, _channel, _pwmValue);
}

double MotorController::getSpeed() {
    return _currentSpeed;
}

double MotorController::ComputePID() {
    updateSpeed();
    _error = _targetSpeed - _currentSpeed;

    if(_integral >= INTEGRAL_LIMIT) _integral = INTEGRAL_LIMIT;
    else if(_integral <= -INTEGRAL_LIMIT) _integral = -INTEGRAL_LIMIT;
    else _integral += _error * (DT / 1000.0);

    double derivative = (_error - _lastError) / (DT / 1000.0);
    
    _pidOutput = (_kp * _error) + (_ki * _integral) + (_kd * derivative);
    
    // Update last error
    _lastError = _error;
    return _pidOutput;
}

double MotorController::updateSpeed() {
    if(_last_cnt == __HAL_TIM_GET_COUNTER(_enc))
        _isCountingDown = false;
    else
        _isCountingDown = __HAL_TIM_IS_TIM_COUNTING_DOWN(_enc);

    if(_isCountingDown)
        _currentSpeed = -((_enc->Instance->ARR - __HAL_TIM_GET_COUNTER(_enc)) / ENCODER_RESOLUTION / REDUCTION_RATIO / 4) / (DT / 1000.0); // RPS
    else
        _currentSpeed = (__HAL_TIM_GET_COUNTER(_enc) / ENCODER_RESOLUTION / REDUCTION_RATIO / 4) / (DT / 1000.0); // RPS

    __HAL_TIM_SET_COUNTER(_enc, 0);
    return _currentSpeed;
}
