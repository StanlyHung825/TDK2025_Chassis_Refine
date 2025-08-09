#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "math.h"
#include "motor_ctrl.hpp"
#include "chassis_config.h"

/**********************************************************************/
/*******All the parameter about chassis is in "chassis_config.h"*******/
/**********************************************************************/

class Chassis {
    public:
        Chassis(MotorController motorFR, MotorController motorFL, MotorController motorBR, MotorController motorBL):
                _motorFR(motorFR), _motorFL(motorFL), _motorBR(motorBR), _motorBL(motorBL){}
        ~Chassis() = default;

        void setSpeed(double Vx_goal, double Vy_goal, double W_goal);       // Set the speed of chassis
        void getLocation();                                                 // Get the location of the chassis
        void Mecan_ForwardKinematics();                                     // Compute the forward kinematics of mecanum chassis
        void Mecan_InverseKinematics();                                     // Compute the inverse kinematics of mecanum chassis
        double x = INIT_X, y = INIT_Y, theta = INIT_THETA;
        
    private:
        MotorController _motorFR, _motorFL, _motorBR, _motorBL;
        double _V_FR_goal = 0, _V_FL_goal = 0, _V_BR_goal = 0, _V_BL_goal = 0;
        double _Vx_goal = 0, _Vy_goal = 0, _W_goal = 0;
        double _Vx_now = 0, _Vy_now = 0, _W_now = 0;
        double _chassis_factor = (CHASSIS_LENGTH + CHASSIS_WIDTH) / (PI * WHEEL_DIA);
        double _V_FR_now = 0, _V_FL_now = 0, _V_BR_now = 0, _V_BL_now = 0;
};


#ifdef __cplusplus
}
#endif

#endif // CHASSIS_HPP