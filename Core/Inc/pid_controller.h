/*
 * pid_controller.h
 *
 *  Created on: Sep 27, 2024
 *      Author: JimGr
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#include <math.h>
#include <stdint.h>

#define MAX_OUT_VAL                 2000.0
#define MAX_INTEGRAL_VAL            5000.0

typedef struct
{
    float goal_val;
    float input_val;
    float out_val;
    float last_out_val;
    float integral_i;

    float Kp;
    float Ki;
    float Kd;

    /* interface */
	float (*getFeedbackVal)();

}TargetPID;

float PID_Control(TargetPID *target);
void PID_SetGoalVal(TargetPID *target, float goal_val);
void PID_SetPidPara(TargetPID *target, float kp, float ki, float kd);



#endif /* INC_PID_CONTROLLER_H_ */
