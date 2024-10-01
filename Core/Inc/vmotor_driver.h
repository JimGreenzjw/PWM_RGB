/*
 * vmotor_driver.h
 *
 *  Created on: Sep 28, 2024
 *      Author: JimGr
 */

#ifndef INC_VMOTOR_DRIVER_H_
#define INC_VMOTOR_DRIVER_H_

#include <math.h>
#include <stdint.h>


#define     MOTOR_DIR_IN1    PAout(2)  //电机方向控制IO
#define     MOTOR_DIR_IN2    PAout(3)

void voiceBspInit();
float getMotorPos();
void setMotorPos(float pos);
float getMotorVel();
void setMotorCur(float cur);


#endif /* INC_VMOTOR_DRIVER_H_ */
