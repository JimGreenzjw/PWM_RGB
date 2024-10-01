/*
 * vmotor_driver.c
 *
 *  Created on: Sep 28, 2024
 *      Author: JimGr
 */

#include "tim.h"

#include "vmotor_driver.h"

//针对硬盘音圈电机驱动程序

extern float motor_pos, motor_vel;

/**
  * @brief
  * @param
  * @return
  */
float getMotorPos()
{
	return motor_pos;
}

/**
  * @brief
  * @param
  * @return
  */
float getMotorVel()
{
	return motor_vel;
}

/**
  * @brief
  * @param
  * @return
  */
void setMotorCur(float cur)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, cur);
}
