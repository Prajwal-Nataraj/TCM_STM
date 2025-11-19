/**
 *  @file MotorParams.h
 *  @brief Set/Modify Motor Parameters
 *  @author Prajwal Nataraj
 *
 **/

#ifndef MOTORPARAMS_H
#define MOTORPARAMS_H

#include "MotorCmds.h"

#define PULSE_COUNT	TIM2->CNT

#define ONGOING		1
#define COMPLETED	0
#define DIR_UP		1
#define DIR_DOWN	0
#define ACCEL		1
#define DECEL		0

typedef struct
{
	float distance;
	float newSpeedMMPM;
	float currentSpeedMMPM;
	float newSpeedRPM;
	float currentSpeedRPM;
	bool direction;
	float accel;
	float decel;

	bool customGains;
	bool defGains;

	int16_t spdKP;
	int16_t spdKI;
	int16_t trqKP;
	int16_t trqKI;
	int16_t flxKP;
	int16_t flxKI;

	float currFactor;
	bool drvToDist;
}MotorParams;

/* Initialize/Reset Motor Parameters */
bool Motor_Init(void);
/* Set the Current Factor */
void SetCurrentFactor(float);
/* Get the Current Factor */
float GetCurrentFactor(void);
/* Set Vertical Distance in mm */
bool Motor_SetDistance(float);
/* Get Vertical Distance in mm */
float Motor_GetDistance(void);
/* Set Vertical Speed in mm/min */
bool Motor_SetSpeed(float);
/* Get Vertical Speed in mm/min */
float Motor_GetSpeed(void);
/* Set Acceleration in ms */
bool Motor_SetAccel(float);
/* Get Acceleration in ms */
float Motor_GetAccel(void);
/* Set Deceleration in rpm/s */
bool Motor_SetDecel(float);
/* Get Deceleration in rpm/s */
float Motor_GetDecel(void);
/* Set Vertical Direction */
bool Motor_SetDirection(bool);
/* Get Vertical Direction */
bool Motor_GetDirection(void);
/* Calculate delta counts from zero position */
void CalcZeroDelta(void);
/* Reset the PI Gains to default */
bool Motor_ResetPIGains(void);
/* Set Speed Kp */
bool Motor_SetSpdKp(int16_t);
/* Get Speed Kp */
int16_t Motor_GetSpdKp(void);
/* Set Speed Ki */
bool Motor_SetSpdKi(int16_t);
/* Get Speed Ki */
int16_t Motor_GetSpdKi(void);
/* Set Torque Kp */
bool Motor_SetTrqKp(int16_t);
/* Get Torque Kp */
int16_t Motor_GetTrqKp(void);
/* Set Torque Ki */
bool Motor_SetTrqKi(int16_t);
/* Get Torque Ki */
int16_t Motor_GetTrqKi(void);
/* Set Drive to distance */
bool Motor_SetDrvToDist(bool);
/* Get Drive to distance */
bool Motor_GetDrvToDist(void);
/* Reset Motor Parameters */
bool Motor_ResetParams(void);
/* Enable Bridge */
bool Motor_EnBridge(void);
/* Disable Bridge */
bool Motor_DisBridge(void);
/* Start Vertical Movement with Acceleration */
bool Motor_Start(void);
/* Stop Vertical Movement with Deceleration */
bool Motor_Stop(void);
/* Critical Stop (max Deceleration) */
bool Motor_CriticalStop(void);
/* Send Data to COM Port */
bool sendToPort(UART_HandleTypeDef *, float);
/* Alert a timeout */
bool IsTimedOut(uint32_t *, uint32_t);
/* Stop the Motor when Target Distance is reached */
bool Motor_StopAtTarget(void);
/* Set Zero Position */
bool Motor_SetZeroPos(void);
/* Return Motor back to Zero Pos */
bool Motor_RTZ(void);
/* Stop Motor when Zero Position is reached */
bool Motor_CheckRTZ(void);
/* Checks for Motor Stall Condition */
void Motor_StallCheck(void);

#endif

/********************************* END OF FILE ********************************/
