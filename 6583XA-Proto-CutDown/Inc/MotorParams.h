/**
 *  @file MotorParams.h
 *  @brief Set/Modify Motor Parameters
 *  @author Prajwal Nataraj
 *
 **/

#ifndef MOTORPARAMS_H
#define MOTORPARAMS_H

#include "MotorCmds.h"

#define ONGOING		1
#define COMPLETED	0
#define DIR_UP		1
#define DIR_DOWN	0

typedef struct
{
	uint32_t prev_time_send;
	bool sendAccess;
}Speed_t;

typedef struct
{
	float distance;
	int32_t targetMecAng;
	int32_t targetMecAngCache;
	float newSpeed;
	float currentSpeed;
	bool direction;
	uint16_t rampTime;
	float accel;
	int32_t zeroPosition;
	bool RTZstate;
	bool stopAtTarget;
	bool restoreTarMecAng;
}MotorParams;

/* Initialize/Reset Motor Parameters */
bool Motor_Init(void);
/* Set Vertical Distance in mm */
bool Motor_SetDistance(float);
/* Get Vertical Distance in mm */
float Motor_GetDistance(void);
/* Set Vertical Speed in mm/min */
bool Motor_SetSpeed(float);
/* Get Vertical Speed in mm/min */
float Motor_GetSpeed(void);
/* Set Ramp Time in ms */
bool Motor_SetRampTime(uint16_t);
/* Get Ramp Time in ms */
uint16_t Motor_GetRampTime(void);
/* Set Acceleration in ms */
bool Motor_SetAccel(float);
/* Get Acceleration in ms */
float Motor_GetAccel(void);
/* Set Vertical Direction */
bool Motor_SetDirection(bool);
/* Get Vertical Direction */
bool Motor_GetDirection(void);
/* Reset Motor Parameters to 0 */
bool Motor_ResetParams(void);
/* Resets the Motor's Drive Parameters */
void Motor_ResetDriveParams(void);
/* Conversion from mm/min to rpm */
float mmpm_to_rpm(float mmpm);
/* Calculate Acceleration time in ms */
uint16_t Motor_CalcAccelTimeMs(void);
/* Enable Bridge */
bool Motor_EnBridge(void);
/* Disable Bridge */
bool Motor_DisBridge(void);
/* Start Vertical Movement with Acceleration */
bool Motor_Start(void);
/* Stop Vertical Movement with Deceleration */
bool Motor_Stop(void);
/* Critical Stop (without Deceleration) */
bool Motor_CriticalStop(void);

///* Set the Motor Parameters */
//bool Motor_SetParams(void);

uint32_t Motor_GetPrevSendTick(void);

void Motor_SetPrevSendTick(uint32_t);

bool getSendAccess(void);
/* Send Data to COM Port */
bool sendToPort(UART_HandleTypeDef *, float);
/* Alert a timeout */
bool IsTimedOut(uint32_t, uint32_t);
/* Stop the Motor when Target Distance is reached */
bool Motor_StopAtTarget(void);
/* Set Zero Position */
bool Motor_SetZeroPos(void);
/* Return Motor back to Zero Pos */
bool Motor_RTZ(void);
/* Stop Motor when Zero Position is reached */
bool Motor_CheckRTZ(void);

#endif

/********************************* END OF FILE ********************************/
