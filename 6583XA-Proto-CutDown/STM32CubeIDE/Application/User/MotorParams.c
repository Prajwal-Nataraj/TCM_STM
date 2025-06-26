/**
 *  @file MotorParams.c
 *  @brief Set/Modify Motor Parameters
 *  @author Prajwal Nataraj
 *
 **/

#include "math.h"
#include "MotorParams.h"

#define GEAR_RATIO			6.0f
#define MM_PER_THREAD		5.08f
#define SPEED_ADJ_FACT		0.998013f

extern UART_HandleTypeDef huart_MD;

static uint16_t rampTime;

MotorParams Motor;

/* Initialize/Reset Motor Parameters */
bool Motor_Init(void)
{
	return Motor_ResetParams();
}

/* Set the Current Factor */
void SetCurrentFactor(float currFact)
{
	Motor.currFactor = currFact;
}
/* Get the Current Factor */
float GetCurrentFactor(void)
{
	return Motor.currFactor;
}

/* Set Vertical Distance in mm */
bool Motor_SetDistance(float distance)
{
	Motor.distance = distance;
	return true;
}
/* Get Vertical Distance in mm */
float Motor_GetDistance(void)
{
	return Motor.distance;
}

/* Set Vertical Speed in mm/min */
bool Motor_SetSpeed(float speed)
{
	Motor.newSpeedMMPM = speed;
	return true;
}
/* Get Vertical Speed in mm/min */
float Motor_GetSpeed(void)
{
	return Motor.currentSpeedMMPM;
}

/* Set Acceleration in rpm/s */
bool Motor_SetAccel(float accel)
{
	Motor.accel = accel;
	return true;
}
/* Get Acceleration in rpm/s */
float Motor_GetAccel(void)
{
	return Motor.accel;
}

/* Set Deceleration in rpm/s */
bool Motor_SetDecel(float decel)
{
	Motor.decel = decel;
	return true;
}
/* Get Deceleration in rpm/s */
float Motor_GetDecel(void)
{
	return Motor.decel;
}

/* Set Vertical Direction */
bool Motor_SetDirection(bool dir)
{
	/* 1: UP
	 * 0: DOWN
	 */
	Motor.direction = dir;
	return true;
}
/* Get Vertical Direction */
bool Motor_GetDirection(void)
{
	return Motor.direction;
}

/* Reset Motor Parameters*/
bool Motor_ResetParams(void)
{
	rampTime = 100;

	Motor.distance = 0;
	Motor.newSpeedMMPM = 0;
	Motor.currentSpeedMMPM = 0;
	Motor.newSpeedRPM = 0;
	Motor.currentSpeedRPM = 0;
	Motor.direction = 0;

	Motor_SetAccel(1181.10236);									// 100mm/min / 100ms or 1mm/min / 1ms
	Motor_SetDecel(1181.10236);									// 100mm/min / 100ms or 1mm/min / 1ms

	if(!Motor_DisBridge())
		return false;

	return true;
}

/* Set PI Gains of Speed, Torque and Flux */
static void SetPIGains(float speed)
{
	if(speed < 75.5)
	{
		Motor.currFactor = 0.8f;
		Motor.spdKP = 25000/6;
		Motor.spdKI = 3350/6;
		Motor.trqKP = 8192;
		Motor.trqKI = 2048;
		Motor.flxKP = 8192;
		Motor.flxKI = 2048;
	}
	else if((speed > 75.5) && (speed < 120.5))
	{
		Motor.currFactor = 0.9f;
		Motor.spdKP = 23000/6;
		Motor.spdKI = 3350/6;
		Motor.trqKP = 8192;
		Motor.trqKI = 2100;
		Motor.flxKP = 8192;
		Motor.flxKI = 2100;
	}
	else if((speed > 120.5) && (speed < 230.5))
	{
		Motor.currFactor = 0.9f;
		Motor.spdKP = 23000/6;
		Motor.spdKI = 3660/6;
		Motor.trqKP = 8350;
		Motor.trqKI = 2100;
		Motor.flxKP = 8350;
		Motor.flxKI = 2100;
	}
	else
	{
		Motor.currFactor = 1.0f;
		Motor.spdKP = 28800/6;
		Motor.spdKI = 3360/6;
		Motor.trqKP = 8192;
		Motor.trqKI = 2048;
		Motor.flxKP = 8192;
		Motor.flxKI = 2048;
	}

	PID_SetKP(&PIDIqHandle_M1, (int16_t)Motor.trqKP);
	PID_SetKI(&PIDIqHandle_M1, (int16_t)Motor.trqKI);

	PID_SetKP(&PIDIdHandle_M1, (int16_t)Motor.flxKP);
	PID_SetKI(&PIDIdHandle_M1, (int16_t)Motor.flxKI);

	PID_SetKP(&PIDSpeedHandle_M1, (int16_t)Motor.spdKP);
	PID_SetKI(&PIDSpeedHandle_M1, (int16_t)Motor.spdKI);
}

/* Conversion from mm/min to rpm */
float mmpm_to_rpm(float mmpm)
{
	float rpm = 0;
	rpm = (mmpm * (Motor.direction ? 1 :-1))	\
		* (float)(GEAR_RATIO / MM_PER_THREAD);

	return rpm;
}

/* Calculate Ramp time in ms */
uint16_t Motor_CalcRampTimeMs(bool ad, float targetSpeed)
{
	/* v = u + at;
	 * t = (v - u) / a */

	float rTime = 0;
	rTime = (targetSpeed - Motor.currentSpeedRPM) / ( (ACCEL == ad) ? Motor.accel : Motor.decel);

	return (uint16_t)(abs(rTime * 1000));	// convert to ms
}

/* Enable Bridge */
bool Motor_EnBridge(void)
{
	MCI_StartMotor(pMCI[M1]);
	return true;
}

/* Disable Bridge */
bool Motor_DisBridge(void)
{
	MCI_StopMotor(pMCI[M1]);
	if((Motor.currentSpeedMMPM > 0) || (Motor.currentSpeedRPM > 0))
	{
		MCI_ExecSpeedRamp(pMCI[M1], 0, 50);
		Motor.currentSpeedMMPM = 0;
		Motor.currentSpeedRPM = 0;
	}

	return true;
}

/* Start Vertical Movement with Acceleration */
bool Motor_Start(void)
{
	float adjSpeed = 0;

	Motor.newSpeedRPM = mmpm_to_rpm(Motor.newSpeedMMPM);						// mm/min in UP or DOWN to +RPM or -RPM
	adjSpeed = SPEED_ADJ_FACT * Motor.newSpeedRPM;								// Adjusted Speed

	rampTime = Motor_CalcRampTimeMs(ACCEL, Motor.newSpeedRPM);
	rampTime = (rampTime < 50) ? 50 : rampTime;									// always keep rampTime > 0

	SetPIGains(Motor.newSpeedMMPM);
	MCI_ExecSpeedRamp(pMCI[M1], adjSpeed, rampTime);

	Motor.currentSpeedMMPM = Motor.newSpeedMMPM;
	Motor.currentSpeedRPM = Motor.newSpeedRPM;

	return true;
}

/* Stop Vertical Movement with Deceleration */
bool Motor_Stop(void)
{
	rampTime = Motor_CalcRampTimeMs(DECEL, 0);
	if(Motor.currentSpeedMMPM <= 200)
		rampTime = (rampTime < 50) ? 50 : rampTime;
	else
	{
		uint16_t rampTimeLimit = 0;
		rampTimeLimit = (uint16_t)(Motor.currentSpeedMMPM / 4);
		rampTime = (rampTime < rampTimeLimit) ? rampTimeLimit : rampTime;
	}

	MCI_ExecSpeedRamp(pMCI[M1], 0, rampTime);

	Motor.currentSpeedMMPM = 0;
	Motor.currentSpeedRPM = 0;

	return true;
}

/* Critical Stop (max Deceleration) */
bool Motor_CriticalStop(void)
{
	if(Motor.currentSpeedMMPM <= 200)
		MCI_ExecSpeedRamp(pMCI[M1], 0, 50);

	else
		MCI_ExecSpeedRamp(pMCI[M1], 0, (uint16_t)(Motor.currentSpeedMMPM / 4));

	Motor.currentSpeedMMPM = 0;
	Motor.currentSpeedRPM = 0;

	return true;
}

bool sendToPort(UART_HandleTypeDef *phuart_MD, float sendData)
{
	char sendBuf[15] = {0};
	HAL_StatusTypeDef retVal;

	sprintf(sendBuf, "%.2f\r\n", sendData);
	retVal = HAL_UART_Transmit(phuart_MD, (uint8_t *)sendBuf, 15, 100);

	if(HAL_OK == retVal)
		return true;

	return false;
}

bool IsTimedOut(uint32_t *prevTime, uint32_t timeOut)
{
	if(HAL_GetTick() - *prevTime > timeOut)
	{
		*prevTime = HAL_GetTick();
		return true;
	}
	return false;
}

/* Stop the Motor when Target Distance is reached */
bool Motor_StopAtTarget(void)
{
	return true;
}

/* Set Zero Position */
bool Motor_SetZeroPos(void)
{
	return true;
}

/* Return Motor back to Zero Pos */
bool Motor_RTZ(void)
{
	return true;
}

/* Stop Motor when Zero Position is reached */
bool Motor_CheckRTZ(void)
{
	return true;
}

/********************************* END OF FILE ********************************/
