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

extern UART_HandleTypeDef huart_MD;

static uint16_t rampTime;

MotorParams Motor;
Speed_t Speed;

/* Initialize/Reset Motor Parameters */
bool Motor_Init(void)
{
	return Motor_ResetParams();
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

/* Reset Motor Parameters to 0 */
bool Motor_ResetParams(void)
{
	if(!Motor_Stop())
		return false;

	rampTime = 100;

	Motor.distance = 0;
	Motor.targetMecAng = 0;
	Motor.targetMecAngCache = 0;
	Motor.restoreTarMecAng = DISABLE;
	Motor.newSpeedMMPM = 0;
	Motor.currentSpeedMMPM = 0;
	Motor.newSpeedRPM = 0;
	Motor.currentSpeedRPM = 0;
	Motor.direction = 0;
	Motor.zeroPosition = SPD_GetMecAngle(SpeednTorqCtrlM1.SPD);

	Motor_SetAccel(1181.10236);									// 100mm/min / 100ms or 1mm/min / 1ms
	Motor_SetDecel(1181.10236);									// 100mm/min / 100ms or 1mm/min / 1ms

	return true;
}

void Motor_ResetDriveParams(void)
{
	FOC_Init();
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

//	if(ACCEL == ad)
//		rampTime = (targetSpeed - Motor.currentSpeedRPM) / Motor.accel;
//	else
//		rampTime = (targetSpeed - Motor.currentSpeedRPM) / Motor.decel;

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
	MCI_ExecSpeedRamp(pMCI[M1], 0, 50);
	Motor.currentSpeedMMPM = 0;

	return true;
}

/* Start Vertical Movement with Acceleration */
bool Motor_Start(void)
{
//	Motor.targetMecAng += (376 * (Motor.distance * (Motor.direction ? 1 :-1)));		// 1mm = 376 MecAngle

	float adjSpeed = 0;

	Motor.newSpeedRPM = mmpm_to_rpm(Motor.newSpeedMMPM);						// mm/min in UP or DOWN to +RPM or -RPM
	adjSpeed = 0.9972 * Motor.newSpeedRPM;										// Adjusted Speed

	rampTime = Motor_CalcRampTimeMs(ACCEL, Motor.newSpeedRPM);
	rampTime = (rampTime < 1) ? 1 : rampTime;		// always keep rampTime > 0

	//	Motor.stopAtTarget = ENABLE;
	//	Speed.prev_time_send = HAL_GetTick();
	//	Speed.sendAccess = true;

	MCI_ExecSpeedRamp(pMCI[M1], adjSpeed, rampTime);

	Motor.currentSpeedMMPM = Motor.newSpeedMMPM;
	Motor.currentSpeedRPM = Motor.newSpeedRPM;

	return true;
}

/* Stop Vertical Movement with Deceleration */
bool Motor_Stop(void)
{
	rampTime = Motor_CalcRampTimeMs(DECEL, 0);

	MCI_ExecSpeedRamp(pMCI[M1], 0, rampTime);

	Motor.currentSpeedMMPM = 0;
	Motor.currentSpeedRPM = 0;

	//	Speed.sendAccess = false;
	//	sendToPort(&huart_MD, SPD_GetMecAngle(SpeednTorqCtrlM1.SPD));

	return true;
}

/* Critical Stop (max Deceleration) */
bool Motor_CriticalStop(void)
{
	MCI_ExecSpeedRamp(pMCI[M1], 0, 50);
	Motor.currentSpeedMMPM = 0;
	Motor.currentSpeedRPM = 0;

	//	Speed.sendAccess = false;
	//	sendToPort(&huart_MD, SPD_GetMecAngle(SpeednTorqCtrlM1.SPD));

	return true;
}

uint32_t Motor_GetPrevSendTick(void)
{
	return Speed.prev_time_send;
}
void Motor_SetPrevSendTick(uint32_t val)
{
	Speed.prev_time_send = val;
}

bool getSendAccess(void)
{
	return Speed.sendAccess;
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

bool IsTimedOut(uint32_t prevTime, uint32_t timeOut)
{
	if(HAL_GetTick() - prevTime > timeOut)
		return true;
	return false;
}

/* Stop the Motor when Target Distance is reached */
bool Motor_StopAtTarget(void)
{
	if((DIR_UP == Motor_GetDirection()) && (ENABLE == Motor.stopAtTarget))
	{
		if(SPD_GetMecAngle(SpeednTorqCtrlM1.SPD) >= Motor.targetMecAng)
		{
//			MC_StopMotor1();
			MCI_StopMotor(pMCI[M1]);
			Motor.stopAtTarget = DISABLE;
			if(ENABLE == Motor.restoreTarMecAng)
				Motor.targetMecAng = Motor.targetMecAngCache;
		}
	}
	else
	{
		if(ENABLE == Motor.stopAtTarget)
		{
			if(SPD_GetMecAngle(SpeednTorqCtrlM1.SPD) <= Motor.targetMecAng)
			{
//				MC_StopMotor1();
				MCI_StopMotor(pMCI[M1]);
				Motor.stopAtTarget = DISABLE;
				if(ENABLE == Motor.restoreTarMecAng)
					Motor.targetMecAng = Motor.targetMecAngCache;
			}
		}
	}

	return true;
}

/* Set Zero Position */
bool Motor_SetZeroPos(void)
{
	Motor.zeroPosition = SPD_GetMecAngle(SpeednTorqCtrlM1.SPD);
	return true;
}

/* Return Motor back to Zero Pos */
bool Motor_RTZ(void)
{
//	bool prevDir = 0;
//	float prevSpeed = 0;
//	uint16_t prevRampTime = 0;
//
//	prevDir = Motor.direction;
//	prevSpeed = Motor.speed;
//	prevRampTime = rampTime;
//
//	Motor.direction = (SPD_GetMecAngle(SpeednTorqCtrlM1.SPD) > Motor.zeroPosition) ? 0 : 1;				// Do direction for RTZ
//	Motor.speed = 1000.0;
//	rampTime = 1000;
//
//	if(Motor_Run())
//	{
//		Motor.direction = prevDir;
//		Motor.speed = prevSpeed;
//		rampTime = prevRampTime;
//	}
//
//	Motor.targetMecAngCache = Motor.targetMecAng;
//	Motor.targetMecAng = Motor.zeroPosition;
//	Motor.restoreTarMecAng = ENABLE;
////	MC_StartMotor1();
//	MCI_StartMotor(pMCI[M1]);
//	Motor.RTZstate = ONGOING;
//
	return true;
}

/* Stop Motor when Zero Position is reached */
bool Motor_CheckRTZ(void)
{
//	if(ONGOING == Motor.RTZstate)
//	{
//		if(DIR_UP == Motor_GetDirection())
//		{
//			if(SPD_GetMecAngle(SpeednTorqCtrlM1.SPD) <= Motor.zeroPosition)
//			{
//				Motor_Stop();
//				Motor_SetParams();
//				Motor.RTZstate = COMPLETED;
//			}
//		}
//		else
//		{
//			if(SPD_GetMecAngle(SpeednTorqCtrlM1.SPD) >= Motor.zeroPosition)
//			{
//				Motor_Stop();
//				Motor_SetParams();
//				Motor.RTZstate = COMPLETED;
//			}
//		}
//	}

	return true;
}

/********************************* END OF FILE ********************************/
