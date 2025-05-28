/**
 *  @file MotorParams.c
 *  @brief Set/Modify Motor Parameters
 *  @author Prajwal Nataraj
 *
 **/

#include "math.h"
#include "MotorParams.h"

#define GEAR_RATIO		6.0f
#define MM_PER_THREAD	5.08f

extern UART_HandleTypeDef huart_MD;

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
	Motor.speed = speed;
	return true;
}
/* Get Vertical Speed in mm/min */
float Motor_GetSpeed(void)
{
	return Motor.speed;
}

/* Set Ramp Time in ms */
bool Motor_SetRampTime(uint16_t rtime)
{
	Motor.rampTime = rtime;
	return true;
}
/* Get Ramp Time in ms */
uint16_t Motor_GetRampTime(void)
{
	return Motor.rampTime;
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

/* Set Zero Position */
bool Motor_SetZeroPos(void)
{
	Motor.zeroPosition = SPD_GetMecAngle(SpeednTorqCtrlM1.SPD);
	return true;
}

/* Return Motor back to Zero Pos */
bool Motor_RTZ(void)
{
	bool prevDir = 0;
	float prevSpeed = 0;
	uint16_t prevRampTime = 0;

	prevDir = Motor.direction;
	prevSpeed = Motor.speed;
	prevRampTime = Motor.rampTime;

	Motor.direction = (SPD_GetMecAngle(SpeednTorqCtrlM1.SPD) > Motor.zeroPosition) ? 0 : 1;				// Do direction for RTZ
	Motor.speed = 1000.0;
	Motor.rampTime = 1000;

	if(Motor_Run())
	{
		Motor.direction = prevDir;
		Motor.speed = prevSpeed;
		Motor.rampTime = prevRampTime;
	}

	Motor.targetMecAngCache = Motor.targetMecAng;
	Motor.targetMecAng = Motor.zeroPosition;
	Motor.restoreTarMecAng = ENABLE;
	MC_StartMotor1();
	Motor.RTZstate = ONGOING;

	return true;
}

/* Reset Motor Parameters to 0 */
bool Motor_ResetParams(void)
{
	if(!Motor_Stop(false))
		return false;

	Motor.distance = 0;
	Motor.targetMecAng = 0;
	Motor.targetMecAngCache = 0;
	Motor.restoreTarMecAng = DISABLE;
	Motor.speed = 0;
	Motor.direction = 0;
	Motor.zeroPosition = SPD_GetMecAngle(SpeednTorqCtrlM1.SPD);
	Motor.rampTime = 100;				/* Default 100ms. Should not be zero. */

	return true;
}

void Motor_ResetDriveParams(void)
{
	FOC_Init();
}

/* Turn ON motor and align Axis */
bool Motor_Start(void)
{
	Motor.targetMecAng += (376 * (Motor.distance * (Motor.direction ? 1 :-1)));		// 1mm = 376 MecAngle

	MC_StartMotor1();
	Motor.stopAtTarget = ENABLE;
	Speed.prev_time_send = HAL_GetTick();
	Speed.sendAccess = true;

	return true;
}

/* Stop Vertical Movement */
bool Motor_Stop(bool clear)
{
	MC_StopMotor1();

	if(clear)
		MC_ProgramSpeedRampMotor1(0, 100);

	Speed.sendAccess = false;
	sendToPort(&huart_MD, SPD_GetMecAngle(SpeednTorqCtrlM1.SPD));

	return true;
}

/* Start the Vertical Movement (post Motor_Start()) */
float speed123 = 0;
bool Motor_Run(void)
{
//	float speed = 0;
	uint16_t rampTime = 0;

	speed123 = (float)(Motor.speed * (Motor.direction ? 1 :-1)) * (float)(6.0 / 5.08);		// RPM to mm/min
	rampTime = Motor.rampTime; //in ms

	MC_ProgramSpeedRampMotor1(speed123, rampTime);

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
			MC_StopMotor1();
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
				MC_StopMotor1();
				Motor.stopAtTarget = DISABLE;
				if(ENABLE == Motor.restoreTarMecAng)
					Motor.targetMecAng = Motor.targetMecAngCache;
			}
		}
	}

	return true;
}

/* Stop Motor when Zero Position is reached */
bool Motor_CheckRTZ(void)
{
	if(ONGOING == Motor.RTZstate)
	{
		if(DIR_UP == Motor_GetDirection())
		{
			if(SPD_GetMecAngle(SpeednTorqCtrlM1.SPD) <= Motor.zeroPosition)
			{
				Motor_Stop(false);
				Motor_Run();
				Motor.RTZstate = COMPLETED;
			}
		}
		else
		{
			if(SPD_GetMecAngle(SpeednTorqCtrlM1.SPD) >= Motor.zeroPosition)
			{
				Motor_Stop(false);
				Motor_Run();
				Motor.RTZstate = COMPLETED;
			}
		}
	}

	return true;
}

/********************************* END OF FILE ********************************/
