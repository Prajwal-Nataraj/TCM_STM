/**
 *  @file MotorParams.c
 *  @brief Set/Modify Motor Parameters
 *  @author Prajwal Nataraj
 *
 **/

/* Private includes ----------------------------------------------------------*/
#include "math.h"
#include "MotorParams.h"


/* Private define ------------------------------------------------------------*/
#define GEAR_RATIO			6.0f
#define MM_PER_THREAD		5.08f
#define SPEED_ADJ_FACT		0.998013f

/* External Variable/Handle --------------------------------------------------*/
/* UART Handle */
extern UART_HandleTypeDef huart_MD;

/* Private variables ---------------------------------------------------------*/
/* For Accel/Decel */
static uint16_t rampTime = 0;
/* RTZ variables */
static bool prevDir = DIR_DOWN;
static int32_t zeroDeltaCnt = 0;
bool rtzInProgress = false;
static bool dir_bef_rtz;			/* Direction before RTZ */
static float spd_bef_rtz;			/* Speed before RTZ */

bool stopExec = false;
bool rtzExec = false;

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

/* Conversion from mm/min to rpm */
static float mmpm_to_rpm(float mmpm)
{
	float rpm = 0;
	rpm = (mmpm * (Motor.direction ? 1 :-1))	\
		* (float)(GEAR_RATIO / MM_PER_THREAD);

	return rpm;
}

/* Calculate Ramp time in ms */
static uint16_t Motor_CalcRampTimeMs(bool ad, float targetSpeed)
{
	/* v = u + at;
	 * t = (v - u) / a */

	float rTime = 0;
	rTime = (targetSpeed - Motor.currentSpeedRPM) / ( (ACCEL == ad) ? Motor.accel : Motor.decel);

	return (uint16_t)(abs(rTime * 1000));	// convert to ms
}

/* Calculate deceleration counts */
static uint32_t Motor_CalcDecelCounts(float rpm)
{
	float decelCounts = 0;

	decelCounts = rpm * (1181.10236f / Motor.decel) * 42;
	return (uint32_t)(decelCounts);
}

/* Program the PI Gains */
static void SetPIGains(void)
{
	PID_SetKP(&PIDIqHandle_M1, (int16_t)Motor.trqKP);
	PID_SetKI(&PIDIqHandle_M1, (int16_t)Motor.trqKI);

	PID_SetKP(&PIDIdHandle_M1, (int16_t)Motor.flxKP);
	PID_SetKI(&PIDIdHandle_M1, (int16_t)Motor.flxKI);

	PID_SetKP(&PIDSpeedHandle_M1, (int16_t)Motor.spdKP);
	PID_SetKI(&PIDSpeedHandle_M1, (int16_t)Motor.spdKI);
}

/* Set Default PI Gains */
static void SetDefPIGains(void)
{
	Motor.currFactor = 1.0f;
	Motor.spdKP = 28800/6;
	Motor.spdKI = 3360/6;
	Motor.trqKP = 8192;
	Motor.trqKI = 2048;
	Motor.flxKP = 8192;
	Motor.flxKI = 2048;

	SetPIGains();
	Motor.defGains = true;
}

/* Set TCM PI Gains */
static void SetTcmPIGains(float speed)
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

	SetPIGains();
}

/* Set Custom PI Gains */
static void SetCustPIGains(void)
{
	SetPIGains();
}

/* Calculate Delta counts from zero position */
void CalcZeroDelta(void)
{
	if(DIR_UP == prevDir)
		zeroDeltaCnt += PULSE_COUNT;
	else
		zeroDeltaCnt -= PULSE_COUNT;
}

/* Reset the PI Gains to default */
bool Motor_ResetPIGains(void)
{
	SetDefPIGains();
	Motor.defGains = false;
	Motor.customGains = false;
	return true;
}

/* Set Speed Kp */
bool Motor_SetSpdKp(int16_t spdKp)
{
	Motor.customGains = true;
	if(!Motor.defGains)
		SetDefPIGains();

	Motor.spdKP = spdKp/6;
	return true;
}
/* Get Speed Kp */
int16_t Motor_GetSpdKp(void)
{
	return Motor.spdKP*6;
}

/* Set Speed Ki */
bool Motor_SetSpdKi(int16_t spdKi)
{
	Motor.customGains = true;
	if(!Motor.defGains)
		SetDefPIGains();

	Motor.spdKI = spdKi/6;
	return true;
}
/* Get Speed Ki */
int16_t Motor_GetSpdKi(void)
{
	return Motor.spdKI*6;
}

/* Set Torque Kp */
bool Motor_SetTrqKp(int16_t trqKp)
{
	Motor.customGains = true;
	if(!Motor.defGains)
		SetDefPIGains();

	Motor.trqKP = trqKp;
	Motor.flxKP = trqKp;
	return true;
}
/* Get Torque Kp */
int16_t Motor_GetTrqKp(void)
{
	return Motor.trqKP;
}

/* Set Torque Ki */
bool Motor_SetTrqKi(int16_t trqKi)
{
	Motor.customGains = true;
	if(!Motor.defGains)
		SetDefPIGains();

	Motor.trqKI = trqKi;
	Motor.flxKI = trqKi;
	return true;
}
/* Get Torque Ki */
int16_t Motor_GetTrqKi(void)
{
	return Motor.trqKI;
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

	Motor.defGains = false;
	Motor.customGains = false;

	Motor.currFactor = 1.0f;
	Motor.spdKP = 28800/6;
	Motor.spdKI = 3360/6;
	Motor.trqKP = 8192;
	Motor.trqKI = 2048;
	Motor.flxKP = 8192;
	Motor.flxKI = 2048;


	Motor.accel = 1181.10236;									// 100mm/min / 100ms or 1mm/min / 1ms
	Motor.decel = 1181.10236;									// 100mm/min / 100ms or 1mm/min / 1ms

	if(!Motor_DisBridge())
		return false;

	return true;
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

	if(Motor.customGains)
		SetCustPIGains();
	else
		SetTcmPIGains(Motor.newSpeedMMPM);

	MCI_ExecSpeedRamp(pMCI[M1], adjSpeed, rampTime);

	if((Motor.currentSpeedMMPM >= 1.0) && (!rtzInProgress))
		CalcZeroDelta();

	PULSE_COUNT = 0;

	prevDir = Motor.direction;

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

	stopExec = true;
//	if(Motor.currentSpeedMMPM >= 1)
//		CalcZeroDelta();

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
	PULSE_COUNT = 0;
	zeroDeltaCnt = 0;
	return true;
}

/* Return Motor back to Zero Pos */
bool Motor_RTZ(void)
{
	dir_bef_rtz = Motor.direction;
	spd_bef_rtz = Motor.newSpeedMMPM;
	bool execRTZ = false;

	if(Motor.currentSpeedMMPM >= 1)
		CalcZeroDelta();

	if(zeroDeltaCnt > 0)
	{
		Motor.direction = DIR_DOWN;
		execRTZ = true;
	}
	else if(zeroDeltaCnt < 0)
	{
		Motor.direction = DIR_UP;
		execRTZ = true;
	}
	else
	{
		execRTZ = false;
	}

	if(execRTZ)
	{
		Motor.newSpeedMMPM = 1000;
		rtzInProgress = true;
		Motor_Start();
	}

	return true;
}

/* Stop Motor when Zero Position is reached */
bool Motor_CheckRTZ(void)
{
	uint32_t rtzCounts = 0;

	if(rtzInProgress)
	{
		rtzCounts = abs(zeroDeltaCnt) - Motor_CalcDecelCounts(1181.10236);		// Subtracting deceleration counts
		if(PULSE_COUNT >= rtzCounts)//abs(zeroDeltaCnt))
		{
			rtzExec = true;
			Motor_Stop();
			Motor.direction = dir_bef_rtz;
			Motor.newSpeedMMPM = spd_bef_rtz;
			rtzInProgress = false;
		}
	}
	return true;
}

/********************************* END OF FILE ********************************/
