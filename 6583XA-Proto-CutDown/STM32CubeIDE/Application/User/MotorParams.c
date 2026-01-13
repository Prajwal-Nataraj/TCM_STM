/**
 *  @file MotorParams.c
 *  @brief Set/Modify Motor Parameters
 *  @author Prajwal Nataraj
 *
 **/

/* Private includes ---------------------------------------------------------*/
#include "math.h"
#include "MotorParams.h"


/* Private define -----------------------------------------------------------*/
#define GEAR_RATIO			6.0f
#define MM_PER_THREAD		5.08f
#define SPEED_ADJ_FACT		1.001f

/* External Variable/Handle -------------------------------------------------*/
/* UART Handle */
extern UART_HandleTypeDef huart_MD;

/* Private variables --------------------------------------------------------*/
/* For Accel/Decel */
static uint16_t rampTime = 0;

/* RTZ variables */
static bool prevDir = DIR_DOWN;
static int32_t zeroDeltaCnt = 0;
static bool dir_bef_rtz;			/* Direction before RTZ */
static float spd_bef_rtz;			/* Speed before RTZ */
static uint32_t rtzCounts = 0;
static bool rtzInProgress = false;
bool rtzExec = false;

/* For extension */
static uint32_t extCounts = 0;

bool stopExec = false;

MotorParams Motor;

/*****************************************************************************/

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

/* Set Vertical Extension in mm */
bool Motor_SetExtension(float extension)
{
	Motor.extension = extension;
	return true;
}
/* Get Vertical Extension in mm */
float Motor_GetExtension(void)
{
	return Motor.extension;
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

/* Calculate encoder counts based on extension in mm */
static uint32_t mm_to_counts(float ext_mm)
{
	uint32_t counts = 0;
	counts = (ext_mm / MM_PER_THREAD) * GEAR_RATIO * 5000;		/* 5000 is the counts per rotation */
	return counts;
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
    uint32_t steps = 0;
    float incDecAmt = 0;
    float wCurrentRPM = 0;
    float currentRPM = 0;
    float counts = 0;

    rpm = abs(rpm) * SPEED_ADJ_FACT;

    steps = rpm / Motor.decel * 2000;         /* (rampTime=rpm/decel) * FREQ */
    steps++;

    incDecAmt = (rpm * 65536.0) / steps;

    for(uint32_t i = 0; i < steps; i++)
    {
        wCurrentRPM = incDecAmt*(steps-i);
        currentRPM = wCurrentRPM / 65536.0;

        counts += (currentRPM / 60.0) * 0.5 * 5.0;      /* (currentRPM / 60) * 0.5ms * 5000 pulsesPERrotation */
    }

    return (uint32_t)(counts+0.5);      /* +0.5 to round-off */
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
	if(speed < 55.5)
	{
		Motor.currFactor = 0.9f;
		Motor.spdKP = 25000/6;
		Motor.spdKI = 3350/6;
		Motor.trqKP = 8192;
		Motor.trqKI = 2048;
		Motor.flxKP = 8192;
		Motor.flxKI = 2048;
	}
	else if((speed > 55.5) && (speed < 120.5))
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

/* Set Drive to extension */
bool Motor_SetDrvToExt(bool drvToExt)
{
	Motor.drvToExt = drvToExt;
	return true;
}
/* Get Drive to extension */
bool Motor_GetDrvToExt(void)
{
	return Motor.drvToExt;
}

/* Reset Motor Parameters*/
bool Motor_ResetParams(void)
{
	rampTime = 100;

	Motor.extension = 0;
	Motor.newSpeedMMPM = 0;
	Motor.currentSpeedMMPM = 0;
	Motor.newSpeedRPM = 0;
	Motor.currentSpeedRPM = 0;
	Motor.direction = 0;
	Motor.drvToExt = false;

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
	rampTime = (rampTime < 1) ? 1 : rampTime;									// always keep rampTime >= 1

	if(Motor.customGains)
		SetCustPIGains();
	else
		SetTcmPIGains(Motor.newSpeedMMPM);

	MCI_ExecSpeedRamp(pMCI[M1], adjSpeed, rampTime);

	if((Motor.currentSpeedMMPM >= 0.1) && (!rtzInProgress))
		CalcZeroDelta();

	PULSE_COUNT = 0;

	prevDir = Motor.direction;

	Motor.currentSpeedMMPM = Motor.newSpeedMMPM;
	Motor.currentSpeedRPM = Motor.newSpeedRPM;

	if((Motor.extension > 0.1) && (Motor.currentSpeedMMPM > 0.1) && Motor.drvToExt)
		extCounts = mm_to_counts(Motor.extension) - Motor_CalcDecelCounts(Motor.currentSpeedRPM);

	return true;
}

/* Stop Vertical Movement with Deceleration */
bool Motor_Stop(void)
{
	rampTime = Motor_CalcRampTimeMs(DECEL, 0);
	rampTime = (rampTime < 1) ? 1 : rampTime;		// always keep rampTime >= 1

	MCI_ExecSpeedRamp(pMCI[M1], 0, rampTime);

	stopExec = true;

	Motor.currentSpeedMMPM = 0;
	Motor.currentSpeedRPM = 0;

	return true;
}

/* Critical Stop (max Deceleration) */
bool Motor_CriticalStop(void)
{
	MCI_ExecSpeedRamp(pMCI[M1], 0, 1);				// always keep rampTime >= 1

	stopExec = true;

	Motor.currentSpeedMMPM = 0;
	Motor.currentSpeedRPM = 0;

	return true;
}

bool sendToPort(float sendData)
{
	char sendBuf[15] = {0};

	sprintf(sendBuf, "%.2f\r\n", sendData);

	if(true == sendOut((uint8_t *)sendBuf, sizeof(sendBuf), false))
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

/* Stop the Motor when Target Extension is reached */
bool Motor_StopAtTarget(void)
{
	uint8_t d2eAck[5] = {0x01, 0x0E, 0x01, 0x03, 0x00};
	if(Motor.drvToExt && (!rtzInProgress) && (Motor.currentSpeedMMPM > 0.1))
	{
		if(PULSE_COUNT >= extCounts)
		{
			Motor_Stop();
			if(true != sendOut(d2eAck, sizeof(d2eAck), true))
				return false;
		}
	}
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

	if(Motor.currentSpeedMMPM >= 0.1)
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
		/* Subtracting deceleration counts from total counts */
		rtzCounts = abs(zeroDeltaCnt) - Motor_CalcDecelCounts(1181.10236);		// 1181.10236 rpm = 1000 mmpm
	}

	return true;
}

/* Stop Motor when Zero Position is reached */
bool Motor_CheckRTZ(void)
{
	if(rtzInProgress)
	{
		if(PULSE_COUNT >= rtzCounts)
		{
			rtzExec = true;
			Motor_Stop();
			Motor.direction = dir_bef_rtz;
			Motor.newSpeedMMPM = spd_bef_rtz;
			rtzInProgress = false;
			rtzCounts = 0;
		}
	}
	return true;
}

/* Checks for Motor Stall Condition */
void Motor_StallCheck(void)
{
	if(abs(MCI_GetIqd(pMCI[M1]).q) > 20000.0)
	{
		Motor_DisBridge();
		Motor_Stop();
	}
}

/* Checks for any Falut Condition */
void Motor_FaultCheck(void)
{
	uint16_t curFault = MC_NO_FAULTS;
	uint16_t pastFault = MC_NO_FAULTS;
	uint8_t faultCmd[5] = {0x01, 0x16, 0x01, 0x00, 0x00};

	curFault = MCI_GetCurrentFaults(pMCI[M1]);
	pastFault = MCI_GetOccurredFaults(pMCI[M1]);

	if((MC_NO_FAULTS != curFault) ||
	   (MC_NO_FAULTS != pastFault))
	{
		     if	 ((MC_DURATION & curFault) || (MC_DURATION & pastFault))
			faultCmd[3] = 0x01;
		else if ((MC_OVER_VOLT & curFault) || (MC_OVER_VOLT & pastFault))
			faultCmd[3] = 0x02;
		else if((MC_UNDER_VOLT & curFault) || (MC_UNDER_VOLT & pastFault))
			faultCmd[3] = 0x03;
		else if ((MC_OVER_CURR & curFault) || (MC_OVER_CURR & pastFault))
			faultCmd[3] = 0x04;
		else if  ((MC_SW_ERROR & curFault) || (MC_SW_ERROR & pastFault))
			faultCmd[3] = 0x05;
		else if  ((MC_DP_FAULT & curFault) || (MC_DP_FAULT & pastFault))
			faultCmd[3] = 0x06;
		else
			return;

		sendOut(faultCmd, sizeof(faultCmd), true);
	}
}

/********************************* END OF FILE ********************************/
