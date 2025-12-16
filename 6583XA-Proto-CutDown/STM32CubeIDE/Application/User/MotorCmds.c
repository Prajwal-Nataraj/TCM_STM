/**
 *  @file MotorCmds.c
 *  @brief Motor Controller Commands
 *  @author Prajwal Nataraj
 *
 **/

/* Private includes ----------------------------------------------------------*/
#include "MotorCmds.h"

/* External Variable/Handle --------------------------------------------------*/
/* UART Handle */
extern UART_HandleTypeDef huart_MD;

const char *Error_msg[13] =
{
	"RET_OK\n",     	// Success
	"RET_NOK\n",        // General failure
	"RET_HW_NOK\n",     // Bad hardware / not responding
	"RET_ARGS_NOK\n",   // Improper arguments
	"RET_ENV_NOK\n",    // Improper conditions
	"RET_TIMEDOUT\n",   // Timed out
	"RET_NO_IMPL\n",    // Not implemented
	"RET_BUF_FULL\n",   // Buffer is full
	"RET_BUF_EMPTY\n",  // Buffer is empty
	"RET_PROCESSING\n", // Processing
	"RET_CRC_NOK\n",	// CRC failed
	"RET_COM_NOK\n",	// Communication failed
	"RET_DEVADDR_NOK\n" // Invalid Device Address
};

uint8_t checkCRC(uint8_t *Buf, uint8_t len)
{
    uint8_t size = 0;
    uint8_t dividend = 0;
    uint8_t nextDiv = 0;

    size = 4 + len;
    dividend = Buf[0];

    for(uint8_t i = 1; i < size; i++)
    {
        nextDiv = Buf[i];
        for(uint8_t j = 0; j < 8; j++)
        {
            dividend = (dividend & 0x80) ? (dividend ^ CRC_POL) : (dividend ^ 0);

            if((i == size - 1) && (j == 7))
            	break;

            dividend <<= 1;

            if(nextDiv & 0x80)
                dividend |= 0x01;

            nextDiv <<= 1;
        }
    }

    return dividend << 1;
}

uint8_t Check_DevAddr(uint8_t devAddr)
{
	if(DEVADDR == devAddr)
		return RET_OK;
	return RET_DEVADDR_NOK;
}

static inline uint8_t GetArgUINT8(uint8_t *Buf)
{
    return *Buf;
}

static inline float32_t GetArgFLT32(uint8_t *Buf)
{
    float32_t arg;
    memcpy((void*)&arg, (void*)Buf, sizeof(float32_t));
    return arg;
}

static inline void SetValFLT32(float32_t Val, uint8_t *Buf)
{
    memcpy((void*)Buf, (void*)&Val, sizeof(float32_t));
}

static uint8_t GetAddr(void)
{
	return DEVADDR;
}

static void ACK(uint8_t FuncCode, uint8_t *RspBuf, uint32_t *RspLen)
{
    RspBuf[0] = GetAddr();
    RspBuf[1] = FuncCode;
    RspBuf[2] = 0x01;
    RspBuf[3] = 0x00;
    *RspLen = 4;
}

static void NACK(uint8_t FuncCode, uint8_t Exception, uint8_t *RspBuf, uint32_t *RspLen)
{
    RspBuf[0] = GetAddr();
    RspBuf[1] = FuncCode;
    RspBuf[2] = 0x02;
    RspBuf[3] = CMD_EXC_CMDS;
    RspBuf[4] = Exception;
    *RspLen = 5;
}

/* Fill response */
static void RESP(uint8_t FuncCode, uint8_t *Data, uint8_t DataLen, uint8_t *RspBuf, uint32_t *RspLen)
{
    uint8_t *pData = Data;
    RspBuf[0] = GetAddr();
    RspBuf[1] = FuncCode;
    RspBuf[2] = DataLen;
    for(uint8_t i = 0; i < DataLen; i++)
        RspBuf[3 + i] = *pData++;
    *RspLen = (3 + DataLen);
}



/* Get MotorDrive Application Protocol version */
static void CmdProc_AppVer(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
    uint8_t data[2];
    data[0] = APP_PROTOCOL_VER_MAJOR;
    data[1] = APP_PROTOCOL_VER_MINOR;
    RESP(CMDBYTE_FUNCCODE, data, sizeof(data), RspBuf, RspLen);
    return;
}

void CmdProc_Init(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_Init())
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_Extension(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	uint8_t *pCmdBuf = &CMDBYTE_DATA0;
	uint8_t argGS = GetArgUINT8(pCmdBuf);

	float32_t extension = 0;

	if(argGS == CMD_GET)
	{
		extension = Motor_GetExtension();

		uint8_t data[4] = {0};
		SetValFLT32(extension, data);

		RESP(CMDBYTE_FUNCCODE, (uint8_t*)data, sizeof(data), RspBuf, RspLen);
		return;
	}

	if(argGS == CMD_SET)
	{
		pCmdBuf += 1;
		extension = GetArgFLT32(pCmdBuf);

		if(!Motor_SetExtension(extension))
			NACK(CMDBYTE_FUNCCODE, CMD_RET_WRONGARGS, RspBuf, RspLen);
		else
			ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
		return;
	}
}

void CmdProc_Speed(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	uint8_t *pCmdBuf = &CMDBYTE_DATA0;
	uint8_t argGS = GetArgUINT8(pCmdBuf);

	float32_t speed = 0;

	if(argGS == CMD_GET)
	{
		speed = Motor_GetSpeed();

		uint8_t data[4] = {0};
		SetValFLT32(speed, data);

		RESP(CMDBYTE_FUNCCODE, (uint8_t*)data, sizeof(data), RspBuf, RspLen);
		return;
	}

	if(argGS == CMD_SET)
	{
		pCmdBuf += 1;
		speed = GetArgFLT32(pCmdBuf);

		if(!Motor_SetSpeed(speed))
			NACK(CMDBYTE_FUNCCODE, CMD_RET_WRONGARGS, RspBuf, RspLen);
		else
			ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
		return;
	}
}

void CmdProc_Accel(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	uint8_t *pCmdBuf = &CMDBYTE_DATA0;
	uint8_t argGS = GetArgUINT8(pCmdBuf);

	float32_t accel = 0;

	if(argGS == CMD_GET)
	{
		accel = (float32_t)Motor_GetAccel();

		uint8_t data[4] = {0};
		SetValFLT32(accel, data);

		RESP(CMDBYTE_FUNCCODE, (uint8_t*)data, sizeof(data), RspBuf, RspLen);
		return;
	}

	if(argGS == CMD_SET)
	{
		pCmdBuf += 1;
		accel = GetArgFLT32(pCmdBuf);

		if(!Motor_SetAccel(accel))
			NACK(CMDBYTE_FUNCCODE, CMD_RET_WRONGARGS, RspBuf, RspLen);
		else
			ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
		return;
	}
}

void CmdProc_Decel(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	uint8_t *pCmdBuf = &CMDBYTE_DATA0;
	uint8_t argGS = GetArgUINT8(pCmdBuf);

	float32_t decel = 0;

	if(argGS == CMD_GET)
	{
		decel = (float32_t)Motor_GetDecel();

		uint8_t data[4] = {0};
		SetValFLT32(decel, data);

		RESP(CMDBYTE_FUNCCODE, (uint8_t*)data, sizeof(data), RspBuf, RspLen);
		return;
	}

	if(argGS == CMD_SET)
	{
		pCmdBuf += 1;
		decel = GetArgFLT32(pCmdBuf);

		if(!Motor_SetDecel(decel))
			NACK(CMDBYTE_FUNCCODE, CMD_RET_WRONGARGS, RspBuf, RspLen);
		else
			ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
		return;
	}
}

void CmdProc_Direction(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	uint8_t *pCmdBuf = &CMDBYTE_DATA0;
	uint8_t argGS = GetArgUINT8(pCmdBuf);

	uint8_t dir = 0;

	if(argGS == CMD_GET)
	{
		dir = Motor_GetDirection();

		RESP(CMDBYTE_FUNCCODE, (uint8_t*)&dir, sizeof(dir), RspBuf, RspLen);
		return;
	}

	if(argGS == CMD_SET)
	{
		pCmdBuf += 1;
		dir = GetArgUINT8(pCmdBuf);

		if(!Motor_SetDirection(dir))
			NACK(CMDBYTE_FUNCCODE, CMD_RET_WRONGARGS, RspBuf, RspLen);
		else
			ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
		return;
	}
}

void CmdProc_ResetPIGains(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_ResetPIGains())
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_SpdKp(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	uint8_t *pCmdBuf = &CMDBYTE_DATA0;
	uint8_t argGS = GetArgUINT8(pCmdBuf);

	float32_t spdKp = 0;

	if(argGS == CMD_GET)
	{
		spdKp = (float32_t)Motor_GetSpdKp();

		uint8_t data[4] = {0};
		SetValFLT32(spdKp, data);

		RESP(CMDBYTE_FUNCCODE, (uint8_t*)data, sizeof(data), RspBuf, RspLen);
		return;
	}

	if(argGS == CMD_SET)
	{
		pCmdBuf += 1;
		spdKp = GetArgFLT32(pCmdBuf);

		if(!Motor_SetSpdKp((int16_t)spdKp))
			NACK(CMDBYTE_FUNCCODE, CMD_RET_WRONGARGS, RspBuf, RspLen);
		else
			ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
		return;
	}
}

void CmdProc_SpdKi(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	uint8_t *pCmdBuf = &CMDBYTE_DATA0;
	uint8_t argGS = GetArgUINT8(pCmdBuf);

	float32_t spdKi = 0;

	if(argGS == CMD_GET)
	{
		spdKi = (float32_t)Motor_GetSpdKi();

		uint8_t data[4] = {0};
		SetValFLT32(spdKi, data);

		RESP(CMDBYTE_FUNCCODE, (uint8_t*)data, sizeof(data), RspBuf, RspLen);
		return;
	}

	if(argGS == CMD_SET)
	{
		pCmdBuf += 1;
		spdKi = GetArgFLT32(pCmdBuf);

		if(!Motor_SetSpdKi((int16_t)spdKi))
			NACK(CMDBYTE_FUNCCODE, CMD_RET_WRONGARGS, RspBuf, RspLen);
		else
			ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
		return;
	}
}

void CmdProc_TrqKp(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	uint8_t *pCmdBuf = &CMDBYTE_DATA0;
	uint8_t argGS = GetArgUINT8(pCmdBuf);

	float32_t trqKp = 0;

	if(argGS == CMD_GET)
	{
		trqKp = (float32_t)Motor_GetTrqKp();

		uint8_t data[4] = {0};
		SetValFLT32(trqKp, data);

		RESP(CMDBYTE_FUNCCODE, (uint8_t*)data, sizeof(data), RspBuf, RspLen);
		return;
	}

	if(argGS == CMD_SET)
	{
		pCmdBuf += 1;
		trqKp = GetArgFLT32(pCmdBuf);

		if(!Motor_SetTrqKp((int16_t)trqKp))
			NACK(CMDBYTE_FUNCCODE, CMD_RET_WRONGARGS, RspBuf, RspLen);
		else
			ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
		return;
	}
}

void CmdProc_TrqKi(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	uint8_t *pCmdBuf = &CMDBYTE_DATA0;
	uint8_t argGS = GetArgUINT8(pCmdBuf);

	float32_t trqKi = 0;

	if(argGS == CMD_GET)
	{
		trqKi = (float32_t)Motor_GetTrqKi();

		uint8_t data[4] = {0};
		SetValFLT32(trqKi, data);

		RESP(CMDBYTE_FUNCCODE, (uint8_t*)data, sizeof(data), RspBuf, RspLen);
		return;
	}

	if(argGS == CMD_SET)
	{
		pCmdBuf += 1;
		trqKi = GetArgFLT32(pCmdBuf);

		if(!Motor_SetTrqKi((int16_t)trqKi))
			NACK(CMDBYTE_FUNCCODE, CMD_RET_WRONGARGS, RspBuf, RspLen);
		else
			ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
		return;
	}
}

void CmdProc_EnDrvToExt(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_SetDrvToExt(true))
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_DisDrvToExt(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_SetDrvToExt(false))
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_Reserved(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(0)
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_NOIMPL, RspBuf, RspLen);
}

void CmdProc_ResetParams(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_ResetParams())
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_EnBridge(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_EnBridge())
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_DisBridge(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_DisBridge())
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_Start(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_Start())
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_Stop(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_Stop())
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_CriticalStop(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_CriticalStop())
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_SetZero(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_SetZeroPos())
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_RTZ(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(Motor_RTZ())
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

void CmdProc_Voltage(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	uint8_t *pCmdBuf = &CMDBYTE_DATA0;
	uint8_t argGS = GetArgUINT8(pCmdBuf);

	float32_t volt = 0;
	float32_t convFact = 0;
	uint16_t v16 = 0;

	if(argGS == CMD_GET)
	{
		v16 = VBS_GetBusVoltage_d(&BusVoltageSensor_M1._Super);
		convFact = ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR;
		volt = ((float32_t)v16) * convFact / 65536.0;

		uint8_t data[4] = {0};
		SetValFLT32(volt, data);

		RESP(CMDBYTE_FUNCCODE, (uint8_t*)data, sizeof(data), RspBuf, RspLen);
		return;
	}

	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_WRONGARGS, RspBuf, RspLen);
}

void CmdProc_FaultAck(uint8_t *CmdBuf, uint32_t CmdLen, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(MCI_FaultAcknowledged(pMCI[M1]))
		ACK(CMDBYTE_FUNCCODE, RspBuf, RspLen);
	else
		NACK(CMDBYTE_FUNCCODE, CMD_RET_GENERROR, RspBuf, RspLen);
}

static const CmdHandler_t CmdTable[] =
{
		{ 	CMD_INIT		, 		CmdProc_Init 		},
		{ 	CMD_EXT			, 		CmdProc_Extension 	},
		{ 	CMD_SPEED		, 		CmdProc_Speed 		},
		{ 	CMD_DIR			, 		CmdProc_Direction 	},
		{ 	CMD_RESETPRM	, 		CmdProc_ResetParams },
		{ 	CMD_START		, 		CmdProc_Start 		},
		{ 	CMD_STOP		, 		CmdProc_Stop		},
		{ 	CMD_CRITSTOP	, 		CmdProc_CriticalStop},
		{ 	CMD_SETZERO		, 		CmdProc_SetZero		},
		{ 	CMD_RTZ			, 		CmdProc_RTZ 		},
		{ 	CMD_ACCEL		, 		CmdProc_Accel		},
		{ 	CMD_DECEL		, 		CmdProc_Decel		},
		{ 	CMD_ENBRIDGE	, 		CmdProc_EnBridge	},
		{ 	CMD_DISBRIDGE	, 		CmdProc_DisBridge	},

		{ 	CMD_ENDRVTOEXT	, 		CmdProc_EnDrvToExt	},
		{ 	CMD_DISDRVTOEXT	, 		CmdProc_DisDrvToExt },

		{	CMD_RSTPIGAIN	,		CmdProc_ResetPIGains},
		{ 	CMD_SPDKP		, 		CmdProc_SpdKp		},
		{ 	CMD_SPDKI		, 		CmdProc_SpdKi		},
		{ 	CMD_TRQKP		, 		CmdProc_TrqKp		},
		{ 	CMD_TRQKI		, 		CmdProc_TrqKi		},
		{ 	CMD_VOLT		, 		CmdProc_Voltage		},
		{ 	CMD_FAULT_ACK	, 		CmdProc_FaultAck	},
		{ 	CMD_APPVER		, 		CmdProc_AppVer		},
};

StdReturn_t Cmd_Process(uint8_t *CmdBuf, uint8_t *RspBuf, uint32_t *RspLen)
{
	if(0 == checkCRC(CmdBuf, CMDBYTE_DATALEN))
	{
		if(RET_OK == Check_DevAddr(CMDBYTE_DEVADDR))
		{
			if(CMDBYTE_FUNCCODE == CmdTable[CMDBYTE_FUNCCODE].FuncCode)
				CmdTable[CMDBYTE_FUNCCODE].FuncHandler(CmdBuf, CMDBYTE_DATALEN, RspBuf, RspLen);
			else
				return RET_NO_IMPL;

			if (*RspLen != 0)
			{
				RspBuf[*RspLen] = 0x00;								// Appending zeros for CRC calculation.
				RspBuf[*RspLen] = GetCRC(RspBuf, (*RspLen)-3);		// Send only the no. of data bytes for CRC calculation.
				*RspLen += 1;										// +1 for CRC byte.

				HAL_UART_Transmit(&huart_MD, RspBuf, *RspLen, UART_TIMEOUT);
			}
		}
		else
			return RET_DEVADDR_NOK;
	}
	else
		return RET_CRC_NOK;

	return RET_OK;
}
void Send_ErrorMsg(uint8_t stdRet)
{
	HAL_UART_Transmit(&huart_MD, (uint8_t *)Error_msg[stdRet], strlen(Error_msg[stdRet]), UART_TIMEOUT);
}

bool sendOut(uint8_t *CmdBuf, uint32_t size, bool crc)
{
	HAL_StatusTypeDef retVal;

	if(crc)
	{
		CmdBuf[size-1] = 0x00;
		CmdBuf[size-1] = GetCRC(CmdBuf, CMDBYTE_DATALEN);
	}
	retVal = HAL_UART_Transmit(&huart_MD, CmdBuf, size, UART_TIMEOUT);

	if(HAL_OK == retVal)
		return true;

	return false;
}

/********************************* END OF FILE ********************************/
