/**
  ******************************************************************************
  * @file    mb_API.c
  * @author  Motor Control Team, ST Microelectronics
  * @brief   MODBUS API
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mb_API.h"
#include "port.h"
#include "drive_parameters.h"

/* External variables --------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/

void setMovementCompleted(uint8_t status)
{
  if (status == 1) ucRegDiscBuf[0] |= 1UL << 0;
  if (status == 0) ucRegDiscBuf[0] &= ~(1UL << 0);
}

void setZeroReached(uint8_t zero)
{
  if (zero == 1) ucRegDiscBuf[0] |= 1UL << 1;
  if (zero == 0) ucRegDiscBuf[0] &= ~(1UL << 1);
}

int32_t getTargetPos_Deg()
{
  uint32_t word  =  usRegHoldingBuf[MODBUS_REQUESTED_POSITION+1];
           word |= (usRegHoldingBuf[MODBUS_REQUESTED_POSITION] << 16);
  return (int32_t)(word);
}

int16_t getTargetSpeed()
{
  return usRegHoldingBuf[MODBUS_REQUESTED_SPEED];
}

float getMovementDuration_s()
{
  return ((float)(usRegHoldingBuf[2])/1000.0f);
}

bool getNewCommand(int bit)
{
  return  ((ucRegCoilsBuf[0] & (0x01 << bit)) != 0x00);
}

STC_Modality_t getControlMode()
{
  if (usRegHoldingBuf[MODBUS_DEFAULT_CONTROL_MODE] == 1) return STC_TORQUE_MODE;
  else if (usRegHoldingBuf[MODBUS_DEFAULT_CONTROL_MODE] == 2) return STC_SPEED_MODE;
  return STC_TORQUE_MODE;
}

void clearCommand()
{
  ucRegCoilsBuf[0] &= ~(0xFF);
}

void setMotorDetails(int16_t voltage, 
                      int32_t position,
                      int32_t positionRef,
                      int16_t speed,
                     int16_t avgSpeed,
                     int16_t torqueRef, 
                     qd_t iqdref,
                     int16_t currentAmplitude, 
                     int16_t voltageAmplitude,
                     ab_t focIab, 
                    alphabeta_t focAlphabeta,
                    qd_t focIqd,
                    qd_t focIqdref,
                    qd_t focVIqd,
                    alphabeta_t focVAlphabeta,
                    int16_t focElAngledpp,
                    int16_t focTeref)
{
  usRegHoldingBuf[MODBUS_VOLTAGE] = voltage;
  usRegHoldingBuf[MODBUS_POSITION+1] = position & 0xffff;
  usRegHoldingBuf[MODBUS_POSITION] = (position >> 16) & 0xffff;
  usRegHoldingBuf[MODBUS_POSITION_REF+1] = positionRef & 0xffff;
  usRegHoldingBuf[MODBUS_POSITION_REF] = (positionRef >> 16) & 0xffff;
  usRegHoldingBuf[MODBUS_SPEED] = speed;
  usRegHoldingBuf[MODBUS_AVG_SPEED] = avgSpeed;
  usRegHoldingBuf[MODBUS_TORQUE_REF] = torqueRef;
  usRegHoldingBuf[MODBUS_DEFAULT_TORQUE_REF] = iqdref.q;	// default torque
  usRegHoldingBuf[MODBUS_DEFAULT_ID_REF] = iqdref.d;	        // default id
  usRegHoldingBuf[MODBUS_CURRENT_AMPLITUDE] = currentAmplitude;	// current amplitude
  usRegHoldingBuf[MODBUS_VOLTAGE_AMPLITUDE] = voltageAmplitude;	// voltage amplitude 
  usRegHoldingBuf[MODBUS_FOC_IA] = focIab.a;
  usRegHoldingBuf[MODBUS_FOC_IB] = focIab.b;
  usRegHoldingBuf[MODBUS_FOC_ALPHA] = focAlphabeta.alpha;
  usRegHoldingBuf[MODBUS_FOC_BETA] = focAlphabeta.beta;
  usRegHoldingBuf[MODBUS_FOC_IQ] = focIqd.q;
  usRegHoldingBuf[MODBUS_FOC_ID] = focIqd.d;
  usRegHoldingBuf[MODBUS_FOC_IQREF] = focIqdref.q;
  usRegHoldingBuf[MODBUS_FOC_IDREF] = focIqdref.d;
  usRegHoldingBuf[MODBUS_FOC_VIQ] = focVIqd.q;
  usRegHoldingBuf[MODBUS_FOC_VID] = focVIqd.d;
  usRegHoldingBuf[MODBUS_FOC_VALPHA] = focVAlphabeta.alpha;
  usRegHoldingBuf[MODBUS_FOC_VBETA] = focVAlphabeta.beta;
  usRegHoldingBuf[MODBUS_FOC_EL_ANGLE_DPP] = focElAngledpp;
  usRegHoldingBuf[MODBUS_FOC_TEREF] = focTeref;
}

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/