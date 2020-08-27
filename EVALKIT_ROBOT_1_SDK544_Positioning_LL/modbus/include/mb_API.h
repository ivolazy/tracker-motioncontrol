/**
  ******************************************************************************
  * @file    mb_API.h
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

#ifndef __MB_API_H
#define __MB_API_H

/* Includes ------------------------------------------------------------------*/
#include "port.h"
#include "mbtask.h"
#include "mc_type.h"

/* Exported types ------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Registers ---------------------------------------------------------*/

#define MODBUS_REQUESTED_POSITION		0        // 32 bit
#define MODBUS_REQUESTED_DURATION		2
#define MODBUS_VOLTAGE				3
#define MODBUS_POSITION			4        // 32 bit
#define MODBUS_AVG_SPEED			6
#define MODBUS_SPEED				9
#define MODBUS_REQUESTED_SPEED			7
#define MODBUS_DEFAULT_CONTROL_MODE		8
#define MODBUS_TORQUE_REF			10
#define MODBUS_DEFAULT_TORQUE_REF		11
#define MODBUS_DEFAULT_ID_REF			12
#define MODBUS_CURRENT_AMPLITUDE		13
#define MODBUS_VOLTAGE_AMPLITUDE		14
#define MODBUS_FOC_IA                          15
#define MODBUS_FOC_IB                         16
#define MODBUS_FOC_ALPHA                       17
#define MODBUS_FOC_BETA                        18
#define MODBUS_FOC_IQ                          19
#define MODBUS_FOC_ID                          20
#define MODBUS_FOC_IQREF                       21
#define MODBUS_FOC_IDREF                       22
#define MODBUS_FOC_VIQ                         23
#define MODBUS_FOC_VID                         24
#define MODBUS_FOC_VALPHA                     25
#define MODBUS_FOC_VBETA                       26
#define MODBUS_FOC_EL_ANGLE_DPP                27
#define MODBUS_FOC_TEREF                       28
#define MODBUS_POSITION_REF                    30		// 32 bit

/* Exported functions ---------------------------------------------------------*/

void setMovementCompleted(uint8_t status);
void setZeroReached(uint8_t zero);

int32_t  getTargetPos_Deg();
int16_t  getTargetSpeed();
float getMovementDuration_s();
BOOL getNewCommand();
STC_Modality_t getControlMode();
void clearCommand();
void setMotorDetails(int16_t voltage, 
                      int32_t position,
                      int32_t positionRef,
                      int16_t speed,
                     int16_t avgSpeed,
                     int16_t torqueRef, 
                     qd_t iqdref,
                     int16_t currentAmplitude, 
                     int16_t voltageAmplitudes,
                     ab_t focIab, 
                    alphabeta_t focAlphabeta,
                    qd_t focIqd,
                    qd_t focIqdref,
                    qd_t focVIqd,
                    alphabeta_t focVAlphabeta,
                    int16_t focElAngledpp,
                    int16_t focTeref);
    
#endif /* __MB_API_H */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
