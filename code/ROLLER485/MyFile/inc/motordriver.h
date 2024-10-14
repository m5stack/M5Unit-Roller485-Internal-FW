/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __MOTORDRIVER_H__
#define __MOTORDRIVER_H__
#include "stm32g4xx.h"

#include "arm_const_structs.h"

#define MDRV_MODE_OFF 0
#define MDRV_MODE_RUN 1
#define MDRV_MODE_ENC_CAL 2

extern uint16_t eangle_get;
extern float32_t angle_corrected;
extern float32_t last_angle_corrected;
extern float32_t angle_corrected;

void MotorDriverInit(void);
void MotorDriverProcess(void);
void MotorDriverInterrupt(void);

uint16_t MotorDriverGetMechanicalAngle(void);

void MotorDriverSetUq(float32_t uq_set);

void mt6816_update(void);

void MotorDriverSetAngleOffset(float32_t offset);

void MotorDriverSetMode(uint8_t val);

uint8_t IsMotorDriverEncCalBusy(void);

uint16_t GetMotorDriverEncCalOffset(void);

void MotorDriverSetCurrentAdc(int32_t phase_current);

void MotorDriverSetCurrentReal(float32_t phase_current);

float32_t MotorDriverGetPhaseCurrentAdc(void);

float32_t MotorDriverGetPhaseCurrentReal(void);

#endif
