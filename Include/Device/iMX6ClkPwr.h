/** @file
*
*  Copyright (c) 2018 Microsoft Corporation. All rights reserved.
*  Copyright 2018 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#ifndef _IMX6_CLK_PWR_H_
#define _IMX6_CLK_PWR_H_

// The valid value for PLL loop divider is 27-54 so define the range of valid
// frequency for PLL5 below before divider is applied.
#define PLL5_MIN_FREQ 648000000
#define PLL5_MAX_FREQ 1296000000

#include "iMX6ClkPwr_ULL.h"

typedef enum {
  IMX_CLOCK_GATE_STATE_OFF = 0x0,
  IMX_CLOCK_GATE_STATE_ON_RUN = 0x1,
  IMX_CLOCK_GATE_STATE_ON = 0x3,
} IMX_CLOCK_GATE_STATE;

typedef struct {
  UINT32 Frequency;
  IMX_CLK Parent;
} IMX_CLOCK_INFO;

VOID
ImxClkPwrSetClockGate (
  IN IMX_CLK_GATE ClockGate,
  IN IMX_CLOCK_GATE_STATE State
  );

// Set multiple clock gates to a given state
VOID
ImxClkPwrSetClockGates (
  IN CONST IMX_CLK_GATE *ClockGateList,
  IN UINTN ClockGateCount,
  IN IMX_CLOCK_GATE_STATE State
  );

IMX_CLOCK_GATE_STATE
ImxClkPwrGetClockGate (
  IN IMX_CLK_GATE ClockGate
  );

EFI_STATUS
ImxClkPwrGetClockInfo (
  IN IMX_CLK ClockId,
  OUT IMX_CLOCK_INFO *ClockInfo
);

EFI_STATUS
ImxClkPwrGpuEnable (
  );

EFI_STATUS
ImxClkPwrIpuDIxEnable (
  );

EFI_STATUS ImxClkPwrIpuLDBxEnable (
  );

EFI_STATUS
ImxSetPll5ReferenceRate (
  UINT32 ClockRate
  );

EFI_STATUS
ImxClkPwrClkOut1Enable (
  IMX_CLK Clock,
  UINT32 Divider
  );

VOID
ImxClkPwrClkOut1Disable (
  );

EFI_STATUS
ImxClkPwrValidateClocks (
  );

CONST CHAR16
*StringFromImxClk (
  IN IMX_CLK Value
  );

#endif // _IMX6_CLK_PWR_H_
