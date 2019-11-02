/** @file
*
*  Provides definitions for the GPT (General Purpose Timer) module
*  that are common to Freescale SoCs.
*
*  Copyright (c) 2018 Microsoft Corporation. All rights reserved.
*  Copyright (c) 2004-2010, Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifndef __COMMON_GPT_H
#define __COMMON_GPT_H

#include <Uefi.h>

typedef struct {
  UINT32 CR;
  UINT32 PR;
  UINT32 SR;
  UINT32 IR;
  UINT32 OCR1;
  UINT32 OCR2;
  UINT32 OCR3;
  UINT32 ICR1;
  UINT32 ICR2;
  UINT32 CNT;
} CSP_GPT_REGS, *PCSP_GPT_REGS;

#define GPT_CR_OFFSET          0x0000
#define GPT_PR_OFFSET          0x0004
#define GPT_SR_OFFSET          0x0008
#define GPT_IR_OFFSET          0x000C
#define GPT_OCR1_OFFSET        0x0010
#define GPT_OCR2_OFFSET        0x0014
#define GPT_OCR3_OFFSET        0x0018
#define GPT_ICR1_OFFSET        0x001C
#define GPT_ICR2_OFFSET        0x0020
#define GPT_CNT_OFFSET         0x0024


#define GPT_CR_EN_LSH          0
#define GPT_CR_ENMOD_LSH       1
#define GPT_CR_DBGEN_LSH       2
#define GPT_CR_WAITEN_LSH      3
#define GPT_CR_STOPEN_LSH      5
#define GPT_CR_CLKSRC_LSH      6
#define GPT_CR_FRR_LSH         9
#define GPT_CR_EN_24M_LSH      10
#define GPT_CR_SWR_LSH         15
#define GPT_CR_IM1_LSH         16
#define GPT_CR_IM2_LSH         18
#define GPT_CR_OM1_LSH         20
#define GPT_CR_OM2_LSH         23
#define GPT_CR_OM3_LSH         26
#define GPT_CR_FO1_LSH         29
#define GPT_CR_FO2_LSH         30
#define GPT_CR_FO3_LSH         31

#define GPT_PR_PRESCALER_LSH   0

#define GPT_SR_OF1_LSH         0
#define GPT_SR_OF2_LSH         1
#define GPT_SR_OF3_LSH         2
#define GPT_SR_IF1_LSH         3
#define GPT_SR_IF2_LSH         4
#define GPT_SR_ROV_LSH         5

#define GPT_IR_OF1IE_LSH       0
#define GPT_IR_OF2IE_LSH       1
#define GPT_IR_OF3IE_LSH       2
#define GPT_IR_IF1IE_LSH       3
#define GPT_IR_IF2IE_LSH       4
#define GPT_IR_ROVIE_LSH       5

#define GPT_OCR1_COMP_LSH      0
#define GPT_OCR2_COMP_LSH      0
#define GPT_OCR3_COMP_LSH      0
#define GPT_ICR1_CAPT_LSH      0
#define GPT_ICR2_CAPT_LSH      0
#define GPT_CNT_COUNT_LSH      0

#define GPT_CR_EN_WID          1
#define GPT_CR_ENMOD_WID       1
#define GPT_CR_DBGEN_WID       1
#define GPT_CR_WAITEN_WID      1
#define GPT_CR_STOPEN_WID      1
#define GPT_CR_CLKSRC_WID      3
#define GPT_CR_FRR_WID         1
#define GPT_CR_SWR_WID         1
#define GPT_CR_IM1_WID         2
#define GPT_CR_IM2_WID         2
#define GPT_CR_OM1_WID         3
#define GPT_CR_OM2_WID         3
#define GPT_CR_OM3_WID         3
#define GPT_CR_FO1_WID         1
#define GPT_CR_FO2_WID         1
#define GPT_CR_FO3_WID         1

#define GPT_PR_PRESCALER_WID   12

#define GPT_SR_OF1_WID         1
#define GPT_SR_OF2_WID         1
#define GPT_SR_OF3_WID         1
#define GPT_SR_IF1_WID         1
#define GPT_SR_IF2_WID         1
#define GPT_SR_ROV_WID         1

#define GPT_IR_OF1IE_WID       1
#define GPT_IR_OF2IE_WID       1
#define GPT_IR_OF3IE_WID       1
#define GPT_IR_IF1IE_WID       1
#define GPT_IR_IF2IE_WID       1
#define GPT_IR_ROVIE_WID       1

#define GPT_OCR1_COMP_WID      32
#define GPT_OCR2_COMP_WID      32
#define GPT_OCR3_COMP_WID      32
#define GPT_ICR1_CAPT_WID      32
#define GPT_ICR2_CAPT_WID      32
#define GPT_CNT_COUNT_WID      32


//------------------------------------------------------------------------------
// REGISTER BIT WRITE VALUES
//------------------------------------------------------------------------------

// GPTCR
#define GPT_CR_EN_ENABLE                1 // GPT enabled
#define GPT_CR_EN_DISABLE               0 // GPT disabled

#define GPT_CR_ENMOD_RESET              1 // GPT counter reset to
                                          //   0 when disabled
#define GPT_CR_ENMOD_RETAIN             0 // GPT counter retains
                                          //   value when disabled

#define GPT_CR_DBGEN_ENABLE             1 // GPT enabled in debug mode
#define GPT_CR_DBGEN_DISABLE            0 // GPT disabled in debug mode

#define GPT_CR_WAITEN_ENABLE            1 // GPT enabled in wait mode
#define GPT_CR_WAITEN_DISABLE           0 // GPT disabled in wait mode

#define GPT_CR_STOPEN_ENABLE            1 // GPT enabled in stopdoze mode
#define GPT_CR_STOPEN_DISABLE           0 // GPT disabled in stopoze mode

#define GPT_CR_CLKSRC_NOCLK             0 // No clock to GPT
#define GPT_CR_CLKSRC_IPGCLK            1 // ipg_clk is the clock source
#define GPT_CR_CLKSRC_HIGHFREQ          2 // ipg_clk_highfreq
#define GPT_CR_CLKSRC_EXTCLK            3 // ipp_gpt_clkin (external clock
#define GPT_CR_CLKSRC_CLK32K            4 // ipg_clk_32k is clock source
#define GPT_CR_CLKSRC_CLK24M            5 // crystal oscillator (24 Mhz) is clock source

#define GPT_CR_FRR_FREERUN              1 // Freerun mode (counter
                                          //   continues after compare)
#define GPT_CR_FRR_RESTART              0 // Restart mode (counter set
                                          //   to zero after compare)
#define GPT_CR_EN_24M_DISABLE           0  // 24M clock disabled
#define GPT_CR_EN_24M_ENABLE            1  // 24M clock enabled

#define GPT_CR_SWR_RESET                1 // Self-clearing software reset
#define GPT_CR_SWR_NORESET              0 // Do not activate software reset

#define GPT_CR_IM1_DISABLE              0 // Capture Disabled
#define GPT_CR_IM1_EDGE_RISE            1 // Capture on rising edge
#define GPT_CR_IM1_EDGE_FALL            2 // Capture on falling edge
#define GPT_CR_IM1_EDGE_BOTH            3 // Capture on both edges

#define GPT_CR_IM2_DISABLE              0 // Capture Disabled
#define GPT_CR_IM2_EDGE_RISE            1 // Capture on rising edge
#define GPT_CR_IM2_EDGE_FALL            2 // Capture on falling edge
#define GPT_CR_IM2_EDGE_BOTH            3 // Capture on both edges

#define GPT_CR_OM1_DISABLE              0 // Compare generates no response
#define GPT_CR_OM1_TOGGLE               1 // Compare toggles output pin
#define GPT_CR_OM1_CLEAR                2 // Compare clears output pin
#define GPT_CR_OM1_SET                  3 // Compare sets output pin
#define GPT_CR_OM1_PULSE                4 // Compare event generates a
                                          //   single count duration pulse
                                          //   on output pin

#define GPT_CR_OM2_DISABLE              0 // Compare generates no response
#define GPT_CR_OM2_TOGGLE               1 // Compare toggles output pin
#define GPT_CR_OM2_CLEAR                2 // Compare clears output pin
#define GPT_CR_OM2_SET                  3 // Compare sets output pin
#define GPT_CR_OM2_PULSE                4 // Compare event generates a
                                          //   single count duration pulse
                                          //   on output pin

#define GPT_CR_OM3_DISABLE              0 // Compare generates no response
#define GPT_CR_OM3_TOGGLE               1 // Compare toggles output pin
#define GPT_CR_OM3_CLEAR                2 // Compare clears output pin
#define GPT_CR_OM3_SET                  3 // Compare sets output pin
#define GPT_CR_OM3_PULSE                4 // Compare event generates a
                                          //   single count duration pulse
                                          //   on output pin

#define GPT_CR_FO1_FORCE                1 // Force pin action programmed
                                          //   for output compare 1 pin.
                                          //   Pin is self-negating.
#define GPT_CR_FO1_NOFORCE              0 // Do not force pin

#define GPT_CR_FO2_FORCE                1 // Force pin action programmed
                                          //   for output compare 1 pin
                                          //   Pin is self-negating.
#define GPT_CR_FO2_NOFORCE              0 // Do not force pin

#define GPT_CR_FO3_FORCE                1 // Force pin action programmed
                                          //   for output compare 1 pin
                                          //   Pin is self-negating.
#define GPT_CR_FO3_NOFORCE              0 // Do not force pin

// GPTSR
#define GPT_SR_OF1_STATUS_CLEAR         1 // Output compare 1 status clear
#define GPT_SR_OF2_STATUS_CLEAR         1 // Output compare 2 status clear
#define GPT_SR_OF3_STATUS_CLEAR         1 // Output compare 3 status clear
#define GPT_SR_IF1_STATUS_CLEAR         1 // Input capture 1 status clear
#define GPT_SR_IF2_STATUS_CLEAR         1 // Input capture 2 status clear
#define GPT_SR_ROV_STATUS_CLEAR         1 // Rollover status clear

// GPTIR
#define GPT_IR_OF1IE_INT_ENABLE         1 // Output compare 1 int enabled
#define GPT_IR_OF1IE_INT_DISABLE        0 // Output compare 1 int disabled

#define GPT_IR_OF2IE_INT_ENABLE         1 // Output compare 2 int enabled
#define GPT_IR_OF2IE_INT_DISABLE        0 // Output compare 2 int disabled

#define GPT_IR_OF3IE_INT_ENABLE         1 // Output compare 3 int enabled
#define GPT_IR_OF3IE_INT_DISABLE        0 // Output compare 3 int disabled

#define GPT_IR_IF1IE_INT_ENABLE         1 // Input capture 1 int enabled
#define GPT_IR_IF1IE_INT_DISABLE        0 // Input capture 1 int disabled

#define GPT_IR_IF2IE_INT_ENABLE         1 // Input capture 2 int enabled
#define GPT_IR_IF2IE_INT_DISABLE        0 // Input capture 2 int disabled

#define GPT_IR_ROVIE_INT_ENABLE         1 // Rollover int enabled
#define GPT_IR_ROVIE_INT_DISABLE        0 // Rollover int disabled

#endif // __COMMON_GPT_H
