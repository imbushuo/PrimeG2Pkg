/** @file
*
*  Copyright (c) 2018 Microsoft Corporation. All rights reserved.
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

#ifndef _IMX_GPIO_H_
#define _IMX_GPIO_H_

#include <Library/PcdLib.h>

typedef enum {
  IMX_GPIO_LOW = 0,
  IMX_GPIO_HIGH = 1
} IMX_GPIO_VALUE;

typedef enum {
  IMX_GPIO_DIR_INPUT,
  IMX_GPIO_DIR_OUTPUT
} IMX_GPIO_DIR;

typedef enum {
  IMX_GPIO_BANK1 = 1,
  IMX_GPIO_BANK2,
  IMX_GPIO_BANK3,
  IMX_GPIO_BANK4,
  IMX_GPIO_BANK5,
  IMX_GPIO_BANK6,
  IMX_GPIO_BANK7,
} IMX_GPIO_BANK;

#pragma pack(push, 1)

#define GPIO_RESERVED_SIZE \
    ((FixedPcdGet32(PcdGpioBankMemoryRange) / 4) - 8)

typedef struct {
  UINT32 DR;                            // 0x00 GPIO data register (GPIO1_DR)
  UINT32 GDIR;                          // 0x04 GPIO direction register (GPIO1_GDIR)
  UINT32 PSR;                           // 0x08 GPIO pad status register (GPIO1_PSR)
  UINT32 ICR1;                          // 0x0C GPIO interrupt configuration register1 (GPIO1_ICR1)
  UINT32 ICR2;                          // 0x10 GPIO interrupt configuration register2 (GPIO1_ICR2)
  UINT32 IMR;                           // 0x14 GPIO interrupt mask register (GPIO1_IMR)
  UINT32 ISR;                           // 0x18 GPIO interrupt status register (GPIO1_ISR)
  UINT32 EDGE_SEL;                      // 0x1C GPIO edge select register (GPIO1_EDGE_SEL)
  UINT32 reserved[GPIO_RESERVED_SIZE];
} IMX_GPIO_BANK_REGISTERS;

#pragma pack(pop)

typedef struct {
  IMX_GPIO_BANK_REGISTERS Banks[7];
} IMX_GPIO_REGISTERS;

/**
    Set the specified GPIO to the specified direction.
**/
VOID
ImxGpioDirection (
  IMX_GPIO_BANK Bank,
  UINT32 IoNumber,
  IMX_GPIO_DIR Direction
  );

/**
    Write a value to a GPIO pin.
**/
VOID
ImxGpioWrite (
  IMX_GPIO_BANK Bank,
  UINT32 IoNumber,
  IMX_GPIO_VALUE Value
  );

/**
    Read a GPIO pin input value.
**/
IMX_GPIO_VALUE
ImxGpioRead (
  IMX_GPIO_BANK Bank,
  UINT32 IoNumber
  );

#endif
