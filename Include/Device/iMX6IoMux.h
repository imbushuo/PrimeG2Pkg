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

#ifndef _IMX6_IOMUX_H_
#define _IMX6_IOMUX_H_

//
// IOMux common definition
//
#include "iMXIoMux.h"

//
// GPIO common definition
//
#include "iMXGpio.h"

#include "iMX6IoMux_ULL.h"

typedef UINT64 IMX_PADCFG;

//
// Pad control settings
//
typedef enum {
  IMX_HYS_DISABLED,
  IMX_HYS_ENABLED,
} IMX_HYS;

typedef enum {
  IMX_PUS_100K_OHM_PD,
  IMX_PUS_47K_OHM_PU,
  IMX_PUS_100K_OHM_PU,
  IMX_PUS_22K_OHM_PU,
} IMX_PUS;

typedef enum {
  IMX_PUE_KEEP,
  IMX_PUE_PULL,
} IMX_PUE;

typedef enum {
  IMX_PKE_DISABLE,
  IMX_PKE_ENABLE,
} IMX_PKE;

typedef enum {
  IMX_ODE_DISABLE,
  IMX_ODE_ENABLE,
} IMX_ODE;

typedef enum {
  IMX_SPEED_LOW,
  IMX_SPEED_MEDIUM = 2,
  IMX_SPEED_MAXIMUM,
} IMX_SPEED;

typedef enum {
  IMX_DSE_HIZ,
  IMX_DSE_260_OHM,
  IMX_DSE_130_OHM,
  IMX_DSE_90_OHM,
  IMX_DSE_60_OHM,
  IMX_DSE_50_OHM,
  IMX_DSE_40_OHM,
  IMX_DSE_33_OHM,
} IMX_DSE;

typedef enum {
  IMX_SRE_SLOW,
  IMX_SRE_FAST,
} IMX_SRE;

typedef enum {
  IMX_SION_DISABLED,
  IMX_SION_ENABLED,
} IMX_IOMUXC_CTL_SION;

typedef union {
  UINT32 AsUint32;
  struct {
    UINT32 SRE : 1;
    UINT32 reserved0 : 2;
    UINT32 DSE : 3;
    UINT32 SPEED : 2;
    UINT32 reserved1 : 3;
    UINT32 ODE : 1 ;
    UINT32 PKE : 1;
    UINT32 PUE : 1;
    UINT32 PUS : 2;
    UINT32 HYS : 1;
    UINT32 reserved2 : 15;
  } Fields;
} IMX_IOMUXC_PAD_CTL;

typedef union {
  UINT32 AsUint32;
  struct {
    UINT32 MUX_MODE : 3;
    UINT32 reserved0 : 1;
    UINT32 SION : 1;
    UINT32 reserved1 : 27;
  } Fields;
} IMX_IOMUXC_MUX_CTL;

typedef union {
  UINT32 AsUint32;
  struct {
    UINT32 DAISY : 3;
    UINT32 reserved : 29;
  } Fields;
} IMX_IOMUXC_SEL_INP_CTL;

#define _IMX_SEL_INP_VALUE(InpSel) \
          (((InpSel) >> 8) & 0x07)

#define _IMX_SEL_INP_REGISTER(InpSel) \
          ((((InpSel) & 0xFF) * 4) + IOMUXC_SELECT_INPUT_BASE_ADDRESS)

#define _IMX_MAKE_INP_SEL(InpSelReg, InpSelVal) \
          (((((InpSelReg) - IOMUXC_SELECT_INPUT_BASE_ADDRESS) / 4) & 0xFF) | \
          (((InpSelVal) & 0x7) << 8))

#define _IMX_MAKE_MUX_CTL(Sion, MuxAlt) \
          (((MuxAlt) & 0x7) | \
          (((Sion) & 0x1) << 4))

#define _IMX_MAKE_PAD_CTL(Sre, Dse, Speed, Ode, Pke, Pue, Pus, Hys) \
          (((Sre) & 0x1) | \
          (((Dse) & 0x7) << 3) | \
          (((Speed) & 0x3) << 6) | \
          (((Ode) & 0x1) << 11) | \
          (((Pke) & 0x1) << 12) | \
          (((Pue) & 0x1) << 13) | \
          (((Pus) & 0x3) << 14) | \
          (((Hys) & 0x1) << 16))

/**
  Define a configuration for a pad, including drive settings,
  MUX setting and Select Input setting and offset.

  Sre - IMX_SRE - Slew Rate setting
  Dse - IMX_DSE - Drive strength
  Speed - IMX_SPEED - Pad speed setting
  Ode - IMX_ODE - Open drain enable
  Pke - IMX_PKE - Pull/Keeper enable
  Pue - IMX_PUE - Pull/Keep mode select
  Pus - IMX_PUS - Pull strength
  Hys - IMX_HYS - Hysteresis enable/disable
  Sion - Software Input on Field
  MuxAlt- Alternate function number
  SelInpReg - select input register offset div 4
  SelInpVal - select input value

  NOTE: _IMX_MAKE_PADCFG_INPSEL cannot take SelInpValue of 4 or higher otherwise
  the macro overflows the size of an int.
  For SelInpValue higher than 4, set SELECT_INPUT register manually using
  IMX_IOMUXC_SEL_INP_CTL structure.
**/
#define _IMX_MAKE_PADCFG_INPSEL(Sre, Dse, Speed, Ode, Pke, Pue, Pus, Hys, Sion, MuxAlt, SelInpReg, SelInpValue) \
          (_IMX_MAKE_PAD_CTL(Sre, Dse, Speed, Ode, Pke, Pue, Pus, Hys) | \
           (_IMX_MAKE_MUX_CTL(Sion, MuxAlt) << 17) | \
           (_IMX_MAKE_INP_SEL(SelInpReg, SelInpValue) << 22))

#define _IMX_MAKE_PADCFG(Sre, Dse, Speed, Ode, Pke, Pue, Pus, Hys, Sion, MuxAlt) \
          (_IMX_MAKE_PAD_CTL(Sre, Dse, Speed, Ode, Pke, Pue, Pus, Hys) | \
           _IMX_MAKE_MUX_CTL(Sion, MuxAlt) << 17)

#define _IMX_MAKE_PADCFG2(PadCtl, Sion, MuxAlt) \
          ((PadCtl) | \
           _IMX_MAKE_MUX_CTL(Sion, MuxAlt) << 17)

#define _IMX_PADCFG_PAD_CTL(PadCfg) ((PadCfg) & 0x0001F8F9)
#define _IMX_PADCFG_MUX_CTL(PadCfg) (((PadCfg) >> 17) & 0x00000017)
#define _IMX_PADCFG_SEL_INP(PadCfg) (((PadCfg) >> 22) & 0x000007FF)

/**
  Put a pad in the specified configuration.

  For example, to configure GPIO0 as CCM_CLK01 output:
    ImxPadConfig (IMX_PAD_GPIO_0, IMX_PAD_GPIO_0_CCM_CLKO1);

**/
VOID ImxPadConfig (IMX_PAD Pad, IMX_PADCFG PadConfig);

/**
  Dumps to console the specified PAD mux/control configuration.
**/
VOID ImxPadDumpConfig (char *PadName, IMX_PAD Pad);

#endif // _IMX6_IOMUX_H_
