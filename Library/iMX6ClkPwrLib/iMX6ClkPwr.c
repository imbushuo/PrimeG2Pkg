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

#include <PiDxe.h>

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>

#include <Device/imx6_ull.h>
#include <Device/iMX6ClkPwr.h>
#include "iMX6ClkPwr_private.h"

static IMX_CLOCK_TREE_CACHE mImxpClockPwrCache;   // Cached clock value

STATIC CONST IMX_CCGR_INDEX ImxpCcgrIndexMap[] = {
  {0, 0},  // MX6_AIPS_TZ1_CLK_ENABLE
  {0, 1},  // MX6_AIPS_TZ2_CLK_ENABLE
  {0, 2},  // MX6_APBHDMA_HCLK_ENABLE
  {0, 3},  // MX6_ASRC_CLK_ENABLE
  {0, 4},  // MX6_CAAM_SECURE_MEM_CLK_ENABLE
  {0, 5},  // MX6_CAAM_WRAPPER_ACLK_ENABLE
  {0, 6},  // MX6_CAAM_WRAPPER_IPG_ENABLE
  {0, 7},  // MX6_CAN1_CLK_ENABLE
  {0, 8},  // MX6_CAN1_SERIAL_CLK_ENABLE
  {0, 9},  // MX6_CAN2_CLK_ENABLE
  {0, 10}, // MX6_CAN2_SERIAL_CLK_ENABLE
  {0, 11}, // MX6_ARM_DBG_CLK_ENABLE
  {0, 12}, // MX6_DCIC1_CLK_ENABLE
  {0, 13}, // MX6_DCIC2_CLK_ENABLE
  {0, 14}, // MX6_DTCP_CLK_ENABLE
  {1, 0},  // MX6_ECSPI1_CLK_ENABLE
  {1, 1},  // MX6_ECSPI2_CLK_ENABLE
  {1, 2},  // MX6_ECSPI3_CLK_ENABLE
  {1, 3},  // MX6_ECSPI4_CLK_ENABLE
  {1, 4},  // MX6_ECSPI5_CLK_ENABLE
  {1, 5},  // MX6_ENET_CLK_ENABLE
  {1, 6},  // MX6_EPIT1_CLK_ENABLE
  {1, 7},  // MX6_EPIT2_CLK_ENABLE
  {1, 8},  // MX6_ESAI_CLK_ENABLE
  {1, 10}, // MX6_GPT_CLK_ENABLE
  {1, 11}, // MX6_GPT_SERIAL_CLK_ENABLE
  {1, 12}, // MX6_GPU2D_CLK_ENABLE
  {1, 13}, // MX6_GPU3D_CLK_ENABLE
  {2, 0},  // MX6_HDMI_TX_ENABLE
  {2, 2},  // MX6_HDMI_TX_ISFRCLK_ENABLE
  {2, 3},  // MX6_I2C1_SERIAL_CLK_ENABLE
  {2, 4},  // MX6_I2C2_SERIAL_CLK_ENABLE
  {2, 5},  // MX6_I2C3_SERIAL_CLK_ENABLE
  {2, 6},  // MX6_IIM_CLK_ENABLE
  {2, 7},  // MX6_IOMUX_IPT_CLK_IO_ENABLE
  {2, 8},  // MX6_IPMUX1_CLK_ENABLE
  {2, 9},  // MX6_IPMUX2_CLK_ENABLE
  {2, 10}, // MX6_IPMUX3_CLK_ENABLE
  {2, 11}, // MX6_IPSYNC_IP2APB_TZASC1_IPG_MASTER_CLK_ENABLE
  {2, 12}, // MX6_IPSYNC_IP2APB_TZASC2_IPG_MASTER_CLK_ENABLE
  {2, 13}, // MX6_IPSYNC_VDOA_IPG_MASTER_CLK_ENABLE
  {3, 0},  // MX6_IPU1_IPU_CLK_ENABLE
  {3, 1},  // MX6_IPU1_IPU_DI0_CLK_ENABLE
  {3, 2},  // MX6_IPU1_IPU_DI1_CLK_ENABLE
  {3, 3},  // MX6_IPU2_IPU_CLK_ENABLE
  {3, 4},  // MX6_IPU2_IPU_DI0_CLK_ENABLE
  {3, 5},  // MX6_IPU2_IPU_DI1_CLK_ENABLE
  {3, 6},  // MX6_LDB_DI0_CLK_ENABLE
  {3, 7},  // MX6_LDB_DI1_CLK_ENABLE
  {3, 8},  // MX6_MIPI_CORE_CFG_CLK_ENABLE
  {3, 9},  // MX6_MLB_CLK_ENABLE
  {3, 10}, // MX6_MMDC_CORE_ACLK_FAST_CORE_P0_ENABLE
  {3, 12}, // MX6_MMDC_CORE_IPG_CLK_P0_ENABLE
  {3, 14}, // MX6_OCRAM_CLK_ENABLE
  {3, 15}, // MX6_OPENVGAXICLK_CLK_ROOT_ENABLE
  {4, 0},  // MX6_PCIE_ROOT_ENABLE
  {4, 4},  // MX6_PL301_MX6QFAST1_S133CLK_ENABLE
  {4, 6},  // MX6_PL301_MX6QPER1_BCHCLK_ENABLE
  {4, 7},  // MX6_PL301_MX6QPER2_MAINCLK_ENABLE
  {4, 8},  // MX6_PWM1_CLK_ENABLE
  {4, 9},  // MX6_PWM2_CLK_ENABLE
  {4, 10}, // MX6_PWM3_CLK_ENABLE
  {4, 11}, // MX6_PWM4_CLK_ENABLE
  {4, 12}, // MX6_RAWNAND_U_BCH_INPUT_APB_CLK_ENABLE
  {4, 13}, // MX6_RAWNAND_U_GPMI_BCH_INPUT_BCH_CLK_ENABLE
  {4, 14}, // MX6_RAWNAND_U_GPMI_BCH_INPUT_GPMI_IO_CLK_ENABLE
  {4, 15}, // MX6_RAWNAND_U_GPMI_INPUT_APB_CLK_ENABLE
  {5, 0},  // MX6_ROM_CLK_ENABLE
  {5, 2},  // MX6_SATA_CLK_ENABLE
  {5, 3},  // MX6_SDMA_CLK_ENABLE
  {5, 6},  // MX6_SPBA_CLK_ENABLE
  {5, 7},  // MX6_SPDIF_CLK_ENABLE
  {5, 9},  // MX6_SSI1_CLK_ENABLE
  {5, 10}, // MX6_SSI2_CLK_ENABLE
  {5, 11}, // MX6_SSI3_CLK_ENABLE
  {5, 12}, // MX6_UART_CLK_ENABLE
  {5, 13}, // MX6_UART_SERIAL_CLK_ENABLE
  {6, 0},  // MX6_USBOH3_CLK_ENABLE
  {6, 1},  // MX6_USDHC1_CLK_ENABLE
  {6, 2},  // MX6_USDHC2_CLK_ENABLE
  {6, 3},  // MX6_USDHC3_CLK_ENABLE
  {6, 4},  // MX6_USDHC4_CLK_ENABLE
  {6, 5},  // MX6_EIM_SLOW_CLK_ENABLE
  {6, 6},  // MX6_VDOAXICLK_CLK_ENABLE
  {6, 7},  // MX6_VPU_CLK_ENABLE
};

STATIC IMX_CLOCK_CONTEXT ExpectedClocks[] = {
  {IMX_OSC_CLK, {24000000, IMX_CLK_NONE}},
  {IMX_PLL1_MAIN_CLK, {792000000, IMX_OSC_CLK}},
  {IMX_PLL2_MAIN_CLK, {528000000, IMX_OSC_CLK}},
  {IMX_PLL2_PFD0, {306580645, IMX_PLL2_MAIN_CLK}},
  {IMX_PLL2_PFD1, {528000000, IMX_PLL2_MAIN_CLK}},
  {IMX_PLL2_PFD2, {396000000, IMX_PLL2_MAIN_CLK}},
  {IMX_PLL3_MAIN_CLK, {480000000, IMX_OSC_CLK}},
  {IMX_PLL3_PFD0, {720000000, IMX_PLL3_MAIN_CLK}},
  {IMX_PLL3_PFD1, {540000000, IMX_PLL3_MAIN_CLK}},
  {IMX_PLL3_PFD2, {508235294, IMX_PLL3_MAIN_CLK}},
  {IMX_PLL3_PFD3, {454736842, IMX_PLL3_MAIN_CLK}},
  {IMX_AXI_CLK_ROOT, {198000000, IMX_PERIPH_CLK}},
  {IMX_MMDC_CH0_CLK_ROOT, {396000000, IMX_PERIPH_CLK}},
};

/**
  Get the CCGR register index and gate number for a clock gate.

  @param[in]  ClockGate   Specific clock gate to get CCGR index
**/
IMX_CCGR_INDEX
ImxpCcgrIndexFromClkGate (
  IN  IMX_CLK_GATE    ClockGate
  )
{
  return ImxpCcgrIndexMap[ClockGate];
}

CONST CHAR16*
StringFromImxClk (
  IN  IMX_CLK   Value
  )
{
  switch (Value) {
  case IMX_CLK_NONE:
    return L"(none)";
  case IMX_OSC_CLK:
    return L"OSC_CLK";
  case IMX_PLL1_MAIN_CLK:
    return L"PLL1_MAIN_CLK";
  case IMX_PLL2_MAIN_CLK:
    return L"PLL2_MAIN_CLK";
  case IMX_PLL2_PFD0:
    return L"PLL2_PFD0";
  case IMX_PLL2_PFD1:
    return L"PLL2_PFD1";
  case IMX_PLL2_PFD2:
    return L"PLL2_PFD2";
  case IMX_PLL3_MAIN_CLK:
    return L"PLL3_MAIN_CLK";
  case IMX_PLL3_PFD0:
    return L"PLL3_PFD0";
  case IMX_PLL3_PFD1:
    return L"PLL3_PFD1";
  case IMX_PLL3_PFD2:
    return L"PLL3_PFD2";
  case IMX_PLL3_PFD3:
    return L"PLL3_PFD3";
  case IMX_PLL4_MAIN_CLK:
    return L"PLL4_MAIN_CLK";
  case IMX_PLL5_MAIN_CLK:
    return L"PLL5_MAIN_CLK";
  case IMX_CLK1:
    return L"CLK1";
  case IMX_CLK2:
    return L"CLK2";
  case IMX_PLL1_SW_CLK:
    return L"PLL1_SW_CLK";
  case IMX_STEP_CLK:
    return L"STEP_CLK";
  case IMX_PLL3_SW_CLK:
    return L"PLL3_SW_CLK";
  case IMX_AXI_ALT:
    return L"AXI_ALT";
  case IMX_AXI_CLK_ROOT:
    return L"AXI_CLK_ROOT";
  case IMX_PERIPH_CLK2:
    return L"PERIPH_CLK2";
  case IMX_PERIPH_CLK:
    return L"PERIPH_CLK";
  case IMX_PRE_PERIPH_CLK:
    return L"PRE_PERIPH_CLK";
  case IMX_PRE_PERIPH2_CLK:
    return L"PRE_PERIPH2_CLK";
  case IMX_PERIPH2_CLK:
    return L"PERIPH2_CLK";
  case IMX_ARM_CLK_ROOT:
    return L"ARM_CLK_ROOT";
  case IMX_MMDC_CH0_CLK_ROOT:
    return L"MMDC_CH0_CLK_ROOT";
  case IMX_MMDC_CH1_CLK_ROOT:
    return L"MMDC_CH1_CLK_ROOT";
  case IMX_AHB_CLK_ROOT:
    return L"AHB_CLK_ROOT";
  case IMX_IPG_CLK_ROOT:
    return L"IPG_CLK_ROOT";
  case IMX_PERCLK_CLK_ROOT:
    return L"PERCLK_CLK_ROOT";
  case IMX_USDHC1_CLK_ROOT:
    return L"USDHC1_CLK_ROOT";
  case IMX_USDHC2_CLK_ROOT:
    return L"USDHC2_CLK_ROOT";
  case IMX_USDHC3_CLK_ROOT:
    return L"USDHC3_CLK_ROOT";
  case IMX_USDHC4_CLK_ROOT:
    return L"USDHC4_CLK_ROOT";
  case IMX_SSI1_CLK_ROOT:
    return L"SSI1_CLK_ROOT";
  case IMX_SSI2_CLK_ROOT:
    return L"SSI2_CLK_ROOT";
  case IMX_SSI3_CLK_ROOT:
    return L"SSI3_CLK_ROOT";
  case IMX_GPU2D_AXI_CLK_ROOT:
    return L"GPU2D_AXI_CLK_ROOT";
  case IMX_GPU3D_AXI_CLK_ROOT:
    return L"GPU3D_AXI_CLK_ROOT";
  case IMX_PCIE_AXI_CLK_ROOT:
    return L"PCIE_AXI_CLK_ROOT";
  case IMX_VDO_AXI_CLK_ROOT:
    return L"VDO_AXI_CLK_ROOT";
  case IMX_IPU1_HSP_CLK_ROOT:
    return L"IPU1_HSP_CLK_ROOT";
  case IMX_GPU2D_CORE_CLK_ROOT:
    return L"GPU2D_CORE_CLK_ROOT";
  case IMX_ACLK_EIM_SLOW_CLK_ROOT:
    return L"ACLK_EIM_SLOW_CLK_ROOT";
  case IMX_ACLK_CLK_ROOT:
    return L"ACLK_CLK_ROOT";
  case IMX_ENFC_CLK_ROOT:
    return L"ENFC_CLK_ROOT";
  case IMX_GPU3D_CORE_CLK_ROOT:
    return L"GPU3D_CORE_CLK_ROOT";
  case IMX_GPU3D_SHADER_CLK_ROOT:
    return L"GPU3D_SHADER_CLK_ROOT";
  case IMX_VPU_AXI_CLK_ROOT:
    return L"VPU_AXI_CLK_ROOT";
  case IMX_IPU1_DI0_CLK_ROOT:
    return L"IPU1_DI0_CLK_ROOT";
  case IMX_IPU1_DI1_CLK_ROOT:
    return L"IPU1_DI1_CLK_ROOT";
  case IMX_LDB_DI0_SERIAL_CLK_ROOT:
    return L"LDB_DI0_SERIAL_CLK_ROOT";
  case IMX_LDB_DI0_IPU:
    return L"LDB_DI0_IPU";
  case IMX_LDB_DI1_SERIAL_CLK_ROOT:
    return L"LDB_DI1_SERIAL_CLK_ROOT";
  case IMX_LDB_DI1_IPU:
    return L"LDB_DI1_IPU";
  case IMX_SPDIF0_CLK_ROOT:
    return L"SPDIF0_CLK_ROOT";
  case IMX_SPDIF1_CLK_ROOT:
    return L"SPDIF1_CLK_ROOT";
  case IMX_ESAI_CLK_ROOT:
    return L"ESAI_CLK_ROOT";
  case IMX_HSI_TX_CLK_ROOT:
    return L"HSI_TX_CLK_ROOT";
  case IMX_CAN_CLK_ROOT:
    return L"CAN_CLK_ROOT";
  case IMX_ECSPI_CLK_ROOT:
    return L"ECSPI_CLK_ROOT";
  case IMX_UART_CLK_ROOT:
    return L"UART_CLK_ROOT";
  case IMX_VIDEO_27M_CLK_ROOT:
    return L"VIDEO_27M_CLK_ROOT";
  default:
    ASSERT (FALSE);
    return L"[Invalid IMX_CLK value]";
  }
}

IMX_CLK
ImxpClkFromBypassClkSource (
  IN  IMX_PLL_BYPASS_CLK_SRC    BypassClockSource
  )
{
  switch (BypassClockSource) {
  case IMX_PLL_BYPASS_CLK_SRC_REF_CLK_24M:
    return IMX_OSC_CLK;
  case IMX_PLL_BYPASS_CLK_SRC_CLK1:
    return IMX_CLK1;
  case IMX_PLL_BYPASS_CLK_SRC_CLK2:
    return IMX_CLK2;
  case IMX_PLL_BYPASS_CLK_SRC_XOR:
  default:
    ASSERT (FALSE);
    return IMX_CLK_NONE;
  }
}

/**
  Configure the GPU clock tree so that GPU2D and GPU3D are clocked from
  the AXI clock root and are within the allowed frequency range.

  The GPU must be powered down, and GPU clocks must be gated when this
  function is called.
**/
VOID
ImxCcmConfigureGpuClockTree (
  VOID
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCMR_REG           CbcmrReg;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CbcmrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCMR);

  CbcmrReg.gpu2d_axi_clk_sel = IMX_CCM_GPU2D_AXI_CLK_SEL_AXI;
  CbcmrReg.gpu3d_axi_clk_sel = IMX_CCM_GPU3D_AXI_CLK_SEL_AXI;
  CbcmrReg.gpu2d_core_clk_sel = IMX_CCM_GPU2D_CORE_CLK_SEL_PLL2_PFD0;
  CbcmrReg.gpu3d_core_clk_sel = IMX_CCM_GPU3D_CORE_CLK_SEL_MMDC_CH0_AXI;
  CbcmrReg.gpu3d_shader_clk_sel = IMX_CCM_GPU3D_SHADER_CLK_SEL_MMDC_CH0_AXI;
  CbcmrReg.gpu2d_core_clk_podf = 0;
  CbcmrReg.gpu3d_core_podf = 0;
  CbcmrReg.gpu3d_shader_podf = 0;

  ImxpClkPwrCacheReset ();
  MmioWrite32 ((UINTN) &pCcmRegisters->CBCMR, CbcmrReg.AsUint32);
}

/**
  Configure all of DIx clock tree for both IPU1 and IPU2. For flexibility
  purpose use PLL5 (PLL Video) as main reference clock. PLL 5 has flexible
  divider making it easily configurable. Muxing and clock programming needs
  when to be updated when supporting multiple display.
**/
VOID
ImxCcmConfigureIPUDIxClockTree (
  VOID
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CHSCCDR_REG         ChscddrReg;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  ChscddrReg.AsUint32 = MmioRead32 ((UINTN)&pCcmRegisters->CHSCCDR);

  // Setup muxing to pre-mux
  if (FeaturePcdGet (PcdLvdsEnable)) {
    ChscddrReg.ipu1_di0_clk_sel = IMX_CHSCCDR_IPU1_DI0_CLK_SEL_LDB_DI0_CLK;
    ChscddrReg.ipu1_di1_clk_sel = IMX_CHSCCDR_IPU1_DI0_CLK_SEL_LDB_DI0_CLK;
  } else {
    ChscddrReg.ipu1_di0_clk_sel = IMX_CHSCCDR_IPU1_DI0_CLK_SEL_PREMUX;
    ChscddrReg.ipu1_di1_clk_sel = IMX_CHSCCDR_IPU1_DI1_CLK_SEL_PREMUX;
  }
  ChscddrReg.ipu1_di0_podf = IMX_CHSCCDR_IPU1_DI0_PODF_DIV_1;
  ChscddrReg.ipu1_di1_podf = IMX_CHSCCDR_IPU1_DI1_PODF_DIV_1;
  ChscddrReg.ipu1_di0_pre_clk_sel = IMX_CHSCCDR_IPU1_DI0_PRE_CLK_SEL_PLL5;
  ChscddrReg.ipu1_di1_pre_clk_sel = IMX_CHSCCDR_IPU1_DI1_PRE_CLK_SEL_PLL5;

  MmioWrite32 ((UINTN)&pCcmRegisters->CHSCCDR, ChscddrReg.AsUint32);
}

#if defined(CPU_IMX6D) || defined(CPU_IMX6Q) || defined(CPU_IMX6DP) || defined(CPU_IMX6QP)
/**
    Configure both LDB0/1 to use PLL5 clock
**/
VOID
ImxCcmConfigureIPULDBxClockTree (
  VOID
  )
{
  volatile IMX_CCM_REGISTERS *pCcmRegisters;
  IMX_CCM_CS2CDR_REG Cs2cdrReg;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  Cs2cdrReg.AsUint32 = MmioRead32 ((UINTN)&pCcmRegisters->CS2CDR);

  Cs2cdrReg.ldb_di0_clk_sel = 0x0;
  Cs2cdrReg.ldb_di1_clk_sel = 0x0;

  MmioWrite32 ((UINTN)&pCcmRegisters->CS2CDR, Cs2cdrReg.AsUint32);
}
#endif

/**
    Configure PLL 5 clock rate to the desired clock rate
**/
VOID
ImxSetClockRatePLL5 (
  IN  UINT32                                  ClockRate,
  IN  IMX_CCM_PLL_VIDEO_CTRL_POST_DIV_SELECT  PostDivSelect
  )
{
  volatile IMX_CCM_ANALOG_REGISTERS   *pCcmAnalogRegisters;
  UINT32                              Counter;
  UINT32                              Denom;
  UINT32                              DivSelect;
  UINT32                              Numerator;
  IMX_CCM_PLL_VIDEO_CTRL_REG          PllVideoCtrlClearReg;
  IMX_CCM_PLL_VIDEO_CTRL_REG          PllVideoCtrlReg;
  IMX_CCM_PLL_VIDEO_CTRL_REG          PllVideoCtrlSetReg;

  // Use clock rate as Denom for simple fractional calculation
  Denom = IMX_REF_CLK_24M_FREQ;
  DivSelect = ClockRate / IMX_REF_CLK_24M_FREQ;
  Numerator = ClockRate % IMX_REF_CLK_24M_FREQ;
  pCcmAnalogRegisters = (IMX_CCM_ANALOG_REGISTERS *)IMX_CCM_ANALOG_BASE;
  PllVideoCtrlReg.AsUint32 = MmioRead32 ((UINTN)&pCcmAnalogRegisters->PLL_VIDEO);

  ASSERT (Numerator < Denom);
  ASSERT ((DivSelect >= 27) && (DivSelect <= 54));

  // PLL output frequency = Fref * (DIV_SELECT + NUM / DENOM)
  PllVideoCtrlReg.DIV_SELECT = DivSelect;
  PllVideoCtrlReg.POST_DIV_SELECT = PostDivSelect;

  MmioWrite32 (
    (UINTN)&pCcmAnalogRegisters->PLL_VIDEO, PllVideoCtrlReg.AsUint32);
  MmioWrite32 (
    (UINTN)&pCcmAnalogRegisters->PLL_VIDEO_NUM, Numerator);
  MmioWrite32 (
    (UINTN)&pCcmAnalogRegisters->PLL_VIDEO_DENOM, Denom);

  PllVideoCtrlReg.AsUint32 = MmioRead32 (
                               (UINTN)&pCcmAnalogRegisters->PLL_VIDEO);

  // Check to see if pll is locked, if not attempt to enable it
  if (PllVideoCtrlReg.LOCK == 0) {
    PllVideoCtrlClearReg.AsUint32 = 0;
    PllVideoCtrlClearReg.POWERDOWN = 1;
    MmioWrite32 (
      (UINTN)&pCcmAnalogRegisters->PLL_VIDEO_CLR,
      PllVideoCtrlClearReg.AsUint32);
    PllVideoCtrlReg.AsUint32 = MmioRead32 ((UINTN)&pCcmAnalogRegisters->PLL_VIDEO);

    PllVideoCtrlSetReg.AsUint32 = 0;
    PllVideoCtrlSetReg.ENABLE = 1;
    MmioWrite32 (
      (UINTN)&pCcmAnalogRegisters->PLL_VIDEO_SET,
      PllVideoCtrlSetReg.AsUint32);
    PllVideoCtrlReg.AsUint32 = MmioRead32 ((UINTN)&pCcmAnalogRegisters->PLL_VIDEO);

    PllVideoCtrlClearReg.AsUint32 = 0;
    PllVideoCtrlClearReg.BYPASS = 1;
    MmioWrite32 (
      (UINTN)&pCcmAnalogRegisters->PLL_VIDEO_CLR,
      PllVideoCtrlClearReg.AsUint32);
    PllVideoCtrlReg.AsUint32 = MmioRead32 ((UINTN)&pCcmAnalogRegisters->PLL_VIDEO);

    for (Counter = 0; Counter < 10000; Counter++) {
      PllVideoCtrlReg.AsUint32 = MmioRead32 ((UINTN)&pCcmAnalogRegisters->PLL_VIDEO);
      if (PllVideoCtrlReg.LOCK == 1) {
        DEBUG ((DEBUG_VERBOSE, "%a: PLL5 Video locked.\n", __FUNCTION__));
        break;
      }
    }
    if (PllVideoCtrlReg.LOCK == 0) {
      DEBUG ((DEBUG_ERROR, "%a: PLL5 Video not locked.\n", __FUNCTION__));
    }
  }
}

EFI_STATUS
ImxpGetPll2PfdClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  IN      IMX_PLL_PFD           PfdIndex,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_ANALOG_REGISTERS *pCcmAnalogRegisters;
  IMX_CLOCK_INFO                    ParentInfo;
  IMX_CCM_PFD_528_REG               Pfd528Reg;
  UINT32                            PfdFrac;
  EFI_STATUS                        Status;

  pCcmAnalogRegisters = (IMX_CCM_ANALOG_REGISTERS *) IMX_CCM_ANALOG_BASE;
  Pfd528Reg.AsUint32 = MmioRead32 ((UINTN) &pCcmAnalogRegisters->PFD_528);
  switch (PfdIndex) {
  case IMX_PLL_PFD0:
    PfdFrac = Pfd528Reg.PFD0_FRAC;
    break;
  case IMX_PLL_PFD1:
    PfdFrac = Pfd528Reg.PFD1_FRAC;
    break;
  case IMX_PLL_PFD2:
    PfdFrac = Pfd528Reg.PFD2_FRAC;
    break;
  default:
    ASSERT (FALSE);
    return EFI_INVALID_PARAMETER;
  }

  Status = ImxpGetClockInfo (Cache, IMX_PLL2_MAIN_CLK, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // The resulting frequency shall be 528*18/PFDn_FRAC
  // where PFD0_FRAC is in the range 12-35.
  ASSERT ((PfdFrac >= 12) && (PfdFrac <= 35));
  ClockInfo->Frequency = (UINT32) ((UINT64) ParentInfo.Frequency * 18 / PfdFrac);
  ClockInfo->Parent = IMX_PLL2_MAIN_CLK;

  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetAxiClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCDR_REG           CbcdrReg;
  IMX_CLK                     Parent;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CbcdrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCDR);

  if (CbcdrReg.axi_sel == IMX_CCM_AXI_SEL_PERIPH_CLK) {
    Parent = IMX_PERIPH_CLK;
  } else {
    ASSERT (CbcdrReg.axi_sel == IMX_CCM_AXI_SEL_AXI_ALT);
    Parent = IMX_AXI_ALT;
  }

  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  ClockInfo->Frequency = ParentInfo.Frequency / (1 + CbcdrReg.axi_podf);
  ClockInfo->Parent = Parent;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetGpu2dCoreClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCMR_REG           CbcmrReg;
  IMX_CLK                     Parent;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CbcmrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCMR);
  switch (CbcmrReg.gpu2d_core_clk_sel) {
  case IMX_CCM_GPU2D_CORE_CLK_SEL_AXI:
    Parent = IMX_AXI_CLK_ROOT;
    break;
  case IMX_CCM_GPU2D_CORE_CLK_SEL_PLL3_SW:
    Parent = IMX_PLL3_SW_CLK;
    break;
  case IMX_CCM_GPU2D_CORE_CLK_SEL_PLL2_PFD0:
    Parent = IMX_PLL2_PFD0;
    break;
  case IMX_CCM_GPU2D_CORE_CLK_SEL_PLL2_PFD2:
    Parent = IMX_PLL2_PFD2;
    break;
  default:
    ASSERT (FALSE);
    return EFI_INVALID_PARAMETER;
  }

  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  ClockInfo->Frequency = ParentInfo.Frequency / (1 + CbcmrReg.gpu2d_core_clk_podf);
  ClockInfo->Parent = Parent;
  if (ClockInfo->Frequency > IMX_GPU2D_CORE_CLK_MAX) {
    DEBUG ((
            DEBUG_WARN,
            "%a: GPU2D_CORE_CLK exceeds maximum. (Value = %d, Max = %d)\n",
            __FUNCTION__,
            ClockInfo->Frequency,
            IMX_GPU2D_CORE_CLK_MAX));
  }
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetGpu3dCoreClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCMR_REG           CbcmrReg;
  IMX_CLK                     Parent;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CbcmrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCMR);
  switch (CbcmrReg.gpu3d_core_clk_sel) {
  case IMX_CCM_GPU3D_CORE_CLK_SEL_MMDC_CH0_AXI:
    Parent = IMX_MMDC_CH0_CLK_ROOT;
    break;
  case IMX_CCM_GPU3D_CORE_CLK_SEL_PLL3_SW:
    Parent = IMX_PLL3_SW_CLK;
    break;
  case IMX_CCM_GPU3D_CORE_CLK_SEL_PLL2_PFD1:
    Parent = IMX_PLL2_PFD1;
    break;
  case IMX_CCM_GPU3D_CORE_CLK_SEL_PLL2_PFD2:
    Parent = IMX_PLL2_PFD2;
    break;
  default:
    ASSERT (FALSE);
    return EFI_UNSUPPORTED;
  }

  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  ClockInfo->Frequency = ParentInfo.Frequency / (1 + CbcmrReg.gpu3d_core_podf);
  ClockInfo->Parent = Parent;
  if (ClockInfo->Frequency > IMX_GPU3D_CORE_CLK_MAX) {
    DEBUG ((
            DEBUG_WARN,
            "%a: GPU3D_CORE_CLK exceeds maximum. (Value = %d, Max = %d)\n",
            __FUNCTION__,
            ClockInfo->Frequency,
            IMX_GPU3D_CORE_CLK_MAX));
  }
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetGpu3dShaderClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCMR_REG           CbcmrReg;
  IMX_CLK                     Parent;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CbcmrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCMR);
  switch (CbcmrReg.gpu3d_shader_clk_sel) {
  case IMX_CCM_GPU3D_SHADER_CLK_SEL_MMDC_CH0_AXI:
    Parent = IMX_MMDC_CH0_CLK_ROOT;
    break;
  case IMX_CCM_GPU3D_SHADER_CLK_SEL_PLL3_SW:
    Parent = IMX_PLL3_SW_CLK;
    break;
  case IMX_CCM_GPU3D_SHADER_CLK_SEL_PLL2_PFD1:
    Parent = IMX_PLL2_PFD1;
    break;
  case IMX_CCM_GPU3D_SHADER_CLK_SEL_PLL3_PFD0:
    Parent = IMX_PLL3_PFD0;
    break;
  default:
    ASSERT (FALSE);
    return EFI_INVALID_PARAMETER;
  }

  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  ClockInfo->Frequency = ParentInfo.Frequency / (1 + CbcmrReg.gpu3d_shader_podf);
  ClockInfo->Parent = Parent;

#if defined(CPU_IMX6D) || defined(CPU_IMX6Q) || defined(CPU_IMX6DP) || defined(CPU_IMX6QP)
  if (ClockInfo->Frequency > IMX_GPU3D_SHADER_CLK_MAX) {
    DEBUG ((
            DEBUG_WARN,
            "%a: GPU3D_SHADER_CLK exceeds maximum. (Value = %d, Max = %d)",
            __FUNCTION__,
            ClockInfo->Frequency,
            IMX_GPU3D_SHADER_CLK_MAX));
  }
#endif

  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetPeriphClk2Info (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS *pCcmRegisters;
  IMX_CCM_CBCDR_REG CbcdrReg;
  IMX_CCM_CBCMR_REG CbcmrReg;
  IMX_CLK Parent;
  IMX_CLOCK_INFO ParentInfo;
  EFI_STATUS Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CbcmrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCMR);
  switch (CbcmrReg.periph_clk2_sel) {
  case IMX_CCM_PERIPH_CLK2_SEL_PLL3_SW_CLK:
    Parent = IMX_PLL3_SW_CLK;
    break;
  case IMX_CCM_PERIPH_CLK2_SEL_OSC_CLK:
    Parent = IMX_OSC_CLK;
    break;
  case IMX_CCM_PERIPH_CLK2_SEL_PLL2:
    Parent = IMX_PLL2_MAIN_CLK;
    break;
  default:
    ASSERT (FALSE);
    return EFI_INVALID_PARAMETER;
  }

  CbcdrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCDR);
  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  ClockInfo->Frequency = ParentInfo.Frequency / (1 + CbcdrReg.periph_clk2_podf);
  ClockInfo->Parent = Parent;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetPeriphClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCDR_REG           CbcdrReg;
  IMX_CLK                     Parent;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CbcdrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCDR);

  // NOTE: periph_clk_sel is OR'd with PLL_bypass_en2 (from jtag) to
  //       produce the input value to the MUX. We assume PLL_bypass_en2 is 0.
  if (CbcdrReg.periph_clk_sel == 0) {
    Parent = IMX_PRE_PERIPH_CLK;
  } else {
    ASSERT (CbcdrReg.periph_clk_sel == 1);
    Parent = IMX_PERIPH_CLK2;
  }

  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  ClockInfo->Frequency = ParentInfo.Frequency / (1 + CbcdrReg.mmdc_ch0_axi_podf);
  ClockInfo->Parent = Parent;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetMmdcCh0ClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCDR_REG           CbcdrReg;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  Status = ImxpGetClockInfo (Cache, IMX_PERIPH_CLK, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  CbcdrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCDR);
  ClockInfo->Frequency = ParentInfo.Frequency / (1 + CbcdrReg.mmdc_ch0_axi_podf);
  ClockInfo->Parent = IMX_PERIPH_CLK;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetGpu2dAxiClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCMR_REG           CbcmrReg;
  IMX_CLK                     Parent;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CbcmrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCMR);
  if (CbcmrReg.gpu2d_axi_clk_sel == IMX_CCM_GPU2D_AXI_CLK_SEL_AXI) {
    Parent = IMX_AXI_CLK_ROOT;
  } else {
    ASSERT (CbcmrReg.gpu2d_axi_clk_sel == IMX_CCM_GPU2D_AXI_CLK_SEL_AHB);
    Parent = IMX_AHB_CLK_ROOT;
  }

  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  ClockInfo->Frequency = ParentInfo.Frequency;
  ClockInfo->Parent = Parent;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetGpu3dAxiClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCMR_REG           CbcmrReg;
  IMX_CLK                     Parent;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CbcmrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCMR);
  if (CbcmrReg.gpu3d_axi_clk_sel == IMX_CCM_GPU3D_AXI_CLK_SEL_AXI) {
    Parent = IMX_AXI_CLK_ROOT;
  } else {
    ASSERT (CbcmrReg.gpu3d_axi_clk_sel == IMX_CCM_GPU3D_AXI_CLK_SEL_AHB);
    Parent = IMX_AHB_CLK_ROOT;
  }

  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  ClockInfo->Frequency = ParentInfo.Frequency;
  ClockInfo->Parent = Parent;
  return EFI_SUCCESS;
}

VOID
ImxEnableGpuVpuPowerDomain (
  VOID
  )
{
  volatile IMX_CCM_ANALOG_REGISTERS   *pAnalogRegisters;
  volatile IMX_GPC_REGISTERS          *pGpcRegisters;
  volatile IMX_GPC_PGC_REGISTERS      *pGpuPgcRegisters;
  IMX_GPC_CNTR_REG                    GpcCntrReg;
  IMX_PMU_REG_CORE_REG                PmuCoreReg;
  IMX_GPC_PGC_PUPSCR_REG              PupscrReg;

  pAnalogRegisters = (IMX_CCM_ANALOG_REGISTERS *) IMX_CCM_ANALOG_BASE;
  pGpcRegisters = (IMX_GPC_REGISTERS *) IMX_GPC_BASE;
  pGpuPgcRegisters = &pGpcRegisters->PGC_GPU;

  // Configure GPC/PGC PUPSCR Register SW2ISO bits
  PupscrReg.AsUint32 = MmioRead32 ((UINTN) &pGpuPgcRegisters->PUPSCR);
  PupscrReg.SW = IMX_GPC_PGC_PUPSCR_SW_DEFAULT;
  PupscrReg.SW2ISO = IMX_GPC_PGC_PUPSCR_SW2ISO_DEFAULT;
  MmioWrite32 ((UINTN) &pGpuPgcRegisters->PUPSCR, PupscrReg.AsUint32);

  // Turn on LDO_PU to 1.250V
  PmuCoreReg.AsUint32 = 0;
  PmuCoreReg.REG1_TARG = 0x1f;
  MmioWrite32 ((UINTN) &pAnalogRegisters->PMU_REG_CORE_CLR, PmuCoreReg.AsUint32);
  PmuCoreReg.REG1_TARG = 22;
  MmioWrite32 ((UINTN) &pAnalogRegisters->PMU_REG_CORE_SET, PmuCoreReg.AsUint32);
  MicroSecondDelay (100);

  // Assert power up request
  GpcCntrReg.AsUint32 = MmioRead32 ((UINTN) &pGpcRegisters->CNTR);
  GpcCntrReg.gpu_vpu_pdn_req = 0;
  GpcCntrReg.gpu_vpu_pup_req = 1;
  MmioWrite32 ((UINTN) &pGpcRegisters->CNTR, GpcCntrReg.AsUint32);

  // Wait for power up request to complete
  do {
    GpcCntrReg.AsUint32 = MmioRead32 ((UINTN) &pGpcRegisters->CNTR);
  } while (GpcCntrReg.gpu_vpu_pup_req != 0);
}

VOID
ImxDisableGpuVpuPowerDomain (
  VOID
  )
{
  volatile IMX_GPC_REGISTERS      *pGpcRegisters;
  volatile IMX_GPC_PGC_REGISTERS  *pGpuPgcRegisters;
  IMX_GPC_PGC_PGCR_REG            CtrlReg;
  IMX_GPC_CNTR_REG                GpcCntrReg;
  IMX_GPC_PGC_PDNSCR_REG          PdnscrReg;

  pGpcRegisters = (IMX_GPC_REGISTERS *) IMX_GPC_BASE;
  pGpuPgcRegisters = &pGpcRegisters->PGC_GPU;

  // Configure GPC/PGC PDNSCR Register ISO bits
  PdnscrReg.AsUint32 = MmioRead32 ((UINTN) &pGpuPgcRegisters->PDNSCR);
  PdnscrReg.ISO = IMX_GPC_PGC_PDNSCR_ISO_DEFAULT;
  PdnscrReg.ISO2SW = IMX_GPC_PGC_PDNSCR_ISO2SW_DEFAULT;
  MmioWrite32 ((UINTN) &pGpuPgcRegisters->PDNSCR, PdnscrReg.AsUint32);

  // Configure GPC/PGC CTRL[PCR] bit to allow power down of the blocks
  CtrlReg.AsUint32 = MmioRead32 ((UINTN) &pGpuPgcRegisters->CTRL);
  CtrlReg.PCR = 1;    // Enable powering down of the blocks
  MmioWrite32 ((UINTN) &pGpuPgcRegisters->CTRL, CtrlReg.AsUint32);

  // Assert power down request
  GpcCntrReg.AsUint32 = MmioRead32 ((UINTN) &pGpcRegisters->CNTR);
  GpcCntrReg.gpu_vpu_pdn_req = 1;
  GpcCntrReg.gpu_vpu_pup_req = 0;
  MmioWrite32 ((UINTN) &pGpcRegisters->CNTR, GpcCntrReg.AsUint32);

  // Wait for power down request to complete
  do {
    GpcCntrReg.AsUint32 = MmioRead32 ((UINTN) &pGpcRegisters->CNTR);
  } while (GpcCntrReg.gpu_vpu_pdn_req != 0);
}

EFI_STATUS
ImxpGetClockInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  IN      IMX_CLK               ClockId,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  UINTN       CacheValidBits;
  EFI_STATUS  Status;

  ASSERT (ClockId < ARRAYSIZE (Cache->Table));

  // First try to satisfy from cache
  CacheValidBits = Cache->Valid[ClockId / _BITS_PER_UINTN];
  if (CacheValidBits & (1 << (ClockId % _BITS_PER_UINTN))) {
    *ClockInfo = Cache->Table[ClockId];
    return EFI_SUCCESS;
  }

  switch (ClockId) {
  case IMX_OSC_CLK:
    ImxpGetOsc24ClkInfo (ClockInfo);
    Status = EFI_SUCCESS;
    break;
  case IMX_PLL1_MAIN_CLK:
    Status = ImxpGetPll1MainClkInfo (Cache, ClockInfo);
    break;
  case IMX_PLL2_MAIN_CLK:
    Status = ImxpGetPll2MainClkInfo (Cache, ClockInfo);
    break;
  case IMX_PLL2_PFD0:
    Status = ImxpGetPll2PfdClkInfo (Cache, IMX_PLL_PFD0, ClockInfo);
    break;
  case IMX_PLL2_PFD1:
    Status = ImxpGetPll2PfdClkInfo (Cache, IMX_PLL_PFD1, ClockInfo);
    break;
  case IMX_PLL2_PFD2:
    Status = ImxpGetPll2PfdClkInfo (Cache, IMX_PLL_PFD2, ClockInfo);
    break;
  case IMX_PLL3_MAIN_CLK:
    Status = ImxpGetPll3MainClkInfo (Cache, ClockInfo);
    break;
  case IMX_PLL3_PFD0:
    Status = ImxpGetPll3PfdClkInfo (Cache, IMX_PLL_PFD0, ClockInfo);
    break;
  case IMX_PLL3_PFD1:
    Status = ImxpGetPll3PfdClkInfo (Cache, IMX_PLL_PFD1, ClockInfo);
    break;
  case IMX_PLL3_PFD2:
    Status = ImxpGetPll3PfdClkInfo (Cache, IMX_PLL_PFD2, ClockInfo);
    break;
  case IMX_PLL3_PFD3:
    Status = ImxpGetPll3PfdClkInfo (Cache, IMX_PLL_PFD3, ClockInfo);
    break;
  case IMX_PLL3_SW_CLK:
    Status = ImxpGetPll3SwClkInfo (Cache, ClockInfo);
    break;
  case IMX_AXI_CLK_ROOT:
    Status = ImxpGetAxiClkRootInfo (Cache, ClockInfo);
    break;
  case IMX_PERIPH_CLK2:
    Status = ImxpGetPeriphClk2Info (Cache, ClockInfo);
    break;
  case IMX_PERIPH_CLK:
    Status = ImxpGetPeriphClkInfo (Cache, ClockInfo);
    break;
  case IMX_PRE_PERIPH_CLK:
    Status = ImxpGetPrePeriphClkInfo (Cache, ClockInfo);
    break;
  case IMX_ARM_CLK_ROOT:
    Status = ImxpGetArmClkRootInfo (Cache, ClockInfo);
    break;
  case IMX_MMDC_CH0_CLK_ROOT:
    Status = ImxpGetMmdcCh0ClkRootInfo (Cache, ClockInfo);
    break;
  case IMX_AHB_CLK_ROOT:
    Status = ImxpGetAhbClkRootInfo (Cache, ClockInfo);
    break;
  case IMX_IPG_CLK_ROOT:
    Status = ImxpGetIpgClkRootInfo (Cache, ClockInfo);
    break;
  case IMX_GPU2D_AXI_CLK_ROOT:
    Status = ImxpGetGpu2dAxiClkRootInfo (Cache, ClockInfo);
    break;
  case IMX_GPU3D_AXI_CLK_ROOT:
    Status = ImxpGetGpu3dAxiClkRootInfo (Cache, ClockInfo);
    break;
  case IMX_GPU2D_CORE_CLK_ROOT:
    Status = ImxpGetGpu2dCoreClkInfo (Cache, ClockInfo);
    break;
  case IMX_GPU3D_CORE_CLK_ROOT:
    Status = ImxpGetGpu3dCoreClkInfo (Cache, ClockInfo);
    break;
  case IMX_GPU3D_SHADER_CLK_ROOT:
    Status = ImxpGetGpu3dShaderClkInfo (Cache, ClockInfo);
    break;
  default:
    return EFI_UNSUPPORTED;
  }

  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Update the cache
  Cache->Table[ClockId] = *ClockInfo;
  Cache->Valid[ClockId / _BITS_PER_UINTN] |= (1 << (ClockId % _BITS_PER_UINTN));
  return EFI_SUCCESS;
}

#if defined(CPU_IMX6D) || defined(CPU_IMX6Q) || defined(CPU_IMX6DP) || defined(CPU_IMX6QP)
/**
    Setup the clock tree for LDB0/1
**/
EFI_STATUS
ImxClkPwrIpuLDBxEnable (
  VOID
  )
{
  ImxClkPwrSetClockGate (IMX_LDB_DI0_CLK_ENABLE, IMX_CCM_CCGR_OFF);
  ImxClkPwrSetClockGate (IMX_LDB_DI1_CLK_ENABLE, IMX_CCM_CCGR_OFF);
  ImxCcmConfigureIPULDBxClockTree();
  ImxClkPwrSetClockGate (IMX_LDB_DI0_CLK_ENABLE, IMX_CCM_CCGR_ON);
  ImxClkPwrSetClockGate (IMX_LDB_DI1_CLK_ENABLE, IMX_CCM_CCGR_ON);

  return EFI_SUCCESS;
}
#endif

/**
    Configure PLL5 to the desired clock rate for all Display Interface (DI).
    Currently only support one display to IPU1 DI0.
**/
EFI_STATUS
ImxSetPll5ReferenceRate (
  IN  UINT32  ClockRate
  )
{
  volatile IMX_CCM_REGISTERS              *pCcmRegisters;
  IMX_CCM_CHSCCDR_REG                     ChscddrReg;
  UINT32                                  DxPodfDivider;
  BOOLEAN                                 FoundConfig;
  IMX_CCM_PLL_VIDEO_CTRL_POST_DIV_SELECT  postDivSelect[3];
  UINT32                                  PostDivSelectCount;
  UINT32                                  PostDivSelectValue[3];
  UINT32                                  TargetFreq;

  FoundConfig = FALSE;
  PostDivSelectValue[0] = 1;
  PostDivSelectValue[1] = 2;
  PostDivSelectValue[2] = 4;
  postDivSelect[0] = IMX_POST_DIV_SELECT_DIVIDE_1;
  postDivSelect[1] = IMX_POST_DIV_SELECT_DIVIDE_2;
  postDivSelect[2] = IMX_POST_DIV_SELECT_DIVIDE_4;

  for (PostDivSelectCount = 0;
       PostDivSelectCount < ARRAYSIZE (PostDivSelectValue);
       ++PostDivSelectCount) {
    for (DxPodfDivider = 1; DxPodfDivider < 9; ++DxPodfDivider) {
      TargetFreq = DxPodfDivider * ClockRate * PostDivSelectValue[PostDivSelectCount];
      // The valid range for PPL loop divider is 27-54 so target freq needs
      //   to fit within the valid range.
      if ((TargetFreq >= PLL5_MIN_FREQ) &&
          (TargetFreq <= PLL5_MAX_FREQ)) {
        FoundConfig = TRUE;
        break;
      }
    }

    if (FoundConfig == TRUE) {
      break;
    }
  }

  if (FoundConfig == FALSE) {
    DEBUG ((DEBUG_ERROR, "%a: ClockRate %d\n", __FUNCTION__, ClockRate));
    ASSERT (FALSE);
    return EFI_INVALID_PARAMETER;
  }
  DEBUG ((
          DEBUG_INFO,
          "%a: PLL 5 setting (%d) Target Freq %d Divider %d PostDiv %d\n",
          __FUNCTION__,
          ClockRate,
          TargetFreq,
          DxPodfDivider,
          PostDivSelectValue[PostDivSelectCount]
        ));

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  ChscddrReg.AsUint32 = MmioRead32 ((UINTN)&pCcmRegisters->CHSCCDR);
  ChscddrReg.ipu1_di0_podf = DxPodfDivider - 1;
  ChscddrReg.ipu1_di1_podf = DxPodfDivider - 1;
  MmioWrite32 ((UINTN)&pCcmRegisters->CHSCCDR, ChscddrReg.AsUint32);
  ImxSetClockRatePLL5 (TargetFreq, postDivSelect[PostDivSelectCount]);
  return EFI_SUCCESS;
}

EFI_STATUS
ImxClkPwrClkOut1Enable (
  IN  IMX_CLK   Clock,
  IN  UINT32    Divider
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CCOSR_REG           CcosrReg;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  if ((Divider < 1) || (Divider > 8)) {
    return EFI_INVALID_PARAMETER;
  }

  CcosrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CCOSR);
  switch (Clock) {
  case IMX_OSC_CLK:
    CcosrReg.CLKO2_SEL = IMX_CCM_CLKO2_SEL_OSC_CLK;
    CcosrReg.CLKO2_DIV = Divider - 1;
    CcosrReg.CLKO2_EN = 1;
    CcosrReg.CLK_OUT_SEL = IMX_CCM_CLK_OUT_SEL_CCM_CLKO2;
    break;

  case IMX_PLL2_MAIN_CLK:
    CcosrReg.CLKO1_SEL = IMX_CCM_CLKO1_SEL_PLL2_MAIN_CLK_2;
    CcosrReg.CLKO1_DIV = Divider - 1;
    CcosrReg.CLKO1_EN = 1;
    CcosrReg.CLK_OUT_SEL = IMX_CCM_CLK_OUT_SEL_CCM_CLKO1;
    break;

  case IMX_AXI_CLK_ROOT:
    CcosrReg.CLKO1_SEL = IMX_CCM_CLKO1_SEL_AXI_CLK_ROOT;
    CcosrReg.CLKO1_DIV = Divider - 1;
    CcosrReg.CLKO1_EN = 1;
    CcosrReg.CLK_OUT_SEL = IMX_CCM_CLK_OUT_SEL_CCM_CLKO1;
    break;

  case IMX_IPG_CLK_ROOT:
    CcosrReg.CLKO1_SEL = IMX_CCM_CLKO1_SEL_IPG_CLK_ROOT;
    CcosrReg.CLKO1_DIV = Divider - 1;
    CcosrReg.CLKO1_EN = 1;
    CcosrReg.CLK_OUT_SEL = IMX_CCM_CLK_OUT_SEL_CCM_CLKO1;
    break;

  case IMX_GPU2D_AXI_CLK_ROOT:
    CcosrReg.CLKO2_SEL = IMX_CCM_CLKO2_SEL_GPU2D_AXI_CLK_ROOT;
    CcosrReg.CLKO2_DIV = Divider - 1;
    CcosrReg.CLKO2_EN = 1;
    CcosrReg.CLK_OUT_SEL = IMX_CCM_CLK_OUT_SEL_CCM_CLKO2;
    break;

  case IMX_GPU3D_AXI_CLK_ROOT:
    CcosrReg.CLKO2_SEL = IMX_CCM_CLKO2_SEL_GPU3D_AXI_CLK_ROOT;
    CcosrReg.CLKO2_DIV = Divider - 1;
    CcosrReg.CLKO2_EN = 1;
    CcosrReg.CLK_OUT_SEL = IMX_CCM_CLK_OUT_SEL_CCM_CLKO2;
    break;

  case IMX_GPU2D_CORE_CLK_ROOT:
    CcosrReg.CLKO2_SEL = IMX_CCM_CLKO2_SEL_GPU2D_CORE_CLK_ROOT;
    CcosrReg.CLKO2_DIV = Divider - 1;
    CcosrReg.CLKO2_EN = 1;
    CcosrReg.CLK_OUT_SEL = IMX_CCM_CLK_OUT_SEL_CCM_CLKO2;
    break;

  case IMX_GPU3D_CORE_CLK_ROOT:
    CcosrReg.CLKO2_SEL = IMX_CCM_CLKO2_SEL_GPU3D_CORE_CLK_ROOT;
    CcosrReg.CLKO2_DIV = Divider - 1;
    CcosrReg.CLKO2_EN = 1;
    CcosrReg.CLK_OUT_SEL = IMX_CCM_CLK_OUT_SEL_CCM_CLKO2;
    break;

  case IMX_GPU3D_SHADER_CLK_ROOT:
    CcosrReg.CLKO2_SEL = IMX_CCM_CLKO2_SEL_GPU3D_SHADER_CLK_ROOT;
    CcosrReg.CLKO2_DIV = Divider - 1;
    CcosrReg.CLKO2_EN = 1;
    CcosrReg.CLK_OUT_SEL = IMX_CCM_CLK_OUT_SEL_CCM_CLKO2;
    break;

  case IMX_UART_CLK_ROOT:
    CcosrReg.CLKO2_SEL = IMX_CCM_CLKO2_SEL_UART_CLK_ROOT;
    CcosrReg.CLKO2_DIV = Divider - 1;
    CcosrReg.CLKO2_EN = 1;
    CcosrReg.CLK_OUT_SEL = IMX_CCM_CLK_OUT_SEL_CCM_CLKO2;
    break;

  default:
    return EFI_UNSUPPORTED;
  }

  MmioWrite32 ((UINTN) &pCcmRegisters->CCOSR, CcosrReg.AsUint32);
  return EFI_SUCCESS;
}

VOID
ImxClkPwrClkOut1Disable (
  VOID
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CCOSR_REG           CcosrReg;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CcosrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CCOSR);
  CcosrReg.CLKO1_EN = 0;
  CcosrReg.CLKO2_EN = 0;
  MmioWrite32 ((UINTN) &pCcmRegisters->CCOSR, CcosrReg.AsUint32);
}

EFI_STATUS
ImxClkPwrValidateClocks (
  VOID
  )
{
  IMX_CLOCK_INFO  ActualInfo;
  UINT32          i;
  BOOLEAN         Invalid;
  EFI_STATUS      Status;

  Invalid = FALSE;
  for (i = 0; i < ARRAYSIZE (ExpectedClocks); ++i) {
    DEBUG ((
            DEBUG_INFO,
            "%a: Validating clock %s. Expecting: Frequency = %d (%d Mhz), Parent = %s\n",
            __FUNCTION__,
            StringFromImxClk (ExpectedClocks[i].Clock),
            ExpectedClocks[i].Info.Frequency,
            ExpectedClocks[i].Info.Frequency / 1000000,
            StringFromImxClk (ExpectedClocks[i].Info.Parent)
          ));
    Status = ImxClkPwrGetClockInfo (ExpectedClocks[i].Clock, &ActualInfo);
    if (EFI_ERROR (Status)) {
      DEBUG ((
              DEBUG_ERROR,
              "%a: Failed to get clock info. (Clock = %s, Status = 0x%x)\n",
              __FUNCTION__,
              StringFromImxClk (ExpectedClocks[i].Clock),
              Status
            ));
      return Status;
    }
    if ((ActualInfo.Frequency != ExpectedClocks[i].Info.Frequency) ||
        (ActualInfo.Parent != ExpectedClocks[i].Info.Parent)) {
      DEBUG ((
              DEBUG_ERROR,
              "%a: Clock settings do not match expected! Clock = %s (Expected, Actual) "
              "Frequency: %d, %d. Parent: %s, %s\n",
              __FUNCTION__,
              StringFromImxClk (ExpectedClocks[i].Clock),
              ExpectedClocks[i].Info.Frequency,
              ActualInfo.Frequency,
              StringFromImxClk (ExpectedClocks[i].Info.Parent),
              StringFromImxClk (ActualInfo.Parent)
            ));
      Invalid = TRUE;
    }
  }

  return Invalid ? EFI_DEVICE_ERROR : EFI_SUCCESS;
}

VOID ImxClkPwrLcdClockDisable ()
{
    IMX_CCM_CCGR2_REG value32; 
    volatile IMX_CCM_REGISTERS *ccmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;

    value32.AsUint32 = MmioRead32((UINTN) &ccmRegisters->CCGR[2]);
    value32.lcd_clk_enable = IMX6ULL_CCM_CLOCK_OFF;
    MmioWrite32((UINTN) &ccmRegisters->CCGR[2], value32.AsUint32);
}

VOID ImxClkPwrLcdClockEnable ()
{
    IMX_CCM_CCGR2_REG value32; 
    volatile IMX_CCM_REGISTERS *ccmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;

    value32.AsUint32 = MmioRead32((UINTN) &ccmRegisters->CCGR[2]);
    value32.lcd_clk_enable = IMX6ULL_RUN_AND_WAIT; 
    MmioWrite32((UINTN) &ccmRegisters->CCGR[2], value32.AsUint32);
}

VOID IMXSetVideoPllClockRate(
  UINT32 TargetClockRate,
  UINT32 PreDividerLcdif1Val,
  UINT32 PostDividerLcdif1Val
  )
{
    IMX_CCM_CCGR3_REG ccgr3;
    UINT32 value32;
    IMX_CCM_PLL_VIDEO_CTRL_REG videoControl;
    volatile IMX_CCM_REGISTERS *ccmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
    volatile IMX_CCM_ANALOG_REGISTERS *analogRegisters = (IMX_CCM_ANALOG_REGISTERS *) IMX_CCM_ANALOG_BASE;

    // turn off LCD clocks
    ImxClkPwrLcdClockDisable();

    // gate the LCD pixel and AXI clocks CCGR3.CG5
    ccgr3.AsUint32 = MmioRead32((UINTN) &ccmRegisters->CCGR[3]);
    ccgr3.lcdif1_pix_clk_enable = IMX6ULL_CCM_CLOCK_OFF;
    MmioWrite32((UINTN) &ccmRegisters->CCGR[3], ccgr3.AsUint32);

    //
    // set the divider for the source clock to the video PLL to divide by 1 
    //
    MmioWrite32((UINTN) &analogRegisters->MISC0_CLR, 0x80000000);

    //
    // fire up the video PLL to the correct frequency
    // before division
    //

    videoControl.AsUint32 = 0;
    videoControl.POST_DIV_SELECT = 0x03;
    videoControl.BYPASS = 0x01;
    videoControl.POWERDOWN = 0x01;
    videoControl.DIV_SELECT = 0x7f;
    MmioWrite32((UINTN) &analogRegisters->PLL_VIDEO_CLR, videoControl.AsUint32);

    //
    // PLL output frequency = (Reference Freq) * (DIV_SELECT + NUM / DENOM)
    //
    // Use clock rate as denominator for simple fractional calculation.
    // This way we just need to figure out the target clock rate ratio
    // to the 24MHz reference.
    //
    {
        IMX_CCM_PLL_VIDEO_CTRL_REG pllVideoCtrlReg;
        UINT32 denom = IMX_REF_CLK_24M_FREQ;
        UINT32 divSelect = TargetClockRate / IMX_REF_CLK_24M_FREQ;
        UINT32 numerator = TargetClockRate % IMX_REF_CLK_24M_FREQ;

        pllVideoCtrlReg.AsUint32 = MmioRead32((UINTN) &analogRegisters->PLL_VIDEO);

        ASSERT (numerator < denom);
        ASSERT ((divSelect >= 27) && (divSelect <= 54));

        pllVideoCtrlReg.DIV_SELECT = divSelect;
        pllVideoCtrlReg.POST_DIV_SELECT = IMX_POST_DIV_SELECT_DIVIDE_2;

        DEBUG ((
            DEBUG_INFO,
            "PLL 5 divSelect (%d) numerator (%d) denom %d\n",
            divSelect,
            numerator,
            denom
            ));

        MmioWrite32((UINTN) &analogRegisters->PLL_VIDEO, pllVideoCtrlReg.AsUint32);
        MmioWrite32((UINTN) &analogRegisters->PLL_VIDEO_NUM, numerator);
        MmioWrite32((UINTN) &analogRegisters->PLL_VIDEO_DENOM, denom);
    }

    // wait for PLL to lock
    do {
        videoControl.AsUint32 = MmioRead32((UINTN) &analogRegisters->PLL_VIDEO);
    } while (!(videoControl.LOCK));

    //
    // select the video PLL in the LCDIF clock selector
    // and set the CDIF1_PRED value
    //
    value32 = MmioRead32((UINTN) &ccmRegisters->CSCDR2);
    // Clear LCDIF1_CLK_SEL, LCDIF1_PRED and LCDIF1_PRE_CLK_SEL
    value32 &= ~0x0003FE00;
    // Set the predivider value and derive clock from PLL5
    value32 |= ((PreDividerLcdif1Val - 1) << 12) | (2 << 15);
    MmioWrite32((UINTN) &ccmRegisters->CSCDR2, value32);

    //
    // set the post divider in CBCMR
    //
    value32 = MmioRead32((UINTN) &ccmRegisters->CBCMR);
    // Clear LCDIF1_PODF
    value32 &= ~0x03800000;
    value32 |= ((PostDividerLcdif1Val - 1) << 23);
    MmioWrite32((UINTN) &ccmRegisters->CBCMR, value32);

    // enable the PLL output
    videoControl.AsUint32 = 0;
    videoControl.ENABLE = 1;
    MmioWrite32((UINTN) &analogRegisters->PLL_VIDEO_SET, videoControl.AsUint32);

    // Ungate the LCD pixel clock
    ccgr3.AsUint32 = MmioRead32((UINTN) &ccmRegisters->CCGR[3]);
    ccgr3.lcdif1_pix_clk_enable = IMX6ULL_RUN_AND_WAIT;
    MmioWrite32((UINTN) &ccmRegisters->CCGR[3], ccgr3.AsUint32);

    // turn on LCD clocks
    ImxClkPwrLcdClockEnable();
}


EFI_STATUS
ImxSetLcdIfClockRate (
    UINT32 ClockRate
    )   
{
    BOOLEAN foundConfig = FALSE;
    UINT32 targetFreq;
    UINT32 preDivSelectCount;
    UINT32 postDivSelectCount;
    UINT32 preDividerLcdif1[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    UINT32 postDividerLcdif1[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };

    for (postDivSelectCount = 0;
        postDivSelectCount < ARRAYSIZE (postDividerLcdif1);
        ++postDivSelectCount) {

        for (preDivSelectCount = 0;
            preDivSelectCount < ARRAYSIZE (preDividerLcdif1);
            ++preDivSelectCount) {

            //
            // Need to verify the meaning of post div and test div values
            //
            targetFreq =
                ClockRate *
                preDividerLcdif1[preDivSelectCount] *
                postDividerLcdif1[postDivSelectCount] *
                1 *
		2; // IMX_POST_DIV_SEL_DIVIDE_2

            //
            // The valid range for PLL loop divider is 27-54 so we
            // need to target freq need to fit within the valid range.
            //
            if ((targetFreq >= PLL5_MIN_FREQ) &&
                (targetFreq <= PLL5_MAX_FREQ)) {
                foundConfig = TRUE;
                break;
            }   
        }

        if (foundConfig == TRUE) {
            break;
        }   
    }

    if (foundConfig == FALSE) {
        DEBUG((DEBUG_ERROR, "No valid configuration found for clock rate %d\n", ClockRate));
        ASSERT(FALSE);
        return EFI_INVALID_PARAMETER;
    }

    DEBUG ((
        DEBUG_INFO,
        "Video PLL setting (%d) Target Freq (%d) PreDiv %d PostDiv %d\n",
        ClockRate,
        targetFreq,
        preDividerLcdif1[preDivSelectCount],
        postDividerLcdif1[postDivSelectCount]
        ));

    IMXSetVideoPllClockRate(
        targetFreq,
        preDividerLcdif1[preDivSelectCount],
        postDividerLcdif1[postDivSelectCount]);

    return EFI_SUCCESS;
}

/**
  Reset/invalidate the clock tree cache.

  The clock tree cache must be invalidated whenever the clock tree is modified,
  e.g. when changing PLL configuration, clock mux, or divider.

**/
VOID
ImxpClkPwrCacheReset (
  VOID
  )
{
  SetMem (&mImxpClockPwrCache.Valid, sizeof (mImxpClockPwrCache.Valid), 0);
}

/**
  Configure clock gating for the specified clock signal.

  @param[in]  ClockGate   Specific clock signal to configure.
  @param[in]  State       State to set the clock signal to.
**/
VOID
ImxClkPwrSetClockGate (
  IN  IMX_CLK_GATE          ClockGate,
  IN  IMX_CLOCK_GATE_STATE  State
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  UINTN                       EndBit;
  IMX_CCGR_INDEX              Index;
  UINTN                       StartBit;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;

  // Extract register index
  Index = ImxpCcgrIndexFromClkGate (ClockGate);
  StartBit = Index.GateNumber * 2;
  EndBit = StartBit + 1;

  MmioBitFieldWrite32 (
    (UINTN) &pCcmRegisters->CCGR[Index.RegisterIndex],
    StartBit,
    EndBit,
    State);
}

/**
  Determine if gating TZASC1_IPG_MASTER_CLK should be skipped.
 **/
BOOLEAN
ImxClkPwrShouldSkipTZASC1 (
  VOID
  )
{
#if defined(CPU_IMX6D) || defined(CPU_IMX6Q) || defined(CPU_IMX6DP) || defined(CPU_IMX6QP)
  IMX_IOMUXC_GPR_REGISTERS  *IoMuxMmioBasePtr;
  UINTN                     IomuxGPR9;
#endif
  BOOLEAN                   Skip;

  Skip = FALSE;
#if defined(CPU_IMX6D) || defined(CPU_IMX6Q) || defined(CPU_IMX6DP) || defined(CPU_IMX6QP)
  IoMuxMmioBasePtr = (IMX_IOMUXC_GPR_REGISTERS *)IOMUXC_GPR_BASE_ADDRESS;

  IomuxGPR9 = MmioRead32 ((UINTN) &IoMuxMmioBasePtr->GPR9);
  if (IomuxGPR9 & IMX_IOMUXC_TZASC1_BYP) {
    // TZASC-1 is active.
    Skip = TRUE;
  }
#endif

  return Skip;
}

/**
  Determine if a clock gate should be skipped

  @param[in]  ClockGate   Specific clock signal to configure.
 **/
BOOLEAN
ImxClkPwrShouldSkipGate (
  IN  IMX_CLK_GATE  ClockGate
  )
{
  switch (ClockGate) {
  case IMX_IPSYNC_IP2APB_TZASC1_IPG_MASTER_CLK_ENABLE:
    return ImxClkPwrShouldSkipTZASC1 ();

  default:
    return FALSE;
  }
}

/**
  Set multiple clock signals to a given state.

  @param[in]  ClockGateList   Pointer to list of possible clock signals.
  @param[in]  ClockGateCount  Number of clock signals to gate.
  @param[in]  State           State to set the clock signal to.
**/
VOID
ImxClkPwrSetClockGates (
  CONST IMX_CLK_GATE *ClockGateList,
  UINTN ClockGateCount,
  IMX_CLOCK_GATE_STATE State
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  UINTN                       i;
  IMX_CCGR_INDEX              Index;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;

  // Read all CCGR registers to local copy
  UINT32 ccgrRegisters[ARRAYSIZE (pCcmRegisters->CCGR)];
  for (i = 0; i < ARRAYSIZE (ccgrRegisters); ++i) {
    ccgrRegisters[i] = MmioRead32 ((UINTN) &pCcmRegisters->CCGR[i]);
  }

  // Compute new CCGR register values
  for (i = 0; i < ClockGateCount; ++i) {
    if (ImxClkPwrShouldSkipGate (ClockGateList[i])) {
      continue;
    }

    Index = ImxpCcgrIndexFromClkGate (ClockGateList[i]);
    ccgrRegisters[Index.RegisterIndex] =
      (ccgrRegisters[Index.RegisterIndex] & ~(0x3 << (2 * Index.GateNumber))) |
      (State << (2 * Index.GateNumber));
  }

  // Write back to registers
  for (i = 0; i < ARRAYSIZE (ccgrRegisters); ++i) {
    MmioWrite32 ((UINTN) &pCcmRegisters->CCGR[i], ccgrRegisters[i]);
  }
}

/**
  Get the current clock gating setting for the specified clock gate.

  @param[in]  ClockGate     Specific clock signal to fetch clock gate info from.
**/
IMX_CLOCK_GATE_STATE
ImxClkPwrGetClockGate (
  IN  IMX_CLK_GATE  ClockGate
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  UINTN                       EndBit;
  IMX_CCGR_INDEX              Index;
  UINTN                       StartBit;
  UINT32                      Value;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  Index = ImxpCcgrIndexFromClkGate (ClockGate);
  StartBit = Index.GateNumber * 2;
  EndBit = StartBit + 1;

  Value = MmioBitFieldRead32 (
            (UINTN) &pCcmRegisters->CCGR[Index.RegisterIndex],
            StartBit,
            EndBit);

  if ((Value != IMX_CCM_CCGR_OFF) &&
      (Value != IMX_CCM_CCGR_ON_RUN) &&
      (Value != IMX_CCM_CCGR_ON)) {
    ASSERT (FALSE);
  }

  return (IMX_CLOCK_GATE_STATE) Value;
}

EFI_STATUS
ImxpGetPll3MainClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_ANALOG_REGISTERS   *pCcmAnalogRegisters;
  IMX_CLK                             Parent;
  IMX_CLOCK_INFO                      ParentInfo;
  IMX_CCM_ANALOG_PLL_USB1_REG         PllUsb1Reg;
  EFI_STATUS                          Status;

  pCcmAnalogRegisters = (IMX_CCM_ANALOG_REGISTERS *) IMX_CCM_ANALOG_BASE;
  PllUsb1Reg.AsUint32 = MmioRead32 ((UINTN)&pCcmAnalogRegisters->PLL_USB1);
  Parent = ImxpClkFromBypassClkSource (PllUsb1Reg.BYPASS_CLK_SRC);
  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (PllUsb1Reg.DIV_SELECT == 0) {
    ClockInfo->Frequency = ParentInfo.Frequency * 20;
  } else {
    ClockInfo->Frequency = ParentInfo.Frequency * 22;
  }

  ClockInfo->Parent = Parent;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetPll3PfdClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  IN      IMX_PLL_PFD           PfdIndex,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_ANALOG_REGISTERS   *pCcmAnalogRegisters;
  IMX_CLOCK_INFO                      ParentInfo;
  IMX_CCM_PFD_480_REG                 Pfd480Reg;
  UINT32                              PfdFrac;
  EFI_STATUS                          Status;

  pCcmAnalogRegisters = (IMX_CCM_ANALOG_REGISTERS *) IMX_CCM_ANALOG_BASE;
  Pfd480Reg.AsUint32 = MmioRead32 ((UINTN) &pCcmAnalogRegisters->PFD_480);
  switch (PfdIndex) {
  case IMX_PLL_PFD0:
    PfdFrac = Pfd480Reg.PFD0_FRAC;
    break;
  case IMX_PLL_PFD1:
    PfdFrac = Pfd480Reg.PFD1_FRAC;
    break;
  case IMX_PLL_PFD2:
    PfdFrac = Pfd480Reg.PFD2_FRAC;
    break;
  case IMX_PLL_PFD3:
    PfdFrac = Pfd480Reg.PFD3_FRAC;
    break;
  default:
    ASSERT (FALSE);
    return EFI_INVALID_PARAMETER;
  }

  Status = ImxpGetClockInfo (Cache, IMX_PLL3_MAIN_CLK, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // The resulting frequency shall be 480*18/PFDn_FRAC
  // where PFD0_FRAC is in the range 12-35.
  ASSERT ((PfdFrac >= 12) && (PfdFrac <= 35));
  ClockInfo->Frequency = (UINT32) ((UINT64) ParentInfo.Frequency * 18 / PfdFrac);
  ClockInfo->Parent = IMX_PLL3_MAIN_CLK;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetPll3SwClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CCSR_REG            CcsrReg;
  IMX_CLK                     Parent;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CcsrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CCSR);
  if (CcsrReg.pll3_sw_clk_sel == IMX_CCM_PLL3_SW_CLK_SEL_PLL3_MAIN_CLK) {
    Parent = IMX_PLL3_MAIN_CLK;
  } else {
    ASSERT (CcsrReg.pll3_sw_clk_sel == IMX_CCM_PLL3_SW_CLK_SEL_PLL3_BYPASS_CLK);
    ASSERT (!"Not implemented");
    return EFI_UNSUPPORTED;
  }

  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  ClockInfo->Frequency = ParentInfo.Frequency;
  ClockInfo->Parent = Parent;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetPll1MainClkInfo  (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_ANALOG_REGISTERS   *pCcmAnalogRegisters;
  IMX_CLK                             Parent;
  IMX_CLOCK_INFO                      ParentInfo;
  IMX_CCM_ANALOG_PLL_ARM_REG          PllArmReg;
  EFI_STATUS                          Status;

  pCcmAnalogRegisters = (IMX_CCM_ANALOG_REGISTERS *) IMX_CCM_ANALOG_BASE;
  PllArmReg.AsUint32 = MmioRead32 ((UINTN) &pCcmAnalogRegisters->PLL_ARM);
  Parent = ImxpClkFromBypassClkSource (PllArmReg.BYPASS_CLK_SRC);
  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (PllArmReg.BYPASS != 0) {
    ClockInfo->Frequency = ParentInfo.Frequency;
    ClockInfo->Parent = Parent;
    return EFI_SUCCESS;
  }

  ClockInfo->Frequency = (UINT32) ((UINT64) ParentInfo.Frequency * PllArmReg.DIV_SELECT / 2);
  ClockInfo->Parent = Parent;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetPll2MainClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_ANALOG_REGISTERS   *pCcmAnalogRegisters;
  IMX_CLK                             Parent;
  IMX_CLOCK_INFO                      ParentInfo;
  IMX_CCM_ANALOG_PLL_SYS_REG          PllSysReg;
  EFI_STATUS                          Status;

  pCcmAnalogRegisters = (IMX_CCM_ANALOG_REGISTERS *) IMX_CCM_ANALOG_BASE;
  PllSysReg.AsUint32 = MmioRead32 ((UINTN) &pCcmAnalogRegisters->PLL_SYS);
  // Determine the reference clock source
  Parent = ImxpClkFromBypassClkSource (PllSysReg.BYPASS_CLK_SRC);
  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (PllSysReg.BYPASS != 0) {
    ClockInfo->Frequency = ParentInfo.Frequency;
    ClockInfo->Parent = Parent;
    return EFI_SUCCESS;
  }

  if (PllSysReg.DIV_SELECT == 0) {
    ClockInfo->Frequency = ParentInfo.Frequency * 20;
  } else {
    ASSERT (PllSysReg.DIV_SELECT == 1);
    ClockInfo->Frequency = ParentInfo.Frequency * 22;
  }

  ClockInfo->Parent = Parent;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetArmClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CACRR_REG           CacrrReg;
  IMX_CLOCK_INFO              Pll1Info;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  Status = ImxpGetClockInfo (Cache, IMX_PLL1_MAIN_CLK, &Pll1Info);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  CacrrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CACRR);
  ClockInfo->Frequency = Pll1Info.Frequency / (1 + CacrrReg.arm_podf);
  ClockInfo->Parent = IMX_PLL1_MAIN_CLK;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetPrePeriphClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCMR_REG           CbcmrReg;
  IMX_CLK                     Parent;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  CbcmrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCMR);
  switch (CbcmrReg.pre_periph_clk_sel) {
  case IMX_CCM_PRE_PERIPH_CLK_SEL_PLL2:
    Parent = IMX_PLL2_MAIN_CLK;
    break;
  case IMX_CCM_PRE_PERIPH_CLK_SEL_PLL2_PFD2:
    Parent = IMX_PLL2_PFD2;
    break;
  case IMX_CCM_PRE_PERIPH_CLK_SEL_PLL2_PFD0:
    Parent = IMX_PLL2_PFD0;
    break;
  case IMX_CCM_PRE_PERIPH_CLK_SEL_PLL2_PFD2_DIV2:
    Parent = IMX_PLL2_PFD2;
    break;
  default:
    ASSERT (FALSE);
    return EFI_INVALID_PARAMETER;
  }

  Status = ImxpGetClockInfo (Cache, Parent, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (CbcmrReg.pre_periph_clk_sel == IMX_CCM_PRE_PERIPH_CLK_SEL_PLL2_PFD2_DIV2) {
    ClockInfo->Frequency = ParentInfo.Frequency / 2;
  } else {
    ClockInfo->Frequency = ParentInfo.Frequency;
  }

  ClockInfo->Parent = Parent;
  return EFI_SUCCESS;
}

VOID
ImxpGetOsc24ClkInfo (
  OUT IMX_CLOCK_INFO  *ClockInfo
  )
{
  ClockInfo->Frequency = IMX_REF_CLK_24M_FREQ;
  ClockInfo->Parent = IMX_CLK_NONE;
}

EFI_STATUS
ImxpGetAhbClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCDR_REG           CbcdrReg;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  Status = ImxpGetClockInfo (Cache, IMX_PERIPH_CLK, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  CbcdrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCDR);
  ClockInfo->Frequency = ParentInfo.Frequency / (1 + CbcdrReg.ahb_podf);
  ClockInfo->Parent = IMX_PERIPH_CLK;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxpGetIpgClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  )
{
  volatile IMX_CCM_REGISTERS  *pCcmRegisters;
  IMX_CCM_CBCDR_REG           CbcdrReg;
  IMX_CLOCK_INFO              ParentInfo;
  EFI_STATUS                  Status;

  pCcmRegisters = (IMX_CCM_REGISTERS *) IMX_CCM_BASE;
  Status = ImxpGetClockInfo (Cache, IMX_AHB_CLK_ROOT, &ParentInfo);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  CbcdrReg.AsUint32 = MmioRead32 ((UINTN) &pCcmRegisters->CBCDR);
  ClockInfo->Frequency = ParentInfo.Frequency / (1 + CbcdrReg.ipg_podf);
  ClockInfo->Parent = IMX_AHB_CLK_ROOT;
  return EFI_SUCCESS;
}

EFI_STATUS
ImxClkPwrGetClockInfo (
  IN  IMX_CLK         ClockId,
  OUT IMX_CLOCK_INFO  *ClockInfo
  )
{
  return ImxpGetClockInfo (&mImxpClockPwrCache, ClockId, ClockInfo);
}
