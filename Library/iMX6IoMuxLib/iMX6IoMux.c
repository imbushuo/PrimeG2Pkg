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

#include <Library/DebugLib.h>
#include <Library/IoLib.h>

#include <Device/imx6_ull.h>
#include <Device/iMX6IoMux.h>

// Muxing functions
VOID
ImxPadConfig (
  IN  IMX_PAD     Pad,
  IN  IMX_PADCFG  PadConfig
  )
{
  // Configure Mux Control
  MmioWrite32 (
    IMX_IOMUXC_BASE + IMX_IOMUX_PAD_MUX_OFFSET (Pad),
    _IMX_PADCFG_MUX_CTL (PadConfig));

  // Configure Select Input Control
  if (_IMX_PADCFG_SEL_INP (PadConfig) != 0) {
    DEBUG ((DEBUG_INFO, "Setting INPUT_SELECT %x value %x\n",
            _IMX_SEL_INP_REGISTER (_IMX_PADCFG_SEL_INP (PadConfig)),
            _IMX_SEL_INP_VALUE (_IMX_PADCFG_SEL_INP (PadConfig))));

    MmioWrite32 (
      _IMX_SEL_INP_REGISTER (_IMX_PADCFG_SEL_INP (PadConfig)),
      _IMX_SEL_INP_VALUE (_IMX_PADCFG_SEL_INP (PadConfig)));
  }

  // Configure Pad Control
  MmioWrite32 (
    IMX_IOMUXC_BASE + IMX_IOMUX_PAD_CTL_OFFSET (Pad),
    _IMX_PADCFG_PAD_CTL (PadConfig));
}

VOID
ImxPadDumpConfig (
  IN  CHAR8 *SignalFriendlyName,
  IN  IMX_PAD Pad
  )
{
  IMX_IOMUXC_MUX_CTL  MuxCtl;
  IMX_IOMUXC_PAD_CTL  PadCtl;

  MuxCtl.AsUint32 = MmioRead32 (
                      IMX_IOMUXC_BASE + IMX_IOMUX_PAD_MUX_OFFSET (Pad));

  DEBUG ((
           DEBUG_INIT,
           "- %a MUX_CTL(0x%p)=0x%08x: MUX_MODE:%d SION:%d | ",
           SignalFriendlyName,
           IMX_IOMUXC_BASE + IMX_IOMUX_PAD_MUX_OFFSET (Pad),
           MuxCtl.AsUint32,
           MuxCtl.Fields.MUX_MODE,
           MuxCtl.Fields.SION));

  PadCtl.AsUint32 = MmioRead32 (
                      IMX_IOMUXC_BASE + IMX_IOMUX_PAD_CTL_OFFSET (Pad));

  DEBUG ((
           DEBUG_INIT,
           "PAD_CTL(0x%p)=0x%08x: SRE:%d DSE:%d SPEED:%d ODE:%d PKE:%d PUE:%d PUS:%d HYS:%d\n",
           IMX_IOMUXC_BASE + IMX_IOMUX_PAD_CTL_OFFSET (Pad),
           PadCtl.AsUint32,
           PadCtl.Fields.SRE,
           PadCtl.Fields.DSE,
           PadCtl.Fields.SPEED,
           PadCtl.Fields.ODE,
           PadCtl.Fields.PKE,
           PadCtl.Fields.PUE,
           PadCtl.Fields.PUS,
           PadCtl.Fields.HYS));
}

// GPIO functions
VOID
ImxGpioDirection (
  IN  IMX_GPIO_BANK   Bank,
  IN  UINT32          IoNumber,
  IN  IMX_GPIO_DIR    Direction
  )
{
  volatile IMX_GPIO_REGISTERS   *gpioRegisters;

  ASSERT (IoNumber < 32);

  gpioRegisters = (IMX_GPIO_REGISTERS *) IMX_GPIO_BASE;
  if (Direction == IMX_GPIO_DIR_INPUT) {
    MmioAnd32 ((UINTN) &gpioRegisters->Banks[Bank - 1].GDIR, ~ (1 << IoNumber));
  } else {
    MmioOr32 ((UINTN) &gpioRegisters->Banks[Bank - 1].GDIR, 1 << IoNumber);
  }
}

VOID
ImxGpioWrite (
  IN  IMX_GPIO_BANK   Bank,
  IN  UINT32          IoNumber,
  IN  IMX_GPIO_VALUE  Value
  )
{
  volatile IMX_GPIO_REGISTERS   *gpioRegisters;

  ASSERT (IoNumber < 32);

  gpioRegisters = (IMX_GPIO_REGISTERS *) IMX_GPIO_BASE;
  if (Value == IMX_GPIO_LOW) {
    MmioAnd32 ((UINTN) &gpioRegisters->Banks[Bank - 1].DR, ~ (1 << IoNumber));
  } else {
    MmioOr32 ((UINTN) &gpioRegisters->Banks[Bank - 1].DR, 1 << IoNumber);
  }
}

IMX_GPIO_VALUE
ImxGpioRead (
  IN  IMX_GPIO_BANK   Bank,
  IN  UINT32          IoNumber
  )
{
  volatile IMX_GPIO_REGISTERS   *gpioRegisters;
  UINT32                        Mask;
  UINT32                        Psr;

  ASSERT (IoNumber < 32);

  gpioRegisters = (IMX_GPIO_REGISTERS *) IMX_GPIO_BASE;
  Mask = (1 << IoNumber);
  Psr = MmioRead32 ((UINTN) &gpioRegisters->Banks[Bank - 1].PSR);

  if (Psr & Mask) {
    return IMX_GPIO_HIGH;
  } else {
    return IMX_GPIO_LOW;
  }
}
