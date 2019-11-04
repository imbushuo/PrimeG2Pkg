//
// Copyright (C) Microsoft. All rights reserved
//
/** @file

  Copyright (c) 2008-2009, Apple Inc. All rights reserved.

  All rights reserved. This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <PiDxe.h>

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>

#include <Library/NonDiscoverableDeviceRegistrationLib.h>

#include <Device/imx6_ull.h>
#include <Device/iMX6IoMux.h>
#include <Device/iMX6UsbPhy.h>
#include <Device/iMX6ClkPwr.h>

#include <UBoot/ehci-ci.h>

// Prebaked pad configurations that include mux and drive settings where
// each enum named as IMX_<MODULE-NAME>_PADCFG contains configurations
// for pads used by that module

typedef enum {
    IMX_PAD_CFG_USB_OTG_PWR = _IMX_MAKE_PADCFG(
                              IMX_SRE_SLOW,
                              IMX_DSE_40_OHM,
                              IMX_SPEED_LOW,
                              IMX_ODE_DISABLE,
                              IMX_PKE_ENABLE,
                              IMX_PUE_PULL,
                              IMX_PUS_100K_OHM_PU,
                              IMX_HYS_ENABLED,
                              IMX_SION_DISABLED,
                              IMX_IOMUXC_GPIO_0_ALT2_USB_OTG),

    IMX_PAD_CFG_USB_OTG1_OC = _IMX_MAKE_PADCFG_INPSEL(
                              IMX_SRE_SLOW,
                              IMX_DSE_40_OHM,
                              IMX_SPEED_LOW,
                              IMX_ODE_DISABLE,
                              IMX_PKE_ENABLE,
                              IMX_PUE_PULL,
                              IMX_PUS_100K_OHM_PU,
                              IMX_HYS_ENABLED,
                              IMX_SION_DISABLED,
                              IMX_IOMUXC_GPIO_0_ALT2_USB_OTG,
                              IOMUXC_USB_OTG1_OC_SELECT_INPUT,
                              0),
    IMX_PAD_CFG_USB_OTG2_OC = _IMX_MAKE_PADCFG_INPSEL(
                              IMX_SRE_SLOW,
                              IMX_DSE_40_OHM,
                              IMX_SPEED_LOW,
                              IMX_ODE_DISABLE,
                              IMX_PKE_ENABLE,
                              IMX_PUE_PULL,
                              IMX_PUS_100K_OHM_PU,
                              IMX_HYS_ENABLED,
                              IMX_SION_DISABLED,
                              IMX_IOMUXC_GPIO_0_ALT2_USB_OTG,
                              IOMUXC_USB_OTG2_OC_SELECT_INPUT,
                              0),

    IMX_PAD_CFG_USB_OTG1_ID = _IMX_MAKE_PADCFG_INPSEL(
                              IMX_SRE_FAST,
                              IMX_DSE_90_OHM,
                              IMX_SPEED_LOW,
                              IMX_ODE_DISABLE,
                              IMX_PKE_ENABLE,
                              IMX_PUE_PULL,
                              IMX_PUS_100K_OHM_PD,
                              IMX_HYS_ENABLED,
                              IMX_SION_DISABLED,
                              IMX_IOMUXC_GPIO_0_ALT2_USB_OTG,
                              IOMUXC_USB_OTG1_ID_SELECT_INPUT,
                              0),

} IMX_EHCI_PADCFG;

#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

EFI_STATUS
EFIAPI
PciEmulationEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  static const IMX_CLK_GATE gatesToTurnOn[] = {
    IMX_USBOH3_CLK_ENABLE,
  };

  struct usb_ehci *ehci = (struct usb_ehci *) (EFI_PHYSICAL_ADDRESS) FixedPcdGet32(PcdEHCIBase);
  UINT32 portsc;
  UINT32 usbnc_usb_ctrl;

  ImxClkPwrSetClockGates(
    gatesToTurnOn,
    sizeof(gatesToTurnOn) / sizeof(gatesToTurnOn[0]),
    IMX_CLOCK_GATE_STATE_ON);
  MicroSecondDelay(1);

  portsc = MmioRead32((UINTN) &ehci->portsc);
  if (portsc & PORT_PTS_PHCD) {
		DEBUG((EFI_D_INFO, "suspended: portsc %x, enabled it.\n", portsc));
    MmioWrite32((UINTN) &ehci->portsc, portsc & ~PORT_PTS_PHCD);
	}

  /* Do board specific initialization */
  ImxPadConfig (IMX_PAD_GPIO_0, IMX_PAD_CFG_USB_OTG1_ID);
  ImxPadConfig (IMX_PAD_GPIO_4, IMX_PAD_CFG_USB_OTG_PWR);

  /* Set Power polarity */
  usbnc_usb_ctrl = MmioRead32((UINTN) FixedPcdGet32(PcdEHCIBase) + USB_OTHERREGS_OFFSET);
  MmioWrite32((UINTN) FixedPcdGet32(PcdEHCIBase) + USB_OTHERREGS_OFFSET, usbnc_usb_ctrl | UCTRL_PWR_POL);

  /* Configure power and OC */
  ImxUsbPhyInit (IMX_USBPHY0);
  
  return RegisterNonDiscoverableMmioDevice (
           NonDiscoverableDeviceTypeEhci,
           NonDiscoverableDeviceDmaTypeNonCoherent,
           NULL,
           NULL,
           1,
           FixedPcdGet32(PcdEHCIBase) + 0x100, FixedPcdGet32(PcdEHCILength) - 0x100
           );

  return EFI_SUCCESS;
}
