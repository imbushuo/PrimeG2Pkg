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
#include <Device/iMX6IoMux.h>
#include <Device/iMX6UsbPhy.h>

#define USB_PHY_PLL_LOCK_TIMEOUT_USEC (UINT32)(1000*1000)
#define USB_EHCI_STOP_RESET_TIMEOUT_USEC (UINT32)(1000*1000)

#define IMX_CCM_ANALOG_PLL_USB1_REG_LOCK 0x80000000
#define IMX_USB_CMD_REG_RUN 0x00000001
#define IMX_USB_CMD_REG_RESET 0x00000002

/**
  Wait for a register bit to be on/off
**/
EFI_STATUS
RegisterWaitBit (
  IN  volatile VOID   *RegisterAddr,
  IN  UINT32          Mask,
  IN  BOOLEAN         IsWaitOn,
  IN  UINT32          TimeOutUsec
  )
{
  UINT32 RegisterValue;
  UINT32 TimeUsec;

  TimeUsec = 0;
  do {
    RegisterValue = MmioRead32 ((UINTN)RegisterAddr) & Mask;
    if (((RegisterValue == Mask) && IsWaitOn) || ((RegisterValue == 0)
        && !IsWaitOn)) {
      return EFI_SUCCESS;
    }

    MicroSecondDelay (10);
    TimeUsec += 10;
  } while (TimeUsec < TimeOutUsec);

  return EFI_TIMEOUT;
}

/**
  Turn on the 480Mhz PLL
**/
EFI_STATUS
ImxUsbPhyEnablePll (
  IN  IMX_USBPHY_ID   ImxUsbPhyId
  )
{
  volatile IMX_CCM_ANALOG_REGISTERS     *CcmAnaRegsPtr;
  volatile IMX_CCM_ANALOG_PLL_USB1_REG  *PllUsbClrRegPtr;
  volatile IMX_CCM_ANALOG_PLL_USB1_REG  *PllUsbSetRegPtr;
  IMX_CCM_ANALOG_PLL_USB1_REG           PllUsbClrReg;
  IMX_CCM_ANALOG_PLL_USB1_REG           PllUsbSetReg;
  EFI_STATUS                            Status;

  CcmAnaRegsPtr = (IMX_CCM_ANALOG_REGISTERS *)IMX_CCM_ANALOG_BASE;

  switch (ImxUsbPhyId) {
  case IMX_USBPHY0:
    PllUsbClrRegPtr = (IMX_CCM_ANALOG_PLL_USB1_REG *)&CcmAnaRegsPtr->PLL_USB1_CLR;
    PllUsbSetRegPtr = (IMX_CCM_ANALOG_PLL_USB1_REG *)&CcmAnaRegsPtr->PLL_USB1_SET;
    break;
  case IMX_USBPHY1:
    PllUsbClrRegPtr = (IMX_CCM_ANALOG_PLL_USB1_REG *)&CcmAnaRegsPtr->PLL_USB2_CLR;
    PllUsbSetRegPtr = (IMX_CCM_ANALOG_PLL_USB1_REG *)&CcmAnaRegsPtr->PLL_USB2_SET;
    break;
  default:
    return EFI_INVALID_PARAMETER;
  }

  PllUsbClrReg.AsUint32 = 0;
  PllUsbClrReg.BYPASS = 1;
  MmioWrite32 ((UINTN)PllUsbClrRegPtr, PllUsbClrReg.AsUint32);

  PllUsbSetReg.AsUint32 = 0;
  PllUsbSetReg.EN_USB_CLKS = 1;
  PllUsbSetReg.POWER = 1;
  PllUsbSetReg.ENABLE = 1;
  MmioWrite32 ((UINTN)PllUsbSetRegPtr, PllUsbSetReg.AsUint32);

  // Wait for PLL to lock
  Status = RegisterWaitBit (
             PllUsbSetRegPtr,
             IMX_CCM_ANALOG_PLL_USB1_REG_LOCK,
             TRUE,
             USB_PHY_PLL_LOCK_TIMEOUT_USEC);

  if (Status != EFI_SUCCESS) {
    DEBUG ((DEBUG_ERROR, "PLL 480Mhz failed to lock for PHY %d\n",
            (UINT32)ImxUsbPhyId));
    // On failure disable the PHY
    PllUsbClrReg.AsUint32 = 0;
    PllUsbSetReg.EN_USB_CLKS = 1;
    PllUsbSetReg.POWER = 1;
    PllUsbSetReg.ENABLE = 1;
    MmioWrite32 ((UINTN)PllUsbClrRegPtr, PllUsbSetReg.AsUint32);
    return Status;
  }

  return EFI_SUCCESS;
}

/**
  Reset the EHCI controller associated with the given PHY.
**/
EFI_STATUS
ImxUsbEhciResetController (
  IN  IMX_USBPHY_ID   ImxUsbPhyId
  )
{
  volatile USB_USBCMD_REG   *UsbCmdRegPtr;
  volatile USB_USBMODE_REG  *UsbModeRegPtr;
  EFI_STATUS                Status;
  USB_USBCMD_REG            UsbCmdReg;
  USB_USBMODE_REG           UsbModeReg;

  switch (ImxUsbPhyId) {
  case IMX_USBPHY0:
    UsbCmdRegPtr = (USB_USBCMD_REG *) (IMX_USBCORE_BASE + IMX_USBCORE_LENGTH * 0 +
                                       IMX_USBCMD_OFFSET);
    break;
  case IMX_USBPHY1:
    UsbCmdRegPtr = (USB_USBCMD_REG *) (IMX_USBCORE_BASE + IMX_USBCORE_LENGTH * 1 +
                                       IMX_USBCMD_OFFSET);
    break;
  default:
    return EFI_INVALID_PARAMETER;
  }

  // The host controller can only be reset when it is stopped.
  UsbCmdReg.AsUint32 = MmioRead32 ((UINTN)UsbCmdRegPtr);
  UsbCmdReg.RS = 0;
  MmioWrite32 ((UINTN)UsbCmdRegPtr, UsbCmdReg.AsUint32);

  // Wait for controller to stop
  Status = RegisterWaitBit (
             UsbCmdRegPtr,
             IMX_USB_CMD_REG_RUN,
             FALSE,
             USB_EHCI_STOP_RESET_TIMEOUT_USEC);
  if (Status != EFI_SUCCESS) {
    ASSERT_EFI_ERROR (Status);
    DEBUG ((DEBUG_ERROR, "Failed to stop EHCI controller (PHY %d)\n",
            (UINT32)ImxUsbPhyId));
    return Status;
  }

  // Reset the controller
  UsbCmdReg.AsUint32 = MmioRead32 ((UINTN)UsbCmdRegPtr);
  UsbCmdReg.RST = 1;
  MmioWrite32 ((UINTN)UsbCmdRegPtr, UsbCmdReg.AsUint32);

  // Wait for controller reset to complete
  Status = RegisterWaitBit (
             UsbCmdRegPtr,
             IMX_USB_CMD_REG_RESET,
             FALSE,
             USB_EHCI_STOP_RESET_TIMEOUT_USEC);

  if (Status != EFI_SUCCESS) {
    ASSERT_EFI_ERROR (Status);
    DEBUG ((DEBUG_ERROR, "Failed to reset EHCI controller (PHY %d)\n",
            (UINT32)ImxUsbPhyId));
    return Status;
  }

  // Force OTG port into Host mode. We have seen that ID_PIN can be
  // unreliable in some board designs (e.g. SABRESED).
  // If the OTG port is not forced into Host mode, the USB stack fails to
  // start.
  if (ImxUsbPhyId == IMX_USBPHY0) {
    DEBUG ((DEBUG_INFO, "Switching USB OTG Port to Host\n"));

    UsbModeRegPtr = (USB_USBMODE_REG *) (IMX_USBCORE_BASE + IMX_USBMODE_OFFSET);
    UsbModeReg.AsUint32 = MmioRead32 ((UINTN)UsbModeRegPtr);
    UsbModeReg.CM = IMX_USBMODE_HOST;
    MmioWrite32 ((UINTN)UsbModeRegPtr, UsbModeReg.AsUint32);

    UsbModeReg.AsUint32 = MmioRead32 ((UINTN)UsbModeRegPtr);
    ASSERT (UsbModeReg.CM == IMX_USBMODE_HOST);
  }

  if (ImxUsbPhyId == IMX_USBPHY1) {
    volatile USB_USBMODE_REG* UsbModeRegPtr;
    USB_USBMODE_REG UsbModeReg;
    DEBUG ((DEBUG_INFO, "Switching USB OTG Port to Host\n"));
    UsbModeRegPtr = (USB_USBMODE_REG*)(IMX_USBCORE_BASE + IMX_USBCORE_LENGTH + IMX_USBMODE_OFFSET);
    UsbModeReg.AsUint32 = MmioRead32 ((UINTN)UsbModeRegPtr);
    UsbModeReg.CM = IMX_USBMODE_HOST;
    MmioWrite32 ((UINTN)UsbModeRegPtr, UsbModeReg.AsUint32);
    DEBUG_CODE_BEGIN();
    UsbModeReg.AsUint32 = MmioRead32 ((UINTN)UsbModeRegPtr);
    ASSERT (UsbModeReg.CM == IMX_USBMODE_HOST);
    DEBUG_CODE_END();
  }

  return EFI_SUCCESS;
}

/**
  Initialize a USB PHY
**/
EFI_STATUS
ImxUsbPhyInit (
  IN  IMX_USBPHY_ID   ImxUsbPhyId
  )
{
  volatile IMX_USBANA_REGISTERS       *UsbAnaRegsPtr;
  volatile IMX_USBANA_USB_REGISTERS   *UsbAnaUsbRegsPtr;
  volatile USBNC_USB_UH_CTRL_REG      *UsbNcUhCtrlRegPtr;
  volatile IMX_USBNONCORE_REGISTERS   *UsbNonCoreRegPtr;
  volatile IMX_USBPHY_REGISTERS       *UsbPhyRegsPtr;
  EFI_STATUS                          Status;
  USB_ANALOG_USB_CHRG_DETECT_REG      UsbAnaChrgDetReg;
  USBNC_USB_UH_CTRL_REG               UsbNcHcCtrlReg;
  USBPHYx_CTRL_REG                    UsbPhyCtrlReg;
  USB_ANALOG_USB_MISC_REG             UsbAnaMicReg;

  UsbAnaRegsPtr = (IMX_USBANA_REGISTERS *)IMX_USBANA_BASE;
  UsbNonCoreRegPtr = (IMX_USBNONCORE_REGISTERS *)IMX_USBNONCORE_BASE;

  switch (ImxUsbPhyId) {
  case IMX_USBPHY0:
    UsbPhyRegsPtr = (IMX_USBPHY_REGISTERS *)IMX_USBPHY1_BASE;
    UsbAnaUsbRegsPtr = &UsbAnaRegsPtr->USBANA[0];
    UsbNcUhCtrlRegPtr = (USBNC_USB_UH_CTRL_REG *)
                        &UsbNonCoreRegPtr->USBNC_USB_OTG_CTRL;
    break;
  case IMX_USBPHY1:
    UsbPhyRegsPtr = (IMX_USBPHY_REGISTERS *)IMX_USBPHY2_BASE;
    UsbAnaUsbRegsPtr = &UsbAnaRegsPtr->USBANA[1];
    UsbNcUhCtrlRegPtr = (USBNC_USB_UH_CTRL_REG *)
                        &UsbNonCoreRegPtr->USBNC_USB_UH1_CTRL;
    break;
  default:
    return EFI_INVALID_PARAMETER;
  }

  // USB power configuration:
  // Set power polarity
  UsbNcHcCtrlReg.AsUint32 = MmioRead32 ((UINTN)UsbNcUhCtrlRegPtr);
  UsbNcHcCtrlReg.PWR_POL = 1;
  MmioWrite32 ((UINTN)UsbNcUhCtrlRegPtr, UsbNcHcCtrlReg.AsUint32);

  // Disable external USB charger detector
  UsbAnaChrgDetReg.AsUint32 = 0;
  UsbAnaChrgDetReg.EN_B = 1;
  UsbAnaChrgDetReg.CHK_CHRG_B = 1;
  MmioWrite32 ((UINTN)&UsbAnaUsbRegsPtr->USB_ANALOG_USB_CHRG_DETECT_SET,
                UsbAnaChrgDetReg.AsUint32);

  // Enable the 480Mhz PLL
  Status = ImxUsbPhyEnablePll (ImxUsbPhyId);
  if (Status != EFI_SUCCESS) {
    ASSERT_EFI_ERROR (Status);
    DEBUG ((DEBUG_ERROR, "Failed to enable PLL 480Mhz failed for PHY %d\n",
            (UINT32)ImxUsbPhyId));
    return Status;
  }

  // Configure Over Current
  UsbNcHcCtrlReg.AsUint32 = MmioRead32 ((UINTN)UsbNcUhCtrlRegPtr);
  UsbNcHcCtrlReg.OVER_CUR_POL = 0;
  UsbNcHcCtrlReg.OVER_CUR_DIS = 1;
  MmioWrite32 ((UINTN)UsbNcUhCtrlRegPtr, UsbNcHcCtrlReg.AsUint32);

  // Enable USBH PHY clock
  UsbPhyCtrlReg.AsUint32 = 0;
  UsbPhyCtrlReg.CLKGATE = 1;
  MmioWrite32 ((UINTN)&UsbPhyRegsPtr->USBPHY_CTRL_CLR, UsbPhyCtrlReg.AsUint32);
  MicroSecondDelay (10);

  // Enable clock to UTMI block
  UsbAnaMicReg.AsUint32 = 0;
  UsbAnaMicReg.EN_CLK_UTMI = 1;
  MmioWrite32 ((UINTN)&UsbAnaUsbRegsPtr->USB_ANALOG_USB_MISC_SET,
                UsbAnaMicReg.AsUint32);
  MicroSecondDelay (10);

  // Enable USBH PHY
  // Reset the associated EHCI controller
  Status = ImxUsbEhciResetController (ImxUsbPhyId);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  // Reset the PHY
  UsbPhyCtrlReg.AsUint32 = 0;
  UsbPhyCtrlReg.SFTRST = 1;
  MmioWrite32 ((UINTN)&UsbPhyRegsPtr->USBPHY_CTRL_SET, UsbPhyCtrlReg.AsUint32);
  MicroSecondDelay (10);

  UsbPhyCtrlReg.AsUint32 = 0;
  UsbPhyCtrlReg.SFTRST = 1;
  UsbPhyCtrlReg.CLKGATE = 1;
  MmioWrite32 ((UINTN)&UsbPhyRegsPtr->USBPHY_CTRL_CLR, UsbPhyCtrlReg.AsUint32);
  MicroSecondDelay (10);

  // Power UP the PHY
  MmioWrite32 ((UINTN)&UsbPhyRegsPtr->USBPHY_PWD, 0);

  // Apply PHY configuration:
  // - Enable low/full speed devices.
  UsbPhyCtrlReg.AsUint32 = 0;
  UsbPhyCtrlReg.ENAUTOSET_USBCLKS = 1;
  UsbPhyCtrlReg.ENAUTOCLR_USBCLKGATE = 1;
  UsbPhyCtrlReg.ENAUTO_PWRON_PLL = 1;
  MmioWrite32 ((UINTN)&UsbPhyRegsPtr->USBPHY_CTRL_SET, UsbPhyCtrlReg.AsUint32);
  MmioWrite32 ((UINTN)&UsbPhyRegsPtr->USBPHY_IP_SET, IMX_USBPHY_IP_FIX);

  return EFI_SUCCESS;
}
