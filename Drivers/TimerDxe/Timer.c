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

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#include <Protocol/HardwareInterrupt.h>
#include <Protocol/Timer.h>

#include <Device/common_epit.h>
#include <Device/imx6_ull.h>

// The notification function to call on every timer interrupt.
volatile EFI_TIMER_NOTIFY mTimerNotifyFunction = (EFI_TIMER_NOTIFY) NULL;
EFI_EVENT EfiExitBootServicesEvent = (EFI_EVENT) NULL;

// Cached copy of the Hardware Interrupt protocol instance
EFI_HARDWARE_INTERRUPT_PROTOCOL *gInterrupt = NULL;

// Cached interrupt vector
volatile UINTN  mVector;
UINT64 mCurrentTimerPeriod;

EFI_STATUS
EFIAPI
TimerDriverRegisterHandler (
  IN EFI_TIMER_ARCH_PROTOCOL  *This,
  IN EFI_TIMER_NOTIFY         NotifyFunction
  )
{
  DEBUG ((DEBUG_VERBOSE, "++TimerDriverRegisterHandler()\n"));
  if ((NotifyFunction == NULL) && (mTimerNotifyFunction == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  if ((NotifyFunction != NULL) && (mTimerNotifyFunction != NULL)) {
    return EFI_ALREADY_STARTED;
  }

  mTimerNotifyFunction = NotifyFunction;
  DEBUG ((DEBUG_VERBOSE, "--TimerDriverRegisterHandler()=ok\n"));
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
TimerDriverSetTimerPeriod (
  IN EFI_TIMER_ARCH_PROTOCOL  *This,
  IN UINT64                   TimerPeriod
  )
{
  PCSP_EPIT_REG   pEpit;
  UINT16          EpitPreScalar;
  EFI_STATUS      Status;
  UINT32          TimerCount;
  UINT32          Value;

  DEBUG ((DEBUG_VERBOSE, "++TimerDriverSetTimerPeriod(%d)\n", TimerPeriod));

  pEpit = (PCSP_EPIT_REG) CSP_BASE_REG_PA_EPIT1;
  DEBUG ((DEBUG_VERBOSE,
          "TimerDriverSetTimerPeriod() disable timer. EPIT_REG adr=%p\n", pEpit));

  // First stop the timer.
  Value = MmioRead32 ((UINTN)&pEpit->CR);
  Value &= ~(((1 << EPIT_CR_EN_WID) - 1) << EPIT_CR_EN_LSH);
  Value |= (EPIT_CR_EN_DISABLE << EPIT_CR_EN_LSH);
  MmioWrite32 ((UINTN)&pEpit->CR, Value);

  if (TimerPeriod == 0) {
    Status = gInterrupt->DisableInterruptSource (gInterrupt, mVector);
    mCurrentTimerPeriod = 0;
    DEBUG ((DEBUG_VERBOSE, "--TimerDriverSetTimerPeriod() Timer Disabled\n"));
    return Status;
  }

  // Configure EPIT to be sourced from iMX6 24 MHz crystal oscialltor
  // Aim to have UEFI tick counting at 1 MHz clock or another frequency as set in pcd
  EpitPreScalar = 68;
  DEBUG ((DEBUG_VERBOSE,
          "TimerDriverSetTimerPeriod() using corrected EPIT prescalar=%d\n",
          EpitPreScalar));

  MmioWrite32 ((UINTN)&pEpit->CR,
            (EPIT_CR_ENMOD_LOAD << EPIT_CR_ENMOD_LSH) |
            (EPIT_CR_OCIEN_ENABLE << EPIT_CR_OCIEN_LSH) |
            (EPIT_CR_RLD_RELOAD << EPIT_CR_RLD_LSH) |
            ((EpitPreScalar - 1) << EPIT_CR_PRESCALAR_LSH) |
            (EPIT_CR_SWR_NORESET << EPIT_CR_SWR_LSH) |
            (EPIT_CR_IOVW_OVR << EPIT_CR_IOVW_LSH) |
            (EPIT_CR_DBGEN_ACTIVE << EPIT_CR_DBGEN_LSH) |
            (EPIT_CR_WAITEN_ENABLE << EPIT_CR_WAITEN_LSH) |
            (EPIT_CR_DOZEN_ENABLE << EPIT_CR_DOZEN_LSH) |
            (EPIT_CR_STOPEN_ENABLE << EPIT_CR_STOPEN_LSH) |
            (EPIT_CR_OM_DICONNECT << EPIT_CR_OM_LSH) |
            (EPIT_CR_CLKSRC_IPGCLK << EPIT_CR_CLKSRC_LSH));

  // Clear timer compare interrupt flag (write-1-clear)
  MmioWrite32 ((UINTN)&pEpit->SR, ((1 << EPIT_SR_OCIF_WID) - 1) << EPIT_SR_OCIF_LSH);
  TimerCount = (UINT32) (TimerPeriod / 10);
  if ((UINT64)TimerCount > (UINT64)0xffffffff) {
    TimerCount = 0xffffffff;
  }

  mCurrentTimerPeriod = TimerPeriod;
  MmioWrite32 ((UINTN)&pEpit->CMPR, TimerCount);
  MmioWrite32 ((UINTN)&pEpit->LR, TimerCount);
  Status = gInterrupt->EnableInterruptSource (gInterrupt, mVector);

  // Turn the timer on
  Value = MmioRead32 ((UINTN)&pEpit->CR);
  Value &= ~(((1 << EPIT_CR_EN_WID) - 1) << EPIT_CR_EN_LSH);
  Value |= EPIT_CR_EN_ENABLE << EPIT_CR_EN_LSH;
  MmioWrite32 ((UINTN)&pEpit->CR, Value);

  DEBUG ((DEBUG_VERBOSE, "--TimerDriverSetTimerPeriod(%d)=%Xh\n", TimerPeriod,
          Status));
  return Status;
}

EFI_STATUS
EFIAPI
TimerDriverGetTimerPeriod (
  IN EFI_TIMER_ARCH_PROTOCOL   *This,
  OUT UINT64                   *TimerPeriod
  )
{
  *TimerPeriod = mCurrentTimerPeriod;
  DEBUG ((DEBUG_VERBOSE, "+-TimerDriverGetTimerPeriod(%d)=ok\n",
          mCurrentTimerPeriod));
  return EFI_SUCCESS;
}

VOID
EFIAPI
TimerInterruptHandler (
  IN  HARDWARE_INTERRUPT_SOURCE   Source,
  IN  EFI_SYSTEM_CONTEXT          SystemContext
  )
{
  EFI_TPL OriginalTPL;
  PCSP_EPIT_REG pEpit;

  pEpit = (PCSP_EPIT_REG) CSP_BASE_REG_PA_EPIT1;

  // DXE core uses this callback for the EFI timer tick. The DXE core uses locks
  // that raise to TPL_HIGH and then restore back to current level. Thus we need
  // to make sure TPL level is set to TPL_HIGH while we are handling the timer tick.
  OriginalTPL = gBS->RaiseTPL (TPL_HIGH_LEVEL);

  // Check if the timer interrupt is active
  if (MmioRead32 ((UINTN)&pEpit->SR) != 0) {
    // Acknowledge the EPIT interrupt
    MmioWrite32 ((UINTN)&pEpit->SR, 0x1);

    // Signal EOI to avoid losing subsequent ticks from long duration handlers
    gInterrupt->EndOfInterrupt (gInterrupt, Source);

    if (mTimerNotifyFunction) {
      mTimerNotifyFunction (mCurrentTimerPeriod);
    }
  }

  gBS->RestoreTPL (OriginalTPL);
}

EFI_STATUS
EFIAPI
TimerDriverGenerateSoftInterrupt (
  IN EFI_TIMER_ARCH_PROTOCOL  *This
  )
{
  return EFI_UNSUPPORTED;
}

EFI_TIMER_ARCH_PROTOCOL gTimer = {
  TimerDriverRegisterHandler,
  TimerDriverSetTimerPeriod,
  TimerDriverGetTimerPeriod,
  TimerDriverGenerateSoftInterrupt
};

VOID
EFIAPI
ExitBootServicesEvent (
  IN EFI_EVENT  Event,
  IN VOID       *Context
  )
{
  EFI_STATUS  Status = EFI_SUCCESS;

  DEBUG ((DEBUG_INFO, "Disabling EPIT timer on ExitBootServicesEvent"));

  // Disable the timer
  Status = TimerDriverSetTimerPeriod (&gTimer, 0);
  ASSERT_EFI_ERROR (Status);
}

EFI_STATUS
EFIAPI
TimerInitialize (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
)
{
  EFI_HANDLE  Handle;
  EFI_STATUS  Status;
  DEBUG ((DEBUG_VERBOSE, "++TimerInitialize()\n"));

  Handle = NULL;
  mVector = IRQ_EPIT1;

  // Find the interrupt controller protocol.
  Status = gBS->LocateProtocol (
             &gHardwareInterruptProtocolGuid,
             NULL,
             (VOID **) &gInterrupt);
  ASSERT_EFI_ERROR (Status);

  // Disable the timer
  Status = TimerDriverSetTimerPeriod (&gTimer, 0);
  ASSERT_EFI_ERROR (Status);

  // Install interrupt handler
  Status = gInterrupt->RegisterInterruptSource (gInterrupt, mVector,
           TimerInterruptHandler);
  ASSERT_EFI_ERROR (Status);

  // Set up default timer
  Status = TimerDriverSetTimerPeriod (&gTimer, FixedPcdGet32 (PcdTimerPeriod));
  ASSERT_EFI_ERROR (Status);

  DEBUG ((
           DEBUG_VERBOSE,
           "EPIT Timer initialized to default period %d x 100ns ~ %dms\n",
           FixedPcdGet32 (PcdTimerPeriod),
           FixedPcdGet32 (PcdTimerPeriod) / 10000));

  // Install the Timer Architectural Protocol onto a new handle
  Status = gBS->InstallMultipleProtocolInterfaces (
             &Handle,
             &gEfiTimerArchProtocolGuid,
             &gTimer,
             NULL);

  ASSERT_EFI_ERROR (Status);

  // Register for ExitBootServicesEvent
  Status = gBS->CreateEvent (
             EVT_SIGNAL_EXIT_BOOT_SERVICES,
             TPL_NOTIFY,
             ExitBootServicesEvent,
             NULL,
             &EfiExitBootServicesEvent);

  ASSERT_EFI_ERROR (Status);

  DEBUG ((DEBUG_VERBOSE, "--TimerInitialize()\n"));
  return Status;
}
