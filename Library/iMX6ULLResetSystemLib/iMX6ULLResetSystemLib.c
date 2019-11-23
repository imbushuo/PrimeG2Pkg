#include <PiDxe.h>

#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/EfiResetSystemLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/DxeServicesTableLib.h>

#include <Device/imx6_ull.h>

STATIC UINTN pWatchDog1ControlRegister = IMX_WDOG1_WCR;
EFI_EVENT mRuntimeVirtualAddressChangedEvent;

VOID EFIAPI RtConvertPointers(IN EFI_EVENT Event, IN VOID *Context)
{
  gRT->ConvertPointer(0, (VOID **) &pWatchDog1ControlRegister);
}

VOID EFIAPI AssertWatchdogSwReset()
{
  // TODO: Turn off the LCD interface

  MmioWrite16(pWatchDog1ControlRegister + WDT_WCR, 0x24);
  ArmDataMemoryBarrier();

  if (MmioRead16(pWatchDog1ControlRegister + WDT_WCR) & WDT_WCR_WDE) {
		MmioWrite16(pWatchDog1ControlRegister + WDT_WSR, WDT_SEQ1);
		MmioWrite16(pWatchDog1ControlRegister + WDT_WSR, WDT_SEQ2);
	}

  MmioWrite16(pWatchDog1ControlRegister + WDT_WCR, 0x24);
	MmioWrite16(pWatchDog1ControlRegister + WDT_WCR, 0x24);
  CpuDeadLoop();
}

/**
  Resets the entire platform.

  @param  ResetType             The type of reset to perform.
  @param  ResetStatus           The status code for the reset.
  @param  DataSize              The size, in bytes, of WatchdogData.
  @param  ResetData             For a ResetType of EfiResetCold, EfiResetWarm, or
                                EfiResetShutdown the data buffer starts with a Null-terminated
                                Unicode string, optionally followed by additional binary data.

**/
EFI_STATUS
EFIAPI
LibResetSystem (
  IN EFI_RESET_TYPE   ResetType,
  IN EFI_STATUS       ResetStatus,
  IN UINTN            DataSize,
  IN CHAR16           *ResetData OPTIONAL
  )
{
  switch (ResetType) {
  case EfiResetPlatformSpecific:
  case EfiResetWarm:
  case EfiResetCold:
  case EfiResetShutdown:
    AssertWatchdogSwReset();
    break;
  default:
    ASSERT (FALSE);
    return EFI_UNSUPPORTED;
  }

  DEBUG ((EFI_D_ERROR, "%a: WDOG Reset failed\n", __FUNCTION__));
  CpuDeadLoop ();
  return EFI_UNSUPPORTED;
}

/**
  Initialize any infrastructure required for LibResetSystem () to function.

  @param  ImageHandle   The firmware allocated handle for the EFI image.
  @param  SystemTable   A pointer to the EFI System Table.

  @retval EFI_SUCCESS   The constructor always returns EFI_SUCCESS.

**/
EFI_STATUS
EFIAPI
LibInitializeResetSystem (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS Status;
  EFI_GCD_MEMORY_SPACE_DESCRIPTOR pDescriptor;

  // Register the WDOG for runtime memory change
  Status = gDS->GetMemorySpaceDescriptor(
    ROUND_TO_PAGE(pWatchDog1ControlRegister),
    &pDescriptor
  );

  if (EFI_ERROR (Status)) {
    goto exit;
  }

  Status = gDS->SetMemorySpaceAttributes(
    ROUND_TO_PAGE(pWatchDog1ControlRegister),
    0x1000, /* 4K align */
    pDescriptor.Attributes | EFI_MEMORY_RUNTIME
  );
  ASSERT_EFI_ERROR (Status);

  // Register BS event for virtual address change
  Status = gBS->CreateEventEx(
    EVT_NOTIFY_SIGNAL, TPL_NOTIFY, RtConvertPointers, NULL,
    &gEfiEventVirtualAddressChangeGuid,
    &mRuntimeVirtualAddressChangedEvent
  );

exit:
  return Status;
}
