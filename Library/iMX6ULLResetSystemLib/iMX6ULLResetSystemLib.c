#include <PiDxe.h>

#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/EfiResetSystemLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>

#include <Device/imx6_ull.h>

STATIC UINTN pWatchDog1ControlRegister = IMX_WDOG1_WCR;
STATIC UINTN pWatchDog2ControlRegister = IMX_WDOG2_WCR;
EFI_EVENT mRuntimeVirtualAddressChangedEvent;

VOID EFIAPI RtConvertPointers(IN EFI_EVENT Event, IN VOID *Context)
{
  gRT->ConvertPointer(0, (VOID **) &pWatchDog1ControlRegister);
  gRT->ConvertPointer(0, (VOID **) &pWatchDog2ControlRegister);
}

VOID EFIAPI AssertWatchdogSwReset()
{
  MmioWrite16(pWatchDog1ControlRegister, MmioRead16(pWatchDog1ControlRegister) & ~(1UL << 4));
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

  // Register BS event for virtual address change
  Status = gBS->CreateEventEx(
      EVT_NOTIFY_SIGNAL, TPL_NOTIFY, RtConvertPointers, NULL,
      &gEfiEventVirtualAddressChangeGuid,
      &mRuntimeVirtualAddressChangedEvent
  );

  return Status;
}
