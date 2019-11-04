/* Heartbeat: periodically flash the LED to indicate firmware status */

#include <PiDxe.h>
#include <Uefi.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>

#include <Device/imx6_ull.h>
#include <Device/iMX6IoMux.h>

EFI_EVENT m_CallbackTimer = NULL;
IMX_GPIO_VALUE LastLedValue = IMX_GPIO_HIGH;

VOID EFIAPI LedHeartbeatCallback(
    IN EFI_EVENT Event, 
    IN VOID *Context
)
{
    // GPIO1_2: LED_GREEN_N
    LastLedValue = (LastLedValue == IMX_GPIO_LOW) ? IMX_GPIO_HIGH : IMX_GPIO_LOW;
    ImxGpioWrite(IMX_GPIO_BANK1, 2, LastLedValue);
}

EFI_STATUS
EFIAPI
HeartbeatInitialize(
    IN EFI_HANDLE ImageHandle, 
    IN EFI_SYSTEM_TABLE *SystemTable
)
{
    EFI_STATUS Status;

    // Set LED direction
    ImxGpioDirection(IMX_GPIO_BANK1, 2, IMX_GPIO_DIR_OUTPUT);
    ImxGpioDirection(IMX_GPIO_BANK1, 7, IMX_GPIO_DIR_OUTPUT);
    ImxGpioDirection(IMX_GPIO_BANK1, 9, IMX_GPIO_DIR_OUTPUT);

    // Turn on all LEDs
    ImxGpioWrite(IMX_GPIO_BANK1, 2, IMX_GPIO_LOW);
    ImxGpioWrite(IMX_GPIO_BANK1, 7, IMX_GPIO_LOW);
    ImxGpioWrite(IMX_GPIO_BANK1, 9, IMX_GPIO_LOW);

    // Wait a bit (screen will be unresponsive)
    MicroSecondDelay(500000);

    // Turn off all LEDs
    ImxGpioWrite(IMX_GPIO_BANK1, 2, IMX_GPIO_HIGH);
    ImxGpioWrite(IMX_GPIO_BANK1, 7, IMX_GPIO_HIGH);
    ImxGpioWrite(IMX_GPIO_BANK1, 9, IMX_GPIO_HIGH);

    Status = gBS->CreateEvent(
        EVT_NOTIFY_SIGNAL | EVT_TIMER,
        TPL_CALLBACK, LedHeartbeatCallback, NULL,
        &m_CallbackTimer
    );

    ASSERT_EFI_ERROR(Status);

    Status = gBS->SetTimer(
        m_CallbackTimer, TimerPeriodic,
        EFI_TIMER_PERIOD_MILLISECONDS(500)
    );

    ASSERT_EFI_ERROR(Status);
    return Status;
}
