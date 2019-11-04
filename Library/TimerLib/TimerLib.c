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

#include <Base.h>
#include <Uefi.h>

#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/TimerLib.h>

#include <Device/common_gpt.h>
#include <Device/imx6_ull.h>
#include <Library/iMX6Timer.h>

RETURN_STATUS
EFIAPI
TimerConstructor (
  VOID
  )
{
  PCSP_GPT_REGS pGpt;
  UINT32 FreqPreScale;

  pGpt = (PCSP_GPT_REGS)CSP_BASE_REG_PA_GPT;

  ASSERT (SOC_OSC_FREQUENCY_REF_HZ >= PcdGet32 (PcdArmArchTimerFreqInHz));

  // Set up this timer
  MmioWrite32((UINTN) &pGpt->CR, (GPT_CR_SWR_RESET << GPT_CR_SWR_LSH));

  // MicroSecondDelay will break now
  for (UINTN i = 0; i < 100; i++)
  {
    MmioWrite32((UINTN) &pGpt->CR, 0);
  }

  // Calculate the scale factor since we are using the 24Mhz oscillator
  // as reference.
  FreqPreScale = SOC_OSC_FREQUENCY_REF_HZ / PcdGet32 (PcdArmArchTimerFreqInHz);
  ASSERT (FreqPreScale <= (1 << GPT_PR_PRESCALER_WID));

  // Set the frequency scale
  MmioWrite32 ((UINTN)&pGpt->PR, FreqPreScale - 1);

  // Set GPT configuration:
  // - GPT Enabled
  // - Enable 24 Mhz Oscillator
  // - Use the 24Mhz oscillator source
  MmioWrite32 ((UINTN)&pGpt->CR,
            (GPT_CR_EN_ENABLE << GPT_CR_EN_LSH) |
            (GPT_CR_EN_24M_ENABLE << GPT_CR_EN_24M_LSH) |
            (GPT_CR_CLKSRC_CLK24M << GPT_CR_CLKSRC_LSH));

  return EFI_SUCCESS;
}

/**
  Stalls the CPU for at least the given number of microseconds.

  Stalls the CPU for the number of microseconds specified by MicroSeconds.

  @param  MicroSeconds  The minimum number of microseconds to delay.

  @return The value of MicroSeconds inputted.

**/
UINTN
EFIAPI
MicroSecondDelay (
  IN  UINTN   MicroSeconds
  )
{
  UINT64 TimerTicks64;
  UINT32 CurCounterRead;
  UINT32 PrevCounterRead;
  UINT64 CountOffset;

  // Convert uSec delay to counter ticks:
  TimerTicks64      = ((UINT64)MicroSeconds * PcdGet32 (
                         PcdArmArchTimerFreqInHz)) / 1000000U;
  CurCounterRead    = (UINT32)GetPerformanceCounter();
  PrevCounterRead   = CurCounterRead;
  TimerTicks64      += (UINT64)CurCounterRead;
  CountOffset       = 0;

  // GPT is a 32bit counter, thus we need to handle rollover cases.
  while (((UINT64)CurCounterRead + CountOffset) < TimerTicks64) {
    CurCounterRead = (UINT32)GetPerformanceCounter();
    if (CurCounterRead < PrevCounterRead) {
      CountOffset += 0x100000000;
    }
    PrevCounterRead = CurCounterRead;
  }

  return MicroSeconds;
}

/**
  Stalls the CPU for at least the given number of nanoseconds.

  Stalls the CPU for the number of nanoseconds specified by NanoSeconds.

  @param  NanoSeconds The minimum number of nanoseconds to delay.

  @return The value of NanoSeconds inputted.

**/
UINTN
EFIAPI
NanoSecondDelay (
  IN  UINTN   NanoSeconds
  )
{
  if (NanoSeconds < (0xffffffff - 999)) {
    NanoSeconds += 999;
  }
  MicroSecondDelay (NanoSeconds / 1000);

  return 0;
}

/**
  Retrieves the current value of a 64-bit free running performance counter.

  The counter can either count up by 1 or count down by 1. If the physical
  performance counter counts by a larger increment, then the counter values
  must be translated. The properties of the counter can be retrieved from
  GetPerformanceCounterProperties().

  @return The current value of the free running performance counter.

**/
UINT64
EFIAPI
GetPerformanceCounter (
  VOID
  )
{
  PCSP_GPT_REGS pGpt;

  pGpt = (PCSP_GPT_REGS)CSP_BASE_REG_PA_GPT;
  return MmioRead32 ((UINTN)(&pGpt->CNT));
}

/**
  Retrieves the 64-bit frequency in Hz and the range of performance counter
  values.

  If StartValue is not NULL, then the value that the performance counter starts
  with immediately after is it rolls over is returned in StartValue. If
  EndValue is not NULL, then the value that the performance counter end with
  immediately before it rolls over is returned in EndValue. The 64-bit
  frequency of the performance counter in Hz is always returned. If StartValue
  is less than EndValue, then the performance counter counts up. If StartValue
  is greater than EndValue, then the performance counter counts down. For
  example, a 64-bit free running counter that counts up would have a StartValue
  of 0 and an EndValue of 0xFFFFFFFFFFFFFFFF. A 24-bit free running counter
  that counts down would have a StartValue of 0xFFFFFF and an EndValue of 0.

  @param  StartValue  The value the performance counter starts with when it
                      rolls over.
  @param  EndValue    The value that the performance counter ends with before
                      it rolls over.

  @return The frequency in Hz.

**/
UINT64
EFIAPI
GetPerformanceCounterProperties (
  OUT UINT64  *StartValue,  OPTIONAL
  OUT UINT64  *EndValue     OPTIONAL
  )
{
  if (StartValue != NULL) {
    *StartValue = 0x0;
  }

  if (EndValue != NULL) {
    *EndValue = MAX_UINT64;
  }

  return PcdGet32 (PcdArmArchTimerFreqInHz);
}

/**
  Converts elapsed ticks of performance counter to time in nanoseconds.

  This function converts the elapsed ticks of running performance counter to
  time value in unit of nanoseconds.

  @param  Ticks     The number of elapsed ticks of running performance counter.

  @return The elapsed time in nanoseconds.

**/
UINT64
EFIAPI
GetTimeInNanoSecond (
  IN  UINT64    Ticks
  )
{
  UINT64  NanoSeconds;
  UINT32  Remainder;
  UINT32  TimerFreq;

  TimerFreq = PcdGet32 (PcdArmArchTimerFreqInHz);

  //          Ticks
  // Time = --------- x 1,000,000,000
  //        Frequency
  NanoSeconds = MultU64x32 (
                  DivU64x32Remainder (
                    Ticks,
                    TimerFreq,
                    &Remainder),
                  1000000000U
                );

  // Frequency < 0x100000000, so Remainder < 0x100000000,
  // then (Remainder * 1,000,000,000) will not overflow 64-bit.
  NanoSeconds += DivU64x32 (
                   MultU64x32 (
                     (UINT64) Remainder,
                     1000000000U),
                   TimerFreq
                 );

  return NanoSeconds;
}
