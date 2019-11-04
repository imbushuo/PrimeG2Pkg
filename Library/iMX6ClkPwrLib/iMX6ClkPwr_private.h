/** @file
*
*  Copyright (c) 2018 Microsoft Corporation. All rights reserved.
*  Copyright 2018 NXP
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

#ifndef _IMX6ULL_CLK_PWR_PRIVATE_H_
#define _IMX6ULL_CLK_PWR_PRIVATE_H_

#ifndef ARRAYSIZE
#define ARRAYSIZE(a) (sizeof(a) / sizeof(a[0]))
#endif // ARRAYSIZE

#define _BITS_PER_UINTN (8 * sizeof(UINTN))

typedef enum {
  IMX_PLL_PFD0,
  IMX_PLL_PFD1,
  IMX_PLL_PFD2,
  IMX_PLL_PFD3,
} IMX_PLL_PFD;

typedef struct {
  IMX_CLK Clock;
  IMX_CLOCK_INFO Info;
} IMX_CLOCK_CONTEXT;

typedef struct {
  UINT16 RegisterIndex;   // Register index (0-6)
  UINT16 GateNumber;      // Gate number within register (0-15)
} IMX_CCGR_INDEX;

typedef struct {
  UINTN Valid[(IMX_CLK_MAX + _BITS_PER_UINTN) / _BITS_PER_UINTN];
  IMX_CLOCK_INFO Table[IMX_CLK_MAX];
} IMX_CLOCK_TREE_CACHE;

IMX_CCGR_INDEX
ImxpCcgrIndexFromClkGate (
  IN  IMX_CLK_GATE    ClockGate
  );

VOID
ImxpClkPwrCacheReset (
  VOID
  );

IMX_CLK
ImxpClkFromBypassClkSource (
  IN  IMX_PLL_BYPASS_CLK_SRC    BypassClockSource
  );

VOID
ImxCcmConfigureGpuClockTree (
  VOID
  );

VOID
ImxCcmConfigureIPUDIxClockTree (
  VOID
  );

VOID
ImxCcmConfigureIPULDBxClockTree (
  VOID
  );

VOID
ImxSetClockRatePLL5 (
  IN  UINT32                                  ClockRate,
  IN  IMX_CCM_PLL_VIDEO_CTRL_POST_DIV_SELECT  PostDivSelect
  );

EFI_STATUS
ImxpGetClockInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  IN      IMX_CLK               ClockId,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

VOID
ImxpGetOsc24ClkInfo (
  OUT IMX_CLOCK_INFO  *ClockInfo
  );

EFI_STATUS
ImxpGetPll1MainClkInfo  (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetPll2MainClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetPll2PfdClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  IN      IMX_PLL_PFD           PfdIndex,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetPll3MainClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetPll3PfdClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  IN      IMX_PLL_PFD           PfdIndex,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetPll3SwClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetAxiClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetPeriphClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetPrePeriphClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetPeriphClk2Info (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetArmClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetMmdcCh0ClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetAhbClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetIpgClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetGpu2dAxiClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetGpu3dAxiClkRootInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetGpu2dCoreClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetGpu3dCoreClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

EFI_STATUS
ImxpGetGpu3dShaderClkInfo (
  IN OUT  IMX_CLOCK_TREE_CACHE  *Cache,
  OUT     IMX_CLOCK_INFO        *ClockInfo
  );

VOID
ImxEnableGpuVpuPowerDomain (
  VOID
  );

VOID
ImxDisableGpuVpuPowerDomain (
  VOID
  );

#endif // _IMX6ULL_CLK_PWR_PRIVATE_H_
