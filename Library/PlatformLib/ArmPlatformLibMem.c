/** @file
*
*  Copyright (c) 2011, ARM Limited. All rights reserved.
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

#include <Library/ArmPlatformLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>

#include <Device/regs-lcdif.h>
#include <Device/imx6_ull.h>

#define MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS  16

/**
  Return the Virtual Memory Map of your platform

  This Virtual Memory Map is used by MemoryInitPei Module to initialize the MMU on your platform.

  @param[out]   VirtualMemoryMap    Array of ARM_MEMORY_REGION_DESCRIPTOR describing a Physical-to-
                                    Virtual Memory mapping. This array must be ended by a zero-filled
                                    entry

**/
VOID
ArmPlatformGetVirtualMemoryMap (
  IN ARM_MEMORY_REGION_DESCRIPTOR** VirtualMemoryMap
  )
{
  ARM_MEMORY_REGION_ATTRIBUTES  CacheAttributes;
  UINTN                         Index = 0;
  ARM_MEMORY_REGION_DESCRIPTOR  *VirtualMemoryTable;

  ASSERT (VirtualMemoryMap != NULL);
  DEBUG ((DEBUG_VERBOSE, "Enter: ArmPlatformGetVirtualMemoryMap\n"));
  VirtualMemoryTable = (ARM_MEMORY_REGION_DESCRIPTOR *) AllocatePages (
      EFI_SIZE_TO_PAGES (sizeof (ARM_MEMORY_REGION_DESCRIPTOR) *
      MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS));
  if (VirtualMemoryTable == NULL) {
    return;
  }

  // SOC registers region
  VirtualMemoryTable[Index].PhysicalBase   = SOC_REGISTERS_PHYSICAL_BASE1;
  VirtualMemoryTable[Index].VirtualBase    = SOC_REGISTERS_PHYSICAL_BASE1;
  VirtualMemoryTable[Index].Length         = SOC_REGISTERS_PHYSICAL_LENGTH1;
  VirtualMemoryTable[Index].Attributes     = SOC_REGISTERS_ATTRIBUTES;

  // DMA registers region
  VirtualMemoryTable[++Index].PhysicalBase = APBH_DMA_REGISTERS_PHYSICAL_BASE;
  VirtualMemoryTable[Index].VirtualBase    = APBH_DMA_REGISTERS_PHYSICAL_BASE;
  VirtualMemoryTable[Index].Length         = APBH_DMA_REGISTERS_PHYSICAL_LENGTH;
  VirtualMemoryTable[Index].Attributes     = SOC_REGISTERS_ATTRIBUTES;

  // GIC-400 registers region
  VirtualMemoryTable[++Index].PhysicalBase = GIC_REGISTERS_PHYSICAL_BASE;
  VirtualMemoryTable[Index].VirtualBase    = GIC_REGISTERS_PHYSICAL_BASE;
  VirtualMemoryTable[Index].Length         = GIC_REGISTERS_PHYSICAL_LENGTH;
  VirtualMemoryTable[Index].Attributes     = SOC_REGISTERS_ATTRIBUTES;

  // Framebuffer
  VirtualMemoryTable[++Index].PhysicalBase = 0x80000000;
  VirtualMemoryTable[Index].VirtualBase    = 0x80000000;
  VirtualMemoryTable[Index].Length         = 0x800000;
  VirtualMemoryTable[Index].Attributes     = DDR_ATTRIBUTES_UNCACHED;

  // FD region
  VirtualMemoryTable[++Index].PhysicalBase   = BOOT_IMAGE_PHYSICAL_BASE;
  VirtualMemoryTable[Index].VirtualBase      = BOOT_IMAGE_PHYSICAL_BASE;
  VirtualMemoryTable[Index].Length           = BOOT_IMAGE_PHYSICAL_LENGTH;
  VirtualMemoryTable[Index].Attributes       = BOOT_IMAGE_ATTRIBUTES;

  // DRAM
  VirtualMemoryTable[++Index].PhysicalBase = BOOT_IMAGE_PHYSICAL_BASE + BOOT_IMAGE_PHYSICAL_LENGTH;
  VirtualMemoryTable[Index].VirtualBase    = BOOT_IMAGE_PHYSICAL_BASE + BOOT_IMAGE_PHYSICAL_LENGTH;
  VirtualMemoryTable[Index].Length         = 0xe600000;
  VirtualMemoryTable[Index].Attributes     = CacheAttributes;

  // End of Table
  VirtualMemoryTable[++Index].PhysicalBase = 0;
  VirtualMemoryTable[Index].VirtualBase    = 0;
  VirtualMemoryTable[Index].Length         = 0;
  VirtualMemoryTable[Index].Attributes     = (ARM_MEMORY_REGION_ATTRIBUTES)0;

  ASSERT ((Index + 1) <= MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS);

  *VirtualMemoryMap = VirtualMemoryTable;
}
