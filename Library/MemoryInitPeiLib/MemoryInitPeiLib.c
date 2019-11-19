/** @file
*
*  Copyright (c) 2011-2015, ARM Limited. All rights reserved.
*  Copyright (c), 2017-2018, Andrey Warkentin <andrey.warkentin@gmail.com>
*  Copyright (c), 2019, Bingxing Wang <uefi-oss-projects@svc.nextplay.imbushuo.net>
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

#include <PiPei.h>

#include <Library/ArmMmuLib.h>
#include <Library/ArmPlatformLib.h>
#include <Library/DebugLib.h>
#include <Library/HobLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>

extern UINT64 mSystemMemoryEnd;

#define SYSTEM_MEMORY_RESOURCE_ATTR_CAPABILITIES                               \
  EFI_RESOURCE_ATTRIBUTE_PRESENT | EFI_RESOURCE_ATTRIBUTE_INITIALIZED |        \
      EFI_RESOURCE_ATTRIBUTE_TESTED | EFI_RESOURCE_ATTRIBUTE_UNCACHEABLE |     \
      EFI_RESOURCE_ATTRIBUTE_WRITE_COMBINEABLE |                               \
      EFI_RESOURCE_ATTRIBUTE_WRITE_THROUGH_CACHEABLE |                         \
      EFI_RESOURCE_ATTRIBUTE_WRITE_BACK_CACHEABLE |                            \
      EFI_RESOURCE_ATTRIBUTE_EXECUTION_PROTECTABLE

VOID
BuildMemoryTypeInformationHob (
  VOID
  );

STATIC
VOID
InitMmu (
  IN ARM_MEMORY_REGION_DESCRIPTOR  *MemoryTable
  )
{

  VOID                          *TranslationTableBase;
  UINTN                         TranslationTableSize;
  RETURN_STATUS                 Status;

  //Note: Because we called PeiServicesInstallPeiMemory() before to call InitMmu() the MMU Page Table resides in
  //      DRAM (even at the top of DRAM as it is the first permanent memory allocation)
  Status = ArmConfigureMmu (MemoryTable, &TranslationTableBase, &TranslationTableSize);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Error: Failed to enable MMU\n"));
  }
}

STATIC
VOID
Add(ARM_MEMORY_REGION_DESCRIPTOR *Desc, EFI_RESOURCE_TYPE ResType, 
  EFI_RESOURCE_ATTRIBUTE_TYPE ResAttrib, EFI_MEMORY_TYPE MemType)
{
  BuildResourceDescriptorHob (
    ResType,
    ResAttrib,
    Desc->PhysicalBase,
    Desc->Length
  );

  BuildMemoryAllocationHob (
    Desc->PhysicalBase,
    Desc->Length,
    MemType
  );
}

/*++

Routine Description:



Arguments:

  FileHandle  - Handle of the file being invoked.
  PeiServices - Describes the list of possible PEI Services.

Returns:

  Status -  EFI_SUCCESS if the boot mode could be set

--*/
EFI_STATUS
EFIAPI
MemoryPeim (
  IN EFI_PHYSICAL_ADDRESS               UefiMemoryBase,
  IN UINT64                             UefiMemorySize
  )
{
  ARM_MEMORY_REGION_DESCRIPTOR *MemoryTable;

  // Get Virtual Memory Map from the Platform Library
  ArmPlatformGetVirtualMemoryMap (&MemoryTable);

  // Ensure PcdSystemMemorySize has been set
  ASSERT (PcdGet64 (PcdSystemMemorySize) != 0);

  // SOC registers region, DMA registers region and GIC-400 registers region
  Add(&MemoryTable[0], EFI_RESOURCE_MEMORY_MAPPED_IO,
    EFI_RESOURCE_ATTRIBUTE_UNCACHEABLE, 
    EfiMemoryMappedIO
  );

  // Free memory
  Add(&MemoryTable[1], EFI_RESOURCE_SYSTEM_MEMORY,
    SYSTEM_MEMORY_RESOURCE_ATTR_CAPABILITIES, 
    EfiConventionalMemory
  );

  // MPPark mailbox
  Add(&MemoryTable[2], EFI_RESOURCE_MEMORY_RESERVED,
    EFI_RESOURCE_ATTRIBUTE_UNCACHEABLE,
    EfiRuntimeServicesCode
  );

  // Framebuffer
  Add(&MemoryTable[3], EFI_RESOURCE_MEMORY_RESERVED,
    EFI_RESOURCE_ATTRIBUTE_WRITE_THROUGH_CACHEABLE, 
    EfiReservedMemoryType
  );

  // FD region
  Add(&MemoryTable[4], EFI_RESOURCE_SYSTEM_MEMORY,
    SYSTEM_MEMORY_RESOURCE_ATTR_CAPABILITIES, 
    EfiBootServicesData
  );

  // Usable memory.
  Add(&MemoryTable[5], EFI_RESOURCE_SYSTEM_MEMORY,
    SYSTEM_MEMORY_RESOURCE_ATTR_CAPABILITIES, 
    EfiConventionalMemory
  );

  // Build Memory Allocation Hob
  InitMmu (MemoryTable);

  if (FeaturePcdGet (PcdPrePiProduceMemoryTypeInformationHob)) {
    // Optional feature that helps prevent EFI memory map fragmentation.
    BuildMemoryTypeInformationHob ();
  }

  return EFI_SUCCESS;
}