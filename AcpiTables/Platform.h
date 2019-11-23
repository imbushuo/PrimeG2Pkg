/** @file
*
*  i.MX Platform specific defines for constructing ACPI tables
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

#ifndef _PLATFORM_IMX_H_
#define _PLATFORM_IMX_H_

#include <IndustryStandard/Acpi50.h>

//
// Platform specific definition
//
#define EFI_ACPI_OEM_TABLE_ID                   SIGNATURE_64('I','M','X','6','E','D','K','2') // OEM table id 8 bytes long
#define EFI_ACPI_OEM_REVISION                   0x01000105
#define EFI_ACPI_CREATOR_ID                     SIGNATURE_32('I','M','X','6')
#define EFI_ACPI_CREATOR_REVISION               0x00000001

#define EFI_ACPI_OEM_ID               {'M','C','R','S','F','T'} // OEMID 6 bytes
#define EFI_ACPI_VENDOR_ID            SIGNATURE_32('N','X','P','I')
#define EFI_ACPI_CSRT_REVISION        0x00000005
#define EFI_ACPI_5_0_CSRT_REVISION    0x00000000

// Resource Descriptor Types
#define EFI_ACPI_CSRT_RD_TYPE_INTERRUPT 1
#define EFI_ACPI_CSRT_RD_TYPE_TIMER 2
#define EFI_ACPI_CSRT_RD_TYPE_DMA 3
#define EFI_ACPI_CSRT_RD_TYPE_CACHE 4

// Resource Descriptor Subtypes
#define EFI_ACPI_CSRT_RD_SUBTYPE_INTERRUPT_LINES 0
#define EFI_ACPI_CSRT_RD_SUBTYPE_INTERRUPT_CONTROLLER 1
#define EFI_ACPI_CSRT_RD_SUBTYPE_TIMER 0
#define EFI_ACPI_CSRT_RD_SUBTYPE_DMA_CHANNEL 0
#define EFI_ACPI_CSRT_RD_SUBTYPE_DMA_CONTROLLER 1
#define EFI_ACPI_CSRT_RD_SUBTYPE_CACHE 0

#pragma pack(push, 1)
//------------------------------------------------------------------------
// CSRT Resource Group header 24 bytes long
//------------------------------------------------------------------------
typedef struct {
  UINT32 Length;
  UINT32 VendorID;
  UINT32 SubVendorId;
  UINT16 DeviceId;
  UINT16 SubdeviceId;
  UINT16 Revision;
  UINT16 Reserved;
  UINT32 SharedInfoLength;
} EFI_ACPI_5_0_CSRT_RESOURCE_GROUP_HEADER;

//------------------------------------------------------------------------
// CSRT Resource Descriptor 12 bytes total
//------------------------------------------------------------------------
typedef struct {
  UINT32 Length;
  UINT16 ResourceType;
  UINT16 ResourceSubType;
  UINT32 UID;
} EFI_ACPI_5_0_CSRT_RESOURCE_DESCRIPTOR_HEADER;
#pragma pack (pop)

#endif // !_PLATFORM_IMX_H_
