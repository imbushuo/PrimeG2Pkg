/** @file
*
*  Copyright (c) 2015, ARM Limited. All rights reserved.
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

#ifndef _PLATFORM_SMBIOS_DXE_H_
#define _PLATFORM_SMBIOS_DXE_H_

#define OCOTP_BANK_0_WORD_1 0x021BC410
#define OCOTP_BANK_0_WORD_2 0x021BC420

#define GLOBAL_PAGE_SIGNATURE 0x474C424C // 'GLBL'

enum SMBIOS_REFRENCE_HANDLES {
  SMBIOS_HANDLE_BOARD = 0x1000,
  SMBIOS_HANDLE_CHASSIS,
  SMBIOS_HANDLE_PROCESSOR,
  SMBIOS_HANDLE_L1I,
  SMBIOS_HANDLE_L1D,
  SMBIOS_HANDLE_L2U,
  SMBIOS_HANDLE_MEMORY_ARRAY,
  SMBIOS_HANDLE_MEMORY_DEVICE
};

typedef struct {
  UINT32 Signature;
  UINT8 Revision;
  UINT8 reserved[3];
  UINT8 Mac0Id;
  UINT8 Mac0Valid;
  UINT8 MacAddress[6];
} GLOBAL_PAGE_DATA;

#endif  /* _PLATFORM_SMBIOS_DXE_H_ */
