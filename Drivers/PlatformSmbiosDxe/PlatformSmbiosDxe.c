/** @file

  Static SMBIOS Table for the SolidRun HummingBoard-Edge iMX6 Quad platform
  Derived from EmulatorPkg package

  Note SMBIOS 2.7.1 Required structures:
    BIOS Information (Type 0)
    System Information (Type 1)
    Board Information (Type 2)
    System Enclosure (Type 3)
    Processor Information (Type 4) - CPU Driver
    Cache Information (Type 7) - For cache that is external to processor
    Physical Memory Array (Type 16)
    Memory Device (Type 17) - For each socketed system-memory Device
    Memory Array Mapped Address (Type 19) - One per contiguous block per Physical Memroy Array
    System Boot Information (Type 32)

  Copyright (c) 2006 - 2011, Intel Corporation. All rights reserved.<BR>
  Copyright (c) 2012, Apple Inc. All rights reserved.<BR>
  Copyright (c) 2015, Linaro Limited. All rights reserved.<BR>
  Copyright (c) 2015, Hisilicon Limited. All rights reserved.<BR>
  Copyright (c) 2015, ARM Limited. All rights reserved.
  Copyright (c) 2018 Microsoft Corporation. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Guid/SmBios.h>

#include <IndustryStandard/SmBios.h>

#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Library/UefiLib.h>

#include <Protocol/Smbios.h>

#include "PlatformSmbiosDxe.h"

// Default SMBIOS Tables for i.MX6
SMBIOS_TABLE_TYPE0 mBiosInfoType0 = {
  {
    EFI_SMBIOS_TYPE_BIOS_INFORMATION,
    sizeof (SMBIOS_TABLE_TYPE0),
    SMBIOS_HANDLE_PI_RESERVED
  },
  1,                    // Vendor String
  2,                    // BiosVersion String
  0xE000,               // BiosSegment
  3,                    // BiosReleaseDate String
  (FixedPcdGet32 (PcdFdSize) - 1) / SIZE_64KB, // BiosSize
  {       // BiosCharacteristics
    0,    //  Reserved                          :2;
    0,    //  Unknown                           :1;
    0,    //  BiosCharacteristicsNotSupported   :1;
    0,    //  IsaIsSupported                    :1;
    0,    //  McaIsSupported                    :1;
    0,    //  EisaIsSupported                   :1;
    1,    //  PciIsSupported                    :1;
    0,    //  PcmciaIsSupported                 :1;
    0,    //  PlugAndPlayIsSupported            :1;
    0,    //  ApmIsSupported                    :1;
    1,    //  BiosIsUpgradable                  :1;
    0,    //  BiosShadowingAllowed              :1;
    0,    //  VlVesaIsSupported                 :1;
    0,    //  EscdSupportIsAvailable            :1;
    0,    //  BootFromCdIsSupported             :1;
    1,    //  SelectableBootIsSupported         :1;
    0,    //  RomBiosIsSocketed                 :1;
    0,    //  BootFromPcmciaIsSupported         :1;
    0,    //  EDDSpecificationIsSupported       :1;
    0,    //  JapaneseNecFloppyIsSupported      :1;
    0,    //  JapaneseToshibaFloppyIsSupported  :1;
    0,    //  Floppy525_360IsSupported          :1;
    0,    //  Floppy525_12IsSupported           :1;
    0,    //  Floppy35_720IsSupported           :1;
    0,    //  Floppy35_288IsSupported           :1;
    0,    //  PrintScreenIsSupported            :1;
    0,    //  Keyboard8042IsSupported           :1;
    0,    //  SerialIsSupported                 :1;
    0,    //  PrinterIsSupported                :1;
    0,    //  CgaMonoIsSupported                :1;
    0,    //  NecPc98                           :1;
    0     //  ReservedForVendor                 :32;
  },
  {       // BIOSCharacteristicsExtensionBytes[]
    0x01, //  AcpiIsSupported                   :1;
          //  UsbLegacyIsSupported              :1;
          //  AgpIsSupported                    :1;
          //  I2OBootIsSupported                :1;
          //  Ls120BootIsSupported              :1;
          //  AtapiZipDriveBootIsSupported      :1;
          //  Boot1394IsSupported               :1;
          //  SmartBatteryIsSupported           :1;
    0x0C, //  BiosBootSpecIsSupported           :1;
          //  FunctionKeyNetworkBootIsSupported :1;
          //  TargetContentDistributionEnabled  :1;
          //  UefiSpecificationSupported        :1;
          //  VirtualMachineSupported           :1;
          //  ExtensionByte2Reserved            :3;
  },
  FixedPcdGet32 (PcdFirmwareRevision) >> 16, // SystemBiosMajorRelease
  FixedPcdGet32 (PcdFirmwareRevision) & 0xff, // SystemBiosMinorRelease
  0xFF,   // EmbeddedControllerFirmwareMajorRelease
  0xFF,   // EmbeddedControllerFirmwareMinorRelease
};

SMBIOS_TABLE_TYPE1 mSysInfoType1 = {
  {
    EFI_SMBIOS_TYPE_SYSTEM_INFORMATION,
    sizeof (SMBIOS_TABLE_TYPE1),
    SMBIOS_HANDLE_PI_RESERVED
  },
  1,    // Manufacturer String
  2,    // ProductName String
  3,    // Version String
  4,    // SerialNumber String
  { 0xFFFFFFFF, 0xFFFF, 0xFFFF, { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } },
  SystemWakeupTypePowerSwitch, // WakeUp Type
  5,    // SKUNumber String
  6,    // Family String
};

SMBIOS_TABLE_TYPE2 mBoardInfoType2 = {
  {
    EFI_SMBIOS_TYPE_BASEBOARD_INFORMATION,
    sizeof (SMBIOS_TABLE_TYPE2),
    SMBIOS_HANDLE_BOARD
  },
  1,      // Manufacturer String
  2,      // ProductName String
  3,      // Version String
  4,      // SerialNumber String
  5,      // AssetTag String
  {       // FeatureFlag
    1,    //  Motherboard           :1;
    0,    //  RequiresDaughterCard  :1;
    0,    //  Removable             :1;
    0,    //  Replaceable           :1;
    0,    //  HotSwappable          :1;
    0,    //  Reserved              :3;
  },
  6,                        // LocationInChassis String
  SMBIOS_HANDLE_CHASSIS,    // ChassisHandle;
  BaseBoardTypeMotherBoard, // BoardType;
  1,                        // NumberOfContainedObjectHandles;
  { SMBIOS_HANDLE_PROCESSOR } // ContainedObjectHandles[1];
};

SMBIOS_TABLE_TYPE3 mEnclosureInfoType3 = {
  {
    EFI_SMBIOS_TYPE_SYSTEM_ENCLOSURE,
    sizeof (SMBIOS_TABLE_TYPE3),
    SMBIOS_HANDLE_CHASSIS
  },
  1,                          // Manufacturer String
  MiscChassisTypeOther,       // Type;
  2,                          // Version String
  3,                          // SerialNumber String
  4,                          // AssetTag String
  ChassisStateSafe,           // BootupState;
  ChassisStateSafe,           // PowerSupplyState;
  ChassisStateSafe,           // ThermalState;
  ChassisSecurityStatusNone,  // SecurityStatus;
  { 0, 0, 0, 0 },             // OemDefined[4];
  0,                          // Height;
  0,                          // NumberofPowerCords;
  0,                          // ContainedElementCount;
  0,                          // ContainedElementRecordLength;
  { { 0 } },                  // ContainedElements[1];
};

SMBIOS_TABLE_TYPE4 mProcessorInfoType4 = {
  {
    EFI_SMBIOS_TYPE_PROCESSOR_INFORMATION,
    sizeof (SMBIOS_TABLE_TYPE4),
    SMBIOS_HANDLE_PROCESSOR
  },
  1,                                // Socket String;
  CentralProcessor,                 // ProcessorType;
  ProcessorFamilyIndicatorFamily2,  // ProcessorFamily;
  2,                                // ProcessorManufacture String;
  {{0,},{0.}}, // ProcessorId;
  3,      // ProcessorVersion String;
  {       // Voltage;
    1,    // ProcessorVoltageCapability5V        :1;
    1,    // ProcessorVoltageCapability3_3V      :1;
    1,    // ProcessorVoltageCapability2_9V      :1;
    0,    // ProcessorVoltageCapabilityReserved  :1;
    0,    // ProcessorVoltageReserved            :3;
    0     // ProcessorVoltageIndicateLegacy      :1;
  },
  0,                      // ExternalClock;
  0,                      // MaxSpeed;
  0,                      // CurrentSpeed;
  0x41,                   // Status;
  ProcessorUpgradeOther,  // ProcessorUpgrade;
  SMBIOS_HANDLE_L1I,      // L1CacheHandle;
  SMBIOS_HANDLE_L2U,      // L2CacheHandle;
  0xFFFF,                 // L3CacheHandle;
  4,                      // SerialNumber;
  5,                      // AssetTag;
  6,                      // PartNumber;
  0,                      // CoreCount;
  0,                      // EnabledCoreCount;
  0,                      // ThreadCount;
  0,                      // ProcessorCharacteristics;
  ProcessorFamilyARMv7,   // ProcessorFamily2;
};

SMBIOS_TABLE_TYPE7 mCacheInfoType7L1I = {
  {
    EFI_SMBIOS_TYPE_CACHE_INFORMATION,
    sizeof (SMBIOS_TABLE_TYPE7),
    SMBIOS_HANDLE_L1I
  },
  1,      // SocketDesignation String
  0x0180, // Cache Configuration
  0x0020, // Maximum Size 32k
  0x0020, // Install Size 32k
  {       // Supported SRAM Type
    0,    //Other             :1
    0,    //Unknown           :1
    0,    //NonBurst          :1
    1,    //Burst             :1
    0,    //PiplelineBurst    :1
    1,    //Synchronous       :1
    0,    //Asynchronous      :1
    0     //Reserved          :9
  },
  {       // Current SRAM Type
    0,    //Other             :1
    0,    //Unknown           :1
    0,    //NonBurst          :1
    1,    //Burst             :1
    0,    //PiplelineBurst    :1
    1,    //Synchronous       :1
    0,    //Asynchronous      :1
    0     //Reserved          :9
  },
  0,                      // Cache Speed unknown
  CacheErrorMultiBit,     // Error Correction Multi
  CacheTypeInstruction,   // System Cache Type
  CacheAssociativity2Way  // Associativity
};

SMBIOS_TABLE_TYPE7 mCacheInfoType7L1D = {
  {
    EFI_SMBIOS_TYPE_CACHE_INFORMATION,
    sizeof (SMBIOS_TABLE_TYPE7),
    SMBIOS_HANDLE_L1D
  },
  1,      // SocketDesignation String
  0x0180, // Cache Configuration
  0x0020, // Maximum Size 32k
  0x0020, // Install Size 32k
  {       // Supported SRAM Type
    0,    //Other             :1
    0,    //Unknown           :1
    0,    //NonBurst          :1
    1,    //Burst             :1
    0,    //PiplelineBurst    :1
    1,    //Synchronous       :1
    0,    //Asynchronous      :1
    0     //Reserved          :9
  },
  {       // Current SRAM Type
    0,    //Other             :1
    0,    //Unknown           :1
    0,    //NonBurst          :1
    1,    //Burst             :1
    0,    //PiplelineBurst    :1
    1,    //Synchronous       :1
    0,    //Asynchronous      :1
    0     //Reserved          :9
  },
  0,                      // Cache Speed unknown
  CacheErrorMultiBit,     // Error Correction Multi
  CacheTypeData,          // System Cache Type
  CacheAssociativity2Way  // Associativity
};

SMBIOS_TABLE_TYPE7 mCacheInfoType7L2U = {
  {
    EFI_SMBIOS_TYPE_CACHE_INFORMATION,
    sizeof (SMBIOS_TABLE_TYPE7),
    SMBIOS_HANDLE_L2U
  },
  1,      // SocketDesignation String
  0x0181, // Cache Configuration
  0,      // Maximum Size
  0,      // Install Size
  {       // Supported SRAM Type
    0,    //Other             :1
    0,    //Unknown           :1
    0,    //NonBurst          :1
    1,    //Burst             :1
    0,    //PiplelineBurst    :1
    1,    //Synchronous       :1
    0,    //Asynchronous      :1
    0     //Reserved          :9
  },
  {       // Current SRAM Type
    0,    //Other             :1
    0,    //Unknown           :1
    0,    //NonBurst          :1
    1,    //Burst             :1
    0,    //PiplelineBurst    :1
    1,    //Synchronous       :1
    0,    //Asynchronous      :1
    0     //Reserved          :9
  },
  0,                      // Cache Speed unknown
  CacheErrorMultiBit,     // Error Correction Multi
  CacheTypeUnified,       // System Cache Type
  CacheAssociativity2Way  // Associativity
};

SMBIOS_TABLE_TYPE16 mPhysicalMemoryArrayInfoType16 = {
  {
    EFI_SMBIOS_TYPE_PHYSICAL_MEMORY_ARRAY,
    sizeof (SMBIOS_TABLE_TYPE16),
    SMBIOS_HANDLE_MEMORY_ARRAY
  },
  MemoryArrayLocationSystemBoard, // Location;
  MemoryArrayUseSystemMemory,     // Use;
  MemoryErrorCorrectionNone,      // MemoryErrorCorrection;
  0x80000000,                     // MaximumCapacity
  0xFFFE,                         // MemoryErrorInformationHandle;
  1,                              // NumberOfMemoryDevices;
  0x80000000ULL,                  // ExtendedMaximumCapacity
};

SMBIOS_TABLE_TYPE17 mMemoryDeviceInfoType17 = {
  {
    EFI_SMBIOS_TYPE_MEMORY_DEVICE,
    sizeof (SMBIOS_TABLE_TYPE17),
    SMBIOS_HANDLE_MEMORY_DEVICE
  },
  SMBIOS_HANDLE_MEMORY_ARRAY, // MemoryArrayHandle;
  0xFFFE,     // MemoryErrorInformationHandle;
  0xFFFF,     // TotalWidth;
  0xFFFF,     // DataWidth;
  0xFFFF,     // Size; When bit 15 is 0: Size in MB
              // When bit 15 is 1: Size in KB, and continues in ExtendedSize
  MemoryFormFactorRowOfChips,  // FormFactor;
  0,                        // DeviceSet;
  1,                        // DeviceLocator String
  2,                        // BankLocator String
  MemoryTypeDdr3,           // MemoryType;
  {           // TypeDetail;
    0,        // Reserved        :1;
    0,        // Other           :1;
    0,        // Unknown         :1;
    0,        // FastPaged       :1;
    0,        // StaticColumn    :1;
    0,        // PseudoStatic    :1;
    0,        // Rambus          :1;
    0,        // Synchronous     :1;
    0,        // Cmos            :1;
    0,        // Edo             :1;
    0,        // WindowDram      :1;
    0,        // CacheDram       :1;
    0,        // Nonvolatile     :1;
    0,        // Registered      :1;
    1,        // Unbuffered      :1;
    0,        // Reserved1       :1;
  },
  0,          // Speed;
  0,          // Manufacturer String
  0,          // SerialNumber String
  0,          // AssetTag String
  0,          // PartNumber String
  0,          // Attributes;
  0,          // ExtendedSize;
  0,          // ConfiguredMemoryClockSpeed;
};

SMBIOS_TABLE_TYPE19 mMemoryArrayMappedInfoType19 = {
  {
    EFI_SMBIOS_TYPE_MEMORY_ARRAY_MAPPED_ADDRESS,
    sizeof (SMBIOS_TABLE_TYPE19),
    SMBIOS_HANDLE_PI_RESERVED
  },
  0xFFFFFFFF, // StartingAddress;
  0xFFFFFFFF, // EndingAddress;
  SMBIOS_HANDLE_MEMORY_ARRAY, // MemoryArrayHandle;
  1,          // PartitionWidth;
  0xFFFFFFFF, // ExtendedStartingAddress;
  0xFFFFFFFF, // ExtendedEndingAddress;
};

SMBIOS_TABLE_TYPE32 mBootInfoType32 = {
  {
    EFI_SMBIOS_TYPE_SYSTEM_BOOT_INFORMATION,
    sizeof (SMBIOS_TABLE_TYPE32),
    SMBIOS_HANDLE_PI_RESERVED
  },
  { 0, 0, 0, 0, 0, 0 },         // Reserved[6];
  BootInformationStatusNoError  // BootStatus
};

struct MonthDescription {
  CONST CHAR8* MonthStr;
  UINT32    MonthInt;
} gMonthDescription[] = {
  { "Jan", 1 },
  { "Feb", 2 },
  { "Mar", 3 },
  { "Apr", 4 },
  { "May", 5 },
  { "Jun", 6 },
  { "Jul", 7 },
  { "Aug", 8 },
  { "Sep", 9 },
  { "Oct", 10 },
  { "Nov", 11 },
  { "Dec", 12 },
  { "???", 1 },  // Use 1 as default month
};

EFI_STATUS
LogSmbiosData (
  IN       UINT8                      *Buffer,
  IN  OUT  EFI_SMBIOS_HANDLE          *SmbiosHandle
  )
{
  EFI_STATUS         Status;
  EFI_SMBIOS_PROTOCOL       *Smbios;

  *SmbiosHandle = ((EFI_SMBIOS_TABLE_HEADER *)Buffer)->Handle;
  Status = gBS->LocateProtocol (&gEfiSmbiosProtocolGuid, NULL, (VOID **)&Smbios);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = Smbios->Add (
                    Smbios,
                    NULL,
                    SmbiosHandle,
                    (EFI_SMBIOS_TABLE_HEADER *)Buffer
                    );

  return Status;
}

VOID
GetReleaseTime (
  EFI_TIME *Time
  )
{
  CONST CHAR8      *ReleaseDate = __DATE__;
  CONST CHAR8      *ReleaseTime = __TIME__;
  UINTN            i;

  for (i = 0; i < 12; i++) {
    if (0 == AsciiStrnCmp (ReleaseDate, gMonthDescription[i].MonthStr, 3)) {
      break;
    }
  }
  Time->Month = gMonthDescription[i].MonthInt;
  Time->Day = AsciiStrDecimalToUintn (ReleaseDate + 4);
  Time->Year = AsciiStrDecimalToUintn (ReleaseDate + 7);
  Time->Hour = AsciiStrDecimalToUintn (ReleaseTime);
  Time->Minute = AsciiStrDecimalToUintn (ReleaseTime + 3);
  Time->Second = AsciiStrDecimalToUintn (ReleaseTime + 6);

  return;
}

UINT64
GetImx6SerialNumber (
  VOID
  )
{
  UINT64 ProcessorSerialNumber;

  ProcessorSerialNumber = ((UINT64)MmioRead32 (OCOTP_BANK_0_WORD_2)) << 32;
  ProcessorSerialNumber |=(UINT64)MmioRead32 (OCOTP_BANK_0_WORD_1);

  DEBUG ((DEBUG_INFO, "iMX6 Serial Number %08X%08Xh \r\n",
          (UINT32) (ProcessorSerialNumber >> 32), (UINT32)ProcessorSerialNumber));
  return ProcessorSerialNumber;
}

EFI_STATUS
BiosInfoUpdateSmbiosType0 (
  VOID
  )
{
  CHAR8                 *OptionalStrStart;
  CHAR16                *ReleaseDate;
  SMBIOS_TABLE_TYPE0    *SmbiosRecord;
  CHAR16                *Vendor;
  CHAR16                *Version;
  UINTN                 ReleaseDateLen;
  UINT32                Revision;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  EFI_STATUS            Status;
  UINTN                 VendorLen;
  UINTN                 VersionLen;
  EFI_TIME              Time;

  Version = NULL;
  ReleaseDate = NULL;
  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE0);

  // 04h - Vendor Name String
  Vendor = (CHAR16 *)FixedPcdGetPtr (PcdFirmwareVendor);
  VendorLen = StrLen (Vendor);
  if (VendorLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdFirmwareVendor not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += VendorLen + 1;

  // 05h - BIOS Version string
  Revision = FixedPcdGet32 (PcdFirmwareRevision);
  if (Revision == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdFirmwareRevision not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  GetReleaseTime (&Time);

  Version = AllocateZeroPool ((sizeof (CHAR16)) * SMBIOS_STRING_MAX_LENGTH);
  if (Version == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  (VOID)UnicodeSPrintAsciiFormat (
    Version,
    (sizeof (CHAR16)) * SMBIOS_STRING_MAX_LENGTH,
    "\nBoot firmware (version %x built on %t)\n",
    Revision,
    &Time
    );
  VersionLen = StrLen(Version);
  SmbiosRecordLen += VersionLen + 1;

  // 08h - BIOS Release Date must be in mm/dd/yyyy format
  ReleaseDate = AllocateZeroPool ((sizeof (CHAR16)) * SMBIOS_STRING_MAX_LENGTH);
  if (ReleaseDate == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  (VOID)UnicodeSPrintAsciiFormat ( ReleaseDate,
                        (sizeof (CHAR16)) * SMBIOS_STRING_MAX_LENGTH,
                        "%02d/%02d/%4d",
                        Time.Month,
                        Time.Day,
                        Time.Year
                        );
  ReleaseDateLen = StrLen(ReleaseDate);
  SmbiosRecordLen += ReleaseDateLen + 1;

  // Record must end in double NULL
  SmbiosRecordLen += 1;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mBiosInfoType0, sizeof (SMBIOS_TABLE_TYPE0));

  // Populate optional string area at the end of the entry
  OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
  UnicodeStrToAsciiStr (Vendor, OptionalStrStart);
  OptionalStrStart += VendorLen + 1;
  UnicodeStrToAsciiStr (Version, OptionalStrStart);
  OptionalStrStart += VersionLen + 1;
  UnicodeStrToAsciiStr (ReleaseDate, OptionalStrStart);

  // Commit SMBIOS entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (Version != NULL) {
    FreePool (Version);
  }
  if (ReleaseDate != NULL) {
    FreePool (ReleaseDate);
  }
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
SysInfoUpdateSmbiosType1 (
  VOID
  )
{
  CHAR16                *Family;
  CHAR16                *Manufacturer;
  CHAR8                 *OptionalStrStart;
  CHAR16                *ProductName;
  CHAR16                *SerialNumber;
  CHAR16                *SkuNumber;
  SMBIOS_TABLE_TYPE1    *SmbiosRecord;
  EFI_GUID              *SystemUuidFromPcd;
  CHAR16                *Version;
  UINT64                ProcessorSerialNumber;
  UINTN                 FamilyLen;
  UINTN                 ManufacturerLen;
  UINTN                 ProductNameLen;
  UINTN                 SerialNumberLen;
  UINTN                 SkuNumberLen;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  EFI_STATUS            Status;
  UINTN                 VersionLen;

  SerialNumber = NULL;
  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE1);

  // 04h - Manufacturer String
  Manufacturer = L"HP";
  ManufacturerLen = StrLen (Manufacturer);
  if (ManufacturerLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdSystemManufacturer not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += ManufacturerLen + 1;

  // 05h - Product Name String
  ProductName = L"HP Prime Calculator G2";
  ProductNameLen = StrLen (ProductName);
  if (ProductNameLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdSystemProductName not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += ProductNameLen + 1;

  // 06h - Version String
  Version = L"Rev D";
  VersionLen = StrLen (Version);
  if (VersionLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdSystemVersionNumber not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += VersionLen + 1;

  // 07h - Serial Number String
  ProcessorSerialNumber = GetImx6SerialNumber ();
  SerialNumber = AllocateZeroPool (sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH);
  if (SerialNumber == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  (VOID)UnicodeSPrintAsciiFormat (SerialNumber,
                                  sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH,
                                  "%08X%08X",
                                  (UINT32)(ProcessorSerialNumber >> 32),
                                  (UINT32)ProcessorSerialNumber
                                  );
  SerialNumberLen = StrLen (SerialNumber);
  SmbiosRecordLen += SerialNumberLen + 1;

  // 08h - UUID
  // RFC4122 - Initial UUID must be the same across all boards of the same type
  //   And the last 6 bytes are to be populated with system's MAC Address
  SystemUuidFromPcd = (EFI_GUID *)FixedPcdGetPtr (PcdSystemUuid);
  mSysInfoType1.Uuid = *SystemUuidFromPcd;

  // 19h - SKU Number String
  SkuNumber = L"Rev D";
  SkuNumberLen = StrLen (SkuNumber);
  if (SkuNumberLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdSystemSkuNumber not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += SkuNumberLen + 1;

  // 1Ah - Family String
  Family = L"HP Prime";
  FamilyLen = StrLen (Family);
  if (FamilyLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdSystemFamily not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += FamilyLen + 1;

  // Record must end in double NULL
  SmbiosRecordLen += 1;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mSysInfoType1, sizeof (SMBIOS_TABLE_TYPE1));

  // Populate optional string area at the end of the entry
  OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
  UnicodeStrToAsciiStr (Manufacturer, OptionalStrStart);
  OptionalStrStart += ManufacturerLen + 1;
  UnicodeStrToAsciiStr (ProductName, OptionalStrStart);
  OptionalStrStart += ProductNameLen + 1;
  UnicodeStrToAsciiStr (Version, OptionalStrStart);
  OptionalStrStart += VersionLen + 1;
  UnicodeStrToAsciiStr (SerialNumber, OptionalStrStart);
  OptionalStrStart += SerialNumberLen + 1;
  UnicodeStrToAsciiStr (SkuNumber, OptionalStrStart);
  OptionalStrStart += SkuNumberLen + 1;
  UnicodeStrToAsciiStr (Family, OptionalStrStart);

  // Commit SMBIOS Entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (SerialNumber != NULL) {
    FreePool (SerialNumber);
  }
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
BoardInfoUpdateSmbiosType2 (
  VOID
  )
{
  CHAR16                *AssetTag;
  CHAR16                *Location;
  CHAR16                *Manufacturer;
  CHAR8                 *OptionalStrStart;
  CHAR16                *ProductName;
  CHAR16                *SerialNumber;
  SMBIOS_TABLE_TYPE2    *SmbiosRecord;
  CHAR16                *Version;
  UINTN                 AssetTagLen;
  UINT64                ProcessorSerialNumber;
  UINTN                 LocationLen;
  UINTN                 ManufacturerLen;
  UINTN                 ProductNameLen;
  UINTN                 SerialNumberLen;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  EFI_STATUS            Status;
  UINTN                 VersionLen;

  SerialNumber = NULL;
  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE2);

  // 04h - Manufacturer String
  Manufacturer = L"HP";
  ManufacturerLen = StrLen (Manufacturer);
  if (ManufacturerLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdBoardManufacturer not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += ManufacturerLen + 1;

  // 05h - Product Name String
  ProductName = L"HP Prime G2 Calculator";
  ProductNameLen = StrLen (ProductName);
  if (ProductNameLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdBoardProductName not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += ProductNameLen + 1;

  // 06h - Version String
  Version = L"Rev D";
  VersionLen = StrLen (Version);
  if (VersionLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdBoardVersionNumber not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += VersionLen + 1;

  // 07h - Serial Number String
  ProcessorSerialNumber = GetImx6SerialNumber ();
  SerialNumber = AllocateZeroPool (sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH);
  if (SerialNumber == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  (VOID)UnicodeSPrintAsciiFormat (SerialNumber,
                                  sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH,
                                  "%08X%08X",
                                  (UINT32)(ProcessorSerialNumber >> 32),
                                  (UINT32)ProcessorSerialNumber
                                  );
  SerialNumberLen = StrLen (SerialNumber);
  SmbiosRecordLen += SerialNumberLen + 1;

  // 08h - Asset Tag String
  AssetTag = L"No tag";
  AssetTagLen = StrLen (AssetTag);
  if (AssetTagLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdBoardAssetTag not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += AssetTagLen + 1;

  // 0Ah - Location in Chassis String
  Location = L"Open Board";
  LocationLen = StrLen (Location);
  if (LocationLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdBoardLocationInChassis not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += LocationLen + 1;

  // Record must end in double NULL
  SmbiosRecordLen += 1;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mBoardInfoType2, sizeof (SMBIOS_TABLE_TYPE2));

  // Populate optional string area at the end of the entry
  OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
  UnicodeStrToAsciiStr (Manufacturer, OptionalStrStart);
  OptionalStrStart += ManufacturerLen + 1;
  UnicodeStrToAsciiStr (ProductName, OptionalStrStart);
  OptionalStrStart += ProductNameLen + 1;
  UnicodeStrToAsciiStr (Version, OptionalStrStart);
  OptionalStrStart += VersionLen + 1;
  UnicodeStrToAsciiStr (SerialNumber, OptionalStrStart);
  OptionalStrStart += SerialNumberLen + 1;
  UnicodeStrToAsciiStr (AssetTag, OptionalStrStart);
  OptionalStrStart += AssetTagLen + 1;
  UnicodeStrToAsciiStr (Location, OptionalStrStart);

  // Commit SMBIOS Entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (SerialNumber != NULL) {
    FreePool (SerialNumber);
  }
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
EnclosureInfoUpdateSmbiosType3 (
  VOID
  )
{
  CHAR16                *AssetTag;
  CHAR16                *Manufacturer;
  CHAR8                 *OptionalStrStart;
  CHAR16                *SerialNumber;
  SMBIOS_TABLE_TYPE3    *SmbiosRecord;
  CHAR16                *Version;
  UINTN                 AssetTagLen;
  UINT64                ProcessorSerialNumber;
  UINTN                 ManufacturerLen;
  UINTN                 SerialNumberLen;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  EFI_STATUS            Status;
  UINTN                 VersionLen;

  SerialNumber = NULL;
  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE3);

  // 04h - Manufacturer String
  Manufacturer = L"HP";
  ManufacturerLen = StrLen (Manufacturer);
  if (ManufacturerLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdChassisManufacturer not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += ManufacturerLen + 1;

  // 06h - Version String
  Version = L"Rev D";
  VersionLen = StrLen (Version);
  if (VersionLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdChassisVersionNumber not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += VersionLen + 1;

  // 07h - Serial Number String
  ProcessorSerialNumber = GetImx6SerialNumber ();
  SerialNumber = AllocateZeroPool (sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH);
  if (SerialNumber == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  (VOID)UnicodeSPrintAsciiFormat (SerialNumber,
                                  sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH,
                                  "%08X%08X",
                                  (UINT32)(ProcessorSerialNumber >> 32),
                                  (UINT32)ProcessorSerialNumber
                                  );
  SerialNumberLen = StrLen (SerialNumber);
  SmbiosRecordLen += SerialNumberLen + 1;

  // 08h - Asset Tag String
  AssetTag = L"No tag";
  AssetTagLen = StrLen (AssetTag);
  if (AssetTagLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdChassisAssetTag not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += AssetTagLen + 1;

  // Record must end in double NULL
  SmbiosRecordLen += 1;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mEnclosureInfoType3, sizeof (SMBIOS_TABLE_TYPE3));

  // Populate optional string area at the end of the entry
  OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
  UnicodeStrToAsciiStr (Manufacturer, OptionalStrStart);
  OptionalStrStart += ManufacturerLen + 1;
  UnicodeStrToAsciiStr (Version, OptionalStrStart);
  OptionalStrStart += VersionLen + 1;
  UnicodeStrToAsciiStr (SerialNumber, OptionalStrStart);
  OptionalStrStart += SerialNumberLen + 1;
  UnicodeStrToAsciiStr (AssetTag, OptionalStrStart);

  // Commit SMBIOS Entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (SerialNumber != NULL) {
    FreePool (SerialNumber);
  }
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
ProcessorInfoUpdateSmbiosType4 (
  IN  UINTN   MaxCpus
  )
{
  CHAR16                *AssetTag;
  CHAR16                *Manufacturer;
  CHAR8                 *OptionalStrStart;
  CHAR16                *PartNumber;
  CHAR16                *SerialNumber;
  SMBIOS_TABLE_TYPE4    *SmbiosRecord;
  CHAR16                *SocketDesignation;
  CHAR16                *Version;
  UINT32                ArmMidr;
  UINTN                 AssetTagLen;
  UINTN                 i;
  UINT32                Midr;
  UINT64                ProcessorSerialNumber;
  UINTN                 ManufacturerLen;
  UINTN                 PartNumberLen;
  UINTN                 SerialNumberLen;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  UINTN                 SocketDesignationLen;
  EFI_STATUS            Status;
  UINTN                 VersionLen;

  SerialNumber = NULL;
  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE4);
  Midr = 0;

  // 04h - Socket Designation String
  SocketDesignation = L"FCPBGA";
  SocketDesignationLen = StrLen (SocketDesignation);
  if (SocketDesignationLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdProcessorSocketDesignation not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += SocketDesignationLen + 1;

  // 07h - Processor Manufacturer String
  Manufacturer = L"NXP";
  ManufacturerLen = StrLen (Manufacturer);
  if (ManufacturerLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdProcessorManufacturer not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += ManufacturerLen + 1;

  // 08h - Processor ID - On ARM32, first DWORD is MIDR value. Second DWORD is 0
  ArmMidr = (UINT32)ArmReadMidr ();
  DEBUG ((DEBUG_INFO, "%a: ArmMidr = %x\n", __FUNCTION__, ArmMidr));
  // Need to reverse byte order so the value is populated correctly
  for (i = 0; i < 32; i += 8) {
    Midr |= ((ArmMidr >> i) & 0xFF) << (24-i);
  }
  CopyMem(&mProcessorInfoType4.ProcessorId.Signature, &Midr, sizeof (UINT32));

  // 10h - Processor Version String
  Version = L"1.0";
  VersionLen = StrLen (Version);
  if (VersionLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdProcessorVersionNumber not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += VersionLen + 1;

  // 14h - Max Speed
  mProcessorInfoType4.MaxSpeed = 528;
  mProcessorInfoType4.CurrentSpeed = 528;

  // 20h - Processor Serial Number String
  ProcessorSerialNumber = GetImx6SerialNumber ();
  SerialNumber = AllocateZeroPool (sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH);
  if (SerialNumber == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  (VOID)UnicodeSPrintAsciiFormat (SerialNumber,
                                  sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH,
                                  "%08X%08X",
                                  (UINT32)(ProcessorSerialNumber >> 32),
                                  (UINT32)ProcessorSerialNumber
                                  );
  SerialNumberLen = StrLen (SerialNumber);
  SmbiosRecordLen += SerialNumberLen + 1;

  // 21h - Asset Tag String
  AssetTag = L"No tag";
  AssetTagLen = StrLen (AssetTag);
  if (AssetTagLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdProcessorAssetTag not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += AssetTagLen + 1;

  // 22h - Part Number String
  PartNumber = L"i.MX 6ULL";
  PartNumberLen = StrLen (PartNumber);
  if (PartNumberLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdProcessorPartNumber not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += PartNumberLen + 1;

  // 23h - Core Count
  // 24h - Core Enabled
  // 25h - Thread Count
  mProcessorInfoType4.CoreCount        = (UINT8) MaxCpus;
  mProcessorInfoType4.EnabledCoreCount = (UINT8) MaxCpus;
  mProcessorInfoType4.ThreadCount      = (UINT8) MaxCpus;

  // Record must end in double NULL
  SmbiosRecordLen += 1;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mProcessorInfoType4, sizeof (SMBIOS_TABLE_TYPE4));

  // Populate optional string area at the end of the entry
  OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
  UnicodeStrToAsciiStr (SocketDesignation, OptionalStrStart);
  OptionalStrStart += SocketDesignationLen + 1;
  UnicodeStrToAsciiStr (Manufacturer, OptionalStrStart);
  OptionalStrStart += ManufacturerLen + 1;
  UnicodeStrToAsciiStr (Version, OptionalStrStart);
  OptionalStrStart += VersionLen + 1;
  UnicodeStrToAsciiStr (SerialNumber, OptionalStrStart);
  OptionalStrStart += SerialNumberLen + 1;
  UnicodeStrToAsciiStr (AssetTag, OptionalStrStart);
  OptionalStrStart += AssetTagLen + 1;
  UnicodeStrToAsciiStr (PartNumber, OptionalStrStart);

  // Commit SMBIOS Entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (SerialNumber != NULL) {
    FreePool (SerialNumber);
  }
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
CacheInfoUpdateSmbiosType7L1I (
  VOID
  )
{
  CHAR8                 *OptionalStrStart;
  SMBIOS_TABLE_TYPE7    *SmbiosRecord;
  CHAR16                *SocketDesignation;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  UINTN                 SocketDesignationLen;
  EFI_STATUS            Status;

  SocketDesignation = NULL;
  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE7);

  // 04h - Socket Designation String
  SocketDesignation = AllocateZeroPool (sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH);
  if (SocketDesignation == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  (VOID)UnicodeSPrintAsciiFormat (SocketDesignation,
                                  sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH,
                                  "L1 ICache"
                                  );
  SocketDesignationLen = StrLen (SocketDesignation);
  SmbiosRecordLen += SocketDesignationLen + 1;

  // Record must end in double NULL
  SmbiosRecordLen += 1;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mCacheInfoType7L1I, sizeof (SMBIOS_TABLE_TYPE7));

  // Populate optional string area at the end of the entry
  OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
  UnicodeStrToAsciiStr (SocketDesignation, OptionalStrStart);

  // Commit SMBIOS Entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (SocketDesignation != NULL) {
    FreePool (SocketDesignation);
  }
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
CacheInfoUpdateSmbiosType7L1D (
  VOID
  )
{
  CHAR8                 *OptionalStrStart;
  SMBIOS_TABLE_TYPE7    *SmbiosRecord;
  CHAR16                *SocketDesignation;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  UINTN                 SocketDesignationLen;
  EFI_STATUS            Status;

  SocketDesignation = NULL;
  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE7);

  // 04h - Socket Designation String
  SocketDesignation = AllocateZeroPool (sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH);
  if (SocketDesignation == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  (VOID)UnicodeSPrintAsciiFormat (SocketDesignation,
                                  sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH,
                                  "L1 DCache"
                                  );
  SocketDesignationLen = StrLen (SocketDesignation);
  SmbiosRecordLen += SocketDesignationLen + 1;

  // Record must end in double NULL
  SmbiosRecordLen += 1;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mCacheInfoType7L1D, sizeof (SMBIOS_TABLE_TYPE7));

  // Populate optional string area at the end of the entry
  OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
  UnicodeStrToAsciiStr (SocketDesignation, OptionalStrStart);

  // Commit SMBIOS Entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (SocketDesignation != NULL) {
    FreePool (SocketDesignation);
  }
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
CacheInfoUpdateSmbiosType7L2U (
  VOID
  )
{
  CHAR8                 *OptionalStrStart;
  SMBIOS_TABLE_TYPE7    *SmbiosRecord;
  CHAR16                *SocketDesignation;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  UINTN                 SocketDesignationLen;
  EFI_STATUS            Status;

  SocketDesignation = NULL;
  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE7);

  // 04h - Socket Designation String
  SocketDesignation = AllocateZeroPool (sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH);
  if (SocketDesignation == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }
  (VOID)UnicodeSPrintAsciiFormat (SocketDesignation,
                                  sizeof (CHAR16) * SMBIOS_STRING_MAX_LENGTH,
                                  "L2 UCache (PL310)"
                                  );
  SocketDesignationLen = StrLen (SocketDesignation);
  SmbiosRecordLen += SocketDesignationLen + 1;

  // 07h - Maximum Cache Size
  // 09h - Installed Size
  mCacheInfoType7L2U.MaximumCacheSize = 0x80; // 128KB
  mCacheInfoType7L2U.InstalledSize = 0x80; // 128KB

  // Record must end in double NULL
  SmbiosRecordLen += 1;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mCacheInfoType7L2U, sizeof (SMBIOS_TABLE_TYPE7));

  // Populate optional string area at the end of the entry
  OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
  UnicodeStrToAsciiStr (SocketDesignation, OptionalStrStart);

  // Commit SMBIOS Entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (SocketDesignation != NULL) {
    FreePool (SocketDesignation);
  }
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
CacheInfoUpdateSmbiosType7 (
  VOID
  )
{
  EFI_STATUS Status;

  // L1I Table
  Status = CacheInfoUpdateSmbiosType7L1I ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  // L1D Table
  Status = CacheInfoUpdateSmbiosType7L1D ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  // L2U Table
  Status = CacheInfoUpdateSmbiosType7L2U ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

Exit:
  return Status;
}

EFI_STATUS
PhysicalMemoryArrayInfoUpdateSmbiosType16 (
  VOID
  )
{
  SMBIOS_TABLE_TYPE16   *SmbiosRecord;
  UINT32                MaximumCapacity;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  EFI_STATUS            Status;

  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE16);

  // 07h - Maximum Capacity
  MaximumCapacity = FixedPcdGet32 (PcdPhysicalMemoryMaximumCapacity);
  if (MaximumCapacity == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdPhysicalMemoryMaximumCapacity not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  mPhysicalMemoryArrayInfoType16.MaximumCapacity = MaximumCapacity;

  // 0Fh - Extended Maximum Capacity
  mPhysicalMemoryArrayInfoType16.ExtendedMaximumCapacity = 0;

  // Since there are no strings, need to add one extra to the record length
  // in order to have the record end with double NULL.
  SmbiosRecordLen += 2;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mPhysicalMemoryArrayInfoType16, sizeof (SMBIOS_TABLE_TYPE16));

  // Commit SMBIOS Entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
MemoryDeviceInfoUpdateSmbiosType17 (
  VOID
  )
{
  CHAR16                *BankLocation;
  CHAR16                *DeviceLocation;
  CHAR8                 *OptionalStrStart;
  SMBIOS_TABLE_TYPE17   *SmbiosRecord;
  UINTN                 BankLocationLen;
  UINTN                 DeviceLocationLen;
  UINT32                MaximumCapacity;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  EFI_STATUS            Status;

  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE17);

  // 08h - Total Width
  // 0Ah - Data Width
  mMemoryDeviceInfoType17.TotalWidth = 32;
  mMemoryDeviceInfoType17.DataWidth = 32;

  // 0Ch - Size - in MB
  MaximumCapacity = FixedPcdGet32 (PcdPhysicalMemoryMaximumCapacity);
  if (MaximumCapacity == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdPhysicalMemoryMaximumCapacity not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  MaximumCapacity = MaximumCapacity / 1024;
  mMemoryDeviceInfoType17.Size = MaximumCapacity;

  // 10h - Device Locator String
  DeviceLocation = L"On SoM";
  DeviceLocationLen = StrLen (DeviceLocation);
  if (DeviceLocationLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdMemoryDeviceLocation not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += DeviceLocationLen + 1;

  // 11h - Bank Locator String
  BankLocation = L"Bank 0";
  BankLocationLen = StrLen (BankLocation);
  if (BankLocationLen == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdMemoryBankLocation not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  SmbiosRecordLen += BankLocationLen + 1;

  // 15h - Speed
  mMemoryDeviceInfoType17.Speed = 400; // 400 MHz DDR3

  // Record must end in double NULL
  SmbiosRecordLen += 1;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mMemoryDeviceInfoType17, sizeof (SMBIOS_TABLE_TYPE17));

  // Populate optional string area at the end of the entry
  OptionalStrStart = (CHAR8 *)(SmbiosRecord + 1);
  UnicodeStrToAsciiStr (DeviceLocation, OptionalStrStart);
  OptionalStrStart += DeviceLocationLen + 1;
  UnicodeStrToAsciiStr (BankLocation, OptionalStrStart);

  // Commit SMBIOS Entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
MemoryArrayMappingInfoUpdateSmbiosType19 (
  VOID
  )
{
  SMBIOS_TABLE_TYPE19   *SmbiosRecord;
  UINT32                EndAddress;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  UINT32                StartAddress;
  EFI_STATUS            Status;

  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE19);

  // 04h - Starting Address
  StartAddress = 0x80000000;
  if (StartAddress == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdMemoryStartAddress not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  mMemoryArrayMappedInfoType19.StartingAddress = StartAddress;
  mMemoryArrayMappedInfoType19.ExtendedStartingAddress = 0;

  // 08h - Ending Address
  EndAddress = 0x9FFF0000;
  if (EndAddress == 0) {
    DEBUG ((DEBUG_ERROR, "%a: PcdMemoryEndAddress not filled\n", __FUNCTION__));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  if (EndAddress <= StartAddress) {
    DEBUG ((DEBUG_ERROR,
          "%a: Start/End addresses invalid. Start = %x, End = %x\n",
          __FUNCTION__,
          StartAddress,
          EndAddress));
    Status = EFI_INVALID_PARAMETER;
    goto Exit;
  }
  mMemoryArrayMappedInfoType19.EndingAddress = EndAddress;
  mMemoryArrayMappedInfoType19.ExtendedEndingAddress = 0;

  // Since there are no strings, need to add one extra to the record length
  // in order to have the record end with double NULL.
  SmbiosRecordLen += 2;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mMemoryArrayMappedInfoType19, sizeof (SMBIOS_TABLE_TYPE19));

  // Commit SMBIOS Entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
BootInfoUpdateSmbiosType32 (
  VOID
  )
{
  SMBIOS_TABLE_TYPE19   *SmbiosRecord;
  EFI_SMBIOS_HANDLE     SmbiosHandle;
  UINT32                SmbiosRecordLen;
  EFI_STATUS            Status;

  SmbiosRecord = NULL;
  SmbiosRecordLen = sizeof (SMBIOS_TABLE_TYPE32);

  // Since there are no strings, need to add one extra to the record length
  // in order to have the record end with double NULL.
  SmbiosRecordLen += 2;

  // Create new SMBIOS record entry
  SmbiosRecord = AllocateZeroPool (SmbiosRecordLen);
  if (SmbiosRecord == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Exit;
  }

  // Populate entry with default values
  CopyMem (SmbiosRecord, &mBootInfoType32, sizeof (SMBIOS_TABLE_TYPE32));

  // Commit SMBIOS Entry
  Status = LogSmbiosData ((UINT8*)SmbiosRecord, &SmbiosHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Smbios Table Log Failed %x\n", __FUNCTION__, Status));
  }

Exit:
  if (SmbiosRecord != NULL) {
    FreePool (SmbiosRecord);
  }
  return Status;
}

EFI_STATUS
EFIAPI
PlatformSmbiosDriverEntryPoint (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS Status;

  Status = BiosInfoUpdateSmbiosType0 ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  Status = SysInfoUpdateSmbiosType1 ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  Status = BoardInfoUpdateSmbiosType2 ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  Status = EnclosureInfoUpdateSmbiosType3 ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  Status = ProcessorInfoUpdateSmbiosType4 (FixedPcdGet32 (PcdCoreCount));
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  Status = CacheInfoUpdateSmbiosType7 ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  Status = PhysicalMemoryArrayInfoUpdateSmbiosType16 ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  Status = MemoryDeviceInfoUpdateSmbiosType17 ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  Status = MemoryArrayMappingInfoUpdateSmbiosType19 ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  Status = BootInfoUpdateSmbiosType32 ();
  if (EFI_ERROR (Status)) {
    goto Exit;
  }

Exit:
  return Status;
}
