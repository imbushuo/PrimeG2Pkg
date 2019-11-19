/** @file
*
*  iMX6 Solo UART Controllers
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

Device (UAR1)
{
  Name (_HID, "NXP0106")
  Name (_UID, 0x1)
  Name (_DDN, "UART1")
  Method (_STA) {
    Return (0xf)
  }
  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x02020000, 0x4000, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Shared) { 58 }

    // UART1_TX_DATA - GPIO1 - GPIO1_IO16 - 16
    // UART1_RX_DATA - GPIO1 - GPIO1_IO17 - 17
    // MsftFunctionConfig (Exclusive, PullUp, IMX_ALT5, "\\_SB.GPIO", 0,
    //                     ResourceConsumer, ) { 16, 17 }
    //
    // MsftFunctionConfig (Arg0, Arg1, Arg2, Arg3, Arg4, Arg5, Arg6) { Pin List }
    VendorLong () {
      MSFT_UUID,            // Vendor UUID (MSFT UUID)
      MSFT_FUNCTION_CONFIG, // Resource Identifier (MSFT Function Config)
      0x1d,0x00,            // Length (0xF + sizeof(PinList) + sizeof(ResourceName))
      0x01,                 // Revision (0x1)
      RESOURCECONSUMER_EXCLUSIVE, // Flags (Arg5 | Arg0: ResourceConsumer | Exclusive)
      PULL_UP,              // Pin configuration (Arg1: PullUp)
      IMX_ALT5,0x00,        // Function Number (Arg2: IMX_ALT5)
      PIN_TABLE_OFFSET,     // Pin Table Offset (0x12)
      0x00,                 // Resource Source Index (Arg4: 0)
      0x16,0x00,            // Resource Source Name Offset (0x12 + sizeof(PinList))
      0x20,0x00,            // Vendor Data Offset (0x12 + sizeof(PinList) + sizeof(ResourceName))
      0x00,0x00,            // Vendor Data Length (sizeof(Arg6) = 0)
      0x10,0x00,0x11,0x00,  // Pin List (16, 17)
      SB_GPIO               // Resource Name (Arg3: \_SB.GPIO in ASCII)
    }

    UARTSerialBus (
      115200,
      DataBitsEight,
      StopBitsOne,
      0,                // LinesInUse
      LittleEndian,
      ParityTypeNone,
      FlowControlNone,
      0,
      0,
      "\\_SB.CPU0",
      0,
      ResourceConsumer,
      ,)
  })

  Name (_DSD, Package () {
    ToUUID ("daffd814-6eba-4d8c-8a91-bc9bbf4aa301"),
      Package () {
        Package (2) {"SerCx-FriendlyName", "UART1"}
      }
  })
}
