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

Device (I2C1)
{
  Name (_HID, "NXP0104")
  Name (_UID, 0x1)

  Method (_STA) {
    Return (0xf)
  }

  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x021A0000, 0x14, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 68 }
  })
}

Device (I2C2)
{
  Name (_HID, "NXP0104")
  Name (_UID, 0x2)

  Method (_STA) {
    Return (0xf)
  }

  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x021A4000, 0x14, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 69 }
  })
}

Device (I2C3)
{
  Name (_HID, "NXP0104")
  Name (_UID, 0x3)

  Method (_STA) {
    Return (0xf)
  }

  Name (_CRS, ResourceTemplate () {
    MEMORY32FIXED (ReadWrite, 0x021A8000, 0x14, )
    Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 70 }
  })
}
