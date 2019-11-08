/** @file
*
*  iMX6 ULL EHCI USB Controllers
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

// Pre-configured to USB Host mode
Device (USB0) {
    Name(_HID, "PNP0D20")
    Name(_UID, 0x0)

    Name (_S0W, 0x0) // D0 is the lowest supported state to wake itself up
    Method (_STA) {
        Return (0xf)
    }

    Method (_CRS, 0x0, Serialized) {
        Name (RBUF, ResourceTemplate () {
        MEMORY32FIXED(ReadWrite, 0x02184100, 0x100, )
        Interrupt (ResourceConsumer, Level, ActiveHigh, SharedAndWake) { 75 }
        })
        Return(RBUF)
    }

    OperationRegion (OTGM, SystemMemory, 0x02184100, 0x100)
    Field (OTGM, WordAcc, NoLock, Preserve) {
        Offset (0x84),  // skip to register 84h
        PTSC, 32,       // port status control
        Offset (0xA8),  // skip to register A8h
        DSBM, 32,       // UOG_USBMOD
    }

    Name (REG, 0x0)    // Declare register read variable
    Method (_UBF, 0x0, Serialized) {
        // Reset handled by driver so no reset required here
        Store (0x03, DSBM);         // set host mode & little endian
        Store (PTSC, REG);          // read PORTSC status
        Store (OR (REG, 0x2), PTSC); // clear current PORTSC status
    }
}
