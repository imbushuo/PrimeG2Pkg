/*
* iMX6 ULL DSDT
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
*/
#include "Dsdt-Common.h"

DefinitionBlock ("DSDT.aml", "DSDT", 5, "MSFT", "EDK2", 1)
{
  Scope (\_SB_)
  {
    include("Dsdt-Platform.asl")
    include("Dsdt-Gpio.asl")
    include("Dsdt-I2c.asl")
    include("Dsdt-Uart.asl")
    include("Dsdt-Usb.asl")
  } // \_SB_
}
