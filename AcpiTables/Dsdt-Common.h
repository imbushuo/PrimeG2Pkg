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

// IMX alternate settings codes
#define IMX_ALT0 0x0
#define IMX_ALT1 0x1
#define IMX_ALT2 0x2
#define IMX_ALT3 0x3
#define IMX_ALT4 0x4
#define IMX_ALT5 0x5
#define IMX_ALT6 0x6
#define IMX_ALT7 0x7

// Vendor Macro defines for MsftFunctionConfig
#define MSFT_UUID 0x00,0x60,0x44,0xd5,0xf3,0x1f,0x11,0x60,0x4a,0xb8,0xb0,0x9c,0x2d,0x23,0x30,0xdd,0x2f
#define MSFT_FUNCTION_CONFIG 0x8d
#define RESOURCEPRODUCER_EXCLUSIVE 0x00,0x00
#define RESOURCEPRODUCER_SHARED 0x01,0x00
#define RESOURCECONSUMER_EXCLUSIVE 0x10,0x00
#define RESOURCECONSUMER_SHARED 0x11,0x00
#define PULL_DEFAULT 0x0
#define PULL_UP 0x1
#define PULL_DOWN 0x2
#define PULL_NONE 0x3
#define PIN_TABLE_OFFSET 0x12,0x00
#define SB_GPIO 0x5c,0x5f,0x53,0x42,0x2e,0x47,0x50,0x49,0x4f,0x00 // \_SB.GPIO in ASCII

// IMX SDMA request lines.
// These are logical values, the mapping to the SOC
// actual DMA request lines are done in the HAL extension.
#define SDMA_REQ_VPU 0
#define SDMA_REQ_IPU2 1
#define SDMA_REQ_IPU1 2
#define SDMA_REQ_HDMI_AUDIO 3
#define SDMA_REQ_ECSPI1_RX 4
#define SDMA_REQ_ECSPI1_TX 5
#define SDMA_REQ_ECSPI2_RX 6
#define SDMA_REQ_ECSPI2_TX 7
#define SDMA_REQ_ECSPI3_RX 8
#define SDMA_REQ_ECSPI3_TX 9
#define SDMA_REQ_ECSPI4_RX 10
#define SDMA_REQ_ECSPI4_TX 11
#define SDMA_REQ_ECSPI5_RX 12
#define SDMA_REQ_ECSPI5_TX 13
#define SDMA_REQ_I2C1_RX 14
#define SDMA_REQ_I2C1_TX 15
#define SDMA_REQ_I2C2_RX 16
#define SDMA_REQ_I2C2_TX 17
#define SDMA_REQ_I2C3_RX 18
#define SDMA_REQ_I2C3_TX 19
#define SDMA_REQ_UART1_RX 20
#define SDMA_REQ_UART1_TX 21
#define SDMA_REQ_UART2_RX 22
#define SDMA_REQ_UART2_TX 23
#define SDMA_REQ_UART3_RX 24
#define SDMA_REQ_UART3_TX 25
#define SDMA_REQ_UART4_RX 26
#define SDMA_REQ_UART4_TX 27
#define SDMA_REQ_UART5_RX 28
#define SDMA_REQ_UART5_TX 29
#define SDMA_REQ_SPDIF_RX 30
#define SDMA_REQ_SPDIF_TX 31
#define SDMA_REQ_EPIT1 32
#define SDMA_REQ_EPIT2 33
#define SDMA_REQ_GPT 34
#define SDMA_REQ_ASRC_RXA 35
#define SDMA_REQ_ASRC_RXB 36
#define SDMA_REQ_ASRC_RXC 37
#define SDMA_REQ_ASRC_TXA 38
#define SDMA_REQ_ASRC_TXB 39
#define SDMA_REQ_ASRC_TXC 40
#define SDMA_REQ_ESAI_RX 41
#define SDMA_REQ_ESAI_TX 42
#define SDMA_REQ_ASRC_TXA_2_ESAI_TX 43
#define SDMA_REQ_ASRC_TXB_2_ESAI_TX 44
#define SDMA_REQ_ASRC_TXC_2_ESAI_TX 45
#define SDMA_REQ_SSI1_RX1 46
#define SDMA_REQ_SSI1_TX1 47
#define SDMA_REQ_SSI1_RX0 48
#define SDMA_REQ_SSI1_TX0 49
#define SDMA_REQ_SSI2_RX1 50
#define SDMA_REQ_SSI2_TX1 51
#define SDMA_REQ_SSI2_RX0 52
#define SDMA_REQ_SSI2_TX0 53
#define SDMA_REQ_SSI3_RX1 54
#define SDMA_REQ_SSI3_TX1 55
#define SDMA_REQ_SSI3_RX0 56
#define SDMA_REQ_SSI3_TX0 57
#define SDMA_REQ_EXT1 58
#define SDMA_REQ_EXT2 59
#define SDMA_REQ_UART6_RX 60
#define SDMA_REQ_UART6_TX 61
#define SDMA_REQ_ADC1 62
#define SDMA_REQ_ADC2 63
#define SDMA_REQ_I2C4_RX 64
#define SDMA_REQ_I2C4_TX 65
#define SDMA_REQ_CSI1 66
#define SDMA_REQ_CSI2 67
#define SDMA_REQ_PXP 68
#define SDMA_REQ_LCDIF1 69
#define SDMA_REQ_LCDIF2 70
#define SDMA_REQ_QSPI1_RX 71
#define SDMA_REQ_QSPI1_TX 72
#define SDMA_REQ_QSPI2_RX 73
#define SDMA_REQ_QSPI2_TX 74
#define SDMA_REQ_SAI1_TX 75
#define SDMA_REQ_SAI1_RX 76
#define SDMA_REQ_SAI2_TX 77
#define SDMA_REQ_SAI2_RX 78
