[000h 0000   4]                    Signature : "APIC"    [Multiple APIC Description Table (MADT)]
[004h 0004   4]                 Table Length : 00000094
[008h 0008   1]                     Revision : 08
[009h 0009   1]                     Checksum : 00
[00Ah 0010   6]                       Oem ID : "MCRSFT"
[010h 0016   8]                 Oem Table ID : "IMX6EDK2"
[018h 0024   4]                 Oem Revision : 01000101
[01Ch 0028   4]              Asl Compiler ID : "IMX6"
[020h 0032   4]        Asl Compiler Revision : 00000001

[024h 0036   4]           Local Apic Address : 00a02000
[028h 0040   4]        Flags (decoded below) : 00000000
                         PC-AT Compatibility : 0

[02Ch 0044   1]                Subtable Type : 0B [Generic Interrupt Controller]
[02Dh 0045   1]                       Length : 50
[02Eh 0046   2]                     Reserved : 0000
[030h 0048   4]         CPU Interface Number : 00000000
[034h 0052   4]                Processor UID : 00000000
[038h 0056   4]        Flags (decoded below) : 00000001
                           Processor Enabled : 1
          Performance Interrupt Trigger Mode : 0
          Virtual GIC Interrupt Trigger Mode : 0
[03Ch 0060   4]     Parking Protocol Version : 00000001
[040h 0064   4]        Performance Interrupt : 0000007e
[044h 0068   8]               Parked Address : 0000000080100000
[04Ch 0076   8]                 Base Address : 0000000000a02000
[054h 0084   8]     Virtual GIC Base Address : 0000000000000000
[05Ch 0092   8]  Hypervisor GIC Base Address : 0000000000000000
[064h 0100   4]        Virtual GIC Interrupt : 00000000
[068h 0104   8]   Redistributor Base Address : 0000000000000000
[070h 0112   8]                    ARM MPIDR : 0000000000000000
[078h 0120   1]             Efficiency Class : 00
[079h 0121   1]                     Reserved : 00
[07Ah 0122   2]       SPE Overflow Interrupt : 0000

[07Ch 0124   1]                Subtable Type : 0C [Generic Interrupt Distributor]
[07Dh 0125   1]                       Length : 18
[07Eh 0126   2]                     Reserved : 0000
[080h 0128   4]        Local GIC Hardware ID : 00000000
[084h 0132   8]                 Base Address : 0000000000a01000
[08Ch 0140   4]               Interrupt Base : 00000000
[090h 0144   1]                      Version : 00
[091h 0145   3]                     Reserved : 000000
