# Sega Packet Interface 
Repackaged and in some places condensed, version of the `GD-ROM Protocol SPI Specifications` document.
## Signal lines

| Line  |   Name    | Description |
--------|-----------|-------------|
| CS0 |   chip select 0   |   serves for selecting the command block register. Aka 'CS1FX' |
CS1 |   chip select 1   |   serves for selecting the control block register. Aka 'CS3FX'
DA2, DA1, DA0   |   Device address  |   specifies the register and data read/write port
DASP    |   Device active, device 1 present |   Time multiplexed singla which indicates whether the device is active and whether device 1 is present. The signal in this line is an open collector signal. However, the signal cannot be used by the host.
DD0-DD15    |   Device data |   Bidirectional data bus, either 8-bit or 16-bit bus. The 8-bit LSB is used for 8-bit transfer (register etc.)
DIOR    |   Device I/O read |   Read strobe signal from the host. At the falling edge of this signal, the data on the DD0-DD7, DD8-DD15 data bus from the device register or data port become valid. At the rising edge of this signal, the host latches on to the data. Until latching is completed, the host cannot access the data.
DIOW    |   Device I/O write    |   Write strobe signal from the host. At the falling edge of this signal, the data on the DD0- DD7, DD8-DD15 data bus from the device register or data port are latched. Until latching is completed, the device cannot access the data.
DMACK   |   DMA acknowledge (optional)  |   This signal is used by the host in response to DMARQ, for initiating a DMA transfer.
DMARQ   |   DMA request (optional)  |   This signal is used for DMA transfer between the host and this device. When data transfer is possible, the device drives the signal. The data transfer direction is controlled by the DIOR- and DIOW- commands. This signal is used in handshaking with the DMACK- signal. When no device is selected and when no DMA commands are being executed, the signal line is always in the high- impedance (release) state. When DMA transfer is being carried out, the signal is driven by the device.
INTRQ   |   Device interrupt    |   Serves for issuing an interrupt to the host. The INTRQ signal has a hold function which asserts the interrupt only when the device is selected and the host has cleared the nIEN bit in the device control register. When the nIEN bit is "1", or when the device is not selected, the INTRQ signal is in the high-impedance state, regardless of the presence of the hold interrupt. In the interrupt hold state, the following OR conditions are cleared. <ul><li>RESET signal assertion</li><li>Device control register SRSRT bit reset</li><li>When host has written to command register</li><li>When host has read from command register</li></ul> For PIO transfer, the INTRQ signal is asserted at the start of each transferred data. The data block is normally a single sector, unless declared otherwise by a separate method. For DMA transfer, the INTRQ signal is asserted only once, after the command has terminated.
IOCS16  |   Device 16-bit I/O   |   For PIO transfer in mode 0, 1, or 2, the IOCS16- signal indicates that the 16-bit data port is already addressed and that the device can send and receive 16-bit data. This is an open collector output.<ul><li>In any PIO mode, the device supports only 16-bit data transfer. Therefore the IOCS16- signal is always asserted.</li><li>During DMA mode transfer, the host uses the 16-bit DMA channel and the IOCS16- signal is not asserted.</li></ul> However, the signal cannot be used by the host.
IORDY   |   I/O channel ready   | When the device cannot comply with a data transfer request, the signal is negated in order to expand the register access host transfer cycles. <ul><li>When this signal is driven, it is valid only during the DIOR/DIOW cycle for the selected device.</li><li> When the IORDY signal is not negated, it is in the high-impedance state, because it is an open collector signal.</li></ul>
RESET   |   Device reset    |   This signal is issued by the host. It is asserted at the start of power-on, and the assert state is maintained until the power supply level has stabilized. This is a tolerance interval until device reset after power-on
MCK |   Main clock  |    33.8688 MHz clock signal for AICA use.
BCLK    |   Bit clock   |   Clock signal for fetching Digital Audio.
LRCK    |   Left right clock    |   Digital Audio left/right discriminat signal.
SDTATA  |   Serial Data |   Digital Audio data
EMPH    |   Emphasis    |   <ul><li>A High output signal is obtainable if preemphasis is applied to Digital Audio during playback or scan playback. Should there be irregularities in the subcode Q data during playback, the value of the previous time is retained and output. </li><li>Low is output when power is turned ON, when reset, when playback is stopped, when the lid is open and when no disc is inserted.</li></ul>
---

## Command Transfer Mechanism
### Reset Conditons
These are listed in priority order. With **Power-on Reset** being higher than **SPI Soft Reset**, etc... **ATA SRST** has priority over all other states except these.
* **Power-on reset or hardware reset**
    * In the same way as executing the master/slave diagnosis protocol, the electrical circuit diagnosis is executed and the default values are established.
    1) All currently executing commands and I/O operations are cleared.
    2) The device is reset to default condition.
    3) As after normal power-on reset, the operation mode of the device returns to a suitable initial state.
    4) The task file register is initialized:
        * `Status   = 0x00`
        * `Error = 0x10`
        * `Sector Count = 0x01`
        * `Sector Number = 0x01`
        * `Cylinder Low = 0x14`
        * `Cylinder High = 0xEB`
        * `Drive/Head = 0x00`
        * `BSY = 0` following after any reset indicates that the task file register is already initalized for the host.
* **SPI Soft Reset**
    * When the SPI device receives a SPI Soft Reset command, the interface circuit is reset to an interface that operates with the characteristics requested by Set Features.
    * Must be able to return the MCU from a busy or hung-up state so commands can be performed.
    * Can be issued when the `BSY` bit is 1
    1) The BUSY state is set. When the reset sequence in the device is completed, the BUSY state is also cleared. This is the only status that is reported to the host during an SPI Software Reset.
    2) The same information sequence as after power-on reset is performed and the task file is initialized. As an exception, the DRV bit remains unchanged.
* **ATA SRST**
    * The device normally supplies the ATA PDIAG/DASP sequence and executes the task file associated with the command when SRST is detected. This does not cause an actual device reset. There are no startup, abort, or stop commands.


### ATA I/O Register
* Communication between the device and the host occurs via the I/O register selected by the code data on the signal from the host (`CS0`, `CS1`, `DA2`, `DA1`, `DA0`, `DIOR`, `DIOW`)
* Except for the data register, all registers are read/written in 8-bits (byte units)
* The data register is always accessed in 16-bit words.

#### Status Register
<ul>

| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
---|---|---|---|---|---|---|---|
BSY | DRDY | DF | DSC | DRQ | CORR | Reserved | CHECK |

<li>
This register indicates the drive status. When the register is read, any pending interrupt signal is cleared. 
</li>
<li>
When bit 7 (BSY) is "0", the other bits are also valid and the command block can be accessed. 
</li>
<li>
When the bit is "1", the other bits are invalid and the command block cannot be accessed. However, in the case of special commands (NOP, Soft Reset command, etc.) access to the command block is enabled even when this bit is "1".
</li>
<li>
Bit 7 (BSY) becomes valid 400 ns after a command is received.
</li>
</ul>

* Bit 7: `BSY` is always set to 1 when the drive accessess the command block.
* Bit 6: `DRDY` set to 1 when the drive is able to respond to an ATA command.
* Bit 5: `DF` Returns drive fault information.
* Bit 4: `DSC` Becomes 1 when seek processing is complete.
* Bit 3: `DRQ` Becomes 1 when preparations for data transfer between drive and host are completed. 
    * Information held in the Interrupt Reason Register becomes valid in the packet command when DRQ is set.
* Bit 2: `CORR` Indicates a correctable error has occured.
* Bit 1: RESERVED
* Bit 0: `CHECK` Becomes 1 when an error has occured during execution of the command the previous time. 
    * Error detauls can be determined by checking the sense key and error code.

---
#### Alternate Status Register
* This register is the same as the status register, but it does not clear DMA status information when it is accessed.

| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
---|---|---|---|---|---|---|---|
BSY | DRDY | DF | DSC | DRQ | CORR | Reserved | CHECK |

#### Command Register
Commands from the host are set in this register. For information on the meaning of commands, refer to the Table 3.3. TODO - fill out relevant info from referenced table in this section

#### Byte Count Register
* This register serves for controlling the number of bytes sent from the host in response to the DRQ commands.
* It is used only in PIO mode. 
* In DMA mode, the Byte Count Register contents are disregarded. 
* The register is set before a packet command is issued. The register determines the total transfer amount of the data sent in response to one data group transfer command (REC_MODE/SET_MODE, REQ_STAT, etc.).
* For commands which require several DRQ interrupts (CD_READ, CD_READ2, etc.), the expected data length is set in this register.
* When any data are to be transferred, the device sets the number of bytes to be transferred by the host in the Byte Count Register and then issues the DRQ interrupt. 
* The contents of the register do not change unless at least one word or more is transferred from the data register.

---
#### Data Register
* This register is used for reading and writing during data transfer with the host. 
* The register is switchable between 8 and 16 bits. 
* Only 16-bit mode is supported because the host is not using the IOCS-16 signal.

---
#### Device Control Register
Bit 2 (SRST) of this register is the reset switch from the host, but it is not used in the current protocol. When wishing to perform a software reset, use the "Software Reset" command as defined in the SPI protocol. Bit 1 (nIEN) determines whether the host interrupt is made valid or not.

| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
---|---|---|---|---|---|---|---|
Reserved | | | | 1 | SRST | nIEN | 0 |

* Bit 2 `SRST` Software reset from host. 
    * Default is 0
    * Reset is performed when set to "1"
    * NOTE: because this is not used in this protocol use the "SPI Software Reset" command defined in SPI for performing a software reset
* Bit 1 `nIEN` Sets the interrupt for the host.
    * Default is 0
    * Invalid can be selected by setting this to 1

---
#### Drive Select Register (ATA Drive/Head select register)
| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
---|---|---|---|---|---|---|---|
1 | Reserved | 1 | 0 | LUN | | | |

* Bit 0-3 `LUN` Logical unit number to which the command is applied
* This parameter is optional and reserved for future use ( lol :joy: )
---

#### Error Register
* The completion status of the most recent command is set in this register, also at the end of hard disk diagnosis. 
* When the status register bit 0 is "1", an error has occurred, and the error content is set in this register.

| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
---|---|---|---|---|---|---|---|
Sense Key |  |  |  | MCR | ABRT | EMOF | ILI |

* Bits 7-4 `Sense Key`
    * `Sense Key` is only reflected in the SPI Command Mode. Same is true for Additional Sense Code (`ASC`) and Additional Sense Code Qualifier (`ASCQ`)
* Bit 3 `MCR` Media change was requested and media have been ejected (ATA level)
* Bit 2 `ABRT` Drive is not ready and command was made invalid (ATA level)
* Bit 1 `EOM` Media end was detected (option) (is option, optional?)
* Bit 0 `ILI` Command length is not correct (option)
---

#### Features Register
This register normally specifies the data transfer mode, but it can also be used for Set Features parameters of the ATA command. When issuing commands accompanied by data transfer, such as CD_READ, specify in this register whether data should be transferred by PIO or DMA at the time of task file initialization.

**Normal use (specify data transfer mode)**
| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
---|---|---|---|---|---|---|---|
Reserve |  |  |  |  |  |  | DMA |

* Bit 0 `DMA` Send data for command in DMA mode

**Use as parameter for Set Features command** 
| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
---|---|---|---|---|---|---|---|
Set(1), Clear(0), Feature | Feature Number | | | |  |  | |
* Bit 6-0 `Feature Number` Set transfer mode by setting to `3`
    * By writing `3` as `Feature Number` and issuing the `Set Feature` command, the PIO or DMA transfer mode set in the `Sector Count` register can be selected.
    * The actual transfer mode is specified by the `Sector Count Register`

---
#### Interrupt Reason Register (Read Only)
| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
---|---|---|---|---|---|---|---|
Reserved | | | | |  | IO | CoD |
* Bit 0 `COD` (Command or Data)
    * 0 indicates data
    * 1 indicates command

| IO    |   DRQ |   CoD | Function
---|---|---|---|
0   |   1   |   1   | Command packet can be received
1   |   1   |   1   | Message can be sent from device to host
1   |   1   |   0   | Data can be sent to host
0   |   1   |   0   | Data can be received from host
1   |   0   |   1   | Status register contains completion status

---
#### Sector Count Register (Write Only)
* This register is used in combination with the ATA command's `Set Features` command

| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
---|---|---|---|---|---|---|---|
Transfer Mode | | | | Mode Value | | | |

| Value | Transfer Mode |
---|---|
00000 00x | PIO Default Transfer Mode
00001 xxx | PIO Flow Control Transfer Mode x
00010 xxx | Single Word DMA mode x 
00100 xxx | Multi-Word DMA
00011 xxx | Reserved (For Pseudo DMA mode)

---
#### Sector Number Register (ATA Sector Number Register)
| 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
---|---|---|---|---|---|---|---|
Disc Format | | | | Status | | | |

The information obtained by this register is the same as the status data obtained with the REQ_STAT command.

* `Disc Format` becomes valid after the lid is closed and the `UNIT ATTENTION` state is entered via the `NOT READY` state, or after the `BSY` bit becomes "0" following Soft Reset.
* `Status` becomes valid after the power is turned ON and the `UNIT ATTENTION` state is entered via the `NOT READY` state, or after the `BSY` bit becomes "0" following Soft Reset.
* The values of this register are invalid during issue of a command and during execution of a command.
* The modification timing of this register and the status modification timing caused by `REQ_STAT` coincide internally but depending on the timing of the read out of the `Sector Number Register` and the issue of the `REQ_STAT` command there may be times where they do not coincide.
* This register is updated unsyncronized with the host. For this reason, it may return unfixed value if it is read at the timing of updating GD drive side. To avoid this, either read it again to confirm value or use `REQ_STAT` command instead of this register.
    * TODO: Refernce section of manual to build useful table... For details on the `Disc Format` and `Status`, refer to the sections on GD Drive State Transition and `REQ_STAT`.
* Operation of this register differs from ATA specifications.


### Sense Key Definitions
| Sense key | Description |
|---|---|
`0x00` | No sense key information (`NO SENSE`). Indicates that no sense key information is present. This also applies when command execution was successful.
`0x01` | Recovered error (`RECOVERED ERROR`). The last command has performed successful error recovery processing. Details can be obtained by checking the additional sense byte and the information field. If several error processing operations were performed for one command, the device reports the status of the last error processing operation.
`0x02` | Not ready (`NOT READY`). Indicates that the device cannot be accessed.
`0x03` | Media error (`MEDIUM ERROR`). Indicates that the command was terminated unsuccessfully due to a non-recoverable media defect or an error during reading or writing. This sense key may also returned by the device when it is not possible to distinguish between a media defect and a hardware defect (sense key 4).
`0x04` | Hardware error (`HARDWARE ERROR`). During operation or self- diagnosis, an unrecoverable hardware error was detected (for example a controller failure, device failure, parity error etc.).
`0x05` | Illegal request (`ILLEGAL REQUEST`). An illegal parameter was included in the command packet or in additional data for commands. When the device detects an illegal parameter in a command packet, it terminates the command without changing the media. When the device detects an illegal parameter in additional data for commands, the device may already have altered the media. When this sense key is reported, the command is not yet executed.
`0x06` | Unit attention (`UNIT ATTENTION`). Indicates that removable media may have been replaced, or that the device was reset.
`0x07` | Data protect (`DATA PROTECT`). Indicates that a write attempt was made on a protected block.
`0x08`-`0x0A` | Reserved
`0x0B` | Aborted command (`ABORTED COMMAND`). Indicates that the device has aborted the command. Recovery may be possible by re-executing the command.
`0x0C`-`0x0F` | Reserved
---

## ATA Command Flow
#### Command for PIO data transfer to host
* Applies to `Identify Device` command
* Command is accompanied by transfer of one or multiple data blocks from the device to the host

1) Host continues to read the `Status Register` or `Alternate Status Register` until `BSY` bit becomes `0`
2) Host writes params for the commands to be issued to following registers:
    * `Features Register`, `Sector Count Register`, `Sector Number Register`, `Cylinder High Register`, `Cylinder Low Register`, `Device/Head Register`
3) Host writes the command code to the `Command Register`
4) Device sets the `BSY` bit to `1` preparing to execute the command for sending the first data block to the host.
5) When the data block is prepared the device sets the `DRQ` bit. The device asserts `INTRQ` after clearing the `BSY` bit.
    * `DRQ` is optional for certain error conditions.
    * If an error condition has occured, the device sets an appropriate status and error bit for the error condition.
    * When the operation to set the `BSY` bit in step 6 and clear it in step 7 is accomplished very fast, the host may not always notice that the `BSY` bit was set once.
6) After polling the `Alternate Status Register` and waiting for the `BSY` bit to become `0`, or after detecting `INTRQ`, the host reads out and saves the contents of the `Status Register`.
7) If the `DRQ` bit was set, the host reads out and transfers one block Data Register from the `Data Register`. 
    * If an error condition has occurred in the status read-out of step 8, the data transfer will not be legal.
8) When the `Status Register` is read out 
    * The device negates `INTRQ`. 
    * After read-out of one entire data block, one of the following operations is executed.
        * If status has been communicated to the host without any errors occurring in step 8 (this step), and transmission of further blocks is required, the device sets the `BSY` bit and repeats the above sequence from step 7 (last step).
        * If an error status was reported during read-out of the status during step 8 (this step), the device clears the `DRQ` bit and completes the execution of the command.
        * When the final block is transferred, the device clears the `DRQ` bit and completes the execution of the command.

---

    