/*
  Copyright 2026 Tom De Rybel <tom.derybel@robocow.be>

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  <http://www.apache.org/licenses/LICENSE-2.0>

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

MEMORY
{
    /* 
      Memory map for the STM32H7S3L8, with a 32 Mbyte (256 Mbit) Octo-SPI Flash
      connected to XSPI2 which will be memory-mapped by the bootloader.
    
      Note: all 456 Kbytes of AXI-SRAM are assigned as general application RAM,
            including the regions that could be shared with the ITCM/DTCM.
   
      Note: 1 K = 1 KiBi = 1024 bytes

      TODO: properly assign the Octo-SPI flash regions.
    */
    FLASH           (xrw) : ORIGIN = 0x08000000, LENGTH =  64K /* User Flash: BANK1 */
    FLASH_SYSTEM     (rw) : ORIGIN = 0x1FF00000, LENGTH = 128K /* System Flash, exclusive for secure boot */
    FLASH_OTP        (rw) : ORIGIN = 0x08FF0000, LENGTH =   1K /* OTP Flash */
    BOOTLOADER_STATE (rw) : ORIGIN = 0x70000000, LENGTH = 128K /* Octo-SPI Flash on XSPI2 */
    ACTIVE           (rw) : ORIGIN = 0x70020000, LENGTH = 512K
    DFU              (rw) : ORIGIN = 0x700A0000, LENGTH = 640K
    RAM             (xrw) : ORIGIN = 0x24000000, LENGTH = 456K /* AXI: SRAM1 + SRAM2 + SRAM3 + SRAM4 */
    AHB_SRAM         (rw) : ORIGIN = 0x30000000, LENGTH =  32K /* AHB: SRAM1 + SRAM2 */ 
    BACKUP_SRAM      (rw) : ORIGIN = 0x38800000, LENGTH =   4K /* Backup SRAM supported by Vbat */
    ITCM             (rw) : ORIGIN = 0x00000000, LENGTH =  64K /* Instruction Tightly-Coupled Memory */
    DTCM            (xrw) : ORIGIN = 0x20000000, LENGTH =  64K /* Data Tightly-Coupled Memory*/
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE) - ORIGIN(FLASH);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE) - ORIGIN(FLASH);

__bootloader_active_start = ORIGIN(ACTIVE) - ORIGIN(FLASH);
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE) - ORIGIN(FLASH);

__bootloader_dfu_start = ORIGIN(DFU) - ORIGIN(DFU);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU) - ORIGIN(DFU);
