#include "boards.h"
#include "uf2/configkeys.h"

__attribute__((used, section(".bootloaderConfig")))
const uint32_t bootloaderConfig[] =
{
  /* CF2 START */
  CFG_MAGIC0, CFG_MAGIC1,                       // magic
  5, 100,                                       // used entries, total entries

  204, 0x100000,                                // FLASH_BYTES = 0x100000
  205, 0x40000,                                 // RAM_BYTES = 0x40000
  208, (USB_DESC_VID << 16) | USB_DESC_UF2_PID, // BOOTLOADER_BOARD_ID = USB VID+PID, used for verification when updating bootloader via uf2
  209, 0xada52840,                              // UF2_FAMILY = 0xada52840
  210, 0x20,                                    // PINS_PORT_SIZE = PA_32

  0, 0, 0, 0, 0, 0, 0, 0
  /* CF2 END */
};

/**< This variable ensures that the linker script will write output voltage from REG0 regulator stage to the UICR register. This value will be written in the HEX file and thus written to UICR when the bootloader is flashed into the chip. */
//  0: 1.8V
//  1: 2.1V
//  2: 2.4V
//  3: 2.7V
//  4: 3.0V
//  5: 3.3V
//  7: 1.8V (Default voltage)
__attribute__ ((section(".uicrREGOUT0")))
volatile uint32_t m_uicr_regout0 = 5; // 3.3V
