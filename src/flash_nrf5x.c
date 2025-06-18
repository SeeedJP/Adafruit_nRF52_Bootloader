/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <string.h>
#include "nrf_sdm.h"
#include "flash_nrf5x.h"
#include "boards.h"

#define FLASH_PAGE_SIZE           4096
#define FLASH_CACHE_INVALID_ADDR  0xffffffff

static uint32_t _fl_addr = FLASH_CACHE_INVALID_ADDR;
static uint8_t _fl_buf[FLASH_PAGE_SIZE] __attribute__((aligned(4)));

void flash_nrf5x_flush (bool need_erase)
{
  if ( _fl_addr == FLASH_CACHE_INVALID_ADDR ) return;

  // skip the write if contents matches
  if ( memcmp(_fl_buf, (void *) _fl_addr, FLASH_PAGE_SIZE) != 0 )
  {
    // - nRF52832 dfu via uart can miss incoming byte when erasing because cpu is blocked for > 2ms.
    // Since dfu_prepare_func_app_erase() already erase the page for us, we can skip it here.
    // - nRF52840 dfu serial/uf2 are USB-based which are DMA and should have no problems.
    //
    // Note: MSC uf2 does not erase page in advance like dfu serial
    if ( need_erase )
    {
      PRINTF("Erase and ");
      nrfx_nvmc_page_erase(_fl_addr);
    }

    PRINTF("Write 0x%08lX\r\n", _fl_addr);
    nrfx_nvmc_words_write(_fl_addr, (uint32_t *) _fl_buf, FLASH_PAGE_SIZE / 4);
  }

  _fl_addr = FLASH_CACHE_INVALID_ADDR;
}

static void flash_nrf5x_write_single_page (uint32_t dst, void const *src, size_t len, bool need_erase)
{
  uint32_t const dstAddr = dst & ~(FLASH_PAGE_SIZE - 1);
  uint32_t const dstOffset = dst & (FLASH_PAGE_SIZE - 1);

  if (dstOffset == 0)
  {
    if (_fl_addr != FLASH_CACHE_INVALID_ADDR)
    {
      PRINTF("WARNING: The page is not written to flash. 0x%08lX\r\n", _fl_addr);
    }

    memcpy(_fl_buf, (void const *)dstAddr, FLASH_PAGE_SIZE);
    _fl_addr = dstAddr;
  }

  if (dstOffset + len > FLASH_PAGE_SIZE)
  {
    PRINTF("WARNING: Write crosses page boundary. dst=0x%08lX, len=%d\r\n", dst, len);
    len = FLASH_PAGE_SIZE - dstOffset; // Truncate length
  }

  memcpy(_fl_buf + dstOffset, src, len);

  if (dstOffset + len == FLASH_PAGE_SIZE)
  {
    flash_nrf5x_flush(need_erase);
  }
}

void flash_nrf5x_write (uint32_t dst, void const *src, size_t len, bool need_erase)
{
  uint32_t const page_count = NRFX_CEIL_DIV(len, FLASH_PAGE_SIZE);
  for (uint32_t i = 0; i < page_count; i++)
  {
      uint32_t const offset = i * FLASH_PAGE_SIZE;
      flash_nrf5x_write_single_page(dst + offset, (void const *)((uint8_t const *)src + offset), len - offset < FLASH_PAGE_SIZE ? len - offset : FLASH_PAGE_SIZE, false);
  }
}

void flash_nrf5x_erase (uint32_t dst, size_t len)
{
  uint32_t const page_count = NRFX_CEIL_DIV(len, FLASH_PAGE_SIZE);

  for (uint32_t i = 0; i < page_count; i++)
  {
    uint32_t const addr = dst + i * FLASH_PAGE_SIZE;
    PRINTF("Erase 0x%08lX\r\n", addr);
    nrfx_nvmc_page_erase(addr);
  }
}
