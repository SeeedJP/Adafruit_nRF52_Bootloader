/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "sdk_common.h"
#include "bootloader.h"
#include "bootloader_types.h"
#include "bootloader_util.h"
#include "bootloader_settings.h"
#include "dfu.h"
#include "dfu_transport.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_sdm.h"
#include "nrf_mbr.h"
#include "nordic_common.h"
#include "crc16.h"
#include "pstorage.h"
#include "app_scheduler.h"

#include "nrfx.h"
#include "nrf_wdt.h"
#include "app_timer.h"

#include "boards.h"

#ifdef NRF_USBD
#include "tusb.h"
#endif

/**@brief Enumeration for specifying current bootloader status.
 */
typedef enum
{
    BOOTLOADER_UPDATING,                                /**< Bootloader status for indicating that an update is in progress. */
    BOOTLOADER_SETTINGS_SAVING,                         /**< Bootloader status for indicating that saving of bootloader settings is in progress. */
    BOOTLOADER_COMPLETE,                                /**< Bootloader status for indicating that all operations for the update procedure has completed and it is safe to reset the system. */
    BOOTLOADER_TIMEOUT,                                 /**< Bootloader status field for indicating that a timeout has occured and current update process should be aborted. */
    BOOTLOADER_RESET,                                   /**< Bootloader status field for indicating that a reset has been requested and current update process should be aborted. */
} bootloader_status_t;

static pstorage_handle_t        m_bootsettings_handle;  /**< Pstorage handle to use for registration and identifying the bootloader module on subsequent calls to the pstorage module for load and store of bootloader setting in flash. */
static bootloader_status_t      m_update_status;        /**< Current update status for the bootloader module to ensure correct behaviour when updating settings and when update completes. */
static bool m_cancel_timeout_on_usb; /**< If set the timeout is cancelled when USB is enumerated. Otherwise, the timeout is only cancelled when DFU update is started. */

APP_TIMER_DEF( _dfu_startup_timer );
volatile bool dfu_startup_packet_received = false;

/**@brief   Function for handling callbacks from pstorage module.
 *
 * @details Handles pstorage results for clear and storage operation. For detailed description of
 *          the parameters provided with the callback, please refer to \ref pstorage_ntf_cb_t.
 */
static void pstorage_callback_handler(pstorage_handle_t * p_handle,
                                      uint8_t             op_code,
                                      uint32_t            result,
                                      uint8_t           * p_data,
                                      uint32_t            data_len)
{
    // If we are in BOOTLOADER_SETTINGS_SAVING state and we receive an PSTORAGE_STORE_OP_CODE
    // response then settings has been saved and update has completed.
    if ((m_update_status == BOOTLOADER_SETTINGS_SAVING) && (op_code == PSTORAGE_STORE_OP_CODE))
    {
        m_update_status = BOOTLOADER_COMPLETE;
    }

    APP_ERROR_CHECK(result);
}

/* Terminate the forced DFU mode on startup if no packets is received
 * by put an terminal handler to scheduler
 */
static void dfu_startup_timer_handler(void * p_context)
{
#ifdef NRF_USBD
  if (m_cancel_timeout_on_usb && tud_mounted())
  {
    return;
  }
#endif

  // nRF52832 forced DFU on startup
  // No packets are received within timeout, exit DFU mode
  // dfu_startup_packet_received is set by process_dfu_packet() in dfu_transport_serial.c
  if (!dfu_startup_packet_received)
  {
    dfu_update_status_t update_status;
    update_status.status_code = DFU_TIMEOUT;

    bootloader_dfu_update_process(update_status);
  }
}

/**@brief   Function for waiting for events.
 *
 * @details This function will place the chip in low power mode while waiting for events from
 *          the SoftDevice or other peripherals. When interrupted by an event, it will call the
 *          @ref app_sched_execute function to process the received event. This function will return
 *          when the final state of the firmware update is reached OR when a tear down is in
 *          progress.
 */
static void wait_for_events(void)
{
  for ( ;; )
  {
    // Wait in low power state for any events.
//    uint32_t err_code = sd_app_evt_wait();
//    APP_ERROR_CHECK(err_code);

    // Feed all Watchdog just in case application enable it
    // WDT cannot be disabled once started. It even last through NVIC soft reset
    if ( nrf_wdt_started(NRF_WDT) )
    {
      for (uint8_t i=0; i<8; i++) nrf_wdt_reload_request_set(NRF_WDT, i);
    }

    // Event received. Process it from the scheduler.
    app_sched_execute();

#ifdef NRF_USBD
    // skip if usb is not inited ( e.g OTA / finializing sd/bootloader )
    if ( tusb_inited() )
    {
      tud_task();
      tud_cdc_write_flush();
    }
#endif

    if ((m_update_status == BOOTLOADER_COMPLETE) ||
        (m_update_status == BOOTLOADER_TIMEOUT) ||
        (m_update_status == BOOTLOADER_RESET) )
    {
      // When update has completed or a timeout/reset occured we will return.
      return;
    }
  }
}


void matsujirushi_print_bootloader_sesttings()
{
  bootloader_settings_t bootloader_settings;
  bootloader_settings_get(&bootloader_settings);

  PRINTF("Bootloader Settings:\r\n");
  PRINTF("  settings_version: %u\r\n", bootloader_settings.settings_version);
  PRINTF("  bank_0: 0x%04x", bootloader_settings.bank_0);
  switch (bootloader_settings.bank_0) {
  case BANK_VALID_APP: PRINTF("(BANK_VALID_APP)"); break;
  case BANK_VALID_SD:   PRINTF("(BANK_VALID_SD)"); break;
  case BANK_VALID_BOOT: PRINTF("(BANK_VALID_BOOT)"); break;
  case BANK_ERASED:     PRINTF("(BANK_ERASED)"); break;
  case BANK_INVALID_APP: PRINTF("(BANK_INVALID_APP)"); break;
  default:              PRINTF("()"); break;
  }
  PRINTF("  bank_0_crc: 0x%04x", bootloader_settings.bank_0_crc);
  PRINTF("  bank_0_size: %lu\r\n", bootloader_settings.bank_0_size);
  PRINTF("                                 ");
  PRINTF("  sd_image_start: 0x%08lx", bootloader_settings.sd_image_start);
  PRINTF("  sd_image_size: %lu", bootloader_settings.sd_image_size);
  PRINTF("  bl_image_size: %lu", bootloader_settings.bl_image_size);
  PRINTF("  app_image_size: %lu\r\n", bootloader_settings.app_image_size);
  PRINTF("  bank_1: 0x%04x", bootloader_settings.bank_1);
  switch (bootloader_settings.bank_1) {
  case BANK_VALID_APP: PRINTF("(BANK_VALID_APP)"); break;
  case BANK_VALID_SD:   PRINTF("(BANK_VALID_SD)"); break;
  case BANK_VALID_BOOT: PRINTF("(BANK_VALID_BOOT)"); break;
  case BANK_ERASED:     PRINTF("(BANK_ERASED)"); break;
  case BANK_INVALID_APP: PRINTF("(BANK_INVALID_APP)"); break;
  default:              PRINTF("()"); break;
  }
  PRINTF("  bank_1_crc: 0x%04x", bootloader_settings.bank_1_crc);
  PRINTF("  bank_1_size: %lu\r\n", bootloader_settings.bank_1_size);
  PRINTF("\r\n");
}


bool bootloader_app_is_valid(void)
{
  bool success = false;
  uint32_t const app_addr = DFU_BANK_0_REGION_START;

  bootloader_settings_t bootloader_settings;
  bootloader_settings_get(&bootloader_settings);

  enum { EMPTY_FLASH = 0xFFFFFFFFUL };

  // Application is invalid if first 2 words are all 0xFFFFFFF
  if ( *((uint32_t *)app_addr    ) == EMPTY_FLASH &&
       *((uint32_t *)(app_addr+4)) == EMPTY_FLASH )
  {
    return false;
  }

  // The application in CODE region 1 is flagged as valid during update.
  if ( bootloader_settings.bank_0 == BANK_VALID_APP )
  {
    uint16_t image_crc = 0;

    // A stored crc value of 0 indicates that CRC checking is not used.
    if ( bootloader_settings.bank_0_crc != 0 )
    {
      image_crc = crc16_compute((uint8_t*) app_addr,
                                bootloader_settings.bank_0_size,
                                NULL);
    }

    success = (image_crc == bootloader_settings.bank_0_crc);
  }

  return success;
}


static void bootloader_settings_save(bootloader_settings_t * p_settings)
{
  p_settings->settings_version = BOOTLOADER_SETTINGS_VERSION;

  if ( is_ota() )
  {
    uint32_t err_code = pstorage_clear(&m_bootsettings_handle, sizeof(bootloader_settings_t));
    APP_ERROR_CHECK(err_code);

    err_code = pstorage_store(&m_bootsettings_handle, (uint8_t *) p_settings, sizeof(bootloader_settings_t), 0);
    APP_ERROR_CHECK(err_code);
  }
  else
  {
    flash_nrf5x_erase(m_bootsettings_handle.block_id, sizeof(*p_settings));
    flash_nrf5x_write(m_bootsettings_handle.block_id, p_settings, sizeof(*p_settings), false);
    flash_nrf5x_flush(false);
    pstorage_callback_handler(&m_bootsettings_handle, PSTORAGE_STORE_OP_CODE, NRF_SUCCESS, (uint8_t*)p_settings, sizeof(*p_settings));
  }
}


void bootloader_dfu_update_process(dfu_update_status_t update_status)
{
  __attribute__((aligned(4)))  static bootloader_settings_t settings;
  bootloader_settings_t bootloader_settings;

  PRINTF("bootloader_dfu_update_process(%d", update_status.status_code);
  switch (update_status.status_code)
  {
  case DFU_UPDATE_APP_COMPLETE:
    PRINTF("(DFU_UPDATE_APP_COMPLETE)");
    break;
  case DFU_UPDATE_SD_COMPLETE:
    PRINTF("(DFU_UPDATE_SD_COMPLETE)");
    break;
  case DFU_UPDATE_SD_SWAPPED:
    PRINTF("(DFU_UPDATE_SD_SWAPPED)");
    break;
  case DFU_UPDATE_BOOT_COMPLETE:
    PRINTF("(DFU_UPDATE_BOOT_COMPLETE)");
    break;
  case DFU_BANK_0_ERASED:
    PRINTF("(DFU_BANK_0_ERASED)");
    break;
  case DFU_TIMEOUT:
    PRINTF("(DFU_TIMEOUT)");
    break;
  case DFU_RESET:
    PRINTF("(DFU_RESET)");
    break;
  case DFU_UF2_BOOTLOADER_COMPLETE:
    PRINTF("(DFU_UF2_BOOTLOADER_COMPLETE)");
    break;
  case DFU_UPDATE_SWAP_COMPLETE:
    PRINTF("(DFU_UPDATE_SWAP_COMPLETE)");
    break;
  default:
    PRINTF("()");
    break;
  }
  PRINTF(")\r\n");

  matsujirushi_print_bootloader_sesttings();

  bootloader_settings_get(&bootloader_settings);

  if (update_status.status_code == DFU_BANK_0_ERASED)
  {
    settings.bank_0         = BANK_INVALID_APP;
    settings.bank_0_crc     = 0;
    settings.bank_0_size    = 0;
    settings.sd_image_start = 0;
    settings.sd_image_size  = 0;
    settings.bl_image_size  = 0;
    settings.app_image_size = 0;
    settings.bank_1         = bootloader_settings.bank_1;
    settings.bank_1_crc     = bootloader_settings.bank_1_crc;
    settings.bank_1_size    = bootloader_settings.bank_1_size;

    // m_update_status = BOOTLOADER_SETTINGS_SAVING;
    bootloader_settings_save(&settings);
  }
  else if (update_status.status_code == DFU_UPDATE_APP_COMPLETE)
  {
    settings.bank_0         = BANK_VALID_APP;
    settings.bank_0_crc     = update_status.app_crc;
    settings.bank_0_size    = update_status.app_size;
    settings.sd_image_start = 0;
    settings.sd_image_size  = 0;
    settings.bl_image_size  = 0;
    settings.app_image_size = 0;
    settings.bank_1         = BANK_INVALID_APP;
    settings.bank_1_crc     = 0;
    settings.bank_1_size    = 0;

    m_update_status = BOOTLOADER_SETTINGS_SAVING;
    bootloader_settings_save(&settings);
  }
  else if (update_status.status_code == DFU_UPDATE_SWAP_COMPLETE)
  {
    settings.bank_0         = BANK_INVALID_APP;
    settings.bank_0_crc     = 0;
    settings.bank_0_size    = 0;
    settings.sd_image_start = 0;
    settings.sd_image_size  = 0;
    settings.bl_image_size  = 0;
    settings.app_image_size = 0;
    settings.bank_1         = BANK_VALID_APP;
    settings.bank_1_crc     = update_status.app_crc;
    settings.bank_1_size    = update_status.app_size;

    m_update_status = BOOTLOADER_SETTINGS_SAVING;
    bootloader_settings_save(&settings);
  }
  else if (update_status.status_code == DFU_UPDATE_SD_COMPLETE)
  {
    settings.bank_0         = BANK_VALID_SD;
    settings.bank_0_crc     = update_status.app_crc;
    settings.bank_0_size    = update_status.sd_size + update_status.bl_size + update_status.app_size;
    settings.sd_image_start = update_status.sd_image_start;
    settings.sd_image_size  = update_status.sd_size;
    settings.bl_image_size  = update_status.bl_size;
    settings.app_image_size = update_status.app_size;
    settings.bank_1         = BANK_INVALID_APP;
    settings.bank_1_crc     = 0;
    settings.bank_1_size    = 0;

    m_update_status = BOOTLOADER_SETTINGS_SAVING;
    bootloader_settings_save(&settings);
  }
  else if (update_status.status_code == DFU_UPDATE_BOOT_COMPLETE)
  {
    settings.bank_0         = bootloader_settings.bank_0;
    settings.bank_0_crc     = bootloader_settings.bank_0_crc;
    settings.bank_0_size    = bootloader_settings.bank_0_size;
    settings.sd_image_size  = update_status.sd_size;
    settings.bl_image_size  = update_status.bl_size;
    settings.app_image_size = update_status.app_size;
    settings.bank_1         = BANK_VALID_BOOT;
    settings.bank_1_crc     = 0;
    settings.bank_1_size    = 0;

    m_update_status = BOOTLOADER_SETTINGS_SAVING;
    bootloader_settings_save(&settings);
  }
  else if (update_status.status_code == DFU_UPDATE_SD_SWAPPED)
  {
    if (bootloader_settings.bank_0 == BANK_VALID_SD)
    {
      settings.bank_0      = BANK_INVALID_APP;
      settings.bank_0_crc  = 0;
      settings.bank_0_size = 0;
    }
    // This handles cases where SoftDevice was not updated, hence bank0 keeps its settings.
    else
    {
      settings.bank_0      = bootloader_settings.bank_0;
      settings.bank_0_crc  = bootloader_settings.bank_0_crc;
      settings.bank_0_size = bootloader_settings.bank_0_size;
    }
    settings.sd_image_size  = 0;
    settings.bl_image_size  = 0;
    settings.app_image_size = 0;
    settings.bank_1         = BANK_INVALID_APP;
    settings.bank_1_crc     = 0;
    settings.bank_1_size    = 0;

    m_update_status = BOOTLOADER_SETTINGS_SAVING;
    bootloader_settings_save(&settings);
  }
  else if (update_status.status_code == DFU_TIMEOUT)
  {
    // Timeout has occurred. Close the connection with the DFU Controller.
    uint32_t err_code;
    if ( is_ota() )
    {
      err_code = dfu_transport_ble_close();
    }else
    {
      err_code = dfu_transport_serial_close();
    }
    APP_ERROR_CHECK(err_code);

    m_update_status = BOOTLOADER_TIMEOUT;
  }
  else if (update_status.status_code == DFU_RESET)
  {
    m_update_status = BOOTLOADER_RESET;
  }
  else
  {
    // No implementation needed.
  }

  matsujirushi_print_bootloader_sesttings();
}


uint32_t bootloader_init(void)
{
  uint32_t                err_code;
  pstorage_module_param_t storage_params = {.cb = pstorage_callback_handler};

  err_code = pstorage_init();
  VERIFY_SUCCESS(err_code);

  m_bootsettings_handle.block_id = BOOTLOADER_SETTINGS_ADDRESS;
  err_code = pstorage_register(&storage_params, &m_bootsettings_handle);

  return err_code;
}


uint32_t bootloader_dfu_start(bool ota, uint32_t timeout_ms, bool cancel_timeout_on_usb)
{
  uint32_t err_code;

  m_cancel_timeout_on_usb = cancel_timeout_on_usb && !ota;

  // Clear swap if banked update is used.
  err_code = dfu_init();
  VERIFY_SUCCESS(err_code);

  if ( ota )
  {
    err_code = dfu_transport_ble_update_start();
  }else
  {
    // DFU mode with timeout can be
    // - Forced startup DFU for nRF52832 or
    // - Makecode single tap reset but no enumerated (battery power)
    if ( timeout_ms )
    {
      dfu_startup_packet_received = false;

      app_timer_create(&_dfu_startup_timer, APP_TIMER_MODE_SINGLE_SHOT, dfu_startup_timer_handler);
      app_timer_start(_dfu_startup_timer, APP_TIMER_TICKS(timeout_ms), NULL);
    }

    err_code = dfu_transport_serial_update_start();
  }

  wait_for_events();

  return err_code;
}

void bootloader_app_start(void)
{
  // Disable all interrupts
  NVIC->ICER[0]=0xFFFFFFFF;
  NVIC->ICPR[0]=0xFFFFFFFF;
#if defined(__NRF_NVIC_ISER_COUNT) && __NRF_NVIC_ISER_COUNT == 2
  NVIC->ICER[1]=0xFFFFFFFF;
  NVIC->ICPR[1]=0xFFFFFFFF;
#endif

  uint32_t fwd_ret;
  uint32_t app_addr;

  if ( is_sd_existed() )
  {
    PRINTF("SoftDevice exist\r\n");
    // App starts after SoftDevice
    app_addr = SD_SIZE_GET(MBR_SIZE);
    fwd_ret = sd_softdevice_vector_table_base_set(app_addr);
  }else
  {
    PRINTF("SoftDevice not exist\r\n");

    // App starts right after MBR
    app_addr = MBR_SIZE;
    sd_mbr_command_t command =
    {
      .command = SD_MBR_COMMAND_IRQ_FORWARD_ADDRESS_SET,
      .params.irq_forward_address_set.address = app_addr,
    };

    fwd_ret = sd_mbr_command(&command);
  }

  // unlikely failed to forward vector table, manually set forward address
  if ( fwd_ret != NRF_SUCCESS )
  {
    PRINT_HEX(fwd_ret);

    // MBR use first 4-bytes of SRAM to store foward address
    *(uint32_t *)(0x20000000) = app_addr;
  }

  // jump to app
  bootloader_util_app_start(app_addr);
}


bool bootloader_dfu_sd_in_progress(void)
{
  bootloader_settings_t bootloader_settings;
  bootloader_settings_get(&bootloader_settings);

  if (bootloader_settings.bank_0 == BANK_VALID_SD ||
      bootloader_settings.bank_1 == BANK_VALID_BOOT)
  {
    return true;
  }

  return false;
}


uint32_t bootloader_dfu_sd_update_continue(void)
{
  uint32_t err_code;

  if ((dfu_sd_image_validate() == NRF_SUCCESS) &&
      (dfu_bl_image_validate() == NRF_SUCCESS))
  {
      return NRF_SUCCESS;
  }

  // Ensure that flash operations are not executed within the first 100 ms seconds to allow
  // a debugger to be attached.
  NRFX_DELAY_MS(100);

  err_code = dfu_sd_image_swap();
  APP_ERROR_CHECK(err_code);

  err_code = dfu_sd_image_validate();
  APP_ERROR_CHECK(err_code);

  err_code = dfu_bl_image_swap();
  APP_ERROR_CHECK(err_code);

  return err_code;
}


uint32_t bootloader_dfu_sd_update_finalize(void)
{
  dfu_update_status_t update_status = { 0 };
  update_status.status_code = DFU_UPDATE_SD_SWAPPED;

  bootloader_dfu_update_process(update_status);

  wait_for_events();

  return NRF_SUCCESS;
}


void bootloader_settings_get(bootloader_settings_t * p_settings)
{
  bootloader_settings_t const * p_bootloader_settings;
  bootloader_util_settings_get(&p_bootloader_settings);

  switch (p_bootloader_settings->settings_version)
  {
  case BOOTLOADER_SETTINGS_VERSION:
    *p_settings = *p_bootloader_settings;
    break;
  default:
    p_settings->settings_version = BOOTLOADER_SETTINGS_VERSION;
    p_settings->bank_0         = BANK_INVALID_APP;
    p_settings->bank_0_crc     = 0;
    p_settings->bank_0_size    = 0;
    p_settings->sd_image_start = 0;
    p_settings->sd_image_size  = 0;
    p_settings->bl_image_size  = 0;
    p_settings->app_image_size = 0;
    p_settings->bank_1         = BANK_INVALID_APP;
    p_settings->bank_1_crc     = 0;
    p_settings->bank_1_size    = 0;
    break;
  }
}


bool bootloader_dfu_swap_in_progress(void)
{
  bootloader_settings_t bootloader_settings;
  bootloader_settings_get(&bootloader_settings);

  return bootloader_settings.bank_1 == BANK_VALID_APP;
}


uint32_t bootloader_dfu_swap_continue(void)
{
  return dfu_app_image_swap();
}
