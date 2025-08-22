/**
  ******************************************************************************
  * @file    lr11xx_cli.c
  * @brief   CLI for driving LR11xx firmware update process
  * 
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

/* Sidewalk DUT interfaces */
#include <sid_asd_cli.h>

/* Sidewalk interfaces */
#include <sid_hal_reset_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_radio_ifc.h>
#include <halo_lr11xx_radio.h>

/* Semtech's LR11xx SWDR001 driver */
#include <lr11xx_bootloader.h>
#include <lr11xx_crypto_engine.h>
#include <lr11xx_hal.h>
#include <lr11xx_system.h>

#include "app_900_config.h"
#include "lr11xx_firmware_collection.h"

/* Private defines -----------------------------------------------------------*/

#ifdef SYNTAX_ERR
#undef SYNTAX_ERR
#endif
#define SYNTAX_ERR                                    "Syntax err \r\n"


/*----------------------------------------------------------------------------*/

#define SID_STM32_CLI_REBOOT_H                        "Reset the host MCU (not the LR11xx)"

#define SID_STM32_CLI_REBOOT_CMD                      "reboot"

/*----------------------------------------------------------------------------*/

#define SID_STM32_LR11XX_CLI_INIT_H                   "Initialize SPI bus and GPIO to access LR11xx"
#define SID_STM32_LR11XX_CLI_REINIT_H                 "Reinitialize SPI bus, GPIO and reset LR11xx IC"
#define SID_STM32_LR11XX_CLI_DEINIT_H                 "Release LR11xx IC and related hardware resources"
#define SID_STM32_LR11XX_CLI_RESET_H                  "Reset LR11xx IC. Optionally use \"-hard\" or \"-soft\" parameters to select reset type. Software reset is performed by default"
#define SID_STM32_LR11XX_CLI_RUN_MODE_H               "Get or set LR11xx run mode: app or bootloader"
#define SID_STM32_LR11XX_CLI_VERSION_H                "Read and print LR11xx IC version information"

#define SID_STM32_LR11XX_CLI_ROOT                     "lr11xx"

#define SID_STM32_LR11XX_CLI_INIT_CMD                 SID_STM32_LR11XX_CLI_ROOT" init"
#define SID_STM32_LR11XX_CLI_REINIT_CMD               SID_STM32_LR11XX_CLI_ROOT" reinit"
#define SID_STM32_LR11XX_CLI_DEINIT_CMD               SID_STM32_LR11XX_CLI_ROOT" deinit"
#define SID_STM32_LR11XX_CLI_RESET_CMD                SID_STM32_LR11XX_CLI_ROOT" reset"
#define SID_STM32_LR11XX_CLI_RUN_MODE_CMD             SID_STM32_LR11XX_CLI_ROOT" run_mode"
#define SID_STM32_LR11XX_CLI_VERSION_CMD              SID_STM32_LR11XX_CLI_ROOT" version"

/*----------------------------------------------------------------------------*/

#define SID_STM32_LR11XX_CLI_FW_H                     "LR11xx firmware operations"
#define SID_STM32_LR11XX_CLI_FW_AUTO_UPDATE_H         "Automatically perform full update cycle (enter bootloader, flash erase, program, reboot). You may optionally specify target firmware version. Use \"-f\" or \"--force\" to enforce update to the same version"
#define SID_STM32_LR11XX_CLI_FW_CHECK_IMAGE_H         "Use LR11xx crypto engine to validate the firmware image before flashing"
#define SID_STM32_LR11XX_CLI_FW_ERASE_H               "Erase current firmware"
#define SID_STM32_LR11XX_CLI_FW_FLASH_H               "Flash specified firmware image into LR11xx. Device flash should be erased before this command"
#define SID_STM32_LR11XX_CLI_FW_LIST_H                "List available firmware images"

#define SID_STM32_LR11XX_CLI_FW_ROOT                  "fw"

#define SID_STM32_LR11XX_CLI_FW_AUTO_UPDATE_CMD       SID_STM32_LR11XX_CLI_ROOT" "SID_STM32_LR11XX_CLI_FW_ROOT" auto_update"
#define SID_STM32_LR11XX_CLI_FW_CHECK_IMAGE_CMD       SID_STM32_LR11XX_CLI_ROOT" "SID_STM32_LR11XX_CLI_FW_ROOT" check_image"
#define SID_STM32_LR11XX_CLI_FW_ERASE_CMD             SID_STM32_LR11XX_CLI_ROOT" "SID_STM32_LR11XX_CLI_FW_ROOT" erase"
#define SID_STM32_LR11XX_CLI_FW_FLASH_CMD             SID_STM32_LR11XX_CLI_ROOT" "SID_STM32_LR11XX_CLI_FW_ROOT" flash"
#define SID_STM32_LR11XX_CLI_FW_LIST_CMD              SID_STM32_LR11XX_CLI_ROOT" "SID_STM32_LR11XX_CLI_FW_ROOT" list"


/*----------------------------------------------------------------------------*/

#define SID_STM32_LR11XX_CLI_BASE_16                  (16u)
#define SID_STM32_LR11XX_CLI_BASE_10                  (10u)
#define SID_STM32_LR11XX_CLI_CAPITAL_START            (64u)
#define SID_STM32_LR11XX_CLI_CAPITAL_END              (71u)
#define SID_STM32_LR11XX_CLI_CAPITAL_SHIFT            (7u)
#define SID_STM32_LR11XX_CLI_LOWER_START              (96u)
#define SID_STM32_LR11XX_CLI_LOWER_END                (103u)
#define SID_STM32_LR11XX_CLI_LOWER_SHIFT              (39u)
#define SID_STM32_LR11XX_CLI_NUMBER_START             (47u)
#define SID_STM32_LR11XX_CLI_NUMBER_END               (58u)

/*----------------------------------------------------------------------------*/

#define SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK    (0x01u)
#define SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE    (0x00u)
#define SID_STM32_LR11XX_HAL_STAT2_APP_MODE           (0x01u)

#define SID_STM32_LR11XX_HAL_FLASH_CHUNK_SIZE_WORDS   (64u)
#define SID_STM32_LR11XX_HAL_FLASH_CHUNK_SIZE_BYTES   ((SID_STM32_LR11XX_HAL_FLASH_CHUNK_SIZE_WORDS) << 2)

#define SID_STM32_LR11XX_HAL_BOOTSTRAP_WAIT_TIME_MS   (250u)
#define SID_STM32_LR11XX_HAL_BUSY_TOGGLE_WAIT_TIME_MS (50u)

#define SID_STM32_LR11XX_HAL_LR1110_BOOTLOADER_VER    (0x6500u)
#define SID_STM32_LR11XX_HAL_LR1120_BOOTLOADER_VER    (0x2000u)
#define SID_STM32_LR11XX_HAL_LR1121_BOOTLOADER_VER    (0x2100u)

/* Private macro -------------------------------------------------------------*/

#define SID_STM32_LR11XX_CLI_STRINGIFY(__S__)                   #__S__
#define SID_STM32_LR11XX_CLI_ASSERT_RESULT(__FUNC__, __ERROR__) if (sid_err != SID_ERROR_NONE) \
                                                                { \
                                                                    CLI_LOG_ERROR(SID_STM32_LR11XX_CLI_STRINGIFY(__FUNC__) " failed with error %d", (int32_t)(__ERROR__)); \
                                                                    break; \
                                                                }

#define CLI_LOG_OUTCOME(__ERR__)                                if (SID_ERROR_NONE == (__ERR__)) \
                                                                { \
                                                                    CLI_LOG_INFO("CMD: ERR: %d", (int32_t)(__ERR__)); \
                                                                } \
                                                                else \
                                                                { \
                                                                    CLI_LOG_ERROR("CMD: ERR: %d", (int32_t)(__ERR__)); \
                                                                }

/* Private variables ---------------------------------------------------------*/

static sid_pal_radio_rx_packet_t sid_pal_radio_rx_packet;

/* Private function prototypes -----------------------------------------------*/

static inline uint32_t            lr11xx_cli_utils_str_to_uint(const char * const str, const uint32_t base);
static inline int32_t             lr11xx_cli_utils_str_to_int(const char * const str, const uint8_t base);
static        int32_t             lr11xx_cli_utils_parse_input_num(const char * const buf);
static        lr11xx_fw_version_t lr11xx_cli_utils_parse_fw_version_num(const char * const buf);

static void sid_pal_radio_event_notify(sid_pal_radio_events_t events);
static void sid_pal_radio_irq_handler(void);

static sid_error_t lr11xx_enter_bootloader_mode(void);
static sid_error_t lr11xx_enter_app_mode(void);
static void lr11xx_print_version_info(const lr11xx_system_version_t * const ver);
static sid_error_t lr11xx_erase_flash(void);
static sid_error_t lr11xx_flash_image(const lr11xx_fw_descriptor_t * const fw_descriptor);
static sid_error_t lr11xx_check_image_signature(const lr11xx_fw_descriptor_t * const fw_descriptor);
static sid_error_t lr11xx_check_image_compatibility(const lr11xx_fw_descriptor_t * const fw_descriptor, const uint16_t bootloader_version);

static ace_status_t lr11xx_cli_init_cmd(int32_t argc, const char **argv);
static ace_status_t lr11xx_cli_reinit_cmd(int32_t argc, const char **argv);
static ace_status_t lr11xx_cli_deinit_cmd(int32_t argc, const char **argv);
static ace_status_t lr11xx_cli_reset_cmd(int32_t argc, const char **argv);
static ace_status_t lr11xx_cli_run_mode_cmd(int32_t argc, const char **argv);
static ace_status_t lr11xx_cli_version_cmd(int32_t argc, const char **argv);

static ace_status_t lr11xx_cli_fw_auto_update_cmd(int32_t argc, const char **argv);
static ace_status_t lr11xx_cli_fw_check_image_cmd(int32_t argc, const char **argv);
static ace_status_t lr11xx_cli_fw_erase_cmd(int32_t argc, const char **argv);
static ace_status_t lr11xx_cli_fw_flash_cmd(int32_t argc, const char **argv);
static ace_status_t lr11xx_cli_fw_list_cmd(int32_t argc, const char **argv);

static ace_status_t lr11xx_cli_mcu_reset_cmd(int32_t argc, const char **argv);

/* Private constants ---------------------------------------------------------*/

SID_CLI_REGISTER_COMMAND(m_sub_lr11xx_fw)
{
    SID_CLI_DEFINE_COMMAND(        auto_update, NULL, SID_STM32_LR11XX_CLI_FW_AUTO_UPDATE_H, lr11xx_cli_fw_auto_update_cmd),
    SID_CLI_DEFINE_COMMAND(        check_image, NULL, SID_STM32_LR11XX_CLI_FW_CHECK_IMAGE_H, lr11xx_cli_fw_check_image_cmd),
    SID_CLI_DEFINE_COMMAND(        erase,       NULL, SID_STM32_LR11XX_CLI_FW_ERASE_H,       lr11xx_cli_fw_erase_cmd),
    SID_CLI_DEFINE_COMMAND(        flash,       NULL, SID_STM32_LR11XX_CLI_FW_FLASH_H,       lr11xx_cli_fw_flash_cmd),
    SID_CLI_DEFINE_COMMAND(        list,        NULL, SID_STM32_LR11XX_CLI_FW_LIST_H,        lr11xx_cli_fw_list_cmd),
    
    SID_CLI_SUBCMD_SET_END,
};

SID_CLI_REGISTER_COMMAND(m_sub_lr11xx)
{
    SID_CLI_DEFINE_COMMAND(        init,        NULL, SID_STM32_LR11XX_CLI_INIT_H,           lr11xx_cli_init_cmd),
    SID_CLI_DEFINE_COMMAND(        reinit,      NULL, SID_STM32_LR11XX_CLI_REINIT_H,         lr11xx_cli_reinit_cmd),
    SID_CLI_DEFINE_COMMAND(        deinit,      NULL, SID_STM32_LR11XX_CLI_DEINIT_H,         lr11xx_cli_deinit_cmd),
    SID_CLI_DEFINE_COMMAND(        reset,       NULL, SID_STM32_LR11XX_CLI_RESET_H,          lr11xx_cli_reset_cmd),
    SID_CLI_DEFINE_COMMAND(        run_mode,    NULL, SID_STM32_LR11XX_CLI_RUN_MODE_H,       lr11xx_cli_run_mode_cmd),
    SID_CLI_DEFINE_COMMAND(        version,     NULL, SID_STM32_LR11XX_CLI_VERSION_H,        lr11xx_cli_version_cmd),
    SID_CLI_DEFINE_SUB_COMMAND_SET(fw,          NULL, SID_STM32_LR11XX_CLI_FW_H,             m_sub_lr11xx_fw),
    SID_CLI_SUBCMD_SET_END,
};

SID_CLI_REGISTER_COMMAND(m_lr11xx_commands)
{
    SID_CLI_DEFINE_COMMAND(        reboot,      NULL, SID_STM32_CLI_REBOOT_H,                lr11xx_cli_mcu_reset_cmd),
    SID_CLI_DEFINE_SUB_COMMAND_SET(lr11xx,      NULL, "LR11xx control commands",             m_sub_lr11xx),
    SID_CLI_SUBCMD_SET_END,
};

/* Private function definitions ----------------------------------------------*/

static inline uint32_t lr11xx_cli_utils_str_to_uint(const char * const str, const uint32_t base)
{
    uint32_t uint_val;

    do
    {
        char stoi_char;
        uint32_t flag;

        if (NULL == str)
        {
            uint_val = 0;
            break;
        }

        uint_val = 0u;
        const char * str_pos = str;
        while ((*str_pos != '\0') && (*str_pos != '\n') && (*str_pos != '\r') && (*str_pos != ' '))
        {
            stoi_char = *str_pos;
            if (stoi_char != '.') /* skip over decimal point to convert floats to ints */
            {
                uint_val = uint_val * base;   /* mult by base */
                flag = FALSE;

                if ((SID_STM32_LR11XX_CLI_BASE_16 == base) && (stoi_char > SID_STM32_LR11XX_CLI_CAPITAL_START) && (stoi_char < SID_STM32_LR11XX_CLI_CAPITAL_END))
                {
                    stoi_char -= SID_STM32_LR11XX_CLI_CAPITAL_SHIFT;
                    flag = TRUE;
                }

                if ((FALSE == flag) && (SID_STM32_LR11XX_CLI_BASE_16 == base) && (stoi_char > SID_STM32_LR11XX_CLI_LOWER_START) && (stoi_char < SID_STM32_LR11XX_CLI_LOWER_END))
                {
                    stoi_char -= SID_STM32_LR11XX_CLI_LOWER_SHIFT;
                    flag = TRUE;
                }

                if ((FALSE == flag) && (stoi_char > SID_STM32_LR11XX_CLI_NUMBER_START) && (stoi_char < SID_STM32_LR11XX_CLI_NUMBER_END))
                {
                    flag = TRUE;
                }

                if (flag != FALSE)
                {
                    uint_val = uint_val + (uint32_t)stoi_char;
                    uint_val = uint_val - (uint32_t)'0';
                }
                else
                {
                    /* Parsing error */
                    uint_val = 0u;
                    break;
                }
            }
            str_pos++;
        }
    } while (0);

    return uint_val;
}

/*----------------------------------------------------------------------------*/

static inline int32_t lr11xx_cli_utils_str_to_int(const char * const str, const uint8_t base)
{
    int32_t int_val;

    do
    {
        if (NULL == str)
        {
            int_val = 0;
            break;
        }

        if (*str == '-')
        {
            int_val = -lr11xx_cli_utils_str_to_uint(&str[1], base);
        }
        else
        {
            int_val = lr11xx_cli_utils_str_to_uint(str, base);
        }
    } while (0);

    return int_val;
}

/*----------------------------------------------------------------------------*/

static int32_t lr11xx_cli_utils_parse_input_num(const char * const buf)
{
    int32_t int_val;
    uint32_t hex_format;
    uint32_t idx = 0u;

    if ((buf[0] == '0') && ((buf[1] == 'x') || (buf[1] == 'X')))
    {
        idx = 2u;
        hex_format = TRUE;
    }
    else if ((buf[0] == 'x') || (buf[0] == 'X'))
    {
        idx = 1u;
        hex_format = TRUE;
    }
    else
    {
        hex_format = FALSE;
    }

    if (hex_format != FALSE)
    {
        int_val = lr11xx_cli_utils_str_to_int(&(buf[idx]), SID_STM32_LR11XX_CLI_BASE_16);
    }
    else
    {
        int_val = lr11xx_cli_utils_str_to_int(&(buf[idx]), SID_STM32_LR11XX_CLI_BASE_10);
    }

    return int_val;
}

/*----------------------------------------------------------------------------*/

lr11xx_fw_version_t lr11xx_cli_utils_parse_fw_version_num(const char * const buf)
{
    lr11xx_fw_version_t fw_ver = {
        .raw = 0u,
    };
    int32_t int_val = 0; /* Treat 0 as a marker of invalid input */

    do
    {
        /* Try to parse special firmware version specifiers */
        if (strcmp(buf, "factory") == 0)
        {
            const lr11xx_fw_descriptor_t * const fw_descriptor = lr11xx_fw_collection_get_factory_fw();
            if (fw_descriptor != NULL)
            {
                int_val = (int32_t)fw_descriptor->version.raw;
            }
            else
            {
                /* No default value for factory firmware is provided by the collection */
            }

            break;
        }

        if (strcmp(buf, "latest") == 0)
        {
            const lr11xx_fw_descriptor_t * const fw_descriptor = lr11xx_fw_collection_get_latest_fw();
            if (fw_descriptor != NULL)
            {
                int_val = (int32_t)fw_descriptor->version.raw;
            }
            else
            {
                /* No default value for latest firmware is provided by the collection */
            }

            break;
        }

        /* Try to parse version number input as string */
        const char * delimiter_pos = buf;

        while((*delimiter_pos != '\0') && (*delimiter_pos != '.'))
        {
            delimiter_pos++;
        }

        if (*delimiter_pos == '.')
        {
            /* Parse string as hex, decimal point will be skipped over */
            int_val = (int32_t)lr11xx_cli_utils_str_to_uint(buf, SID_STM32_LR11XX_CLI_BASE_16);
            int32_t minor_int_val = (int32_t)lr11xx_cli_utils_str_to_uint(delimiter_pos + 1, SID_STM32_LR11XX_CLI_BASE_16);
            if (int_val != 0)
            {
                if ((minor_int_val < 0x10) && ((int_val & 0xF0) != 0))
                {
                    /* Restore implicit zero after decimal point */
                    int_val = ((int_val & ~0x0F) << 4) | minor_int_val;
                }
                else if (minor_int_val >= 0x100)
                {
                    /* Invalid input, minor version overflow */
                    int_val = 0;
                }
            }
            break;
        }

        /* Try to parse version number input as hex or decimal value */
        int_val = lr11xx_cli_utils_parse_input_num(buf);

        if (int_val != 0)
        {
            if (!((buf[0] == '0') && ((buf[1] == 'x') || (buf[1] == 'X')))
              && !((buf[0] == 'x') || (buf[0] == 'X')))
            {
                /* Convert decimal input to hex value */
                int32_t upper_val = int_val / 100;
                int32_t lower_val = int_val % 100;
                int_val =  ((upper_val / 10) << 12) | ((upper_val % 10) << 8) | ((lower_val / 10) << 4) | (lower_val % 10);
            }
            break;
        }
    } while (0);

    if ((int_val > 0) && (int_val < (int32_t)UINT16_MAX))
    {
        fw_ver.raw = (uint16_t)int_val;
    }
    else
    {
        /* Input is a legit numerical value, but it is out of range for firmware version number */
    }

    return fw_ver;
}

/*----------------------------------------------------------------------------*/

static void sid_pal_radio_event_notify(sid_pal_radio_events_t events)
{
    /* Nothing to do here */
    (void)events;
}

/*----------------------------------------------------------------------------*/

static void sid_pal_radio_irq_handler(void)
{
    /* Nothing to do here */
}

/*----------------------------------------------------------------------------*/

static sid_error_t lr11xx_enter_bootloader_mode(void)
{
    sid_error_t sid_err = SID_ERROR_GENERIC;
    lr11xx_status_t sys_err;

    do
    {
        halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        /* Use bootstrapping mechanism to enter bootloader mode - this method is universal and has no dependency on the actual firmware type on LR11xx device */
        CLI_LOG_INFO("Rebooting LR11xx into bootloader mode...");

        /* Step 1 - set Busy pin as output on MCU side and drive it low */
        /* Configure the output stage of Busy pin GPIO on MCU side to open-dran mode */
        sid_err = sid_pal_gpio_output_mode(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_OUTPUT_OPEN_DRAIN);
        if (sid_err != SID_ERROR_NONE)
        {
            CLI_LOG_ERROR("Failed to configure LR11xx Busy GPIO");
            break;
        }

        /* Pre-set output state to low */
        sid_err = sid_pal_gpio_write(drv_ctx->config->gpio.radio_busy, 0u);
        if (sid_err != SID_ERROR_NONE)
        {
            CLI_LOG_ERROR("Failed to configure LR11xx Busy GPIO");
            break;
        }

        /* Switch Busy GPIO pin from input to output - now theBusy pin is held low by MCU */
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_DIRECTION_OUTPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            CLI_LOG_ERROR("Failed to configure LR11xx Busy GPIO");
            break;
        }

        /* Step 2 - drive NRESET - this will hold NRESET low for at least 100us and return back immediatley since Busy is held low by MCU */
        sys_err = lr11xx_system_reset(drv_ctx);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to reset LR11xx. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Step 3 - wait at least 100ms as prescribed by AN1200-57 */
#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
        if (SID_STM32_UTIL_IS_IRQ() == FALSE)
        {
            sid_pal_scheduler_delay_ms(SID_STM32_LR11XX_HAL_BOOTSTRAP_WAIT_TIME_MS);
        }
        else
        {
            sid_pal_delay_us(SID_STM32_LR11XX_HAL_BOOTSTRAP_WAIT_TIME_MS * 1000u);
        }
#else
        sid_pal_delay_us(SID_STM32_LR11XX_HAL_BOOTSTRAP_WAIT_TIME_MS * 1000u);
#endif /* SID_PAL_ENABLE_SCHEDULER_DELAY */

        /* Step 4 - reconfigure Busy pin back to input mode */
        /* Switch Busy GPIO pin from input to output - now theBusy pin is held low by MCU */
        sid_err = sid_pal_gpio_set_direction(drv_ctx->config->gpio.radio_busy, SID_PAL_GPIO_DIRECTION_INPUT);
        if (sid_err != SID_ERROR_NONE)
        {
            CLI_LOG_ERROR("Failed to configure LR11xx Busy GPIO");
            break;
        }

        /* Give the Busy pin some time to go high */
        #ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
        if (SID_STM32_UTIL_IS_IRQ() == FALSE)
        {
            sid_pal_scheduler_delay_ms(SID_STM32_LR11XX_HAL_BUSY_TOGGLE_WAIT_TIME_MS);
        }
        else
        {
            sid_pal_delay_us(SID_STM32_LR11XX_HAL_BUSY_TOGGLE_WAIT_TIME_MS * 1000u);
        }
#else
        sid_pal_delay_us(SID_STM32_LR11XX_HAL_BUSY_TOGGLE_WAIT_TIME_MS * 1000u);
#endif /* SID_PAL_ENABLE_SCHEDULER_DELAY */

        /* Step 6 - wait for Busy to go back low */
        sys_err = lr11xx_hal_wait_readiness(drv_ctx);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("LR11xx failed to release Busy after bootstrapping. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_INVALID_STATE;
            break;
        };

        /* Readout and store version information */
        sys_err = lr11xx_bootloader_get_version(drv_ctx, (lr11xx_bootloader_version_t *)&drv_ctx->ver);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to read LR11xx version info. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        /* Ensure LR11xx is actually in bootloader mode */
        if ((LR11XX_RADIO_BOOTLOADER_MODE_ID == drv_ctx->ver.type) && ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE))
        {
            CLI_LOG_INFO("LR11xx is now in the bootloader mode. Bootloader version: %02X.%02X", (drv_ctx->ver.fw >> 8), (drv_ctx->ver.fw & 0xFFu));
            sid_err = SID_ERROR_NONE;
        }
        else
        {
            CLI_LOG_ERROR("LR11xx failed to enter bootloader mode. Device type data: 0x%02X, Stat2: 0x%02X", drv_ctx->ver.type, drv_ctx->last.stat2);
            sid_err = SID_ERROR_INVALID_STATE;
            break;
        }
    } while (0);
    
    return sid_err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t lr11xx_enter_app_mode(void)
{
    sid_error_t sid_err = SID_ERROR_GENERIC;
    lr11xx_status_t sys_err;

    do
    {
        halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        CLI_LOG_INFO("Rebooting LR11xx into app mode...");

        sys_err = lr11xx_bootloader_reboot(drv_ctx, false);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to reset LR11xx. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Readout and store version information */
        sys_err = lr11xx_system_get_version(drv_ctx, &drv_ctx->ver);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to read LR11xx version info. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        /* Ensure LR11xx is actually in bootloader mode */
        if ((drv_ctx->ver.type != LR11XX_RADIO_BOOTLOADER_MODE_ID) && ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_APP_MODE))
        {
            CLI_LOG_INFO("LR11xx is now in the app mode");
            sid_err = SID_ERROR_NONE;
        }
        else
        {
            CLI_LOG_ERROR("LR11xx failed to enter app mode. Device type data: 0x%02X, Stat2: 0x%02X", drv_ctx->ver.type, drv_ctx->last.stat2);
            sid_err = SID_ERROR_INVALID_STATE;
            break;
        }
    } while (0);
    
    return sid_err;
}

/*----------------------------------------------------------------------------*/

static void lr11xx_print_version_info(const lr11xx_system_version_t * const ver)
{
    const char * printable_name;

    /* Search for a printable name for known devices */
    switch ((uint32_t)ver->type)
    {
        case LR11XX_RADIO_LR11XX_IC_ID:
            printable_name = "LR1110";
            break;

        case LR11XX_RADIO_LR1120_IC_ID:
            printable_name = "LR1120";
            break;

        case LR11XX_RADIO_LR1121_IC_ID:
            printable_name = "LR1121";
            break;

        case LR11XX_RADIO_BOOTLOADER_MODE_ID:
            printable_name = "LR11xx Bootloader";
            SID_PAL_LOG_WARNING("LR11xx is in bootloader mode, unable to detect specific IC variant");
            break;

        default:
            printable_name = NULL;
            break;
    }

    /* Check if the transceiver is recognized */
    if (printable_name != NULL)
    {
        /* Printout version information */
        SID_PAL_LOG_INFO("Detected %s device. HW: 0x%02X, FW: %02X.%02X", printable_name, ver->hw, ((ver->fw >> 8) & 0xFFu), (ver->fw & 0xFFu));
    }
    else
    {
        SID_PAL_LOG_ERROR("Unrecognized LR11xx type reported (type: 0x%02X, HW: 0x%02X, FW: 0x%04X). Can't proceed", ver->type, ver->hw, ver->fw);
    }
}

/*----------------------------------------------------------------------------*/

static sid_error_t lr11xx_erase_flash(void)
{
    sid_error_t sid_err = SID_ERROR_GENERIC;
    lr11xx_status_t sys_err;

    do
    {
        halo_drv_semtech_ctx_t * const drv_ctx  = lr11xx_get_drv_ctx();

        CLI_LOG_INFO("Erasing LR11xx flash...");

#if LR11XX_RADIO_CFG_USE_STATUS_LED
        (void)lr11xx_radio_hal_tx_led_on(drv_ctx);
        (void)lr11xx_radio_hal_rx_led_on(drv_ctx);
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

        sys_err = lr11xx_bootloader_erase_flash(drv_ctx);

#if LR11XX_RADIO_CFG_USE_STATUS_LED
        (void)lr11xx_radio_hal_tx_led_off(drv_ctx);
        (void)lr11xx_radio_hal_rx_led_off(drv_ctx);
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to erase LR11xx flash. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        CLI_LOG_INFO("LR11xx flash erased");
        sid_err = SID_ERROR_NONE;
    } while (0);
    
    return sid_err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t lr11xx_flash_image(const lr11xx_fw_descriptor_t * const fw_descriptor)
{
    sid_error_t sid_err = SID_ERROR_GENERIC;
    lr11xx_status_t sys_err;

    do
    {
        halo_drv_semtech_ctx_t * const drv_ctx  = lr11xx_get_drv_ctx();
        uint32_t remaining_words = fw_descriptor->word_size;
        uint32_t write_offset = 0u;

        if ((drv_ctx->ver.type != LR11XX_RADIO_BOOTLOADER_MODE_ID) || ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) != SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE))
        {
            CLI_LOG_ERROR("LR11xx is not in bootloader mode. Bootloader mode is required to flash the firmware");
            sid_err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Check image compatibility with the bootloader */
        sid_err = lr11xx_check_image_compatibility(fw_descriptor, drv_ctx->ver.fw);
        if (sid_err != SID_ERROR_NONE)
        {
            CLI_LOG_ERROR("Selected firmware is not compatible with the detected LR11xx device");
            break;
        }

        CLI_LOG_INFO("Flashing LR11xx firmware...");

        sid_err = SID_ERROR_NONE;
        for (uint32_t i = 0u; remaining_words > 0u; i++)
        {

#if LR11XX_RADIO_CFG_USE_STATUS_LED
            /* Toggle Rx and Tx LEDs every 8 chunks to indicate flashing progress */
            if ((i & 0x0Fu) < 8u)
            {
                (void)lr11xx_radio_hal_tx_led_on(drv_ctx);
                (void)lr11xx_radio_hal_rx_led_on(drv_ctx);
            }
            else
            {
                (void)lr11xx_radio_hal_tx_led_off(drv_ctx);
                (void)lr11xx_radio_hal_rx_led_off(drv_ctx);
            }
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

            sys_err = lr11xx_bootloader_write_flash_encrypted(drv_ctx, write_offset,
                                                              &fw_descriptor->fw_binary_data[i * SID_STM32_LR11XX_HAL_FLASH_CHUNK_SIZE_WORDS],
                                                              MIN(SID_STM32_LR11XX_HAL_FLASH_CHUNK_SIZE_WORDS, remaining_words));

            if (sys_err != LR11XX_STATUS_OK)
            {
                CLI_LOG_ERROR("Failed to write LR11xx flash at offset 0x%08X. Stat1: 0x%02X, error %d", write_offset, drv_ctx->last.stat1, (int32_t)sys_err);
                sid_err = SID_ERROR_STORAGE_WRITE_FAIL;
                break;
            }

            write_offset += SID_STM32_LR11XX_HAL_FLASH_CHUNK_SIZE_BYTES;
            remaining_words = (remaining_words < SID_STM32_LR11XX_HAL_FLASH_CHUNK_SIZE_WORDS) ? 0u : (remaining_words - SID_STM32_LR11XX_HAL_FLASH_CHUNK_SIZE_WORDS);
        }

#if LR11XX_RADIO_CFG_USE_STATUS_LED
        (void)lr11xx_radio_hal_tx_led_off(drv_ctx);
        (void)lr11xx_radio_hal_rx_led_off(drv_ctx);
#endif /* LR11XX_RADIO_CFG_USE_STATUS_LED */

        if (SID_ERROR_NONE == sid_err)
        {
            CLI_LOG_INFO("LR11xx firmware uploaded");
        }
    } while (0);

    return sid_err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t lr11xx_check_image_signature(const lr11xx_fw_descriptor_t * const fw_descriptor)
{
    sid_error_t sid_err;
    lr11xx_status_t sys_err;

    do
    {
        halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        /* LR11xx should not be in bootloader mode since crypto engine commands are available in app mode only */
        if ((LR11XX_RADIO_BOOTLOADER_MODE_ID == drv_ctx->ver.type) && ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE))
        {
            CLI_LOG_ERROR("LR11xx is in bootloader mode. App mode is required to perform this operation");
            sid_err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Ensure current firmware supports necessary crypto commands */
        if ((LR11XX_SYSTEM_VERSION_TYPE_LR1110 == drv_ctx->ver.type) && (drv_ctx->ver.fw < 0x0308u))
        {
            CLI_LOG_ERROR("Firmware image check operation requires current firmware version of 03.08 or newer, but it is %02X.%02X. Please update first", (drv_ctx->ver.fw >> 8), (drv_ctx->ver.fw & 0xFFu));
            sid_err = SID_ERROR_NOSUPPORT;
            break;
        }
        else if (((LR11XX_SYSTEM_VERSION_TYPE_LR1120 == drv_ctx->ver.type) || (LR11XX_SYSTEM_VERSION_TYPE_LR1121 == drv_ctx->ver.type)) && (drv_ctx->ver.fw < 0x0102u))
        {
            CLI_LOG_ERROR("Firmware image check operation requires current firmware version of 01.02 or newer, but it is %02X.%02X. Please update first", (drv_ctx->ver.fw >> 8), (drv_ctx->ver.fw & 0xFFu));
            sid_err = SID_ERROR_NOSUPPORT;
            break;
        }
        else
        {
            /* Everything is ok */
        }

        CLI_LOG_INFO("Checking firmware image %02X.%02X...", fw_descriptor->version.major, fw_descriptor->version.minor);

        sys_err = lr11xx_crypto_check_encrypted_firmware_image_full(drv_ctx, 0u, fw_descriptor->fw_binary_data, fw_descriptor->word_size);
        CLI_LOG_FLUSH(); /* The above check may generate excessive log messages due to constant CmdError IRQ if firmware is not valid */
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to send firmware image to LR11xx. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        bool is_encrypted_fw_image_ok;
        sys_err = lr11xx_crypto_get_check_encrypted_firmware_image_result(drv_ctx, &is_encrypted_fw_image_ok);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to get firmware image check status from LR11xx. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        if (false == is_encrypted_fw_image_ok)
        {
            CLI_LOG_ERROR("Firmware image %02X.%02X failed the check. This image is invalid", fw_descriptor->version.major, fw_descriptor->version.minor);
            sid_err = SID_ERROR_DECRYPTION_FAIL;
            break;
        }

        CLI_LOG_INFO("Firmware image %02X.%02X passed the check successfully", fw_descriptor->version.major, fw_descriptor->version.minor);
        sid_err = SID_ERROR_NONE;
    } while (0);

    return sid_err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t lr11xx_check_image_compatibility(const lr11xx_fw_descriptor_t * const fw_descriptor, const uint16_t bootloader_version)
{
    sid_error_t sid_err;

    do
    {
        /* Check that target device type is aligned with bootloader version */
        switch (fw_descriptor->target)
        {
            case LR11XX_SYSTEM_VERSION_TYPE_LR1110:
                sid_err = (bootloader_version == SID_STM32_LR11XX_HAL_LR1110_BOOTLOADER_VER) ? SID_ERROR_NONE : SID_ERROR_NOSUPPORT;
                break;

            case LR11XX_SYSTEM_VERSION_TYPE_LR1120:
                sid_err = (bootloader_version == SID_STM32_LR11XX_HAL_LR1120_BOOTLOADER_VER) ? SID_ERROR_NONE : SID_ERROR_NOSUPPORT;
                break;

            case LR11XX_SYSTEM_VERSION_TYPE_LR1121:
                sid_err = (bootloader_version == SID_STM32_LR11XX_HAL_LR1121_BOOTLOADER_VER) ? SID_ERROR_NONE : SID_ERROR_NOSUPPORT;
                break;

            default:
                CLI_LOG_ERROR("Firmware compatibility check failed. 0x%02X is not a valid LR11xx device type specifier", fw_descriptor->target);
                sid_err = SID_ERROR_INVALID_ARGS;
                break;
        }

        if (sid_err != SID_ERROR_NONE)
        {
            break;
        }

        /* Check firmware flavor is aligned with the target selection */
        switch (fw_descriptor->type)
        {
            case LR11XX_FW_TYPE_TRX:
                /* All LR11xx device can run TRX firmware */
                sid_err = SID_ERROR_NONE;
                break;

            case LR11XX_FW_TYPE_MODEM_E_V1:
                /* Modem-E V1 applies to LR1110 only */
                sid_err = (fw_descriptor->target == LR11XX_SYSTEM_VERSION_TYPE_LR1110) ? SID_ERROR_NONE : SID_ERROR_NOSUPPORT;
                break;

            case LR11XX_FW_TYPE_MODEM_E_V2:
                /* Modem-E V2 applies to LR1121 only */
                sid_err = (fw_descriptor->target == LR11XX_SYSTEM_VERSION_TYPE_LR1121) ? SID_ERROR_NONE : SID_ERROR_NOSUPPORT;
                break;

            default:
                CLI_LOG_ERROR("Firmware compatibility check failed. 0x%02X is not a valid LR11xx firmware type specifier", fw_descriptor->type);
                sid_err = SID_ERROR_INVALID_ARGS;
                break;
        }

        if (sid_err != SID_ERROR_NONE)
        {
            break;
        }
    } while (0);

    return sid_err;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_init_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_LR11XX_CLI_INIT_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        /* Ensure the radio driver is not initialized yet */
        if (drv_ctx->init_done != FALSE)
        {
            CLI_LOG_ERROR("LR11xx driver is initialized already");
            sid_err = SID_ERROR_ALREADY_INITIALIZED;
            break;
        }

#if (SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 == 0) && (SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 == 0)
        /* If both FSK and LoRa links are disabled we have to manually inject radio config data.
         * Otherwise this is handled by sid_platform_init() and no additional actions are required
         */
        lr11xx_radio_set_device_config(get_radio_cfg());
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 && SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

        /* Initialize LR11xx driver */
        sid_err = sid_pal_radio_init(sid_pal_radio_event_notify, sid_pal_radio_irq_handler, &sid_pal_radio_rx_packet);
        SID_STM32_LR11XX_CLI_ASSERT_RESULT(sid_pal_radio_init, sid_err);

        /* Lock the radio */
        drv_ctx->radio_state = SID_PAL_RADIO_BUSY;
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_reinit_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_LR11XX_CLI_REINIT_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        halo_drv_semtech_ctx_t * drv_ctx = lr11xx_get_drv_ctx();

        /* Ensure the radio driver is initialized */
        if (FALSE == drv_ctx->init_done)
        {
            CLI_LOG_ERROR("LR11xx driver is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Deinitialize LR11xx driver */
        sid_err = sid_pal_radio_deinit();
        SID_STM32_LR11XX_CLI_ASSERT_RESULT(sid_pal_radio_deinit, sid_err);

        /* Initialize LR11xx driver */
        sid_err = sid_pal_radio_init(sid_pal_radio_event_notify, sid_pal_radio_irq_handler, &sid_pal_radio_rx_packet);
        SID_STM32_LR11XX_CLI_ASSERT_RESULT(sid_pal_radio_init, sid_err);

        /* Update driver context */
        drv_ctx = lr11xx_get_drv_ctx();

        /* Lock the radio */
        drv_ctx->radio_state = SID_PAL_RADIO_BUSY;
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_deinit_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_LR11XX_CLI_DEINIT_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        const halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        /* Ensure the radio driver is initialized */
        if (FALSE == drv_ctx->init_done)
        {
            CLI_LOG_ERROR("LR11xx driver is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Deinitialize LR11xx driver */
        sid_err = sid_pal_radio_deinit();
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_reset_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;
    lr11xx_status_t sys_err;

    do
    {
        if (argc > 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_LR11XX_CLI_RESET_CMD " takes at most one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        /* Ensure the radio driver is initialized */
        if (FALSE == drv_ctx->init_done)
        {
            CLI_LOG_ERROR("LR11xx driver is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        if ((0 == argc) || (strcmp(argv[0], "-s") == 0) || (strcmp(argv[0], "--soft") == 0))
        {
            /* Perform soft reset via SPI command */
            if ((LR11XX_RADIO_BOOTLOADER_MODE_ID == drv_ctx->ver.type) && ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE))
            {
                /* The IC is in bootloader mode - reset and stay in bootloader mode */
                sys_err = lr11xx_bootloader_reboot(drv_ctx, true);
            }
            else
            {
                /* The IC is in app mode - reset and go back to app mode */
                sys_err = lr11xx_system_reboot(drv_ctx, false);
            }
        }
        else if ((strcmp(argv[0], "-h") == 0) || (strcmp(argv[0], "--hard") == 0))
        {
            sys_err = lr11xx_system_reset(drv_ctx);
        }
        else
        {
            CLI_LOG_ERROR("\"%s\" is not a recognized reset type. Valid values are: --hard (-h), --soft (-s)", argv[0]);
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Check if reset was performed successfully */
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to reset LR11xx. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Readout version info after reset to align on Bootloader/App mode state */
        sys_err = lr11xx_system_get_version(drv_ctx, &drv_ctx->ver);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to read LR11xx version info. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        CLI_LOG_INFO("Reset performed. LR11xx is in %s mode",
                     ((LR11XX_RADIO_BOOTLOADER_MODE_ID == drv_ctx->ver.type) && ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE)) ? "bootloader" :
                     ((drv_ctx->ver.type != LR11XX_RADIO_BOOTLOADER_MODE_ID) && ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_APP_MODE)) ? "app" :
                     "unknown");
        sid_err = SID_ERROR_NONE;
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_run_mode_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;
    lr11xx_status_t sys_err;

    do
    {
        if (argc > 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_LR11XX_CLI_RUN_MODE_CMD " takes at most one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        /* Ensure the radio driver is initialized */
        if (FALSE == drv_ctx->init_done)
        {
            CLI_LOG_ERROR("LR11xx driver is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Readout and store the latest version information */
        sys_err = lr11xx_system_get_version(drv_ctx, &drv_ctx->ver);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to read LR11xx version info. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        if (0 == argc)
        {
            /* Get Run Mode command */
            CLI_LOG_INFO("LR11xx is in %s mode",
                         ((LR11XX_RADIO_BOOTLOADER_MODE_ID == drv_ctx->ver.type) && ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE)) ? "bootloader" :
                         ((drv_ctx->ver.type != LR11XX_RADIO_BOOTLOADER_MODE_ID) && ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_APP_MODE)) ? "app" :
                         "unknown");
            sid_err = SID_ERROR_NONE;
        }
        else
        {
            /* Set Run Mode command */
            const char * const desired_mode_str = argv[0];
            if (strcmp(desired_mode_str, "bootloader") == 0)
            {
                if ((LR11XX_RADIO_BOOTLOADER_MODE_ID == drv_ctx->ver.type) && ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE))
                {
                    CLI_LOG_WARNING("LR11xx is in the bootloader mode already, command skipped");
                    sid_err = SID_ERROR_NONE;
                }
                else
                {
                    sid_err = lr11xx_enter_bootloader_mode();
                }
            }
            else if (strcmp(desired_mode_str, "app") == 0)
            {
                if ((drv_ctx->ver.type != LR11XX_RADIO_BOOTLOADER_MODE_ID) && ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_APP_MODE))
                {
                    CLI_LOG_WARNING("LR11xx is in the app mode already, command skipped");
                    sid_err = SID_ERROR_NONE;
                }
                else
                {
                    sid_err = lr11xx_enter_app_mode();
                }
            }
            else
            {
                CLI_LOG_ERROR("\"%s\" is not a recognized LR11xx run mode. Valid values are: bootlaoder, app", desired_mode_str);
                sid_err = SID_ERROR_INVALID_ARGS;
            }
        }
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_version_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;
    lr11xx_status_t sys_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_LR11XX_CLI_VERSION_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        /* Ensure the radio driver is initialized */
        if (FALSE == drv_ctx->init_done)
        {
            CLI_LOG_ERROR("LR11xx driver is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Readout and store version information */
        sys_err = lr11xx_system_get_version(drv_ctx, &drv_ctx->ver);
        if (sys_err != LR11XX_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read LR11xx version info. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        lr11xx_print_version_info(&drv_ctx->ver);
        sid_err = SID_ERROR_NONE;
        break;
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_fw_auto_update_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;
    lr11xx_status_t sys_err;
    const lr11xx_fw_descriptor_t * fw_descriptor = NULL;
    bool force_update = false;

    do
    {
        if (argc > 2)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_LR11XX_CLI_FW_AUTO_UPDATE_CMD " takes at most two arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        /* Ensure the radio driver is initialized */
        if (FALSE == drv_ctx->init_done)
        {
            CLI_LOG_ERROR("LR11xx driver is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Parse command parameters */
        sid_err = SID_ERROR_NONE;
        for (uint32_t i = 0u; i < argc; i++)
        {
            if ((strcmp(argv[i], "-f") == 0) || (strcmp(argv[i], "--force") == 0))
            {
                force_update = true;
            }
            else
            {
                /* Search for the specified firmware version */
                const lr11xx_fw_version_t target_fw_ver = lr11xx_cli_utils_parse_fw_version_num(argv[i]);
                if (target_fw_ver.raw == 0u)
                {
                    CLI_LOG_ERROR("Wrong firmware version specifier. \"%s\" is not a valid version number string", argv[i]);
                    sid_err = SID_ERROR_INVALID_ARGS;
                    break;
                }

                uint32_t fw_count;
                const lr11xx_fw_descriptor_t * const fw_collection = lr11xx_fw_collection_get_collection(&fw_count);

                /* Ensure firmware collection is not empty */
                if ((NULL == fw_collection) || (0u == fw_count))
                {
                    CLI_LOG_ERROR("Firmware images collection is empty");
                    sid_err = SID_ERROR_NOT_FOUND;
                    break;
                }

                sid_err = SID_ERROR_NOT_FOUND;
                for (uint32_t i = 0u; i < fw_count; i++)
                {
                    if (fw_collection[i].version.raw == target_fw_ver.raw)
                    {
                        fw_descriptor = &fw_collection[i];
                        sid_err = SID_ERROR_NONE;
                        break;
                    }
                }

                if (SID_ERROR_NOT_FOUND == sid_err)
                {
                    CLI_LOG_ERROR("Firmware image %02X.%02X not found in the firmware collection. Please specify an existing version", target_fw_ver.major, target_fw_ver.minor);
                    break;
                }
            }
        }
        if (sid_err != SID_ERROR_NONE)
        {
            break;
        }

        /* Try to pull the latest firmware if no version was explicitly specified in command line parameters */
        if (NULL == fw_descriptor)
        {
            /* Use the latest available firmware if no version was specified explicitly by the user */
            fw_descriptor = lr11xx_fw_collection_get_latest_fw();
            if (NULL == fw_descriptor)
            {
                CLI_LOG_ERROR("Unable to identify the latest firmware version. Please try to specify target version explicitly");
                sid_err = SID_ERROR_NOT_FOUND;
                break;
            }
        }

        CLI_LOG_INFO("Performing automated update to firmware %02X.%02X...", fw_descriptor->version.major, fw_descriptor->version.minor);

        /* Do a hard reset to clear any preceding conditions */
        sys_err = lr11xx_system_reset(drv_ctx);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to reset LR11xx. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        /* Readout and store the latest version information */
        sys_err = lr11xx_system_get_version(drv_ctx, &drv_ctx->ver);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to read LR11xx version info. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        /* Ensure LR11xx is in app mode */
        if ((LR11XX_RADIO_BOOTLOADER_MODE_ID == drv_ctx->ver.type) || ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE))
        {
            /* Switch to app mode */
            sid_err = lr11xx_enter_app_mode();
        }
        else
        {
            sid_err = SID_ERROR_NONE;
        }

        /* Verify image signature if possible. If not - just proceed without this check */
        if (SID_ERROR_NONE == sid_err)
        {
            /* LR11xx managed to enter application mode - check if firmware version differs */
            if ((false == force_update) && (fw_descriptor->version.raw == drv_ctx->ver.fw))
            {
                CLI_LOG_ERROR("LR11xx already runs firmware %02X.%02X. Firmware updated skipped. Use -f or --force flag to update anyway");
                sid_err = SID_ERROR_CANCELED;
                break;
            }

            /* We can try to validate the image signature before proceeding - if this functionality is supported b current firmware */
            CLI_LOG_INFO("Verifying image signature...");

            sid_err = lr11xx_check_image_signature(fw_descriptor);
            if (SID_ERROR_NONE == sid_err)
            {
                /* Just proceed, no actions required */
            }
            else if (SID_ERROR_DECRYPTION_FAIL == sid_err)
            {
                CLI_LOG_ERROR("Selected firmware image failed signature check. Update aborted");
                break;
            }
            else if (SID_ERROR_NOSUPPORT == sid_err)
            {
                CLI_LOG_WARNING("Firmware image signature check is not possible. Proceeding with the update without check");
            }
            else
            {
                CLI_LOG_ERROR("Update process aborted due to unexpected error");
            }
        }

        /* Switch to the bootloader mode */
        sid_err = lr11xx_enter_bootloader_mode();
        if (sid_err != SID_ERROR_NONE)
        {
            break;
        }

        /* Check image compatibility with the bootloader */
        CLI_LOG_INFO("Checking image compatibility...");
        sid_err = lr11xx_check_image_compatibility(fw_descriptor, drv_ctx->ver.fw);
        if (sid_err != SID_ERROR_NONE)
        {
            break;
        }
        else
        {
            CLI_LOG_INFO("Selected image is compatible with the bootloader");
        }

        /* Erase flash */
        sid_err = lr11xx_erase_flash();
        if (sid_err != SID_ERROR_NONE)
        {
            break;
        }

        /* Flash new image */
        sid_err = lr11xx_flash_image(fw_descriptor);
        if (sid_err != SID_ERROR_NONE)
        {
            break;
        }

        /* Hard-reset LR11xx after flashing */
        CLI_LOG_INFO("Resetting LR11xx after update...");
        sys_err = lr11xx_system_reset(drv_ctx);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to reset LR11xx. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        /* Readout and store the latest version information */
        sys_err = lr11xx_system_get_version(drv_ctx, &drv_ctx->ver);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to read LR11xx version info. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        /* Ensure LR11xx is in app mode */
        if ((LR11XX_RADIO_BOOTLOADER_MODE_ID == drv_ctx->ver.type) || ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) == SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE))
        {
            CLI_LOG_ERROR("LR11xx update to firmware %02X.%02X rejeced. LR11xx will stay in bootloader mode until a valid firmware is flashed");
            sid_err = SID_ERROR_TRY_AGAIN;
            break;
        }
        
        /* Ensure the reported firmware version matches the expeted one */
        if (drv_ctx->ver.fw != fw_descriptor->version.raw)
        {
            CLI_LOG_ERROR("LR11xx firmware version mismatch. Reported: %02X.%02X, expected: %02X.%02X", (drv_ctx->ver.fw >> 8), (drv_ctx->ver.fw & 0xFFu), fw_descriptor->version.major, fw_descriptor->version.minor);
            sid_err = SID_ERROR_INVALID_RESPONSE;
            break;
        }

        CLI_LOG_INFO("LR11xx successfully updated to firmware %02X.%02X", fw_descriptor->version.major, fw_descriptor->version.minor);
        sid_err = SID_ERROR_NONE;
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_fw_check_image_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;
    lr11xx_status_t sys_err;

    do
    {
        if (argc != 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_LR11XX_CLI_FW_CHECK_IMAGE_CMD " takes exactly one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        const lr11xx_fw_version_t target_fw_ver = lr11xx_cli_utils_parse_fw_version_num(argv[0]);
        if (target_fw_ver.raw == 0u)
        {
            CLI_LOG_ERROR("Wrong firmware version specifier. \"%s\" is not a valid version number string", argv[0]);
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        /* Ensure the radio driver is initialized */
        if (FALSE == drv_ctx->init_done)
        {
            CLI_LOG_ERROR("LR11xx driver is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Readout and store the latest version information */
        sys_err = lr11xx_system_get_version(drv_ctx, &drv_ctx->ver);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to read LR11xx version info. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        /* Search for the specified firmware image */
        uint32_t fw_count;
        const lr11xx_fw_descriptor_t * const fw_collection = lr11xx_fw_collection_get_collection(&fw_count);

        /* Ensure firmware collection is not empty */
        if ((NULL == fw_collection) || (0u == fw_count))
        {
            CLI_LOG_ERROR("Firmware images collection is empty");
            sid_err = SID_ERROR_NOT_FOUND;
            break;
        }

        sid_err = SID_ERROR_NOT_FOUND;
        for (uint32_t i = 0u; i < fw_count; i++)
        {
            const lr11xx_fw_descriptor_t * const fw_descriptor = &fw_collection[i];

            if (fw_descriptor->version.raw == target_fw_ver.raw)
            {
                sid_err = lr11xx_check_image_signature(fw_descriptor);
                break;
            }
        }

        if (SID_ERROR_NOT_FOUND == sid_err)
        {
            CLI_LOG_ERROR("Firmware image %02X.%02X not found in the firmware collection. Please specify an existing version", target_fw_ver.major, target_fw_ver.minor);
        }
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_fw_erase_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;
    lr11xx_status_t sys_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_LR11XX_CLI_FW_LIST_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        /* Ensure the radio driver is initialized */
        if (FALSE == drv_ctx->init_done)
        {
            CLI_LOG_ERROR("LR11xx driver is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Readout and store the latest version information */
        sys_err = lr11xx_system_get_version(drv_ctx, &drv_ctx->ver);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to read LR11xx version info. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        /* LR11xx should be in bootloader mode */
        if ((drv_ctx->ver.type != LR11XX_RADIO_BOOTLOADER_MODE_ID) || ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) != SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE))
        {
            CLI_LOG_ERROR("LR11xx is not in bootloader mode. Bootloader mode is required to perform this operation");
            sid_err = SID_ERROR_INVALID_STATE;
            break;
        }

        sid_err = lr11xx_erase_flash();
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_fw_flash_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;
    lr11xx_status_t sys_err;

    do
    {
        if (argc != 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_LR11XX_CLI_FW_FLASH_CMD " takes exactly one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        const lr11xx_fw_version_t target_fw_ver = lr11xx_cli_utils_parse_fw_version_num(argv[0]);
        if (target_fw_ver.raw == 0u)
        {
            CLI_LOG_ERROR("Wrong firmware version specifier. \"%s\" is not a valid version number string", argv[0]);
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        halo_drv_semtech_ctx_t * const drv_ctx = lr11xx_get_drv_ctx();

        /* Ensure the radio driver is initialized */
        if (FALSE == drv_ctx->init_done)
        {
            CLI_LOG_ERROR("LR11xx driver is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Readout and store the latest version information */
        sys_err = lr11xx_system_get_version(drv_ctx, &drv_ctx->ver);
        if (sys_err != LR11XX_STATUS_OK)
        {
            CLI_LOG_ERROR("Failed to read LR11xx version info. Error %d", (int32_t)sys_err);
            sid_err = SID_ERROR_IO_ERROR;
            break;
        };

        /* LR11xx should be in bootloader mode */
        if ((drv_ctx->ver.type != LR11XX_RADIO_BOOTLOADER_MODE_ID) || ((drv_ctx->last.stat2 & SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MASK) != SID_STM32_LR11XX_HAL_STAT2_BOOTLOADER_MODE))
        {
            CLI_LOG_ERROR("LR11xx is not in bootloader mode. Bootloader mode is required to perform this operation");
            sid_err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Search for the specified firmware image */
        uint32_t fw_count;
        const lr11xx_fw_descriptor_t * const fw_collection = lr11xx_fw_collection_get_collection(&fw_count);

        /* Ensure firmware collection is not empty */
        if ((NULL == fw_collection) || (0u == fw_count))
        {
            CLI_LOG_ERROR("Firmware images collection is empty");
            sid_err = SID_ERROR_NOT_FOUND;
            break;
        }

        sid_err = SID_ERROR_NOT_FOUND;
        for (uint32_t i = 0u; i < fw_count; i++)
        {
            const lr11xx_fw_descriptor_t * const fw_descriptor = &fw_collection[i];

            if (fw_descriptor->version.raw == target_fw_ver.raw)
            {
                sid_err = lr11xx_flash_image(fw_descriptor);
                break;
            }
        }

        if (SID_ERROR_NOT_FOUND == sid_err)
        {
            CLI_LOG_ERROR("Firmware image %02X.%02X not found in the firmware collection. Please specify an existing version", target_fw_ver.major, target_fw_ver.minor);
        }
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_fw_list_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_LR11XX_CLI_FW_LIST_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        uint32_t fw_count;
        const lr11xx_fw_descriptor_t * const fw_collection = lr11xx_fw_collection_get_collection(&fw_count);

        /* Ensure firmware collection is not empty */
        if ((NULL == fw_collection) || (0u == fw_count))
        {
            CLI_LOG_ERROR("Firmware images collection is empty");
            sid_err = SID_ERROR_NOT_FOUND;
            break;
        }

        for (uint32_t i = 0u; i < fw_count; i++)
        {
            const lr11xx_fw_descriptor_t * const fw_descriptor = &fw_collection[i];
            const char * target_name;
            const char * fw_type_name;

            /* Search for a printable name for known devices */
            switch (fw_descriptor->target)
            {
                case LR11XX_SYSTEM_VERSION_TYPE_LR1110:
                    target_name = "LR1110";
                    break;
        
                case LR11XX_SYSTEM_VERSION_TYPE_LR1120:
                    target_name = "LR1120";
                    break;
        
                case LR11XX_SYSTEM_VERSION_TYPE_LR1121:
                    target_name = "LR1121";
                    break;

                default:
                    target_name = "<INVALID>";
                    break;
            }

            /* Search for a printable name for known firmware flavors */
            switch (fw_descriptor->type)
            {
                case LR11XX_FW_TYPE_TRX:
                    fw_type_name = "TRX";
                    break;

                case LR11XX_FW_TYPE_MODEM_E_V1:
                case LR11XX_FW_TYPE_MODEM_E_V2:
                    fw_type_name = "Modem-E";
                    break;

                default:
                    fw_type_name = "<INVALID>";
                    break;
            }

            CLI_LOG_INFO("Available firmware: target: %s, version: %02X.%02X, type: %s", target_name, fw_descriptor->version.major, fw_descriptor->version.minor, fw_type_name);
        }

        sid_err = SID_ERROR_NONE;
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t lr11xx_cli_mcu_reset_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_CLI_REBOOT_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        CLI_LOG_WARNING("Resetting host MCU on request...");
        CLI_LOG_FLUSH();

        sid_err = sid_hal_reset(SID_HAL_RESET_NORMAL);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/* Global function definitions -----------------------------------------------*/

sid_error_t lr11xx_cli_init(void)
{
    return SID_CLI_REGISTER_SUB_COMMAND_SET(m_lr11xx_commands);
}
