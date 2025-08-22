/**
  ******************************************************************************
  * @file    sid_ble_coexistence_cli.c
  * @brief   CLI for operating user-defined BLE in Sidewalk coexistence mode
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

#include "app_ble_config.h"

/* Sidewalk DUT interfaces */
#include <sid_asd_cli.h>

/* Sidewalk interfaces */
#include <sid_pal_ble_adapter_stm32wba_ext_ifc.h>
#include <sid_pal_crypto_ifc.h>

/* Private defines -----------------------------------------------------------*/

#ifdef SYNTAX_ERR
#undef SYNTAX_ERR
#endif
#define SYNTAX_ERR                                      "Syntax err \r\n"

#define SID_STM32_BLE_COEXIST_CLI_INIT_H                "Initialization of the user-defined BLE profile"
#define SID_STM32_BLE_COEXIST_CLI_DEINIT_H              "Deinitialization of the user-defined BLE profile"
#define SID_STM32_BLE_COEXIST_CLI_FACTORY_RESET_H       "Erase all BLE data (bonding, configf parameters, etc.)"

#define SID_STM32_BLE_COEXIST_CLI_ROOT                  "ble"

#define SID_STM32_BLE_COEXIST_CLI_INIT_CMD              SID_STM32_BLE_COEXIST_CLI_ROOT" init"
#define SID_STM32_BLE_COEXIST_CLI_DEINIT_CMD            SID_STM32_BLE_COEXIST_CLI_ROOT" deinit"
#define SID_STM32_BLE_COEXIST_CLI_FACTORY_RESET_CMD     SID_STM32_BLE_COEXIST_CLI_ROOT" factory_reset"

/*----------------------------------------------------------------------------*/

#define SID_STM32_BLE_COEXIST_CLI_BEACON_H              "Manage virtual BLE beacon device"
#define SID_STM32_BLE_COEXIST_CLI_BEACON_BOOTSTRAP_H    "Bootstrap virtual BLE beacon device. This does not start advertising, but sets up the virtual device in BLE controller"
#define SID_STM32_BLE_COEXIST_CLI_BEACON_START_H        "Start beacon advertisement"
#define SID_STM32_BLE_COEXIST_CLI_BEACON_STOP_H         "Stop beacon advertisement"
#define SID_STM32_BLE_COEXIST_CLI_BEACON_TERMINATE_H    "Unload virtual BLE beacon from the BLE controller. This deallocates all the resources associated with the virtual beacon"

#define SID_STM32_BLE_COEXIST_CLI_BEACON_ROOT           "beacon"
#define SID_STM32_BLE_COEXIST_CLI_BEACON_BOOTSTRAP_CMD  SID_STM32_BLE_COEXIST_CLI_ROOT" "SID_STM32_BLE_COEXIST_CLI_BEACON_ROOT" bootstrap"
#define SID_STM32_BLE_COEXIST_CLI_BEACON_START_CMD      SID_STM32_BLE_COEXIST_CLI_ROOT" "SID_STM32_BLE_COEXIST_CLI_BEACON_ROOT" start"
#define SID_STM32_BLE_COEXIST_CLI_BEACON_STOP_CMD       SID_STM32_BLE_COEXIST_CLI_ROOT" "SID_STM32_BLE_COEXIST_CLI_BEACON_ROOT" stop"
#define SID_STM32_BLE_COEXIST_CLI_BEACON_TERMINATE_CMD  SID_STM32_BLE_COEXIST_CLI_ROOT" "SID_STM32_BLE_COEXIST_CLI_BEACON_ROOT" terminate"

#define SID_STM32_BLE_COEXIST_CLI_EDDYSTONE_SVC_UUID    (0xFEAAu)

/*----------------------------------------------------------------------------*/

#define SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT          (2u)

#define SID_STM32_BLE_COEXIST_CLI_PERIPH_H              "Manage virtual BLE peripheral device"
#define SID_STM32_BLE_COEXIST_CLI_PERIPH_BOOTSTRAP_H    "Bootstrap virtual BLE peripheral device. This does not start advertising, but sets up the virtual device in BLE controller"
#define SID_STM32_BLE_COEXIST_CLI_PERIPH_START_H        "Start peripheral advertisement"
#define SID_STM32_BLE_COEXIST_CLI_PERIPH_STOP_H         "Stop peripheral advertisement"
#define SID_STM32_BLE_COEXIST_CLI_PERIPH_TERMINATE_H    "Unload virtual BLE peripheral from the BLE controller. This deallocates all the resources associated with the virtual peripheral"
#define SID_STM32_BLE_COEXIST_CLI_PERIPH_SEND_H         "Send out data by updating GATT characteristic value and triggering a notification"

#define SID_STM32_BLE_COEXIST_CLI_PERIPH_ROOT           "peripheral"
#define SID_STM32_BLE_COEXIST_CLI_PERIPH_BOOTSTRAP_CMD  SID_STM32_BLE_COEXIST_CLI_ROOT" "SID_STM32_BLE_COEXIST_CLI_BEACON_ROOT" bootstrap"
#define SID_STM32_BLE_COEXIST_CLI_PERIPH_START_CMD      SID_STM32_BLE_COEXIST_CLI_ROOT" "SID_STM32_BLE_COEXIST_CLI_BEACON_ROOT" start"
#define SID_STM32_BLE_COEXIST_CLI_PERIPH_STOP_CMD       SID_STM32_BLE_COEXIST_CLI_ROOT" "SID_STM32_BLE_COEXIST_CLI_BEACON_ROOT" stop"
#define SID_STM32_BLE_COEXIST_CLI_PERIPH_TERMINATE_CMD  SID_STM32_BLE_COEXIST_CLI_ROOT" "SID_STM32_BLE_COEXIST_CLI_BEACON_ROOT" terminate"
#define SID_STM32_BLE_COEXIST_CLI_PERIPH_SEND_CMD       SID_STM32_BLE_COEXIST_CLI_ROOT" "SID_STM32_BLE_COEXIST_CLI_BEACON_ROOT" send"

/*----------------------------------------------------------------------------*/

#define SID_STM32_BLE_COEXIST_CLI_BASE_16               (16u)
#define SID_STM32_BLE_COEXIST_CLI_BASE_10               (10u)
#define SID_STM32_BLE_COEXIST_CLI_CAPITAL_START         (64u)
#define SID_STM32_BLE_COEXIST_CLI_CAPITAL_END           (71u)
#define SID_STM32_BLE_COEXIST_CLI_CAPITAL_SHIFT         (7u)
#define SID_STM32_BLE_COEXIST_CLI_LOWER_START           (96u)
#define SID_STM32_BLE_COEXIST_CLI_LOWER_END             (103u)
#define SID_STM32_BLE_COEXIST_CLI_LOWER_SHIFT           (39u)
#define SID_STM32_BLE_COEXIST_CLI_NUMBER_START          (47u)
#define SID_STM32_BLE_COEXIST_CLI_NUMBER_END            (58u)

/* Private macro -------------------------------------------------------------*/

#define SID_STM32_BLE_CLI_STRINGIFY(__S__)                      #__S__
#define SID_STM32_BLE_CLI_ASSERT_RESULT(__FUNC__, __ERROR__)    if (sid_err != SID_ERROR_NONE) \
                                                                { \
                                                                    CLI_LOG_ERROR(SID_STM32_BLE_CLI_STRINGIFY(__FUNC__) " failed with error %d", (int32_t)(__ERROR__)); \
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

static const sid_pal_ble_adapter_extended_interface_t * ble_ext_ifc = NULL;

static sid_ble_ext_virtual_device_ctx_t * ble_beacon_ctx = NULL;
static sid_ble_ext_virtual_device_ctx_t * ble_periph_ctx[SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT] = {0};

extern const sid_ble_cfg_uuid_info_t * const non_secure_char_with_notify;
extern const sid_ble_cfg_uuid_info_t * const secure_char_with_notify;

/* Private function prototypes -----------------------------------------------*/

static inline uint32_t sid_ble_coexist_cli_utils_str_to_uint(const char * const str, const uint32_t base);
static inline int32_t  sid_ble_coexist_cli_utils_str_to_int(const char * const str, const uint8_t base);
static        int32_t  sid_ble_coexist_cli_utils_parse_input_num(const char * const buf);

static ace_status_t sid_ble_coexist_cli_init_cmd(int32_t argc, const char **argv);
static ace_status_t sid_ble_coexist_cli_deinit_cmd(int32_t argc, const char **argv);
static ace_status_t sid_ble_coexist_cli_factory_reset_cmd(int32_t argc, const char **argv);

static ace_status_t sid_ble_coexist_cli_beacon_bootstrap_cmd(int32_t argc, const char **argv);
static ace_status_t sid_ble_coexist_cli_beacon_start_cmd(int32_t argc, const char **argv);
static ace_status_t sid_ble_coexist_cli_beacon_stop_cmd(int32_t argc, const char **argv);
static ace_status_t sid_ble_coexist_cli_beacon_terminate_cmd(int32_t argc, const char **argv);

static ace_status_t sid_ble_coexist_cli_peripheral_bootstrap_cmd(int32_t argc, const char **argv);
static ace_status_t sid_ble_coexist_cli_peripheral_start_cmd(int32_t argc, const char **argv);
static ace_status_t sid_ble_coexist_cli_peripheral_stop_cmd(int32_t argc, const char **argv);
static ace_status_t sid_ble_coexist_cli_peripheral_terminate_cmd(int32_t argc, const char **argv);
static ace_status_t sid_ble_coexist_cli_peripheral_send_cmd(int32_t argc, const char **argv);

/* Private constants ---------------------------------------------------------*/

SID_CLI_REGISTER_COMMAND(m_sub_ble_beacon)
{
    SID_CLI_DEFINE_COMMAND(bootstrap, NULL, SID_STM32_BLE_COEXIST_CLI_BEACON_BOOTSTRAP_H, sid_ble_coexist_cli_beacon_bootstrap_cmd),
    SID_CLI_DEFINE_COMMAND(start,     NULL, SID_STM32_BLE_COEXIST_CLI_BEACON_START_H,     sid_ble_coexist_cli_beacon_start_cmd),
    SID_CLI_DEFINE_COMMAND(stop,      NULL, SID_STM32_BLE_COEXIST_CLI_BEACON_STOP_H,      sid_ble_coexist_cli_beacon_stop_cmd),
    SID_CLI_DEFINE_COMMAND(terminate, NULL, SID_STM32_BLE_COEXIST_CLI_BEACON_TERMINATE_H, sid_ble_coexist_cli_beacon_terminate_cmd),
    SID_CLI_SUBCMD_SET_END,
};

SID_CLI_REGISTER_COMMAND(m_sub_ble_peripheral)
{
    SID_CLI_DEFINE_COMMAND(bootstrap, NULL, SID_STM32_BLE_COEXIST_CLI_PERIPH_BOOTSTRAP_H, sid_ble_coexist_cli_peripheral_bootstrap_cmd),
    SID_CLI_DEFINE_COMMAND(start,     NULL, SID_STM32_BLE_COEXIST_CLI_PERIPH_START_H,     sid_ble_coexist_cli_peripheral_start_cmd),
    SID_CLI_DEFINE_COMMAND(stop,      NULL, SID_STM32_BLE_COEXIST_CLI_PERIPH_STOP_H,      sid_ble_coexist_cli_peripheral_stop_cmd),
    SID_CLI_DEFINE_COMMAND(terminate, NULL, SID_STM32_BLE_COEXIST_CLI_PERIPH_TERMINATE_H, sid_ble_coexist_cli_peripheral_terminate_cmd),
    SID_CLI_DEFINE_COMMAND(send,      NULL, SID_STM32_BLE_COEXIST_CLI_PERIPH_SEND_H,      sid_ble_coexist_cli_peripheral_send_cmd),
    SID_CLI_SUBCMD_SET_END,
};

SID_CLI_REGISTER_COMMAND(m_sub_ble)
{
    SID_CLI_DEFINE_COMMAND(        init,          NULL, SID_STM32_BLE_COEXIST_CLI_INIT_H,          sid_ble_coexist_cli_init_cmd),
    SID_CLI_DEFINE_COMMAND(        deinit,        NULL, SID_STM32_BLE_COEXIST_CLI_DEINIT_H,        sid_ble_coexist_cli_deinit_cmd),
    SID_CLI_DEFINE_COMMAND(        factory_reset, NULL, SID_STM32_BLE_COEXIST_CLI_FACTORY_RESET_H, sid_ble_coexist_cli_factory_reset_cmd),
    SID_CLI_DEFINE_SUB_COMMAND_SET(beacon,        NULL, SID_STM32_BLE_COEXIST_CLI_BEACON_H,        m_sub_ble_beacon),
    SID_CLI_DEFINE_SUB_COMMAND_SET(peripheral,    NULL, SID_STM32_BLE_COEXIST_CLI_PERIPH_H,        m_sub_ble_peripheral),
    SID_CLI_SUBCMD_SET_END,
};

SID_CLI_REGISTER_COMMAND(m_ble_coexistence_commands)
{
    SID_CLI_DEFINE_SUB_COMMAND_SET(ble, NULL, "BLE coexistence mode commands", m_sub_ble),
    SID_CLI_SUBCMD_SET_END,
};

/* Private function definitions ----------------------------------------------*/

static inline uint32_t sid_ble_coexist_cli_utils_str_to_uint(const char * const str, const uint32_t base)
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

                if ((SID_STM32_BLE_COEXIST_CLI_BASE_16 == base) && (stoi_char > SID_STM32_BLE_COEXIST_CLI_CAPITAL_START) && (stoi_char < SID_STM32_BLE_COEXIST_CLI_CAPITAL_END))
                {
                    stoi_char -= SID_STM32_BLE_COEXIST_CLI_CAPITAL_SHIFT;
                    flag = TRUE;
                }

                if ((FALSE == flag) && (SID_STM32_BLE_COEXIST_CLI_BASE_16 == base) && (stoi_char > SID_STM32_BLE_COEXIST_CLI_LOWER_START) && (stoi_char < SID_STM32_BLE_COEXIST_CLI_LOWER_END))
                {
                    stoi_char -= SID_STM32_BLE_COEXIST_CLI_LOWER_SHIFT;
                    flag = TRUE;
                }

                if ((FALSE == flag) && (stoi_char > SID_STM32_BLE_COEXIST_CLI_NUMBER_START) && (stoi_char < SID_STM32_BLE_COEXIST_CLI_NUMBER_END))
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

static inline int32_t sid_ble_coexist_cli_utils_str_to_int(const char * const str, const uint8_t base)
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
            int_val = -sid_ble_coexist_cli_utils_str_to_uint(&str[1], base);
        }
        else
        {
            int_val = sid_ble_coexist_cli_utils_str_to_uint(str, base);
        }
    } while (0);

    return int_val;
}

/*----------------------------------------------------------------------------*/

static int32_t sid_ble_coexist_cli_utils_parse_input_num(const char * const buf)
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
        int_val = sid_ble_coexist_cli_utils_str_to_int(&(buf[idx]), SID_STM32_BLE_COEXIST_CLI_BASE_16);
    }
    else
    {
        int_val = sid_ble_coexist_cli_utils_str_to_int(&(buf[idx]), SID_STM32_BLE_COEXIST_CLI_BASE_10);
    }

    return int_val;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_init_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_INIT_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Create BLE interface */
        sid_err = sid_pal_ble_adapter_ext_create_extended_ifc(&ble_ext_ifc);
        SID_STM32_BLE_CLI_ASSERT_RESULT(sid_pal_ble_adapter_ext_create_extended_ifc, sid_err);

        if (NULL == ble_ext_ifc)
        {
            CLI_LOG_ERROR("Extended BLE interface is null");
            sid_err = SID_ERROR_UNINITIALIZED;
        }

        /* Initialize BLE driver */
        sid_err = ble_ext_ifc->init();
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_deinit_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_DEINIT_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (NULL == ble_ext_ifc)
        {
            CLI_LOG_ERROR("User mode BLE is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Deinitialize BLE driver */
        sid_err = ble_ext_ifc->deinit();

        /* Invalidate the interface */
        ble_ext_ifc = NULL;
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_factory_reset_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_FACTORY_RESET_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (ble_ext_ifc != NULL)
        {
            CLI_LOG_ERROR("User mode BLE must be deinitialized before the factory reset");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Create temporary BLE interface */
        static const sid_pal_ble_adapter_extended_interface_t * tmp_ble_ext_ifc = NULL;
        sid_err = sid_pal_ble_adapter_ext_create_extended_ifc(&tmp_ble_ext_ifc);
        SID_STM32_BLE_CLI_ASSERT_RESULT(sid_pal_ble_adapter_ext_create_extended_ifc, sid_err);

        if (NULL == tmp_ble_ext_ifc)
        {
            CLI_LOG_ERROR("Extended BLE interface is null");
            sid_err = SID_ERROR_UNINITIALIZED;
        }

        /* Perform factory reset */
        sid_err = tmp_ble_ext_ifc->factory_reset();

        /* Invalidate the interface */
        tmp_ble_ext_ifc = NULL;
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_beacon_bootstrap_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_BEACON_BOOTSTRAP_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (NULL == ble_ext_ifc)
        {
            CLI_LOG_ERROR("User mode BLE is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Activate virtual BLE beacon */
        sid_err = ble_ext_ifc->virtual_dev_activate(APP_BLE_CONFIG_BEACON_VIRT_DEV_ID, &ble_beacon_ctx);
        SID_STM32_BLE_CLI_ASSERT_RESULT(ble_ext_ifc->virtual_dev_activate, sid_err);

        /* Update advertising data to implement Eddystone-URL beacon pointing to https://www.st.com
         *
         * Every Eddystone frame type must contain the following PDU data types:
         * - The Complete List of 16-bit Service UUIDs as defined in The Bluetooth Core Specification Supplement (CSS) v5, Part A, § 1.1. 
         *   The Complete List of 16-bit Service UUIDs must contain the Eddystone Service UUID of 0xFEAA. This is included to allow background scanning on iOS devices.
         * - The Service Data data type, Ibid., § 1.11. The Service Data - 16 bit UUID data type must be the Eddystone Service UUID of 0xFEAA.
         */
        const uint8_t eddystone_url_adv[] = {
            3u, /* Length of AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST */
            AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST,
            (uint8_t)(SID_STM32_BLE_COEXIST_CLI_EDDYSTONE_SVC_UUID & 0xFFu),
            (uint8_t)(SID_STM32_BLE_COEXIST_CLI_EDDYSTONE_SVC_UUID >> 8),
            9u,
            AD_TYPE_SERVICE_DATA,
            (uint8_t)(SID_STM32_BLE_COEXIST_CLI_EDDYSTONE_SVC_UUID & 0xFFu),
            (uint8_t)(SID_STM32_BLE_COEXIST_CLI_EDDYSTONE_SVC_UUID >> 8),
            0x10u, /* Eddystone Frame Type - Eddystone-URL */
            0u, /* Calibrated Tx power at 0 m */
            0x01u, /* URL Scheme - https://www. */
            's',
            't',
            0x00u, /* .com */
        };

        sid_err = ble_ext_ifc->virtual_dev_set_adv_data(APP_BLE_CONFIG_BEACON_VIRT_DEV_ID, eddystone_url_adv, sizeof(eddystone_url_adv));
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_beacon_start_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_BEACON_START_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (NULL == ble_ext_ifc)
        {
            CLI_LOG_ERROR("User mode BLE is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Start advertiding */
        sid_err = ble_ext_ifc->virtual_dev_adv_start(APP_BLE_CONFIG_BEACON_VIRT_DEV_ID);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_beacon_stop_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_BEACON_STOP_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (NULL == ble_ext_ifc)
        {
            CLI_LOG_ERROR("User mode BLE is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Stop advertiding */
        sid_err = ble_ext_ifc->virtual_dev_adv_stop(APP_BLE_CONFIG_BEACON_VIRT_DEV_ID);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_beacon_terminate_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        if (argc != 0)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_BEACON_TERMINATE_CMD " takes no arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (NULL == ble_ext_ifc)
        {
            CLI_LOG_ERROR("User mode BLE is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Terminate virtual BLE beacon */
        sid_err = ble_ext_ifc->virtual_dev_terminate(APP_BLE_CONFIG_BEACON_VIRT_DEV_ID);
        ble_beacon_ctx = NULL;
        SID_STM32_BLE_CLI_ASSERT_RESULT(ble_ext_ifc->virtual_dev_terminate, sid_err);

        CLI_LOG_OUTCOME(sid_err);
    } while (0);

    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_peripheral_bootstrap_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        uint32_t periph_idx;

        if (argc != 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_PERIPH_BOOTSTRAP_CMD " takes exactly one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }
        else
        {
            periph_idx = (uint32_t)sid_ble_coexist_cli_utils_parse_input_num(argv[0]);

            if ((0u == periph_idx) || (periph_idx > SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT))
            {
                CLI_LOG_ERROR("Valid range for peripheral index is 1..%u", SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT);
                sid_err = SID_ERROR_INVALID_ARGS;
                break;
            }

            /* Convert 1..SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT notation to 0..(SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT - 1) */
            periph_idx--;
        }

        if (NULL == ble_ext_ifc)
        {
            CLI_LOG_ERROR("User mode BLE is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        sid_ble_ext_virtual_device_id_t periph_dev_id;
        switch (periph_idx)
        {
            case 0u:
                periph_dev_id = APP_BLE_CONFIG_PERIPH_LARGE_MTU_VIRT_DEV_ID;
                break;

            case 1u:
                periph_dev_id = APP_BLE_CONFIG_PERIPH_SMALL_MTU_VIRT_DEV_ID;
                break;

            default:
                periph_dev_id = 0xFFu; /* Invalid ID */
                break;
        }

        /* Activate virtual BLE peripheral */
        sid_err = ble_ext_ifc->virtual_dev_activate(periph_dev_id, &ble_periph_ctx[periph_idx]);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_peripheral_start_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        uint32_t periph_idx;

        if (argc != 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_PERIPH_START_CMD " takes exactly one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }
        else
        {
            periph_idx = (uint32_t)sid_ble_coexist_cli_utils_parse_input_num(argv[0]);

            if ((0u == periph_idx) || (periph_idx > SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT))
            {
                CLI_LOG_ERROR("Valid range for peripheral index is 1..%u", SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT);
                sid_err = SID_ERROR_INVALID_ARGS;
                break;
            }

            /* Convert 1..SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT notation to 0..(SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT - 1) */
            periph_idx--;
        }

        if (NULL == ble_ext_ifc)
        {
            CLI_LOG_ERROR("User mode BLE is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        sid_ble_ext_virtual_device_id_t periph_dev_id;
        switch (periph_idx)
        {
            case 0u:
                periph_dev_id = APP_BLE_CONFIG_PERIPH_LARGE_MTU_VIRT_DEV_ID;
                break;

            case 1u:
                periph_dev_id = APP_BLE_CONFIG_PERIPH_SMALL_MTU_VIRT_DEV_ID;
                break;

            default:
                periph_dev_id = 0xFFu; /* Invalid ID */
                break;
        }

        /* Ensure device is resumed if it was suspended previously */
        (void)ble_ext_ifc->virtual_dev_resume(periph_dev_id);

        /* Start advertiding */
        sid_err = ble_ext_ifc->virtual_dev_adv_start(periph_dev_id);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_peripheral_stop_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        uint32_t periph_idx;

        if (argc != 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_PERIPH_STOP_CMD " takes exactly one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }
        else
        {
            periph_idx = (uint32_t)sid_ble_coexist_cli_utils_parse_input_num(argv[0]);

            if ((0u == periph_idx) || (periph_idx > SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT))
            {
                CLI_LOG_ERROR("Valid range for peripheral index is 1..%u", SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT);
                sid_err = SID_ERROR_INVALID_ARGS;
                break;
            }

            /* Convert 1..SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT notation to 0..(SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT - 1) */
            periph_idx--;
        }

        if (NULL == ble_ext_ifc)
        {
            CLI_LOG_ERROR("User mode BLE is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        sid_ble_ext_virtual_device_id_t periph_dev_id;
        switch (periph_idx)
        {
            case 0u:
                periph_dev_id = APP_BLE_CONFIG_PERIPH_LARGE_MTU_VIRT_DEV_ID;
                break;

            case 1u:
                periph_dev_id = APP_BLE_CONFIG_PERIPH_SMALL_MTU_VIRT_DEV_ID;
                break;

            default:
                periph_dev_id = 0xFFu; /* Invalid ID */
                break;
        }

        /* Suspend the device - terminate all connections and advertisements */
        sid_err = ble_ext_ifc->virtual_dev_suspend(periph_dev_id);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_peripheral_terminate_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        uint32_t periph_idx;

        if (argc != 1)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_PERIPH_TERMINATE_CMD " takes exactly one argument");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }
        else
        {
            periph_idx = (uint32_t)sid_ble_coexist_cli_utils_parse_input_num(argv[0]);

            if ((0u == periph_idx) || (periph_idx > SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT))
            {
                CLI_LOG_ERROR("Valid range for peripheral index is 1..%u", SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT);
                sid_err = SID_ERROR_INVALID_ARGS;
                break;
            }

            /* Convert 1..SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT notation to 0..(SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT - 1) */
            periph_idx--;
        }

        if (NULL == ble_ext_ifc)
        {
            CLI_LOG_ERROR("User mode BLE is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        sid_ble_ext_virtual_device_id_t periph_dev_id;
        switch (periph_idx)
        {
            case 0u:
                periph_dev_id = APP_BLE_CONFIG_PERIPH_LARGE_MTU_VIRT_DEV_ID;
                break;

            case 1u:
                periph_dev_id = APP_BLE_CONFIG_PERIPH_SMALL_MTU_VIRT_DEV_ID;
                break;

            default:
                periph_dev_id = 0xFFu; /* Invalid ID */
                break;
        }

        /* Terminate virtual BLE peripheral */
        sid_err = ble_ext_ifc->virtual_dev_terminate(periph_dev_id);
        ble_periph_ctx[periph_idx] = NULL;
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/*----------------------------------------------------------------------------*/

static ace_status_t sid_ble_coexist_cli_peripheral_send_cmd(int32_t argc, const char **argv)
{
    sid_error_t sid_err;

    do
    {
        uint32_t periph_idx;

        if (argc != 2)
        {
            CLI_LOG_ERROR(SYNTAX_ERR);
            CLI_LOG_WARNING(SID_STM32_BLE_COEXIST_CLI_PERIPH_SEND_CMD " takes exactly two arguments");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }
        else
        {
            periph_idx = (uint32_t)sid_ble_coexist_cli_utils_parse_input_num(argv[0]);

            if ((0u == periph_idx) || (periph_idx > SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT))
            {
                CLI_LOG_ERROR("Valid range for peripheral index is 1..%u", SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT);
                sid_err = SID_ERROR_INVALID_ARGS;
                break;
            }

            /* Convert 1..SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT notation to 0..(SID_STM32_BLE_COEXIST_CLI_PERIPH_COUNT - 1) */
            periph_idx--;
        }

        const uint8_t * const data = (uint8_t *)argv[1];
        const uint32_t data_length = strlen(argv[1]);

        if (0u == data_length)
        {
            CLI_LOG_ERROR("Data cannot be empty");
            sid_err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (NULL == ble_ext_ifc)
        {
            CLI_LOG_ERROR("User mode BLE is not initialized");
            sid_err = SID_ERROR_UNINITIALIZED;
            break;
        }

        sid_ble_ext_virtual_device_id_t periph_dev_id;
        const sid_ble_cfg_uuid_info_t * char_uuid;
        switch (periph_idx)
        {
            case 0u:
                periph_dev_id = APP_BLE_CONFIG_PERIPH_LARGE_MTU_VIRT_DEV_ID;
                char_uuid = secure_char_with_notify;
                break;

            case 1u:
                periph_dev_id = APP_BLE_CONFIG_PERIPH_SMALL_MTU_VIRT_DEV_ID;
                char_uuid = non_secure_char_with_notify;
                break;

            default:
                periph_dev_id = 0xFFu; /* Invalid ID */
                char_uuid = NULL;
                break;
        }

        /* Update characteristic of the virtual BLE peripheral */
        sid_err = ble_ext_ifc->virtual_dev_update_char(periph_dev_id, char_uuid, data, data_length, TRUE);
    } while (0);

    CLI_LOG_OUTCOME(sid_err);
    CLI_LOG_FLUSH();

    return ACE_STATUS_OK;
}

/* Global function definitions -----------------------------------------------*/

void sid_ble_coexistence_cli_on_pairing_pass_key_request(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, uint32_t * const out_passkey)
{
    (void)device_ctx;
    (void)conn_ctx;

    /* Generate a random passkey */
    (void)sid_pal_crypto_rand((uint8_t *)(void *)out_passkey, sizeof(*out_passkey));

    /* Trim the passkey to be up to 6 digits */
    *out_passkey %= 1000000u;

    /* printout the passkey to the user */
    CLI_LOG_INFO("Numeric comparison passkey set to \e[1;4;32m%06u\e[0m", *out_passkey);
}

/*----------------------------------------------------------------------------*/

void sid_ble_coexistence_cli_on_pairing_numeric_comparison(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const uint32_t numeric_value, uint32_t * const out_numeric_value_matches)
{
    (void)device_ctx;
    (void)conn_ctx;

    CLI_LOG_INFO("Received numeric comparison value \e[1;4;32m%06u\e[0m", numeric_value);

    /* Automatically confirm the value. DON'T DO THIS IN PRODUCTION APPS */
    *out_numeric_value_matches = TRUE;
    CLI_LOG_INFO("Numeric comparison value automatically accepted by demo app");
}

/*----------------------------------------------------------------------------*/

void sid_ble_coexistence_cli_on_periph_data_received(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const struct sid_ble_ext_gatt_server_svc_ctx_s * const svc_ctx,
                                                     const struct sid_ble_ext_gatt_server_char_ctx_s * const char_ctx, const struct sid_ble_ext_gatt_server_char_desc_ctx_s * const desc_ctx, const uint8_t * const data, const uint32_t data_length)
{
    if (NULL == desc_ctx)
    {
        /* This is a characteristic value update */
        CLI_LOG_INFO("\"%s\", characteristic \"%s\" was written with %u bytes over connection 0x%04X" , device_ctx->device_cfg->device_name, char_ctx->char_def->char_name, data_length, conn_ctx->conn_id);
    }
    else
    {
        /* This is a characteristic descriptor update */
        CLI_LOG_INFO("\"%s\", characteristic \"%s\", descriptor \"%s\" was written with %u bytes over connection 0x%04X" , device_ctx->device_cfg->device_name, char_ctx->char_def->char_name, desc_ctx->desc_def->desc_name, data_length, conn_ctx->conn_id);
    }
    CLI_LOG_INFO("Raw data (hex):");
    CLI_LOG_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, data, data_length);
}

/*----------------------------------------------------------------------------*/

void sid_ble_coexistence_cli_on_periph_cccd_modified(const struct sid_ble_ext_virtual_device_ctx_s * const device_ctx, const sid_ble_ext_connection_ctx_t * const conn_ctx, const struct sid_ble_ext_gatt_server_svc_ctx_s * const svc_ctx,
                                                     const struct sid_ble_ext_gatt_server_char_ctx_s * const char_ctx, const sid_ble_ext_cccd_val_t cccd_val)
{
    if (cccd_val.notify_en != FALSE)
    {
        CLI_LOG_INFO("\"%s\", characteristic \"%s\", connection 0x%04X - notifications enabled" , device_ctx->device_cfg->device_name, char_ctx->char_def->char_name, conn_ctx->conn_id);
    }
    else
    {
        CLI_LOG_INFO("\"%s\", characteristic \"%s\", connection 0x%04X - notifications disabled" , device_ctx->device_cfg->device_name, char_ctx->char_def->char_name, conn_ctx->conn_id);
    }
}

/*----------------------------------------------------------------------------*/

sid_error_t sid_ble_coexistence_cli_init(void)
{
    return SID_CLI_REGISTER_SUB_COMMAND_SET(m_ble_coexistence_commands);
}
