/**
  ******************************************************************************
  * @file    app_sidewalk.c
  * @brief   Demo of the User Data Transfer (UDT) feature of STM32WLxx radio app
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

 /* Includes ------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_freertos.h"

#include <sid_api.h>
#include <sid_hal_reset_ifc.h>
#include <sid_log_control.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_common_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_900_cfg.h>
#include "stm32wlxx_radio_ext_ifc.h"
#include <sid_stm32_common_utils.h>

#if defined(NUCLEO_WBA52_BOARD) || defined(NUCLEO_WBA55_BOARD) || defined (NUCLEO_WBA65_BOARD)
#  include "stm32wbaxx_nucleo.h"
#endif

#include SID_APP_VERSION_HEADER
#include <sid_sdk_version.h>

#include "target/memory.h"

#include "app_ble_config.h"
#include "app_900_config.h"
#include "app_common.h"
#include "app_conf.h"
#include <stm32_mcu_info.h>

#if (CFG_LPM_LEVEL != 0)
#include <stm32_lpm.h>
#endif /* (CFG_LPM_LEVEL != 0) */

#include <sid_asd_cli.h>
#include <sid_config_cli.h>
#include <sid_qa.h>

/* Private defines -----------------------------------------------------------*/

#define SID_MAIN_THREAD_PWR_MEAS_UNLOCKED_FLAG (1u << 0)

#if (STM32WLxx_RADIO_CFG_USE_STATUS_LED != 0u)
#  define RADIO_CFG_USE_STATUS_LED
#endif /* STM32WLxx_RADIO_CFG_USE_STATUS_LED */

/* Private typedef -----------------------------------------------------------*/

typedef struct app_context 
{
    osThreadId_t main_task;
    osThreadId_t udt_task;
} app_context_t;

typedef __PACKED_UNION {
    __PACKED_STRUCT {
        uint8_t build;
        uint8_t patch;
        uint8_t minor;
        uint8_t major;
    };
    uint32_t raw;
} hal_version_info_t;

/* Private variables ---------------------------------------------------------*/

static app_context_t app_context = {
    .main_task = NULL,
    .udt_task = NULL,
};

static struct sid_log_control_severity sid_log_settings_storage;

static uint8_t inbound_data_buffer[32];

/* Private constants ---------------------------------------------------------*/

static const osThreadAttr_t sidewalk_stack_task_attributes = {
    .name         = "Sidewalk Stack Task",
    .priority     = SID_MAIN_TASK_PRIO,
    .stack_size   = SID_MAIN_TASK_STACK_SIZE,
    .attr_bits    = TASK_DEFAULT_ATTR_BITS,
    .cb_mem       = TASK_DEFAULT_CB_MEM,
    .cb_size      = TASK_DEFAULT_CB_SIZE,
    .stack_mem    = TASK_DEFAULT_STACK_MEM,
};

static const osThreadAttr_t user_data_transfer_task_attributes = {
    .name         = "UDT Example Task",
    .priority     = SID_MAIN_TASK_PRIO,
    .stack_size   = SID_MAIN_TASK_STACK_SIZE,
    .attr_bits    = TASK_DEFAULT_ATTR_BITS,
    .cb_mem       = TASK_DEFAULT_CB_MEM,
    .cb_size      = TASK_DEFAULT_CB_SIZE,
    .stack_mem    = TASK_DEFAULT_STACK_MEM,
};

/* Private function prototypes -----------------------------------------------*/

static void sidewalk_stack_task_entry(void *context);
static void user_data_transfer_task_entry(void *context);

/* Private function definitions -----------------------------------------------*/

static void pwr_meas_mode_enter(void)
{
    SID_PAL_LOG_FLUSH();

    struct sid_log_control_severity sid_log_settings_current;
    /* Get active log config */
    sid_log_control_get_severity(&sid_log_settings_current);
    /* Store a backup */
    sid_log_settings_storage = sid_log_settings_current;
    sid_log_settings_current.enable = false;
    sid_log_control_set_severity(&sid_log_settings_current);

    /* Temporarily disable Tx/Rx LEDs */
#if defined(SID_RADIO_PLATFORM_LR11XX) && (LR1110_RADIO_CFG_USE_STATUS_LED != 0u)
    const lr1110_radio_device_config_t * const radio_cfg = get_radio_cfg();
#endif /* SID_RADIO_PLATFORM_LR11XX */

#if defined(SID_RADIO_PLATFORM_SX126X) && (SX126X_RADIO_CFG_USE_STATUS_LED != 0u)
    const sx126x_radio_device_config_t * const radio_cfg = get_radio_cfg();
#endif /* SID_RADIO_PLATFORM_SX126X */

#if defined(SID_RADIO_PLATFORM_S2LP) && (S2LP_RADIO_CFG_USE_STATUS_LED != 0u)
    const s2_lp_radio_device_config_t * const radio_cfg = get_radio_cfg();
#endif /* SID_RADIO_PLATFORM_S2LP */

#if defined(SID_RADIO_PLATFORM_STM32WLXX_APP) && (STM32WLxx_RADIO_CFG_USE_STATUS_LED != 0u)
    const stm32wlxx_app_radio_device_config_t * const radio_cfg = get_radio_cfg();
#endif /* SID_RADIO_PLATFORM_STM32WLXX_APP */

#ifdef RADIO_CFG_USE_STATUS_LED
    if (radio_cfg->gpio.rx_led != HALO_GPIO_NOT_CONNECTED)
    {
        (void)sid_pal_gpio_input_mode(radio_cfg->gpio.rx_led, SID_PAL_GPIO_INPUT_DISCONNECT);
        (void)sid_pal_gpio_set_direction(radio_cfg->gpio.rx_led, SID_PAL_GPIO_DIRECTION_INPUT);
    }

    if (radio_cfg->gpio.tx_led != HALO_GPIO_NOT_CONNECTED)
    {
        (void)sid_pal_gpio_input_mode(radio_cfg->gpio.tx_led, SID_PAL_GPIO_INPUT_DISCONNECT);
        (void)sid_pal_gpio_set_direction(radio_cfg->gpio.tx_led, SID_PAL_GPIO_DIRECTION_INPUT);
    }
#endif /* RADIO_CFG_USE_STATUS_LED */

#if CFG_LED_SUPPORTED
    BSP_LED_Off(LED_GREEN);
#endif /* CFG_LED_SUPPORTED */

#if (CFG_LPM_LEVEL != 0)
    /* Enable Stop LPM. CLI may not be available after this point since UART and DMA are not active in Stop mode */
    UTIL_LPM_SetStopMode((1 << CFG_LPM_APP), UTIL_LPM_ENABLE);
#endif /* (CFG_LPM_LEVEL != 0) */
}

/*-----------------------------------------------------------------------------*/

static void pwr_meas_mode_exit(void)
{
#if (CFG_LPM_LEVEL != 0)
    /* Disable Stop LPM to keep CLI UART and related DMA channel running */
    UTIL_LPM_SetStopMode((1 << CFG_LPM_APP), UTIL_LPM_DISABLE);
#endif /* (CFG_LPM_LEVEL != 0) */


    sid_log_control_set_severity(&sid_log_settings_storage);

#if CFG_LED_SUPPORTED
    BSP_LED_On(LED_GREEN);
#endif /* CFG_LED_SUPPORTED */

#if defined(SID_RADIO_PLATFORM_LR11XX) && (LR1110_RADIO_CFG_USE_STATUS_LED != 0u)
    const lr1110_radio_device_config_t * const radio_cfg = get_radio_cfg();
#endif /* SID_RADIO_PLATFORM_LR11XX */

    /* Re-enable Tx/Rx LEDs */
#if defined(SID_RADIO_PLATFORM_SX126X) && (SX126X_RADIO_CFG_USE_STATUS_LED != 0u)
    const sx126x_radio_device_config_t * const radio_cfg = get_radio_cfg();
#endif /* SID_RADIO_PLATFORM_SX126X */

#if defined(SID_RADIO_PLATFORM_S2LP) && (S2LP_RADIO_CFG_USE_STATUS_LED != 0u)
    const s2_lp_radio_device_config_t * const radio_cfg = get_radio_cfg();
#endif /* SID_RADIO_PLATFORM_S2LP */

#if defined(SID_RADIO_PLATFORM_STM32WLXX_APP) && (STM32WLxx_RADIO_CFG_USE_STATUS_LED != 0u)
    const stm32wlxx_app_radio_device_config_t * const radio_cfg = get_radio_cfg();
#endif /* SID_RADIO_PLATFORM_STM32WLXX_APP */

#ifdef RADIO_CFG_USE_STATUS_LED
    if (radio_cfg->gpio.rx_led != HALO_GPIO_NOT_CONNECTED)
    {
        (void)sid_pal_gpio_output_mode(radio_cfg->gpio.rx_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
        (void)sid_pal_gpio_set_direction(radio_cfg->gpio.rx_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
    }

    if (radio_cfg->gpio.tx_led != HALO_GPIO_NOT_CONNECTED)
    {
        (void)sid_pal_gpio_output_mode(radio_cfg->gpio.tx_led, SID_PAL_GPIO_OUTPUT_PUSH_PULL);
        (void)sid_pal_gpio_set_direction(radio_cfg->gpio.tx_led, SID_PAL_GPIO_DIRECTION_OUTPUT);
    }
#endif /* RADIO_CFG_USE_STATUS_LED */
}

/*-----------------------------------------------------------------------------*/

static void pwr_meas_is_blocked(void)
{
    /* sid_qa processing is blocked and requires the owning RTOS task to yield */
    (void)osThreadFlagsWait(SID_MAIN_THREAD_PWR_MEAS_UNLOCKED_FLAG, osFlagsWaitAll, osWaitForever);
}

/*-----------------------------------------------------------------------------*/

static void pwr_meas_is_unblocked(void)
{
    /* sid_qa processing is unblocked therefore the owning RTOS task can be resumed */
    osThreadFlagsSet(app_context.main_task, SID_MAIN_THREAD_PWR_MEAS_UNLOCKED_FLAG);
}

/*-----------------------------------------------------------------------------*/

static void reboot_func(void)
{
    (void)sid_hal_reset(SID_HAL_RESET_NORMAL);
}

/*-----------------------------------------------------------------------------*/

static void set_sub_ghz_cfg(struct sid_sub_ghz_links_config *sub_ghz_cfg)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    if (!sub_ghz_cfg) {
        SID_PAL_LOG_ERROR("Null pointer passed while setting sub ghz cfg");
    }
    struct sid_sub_ghz_links_config *cfg = (struct sid_sub_ghz_links_config *)app_get_sub_ghz_config();
    memcpy(cfg, sub_ghz_cfg, sizeof(*cfg));
#endif
}

/*-----------------------------------------------------------------------------*/

static void sidewalk_stack_task_entry(void *context)
{
    struct sid_config config = {
        .callbacks = NULL,
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_1
        .link_config = app_get_sidewalk_ble_config(),
#else
        .link_config = NULL,
#endif
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        .sub_ghz_link_config = app_get_sub_ghz_config(),
#else
        .sub_ghz_link_config = NULL,
#endif
    };

    struct sid_qa_pwr_meas_if pwr_meas_if = {
        .enter_func     = pwr_meas_mode_enter,
        .exit_func      = pwr_meas_mode_exit,
        .blocked_func   = pwr_meas_is_blocked,
        .unblocked_func = pwr_meas_is_unblocked,
    };

    sid_qa_set_config(&config, &pwr_meas_if);

    SID_PAL_LOG_INFO("sid QA cli application started...");

    while (1) {
        SID_PAL_LOG_FLUSH();

        sid_cli_process();
        sid_qa_process(QA_PROC_NO_WAIT);
    }

    (void)sid_platform_deinit();
    SID_PAL_LOG_INFO("sid QA cli application terminated");
    SID_PAL_LOG_FLUSH();
    osThreadExit();
}

/*-----------------------------------------------------------------------------*/

static void _on_incoming_user_data_cb(const uint8_t * const data, const uint32_t data_len)
{
    if ((data != NULL) && (data_len > 0u))
    {
        /* Store the received data into a static buffer - original data may get corrupted/overwritten by the momemnt UART printout is processed */
        uint32_t len_to_copy = data_len > sizeof(inbound_data_buffer) ? sizeof(inbound_data_buffer) : data_len;
        SID_STM32_UTIL_fast_memcpy(inbound_data_buffer, data, len_to_copy);

        /* Ensure the demo string is always terminated properly */
        inbound_data_buffer[len_to_copy - 1] = '\0';

        /* Print out received data */
        SID_PAL_LOG_INFO("%s", (char *)inbound_data_buffer);
    }
}

/*-----------------------------------------------------------------------------*/

static void user_data_transfer_task_entry(void *context)
{
    sid_error_t sid_err;
    uint8_t cnt = 0u;
    char msg_buf[16];

    /* Configure callback for the incoming data from STM32WLxx side */
    sid_err = sid_pal_radio_stm32wlxx_set_user_data_received_cb(_on_incoming_user_data_cb);
    if (SID_ERROR_NONE == sid_err)
    {
        SID_PAL_LOG_INFO("Configured UDT callback");
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to set UDT callback. Error %d", (int32_t)sid_err);
    }

    /* Send out periodic ping messages */
    while (1)
    {
        osDelay(5000u);

        sprintf(msg_buf, "UDT ping %u", cnt);

        sid_err = sid_pal_radio_stm32wlxx_send_user_data((void *)msg_buf, strlen(msg_buf) + 1u, FALSE);
        if (sid_err != SID_ERROR_NONE)
        {
            /* Failed to send message (e.g. driver is not ready, message queue is full, etc.) */
            SID_PAL_LOG_WARNING("Unable to send UDT message. Error %d", (int32_t)sid_err);
        }
        else
        {
            /* Increment the counter for the next message */
            cnt++;
        }
    }

    osThreadExit();
}

/* Global function definitions -----------------------------------------------*/

void SID_APP_Init(void)
{
    /* Adjust Sidewalk log level dynamically - this is needed if the firmware is compiled using
     * Sidewalk SDK static library and the library was assembled with different debug level
     */
    struct sid_log_control_severity sid_log_settings;
    sid_log_control_get_severity(&sid_log_settings);
    if (SID_PAL_LOG_LEVEL < sid_log_settings.level)
    {
        sid_log_settings.level = SID_PAL_LOG_LEVEL;
        sid_log_control_set_severity(&sid_log_settings);
    }

    /* Printout application version info */
    SID_PAL_LOG_INFO("Application name: %s", SID_APP_PROJECT_NAME);
    SID_PAL_LOG_INFO("Application version %s", SID_APP_PROJECT_VERSION_STRING);
    SID_PAL_LOG_INFO("Application build type: %s", SID_APP_PROJECT_BUILD_TYPE);
    SID_PAL_LOG_INFO("Application commit hash: %s", SID_APP_PROJECT_COMMIT_HASH_STRING);
    SID_PAL_LOG_INFO("Application commit description: %s", SID_APP_PROJECT_COMMIT_DESCRIPTION);
    SID_PAL_LOG_INFO("Sidewalk SDK: %u.%u.%u-%u", SID_SDK_MAJOR_VERSION, SID_SDK_MINOR_VERSION, SID_SDK_PATCH_VERSION, SID_SDK_BUILD_VERSION);
    SID_PAL_LOG_INFO("FreeRTOS Kernel: %s", tskKERNEL_VERSION_NUMBER);

    /* CubeMX pack version */
    const hal_version_info_t cubemx_fw_pack_ver = {
        .raw = HAL_GetHalVersion(),
    };
    SID_PAL_LOG_INFO("STM32CubeWBA: %u.%u.%u", cubemx_fw_pack_ver.major, cubemx_fw_pack_ver.minor, cubemx_fw_pack_ver.patch);

    /* Printout MCU details */
    stm32_mcu_info_t mcu_info = stm32_mcu_info_describe_host();
    SID_PAL_LOG_INFO("Host MCU: %s (0x%X), revision: %s (0x%X)", mcu_info.device_name, mcu_info.device_id, mcu_info.rev_name, mcu_info.rev_id);

    platform_parameters_t platform_parameters = {
        .mfg_store_region.addr_start = MANUFACTURE_FLASH_START,
        .mfg_store_region.addr_end   = MANUFACTURE_FLASH_END,

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        .platform_init_parameters.radio_cfg = get_radio_cfg(),
#endif
    };

    sid_error_t ret_code = sid_platform_init(&platform_parameters);
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Sidewalk Platform Init err: %d", ret_code);
        SID_PAL_ASSERT(0);
    }

    ret_code = sid_cli_init();
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Sidewalk cli init failed err: %d", ret_code);
        SID_PAL_ASSERT(0);
    }

    ret_code = sid_config_cli_init();
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Sidewalk config cli init failed err: %d", ret_code);
        SID_PAL_ASSERT(0);
    }

    struct sid_qa_callbacks qa_callbacks = {
        .reboot_cmd = reboot_func,
        .set_sub_ghz_cfg = set_sub_ghz_cfg,
    };
    ret_code = sid_qa_init(&qa_callbacks);
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Sidewalk qa init failed err: %d", ret_code);
        SID_PAL_ASSERT(0);
    }

    app_context.main_task = osThreadNew(sidewalk_stack_task_entry, &app_context, &sidewalk_stack_task_attributes);
    if (NULL == app_context.main_task)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk processing thread. No memory");
        SID_PAL_ASSERT(0);
    }

    app_context.udt_task = osThreadNew(user_data_transfer_task_entry, &app_context, &user_data_transfer_task_attributes);
    if (NULL == app_context.udt_task)
    {
        SID_PAL_LOG_ERROR("Can't create UDT Example thread. No memory");
        SID_PAL_ASSERT(0);
    }

#if (CFG_LPM_LEVEL != 0)
    /* Disable Stop LPM by default */
    UTIL_LPM_SetStopMode((1 << CFG_LPM_APP), UTIL_LPM_DISABLE);
#endif /* (CFG_LPM_LEVEL != 0) */
}
