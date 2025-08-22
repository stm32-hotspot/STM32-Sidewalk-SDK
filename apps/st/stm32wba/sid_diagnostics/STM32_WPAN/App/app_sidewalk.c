/**
  ******************************************************************************
  * @file    app_sidewalk.c
  * @brief   Standardized Sidewalk Diagnostics app
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
#include <sid_log_control.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_common_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_900_cfg.h>

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
#include <sid_diagnostics_cli.h>
#include <sid_on_dev_cert_cli.h>

/* Private defines -----------------------------------------------------------*/

#define SID_MAIN_THREAD_PWR_MEAS_UNLOCKED_FLAG (1u << 0)

/* Private typedef -----------------------------------------------------------*/

typedef struct app_context 
{
    osThreadId_t main_task;
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
};

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

/* Private function prototypes -----------------------------------------------*/

static void sidewalk_stack_task_entry(void *context);

/* Private function definitions -----------------------------------------------*/

static void sidewalk_stack_task_entry(void *context)
{
    /* Diagnostics CLI init needs to be inside thread as it may auto-start
     * radio test mode, subject to config stored in flash.
     */
    sid_cli_init();
    sid_diagnostics_cli_init();
    sid_on_dev_cert_cli_init();

    SID_PAL_LOG_INFO("sid_diagnostics application started...");

    while (1) {
        SID_PAL_LOG_FLUSH();

        sid_cli_process();
    }

    (void)sid_platform_deinit();
    SID_PAL_LOG_INFO("sid_diagnostics application terminated");
    SID_PAL_LOG_FLUSH();
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

    app_context.main_task = osThreadNew(sidewalk_stack_task_entry, &app_context, &sidewalk_stack_task_attributes);
    if (NULL == app_context.main_task)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk processing thread. No memory");
        SID_PAL_ASSERT(0);
    }

#if (CFG_LPM_LEVEL != 0)
    /* Disable Stop LPM by default */
    UTIL_LPM_SetStopMode((1 << CFG_LPM_APP), UTIL_LPM_DISABLE);
#endif /* (CFG_LPM_LEVEL != 0) */
}
