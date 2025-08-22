/**
  ******************************************************************************
  * @file    app_sidewalk.c
  * @brief   Standardized Sidewalk Bulk Data Transfer (SBDT) demo app
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <sid_app_sbdt_demo.h>
#include <sid_log_control.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_common_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_timer_ifc.h>
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>

#include SID_APP_VERSION_HEADER
#include <sid_sdk_version.h>

#include "target/memory.h"

#include "app_ble_config.h"
#include "app_common.h"
#include <stm32_mcu_info.h>
#if CFG_LED_SUPPORTED
#  include "led_indication.h"
#endif
#include "sid_pal_gpio_ext_ifc.h"
#if defined(NUCLEO_WBA52_BOARD) || defined(NUCLEO_WBA55_BOARD) || defined (NUCLEO_WBA65_BOARD)
#  include "stm32wbaxx_nucleo.h"
#endif

#if (CFG_BUTTON_SUPPORTED != 0) && (CFG_LPM_STDBY_SUPPORTED != 0)
#include "stm32_lpm.h"
#include <sid_pal_gpio_ext_ifc.h>
#endif

/* Private defines -----------------------------------------------------------*/

#if (CFG_BUTTON_SUPPORTED != 0)
#  define BUTTON_LONG_PRESS_MS                (1500u)
#  define DEMO_FACTORY_RESET_BUTTON_ID        (0u)
#  define DEMO_CONNECT_LINK_TYPE_1_BUTTON_ID  (1u)
#  define DEMO_SEND_MESSAGE_BUTTON_ID         (2u)
#  define DEMO_SBDT_CANCEL_TRANSFER_BUTTON_ID (3u)
#endif /* CFG_BUTTON_SUPPORTED */

#define FLASH_WORD_SIZE                       (16u) /* in bytes */
#define ALIGN_TO_FLASH_WORD_BOUNDARY(_VALUE_) ((((size_t)(_VALUE_) + (FLASH_WORD_SIZE - 1u)) / FLASH_WORD_SIZE) * FLASH_WORD_SIZE)

/* Private macro -------------------------------------------------------------*/

#define OS_MS_TO_TICKS( xTimeInMs )    ( ( uint32_t ) ( ( ( uint32_t ) ( xTimeInMs ) * ( uint32_t ) osKernelGetTickFreq() ) / ( uint32_t ) 1000U ) )

/* Private typedef -----------------------------------------------------------*/

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

static struct sid_event_callbacks * sbdt_demo_callbacks;
static struct sid_event_callbacks sbdt_demo_callback_wrappers;

static sid_pal_timer_t reset_delay_timer;

static uint32_t sbdt_storage_start_addr = 0u;

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

UTIL_PLACE_IN_SECTION(".text.sbdt_storage") ALIGN(FLASH_PAGE_SIZE)
static const uint8_t sbdt_storage[FLASH_PAGE_SIZE * 48u];

/* Private function prototypes -----------------------------------------------*/

static void        on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context);
static void        on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context);
static void        on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc, void *context);
static void        on_sidewalk_status_changed(const struct sid_status *status, void *context);
static uint32_t    crc32_compute(const uint8_t * const data, const uint32_t size, const uint32_t * const prev_crc);
static void        reset_delay_timer_cb(void *cookie, sid_pal_timer_t *originator);
static void        trigger_delayed_reset(void);
static void        platform_reset(void);
static void        platform_flash_init(void);
static uint32_t    platform_flash_get_free_space(void);
static bool        platform_flash_write(uint32_t offset, void *data, size_t size);
static bool        platform_flash_check(uint32_t size, uint32_t expected_crc);
static bool        platform_flash_finalize(uint32_t size, uint32_t crc);
static sid_error_t init_and_start_link(app_context_t * const context, struct sid_config * const config);

#if (CFG_BUTTON_SUPPORTED == 1)
static void button1_irq_handler(uint32_t pin, void * callback_arg);
static void button2_irq_handler(uint32_t pin, void * callback_arg);
static void button3_irq_handler(uint32_t pin, void * callback_arg);
#endif

static void sidewalk_stack_task_entry(void *context);

/* Private function definitions ----------------------------------------------*/

static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context)
{
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_RCV_OK);
#endif

    /* Invoke the original callback */
    if (sbdt_demo_callbacks->on_msg_received != NULL)
    {
        sbdt_demo_callbacks->on_msg_received(msg_desc, msg, context);
    }
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context)
{
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_SENT_OK);
#endif

    /* Invoke the original callback */
    if (sbdt_demo_callbacks->on_msg_sent != NULL)
    {
        sbdt_demo_callbacks->on_msg_sent(msg_desc, context);
    }
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc, void *context)
{
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_SEND_ERROR);
#endif

    /* Invoke the original callback */
    if (sbdt_demo_callbacks->on_send_error != NULL)
    {
        sbdt_demo_callbacks->on_send_error(error, msg_desc, context);
    }
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_status_changed(const struct sid_status *status, void *context)
{
    switch (status->state)
    {
        case SID_STATE_READY:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_CONNECTED);
#endif
            break;

        case SID_STATE_NOT_READY:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_BONDING);
#endif
            break;

        case SID_STATE_ERROR:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_ERROR);
#endif
            break;

        default:
            /* Do nothing about it */
            break;
    }

    /* Invoke the original callback */
    if (sbdt_demo_callbacks->on_status_changed != NULL)
    {
        sbdt_demo_callbacks->on_status_changed(status, context);
    }
}

/*----------------------------------------------------------------------------*/

static uint32_t crc32_compute(const uint8_t * const data, const uint32_t size, const uint32_t * const prev_crc)
{
    uint32_t crc;

    crc = (prev_crc == NULL) ? 0xFFFFFFFFu : ~(*prev_crc);

    for (uint32_t i = 0u; i < size; i++)
    {
        crc = crc ^ data[i];
        for (uint32_t j = 8u; j > 0u; j--)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & ((crc & 1u) ? 0xFFFFFFFFu : 0u));
        }
    }

    crc = ~crc;
    return crc;
}

/*----------------------------------------------------------------------------*/

static void reset_delay_timer_cb(void *cookie, sid_pal_timer_t *originator)
{
    (void)cookie;
    (void)originator;

    SID_PAL_LOG_INFO("System reset");
    SID_PAL_LOG_FLUSH();

    HAL_NVIC_SystemReset();
}

/*----------------------------------------------------------------------------*/

static void trigger_delayed_reset(void)
{
    struct sid_timespec now = {};
    struct sid_timespec sid_tsec = {
        .tv_sec  = 5u,
        .tv_nsec = 0u,
    };

    sid_pal_uptime_now(&now);
    sid_time_add(&now, &sid_tsec);
    sid_pal_timer_arm(&reset_delay_timer, SID_PAL_TIMER_PRIO_CLASS_PRECISE, &now, NULL);
}

/*----------------------------------------------------------------------------*/

static void platform_reset(void)
{
    trigger_delayed_reset();
}

/*----------------------------------------------------------------------------*/

static void platform_flash_init(void)
{
    sid_error_t err;

    sbdt_storage_start_addr = (uint32_t)(void *)sbdt_storage;
    SID_PAL_ASSERT((sbdt_storage_start_addr - FLASH_BASE) % FLASH_PAGE_SIZE == 0u);

    err = sid_pal_timer_init(&reset_delay_timer, reset_delay_timer_cb, NULL);
    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to initilaize reset delay timer. Error: %d", (int32_t)err);
    }
}

/*----------------------------------------------------------------------------*/

static uint32_t platform_flash_get_free_space(void)
{
    uint32_t free_bytes = sizeof(sbdt_storage);
    return free_bytes;
}

/*----------------------------------------------------------------------------*/

static bool platform_flash_write(uint32_t offset, void *data, size_t size)
{
    bool success = false;
    HAL_StatusTypeDef hal_err;
    const uint32_t write_start_addr = sbdt_storage_start_addr + offset;

    do
    {
        /* Validate inputs */
        if ((NULL == data) || (0u == size) || ((offset + size) > sizeof(sbdt_storage)))
        {
            SID_PAL_LOG_ERROR("SBDT write failed: invalid parameters. Offset: %u, size: %u, src addr: 0x%08X", offset, size, (uint32_t)data);
            break;
        }

        /* STM32WBA5x can write by 16 bytes chunks only */
        if ((write_start_addr % FLASH_WORD_SIZE) != 0u)
        {
            SID_PAL_LOG_ERROR("SBDT write failed: start address 0x%08X is not aligned to flash word size (16 bytes)", write_start_addr);
            break;
        }

        if ((size % FLASH_WORD_SIZE) != 0u)
        {
            SID_PAL_LOG_ERROR("SBDT write failed: write length of %u is not aligned to flash word size (16 bytes)", size);
            break;
        }

        /* Prepare flash write operation */
        hal_err = HAL_FLASH_Unlock();
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("SBDT write failed: cannot unlock the flash. Error 0x%x", hal_err);
            break;
        }

        uint32_t write_address = write_start_addr;
        uint32_t read_address = (uint32_t)data;
        size_t bytes_processed = 0u;

        /* Write data in quad-word chunks */
        while (bytes_processed < size)
        {
            /* Erase flash page on writing to the first page quadword */
            if (((write_address - FLASH_BASE) % FLASH_PAGE_SIZE) == 0u)
            {
                FLASH_EraseInitTypeDef erase_Config = {
                    .TypeErase = FLASH_TYPEERASE_PAGES,
                    .Page = ((write_address - FLASH_BASE) / FLASH_PAGE_SIZE),
                    .NbPages = 1u,
                };
                uint32_t ulPageError = 0u;

                SID_PAL_LOG_DEBUG("SBDT: erasing flash page at 0x%08X", write_address);
                __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
                hal_err = HAL_FLASHEx_Erase(&erase_Config, &ulPageError);

                if (hal_err != HAL_OK)
                {
                    SID_PAL_LOG_ERROR("SBDT: failed to erase flash page at 0x%08X. Error 0x%x", write_address, hal_err);
                    break;
                }
            }

            /* Write down data chunk */
            __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
            hal_err = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, write_address, read_address);
            if (hal_err != HAL_OK)
            {
                SID_PAL_LOG_ERROR("SBDT write failed at location 0x%08X. Error 0x%x", write_address, hal_err);
                (void)HAL_FLASH_Lock();
                break;
            }
            else
            {
                SID_PAL_LOG_INFO("SBDT: wrote 16 bytes at 0x%08X. Total bytes written: %u, bytes remaining: %u", write_address, (bytes_processed + FLASH_WORD_SIZE), (size - (bytes_processed + FLASH_WORD_SIZE)));
            }

            /* Move to the next quad-word */
            write_address   += FLASH_WORD_SIZE;
            read_address    += FLASH_WORD_SIZE;
            bytes_processed += FLASH_WORD_SIZE;
        }

        if (hal_err != HAL_OK)
        {
            /* Propagate chunk write error */
            break;
        }

        /* Lock the flash after writing */
        hal_err = HAL_FLASH_Lock();
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("SBDT write failed: cannot lock the flash. Error 0x%x", hal_err);
            break;
        }

        SID_PAL_LOG_INFO("SBDT: %u bytes written to address 0x%08X", size, write_start_addr);
        success = true;
    } while (0);

    return success;
}

/*----------------------------------------------------------------------------*/

static bool platform_flash_check(uint32_t size, uint32_t expected_crc)
{
    const uint32_t calculated_crc = crc32_compute((uint8_t *)sbdt_storage_start_addr, size, NULL);
    SID_PAL_LOG_INFO("SBDT calculated CRC: 0x%08X, expcted: 0x%08X, length = %u", calculated_crc, expected_crc, size);
    return calculated_crc == expected_crc;
}

/*----------------------------------------------------------------------------*/

static bool platform_flash_finalize(uint32_t size, uint32_t crc)
{
    HAL_StatusTypeDef hal_err;

    /* Invalidate ICache to write any pending data into flash */
    hal_err = HAL_ICACHE_Invalidate();
    if (hal_err != HAL_OK)
    {
        SID_PAL_LOG_ERROR("SBDT write finalization failed - cache invalidation error 0x%x", hal_err);
        return false;
    }

    trigger_delayed_reset();
    return true;
}

/*----------------------------------------------------------------------------*/

static sid_error_t init_and_start_link(app_context_t * const context, struct sid_config * const config)
{
    sid_error_t ret = SID_ERROR_GENERIC;
    struct sid_handle *sid_handle = NULL;

    /* Validate inputs */
    if (NULL == context)
    {
        SID_PAL_LOG_ERROR("Sidewalk context cannot be null");
        ret = SID_ERROR_NULL_POINTER;
        goto error;
    }

    if (NULL == config)
    {
        SID_PAL_LOG_ERROR("Sidewalk config cannot be null");
        ret = SID_ERROR_NULL_POINTER;
        goto error;
    }

    /* Make sure Sidewalk is not initialized already */
    if (context->sidewalk_handle != NULL)
    {
        SID_PAL_LOG_ERROR("Sidewalk is initialized already");
        ret = SID_ERROR_ALREADY_INITIALIZED;
        goto error;
    }

    /* Initialize Sidewalk */
    ret = sid_init(config, &sid_handle);
    if (ret != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to initialize Sidewalk stack, link_mask:%x, err:%d", (int)config->link_mask, (int)ret);
        goto error;
    }

    /* Register sidewalk handle to the application context */
    context->sidewalk_handle = sid_handle;

    /* Start the Sidewalk stack with the selected link type */
    ret = sid_start(sid_handle, config->link_mask);
    if (ret != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to start Sidewalk stack, link_mask:%x, err:%d", (int)config->link_mask, (int)ret);
        goto error;
    }

    return ret;

error:
    context->sidewalk_handle = NULL;
    config->link_mask = 0;
    return ret;
}

/*----------------------------------------------------------------------------*/

#if (CFG_BUTTON_SUPPORTED == 1)
static void button1_irq_handler(uint32_t pin, void * callback_arg)
{
    static uint32_t uStart = 0u;
    static uint32_t uEnd   = 0u;
    uint8_t pinState;
    sid_error_t ret;

    (void)callback_arg;

    ret = sid_pal_gpio_read(pin, &pinState);
    if (SID_ERROR_NONE == ret)
    {
        if (GPIO_PIN_RESET == pinState)
        {
            uStart = osKernelGetTickCount();
        }
        else
        {
            uEnd = osKernelGetTickCount();

            if ((uEnd - uStart) < OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS))
            {
                /* Short-press event */
                demo_app_button_event_handler(DEMO_SBDT_CANCEL_TRANSFER_BUTTON_ID);
            }
            else
            {
                /* Long-press event */
                demo_app_button_event_handler(DEMO_FACTORY_RESET_BUTTON_ID);
            }
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to read B1 state. Error %d", (int32_t)ret);
    }
}

/*----------------------------------------------------------------------------*/

static void button2_irq_handler(uint32_t pin, void * callback_arg)
{
    (void)pin;
    (void)callback_arg;

    demo_app_button_event_handler(DEMO_CONNECT_LINK_TYPE_1_BUTTON_ID);
}

/*----------------------------------------------------------------------------*/

static void button3_irq_handler(uint32_t pin, void * callback_arg)
{
#  if defined(SID_RADIO_PLATFORM_LR11XX) || defined(SID_RADIO_PLATFORM_SX126X) /* Semtech shields occupy B2 GPIO pin for BUSY signal, use extra-long B3 press instead */
    static uint32_t uStart = 0u;
    static uint32_t uEnd   = 0u;
    uint8_t pinState;
    sid_error_t ret;

    (void)callback_arg;

    ret = sid_pal_gpio_read(pin, &pinState);
    if (SID_ERROR_NONE == ret)
    {
        if (GPIO_PIN_RESET == pinState)
        {
            uStart = osKernelGetTickCount();
        }
        else
        {
            uEnd = osKernelGetTickCount();

            if ((uEnd - uStart) >= (4u * OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS)))
            {
                /* Extra-long-press event */
                demo_app_button_event_handler(DEMO_CONNECT_LINK_TYPE_1_BUTTON_ID);
            }
            else
            {
                /* Short-press event */
                demo_app_button_event_handler(DEMO_SEND_MESSAGE_BUTTON_ID);
            }
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to read B3 state. Error %d", (int32_t)ret);
    }
#  else
    (void)pin;
    (void)callback_arg;

    demo_app_button_event_handler(DEMO_SEND_MESSAGE_BUTTON_ID);
#  endif /* SID_RADIO_PLATFORM_LR11XX || SID_RADIO_PLATFORM_SX126X */
}
#endif /* CFG_BUTTON_SUPPORTED */

/*----------------------------------------------------------------------------*/

static void sidewalk_stack_task_entry(void *context)
{
    app_context_t *app_context = (app_context_t *)context;
    sid_error_t ret;

    struct sid_end_device_characteristics dev_ch = {
        .type = SID_END_DEVICE_TYPE_STATIC,
        .power_type = SID_END_DEVICE_POWERED_BY_BATTERY_AND_LINE_POWER,
        .qualification_id = 0x0001u,
    };

    /* Get function pointers to the original event callbacks of the unified demo app */
    sbdt_demo_callbacks = demo_app_get_sid_event_callbacks();

    /* Prepopulate even callback wrappers with original function pointers */
    sbdt_demo_callback_wrappers = *sbdt_demo_callbacks;

    /* Inject event callback wrappers where needed */
    sbdt_demo_callback_wrappers.on_msg_received   = on_sidewalk_msg_received;
    sbdt_demo_callback_wrappers.on_msg_sent       = on_sidewalk_msg_sent;
    sbdt_demo_callback_wrappers.on_send_error     = on_sidewalk_send_error;
    sbdt_demo_callback_wrappers.on_status_changed = on_sidewalk_status_changed;

    struct sid_config config = {
        .link_mask = SID_LINK_TYPE_1,
        .dev_ch = dev_ch,
        .callbacks = &sbdt_demo_callback_wrappers,
        .link_config = app_get_sidewalk_ble_config(),
        .sub_ghz_link_config = NULL,
    };

    /* Start Sidewalk link */
    ret = init_and_start_link(app_context, &config);
    if (ret != SID_ERROR_NONE)
    {
        goto error;
    }

    while (1)
    {
        struct app_demo_event event;

        if (osMessageQueueGet(app_context->event_queue, &event, NULL, osWaitForever) == osOK)
        {
            /* Call unified event processor */
            demo_app_main_task_event_handler(&event);
        }
    }

error:
    if (app_context->sidewalk_handle != NULL)
    {
        sid_stop(app_context->sidewalk_handle, SID_LINK_TYPE_1);
        sid_deinit(app_context->sidewalk_handle);
        app_context->sidewalk_handle = NULL;
    }

#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_OFF);
#endif

    SID_PAL_LOG_INFO("Sidewalk SBDT demo terminated due to error");
    Error_Handler();

    osThreadExit(); /* Normally this line is not reachable, but keep it if Error_Handler somehow returns */
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
    };

    sid_error_t ret_code = sid_platform_init(&platform_parameters);
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Sidewalk Platform Init err: %d", ret_code);
        SID_PAL_ASSERT(0);
    }

#if CFG_LED_SUPPORTED
    ret_code = led_indication_init();
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Sidewalk Init LED indication err: %d", ret_code);
        SID_PAL_ASSERT(0);
    }
#endif

#if (CFG_BUTTON_SUPPORTED == 1)
    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);

    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);

    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);

    /* Give the GPIO pins some time to reach a stable state before enabling IRQs */
    const uint32_t gpio_settle_delay = 75u * (SystemCoreClock / 1000000u); /* Approx. 75us */
    for (uint32_t i = 0u; i < gpio_settle_delay; i++)
    {
        /* Can't use timers here because initialization is not finished yet */
        __NOP();
    }

    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_IRQ_TRIGGER_EDGE, button1_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);

    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_IRQ_TRIGGER_FALLING, button2_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);

    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_IRQ_TRIGGER_EDGE, button3_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#endif /* CFG_BUTTON_SUPPORTED */

    static app_context_t demo_app_context = {
        .hw_ifc = {
            .platform_reset                = platform_reset,
            .platform_flash_init           = platform_flash_init,
            .platform_flash_get_free_space = platform_flash_get_free_space,
            .platform_flash_write          = platform_flash_write,
            .platform_flash_check          = platform_flash_check,
            .platform_flash_finalize       = platform_flash_finalize,
        },
    };

    demo_sbdt_app_init(&demo_app_context);

    demo_app_context.main_task = osThreadNew(sidewalk_stack_task_entry, &demo_app_context, &sidewalk_stack_task_attributes);
    if (NULL == demo_app_context.main_task)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk processing thread. No memory");
        SID_PAL_ASSERT(0);
    }
}
