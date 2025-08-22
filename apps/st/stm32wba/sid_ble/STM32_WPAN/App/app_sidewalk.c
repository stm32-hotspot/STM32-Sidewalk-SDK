/**
  ******************************************************************************
  * @file    app_sidewalk.c
  * @brief   Sidewalk-over-BLE app implementation
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
#if defined(NUCLEO_WBA52_BOARD) || defined(NUCLEO_WBA55_BOARD) || defined(NUCLEO_WBA65_BOARD)
#  include "stm32wbaxx_nucleo.h"
#endif

#if (CFG_BUTTON_SUPPORTED != 0) && (CFG_LPM_STDBY_SUPPORTED != 0)
#include "stm32_lpm.h"
#include <sid_pal_gpio_ext_ifc.h>
#endif

/* Private defines -----------------------------------------------------------*/

#define SID_MSG_QUEUE_LEN       (10u)

#define DEMO_MESSAGE_DELAY      (60000u)

#define BUTTON_LONG_PRESS_MS    (1500u)

/* Private macro -------------------------------------------------------------*/

#define OS_MS_TO_TICKS( xTimeInMs )    ( ( uint32_t ) ( ( ( uint32_t ) ( xTimeInMs ) * ( uint32_t ) osKernelGetTickFreq() ) / ( uint32_t ) 1000U ) )

/* Private typedef -----------------------------------------------------------*/

enum event_type
{
    EVENT_TYPE_START_LINK,
    EVENT_TYPE_STOP_LINK,
    EVENT_TYPE_TOGGLE_LINK,
    EVENT_TYPE_SIDEWALK,
    EVENT_TYPE_SEND_HELLO,
    EVENT_TYPE_SET_BATTERY_LEVEL,
    EVENT_FACTORY_RESET,
    EVENT_TYPE_CONNECTION_REQUEST,
    EVENT_TYPE_REGISTRATION_COMPLETED,
    EVENT_TYPE_STANDBY
};

enum app_state
{
    STATE_INIT,
    STATE_SIDEWALK_READY,
    STATE_SIDEWALK_NOT_READY,
    STATE_SIDEWALK_SECURE_CONNECTION,
};

struct link_status
{
    uint32_t link_mask;
    uint32_t supported_link_mode[SID_LINK_TYPE_MAX_IDX];
};

typedef struct app_context
{
    osThreadId_t main_task;
    osMessageQueueId_t event_queue;
    struct sid_handle *sidewalk_handle;
    enum app_state state;
    struct link_status link_status;
    uint8_t counter;
    bool connection_request;
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

static osMessageQueueId_t g_event_queue;

static osSemaphoreId_t registration_ready_semaphore;
static osSemaphoreId_t connection_ready_semaphore;
static osThreadId_t demo_counter_task;

/* Indicates if Sidewalk registration process is pending */
static bool registration_pending = false;

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

static const osSemaphoreAttr_t registration_ready_sem_attributes = {
    .name       = "Sidewalk Registration Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t connection_ready_sem_attributes = {
    .name       = "Connection Ready Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osThreadAttr_t demo_counter_task_attributes = {
    .name         = "Demo Counter Task",
    .priority     = DEMO_COUNTER_TASK_PRIO,
    .stack_size   = DEMO_COUNTER_TASK_STACK_SIZE,
    .attr_bits    = TASK_DEFAULT_ATTR_BITS,
    .cb_mem       = TASK_DEFAULT_CB_MEM,
    .cb_size      = TASK_DEFAULT_CB_SIZE,
    .stack_mem    = TASK_DEFAULT_STACK_MEM,
};

/* Private function prototypes -----------------------------------------------*/

static void queue_event(osMessageQueueId_t queue, enum event_type event);
static void on_sidewalk_event(bool in_isr, void *context);
static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context);
static void on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context);
static void on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc, void *context);
static void on_sidewalk_status_changed(const struct sid_status *status, void *context);
static void on_sidewalk_factory_reset(void *context);
static void set_fake_ble_battery_level(app_context_t *context);
static void send_ping(app_context_t *app_context);
static void factory_reset(app_context_t *context);
static void toggle_connection_request(app_context_t *context);
#if (CFG_BUTTON_SUPPORTED == 1)
static void button1_irq_handler(uint32_t pin, void * callback_arg);
static void button2_irq_handler(uint32_t pin, void * callback_arg);
static void button3_irq_handler(uint32_t pin, void * callback_arg);
#endif

static void sidewalk_stack_task_entry(void *context);
static void demo_counter_task_entry(void *context);

/* Private function definitions ----------------------------------------------*/

static void queue_event(osMessageQueueId_t queue, enum event_type event)
{
    (void)osMessageQueuePut(queue, &event, 0u, 0u);
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_event(bool in_isr, void *context)
{
    (void)in_isr;

    app_context_t *app_context = (app_context_t *)context;
    queue_event(app_context->event_queue, EVENT_TYPE_SIDEWALK);
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context)
{
    SID_PAL_LOG_INFO("Received message(type: %d, link_mode: %d, id: %u size %u)", (int)msg_desc->type,
                             (int)msg_desc->link_mode, msg_desc->id, msg->size);
    SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, msg->data, msg->size);
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_RCV_OK);
#endif
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context)
{
    SID_PAL_LOG_INFO("Sent message(type: %d, id: %u)", (int)msg_desc->type, msg_desc->id);
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_SENT_OK);
#endif
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc, void *context)
{
    SID_PAL_LOG_ERROR("Failed to send message(type: %d, id: %u), err:%d",
                  (int)msg_desc->type, msg_desc->id, (int)error);
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_SEND_ERROR);
#endif
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_status_changed(const struct sid_status *status, void *context)
{
    app_context_t *app_context = (app_context_t *)context;
    SID_PAL_LOG_INFO("Status changed: %d", (int)status->state);

    switch (status->state)
    {
        case SID_STATE_READY:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_CONNECTED);
#endif
            if (false == registration_pending)
            {
                /* Notify the user app that Sidewalk link can now be used */
                app_context->state = STATE_SIDEWALK_READY;
                osSemaphoreRelease(connection_ready_semaphore);
            }
            app_context->connection_request = false;
            break;

        case SID_STATE_NOT_READY:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_BONDING);
#endif
            app_context->state = STATE_SIDEWALK_NOT_READY;
            break;

        case SID_STATE_ERROR:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_ERROR);
#endif
            SID_PAL_LOG_ERROR("Sidewalk error: %d", (int)sid_get_error(app_context->sidewalk_handle));
            Error_Handler();
            break;

        case SID_STATE_SECURE_CHANNEL_READY:
            app_context->state = STATE_SIDEWALK_SECURE_CONNECTION;
            break;

        default:
            /* Do nothing about it */
            break;
    }

    SID_PAL_LOG_INFO("Registration Status = %d, Time Sync Status = %d and Link Status Mask = %x",
                     status->detail.registration_status, status->detail.time_sync_status,
                     status->detail.link_status_mask);

    if ((registration_pending != false) && (SID_STATUS_REGISTERED == status->detail.registration_status))
    {
        /* Device registration completed */
        SID_PAL_LOG_INFO("Sidewalk Device Registration done");
        queue_event(g_event_queue, EVENT_TYPE_REGISTRATION_COMPLETED);
    }

    app_context->link_status.link_mask= status->detail.link_status_mask;
    for (int32_t i = 0; i < SID_LINK_TYPE_MAX_IDX; i++)
    {
        app_context->link_status.supported_link_mode[i] = status->detail.supported_link_modes[i];
        SID_PAL_LOG_INFO("Link %d Mode %x", i, status->detail.supported_link_modes[i]);
    }
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_factory_reset(void *context)
{
    SID_PAL_LOG_INFO("Factory reset notification received from sid api");
    SID_PAL_LOG_FLUSH();
    (void)sid_hal_reset(SID_HAL_RESET_NORMAL);
}

/*----------------------------------------------------------------------------*/

static void set_fake_ble_battery_level(app_context_t *context)
{
    static uint8_t fake_bat_lev = 70;
    ++fake_bat_lev;
    if (fake_bat_lev > 100) {
        fake_bat_lev = 0;
    }
    sid_error_t ret = sid_option(context->sidewalk_handle, SID_OPTION_BLE_BATTERY_LEVEL,
                                     &fake_bat_lev, sizeof(fake_bat_lev));
    if (ret != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("Set_fake_ble_battery_level: failed setting sidewalk option!");
    }
    else {
        SID_PAL_LOG_DEBUG("Set battery level to %d", fake_bat_lev);
    }
}

/*----------------------------------------------------------------------------*/

static void send_ping(app_context_t *app_context)
{
    if (app_context->state == STATE_SIDEWALK_READY ||
        app_context->state == STATE_SIDEWALK_SECURE_CONNECTION) {
        SID_PAL_LOG_INFO("Sending counter update: %d", app_context->counter);
        struct sid_msg msg = {.data = (uint8_t*)&app_context->counter, .size = sizeof(uint8_t)};
        struct sid_msg_desc desc = {
            .type = SID_MSG_TYPE_NOTIFY,
            .link_type = SID_LINK_TYPE_ANY,
            .link_mode = SID_LINK_MODE_CLOUD,
        };

        if ((app_context->link_status.link_mask & SID_LINK_TYPE_1) &&
            (app_context->link_status.supported_link_mode[SID_LINK_TYPE_1_IDX] & SID_LINK_MODE_MOBILE)) {
            desc.link_mode = SID_LINK_MODE_MOBILE;
        }

        sid_error_t ret = sid_put_msg(app_context->sidewalk_handle, &msg, &desc);
        if (ret != SID_ERROR_NONE) {
            SID_PAL_LOG_ERROR("Failed queueing data, err:%d", (int) ret);
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_SEND_ERROR);
#endif
        } else {
            SID_PAL_LOG_DEBUG("Queued data message id:%u", desc.id);
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_SEND_ENQUEUED);
#endif
        }
        app_context->counter++;
    } else {
        SID_PAL_LOG_WARNING("Send_ping: Sidewalk is not ready yet!");
#if CFG_LED_SUPPORTED
        (void)led_indication_set(LED_INDICATE_SEND_ERROR);
#endif
    }
}

/*----------------------------------------------------------------------------*/

static void factory_reset(app_context_t * const context)
{
    /* Validate inputs */
    if (NULL == context)
    {
        SID_PAL_LOG_ERROR("Factory reset request failed - context cannot be null");
        return;
    }

    sid_error_t ret = sid_set_factory_reset(context->sidewalk_handle);
    if (ret != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Notification of factory reset to sid api failed! Performing NVIC reset");
        SID_PAL_LOG_FLUSH();
        (void)sid_hal_reset(SID_HAL_RESET_NORMAL);
    }
    else
    {
        SID_PAL_LOG_DEBUG("Wait for SID API to notify to proceed with factory reset!");
    }
}

/*----------------------------------------------------------------------------*/

static void toggle_connection_request(app_context_t *context)
{
    if (context->state == STATE_SIDEWALK_READY) {
        SID_PAL_LOG_WARNING("Toggle_connection_request: Sidewalk ready, operation not valid");
    } else {
        bool next = !context->connection_request;
        SID_PAL_LOG_DEBUG("%s connection request", next ? "Set" : "Clear");
        sid_error_t ret = sid_ble_bcn_connection_request(context->sidewalk_handle, next);
        if (ret == SID_ERROR_NONE) {
            context->connection_request = next;
        } else {
            SID_PAL_LOG_ERROR("Connection request failed %d", ret);
        }
    }
}

/*----------------------------------------------------------------------------*/

static sid_error_t init_and_start_link(app_context_t * const context, struct sid_config * const config)
{
    sid_error_t ret = SID_ERROR_GENERIC;
    struct sid_handle *sid_handle = NULL;
    struct sid_status sid_status;

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

    /* Initialize state indication to the user app */
    context->state = STATE_SIDEWALK_NOT_READY;

    /* Initialize Sidewalk in BLE mode to check device registration status */
    /* WARNING: starting in LoRa-only mode is not allowed if the device is not registered. Calling sid_init() will always return an error */
    config->link_mask = SID_LINK_TYPE_1; /* BLE link */

    ret = sid_init(config, &sid_handle);
    if (ret != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to initialize Sidewalk stack, link_mask:%x, err:%d", (int)config->link_mask, (int)ret);
        goto error;
    }

    /* Register sidewalk handle to the application context */
    context->sidewalk_handle = sid_handle;

    ret = sid_get_status(sid_handle, &sid_status);
    if (ret != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to get Sidewalk stack status, err:%d", (int32_t)ret);
        goto error;
    }

    SID_PAL_LOG_INFO("Sidewalk registration status: %s", sid_status.detail.registration_status == SID_STATUS_REGISTERED ? "Registered" : "Not registered");

    if (SID_STATUS_REGISTERED == sid_status.detail.registration_status)
    {
        /* Indicate to the app that Sidewalk device registration is completed */
        queue_event(g_event_queue, EVENT_TYPE_REGISTRATION_COMPLETED);
    }
    else
    {
        /* Indicate registration process is pending */
        registration_pending = true;
    }

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

static sid_error_t destroy_link(app_context_t *app_context, struct sid_config *const config)
{
    sid_error_t ret;
    do
    {
        if (NULL == app_context)
        {
            ret = SID_ERROR_UNINITIALIZED;
            break;
        }

        if ((NULL == app_context->sidewalk_handle) || (NULL == config))
        {
            ret = SID_ERROR_INVALID_ARGS;
            break;
        }

        ret = sid_stop(app_context->sidewalk_handle, config->link_mask);
        if(ret != SID_ERROR_NONE)
        {
            break;
        }
        ret = sid_deinit(app_context->sidewalk_handle);
        if(ret != SID_ERROR_NONE)
        {
            break;
        }
        app_context->sidewalk_handle = NULL;
#if CFG_LED_SUPPORTED
        (void)led_indication_set(LED_INDICATE_OFF);
#endif
    } while (0);

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

            if((uEnd - uStart) < OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS))
            {
                /* Short-press event */
                queue_event(g_event_queue, EVENT_TYPE_TOGGLE_LINK);
            }
            else
            {
                /* Long-press event */
                queue_event(g_event_queue, EVENT_FACTORY_RESET);
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

    queue_event(g_event_queue, EVENT_TYPE_CONNECTION_REQUEST);
}

/*----------------------------------------------------------------------------*/

static void button3_irq_handler(uint32_t pin, void * callback_arg)
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

            if((uEnd - uStart) < OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS))
            {
                /* Short-press event */
                queue_event(g_event_queue, EVENT_TYPE_SEND_HELLO);
            }
            else
            {
                /* Long-press event */
                queue_event(g_event_queue, EVENT_TYPE_STANDBY);
            }
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to read B3 state. Error %d", (int32_t)ret);
    }
}
#endif /* CFG_BUTTON_SUPPORTED */

/*----------------------------------------------------------------------------*/

static void sidewalk_stack_task_entry(void *context)
{
    SID_PAL_LOG_INFO("Sidewalk demo started");

    app_context_t *app_context = (app_context_t *)context;
    sid_error_t ret;
    osStatus_t os_status;

    struct sid_event_callbacks event_callbacks = {
        .context           = app_context,
        .on_event          = on_sidewalk_event, /* Called from ISR context */
        .on_msg_received   = on_sidewalk_msg_received, /* Called from sid_process() */
        .on_msg_sent       = on_sidewalk_msg_sent,  /* Called from sid_process() */
        .on_send_error     = on_sidewalk_send_error, /* Called from sid_process() */
        .on_status_changed = on_sidewalk_status_changed, /* Called from sid_process() */
        .on_factory_reset  = on_sidewalk_factory_reset, /* Called from sid_process */
    };

    struct sid_config config = {
        .link_mask           = SID_LINK_TYPE_1,
        .callbacks           = &event_callbacks,
        .link_config         = app_get_sidewalk_ble_config(),
        .sub_ghz_link_config = NULL,
    };

    /* Performing a cold start of the Sidewalk since device registration status is unknown at this point */
    queue_event(g_event_queue, EVENT_TYPE_START_LINK);

    while (1)
    {
        enum event_type event;

        if (osMessageQueueGet (app_context->event_queue, &event, NULL, osWaitForever) == osOK)
        {
            switch (event)
            {
                case EVENT_TYPE_START_LINK:
                    /* Performing a hot start of the Sidewalk */
                    ret = init_and_start_link(app_context, &config);
                    if (ret != SID_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Failed to initialize Sidewalk. Error code: %d", (int)ret);
                        goto error;
                    }
                    break;

                case EVENT_TYPE_TOGGLE_LINK:
                    if(app_context->sidewalk_handle == NULL) 
                    {
                        queue_event(g_event_queue, EVENT_TYPE_START_LINK);
                    }
                    else
                    {
                        queue_event(g_event_queue, EVENT_TYPE_STOP_LINK);
                    }  
                    break;

                case EVENT_TYPE_STOP_LINK:                    
                    if (app_context->sidewalk_handle != NULL)
                    {
                        ret = destroy_link(app_context, &config);
                        SID_PAL_ASSERT(ret == SID_ERROR_NONE);
                    }
                    break;

                case EVENT_TYPE_SIDEWALK:
                    if ((NULL == app_context) || (NULL == app_context->sidewalk_handle))
                    {
                        /*  This may happen if Sidewalk link was terminated but the event queue already contained some events - just ignore them */
                        break;
                    }
                    ret = sid_process(app_context->sidewalk_handle);
                    if ((ret != SID_ERROR_NONE) && (ret != SID_ERROR_STOPPED))
                    {
                        SID_PAL_LOG_ERROR("Error processing Sidewalk event, err:%d", (int)ret);
                        goto error;
                    }
                    else if (SID_ERROR_STOPPED == ret)
                    {
                        SID_PAL_LOG_DEBUG("Sidewalk processing discarded - Sidewalk stack is stopped");
                    }
                    else
                    {
                        /* Nothing to do, everything is fine */
                    }
                    break;

                case EVENT_TYPE_SEND_HELLO:
                    send_ping(app_context);
                    break;

                case EVENT_TYPE_SET_BATTERY_LEVEL:
                    set_fake_ble_battery_level(app_context);
                    break;

                case EVENT_TYPE_REGISTRATION_COMPLETED:
                    registration_pending = false;

                    /* Indicate to the app that Sidewalk device registration is completed */
                    os_status = osSemaphoreRelease(registration_ready_semaphore);
                    if (os_status != osOK)
                    {
                        SID_PAL_LOG_ERROR("Failed to indicate Sidewalk registration completion. Error code : %d", (int32_t)os_status);
                        goto error;
                    }
                    break;

                case EVENT_FACTORY_RESET:
                    factory_reset(app_context);
                    break;

                case EVENT_TYPE_CONNECTION_REQUEST:
                    toggle_connection_request(app_context);
                    break;

#if (CFG_BUTTON_SUPPORTED != 0) && (CFG_LPM_STDBY_SUPPORTED != 0)
                case EVENT_TYPE_STANDBY:
                    if ((app_context != NULL) && (app_context->sidewalk_handle != NULL))
                    {
                        ret = destroy_link(app_context, &config);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Error entering in standby mode: unable to terminate Sidewalk link. Error code: %d", (int)ret);
                            goto error;
                        }
                    }

                    ret = sid_pal_gpio_ext_ifc_set_as_wakeup(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_IRQ_TRIGGER_RISING);
                    if (ret != SID_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Error entering in standby mode: unable to configure wakeup pin, err:%d", (int)ret);
                        goto error;
                    }

                    led_indication_set(LED_INDICATE_OFF);
                    UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);
                    break;
#endif /* CFG_BUTTON_SUPPORTED && CFG_LPM_STDBY_SUPPORTED */

                default:
                    /* Ignore unknown events */
                    break;
            }
        }
    }

error:
    if (demo_counter_task != NULL)
    {
        (void)osThreadTerminate(demo_counter_task);
        demo_counter_task = NULL;
    }

    if (app_context->sidewalk_handle != NULL)
    {
        sid_stop(app_context->sidewalk_handle, SID_LINK_TYPE_1);
        sid_deinit(app_context->sidewalk_handle);
        app_context->sidewalk_handle = NULL;
    }

#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_OFF);
#endif

    SID_PAL_LOG_INFO("Sidewalk demo terminated due to error");
    Error_Handler();

    osThreadExit(); /* Normally this line is not reachable, but keep it if Error_Handler somehow returns */
}

/*----------------------------------------------------------------------------*/

static void demo_counter_task_entry(void *context)
{
    app_context_t *app_context = (app_context_t *)context;
    osStatus_t status;

    SID_PAL_LOG_INFO("Demo Counter Thread: waiting for the Sidewalk registration to be completed...");
    status = osSemaphoreAcquire(registration_ready_semaphore, osWaitForever);
    if (status != osOK)
    {
        SID_PAL_LOG_ERROR("Demo Counter Thread - failed to wait for Sidewalk device registration to be completed");
        goto error;
    }
    SID_PAL_LOG_INFO("Demo Counter Thread: Sidewalk device is registered, proceeding with the demo...");

    while (1)
    {
        if ((app_context->state != STATE_SIDEWALK_READY) && (app_context->state != STATE_SIDEWALK_SECURE_CONNECTION))
        {
            SID_PAL_LOG_DEBUG("Demo Counter Thread: initiated Sidewalk connection request");
            queue_event(g_event_queue, EVENT_TYPE_CONNECTION_REQUEST);
        }

        status = osSemaphoreAcquire(connection_ready_semaphore, 15000u);
        if (status != osOK)
        {
            if (osErrorTimeout == status)
            {
                SID_PAL_LOG_WARNING("Demo Counter Thread: Sidewalk connection is not available");
            }
            else
            {
                SID_PAL_LOG_ERROR("Demo Counter Thread: unable to establish Sidewalk connection. Error code: %d", status);
            }
            continue;
        }

        while ((STATE_SIDEWALK_READY == app_context->state) || (STATE_SIDEWALK_SECURE_CONNECTION == app_context->state))
        {
            SID_PAL_LOG_DEBUG("Demo Counter Thread: sending out counter update");
            queue_event(g_event_queue, EVENT_TYPE_SEND_HELLO);
            osDelay(DEMO_MESSAGE_DELAY);
        }

        SID_PAL_LOG_DEBUG("Demo Counter Thread: Sidewalk connection lost");
    }

error:
    SID_PAL_LOG_INFO("Demo Counter Thread: Terminated due to error");
    SID_PAL_LOG_FLUSH();
    Error_Handler();

    /* Normally the lines below are not reachable, but keep them if Error_Handler somehow returns */
    demo_counter_task = NULL;
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

    g_event_queue = osMessageQueueNew(SID_MSG_QUEUE_LEN, sizeof(enum event_type), NULL);
    if (g_event_queue == NULL)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk message queue. No memory");
        SID_PAL_ASSERT(0);
    }

    static app_context_t app_context = {
        .event_queue = NULL,
        .main_task = NULL,
        .sidewalk_handle = NULL,
        .state = STATE_INIT,
    };

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

    app_context.event_queue = g_event_queue;
    app_context.main_task = osThreadNew(sidewalk_stack_task_entry, &app_context, &sidewalk_stack_task_attributes);
    if (NULL == app_context.main_task)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk processing thread. No memory");
        SID_PAL_ASSERT(0);
    }

    /* Create a semaphore to monitor Sidewalk registration status */
    registration_ready_semaphore = osSemaphoreNew(1u, 0u, &registration_ready_sem_attributes);
    if (NULL == registration_ready_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk Registration Status semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

    /* Create a semaphore to monitor Sidewalk connection status */
    connection_ready_semaphore = osSemaphoreNew(1, 0, &connection_ready_sem_attributes);
    if (NULL == connection_ready_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk connection status semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

    demo_counter_task = osThreadNew(demo_counter_task_entry, &app_context, &demo_counter_task_attributes);
    if (NULL == demo_counter_task)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk demo thread. No memory");
        SID_PAL_ASSERT(0);
    }
}
