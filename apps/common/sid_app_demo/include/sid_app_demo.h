/*
 * Copyright 2023 Amazon.com, Inc. or its affiliates. All rights reserved.
 *
 * AMAZON PROPRIETARY/CONFIDENTIAL
 *
 * You may not use this file except in compliance with the terms and
 * conditions set forth in the accompanying LICENSE.TXT file.
 *
 * THESE MATERIALS ARE PROVIDED ON AN "AS IS" BASIS. AMAZON SPECIFICALLY
 * DISCLAIMS, WITH RESPECT TO THESE MATERIALS, ALL WARRANTIES, EXPRESS,
 * IMPLIED, OR STATUTORY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
 */

#ifndef SID_APP_DEMO_H
#define SID_APP_DEMO_H
#include <stdint.h>
#include <stdbool.h>

#include <FreeRTOS.h>
#include <timers.h>
#include <queue.h>
#include <task.h>

#include <sid_api.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PAYLOAD_MAX_SIZE     255
#define APP_DEMO_BUTTONS_MAX 4
#define APP_DEMO_LED_MAX     4

enum event_type
{
    EVENT_TYPE_SIDEWALK,
    EVENT_FACTORY_RESET,
    EVENT_BUTTON_PRESS,
    EVENT_CONNECT_LINK_TYPE_1,
    EVENT_SET_DEVICE_PROFILE_LINK_TYPE_2,
    EVENT_NOTIFICATION_TIMER_FIRED,
    EVENT_BUTTON_PRESS_TIMER_FIRED,
    EVENT_SEND_MESSAGE
};

enum app_sidewalk_state
{
    STATE_SIDEWALK_INIT,
    STATE_SIDEWALK_READY,
    STATE_SIDEWALK_NOT_READY,
    STATE_SIDEWALK_SECURE_CONNECTION,
};

enum demo_app_state
{
    DEMO_APP_STATE_INIT,
    DEMO_APP_STATE_REGISTERED,
    DEMO_APP_STATE_NOTIFY_CAPABILITY,
    DEMO_APP_STATE_NOTIFY_SENSOR_DATA,
};

struct link_status
{
    enum sid_time_sync_status time_sync_status;
    uint32_t link_mask;
    uint32_t supported_link_mode[SID_LINK_TYPE_MAX_IDX];
};

struct app_demo_msg {
    enum sid_link_type link_type;
    enum sid_msg_type msg_type;
    struct sid_demo_msg_desc *msg_desc;
    struct sid_demo_msg *msg;
};

struct app_demo_rx_msg {
    uint16_t msg_id;
    size_t pld_size;
    uint8_t rx_payload[PAYLOAD_MAX_SIZE];
};

struct app_demo_tx_msg {
    struct sid_msg_desc desc;
    size_t pld_size;
    uint8_t tx_payload[PAYLOAD_MAX_SIZE];
};

struct app_demo_event {
    enum event_type type;
    void *data;
};

struct app_demo_hw_ifc {
    void (*turn_on_leds)(uint8_t idx);
    void (*turn_off_leds)(uint8_t idx);
    bool (*is_led_on)(uint8_t idx);
    int16_t (*temp_read)(void);
    void (*platform_reset)(void);
};

typedef struct app_context
{
    TaskHandle_t main_task;
    QueueHandle_t event_queue;
    TimerHandle_t cap_timer_handle;
    TimerHandle_t button_press_timer_handle;
    TimerHandle_t device_profile_timer_handle;
    struct sid_handle *sidewalk_handle;
    enum app_sidewalk_state sidewalk_state;
    enum demo_app_state app_state;
    struct link_status link_status;
    uint8_t *led_id_arr;                    // must be statically allocated
    uint8_t led_num;
    uint8_t *button_id_arr;                 // must be statically allocated
    uint8_t button_num;
    uint32_t button_press_mask;
    uint32_t button_notify_mask;
    uint32_t button_press_time_in_sec_id_arr[APP_DEMO_BUTTONS_MAX];
    bool button_event_pending_processing;
    uint8_t buffer[PAYLOAD_MAX_SIZE];
    struct app_demo_hw_ifc hw_ifc;
} app_context_t;

typedef struct receive_context
{
    TaskHandle_t receive_task;
    QueueHandle_t receive_event_queue;
} receive_context_t;


/**
 * Initialize app demo library
 *
 * @param[in] pointer to main task app context, must be statically allocated
 * @param[in] pointer to receive task context, must be statically allocated
 */
void demo_app_init(app_context_t *app_context, receive_context_t *rcv_context);

/**
 * Return pointer to sidewalk event callbacks. Must be used befor calling sid_init.
 *
 * @param[out] pointer to sidewalk callbacks
 */
struct sid_event_callbacks *demo_app_get_sid_event_callbacks(void);

/**
 * Process app main task even. Must be called when receive_event_queue receives a message.
 *
 * @param[in] pointer to receive event
 */
void demo_app_main_task_event_handler(struct app_demo_event *event);

/**
 * Process app receive task even. Must be called when event_queue receives a message.
 *
 * @param[in] pointer to demo app event
 */
void demo_app_receive_task_event_handler(struct app_demo_rx_msg *rx_msg);

/**
 * Pass notification to demo app that button was prssed.
 *
 * @param[in] button index
 */
void demo_app_button_event_handler(uint8_t idx);

/**
 * Pass notification to demo app that factory reset was triggered by user.
 *
 * @param[in] button index
 */
void demo_app_factory_reset_trigger_handler(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif   // SID_APP_DEMO_H
