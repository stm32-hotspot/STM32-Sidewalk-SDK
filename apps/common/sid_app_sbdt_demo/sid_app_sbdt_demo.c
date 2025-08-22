/*
 * Copyright 2024 Amazon.com, Inc. or its affiliates.  All rights reserved.
 *
 * AMAZON PROPRIETARY/CONFIDENTIAL
 *
 * You may not use this file except in compliance with the terms and conditions
 * set forth in the accompanying LICENSE.TXT file.  This file is a Modifiable
 * File, as defined in the accompanying LICENSE.TXT file.
 *
 * THESE MATERIALS ARE PROVIDED ON AN "AS IS" BASIS. AMAZON SPECIFICALLY
 * DISCLAIMS, WITH RESPECT TO THESE MATERIALS, ALL WARRANTIES, EXPRESS,
 * IMPLIED, OR STATUTORY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
 */

#include <sid_app_sbdt_demo.h>
#include <sid_sdk_version.h>

#include <sid_demo_parser.h>
#include <sid_demo_types.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_storage_kv_ifc.h>

#include <FreeRTOS.h>
#include <timers.h>
#include <queue.h>
#include <task.h>
#include <sid_api.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define MAIN_TASK_STACK_SIZE (2048 / sizeof(configSTACK_DEPTH_TYPE))

#define MSG_QUEUE_LEN 10

#define PAYLOAD_MAX_SIZE 255
#define MSG_LOG_BLOCK_SIZE 80

#define FILE_TRANSFER_OTA_GROUP 0xB
#define FILE_TRANSFER_CRC_KEY 1
#define FILE_OTA_PENDING_KEY 2
#define FILE_OTA_CHECK_VERSION 3

enum OTA_STATUS {
    OTA_STATUS_INVALID = 0,
    OTA_STATUS_PLATFORM_VALIDATION_FAILED = 1,
    OTA_STATUS_PLATFORM_VALIDATION_SUCCESS = 2,
};

static const struct sid_demo_sdk_version current_sdk_version = {
    .major = SID_SDK_MAJOR_VERSION,
    .minor = SID_SDK_MINOR_VERSION,
    .build = SID_SDK_BUILD_VERSION,
    .patch = SID_SDK_PATCH_VERSION,
};

static app_context_t *g_app_context;

static uint32_t crc32_compute(uint8_t const *data, uint32_t size, uint32_t const *prev_crc)
{
    uint32_t crc;

    crc = (prev_crc == NULL) ? 0xFFFFFFFF : ~(*prev_crc);
    for (uint32_t i = 0; i < size; i++) {
        crc = crc ^ data[i];
        for (uint32_t j = 8; j > 0; j--) {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
    }
    return ~crc;
}

static void queue_event(QueueHandle_t queue, enum event_type event, void *data, bool in_isr)
{
    struct app_demo_event evt = {.type = event, .data = data};
    BaseType_t result = pdFALSE;
    if (in_isr) {
        BaseType_t task_woken = pdFALSE;
        result = xQueueSendFromISR(queue, &evt, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    } else {
        result = xQueueSend(queue, &evt, 0);
    }

    if (result == pdFALSE) {
        SID_PAL_LOG_INFO("%s:%d", __func__, result);
    }
}

static void queue_rx_msg(QueueHandle_t queue, struct app_demo_rx_msg *rx_msg, bool in_isr)
{
    if (in_isr) {
        BaseType_t task_woken = pdFALSE;
        xQueueSendToBackFromISR(queue, rx_msg, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    } else {
        xQueueSendToBack(queue, rx_msg, 0);
    }
}

static void log_sid_msg(const struct sid_msg *msg)
{
    char *data = (char *)msg->data;
    SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, data, msg->size);
}

static void get_active_link_type(enum sid_link_type *link_type)
{
    if (g_app_context->link_status.link_mask & SID_LINK_TYPE_1) {
        *link_type = SID_LINK_TYPE_1;
    } else if (g_app_context->link_status.link_mask & SID_LINK_TYPE_2) {
        *link_type = SID_LINK_TYPE_2;
    } else if (g_app_context->link_status.link_mask & SID_LINK_TYPE_3) {
        *link_type = SID_LINK_TYPE_3;
    }
}

static void send_msg(struct sid_msg_desc *desc, struct sid_msg *msg)
{
    if (g_app_context->sidewalk_state == STATE_SIDEWALK_READY
        || g_app_context->sidewalk_state == STATE_SIDEWALK_SECURE_CONNECTION) {
        sid_error_t ret = sid_put_msg(g_app_context->sidewalk_handle, msg, desc);
        if (ret != SID_ERROR_NONE) {
            SID_PAL_LOG_ERROR("failed queueing data, err:%d", (int)ret);
        } else {
            SID_PAL_LOG_INFO("queued data message id:%u", desc->id);
            log_sid_msg(msg);
        }
    } else {
        SID_PAL_LOG_ERROR("sidewalk is not ready yet!");
    }
}

static void factory_reset(void)
{
    sid_error_t ret = sid_set_factory_reset(g_app_context->sidewalk_handle);
    if (ret != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("Notification of factory reset to sid api failed!");
        g_app_context->hw_ifc.platform_reset();
    } else {
        SID_PAL_LOG_INFO("Wait for Sid api to notify to proceed with factory reset!");
    }
}

void demo_app_button_event_handler(uint8_t idx)
{
    switch (idx) {
        case 0:
            queue_event(g_app_context->event_queue, EVENT_FACTORY_RESET, NULL, true);
            break;
        case 1:
            queue_event(g_app_context->event_queue, EVENT_CONNECT_LINK_TYPE_1, NULL, true);
            break;
        case 2:
            queue_event(g_app_context->event_queue, EVENT_SEND_MESSAGE, NULL, true);
            break;
        case 3: {
            struct sbdt_cancel_event *data = calloc(1, sizeof(*data));
            if (data) {
                data->file_id = 0;
                data->reason = SID_BULK_DATA_TRANSFER_REJECT_REASON_NONE;
            }
            queue_event(g_app_context->event_queue, EVENT_CANCEL_TRANSFER, data, true);
        } break;
        default:
            SID_PAL_LOG_INFO("Invalid button %u", idx);
    }
}

static void cap_timer_cb(TimerHandle_t xTimer)
{
    queue_event(g_app_context->event_queue, EVENT_NOTIFY_OTA_CAPABILITY, NULL, true);
}

static void connect_ble_cb(TimerHandle_t xTimer)
{
    queue_event(g_app_context->event_queue, EVENT_CONNECT_LINK_TYPE_1, NULL, true);
}

static void on_sidewalk_event(bool in_isr, void *context)
{
    queue_event(g_app_context->event_queue, EVENT_TYPE_SIDEWALK, NULL, in_isr);
}

static void on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context)
{
    SID_PAL_LOG_INFO("sent message(type: %d, id: %u)", (int)msg_desc->type, msg_desc->id);
}

static void on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc, void *context)
{
    SID_PAL_LOG_ERROR("failed to send message(type: %d, id: %u), err:%d", (int)msg_desc->type, msg_desc->id,
                      (int)error);
}

static void on_sidewalk_status_changed(const struct sid_status *status, void *context)
{
    SID_PAL_LOG_INFO("status changed: %d", (int)status->state);
    switch (status->state) {
        case SID_STATE_READY:
            g_app_context->sidewalk_state = STATE_SIDEWALK_READY;
            break;
        case SID_STATE_NOT_READY:
            g_app_context->sidewalk_state = STATE_SIDEWALK_NOT_READY;
            break;
        case SID_STATE_ERROR:
            SID_PAL_LOG_INFO("sidewalk error: %d", (int)sid_get_error(g_app_context->sidewalk_handle));
            SID_PAL_ASSERT(false);
            break;
        case SID_STATE_SECURE_CHANNEL_READY:
            g_app_context->sidewalk_state = STATE_SIDEWALK_SECURE_CONNECTION;
            break;
    }
    SID_PAL_LOG_INFO("Registration Status = %d, Time Sync Status = %d and Link Status Mask = %x",
                     status->detail.registration_status, status->detail.time_sync_status,
                     status->detail.link_status_mask);

    g_app_context->link_status.link_mask = status->detail.link_status_mask;
    g_app_context->link_status.time_sync_status = status->detail.time_sync_status;

    for (int i = 0; i < SID_LINK_TYPE_MAX_IDX; i++) {
        g_app_context->link_status.supported_link_mode[i] = status->detail.supported_link_modes[i];
        SID_PAL_LOG_INFO("Link %d Mode %x", i, status->detail.supported_link_modes[i]);
    }

    if (g_app_context->sidewalk_state != STATE_SIDEWALK_READY) {
        return;
    }

    if (g_app_context->pending_ota_status) {
        queue_event(g_app_context->event_queue, EVENT_NOTIFY_OTA_STATUS, NULL, false);
    }
}

static void on_sidewalk_factory_reset(void *context)
{
    SID_PAL_LOG_ERROR("factory reset notification received from sid api");
    g_app_context->hw_ifc.platform_reset();
}

static void set_cap_done(void)
{
    g_app_context->cap_done = true;

    BaseType_t ret =
        xTimerIsTimerActive(g_app_context->cap_timer_handle) ? xTimerStop(g_app_context->cap_timer_handle, 0) : pdTRUE;
    SID_PAL_ASSERT(ret == pdPASS);
}

static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context)
{
    static uint8_t temp_msg_payload[64];
    memset(temp_msg_payload, 0, sizeof(temp_msg_payload));
    struct sid_demo_msg demo_msg = {.payload = temp_msg_payload};

    struct sid_parse_state state;
    struct sid_demo_msg_desc demo_msg_desc;
    sid_parse_state_init(&state, msg->data, msg->size);
    sid_demo_app_msg_deserialize(&state, &demo_msg_desc, &demo_msg);
    SID_PAL_LOG_INFO("opc %d, class %d cmd %d status indicator %d status_code %d paylaod size %d", demo_msg_desc.opc,
                     demo_msg_desc.cmd_class, demo_msg_desc.cmd_id, demo_msg_desc.status_hdr_ind,
                     demo_msg_desc.status_code, demo_msg.payload_size);
    log_sid_msg(msg);
    if (state.ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("de-serialize demo app msg failed %d", state.ret_code);
    } else if (demo_msg_desc.status_hdr_ind && demo_msg_desc.opc == SID_DEMO_MSG_TYPE_RESP
               && demo_msg_desc.cmd_class == SID_DEMO_APP_CLASS
               && demo_msg_desc.cmd_id == SID_DEMO_APP_CLASS_CMD_CAP_DISCOVERY_ID
               && demo_msg_desc.status_code == SID_ERROR_NONE && demo_msg.payload_size == 0) {
        SID_PAL_LOG_INFO("Capability response received");
        set_cap_done();
    }
}

struct sid_event_callbacks *demo_app_get_sid_event_callbacks()
{
    static struct sid_event_callbacks sid_callbacks = {
        .on_event = on_sidewalk_event,                   /* Called from ISR context */
        .on_msg_received = on_sidewalk_msg_received,     /* Called from sid_process() */
        .on_msg_sent = on_sidewalk_msg_sent,             /* Called from sid_process() */
        .on_send_error = on_sidewalk_send_error,         /* Called from sid_process() */
        .on_status_changed = on_sidewalk_status_changed, /* Called from sid_process() */
        .on_factory_reset = on_sidewalk_factory_reset,   /* Called from sid_process */
    };
    sid_callbacks.context = g_app_context;
    return &sid_callbacks;
}

static uint8_t get_free_instance()
{
    uint8_t iter;
    for (iter = 0; iter < SBDT_CONCURRENT_SESSIONS; iter++) {
        if (!g_app_context->sbdt[iter].is_consumed) {
            g_app_context->sbdt[iter].is_consumed = true;
            break;
        }
    }
    return iter;
}

static uint8_t get_instace_by_file_id(uint32_t file_id)
{
    uint8_t iter;
    for (iter = 0; iter < SBDT_CONCURRENT_SESSIONS; iter++) {
        if (g_app_context->sbdt[iter].is_consumed && g_app_context->sbdt[iter].file_id == file_id) {
            break;
        }
    }
    return iter;
}

static void on_sbdt_transfer_request(const struct sid_bulk_data_transfer_request *const transfer_request,
                                     struct sid_bulk_data_transfer_response *const transfer_response,
                                     void *context)
{
    if (!transfer_response || !transfer_request) {
        return;
    }
    SID_PAL_LOG_INFO("%s ID %x req_buffer_size %u", __func__, transfer_request->file_id,
                     transfer_request->minimum_scratch_buffer_size);

    uint32_t free_space = g_app_context->hw_ifc.platform_flash_get_free_space();
    if (transfer_request->file_size > free_space) {
        SID_PAL_LOG_ERROR("file too big %d", transfer_request->file_size);
        transfer_response->reject_reason = SID_BULK_DATA_TRANSFER_REJECT_REASON_FILE_TOO_BIG;
        goto err;
    }

    if (transfer_request->minimum_scratch_buffer_size > SBDT_BUFFER_SIZE) {
        goto err;
    }

    uint8_t iter = get_instace_by_file_id(transfer_request->file_id);
    if (iter == SBDT_CONCURRENT_SESSIONS) {
        iter = get_free_instance();
        if (iter == SBDT_CONCURRENT_SESSIONS) {
            goto err;
        }
        g_app_context->sbdt[iter].file_id = transfer_request->file_id;
        g_app_context->sbdt[iter].file_size = transfer_request->file_size;
    }

    uint32_t crc = 0;
    sid_error_t result =
        sid_pal_storage_kv_record_get(FILE_TRANSFER_OTA_GROUP, FILE_TRANSFER_CRC_KEY, &crc, sizeof(uint32_t));
    if (result == SID_ERROR_NOT_FOUND) {
        SID_PAL_LOG_INFO("CRC NOT FOUND IN FLASH!");
    } else if (result != SID_ERROR_NONE) {
        SID_PAL_LOG_INFO("CRC COULD NOT BE LOADED");
    }
    if (result == SID_ERROR_NONE) {
        g_app_context->sbdt[iter].crc = crc;
    }

    transfer_response->status = SID_BULK_DATA_TRANSFER_ACTION_ACCEPT;
    transfer_response->scratch_buffer = g_app_context->sbdt[iter].scratch_buffer;
    transfer_response->scratch_buffer_size = sizeof(g_app_context->sbdt[iter].scratch_buffer);

    SID_PAL_LOG_INFO("PAUSE CAP_DISCOVERY!");
    set_cap_done();

    return;
err:
    transfer_response->status = SID_BULK_DATA_TRANSFER_ACTION_REJECT;
    transfer_response->scratch_buffer = NULL;
    transfer_response->scratch_buffer_size = 0;
}

static void on_sbdt_data_received(const struct sid_bulk_data_transfer_desc *const desc,
                                  const struct sid_bulk_data_transfer_buffer *const buffer,
                                  void *context)
{
    if (desc->file_offset == 0) {
        sid_pal_storage_kv_group_delete(FILE_TRANSFER_OTA_GROUP);
    }

    SID_PAL_LOG_INFO("%s file_id %x file_offset %x, link type %x", __func__, desc->file_id, desc->file_offset,
                     desc->link_type);
    uint8_t iter = get_instace_by_file_id(desc->file_id);
    if (iter == SBDT_CONCURRENT_SESSIONS) {
        goto err;
    }
    SID_PAL_LOG_INFO("size %x", buffer->size);

    uint8_t *tmp = buffer->data;
    for (size_t i = 0; i < buffer->size; i += 217) {
        if (i + 217 > buffer->size) {
            SID_PAL_LOG_INFO("LB %u F %x L %x", i, *(tmp + i), *(tmp + (buffer->size - 1)));
        } else {
            SID_PAL_LOG_INFO("B %u F %x L %x", i, *(tmp + i), *(tmp + (i + 217 - 1)));
        }
        SID_PAL_LOG_FLUSH();
    }
    SID_PAL_LOG_FLUSH();

    uint32_t crc = 0;
    sid_error_t result =
        sid_pal_storage_kv_record_get(FILE_TRANSFER_OTA_GROUP, FILE_TRANSFER_CRC_KEY, &crc, sizeof(uint32_t));
    if (result == SID_ERROR_NOT_FOUND) {
        SID_PAL_LOG_INFO("CRC NOT FOUND IN FLASH!");
    } else if (result != SID_ERROR_NONE) {
        SID_PAL_LOG_INFO("CRC COULD NOT BE LOADED");
    }
    g_app_context->sbdt[iter].crc = crc;

    SID_PAL_LOG_INFO("Prev CRC: 0x%x", g_app_context->sbdt[iter].crc);
    g_app_context->sbdt[iter].crc = crc32_compute(buffer->data, buffer->size, &g_app_context->sbdt[iter].crc);

    if (g_app_context->sbdt[iter].file_size == (desc->file_offset + buffer->size)) {
        SID_PAL_LOG_INFO("Entire File Received: [%x:%u:0%x]", desc->file_id, g_app_context->sbdt[iter].file_size,
                         g_app_context->sbdt[iter].crc);
    } else {
        SID_PAL_LOG_INFO("File Updated Crc: 0x%x", g_app_context->sbdt[iter].crc);
    }
    SID_PAL_LOG_FLUSH();

    SID_PAL_LOG_INFO("PAUSE CAP_DISCOVERY!");
    set_cap_done();

    result = sid_pal_storage_kv_record_set(FILE_TRANSFER_OTA_GROUP, FILE_TRANSFER_CRC_KEY,
                                           &g_app_context->sbdt[iter].crc, sizeof(g_app_context->sbdt[iter].crc));
    if (result != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("COULD NOT STORE CRC: %x", g_app_context->sbdt[iter].crc);
    }
    struct sbdt_buffer_release *evt_data = (struct sbdt_buffer_release *)malloc(sizeof(*evt_data));
    if (evt_data == NULL) {
        SID_PAL_LOG_ERROR("Failed to allocate event buffer !");
        goto err;
    }
    *evt_data = (struct sbdt_buffer_release){
        .file_id = desc->file_id,
        .file_offset = desc->file_offset,
        .data = buffer->data,
        .size = buffer->size,
    };
    queue_event(g_app_context->event_queue, EVENT_BLOCK_RESPONSE, evt_data, false);

    struct sbdt_ota_progress_event *progress_data = (struct sbdt_ota_progress_event *)malloc(sizeof(*progress_data));
    if (progress_data == NULL) {
        SID_PAL_LOG_ERROR("Failed to allocated progress buffer!");
        goto err;
    }
    progress_data->file_id = desc->file_id;
    progress_data->file_completed = desc->file_offset + buffer->size;
    progress_data->file_size = g_app_context->sbdt[iter].file_size;
    queue_event(g_app_context->event_queue, EVENT_NOTIFY_OTA_PROGRESS, progress_data, false);
err:
    return;
}

static void on_sbdt_finalize_request(uint32_t file_id, void *context)
{
    SID_PAL_LOG_INFO("%s", __func__);

    uint8_t iter = get_instace_by_file_id(file_id);
    if (iter == SBDT_CONCURRENT_SESSIONS) {
        SID_PAL_LOG_ERROR("Invalid file id %d", file_id);
        return;
    }

    if (g_app_context->sbdt[iter].crc == 0) {
        SID_PAL_LOG_INFO("CRC COULD NOT BE LOADED");
    }

    bool flash_check =
        g_app_context->hw_ifc.platform_flash_check(g_app_context->sbdt[iter].file_size, g_app_context->sbdt[iter].crc);

    sid_error_t result = sid_bulk_data_transfer_finalize(g_app_context->sidewalk_handle, 0,
                                                         flash_check ? SID_BULK_DATA_TRANSFER_FINAL_STATUS_SUCCESS
                                                                     : SID_BULK_DATA_TRANSFER_FINAL_STATUS_FAILURE);
    SID_PAL_LOG_INFO("FILE_TRANSFER_FINALIZE: %d", result);

    uint32_t status = flash_check ? OTA_STATUS_PLATFORM_VALIDATION_SUCCESS : OTA_STATUS_PLATFORM_VALIDATION_FAILED;

    result = sid_pal_storage_kv_record_set(FILE_TRANSFER_OTA_GROUP, FILE_OTA_PENDING_KEY, &status, sizeof(status));
    if (result != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("Setting OTA_PEND FAILED: %d", result);
        SID_PAL_LOG_FLUSH();
    }

    uint8_t build_version[sizeof(struct sid_demo_sdk_version)] = {};
    struct sid_parse_state state;
    sid_parse_state_init(&state, build_version, sizeof(build_version));
    sid_demo_sdk_version_serialize(&state, &current_sdk_version);
    SID_PAL_ASSERT(state.ret_code == SID_ERROR_NONE);
    result =
        sid_pal_storage_kv_record_set(FILE_TRANSFER_OTA_GROUP, FILE_OTA_CHECK_VERSION, &build_version, state.offset);
    if (result != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("Saving VERSION FAILED: %d", result);
        SID_PAL_LOG_FLUSH();
    }

    g_app_context->finalize_on_release = true;
}

static void on_sbdt_release_scratch_buffer(uint32_t file_id, void *context)
{
    SID_PAL_LOG_INFO("%s", __func__);

    uint8_t iter = get_instace_by_file_id(file_id);
    if (iter == SBDT_CONCURRENT_SESSIONS) {
        SID_PAL_LOG_ERROR("Invalid file id %d", file_id);
        return;
    }

    SID_PAL_LOG_INFO("Finalize on release: %d", g_app_context->finalize_on_release);
    if (g_app_context->finalize_on_release) {
        bool finalize_result = g_app_context->hw_ifc.platform_flash_finalize(g_app_context->sbdt[iter].file_size,
                                                                             g_app_context->sbdt[iter].crc);
        SID_PAL_LOG_INFO("Transfer finalize status: %d", finalize_result);
        SID_PAL_LOG_FLUSH();
    }

    memset(&g_app_context->sbdt[iter], 0, sizeof(g_app_context->sbdt[iter]));
    g_app_context->sbdt[iter].is_consumed = false;
}

static void on_sbdt_error(uint32_t file_id, void *context)
{
    SID_PAL_LOG_INFO("%s", __func__);
}

static void on_sbdt_cancel_request(uint32_t file_id, void *context)
{
    SID_PAL_LOG_INFO("%s", __func__);
}

struct sid_bulk_data_transfer_event_callbacks *demo_sbdt_app_get_sid_event_callbacks()
{
    static struct sid_bulk_data_transfer_event_callbacks sbdt_event_callbacks = {
        .on_transfer_request = on_sbdt_transfer_request,
        .on_data_received = on_sbdt_data_received,
        .on_finalize_request = on_sbdt_finalize_request,
        .on_cancel_request = on_sbdt_cancel_request,
        .on_error = on_sbdt_error,
        .on_release_scratch_buffer = on_sbdt_release_scratch_buffer,
    };
    sbdt_event_callbacks.context = g_app_context;
    return &sbdt_event_callbacks;
}

struct demo_event_handler {
    enum event_type type;
    bool needs_data;
    void (*process_event)(const struct app_demo_event *event);
};

static void process_event_type_sidewalk(const struct app_demo_event *event)
{
    sid_process(g_app_context->sidewalk_handle);
}

static void process_event_type_factory_reset(const struct app_demo_event *event)
{
    factory_reset();
}

static void process_event_connect_request_type_1(const struct app_demo_event *event)
{
    SID_PAL_LOG_INFO("Connecting link type 1");
    sid_error_t ret = sid_ble_bcn_connection_request(g_app_context->sidewalk_handle, true);
    if (ret != SID_ERROR_ALREADY_EXISTS && ret) {
        SID_PAL_LOG_ERROR("Failed to set connect request on link type 1 %d", ret);
    }

    BaseType_t timer_ret = g_app_context->cap_done && xTimerIsTimerActive(g_app_context->connect_ble_timer_handle)
                               ? xTimerStop(g_app_context->connect_ble_timer_handle, 0)
                               : pdTRUE;
    SID_PAL_ASSERT(timer_ret == pdPASS);
}

static void process_event_send_message(const struct app_demo_event *event)
{
    g_app_context->counter++;
    struct sid_msg msg = {
        .data = (uint8_t *)&g_app_context->counter,
        .size = sizeof(uint8_t),
    };
    struct sid_msg_desc desc = {
        .type = SID_MSG_TYPE_NOTIFY,
        .link_type = SID_LINK_TYPE_ANY,
        .link_mode = SID_LINK_MODE_CLOUD,
    };
    send_msg(&desc, &msg);
}

static void process_event_block_response(const struct app_demo_event *event)
{
    struct sbdt_buffer_release *data = (struct sbdt_buffer_release *)event->data;
    uint8_t iter = get_instace_by_file_id(data->file_id);
    if (!g_app_context->hw_ifc.platform_flash_write(data->file_offset, data->data, data->size)) {
        SID_PAL_LOG_ERROR("Failed to write buffer to flash");
    }
    struct sid_bulk_data_transfer_buffer buffer = {
        .data = data->data,
        .size = data->size,
    };
    if ((sid_bulk_data_transfer_release_buffer(g_app_context->sidewalk_handle, data->file_id, &buffer))
        != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("Failed to release buffer");
    }
}

static void process_event_cancel_transfer(const struct app_demo_event *event)
{
    struct sbdt_cancel_event *data = (struct sbdt_cancel_event *)event->data;
    sid_error_t result = sid_bulk_data_transfer_cancel(g_app_context->sidewalk_handle, data->file_id, data->reason);
    if (result != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("Failed to Cancel: %d", result);
    }
}

static sid_error_t send_sbdt_demo_msg(struct sid_demo_msg_desc *msg_desc, struct sid_demo_msg *demo_msg)
{
    static uint8_t msg_buffer[64];
    struct sid_parse_state state = {};
    sid_parse_state_init(&state, msg_buffer, sizeof(msg_buffer));
    sid_demo_app_msg_serialize(&state, msg_desc, demo_msg);
    SID_PAL_ASSERT(state.ret_code == SID_ERROR_NONE);

    struct sid_status status = {};
    sid_get_status(g_app_context->sidewalk_handle, &status);

    struct sid_msg msg = {
        .data = state.buffer,
        .size = state.offset,
    };
    struct sid_msg_desc desc = {
        .link_type = SID_LINK_TYPE_1,
        .type = SID_MSG_TYPE_NOTIFY,
        .link_mode = SID_LINK_MODE_CLOUD | SID_LINK_MODE_MOBILE,
    };
    SID_PAL_LOG_FLUSH();
    SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, state.buffer, state.offset);
    SID_PAL_LOG_FLUSH();
    return sid_put_msg(g_app_context->sidewalk_handle, &msg, &desc);
}

static void process_event_send_ota_progress(const struct app_demo_event *event)
{
    struct sbdt_ota_progress_event *data = (struct sbdt_ota_progress_event *)event->data;

    struct sid_demo_action_notification notify = {
        .file_id = data->file_id,
        .link_type = SID_LINK_TYPE_1,
        .ota_status = SID_DEMO_OTA_COMPLETION_STATUS_INITIAL,
        .ota_stats =
            {
                .is_valid = true,
                .percent = (data->file_completed * 100) / data->file_size,
                .completed_file_size = data->file_completed,
                .total_file_size = data->file_size,
            },
    };

    SID_PAL_LOG_INFO("Notify: [%d:%d:%d]", notify.ota_stats.percent, notify.ota_stats.completed_file_size,
                     notify.ota_stats.total_file_size);
    SID_PAL_LOG_FLUSH();

    uint8_t temp_buffer[sizeof(notify)] = {0};
    struct sid_parse_state state = {};
    sid_parse_state_init(&state, temp_buffer, sizeof(temp_buffer));
    sid_demo_app_action_notification_serialize(&state, &notify);
    if (state.ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("OTA STATS SERIALIZE FAILED :%d", state.ret_code);
        return;
    }

    struct sid_demo_msg_desc msg_desc = {
        .status_hdr_ind = false,
        .opc = SID_DEMO_MSG_TYPE_NOTIFY,
        .cmd_class = SID_DEMO_APP_CLASS,
        .cmd_id = SID_DEMO_APP_CLASS_CMD_ACTION,
    };
    struct sid_demo_msg demo_msg = {
        .payload = temp_buffer,
        .payload_size = state.offset,
    };
    sid_error_t ret = send_sbdt_demo_msg(&msg_desc, &demo_msg);
    SID_PAL_LOG_INFO("Sending stats: %d, progress: %d", ret, notify.ota_stats.percent);
}

static void process_event_send_ota_capability(const struct app_demo_event *event)
{
    struct sid_status status = {};
    sid_error_t result = sid_get_status(g_app_context->sidewalk_handle, &status);
    if (result != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("GET_STATUS FAILED: %d", result);
        return;
    }

    if (!(status.detail.link_status_mask & SID_LINK_TYPE_1)) {
        SID_PAL_LOG_INFO("WAITING_FOR_CONNECTION: %d", result);
        return;
    }

    struct sid_demo_capability_discovery discovery = {
        .ota_support = true,
        .major = SID_SDK_MAJOR_VERSION,
        .minor = SID_SDK_MINOR_VERSION,
        .patch = SID_SDK_PATCH_VERSION,
        .build = SID_SDK_BUILD_VERSION,
        .link_type = SID_LINK_TYPE_1,
    };

    uint8_t temp_buffer[sizeof(discovery)] = {0};
    struct sid_parse_state state = {};
    sid_parse_state_init(&state, temp_buffer, sizeof(temp_buffer));
    sid_demo_app_capability_discovery_notification_serialize(&state, &discovery);
    if (state.ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("OTA SUPPORT NOTIFY SERIALIZE FAILED :%d", state.ret_code);
        return;
    }

    struct sid_demo_msg_desc msg_desc = {
        .status_hdr_ind = false,
        .opc = SID_DEMO_MSG_TYPE_NOTIFY,
        .cmd_class = SID_DEMO_APP_CLASS,
        .cmd_id = SID_DEMO_APP_CLASS_CMD_CAP_DISCOVERY_ID,
    };

    struct sid_demo_msg demo_msg = {
        .payload = temp_buffer,
        .payload_size = state.offset,
    };
    sid_error_t ret = send_sbdt_demo_msg(&msg_desc, &demo_msg);
    SID_PAL_LOG_INFO("SEND OTA CAPABILITY: %d", ret);
}

static void process_event_send_ota_status(uint32_t file_id, uint32_t status)
{
    struct sid_demo_action_notification notify = {
        .file_id = 0,
        .link_type = SID_LINK_TYPE_1,
        .ota_status = status,
        .ota_stats =
            {
                .is_valid = false,
            },
    };

    uint8_t temp_buffer[sizeof(notify)] = {0};
    struct sid_parse_state state = {};
    sid_parse_state_init(&state, temp_buffer, sizeof(temp_buffer));
    sid_demo_app_action_notification_serialize(&state, &notify);
    if (state.ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("OTA STATUS SERIALIZE FAILED :%d", state.ret_code);
        return;
    }

    struct sid_demo_msg_desc msg_desc = {
        .status_hdr_ind = false,
        .opc = SID_DEMO_MSG_TYPE_NOTIFY,
        .cmd_class = SID_DEMO_APP_CLASS,
        .cmd_id = SID_DEMO_APP_CLASS_CMD_ACTION,
    };
    struct sid_demo_msg demo_msg = {
        .payload = temp_buffer,
        .payload_size = state.offset,
    };
    sid_error_t ret = send_sbdt_demo_msg(&msg_desc, &demo_msg);
    SID_PAL_LOG_INFO("Sending ota Status: %d, status: %d", ret, status);
}

static void process_event_ota_status(const struct app_demo_event *event)
{
#ifndef STR
#define STR(a_) STR_(a_)
#define STR_(a_) #a_
#endif

#define SID_SDK_VERSION_STR    \
    STR(SID_SDK_MAJOR_VERSION) \
    "." STR(SID_SDK_MINOR_VERSION) "." STR(SID_SDK_PATCH_VERSION) "-" STR(SID_SDK_BUILD_VERSION)

    uint32_t ota = 0;
    sid_pal_storage_kv_record_get(FILE_TRANSFER_OTA_GROUP, FILE_OTA_PENDING_KEY, &ota, sizeof(ota));
    if (!ota) {
        goto start_capability;
    }

    uint8_t build_version[sizeof(struct sid_demo_sdk_version)] = {};
    sid_pal_storage_kv_record_get(FILE_TRANSFER_OTA_GROUP, FILE_OTA_CHECK_VERSION, build_version,
                                  sizeof(build_version));
    struct sid_parse_state state;
    sid_parse_state_init(&state, build_version, sizeof(build_version));

    struct sid_demo_sdk_version sdk_version;
    sid_demo_sdk_version_deserialize(&state, &sdk_version);
    SID_PAL_ASSERT(state.ret_code == SID_ERROR_NONE);

    const bool success = sdk_version.major == SID_SDK_MAJOR_VERSION && sdk_version.minor == SID_SDK_MINOR_VERSION
                         && sdk_version.patch == SID_SDK_PATCH_VERSION && sdk_version.build == SID_SDK_BUILD_VERSION;
    if (success) {
        SID_PAL_LOG_INFO("READ_VERSION: %d.%d.%d-%d == " SID_SDK_VERSION_STR, sdk_version.major, sdk_version.minor,
                         sdk_version.patch, sdk_version.build);
    } else {
        SID_PAL_LOG_ERROR("READ_VERSION: %d.%d.%d-%d != " SID_SDK_VERSION_STR, sdk_version.major, sdk_version.minor,
                          sdk_version.patch, sdk_version.build);
    }

    const bool successful_ota = (ota == OTA_STATUS_PLATFORM_VALIDATION_SUCCESS && success);

    process_event_send_ota_status(0, successful_ota ? SID_DEMO_OTA_COMPLETION_STATUS_SUCCESS
                                                    : SID_DEMO_OTA_COMPLETION_STATUS_FAILED);

    sid_pal_storage_kv_group_delete(FILE_TRANSFER_OTA_GROUP);

start_capability:
    if (!g_app_context->cap_done && !xTimerIsTimerActive(g_app_context->cap_timer_handle)) {
        BaseType_t ret = xTimerChangePeriod(g_app_context->cap_timer_handle, pdMS_TO_TICKS(10000), 0);
        SID_PAL_ASSERT(ret == pdPASS);
    }

    g_app_context->pending_ota_status = false;
}

static void process_event_reboot(const struct app_demo_event *event)
{
    g_app_context->hw_ifc.platform_reset();
}

static const struct demo_event_handler event_handlers[] = {
    {.type = EVENT_TYPE_SIDEWALK, .needs_data = false, .process_event = process_event_type_sidewalk},
    {.type = EVENT_FACTORY_RESET, .needs_data = false, .process_event = process_event_type_factory_reset},
    {.type = EVENT_SEND_MESSAGE, .needs_data = false, .process_event = process_event_send_message},
    {.type = EVENT_BLOCK_RESPONSE, .needs_data = true, .process_event = process_event_block_response},
    {.type = EVENT_CANCEL_TRANSFER, .needs_data = true, .process_event = process_event_cancel_transfer},
    {.type = EVENT_NOTIFY_OTA_PROGRESS, .needs_data = true, .process_event = process_event_send_ota_progress},
    {.type = EVENT_NOTIFY_OTA_CAPABILITY, .needs_data = false, .process_event = process_event_send_ota_capability},
    {.type = EVENT_CONNECT_LINK_TYPE_1, .needs_data = false, .process_event = process_event_connect_request_type_1},
    {.type = EVENT_NOTIFY_OTA_STATUS, .needs_data = false, .process_event = process_event_ota_status},
    {.type = EVENT_REBOOT, .needs_data = false, .process_event = process_event_reboot},
};

void demo_app_main_task_event_handler(struct app_demo_event *event)
{
    if (!event) {
        return;
    }

    bool handled = false;

    for (size_t i = 0; i < sizeof(event_handlers) / sizeof(event_handlers[0]); i++) {
        if (event_handlers[i].type == event->type) {
            if (event_handlers[i].needs_data && !event->data) {
                SID_PAL_LOG_ERROR("Event data not set for event: %d", event->type);
            } else {
                event_handlers[i].process_event(event);
                handled = true;
            }
            break;
        }
    }

    if (event->data != NULL) {
        free(event->data);
    }

    if (!handled) {
        SID_PAL_LOG_ERROR("No Handler for %d", event->type);
    }
}

void demo_sbdt_app_init(app_context_t *app_context)
{
    SID_PAL_ASSERT(app_context);

    g_app_context = app_context;
    g_app_context->cap_done = false;
    g_app_context->finalize_on_release = false;
    g_app_context->pending_ota_status = true;
    g_app_context->sidewalk_state = STATE_SIDEWALK_NOT_READY;

    g_app_context->hw_ifc.platform_flash_init();

    g_app_context->event_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(struct app_demo_event));
    if (g_app_context->event_queue == NULL) {
        SID_PAL_ASSERT(false);
    }

    g_app_context->cap_timer_handle =
        xTimerCreate("demo_cap_timer", pdMS_TO_TICKS(10000), pdTRUE, (void *)0, cap_timer_cb);
    if (g_app_context->cap_timer_handle == NULL) {
        SID_PAL_LOG_ERROR("create capability timer failed!");
        SID_PAL_ASSERT(false);
    }

    g_app_context->connect_ble_timer_handle =
        xTimerCreate("ble_timer_handler", pdMS_TO_TICKS(10000), pdTRUE, (void *)0, connect_ble_cb);
    if (g_app_context->connect_ble_timer_handle == NULL) {
        SID_PAL_LOG_ERROR("create ble  connect timer failed!");
        SID_PAL_ASSERT(false);
    }
    BaseType_t ret = xTimerChangePeriod(g_app_context->connect_ble_timer_handle, pdMS_TO_TICKS(10000), 0);
    SID_PAL_ASSERT(ret == pdPASS);
}
