#include <sid_app_demo.h>

#include <sid_demo_parser.h>
#include <sid_demo_types.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_assert_ifc.h>
#include <sid_900_cfg.h>

#include <FreeRTOS.h>
#include <timers.h>
#include <queue.h>
#include <task.h>
#include <sid_api.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MAIN_TASK_STACK_SIZE        (2048 / sizeof(configSTACK_DEPTH_TYPE))
#define RECEIVE_TASK_STACK_SIZE     (2048 / sizeof(configSTACK_DEPTH_TYPE))
#define MSG_QUEUE_LEN 10

#define PAYLOAD_MAX_SIZE 255
#define DEMO_CAPABILITY_PERIOD_MS 5000
#define BUTTON_PRESS_CHECK_PERIOD_SECS 30
#define DEMO_NOTIFY_SENSOR_DATA_PERIOD_MS 15000
#define CONNECT_LINK_TYPE_1_DELAY_MS        60000
#define CONNECT_LINK_TYPE_1_INIT_DELAY_MS   5000
#define PROFILE_CHECK_TIMER_DELAY_MS        60000

#define LED_ACTION_REPONSE_PAYLOAD_SIZE_MAX 32
#define SENSOR_NOTIFY_PAYLOAD_SIZE_MAX 32
#define CAPABILITY_NOTIFY_PAYLOAD_SIZE_MAX 32

#define SID_DEMO_APP_TTL_MAX 60
#define SID_DEMO_APP_RETRIES_MAX 3

static app_context_t *g_app_context;
static receive_context_t *g_receive_context;

static void queue_event(QueueHandle_t queue, enum event_type event, void *data, bool in_isr)
{
    struct app_demo_event evt = {.type = event, .data = data};
    if (in_isr) {
        BaseType_t task_woken = pdFALSE;
        xQueueSendFromISR(queue, &evt, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    }
    else {
        xQueueSend(queue, &evt, 0);
    }
}

static void queue_rx_msg(QueueHandle_t queue, struct app_demo_rx_msg *rx_msg, bool in_isr)
{
    if (in_isr) {
        BaseType_t task_woken = pdFALSE;
        xQueueSendToBackFromISR(queue, rx_msg, &task_woken);
        portYIELD_FROM_ISR(task_woken);
    }
    else {
        xQueueSendToBack(queue, rx_msg, 0);
    }
}

static void log_sid_msg(const struct sid_msg *msg)
{
    char *data = (char*)msg->data;
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
    if (g_app_context->sidewalk_state == STATE_SIDEWALK_READY ||
        g_app_context->sidewalk_state == STATE_SIDEWALK_SECURE_CONNECTION) {
        sid_error_t ret = sid_put_msg(g_app_context->sidewalk_handle, msg, desc);
        if (ret != SID_ERROR_NONE) {
            SID_PAL_LOG_ERROR("failed queueing data, err:%d", (int) ret);
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

static void turn_on_all_leds(app_context_t *app_context)
{
    uint8_t idx = 0;
    for(idx = 0; idx < app_context->led_num; idx++) {
        app_context->hw_ifc.turn_on_leds(app_context->led_id_arr[idx]);
    }
}

static void turn_off_all_leds(app_context_t *app_context)
{
    uint8_t idx = 0;
    for(idx = 0; idx < app_context->led_num; idx++) {
        app_context->hw_ifc.turn_off_leds(app_context->led_id_arr[idx]);
    }
}

static void demo_app_notify_capability(void)
{
    struct sid_parse_state state = {};
    enum sid_link_type link_type = SID_LINK_TYPE_1;
    struct sid_demo_capability_discovery cap = {
        .num_buttons = g_app_context->button_num,
        .button_id_arr = g_app_context->button_id_arr,
        .num_leds = g_app_context->led_num,
        .led_id_arr = g_app_context->led_id_arr,
        .temp_sensor = SID_DEMO_TEMPERATURE_SENSOR_UNITS_CELSIUS,
    };

    uint8_t temp_buffer[CAPABILITY_NOTIFY_PAYLOAD_SIZE_MAX] = {0};
    get_active_link_type(&link_type);
    cap.link_type = link_type;

    sid_parse_state_init(&state, temp_buffer, sizeof(temp_buffer));
    sid_demo_app_capability_discovery_notification_serialize(&state, &cap);
    if (state.ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("capability discovery serialize failed -%d", state.ret_code);
        return;
    }

    struct sid_demo_msg_desc msg_desc = {
        .status_hdr_ind = false,
        .opc = SID_DEMO_MSG_TYPE_NOTIFY,
        .cmd_class = SID_DEMO_APP_CLASS,
        .cmd_id = SID_DEMO_APP_CLASS_CMD_CAP_DISCOVERY_ID,
    };

    struct sid_demo_msg demo_msg = {.payload = temp_buffer, .payload_size = state.offset};

    sid_parse_state_init(&state, g_app_context->buffer, sizeof(g_app_context->buffer));
    sid_demo_app_msg_serialize(&state, &msg_desc, &demo_msg);

    if (state.ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("demo msg capability discovery serialize failed -%d", state.ret_code);
        return;
    }

    struct sid_msg msg = {
        .data = g_app_context->buffer,
        .size = state.offset,
    };
    struct sid_msg_desc desc = {
        .link_type = link_type,
        .type = SID_MSG_TYPE_NOTIFY,
        .link_mode = SID_LINK_MODE_CLOUD,
    };
    SID_PAL_LOG_INFO("Sending demo app message notify capability");
    send_msg(&desc, &msg);
}

static void demo_app_notify_sensor_data(bool button_pressed)
{
    struct sid_parse_state state = {0};
    enum sid_link_type link_type = SID_LINK_TYPE_1;
    struct sid_demo_action_notification action_notify = {0};
    struct sid_timespec curr_time = {0};
    uint8_t temp_buffer[SENSOR_NOTIFY_PAYLOAD_SIZE_MAX] = {0};

    action_notify.button_action_notify.action_resp = (button_pressed) ? SID_DEMO_ACTION_BUTTON_PRESSED :
                                                     SID_DEMO_ACTION_BUTTON_NOT_PRESSED;
    sid_get_time(g_app_context->sidewalk_handle, SID_GET_GPS_TIME, &curr_time);
    if (!button_pressed) {
        action_notify.temp_sensor = SID_DEMO_TEMPERATURE_SENSOR_UNITS_CELSIUS;
        action_notify.temperature = g_app_context->hw_ifc.temp_read();
        action_notify.button_action_notify.action_resp = SID_DEMO_ACTION_BUTTON_NOT_PRESSED;
    } else {
        action_notify.temp_sensor = SID_DEMO_TEMPERATURE_SENSOR_NOT_SUPPORTED;
        action_notify.temperature = 0;
        action_notify.button_action_notify.action_resp = SID_DEMO_ACTION_BUTTON_PRESSED;
        uint8_t temp_button_arr[APP_DEMO_BUTTONS_MAX] = {0};
        uint8_t num_buttons_pressed = 0;
        for (size_t i = 0; i < g_app_context->button_num; i++) {
            if (((1 << g_app_context->button_id_arr[i]) & (g_app_context->button_press_mask)) &&
                 (!((1 << g_app_context->button_id_arr[i]) & (g_app_context->button_notify_mask)))) {
                temp_button_arr[num_buttons_pressed] = g_app_context->button_id_arr[i];
                num_buttons_pressed += 1;
                g_app_context->button_notify_mask |= (1 << g_app_context->button_id_arr[i]);
                g_app_context->button_press_time_in_sec_id_arr[i] = curr_time.tv_sec;
                BaseType_t ret = xTimerIsTimerActive(g_app_context->button_press_timer_handle);
                if (ret == pdFALSE) {
                    BaseType_t ret = xTimerChangePeriod(g_app_context->button_press_timer_handle,
                        pdMS_TO_TICKS(BUTTON_PRESS_CHECK_PERIOD_SECS * 1000), 0);
                    SID_PAL_ASSERT(ret == pdPASS);
                }
            }
        }
        action_notify.button_action_notify.button_id_arr = temp_button_arr;
        action_notify.button_action_notify.num_buttons = num_buttons_pressed;
    }

    action_notify.gps_time_in_seconds = curr_time.tv_sec;
    get_active_link_type(&link_type);
    action_notify.link_type = link_type;

    sid_parse_state_init(&state, temp_buffer, sizeof(temp_buffer));
    sid_demo_app_action_notification_serialize(&state, &action_notify);
    if (state.ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("capability discovery serialize failed -%d", state.ret_code);
        return;
    }

    struct sid_demo_msg_desc msg_desc = {
        .status_hdr_ind = false,
        .opc = SID_DEMO_MSG_TYPE_NOTIFY,
        .cmd_class = SID_DEMO_APP_CLASS,
        .cmd_id = SID_DEMO_APP_CLASS_CMD_ACTION,
    };

    struct sid_demo_msg demo_msg = {.payload = temp_buffer, .payload_size = state.offset};

    sid_parse_state_init(&state, g_app_context->buffer, sizeof(g_app_context->buffer));
    sid_demo_app_msg_serialize(&state, &msg_desc, &demo_msg);

    if (state.ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("demo msg notify sensor data serialize failed -%d", state.ret_code);
        return;
    }

    struct sid_msg msg = {
        .data = g_app_context->buffer,
        .size = state.offset,
    };
    struct sid_msg_desc desc = {
        .link_type = link_type,
        .type = SID_MSG_TYPE_NOTIFY,
        .link_mode = SID_LINK_MODE_CLOUD,
    };

    if (button_pressed) {
        desc.msg_desc_attr.tx_attr.ttl_in_seconds = SID_DEMO_APP_TTL_MAX;
        desc.msg_desc_attr.tx_attr.num_retries = SID_DEMO_APP_RETRIES_MAX;
        desc.msg_desc_attr.tx_attr.request_ack = true;
        SID_PAL_LOG_INFO("Sending demo app message notify button press");
    } else {
        SID_PAL_LOG_INFO("Sending demo app message notify sensor data");
    }
    send_msg(&desc, &msg);
}

static void demo_app_create_led_response(struct sid_demo_action_resp *resp)
{
    struct sid_parse_state state = {0};
    enum sid_link_type link_type = SID_LINK_TYPE_1;
    uint8_t temp_buffer[LED_ACTION_REPONSE_PAYLOAD_SIZE_MAX] = {0};

    sid_parse_state_init(&state, temp_buffer, sizeof(temp_buffer));
    sid_demo_app_action_resp_serialize(&state, resp);

    struct sid_demo_msg_desc msg_desc = {
        .status_hdr_ind = true,
        .opc = SID_DEMO_MSG_TYPE_RESP,
        .cmd_class = SID_DEMO_APP_CLASS,
        .cmd_id = SID_DEMO_APP_CLASS_CMD_ACTION,
        .status_code = SID_ERROR_NONE,
    };

    struct sid_demo_msg demo_msg = {.payload = temp_buffer, .payload_size = state.offset};

    sid_parse_state_init(&state, g_app_context->buffer, sizeof(g_app_context->buffer));
    sid_demo_app_msg_serialize(&state, &msg_desc, &demo_msg);

    if (state.ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("demo msg notify sensor data serialize failed -%d", state.ret_code);
        return;
    }

    get_active_link_type(&link_type);
    struct sid_msg_desc desc = {
        .link_type = link_type,
        .type = SID_MSG_TYPE_NOTIFY,
        .link_mode = SID_LINK_MODE_CLOUD,
        .msg_desc_attr = {
            .tx_attr = {
                .ttl_in_seconds = SID_DEMO_APP_TTL_MAX,
                .num_retries = SID_DEMO_APP_RETRIES_MAX,
                .request_ack = true,
            }
        }
    };

    struct app_demo_tx_msg *data = (struct app_demo_tx_msg *)malloc(sizeof(struct app_demo_tx_msg));
    if (data == NULL) {
        SID_PAL_LOG_ERROR("Failed to allocate tx data memory !");
        return;
    } else {
        data->pld_size = state.offset;
        memcpy(data->tx_payload, g_app_context->buffer, data->pld_size);
        data->desc = desc;
    }

    SID_PAL_LOG_INFO("Led response created");
    queue_event(g_app_context->event_queue, EVENT_SEND_MESSAGE, data, false);
}

static void cap_timer_cb(TimerHandle_t xTimer)
{
    TickType_t delay = pdMS_TO_TICKS(DEMO_CAPABILITY_PERIOD_MS);
    enum event_type event = EVENT_NOTIFICATION_TIMER_FIRED;

    if (BUILD_SID_SDK_LINK_TYPE == 1) {
        if (g_app_context->sidewalk_state != STATE_SIDEWALK_READY) {
            delay = pdMS_TO_TICKS(CONNECT_LINK_TYPE_1_DELAY_MS);
            event = EVENT_CONNECT_LINK_TYPE_1;
        }
    }
    if (g_app_context->app_state == DEMO_APP_STATE_NOTIFY_SENSOR_DATA && g_app_context->sidewalk_state == STATE_SIDEWALK_READY) {
        delay = pdMS_TO_TICKS(DEMO_NOTIFY_SENSOR_DATA_PERIOD_MS);
    }
    BaseType_t ret = xTimerChangePeriod(g_app_context->cap_timer_handle, delay, 0);
    SID_PAL_ASSERT(ret == pdPASS);

    queue_event(g_app_context->event_queue, event, NULL, true);
}

static void button_press_timer_cb(TimerHandle_t xTimer)
{
    queue_event(g_app_context->event_queue, EVENT_BUTTON_PRESS_TIMER_FIRED, NULL, true);
}

static void device_profile_timer_cb(TimerHandle_t xTimer)
{
    queue_event(g_app_context->event_queue, EVENT_SET_DEVICE_PROFILE_LINK_TYPE_2, NULL, true);
}

static void process_action_response(struct sid_parse_state *state)
{
    uint8_t temp_button_id_arr[APP_DEMO_BUTTONS_MAX] = {0};
    struct sid_demo_action_resp action_resp = {0};

    action_resp.button_action_resp.button_id_arr = temp_button_id_arr;
    sid_demo_app_action_resp_deserialize(state, &action_resp);
    if (state->ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("de-serialize action resp failed %d", state->ret_code);
    } else if (action_resp.resp_type == SID_DEMO_ACTION_TYPE_BUTTON) {
        if (action_resp.button_action_resp.num_buttons == 0xFF) {
            g_app_context->button_press_mask = 0;
            g_app_context->button_notify_mask = 0;
            for (size_t i = 0; i < g_app_context->button_num; i++) {
                g_app_context->button_press_time_in_sec_id_arr[i] = 0;
            }
        } else if (action_resp.button_action_resp.num_buttons <= g_app_context->button_num) {
            for (size_t i = 0; i < action_resp.button_action_resp.num_buttons; i++) {
                g_app_context->button_press_mask &= ~(1 << action_resp.button_action_resp.button_id_arr[i]);
                g_app_context->button_notify_mask &= ~(1 << action_resp.button_action_resp.button_id_arr[i]);
                g_app_context->button_press_time_in_sec_id_arr[i] = 0;
            }
        } else {
            SID_PAL_LOG_ERROR("Invalid number of button Max allowed %d received %d",
                    g_app_context->button_num, action_resp.button_action_resp.num_buttons);
        }
    } else {
       SID_PAL_LOG_ERROR("Invalid response received %d", action_resp.resp_type);
    }
}

static void process_turn_all_leds(enum sid_demo_led_action led_action_req, struct sid_demo_action_resp *action_resp)
{
    if (led_action_req == SID_DEMO_ACTION_LED_ON) {
        turn_on_all_leds(g_app_context);
    } else {
        turn_off_all_leds(g_app_context);
    }

    action_resp->led_action_resp.action_resp = led_action_req;

    for (size_t i = 0; i < g_app_context->led_num; i++) {
        bool result = (led_action_req == SID_DEMO_ACTION_LED_ON) ? g_app_context->hw_ifc.is_led_on(g_app_context->led_id_arr[i]) :
                       !g_app_context->hw_ifc.is_led_on(g_app_context->led_id_arr[i]);
        if (result) {
            action_resp->led_action_resp.num_leds += 1;
            action_resp->led_action_resp.led_id_arr[i] = g_app_context->led_id_arr[i];
        }
    }

    if (action_resp->led_action_resp.num_leds) {
        demo_app_create_led_response(action_resp);
    } else {
        SID_PAL_LOG_ERROR("LED response invalid num leds %d", action_resp->led_action_resp.num_leds);
    }
}

static void process_turn_leds(struct sid_demo_led_action_req *led_req, struct sid_demo_action_resp *action_resp)
{
    for (size_t i = 0; i < led_req->num_leds; i++) {
        if (led_req->action_req == SID_DEMO_ACTION_LED_ON) {
            g_app_context->hw_ifc.turn_on_leds(led_req->led_id_arr[i]);
        } else {
            g_app_context->hw_ifc.turn_off_leds(led_req->led_id_arr[i]);
        }
    }
    action_resp->led_action_resp.action_resp = led_req->action_req;
    for (size_t i = 0; i < led_req->num_leds; i++) {
        bool result = (led_req->action_req == SID_DEMO_ACTION_LED_ON) ? g_app_context->hw_ifc.is_led_on(led_req->led_id_arr[i]) :
                       !g_app_context->hw_ifc.is_led_on(led_req->led_id_arr[i]);
        if (result) {
            action_resp->led_action_resp.num_leds += 1;
            action_resp->led_action_resp.led_id_arr[i] = led_req->led_id_arr[i];
        }
    }

    if (action_resp->led_action_resp.num_leds) {
        demo_app_create_led_response(action_resp);
    } else {
        SID_PAL_LOG_ERROR("LED response invalid num leds %d", action_resp->led_action_resp.num_leds);
    }
}

static void process_action_request(struct sid_parse_state *state)
{
    uint8_t temp_led_id_arr[APP_DEMO_LED_MAX] = {0};
    struct sid_demo_led_action_req led_req = {.led_id_arr = temp_led_id_arr};

    sid_demo_app_action_req_deserialize(state, &led_req);
    if (state->ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("de-serialize led action req failed %d", state->ret_code);
    } else if (led_req.action_req == SID_DEMO_ACTION_LED_ON || led_req.action_req == SID_DEMO_ACTION_LED_OFF) {
        uint8_t temp_led_id_arr_resp[APP_DEMO_LED_MAX] = {0};
        struct sid_demo_action_resp action_resp = {.resp_type = SID_DEMO_ACTION_TYPE_LED,};
        action_resp.led_action_resp.led_id_arr = temp_led_id_arr_resp;
        struct sid_timespec curr_time = {0};
        sid_get_time(g_app_context->sidewalk_handle, SID_GET_GPS_TIME, &curr_time);
        action_resp.gps_time_in_seconds = curr_time.tv_sec;
        if ((curr_time.tv_sec - led_req.gps_time_in_seconds) > 0) {
            action_resp.down_link_latency_secs = curr_time.tv_sec - led_req.gps_time_in_seconds;
        }
        if (led_req.num_leds == 0xFF) {
            process_turn_all_leds(led_req.action_req, &action_resp);
        } else if (led_req.num_leds <= g_app_context->led_num) {
            process_turn_leds(&led_req, &action_resp);
        } else {
            SID_PAL_LOG_ERROR("Invalid led action req max allowed %d received  %d", g_app_context->led_num, led_req.num_leds);
        }
    } else {
        SID_PAL_LOG_ERROR("Invalid led action request %d", led_req.action_req);
    }
}

void demo_app_button_event_handler(uint8_t idx)
{
    if (g_app_context->app_state != DEMO_APP_STATE_NOTIFY_SENSOR_DATA) {
        return;
    }
    bool notify_event = false;
    if (!(g_app_context->button_press_mask & (1 << g_app_context->button_id_arr[idx]))) {
        g_app_context->button_press_mask |= (1 << g_app_context->button_id_arr[idx]);
        notify_event = true;
    }
    if (!g_app_context->button_event_pending_processing && notify_event) {
        queue_event(g_app_context->event_queue, EVENT_BUTTON_PRESS, NULL, true);
        g_app_context->button_event_pending_processing = true;
    }
}

void demo_app_factory_reset_trigger_handler()
{
    queue_event(g_app_context->event_queue, EVENT_FACTORY_RESET, NULL, true);
}

void demo_app_receive_task_event_handler(struct app_demo_rx_msg *rx_msg)
{
    if (rx_msg == NULL) {
        SID_PAL_LOG_ERROR("Invalid rx_msg pointer");
        return;
    }
    struct sid_demo_msg_desc msg_desc = {0};
    static uint8_t temp_msg_payload[PAYLOAD_MAX_SIZE];
    memset(temp_msg_payload, 0, sizeof(temp_msg_payload));
    struct sid_demo_msg msg = {.payload = temp_msg_payload};
    static struct sid_parse_state state = {0};

    sid_parse_state_init(&state, rx_msg->rx_payload, rx_msg->pld_size);
    sid_demo_app_msg_deserialize(&state, &msg_desc, &msg);
    SID_PAL_LOG_INFO("opc %d, class %d cmd %d status indicator %d status_code %d paylaod size %d",
            msg_desc.opc, msg_desc.cmd_class, msg_desc.cmd_id, msg_desc.status_hdr_ind,
            msg_desc.status_code,  msg.payload_size);

    if (state.ret_code != SID_ERROR_NONE) {
        SID_PAL_LOG_ERROR("de-serialize demo app msg failed %d", state.ret_code);
    } else if (msg_desc.status_hdr_ind && msg_desc.opc == SID_DEMO_MSG_TYPE_RESP &&
            msg_desc.cmd_class == SID_DEMO_APP_CLASS && msg_desc.cmd_id == SID_DEMO_APP_CLASS_CMD_CAP_DISCOVERY_ID
            && msg_desc.status_code == SID_ERROR_NONE && msg.payload_size == 0) {
        SID_PAL_LOG_INFO("Capability response received");
        g_app_context->app_state = DEMO_APP_STATE_NOTIFY_SENSOR_DATA;
    } else if (msg_desc.status_hdr_ind && msg_desc.opc == SID_DEMO_MSG_TYPE_RESP &&
            msg_desc.cmd_class == SID_DEMO_APP_CLASS && msg_desc.cmd_id == SID_DEMO_APP_CLASS_CMD_ACTION
            && msg_desc.status_code == SID_ERROR_NONE) {
        SID_PAL_LOG_INFO("Action response received");
        sid_parse_state_init(&state, msg.payload, msg.payload_size);
        process_action_response(&state);
    } else if (msg_desc.opc == SID_DEMO_MSG_TYPE_WRITE && msg_desc.cmd_class == SID_DEMO_APP_CLASS && msg_desc.cmd_id == SID_DEMO_APP_CLASS_CMD_ACTION) {
        SID_PAL_LOG_INFO("Action request received");
        sid_parse_state_init(&state, msg.payload, msg.payload_size);
        process_action_request(&state);
    }

}

static void demo_app_check_button_press_notify(void)
{
    struct sid_timespec curr_time = {0};
    uint16_t next_timer_schedule_secs = 0;
    sid_get_time(g_app_context->sidewalk_handle, SID_GET_GPS_TIME, &curr_time);
    for (size_t i = 0; i < g_app_context->button_num; i++) {
        if ((g_app_context->button_notify_mask & (1 << g_app_context->button_id_arr[i])) &&
                g_app_context->button_press_time_in_sec_id_arr[i] != 0) {
            next_timer_schedule_secs = curr_time.tv_sec - g_app_context->button_press_time_in_sec_id_arr[i];
            SID_PAL_LOG_INFO("Button press timeout pre check: button_notify_mask %x next_timer_schedule_secs %d",
                    g_app_context->button_notify_mask, next_timer_schedule_secs);
            if (next_timer_schedule_secs >= BUTTON_PRESS_CHECK_PERIOD_SECS) {
                g_app_context->button_notify_mask &= ~(1 << g_app_context->button_id_arr[i]);
                g_app_context->button_press_mask &= ~(1 << g_app_context->button_id_arr[i]);
                next_timer_schedule_secs = 0;
                g_app_context->button_press_time_in_sec_id_arr[i] = 0;
            } else if (next_timer_schedule_secs > (curr_time.tv_sec - g_app_context->button_press_time_in_sec_id_arr[i])) {
                next_timer_schedule_secs = curr_time.tv_sec - g_app_context->button_press_time_in_sec_id_arr[i];
            }
        } else if (g_app_context->button_press_time_in_sec_id_arr[i]) {
            g_app_context->button_press_time_in_sec_id_arr[i] = 0;
        }
    }
    SID_PAL_LOG_INFO("Button press timeout post check: button_notify_mask %x next_timer_schedule_secs %d",
            g_app_context->button_notify_mask, next_timer_schedule_secs);

    if (next_timer_schedule_secs) {
        BaseType_t ret = xTimerChangePeriod(g_app_context->button_press_timer_handle, pdMS_TO_TICKS(next_timer_schedule_secs * 1000), 0);
        SID_PAL_ASSERT(ret == pdPASS);
    }
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
    SID_PAL_LOG_ERROR("failed to send message(type: %d, id: %u), err:%d",
                  (int)msg_desc->type, msg_desc->id, (int)error);
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

    if (g_app_context->app_state == DEMO_APP_STATE_INIT &&
            status->detail.registration_status == SID_STATUS_REGISTERED) {
        g_app_context->app_state = DEMO_APP_STATE_REGISTERED;
    }

    if (g_app_context->sidewalk_state == STATE_SIDEWALK_READY) {
        TickType_t delay = pdMS_TO_TICKS(DEMO_CAPABILITY_PERIOD_MS);
        if (g_app_context->app_state == DEMO_APP_STATE_REGISTERED) {
            g_app_context->app_state = DEMO_APP_STATE_NOTIFY_CAPABILITY;
        } else if (g_app_context->app_state == DEMO_APP_STATE_NOTIFY_SENSOR_DATA) {
            delay = DEMO_NOTIFY_SENSOR_DATA_PERIOD_MS;
        }
        BaseType_t ret = xTimerIsTimerActive(g_app_context->cap_timer_handle);
        if (ret == pdTRUE) {
            ret = xTimerStop(g_app_context->cap_timer_handle, 0);
            SID_PAL_ASSERT(ret == pdPASS);
        }
        ret = xTimerChangePeriod(g_app_context->cap_timer_handle, pdMS_TO_TICKS(delay), 0);
        SID_PAL_ASSERT(ret == pdPASS);
    }

    if (BUILD_SID_SDK_LINK_TYPE == 1) {
        if (!(status->detail.link_status_mask & SID_LINK_TYPE_1) && (status->detail.registration_status == SID_STATUS_REGISTERED)
            && (status->detail.time_sync_status == SID_STATUS_TIME_SYNCED)) {
        BaseType_t ret = xTimerIsTimerActive(g_app_context->cap_timer_handle);
        if (ret == pdTRUE) {
            ret = xTimerStop(g_app_context->cap_timer_handle, 0);
            SID_PAL_ASSERT(ret == pdPASS);
        }
        ret = xTimerChangePeriod(g_app_context->cap_timer_handle, pdMS_TO_TICKS(CONNECT_LINK_TYPE_1_INIT_DELAY_MS), 0);
        SID_PAL_ASSERT(ret == pdPASS);
        }
    }

    if (BUILD_SID_SDK_LINK_TYPE == 2) {
        if ((status->detail.link_status_mask & SID_LINK_TYPE_2) && (status->detail.registration_status == SID_STATUS_REGISTERED)
            && (status->detail.time_sync_status == SID_STATUS_TIME_SYNCED)) {
            BaseType_t ret = xTimerIsTimerActive(g_app_context->device_profile_timer_handle);
            if (ret == pdTRUE) {
                ret = xTimerStop(g_app_context->device_profile_timer_handle, 0);
                SID_PAL_ASSERT(ret == pdPASS);
            }
            ret = xTimerChangePeriod(g_app_context->device_profile_timer_handle, pdMS_TO_TICKS(PROFILE_CHECK_TIMER_DELAY_MS), 0);
            SID_PAL_ASSERT(ret == pdPASS);
        }
    }
}

static void on_sidewalk_factory_reset(void *context)
{
    SID_PAL_LOG_ERROR("factory reset notification received from sid api");
    g_app_context->hw_ifc.platform_reset();
}

static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context)
{
    SID_PAL_LOG_INFO("received message(type: %d, link_mode: %d, id: %u size %u rssi %d snr %d)", (int)msg_desc->type,
                 (int)msg_desc->link_mode, msg_desc->id, msg->size,
                 (int)msg_desc->msg_desc_attr.rx_attr.rssi,  (int)msg_desc->msg_desc_attr.rx_attr.snr);
    log_sid_msg(msg);

    if (msg_desc->type == SID_MSG_TYPE_RESPONSE && msg_desc->msg_desc_attr.rx_attr.is_msg_ack) {
        SID_PAL_LOG_INFO("Received Ack for msg id %d", msg_desc->id);
    } else {
        struct app_demo_rx_msg rx_msg = {
            .msg_id = msg_desc->id,
            .pld_size = msg->size,
        };
        memcpy(rx_msg.rx_payload, msg->data, msg->size);
        queue_rx_msg(g_receive_context->receive_event_queue, &rx_msg, false);
    }
}

struct sid_event_callbacks *demo_app_get_sid_event_callbacks()
{
    static struct sid_event_callbacks sid_callbacks = {
        .on_event = on_sidewalk_event, /* Called from ISR context */
        .on_msg_received = on_sidewalk_msg_received, /* Called from sid_process() */
        .on_msg_sent = on_sidewalk_msg_sent,  /* Called from sid_process() */
        .on_send_error = on_sidewalk_send_error, /* Called from sid_process() */
        .on_status_changed = on_sidewalk_status_changed, /* Called from sid_process() */
        .on_factory_reset = on_sidewalk_factory_reset, /* Called from sid_process */
    };
    sid_callbacks.context = g_app_context;
    return &sid_callbacks;
}

void demo_app_main_task_event_handler(struct app_demo_event *event)
{
    if (event) {
        switch (event->type) {
            case EVENT_TYPE_SIDEWALK:
                sid_process(g_app_context->sidewalk_handle);
                break;
            case EVENT_FACTORY_RESET:
                factory_reset();
                break;
            case EVENT_BUTTON_PRESS:
                demo_app_notify_sensor_data(true);
                g_app_context->button_event_pending_processing = false;
                break;
            case EVENT_NOTIFICATION_TIMER_FIRED:
                if (g_app_context->sidewalk_state != STATE_SIDEWALK_READY) {
                    if (BUILD_SID_SDK_LINK_TYPE == 1) {
                        if (!(g_app_context->link_status.link_mask & SID_LINK_TYPE_1) &&
                                (g_app_context->app_state == DEMO_APP_STATE_REGISTERED)
                            && (g_app_context->link_status.time_sync_status == SID_STATUS_TIME_SYNCED)) {
                            queue_event(g_app_context->event_queue, EVENT_CONNECT_LINK_TYPE_1, NULL, false);
                        }
                    }
                } else if (g_app_context->app_state == DEMO_APP_STATE_NOTIFY_CAPABILITY) {
                    demo_app_notify_capability();
                } else if (g_app_context->app_state == DEMO_APP_STATE_NOTIFY_SENSOR_DATA) {
                    demo_app_notify_sensor_data(false);
                }
                break;
            case EVENT_BUTTON_PRESS_TIMER_FIRED:
                SID_PAL_LOG_INFO("Button press timeout check timer fired");
                demo_app_check_button_press_notify();
                break;
            case EVENT_CONNECT_LINK_TYPE_1:
                if (BUILD_SID_SDK_LINK_TYPE == 1) {
                    SID_PAL_LOG_INFO("Connecting link type 1");
                    sid_error_t ret = sid_ble_bcn_connection_request(g_app_context->sidewalk_handle, true);
                    if (ret != SID_ERROR_NONE) {
                        SID_PAL_LOG_ERROR("Failed to set connect request on link type 1 %d", ret);
                    }
                }
                break;
            case EVENT_SET_DEVICE_PROFILE_LINK_TYPE_2:
                if (BUILD_SID_SDK_LINK_TYPE == 2) {
                    BaseType_t err = xTimerIsTimerActive(g_app_context->device_profile_timer_handle);
                    if (err == pdTRUE) {
                        err = xTimerStop(g_app_context->device_profile_timer_handle, 0);
                        SID_PAL_ASSERT(err == pdPASS);
                    }
                    struct sid_device_profile curr_dev_cfg = {
                        .unicast_params = {
                            .device_profile_id = SID_LINK2_PROFILE_2,
                        },
                    };
                    sid_error_t ret = sid_option(g_app_context->sidewalk_handle, SID_OPTION_900MHZ_GET_DEVICE_PROFILE, &curr_dev_cfg, sizeof(curr_dev_cfg));
                    if (ret == SID_ERROR_NONE) {
                        struct sid_device_profile target_dev_cfg = {
                            .unicast_params = {
                                .device_profile_id = SID_LINK2_PROFILE_2,
                                .rx_window_count = SID_RX_WINDOW_CNT_INFINITE,
                                .unicast_window_interval = {
                                    .sync_rx_interval_ms = SID_LINK2_RX_WINDOW_SEPARATION_1,
                                },
                                .wakeup_type = SID_TX_AND_RX_WAKEUP,
                            },
                        };
                        if (!memcmp(&curr_dev_cfg, &target_dev_cfg, sizeof(curr_dev_cfg))) {
                            SID_PAL_LOG_INFO("Device profile for Link type 2 already set");
                        } else {
                            ret = sid_option(g_app_context->sidewalk_handle, SID_OPTION_900MHZ_SET_DEVICE_PROFILE, &target_dev_cfg, sizeof(target_dev_cfg));
                            if (ret != SID_ERROR_NONE) {
                                SID_PAL_LOG_ERROR("Device profile configuration for Link type 2 failed ret = %d", ret);
                                err = xTimerChangePeriod(g_app_context->device_profile_timer_handle, pdMS_TO_TICKS(PROFILE_CHECK_TIMER_DELAY_MS), 0);
                                SID_PAL_ASSERT(err == pdPASS);
                            } else {
                                SID_PAL_LOG_INFO("Device profile Link type 2 set success");
                            }
                        }
                    } else {
                        SID_PAL_LOG_ERROR("Failed to get device profile configuration for Link type 2 ret = %d", ret);
                        err = xTimerChangePeriod(g_app_context->device_profile_timer_handle, pdMS_TO_TICKS(PROFILE_CHECK_TIMER_DELAY_MS), 0);
                        SID_PAL_ASSERT(err == pdPASS);
                    }
                }
            case EVENT_SEND_MESSAGE:
                if (event->data) {
                    struct app_demo_tx_msg *evt_data = (struct app_demo_tx_msg*)event->data;
                    struct sid_msg msg = {.size = evt_data->pld_size, .data = evt_data->tx_payload};

                    send_msg(&evt_data->desc, &msg);
                } else {
                    SID_PAL_LOG_ERROR("Invalid data pointer in event");
                }
                break;
            default:
                SID_PAL_LOG_ERROR("Invalid event queued %d", event);
                break;
        }
        if (event->data != NULL) {
            free(event->data);
        }
    }
}

void demo_app_init(app_context_t *app_context, receive_context_t *rcv_context)
{
    SID_PAL_ASSERT(app_context);
    SID_PAL_ASSERT(rcv_context);

    g_app_context = app_context;
    g_receive_context = rcv_context;

    g_app_context->cap_timer_handle = xTimerCreate("demo_cap_timer",
                                                  pdMS_TO_TICKS(DEMO_CAPABILITY_PERIOD_MS),
                                                  pdFALSE, (void *)0, cap_timer_cb);

    if (g_app_context->cap_timer_handle == NULL) {
        SID_PAL_LOG_ERROR("create capability timer failed!");
        SID_PAL_ASSERT(false);
    }

    g_app_context->button_press_timer_handle = xTimerCreate("button_press_timer",
                                              pdMS_TO_TICKS(BUTTON_PRESS_CHECK_PERIOD_SECS * 1000),
                                              pdFALSE, (void *)0, button_press_timer_cb);

    if (g_app_context->button_press_timer_handle == NULL) {
        SID_PAL_LOG_ERROR("create button press timer failed!");
        SID_PAL_ASSERT(false);
    }

    if (BUILD_SID_SDK_LINK_TYPE == 2) {
        g_app_context->device_profile_timer_handle = xTimerCreate("device_profile_timer",
                                                      pdMS_TO_TICKS(PROFILE_CHECK_TIMER_DELAY_MS),
                                                      pdFALSE, (void *)0, device_profile_timer_cb);

        if (g_app_context->device_profile_timer_handle == NULL) {
            SID_PAL_LOG_ERROR("create device profile timer failed!");
            SID_PAL_ASSERT(false);
        }
    }
    g_app_context->sidewalk_state = STATE_SIDEWALK_NOT_READY;

    g_app_context->event_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(struct app_demo_event));
    if (g_app_context->event_queue == NULL) {
        SID_PAL_ASSERT(false);
    }

    g_receive_context->receive_event_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(struct app_demo_rx_msg));
    if (g_receive_context->receive_event_queue == NULL) {
        SID_PAL_ASSERT(false);
    }
}
