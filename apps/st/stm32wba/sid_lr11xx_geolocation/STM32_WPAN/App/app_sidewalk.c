/**
  ******************************************************************************
  * @file    app_sidewalk.c
  * @brief   Sidewalk SubGHz link co-existence with LR11xx geolocation features
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

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_freertos.h"

/* Sidewalk SDK headers */
#include <sid_api.h>
#include <sid_hal_reset_ifc.h>
#include <sid_log_control.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_common_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_crypto_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_timer_ifc.h>
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>
#include <sid_900_cfg.h>
#include <sid_stm32_common_utils.h>

/* Semtech LoRa Basics Modem middleware */
#include <smtc_modem_ext_api.h>
#include <smtc_modem_geolocation_api.h>
#include <smtc_modem_ext_ral_bsp.h>
#include <smtc_modem_utilities.h>

/* Version information */
#include SID_APP_VERSION_HEADER
#include <sid_sdk_version.h>
#include <lora_basics_modem_version.h>
#include <lr11xx_driver_version.h>

#include "target/memory.h"

#include "app_ble_config.h"
#include "app_900_config.h"
#include <lr11xx_radio_ext_ifc.h>
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

#define SID_MSG_QUEUE_LEN                           (10u)

#define BUTTON_LONG_PRESS_MS                        (1500u)

/* Select Sidewalk link type for communication - LoRa or FSK */
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 && !SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
#  define SID_COMM_LINK_TYPE                        SID_LINK_TYPE_2 /* FSK link */
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
#  define SID_COMM_LINK_TYPE                        SID_LINK_TYPE_3 /* LoRa link */
#else
#  error "Invalid configuration. This sample app is designed to use either FSK or LoRa Sidewalk links for normal operation"
#endif

/* Select Sidewalk link type for initial device registration - BLE or FSK */
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_1
#  define SID_REGISTRATION_LINK_TYPE                SID_LINK_TYPE_1 /* BLE link */
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 && !SID_SDK_CONFIG_ENABLE_LINK_TYPE_1
#  define SID_REGISTRATION_LINK_TYPE                SID_LINK_TYPE_2 /* FSK link */
#else
#  error "Sidewalk device registration is only possible via BLE or FSK link, but none of them is enabled in the configuration"
#endif

#if !SID_SDK_CONFIG_ENABLE_LINK_TYPE_1 && SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 && !SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
#  define SID_FSK_ONLY_LINK                         (1)
#else
#  define SID_FSK_ONLY_LINK                         (0)
#endif

#define GEOLOCATION_DEMO_LBM_STACK_ID               (0)    /*!< Stack ID for the LoRa Basics Modem library. By default LBM is built for a single stack, so this value is 0 in most of the cases */
#define GEOLOCATION_DEMO_UPDATE_INTERVAL_S          (180u) /*!< Interval (in seconds) between two successful location scans. The app will idle after a successful scan for this duration before initiating a new scan */
#define GEOLOCATION_DEMO_UPLINK_TTL                 (300u) /*!< TTL (in seconds) for a Sidewalk acknowledged uplink message */
#define GEOLOCATION_DEMO_UPLINK_RETRIES             (5u)   /*!< Number of retries for Sidewalk acknowledged uplink delivery */

#ifndef GEOLOCATION_DEMO_USE_WIFI
#  define GEOLOCATION_DEMO_USE_WIFI                 (1)    /*!< Set to 1 to enable WiFi scanning features of the LR11xx transceiver */
#endif /* GEOLOCATION_DEMO_USE_WIFI */
#define GEOLOCATION_DEMO_WIFI_SCANS_MIN             (2u)   /*!< Minimum quantity of discovered WiFi networks to estimate location. Most of location resolving services require at lest two networks for privacy reasons */
#define GEOLOCATION_DEMO_WIFI_SCAN_RETRY_LIMIT      (3u)   /*!< Maximum number of retries if WiFi scan fails or does not discover enough WiFi networks nearby. 0 means just a single scan, no retries */
#define GEOLOCATION_DEMO_WIFI_SCAN_RETRY_INTERVAL_S (30u)  /*!< Delay between WiFi rescans (in seconds). The scan will be relaunched after this delay */

#ifndef GEOLOCATION_DEMO_USE_GNSS
#  define GEOLOCATION_DEMO_USE_GNSS                 (1)    /*!< Set to 1 to enable GNSS scanning features of the LR11xx transceiver */
#endif /* GEOLOCATION_DEMO_USE_GNSS */
#define GEOLOCATION_DEMO_GNSS_SCAN_RETRY_LIMIT      (5u)   /*!< Maximum number of retries if GNSS scan fails or does not discover enough space vehicles. 0 means just a single scan, no retries */
#define GEOLOCATION_DEMO_GNSS_SCAN_RETRY_INTERVAL_S (60u)  /*!< Delay between GNSS rescans (in seconds). The scan will be relaunched after this delay */
#define GEOLOCATION_DEMO_GNSS_CONSTELLATIONS_TO_USE (SMTC_MODEM_GNSS_CONSTELLATION_GPS_BEIDOU) /*!< Constellations to use in GNSS scan. Can be SMTC_MODEM_GNSS_CONSTELLATION_GPS for GPS-only, SMTC_MODEM_GNSS_CONSTELLATION_BEIDOU for BeiDou, or SMTC_MODEM_GNSS_CONSTELLATION_GPS_BEIDOU for both */
#define GEOLOCATION_DEMO_GNSS_SCAN_MODE_TO_USE      (SMTC_MODEM_GNSS_MODE_MOBILE) /*!< GNSS scan mode. SMTC_MODEM_GNSS_MODE_MOBILE is faster and suits better for moving devices, SMTC_MODEM_GNSS_MODE_STATIC is slower due to additional wait time for sky diversity and suits better for static objects */

#if (!GEOLOCATION_DEMO_USE_WIFI) && (!GEOLOCATION_DEMO_USE_GNSS)
#  error "All geolocation features are disabled. Please enable WiFi scanning, GNSS scanning, or both"
#endif /* !GEOLOCATION_DEMO_USE_WIFI && !GEOLOCATION_DEMO_USE_GNSS */

#define GEOLOCATION_DEMO_FULL_LBM_SID_CONCURRENCY   (((GEOLOCATION_DEMO_USE_WIFI && LR11XX_RADIO_CFG_WIFI_SCAN_USE_BLANKING) || !GEOLOCATION_DEMO_USE_WIFI) \
                                                  && ((GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING) || !GEOLOCATION_DEMO_USE_GNSS))

#define SID_LINK_TYPE_NONE                          (0u)

#define GEOLOCATION_DEMO_TLV_TAG_COUNTER            (0xC7u) /*!< Demo counter value. This has no relation to geolocation. Demo Counter is an incrementing value that is sent to the cloud whenever user presses a button */
#define GEOLOCATION_DEMO_TLV_TAG_WIFI               (0x3Fu) /*!< Nearby WiFi networks discovery data. It contains BSSIDs of the nearby WiFi networks with associated RSSI value for each BSSID */
#define GEOLOCATION_DEMO_TLV_TAG_GNSS_NO_AP         (0x62u) /*!< GNSS scan results for cold scan (no assist position is available). This record provides NAV3 message(s) along with their timestamps (GPST scan time) and timestamp accuracy (in ms) */
#define GEOLOCATION_DEMO_TLV_TAG_GNSS_AP            (0x63u) /*!< GNSS scan results whenever assisted mode was used. It adds assist position reported by the LR11xx and the rest of the payload is identical to cold-start message */

#define SID_MAX_MTU                                 (200u)  /*!< This app uses either FSK or LoRa link to send geolocation data. LoRa maximum MTU is 19 and FSK is 200 as per Sidewalk specification */

/* Private macro -----------------------------------------------------------*/

#define OS_MS_TO_TICKS( xTimeInMs )                 ( ( uint32_t ) ( ( ( uint32_t ) ( xTimeInMs ) * ( uint32_t ) osKernelGetTickFreq() ) / ( uint32_t ) 1000U ) )

/* Private typedef -----------------------------------------------------------*/

enum event_type
{
    EVENT_TYPE_SIDEWALK_INIT,
    EVENT_TYPE_SIDEWALK_START_LINK,
    EVENT_TYPE_SIDEWALK_STOP_LINK,
    EVENT_TYPE_SIDEWALK_PROCESS,
    EVENT_TYPE_SIDEWALK_SEND_DEMO_COUNTER,
    EVENT_TYPE_SIDEWALK_SEND_GEOLOCATION_DATA,
    EVENT_TYPE_SIDEWALK_FACTORY_RESET,
    EVENT_TYPE_SIDEWALK_SET_DEVICE_PROFILE,
    EVENT_TYPE_SIDEWALK_REGISTRATION_COMPLETED,
    EVENT_TYPE_STANDBY,
    EVENT_TYPE_SMTC_LBM_INIT,
    EVENT_TYPE_SMTC_LBM_PROCESS,
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
    bool lbm_initialized;
} app_context_t;

enum sid_subghz_profile_idx {
    SID_LORA_PROFILE_A         = 0,
    SID_LORA_PROFILE_B         = 1,
    SID_FSK_PROFILE_1          = 2,
    SID_FSK_PROFILE_2          = 3,
    SID_SUBGHZ_PROFILE_UNKNOWN = 4,
    PROFILE_LAST               = SID_SUBGHZ_PROFILE_UNKNOWN,
};

#if GEOLOCATION_DEMO_USE_WIFI
typedef __PACKED_STRUCT {
    uint8_t bssid[6]; /*!< BSSID of the discovered WiFi network */
    int8_t  rssi;     /*!< RSSI of the discovered WiFi network */
} sid_uplink_wifi_record_t;
#endif /* GEOLOCATION_DEMO_USE_WIFI */

#if GEOLOCATION_DEMO_USE_GNSS
typedef __PACKED_STRUCT {
    uint32_t timestamp;          /*!< Scan timestamp (GPS time in seconds modulo 1024 weeks) */
    uint8_t  nav_size;           /*!< NAV3 message size (number of valid bytes in nav_msg field) */
    uint8_t  nav_msg[UINT8_MAX]; /*!< NAV3 message payload, the actual length of the message is variable */
} sid_uplink_gnss_scan_data_t;

typedef __PACKED_STRUCT {
    uint16_t                    time_accuracy; /*!< Timestamp accuracy (in ms) */
    sid_uplink_gnss_scan_data_t gnss_scan_results[1];
} sid_uplink_gnss_no_ap_record_t;

typedef __PACKED_STRUCT {
    int16_t                        ap_lat;         /*!< Assist position latitude (uses the same encoding as LR11xx) */
    int16_t                        ap_long;        /*!< Assist position longitude (uses the same encoding as LR11xx) */
    sid_uplink_gnss_no_ap_record_t gnss_scan_data; /*!< The rest of the payload is identical to sid_uplink_gnss_no_ap_record_t to simplify deserialization */
} sid_uplink_gnss_ap_record_t;
#endif /* GEOLOCATION_DEMO_USE_GNSS */

typedef __PACKED_STRUCT {
    uint8_t counter_value; /*!< Counter value that is sent to the cloud each time the user presses the button */
} sid_uplink_demo_counter_record_t;

typedef uint16_t sid_uplink_token_t;

typedef __PACKED_STRUCT {
    __PACKED_STRUCT {
        uint8_t tag;
        uint8_t length;
    } header;
    __PACKED_UNION {
        uint8_t                          raw[1];
#if GEOLOCATION_DEMO_USE_WIFI
        sid_uplink_wifi_record_t         wifi_scan_results[1];
#endif /* GEOLOCATION_DEMO_USE_WIFI */
#if GEOLOCATION_DEMO_USE_GNSS
        sid_uplink_gnss_no_ap_record_t   gnss_no_ap_scan_results;
        sid_uplink_gnss_ap_record_t      gnss_ap_scan_results;
#endif /* GEOLOCATION_DEMO_USE_GNSS */
        sid_uplink_demo_counter_record_t demo_counter;
    } payload; /*!< TLV record payload. THe actual amount of raw bytes or scan records is variable */
} sid_uplink_tlv_record_t;

typedef __PACKED_STRUCT {
    __PACKED_STRUCT {
        sid_uplink_token_t      token;            /*!< A reasonably unique value that is used to identify all fragments belonging to the same uplink message. All fragments should share the same token value */
        uint8_t                 total_fragments;  /*!< Total number of fragments for the current uplink message */
        uint8_t                 current_fragment; /*!< Index of the current fragment */
    } header;
    __PACKED_UNION {
        uint8_t                 raw[1];
        sid_uplink_tlv_record_t tlv_record[1];
    } tlv_payload; /*!< Uplink message payload (collection of TLV records). The actual amount of raw bytes or TLV records is variable */
} sid_uplink_message_t;

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

static osSemaphoreId_t sidewalk_registration_ready_semaphore;
static osSemaphoreId_t sidewalk_connection_ready_semaphore;
static osSemaphoreId_t sidewalk_connection_stopped_semaphore;
static osSemaphoreId_t sidewalk_message_delivered_semaphore;

static osSemaphoreId_t          lbm_ready_semaphore;
static smtc_modem_return_code_t lbm_init_error;

#if GEOLOCATION_DEMO_USE_WIFI
static osSemaphoreId_t                        lbm_wifi_scan_ready_semaphore;
static uint32_t                               lbm_wifi_scan_retry_counter = 0u;
static smtc_modem_wifi_event_data_scan_done_t lbm_wifi_scan_done_data     = {0};
#endif /* GEOLOCATION_DEMO_USE_WIFI */

#if GEOLOCATION_DEMO_USE_GNSS
static osSemaphoreId_t                        lbm_gnss_scan_ready_semaphore;
static uint32_t                               lbm_gnss_scan_retry_counter = 0u;
static smtc_modem_gnss_event_data_scan_done_t lbm_gnss_scan_done_data     = {0};
static bool                                   lbm_almanac_demod_started   = false;
#endif /* GEOLOCATION_DEMO_USE_GNSS */

static osThreadId_t geolocation_demo_task;

static uint8_t geodata_serialization_buffer[256]; /*!< A working buffer to build the serialized payload from WiFi and GNSS scan data */

/* Indicates if Sidewalk registration process is pending */
static bool registration_pending = false;

static sid_pal_timer_t lbm_process_scheduler_timer;

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

static const osSemaphoreAttr_t lbm_ready_sem_attributes = {
    .name       = "LBM Ready Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

#if GEOLOCATION_DEMO_USE_WIFI
static const osSemaphoreAttr_t lbm_wifi_scan_ready_sem_attributes = {
    .name       = "LBM WiFi Ready Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};
#endif /* GEOLOCATION_DEMO_USE_WIFI */

#if GEOLOCATION_DEMO_USE_GNSS
static const osSemaphoreAttr_t lbm_gnss_scan_ready_sem_attributes = {
    .name       = "LBM GNSS Ready Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};
#endif /* GEOLOCATION_DEMO_USE_GNSS */

static const osSemaphoreAttr_t sidewalk_registration_ready_sem_attributes = {
    .name       = "Sidewalk Registration Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t sidewalk_connection_ready_sem_attributes = {
    .name       = "Sidewalk Conn Ready Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t sidewalk_connection_stopped_sem_attributes = {
    .name       = "Sidewalk Conn Stop Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osSemaphoreAttr_t sidewalk_msg_delivered_sem_attributes = {
    .name       = "Sidewalk Msg Delivered Semaphore",
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
};

static const osThreadAttr_t geolocation_demo_task_attributes = {
    .name         = "Geolocation Demo Task",
    .priority     = GEOLOCATION_DEMO_TASK_PRIO,
    .stack_size   = GEOLOCATION_DEMO_TASK_STACK_SIZE,
    .attr_bits    = TASK_DEFAULT_ATTR_BITS,
    .cb_mem       = TASK_DEFAULT_CB_MEM,
    .cb_size      = TASK_DEFAULT_CB_SIZE,
    .stack_mem    = TASK_DEFAULT_STACK_MEM,
};

#if SID_PAL_LOG_ENABLED
static const char * const sid_subghz_profile_strings[] = {
    [SID_LORA_PROFILE_A]         = "LoRa Profile A",
    [SID_LORA_PROFILE_B]         = "LoRa Profile B",
    [SID_FSK_PROFILE_1]          = "FSK Profile 1",
    [SID_FSK_PROFILE_2]          = "FSK Profile 2",
    [SID_SUBGHZ_PROFILE_UNKNOWN] = "Unknown",
};

static const char * const sid_subghz_wakeup_type_strings[] = {
    [SID_NO_WAKEUP]        = "No Wakeup",
    [SID_TX_ONLY_WAKEUP]   = "Tx Only",
    [SID_RX_ONLY_WAKEUP]   = "Rx Only",
    [SID_TX_AND_RX_WAKEUP] = "Tx & Rx",
};
#endif /* SID_PAL_LOG_ENABLED */

/* Private function prototypes -----------------------------------------------*/

static void         queue_event(osMessageQueueId_t queue, enum event_type event);

static void         on_sidewalk_event(bool in_isr, void *context);
static void         on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context);
static void         on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context);
static void         on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc, void *context);
static void         on_sidewalk_status_changed(const struct sid_status *status, void *context);
static void         on_sidewalk_factory_reset(void *context);

static sid_error_t  send_sidewalk_uplink(app_context_t * const app_context, const uint8_t * const data, const size_t data_size);
#if GEOLOCATION_DEMO_USE_WIFI
static sid_error_t  serialize_wifi_scan_data(const smtc_modem_wifi_event_data_scan_done_t * const scan_data, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len);
#endif /* GEOLOCATION_DEMO_USE_WIFI */
#if GEOLOCATION_DEMO_USE_GNSS
static sid_error_t  serialize_gnss_scan_data(const smtc_modem_gnss_event_data_scan_done_t * const scan_data, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len);
#endif /* GEOLOCATION_DEMO_USE_GNSS */
static sid_error_t serialize_demo_counter_data(const uint8_t counter_value, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len);

static void         factory_reset(app_context_t * const context);
static void         print_subghz_link_profile(const app_context_t * const context);
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
static void         adjust_sidewalk_link_log_level(const uint32_t link_mask);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
static sid_error_t  init_link_and_check_sid_registration(app_context_t * const context, struct sid_config * const config);

static void         lbm_process_scheduler_timer_cb(void * arg, sid_pal_timer_t * originator);
static void         schedule_smtc_modem_engine_run(const uint32_t delay_ms);
static void         smtc_lbm_event_callback(void);

static void         sidewalk_stack_task_entry(void *context);
static void         geolocation_demo_task_entry(void *context);

#if SID_PAL_LOG_ENABLED
static const char * sid_subghz_profile_code_to_str(enum sid_device_profile_id code);
#endif /* SID_PAL_LOG_ENABLED */
#if (CFG_BUTTON_SUPPORTED == 1)
#  if !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
static void         button1_irq_handler(uint32_t pin, void * callback_arg);
#  endif /* !STM32WBA6x */
#  if !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
static void         button2_irq_handler(uint32_t pin, void * callback_arg);
#  endif /* !STM32WBA5x */
static void         button3_irq_handler(uint32_t pin, void * callback_arg);
#endif

/* Private function definitions ----------------------------------------------*/

static void queue_event(osMessageQueueId_t queue, enum event_type event)
{
    osStatus_t os_status = osMessageQueuePut(queue, &event, 0u, 0u);

    if (os_status != osOK)
    {
        SID_PAL_LOG_ERROR("Failed to queue event %u. Error %d", (uint32_t)event, (int32_t)os_status);
    }
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_event(bool in_isr, void *context)
{
    (void)in_isr;

    app_context_t *app_context = (app_context_t *)context;
    queue_event(app_context->event_queue, EVENT_TYPE_SIDEWALK_PROCESS);
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_msg_received(const struct sid_msg_desc *msg_desc, const struct sid_msg *msg, void *context)
{
    if ((SID_MSG_TYPE_RESPONSE == msg_desc->type) && (msg_desc->msg_desc_attr.rx_attr.is_msg_ack != false))
    {
        SID_PAL_LOG_INFO("Received Sidewalk ACK for message ID %u (link_mode: %d)", msg_desc->id, (int32_t)msg_desc->link_mode);
        osSemaphoreRelease(sidewalk_message_delivered_semaphore);
    }
    else
    {
        SID_PAL_LOG_INFO("Received Sidewalk downlink (type: %d, link_mode: %d, id: %u size %u)", (int32_t)msg_desc->type,
                                 (int32_t)msg_desc->link_mode, msg_desc->id, msg->size);
        SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, msg->data, msg->size);
    }
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_RCV_OK);
#endif
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_msg_sent(const struct sid_msg_desc *msg_desc, void *context)
{
    SID_PAL_LOG_INFO("Sent Sidewalk uplink (type: %d, id: %u)", (int32_t)msg_desc->type, msg_desc->id);
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_SENT_OK);
#endif
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_send_error(sid_error_t error, const struct sid_msg_desc *msg_desc, void *context)
{
    SID_PAL_LOG_ERROR("Failed to send Sidewalk uplink (type: %d, id: %u), err:%d",
                  (int32_t)msg_desc->type, msg_desc->id, (int32_t)error);
#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_SEND_ERROR);
#endif
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_status_changed(const struct sid_status *status, void *context)
{
    app_context_t *app_context = (app_context_t *)context;
    SID_PAL_LOG_INFO("Sidewalk status changed: %d", (int32_t)status->state);
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
                osSemaphoreRelease(sidewalk_connection_ready_semaphore);
            }
            break;

        case SID_STATE_NOT_READY:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_BONDING);
#endif
            app_context->state = STATE_SIDEWALK_NOT_READY;
            osSemaphoreRelease(sidewalk_connection_stopped_semaphore);
            break;

        case SID_STATE_ERROR:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_ERROR);
#endif
            SID_PAL_LOG_ERROR("sidewalk error: %d", (int32_t)sid_get_error(app_context->sidewalk_handle));
            SID_PAL_ASSERT(0);
            break;

        case SID_STATE_SECURE_CHANNEL_READY:
            app_context->state = STATE_SIDEWALK_SECURE_CONNECTION;
            break;

        default:
#if CFG_LED_SUPPORTED
            (void)led_indication_set(LED_INDICATE_ERROR);
#endif
            SID_PAL_LOG_ERROR("Unknown Sidewalk state received: %d", (int32_t)status->state);
            SID_PAL_ASSERT(0);
            break;
    }

    SID_PAL_LOG_INFO("Registration Status = %d, Time Sync Status = %d and Link Status Mask = %x",
                 status->detail.registration_status, status->detail.time_sync_status,
                 status->detail.link_status_mask);

    if ((registration_pending != false) && (SID_STATUS_REGISTERED == status->detail.registration_status))
    {
        /* Device registration completed */
        SID_PAL_LOG_INFO("Sidewalk Device Registration done");
        queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_REGISTRATION_COMPLETED);
    }

    app_context->link_status.link_mask = status->detail.link_status_mask;
    for (uint32_t i = 0u; i < SID_LINK_TYPE_MAX_IDX; i++)
    {
        app_context->link_status.supported_link_mode[i] = status->detail.supported_link_modes[i];
        SID_PAL_LOG_INFO("Link %d Mode %x", i, status->detail.supported_link_modes[i]);
    }
    if ((status->detail.link_status_mask & (SID_LINK_TYPE_2 | SID_LINK_TYPE_3)) != 0u)
    {
        print_subghz_link_profile(context);
    }
}

/*----------------------------------------------------------------------------*/

static void on_sidewalk_factory_reset(void *context)
{
    (void)context;

    SID_PAL_LOG_INFO("factory reset notification received from sid api");
    SID_PAL_LOG_FLUSH();
    (void)sid_hal_reset(SID_HAL_RESET_NORMAL);
}

/*----------------------------------------------------------------------------*/

static sid_error_t send_sidewalk_uplink(app_context_t * const app_context, const uint8_t * const data, const size_t data_size)
{
    sid_error_t ret;

    do
    {
        size_t mtu;

        /* Ensure Sidewalk link is good */
        if ((app_context->state != STATE_SIDEWALK_READY) &&
            (app_context->state != STATE_SIDEWALK_SECURE_CONNECTION))
        {
            SID_PAL_LOG_ERROR("Can't send Sidewalk uplink, Sidewalk connection is not ready");
            ret = SID_ERROR_PORT_NOT_OPEN;
            break;
        }

        /* Query Sidewalk MTU size */
        ret = sid_get_mtu(app_context->sidewalk_handle, app_context->link_status.link_mask, &mtu);
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Can't get MTU size for Sidewalk link %u. Error %d", app_context->link_status.link_mask, (int32_t)ret);
            break;
        }

        /* Generate uplink token */
        sid_uplink_token_t uplink_token;
        ret = sid_pal_crypto_rand((uint8_t *)(void *)&uplink_token, sizeof(uplink_token));
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Can't generate token for Sidewalk uplink. Error %d", (int32_t)ret);
            break;
        }
        static_assert(sizeof(sid_uplink_token_t) == sizeof(uint16_t));
        const sid_uplink_token_t uplink_token_big_endian = SID_STM32_UTIL_SWAP_BYTES_16(uplink_token); /* Token should be sent as Big Endian value */

        /* Calculate the number of fragments */
        const uint32_t fragment_mtu = mtu - SID_STM32_UTIL_STRUCT_MEMBER_SIZE(sid_uplink_message_t, header); /* This is how many bytes of user data we can put into a single fragment */
        const uint32_t total_fragments =  (data_size + (fragment_mtu - 1u)) / fragment_mtu; /* Use math ceiling */
        SID_PAL_LOG_INFO("Sending %u bytes of app data in %u fragment(s) with token 0x%04X. Sidewalk MTU: %u", data_size, total_fragments, uplink_token, mtu);

        const uint8_t * data_ptr = data;
        size_t remaining_data_bytes = data_size;
        for (uint32_t i = 0u; i < total_fragments; i++)
        {
            uint8_t                sendout_buf[SID_MAX_MTU];
            size_t                 sendout_len = SID_STM32_UTIL_STRUCT_MEMBER_SIZE(sid_uplink_message_t, header);
            sid_uplink_message_t * uplink_msg = (sid_uplink_message_t *)(void *)sendout_buf;

            /* Populate uplink message header */
            #pragma GCC diagnostic push
            #pragma GCC diagnostic ignored "-Warray-bounds"
            uplink_msg->header.token            = uplink_token_big_endian;
            uplink_msg->header.total_fragments  = (uint8_t)total_fragments;
            uplink_msg->header.current_fragment = (uint8_t)(i + 1u);

            /* Copy portion of the app data */
            size_t bytes_to_copy = (remaining_data_bytes > fragment_mtu) ? fragment_mtu : remaining_data_bytes;
            SID_STM32_UTIL_fast_memcpy(&uplink_msg->tlv_payload.raw[0], data_ptr, bytes_to_copy);
            #pragma GCC diagnostic pop

            /* Adjust pointers and counters */
            remaining_data_bytes -= bytes_to_copy;
            data_ptr             += bytes_to_copy;
            sendout_len          += bytes_to_copy;

            /* Prepare Sidewalk message descriptor */
            struct sid_msg msg = {
                .data = sendout_buf,
                .size = sendout_len,
            };
            struct sid_msg_desc desc = {
                .type = SID_MSG_TYPE_NOTIFY,
                .link_type = SID_LINK_TYPE_ANY,
                .link_mode = SID_LINK_MODE_CLOUD,
                /**
                 * Note: Whenever Sidewalk and LBM run concurrently it is highly recommended to send messages with acknowledgments
                 *       because the chances of data loss are significantly higher in this mode, especially when GNSS scan runs
                 *       concurrently as the radio is completely blocked during the scan. Selection of the message delivery attempts
                 *       and TTL is up to the end user and depends on the application and importance of the data. Sidewalk stack
                 *       will send out the message every (ttl_in_seconds / num_retries) seconds until either ACK is received or TTL
                 *       expires
                 */
                .msg_desc_attr = {
                    .tx_attr = {
                        .request_ack     = true,
                        .num_retries     = GEOLOCATION_DEMO_UPLINK_RETRIES,
                        .additional_attr = SID_MSG_DESC_TX_ADDITIONAL_ATTRIBUTES_NONE,
                        .ttl_in_seconds  = GEOLOCATION_DEMO_UPLINK_TTL,
                    },
                },
            };

            /* Adjust link mode for smartphone connections (if used) */
            if ((app_context->link_status.link_mask & SID_LINK_TYPE_1) &&
                (app_context->link_status.supported_link_mode[SID_LINK_TYPE_1_IDX] & SID_LINK_MODE_MOBILE))
            {
                desc.link_mode = SID_LINK_MODE_MOBILE;
            }

            /* Push the message to the Sidewalk stack queue */
            ret = sid_put_msg(app_context->sidewalk_handle, &msg, &desc);
            if (ret != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed queueing Sidewalk uplink fragment %u out of %u for token 0x%04X. Error: %d", (i + 1u), total_fragments, uplink_token, (int32_t)ret);
                break;
            }

            SID_PAL_LOG_INFO("Queued Sidewalk uplink fragment %u out of %u for token 0x%04X, message ID: %u", (i + 1u), total_fragments, uplink_token, desc.id);
        }

        /* Terminate if the for loop above failed */
        if (ret != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        ret = SID_ERROR_NONE;
    } while (0);

#if CFG_LED_SUPPORTED
    if (SID_ERROR_NONE == ret)
    {
        (void)led_indication_set(LED_INDICATE_SEND_ENQUEUED);
    }
    else
    {
        (void)led_indication_set(LED_INDICATE_SEND_ERROR);
    }
#endif /* CFG_LED_SUPPORTED */

    return ret;
}

/*----------------------------------------------------------------------------*/

#if GEOLOCATION_DEMO_USE_WIFI
static sid_error_t serialize_wifi_scan_data(const smtc_modem_wifi_event_data_scan_done_t * const scan_data, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len)
{
    sid_error_t ret;

    /**
     * Serialized payload structure:
     * vvv Universal TLV Header vvvvvvvvvvvv
     * Byte  0: tag (WiFi Scan Data)
     * Byte  1: payload length (excluding Tag and Payload Length bytes)
     * ^^^ End of Universal TLV Header ^^^^^
     * vvv Scan Data (Payload) vvvvvvvvvvvvv
     * Bytes 2-7: BSSID of the first discovered WiFi network (6 bytes)
     * Byte  8: RSSI of the first discovered WiFi network (1 byte, signed)
     * 
     * Bytes 9-14: BSSID of the second discovered WiFi network
     * Byte  15: RSSI of the second discovered WiFi network
     * ...
     * Bytes n..n+5: BSSID of the discovered WiFI network
     * Byte  n+6: RSSI of the discovered WiFi network
     * ^^^ End of Scan Data (Payload) ^^^^^^
     * 
     * Note: the number of the discovered WiFi networks can be calculated based on the overall payload length in TLV header and a fixed length of a single record (7 bytes)
     */
    do
    {
        sid_uplink_tlv_record_t * serialized_payload = (sid_uplink_tlv_record_t *)(void *)buffer;

        /* Validate inputs */
        if ((NULL == scan_data) || (NULL == buffer) || (0u == buffer_size) || (NULL == out_serialized_len) || (0u == scan_data->nbr_results))
        {
            ret = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Calculate the required buffer size and ensure we have enough space */
        const size_t serialized_data_length = sizeof(serialized_payload->header) + (sizeof(sid_uplink_wifi_record_t) * scan_data->nbr_results);
        if (buffer_size < serialized_data_length)
        {
            ret = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Fill-in the header */
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Warray-bounds"
        serialized_payload->header.tag = GEOLOCATION_DEMO_TLV_TAG_WIFI;
        serialized_payload->header.length = (uint8_t)(serialized_data_length - sizeof(serialized_payload->header));

        /* Populate data */
        sid_uplink_wifi_record_t * serialized_wifi_scan_record = &serialized_payload->payload.wifi_scan_results[0];
        for (uint32_t i = 0u; i < scan_data->nbr_results; i++)
        {
            SID_STM32_UTIL_fast_memcpy(serialized_wifi_scan_record->bssid, scan_data->results[i].mac_address, sizeof(serialized_wifi_scan_record->bssid));
            serialized_wifi_scan_record->rssi = scan_data->results[i].rssi;

            serialized_wifi_scan_record++;
        }
        #pragma GCC diagnostic pop

        *out_serialized_len = serialized_data_length;
        ret = SID_ERROR_NONE;
    } while (0);

    return ret;
}
#endif /* GEOLOCATION_DEMO_USE_WIFI */

/*----------------------------------------------------------------------------*/

#if GEOLOCATION_DEMO_USE_GNSS
static inline size_t get_serialized_gnss_scan_record_size(const smtc_modem_gnss_event_data_scan_desc_t * const scan_data)
{
    size_t serialized_record_size = sizeof(sid_uplink_gnss_scan_data_t) - SID_STM32_UTIL_STRUCT_MEMBER_SIZE(sid_uplink_gnss_scan_data_t, nav_msg) + scan_data->nav_size;
    return serialized_record_size;
}
#endif /* GEOLOCATION_DEMO_USE_GNSS */

/*----------------------------------------------------------------------------*/

#if GEOLOCATION_DEMO_USE_GNSS
static inline sid_uplink_gnss_scan_data_t * get_next_serialized_gnss_scan_data_ptr(const sid_uplink_gnss_scan_data_t * const current_ptr)
{
    const size_t offset = sizeof(*current_ptr) - sizeof(current_ptr->nav_msg) + current_ptr->nav_size;
    sid_uplink_gnss_scan_data_t * next_ptr = (sid_uplink_gnss_scan_data_t *)(void *)((size_t)(void *)current_ptr + offset);
    return next_ptr;
}
#endif /* GEOLOCATION_DEMO_USE_GNSS */

/*----------------------------------------------------------------------------*/

#if GEOLOCATION_DEMO_USE_GNSS
static sid_error_t serialize_gnss_scan_data(const smtc_modem_gnss_event_data_scan_done_t * const scan_data, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len)
{
    sid_error_t ret;

    /**
     * Serialized payload structure:
     * vvv Universal TLV Header vvvvvvvvvvvv
     * Byte  0: tag (GNSS Scan Data with Assist Position or Without Assist Position)
     * Byte  1: payload length (excluding Tag and Payload Length bytes)
     * ^^^ End of Universal TLV Header ^^^^^
     * vvvvv Optional data for AP mode vvvvv
     * Bytes 2-3: assist position latitude (Big Endian)
     * Bytes 4-5: assist position longitude (Big Endian)
     * ^^ End of Optional data for AP mode ^
     * vvvvv Common data for all modes vvvvv
     * Bytes 6-7 (2-3): time accuracy in Big Endian
     * ^^ End of Common data for all modes ^
     * vvv Scan Data (Payload) vvvvvvvvvvvvv
     * Bytes 8-11 (4-7): GNSS scan timestamp (Big Endian)
     * Byte  12 (8): number of bytes in NAV3 message
     * Bytes 13 (9)..n: NAV3 message
     * ...
     * Bytes n+1..n+4: GNSS scan timestamp
     * Byte  n+5: number of bytes in NAV3 message
     * Bytes n+6..n+m: NAV3 message
     * ...
     * ^^^ End of Scan Data (Payload) ^^^^^^
     * 
     * Note: the number of the available records can be calculated by traversing the serialized payload and inspecting the length of each individual scan record
     */
    do
    {
        sid_uplink_tlv_record_t *                serialized_payload = (sid_uplink_tlv_record_t *)(void *)buffer;
        lr11xx_gnss_solver_assistance_position_t assist_pos;
        bool                                     assist_pos_valid = false;

        /* Validate inputs */
        if ((NULL == scan_data) || (NULL == buffer) || (0u == buffer_size) || (NULL == out_serialized_len) || (0u == scan_data->nb_scans_valid))
        {
            ret = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Check if assist position is available */
        for (uint32_t i = 0u; i < scan_data->nb_scans_valid; i++)
        {
            if (LR11XX_GNSS_LAST_SCAN_MODE_ASSISTED == scan_data->scans[i].scan_mode_launched)
            {
                assist_pos = scan_data->scans[i].aiding_position;
                assist_pos_valid = true;
                break;
            }
        }

        /* Calculate the required buffer size and ensure we have enough space */
        size_t serialized_data_length = sizeof(serialized_payload->header) + (sizeof(sid_uplink_gnss_no_ap_record_t) - SID_STM32_UTIL_STRUCT_MEMBER_SIZE(sid_uplink_gnss_no_ap_record_t, gnss_scan_results));
        if (assist_pos_valid != false)
        {
            /* Ad overhead to store assist position */
            serialized_data_length += sizeof(sid_uplink_gnss_ap_record_t) - sizeof(sid_uplink_gnss_no_ap_record_t);
        }
        for (uint32_t i = 0u; i < scan_data->nb_scans_valid; i++)
        {
            serialized_data_length += get_serialized_gnss_scan_record_size(&scan_data->scans[i]);
        }

        if (buffer_size < serialized_data_length)
        {
            SID_PAL_LOG_ERROR("Failed to serialize GNSS scan data. Anticipated serialized data length: %u, available buffer space: %u", serialized_data_length, buffer_size);
            ret = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Fill-in the header */
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Warray-bounds"
        serialized_payload->header.tag = (false == assist_pos_valid) ? GEOLOCATION_DEMO_TLV_TAG_GNSS_NO_AP : GEOLOCATION_DEMO_TLV_TAG_GNSS_AP;
        serialized_payload->header.length = (uint8_t)(serialized_data_length - sizeof(serialized_payload->header));

        /* Populate common data */
        sid_uplink_gnss_scan_data_t * serialized_gnss_scan_data;
        if (false == assist_pos_valid)
        {
            sid_uplink_gnss_no_ap_record_t * gnss_payload = &serialized_payload->payload.gnss_no_ap_scan_results;
            uint32_t time_accuracy = scan_data->time_accuracy * 4096u / 1000u;

            /* Convert us to 0.244 ms units */
            time_accuracy = scan_data->time_accuracy * 4096u / 1000u;
            if (time_accuracy > 0u)
            {
                /* Offset by 1 tick so that UINT16_MAX will equal to 16 seconds after reverse conversion */
                time_accuracy--;
            }

            gnss_payload->time_accuracy = SID_STM32_UTIL_SWAP_BYTES_16(time_accuracy > UINT16_MAX ? UINT16_MAX : (uint16_t)time_accuracy);
            serialized_gnss_scan_data   = &gnss_payload->gnss_scan_results[0];
        }
        else
        {
            sid_uplink_gnss_ap_record_t * gnss_payload = &serialized_payload->payload.gnss_ap_scan_results;

            gnss_payload->ap_lat                       = SID_STM32_UTIL_SWAP_BYTES_16((int16_t)(assist_pos.latitude  * 2048.f / 90.f));
            gnss_payload->ap_long                      = SID_STM32_UTIL_SWAP_BYTES_16((int16_t)(assist_pos.longitude * 2048.f / 180.f));
            gnss_payload->gnss_scan_data.time_accuracy = SID_STM32_UTIL_SWAP_BYTES_16(scan_data->time_accuracy > UINT16_MAX ? UINT16_MAX : (uint16_t)scan_data->time_accuracy);
            serialized_gnss_scan_data                  = &gnss_payload->gnss_scan_data.gnss_scan_results[0];
        }
        #pragma GCC diagnostic pop

        /* Serialize individual scan results */
        for (uint32_t i = 0u; i < scan_data->nb_scans_valid; i++)
        {
            SID_STM32_UTIL_fast_memcpy(serialized_gnss_scan_data->nav_msg, scan_data->scans[i].nav, scan_data->scans[i].nav_size);
            serialized_gnss_scan_data->nav_size = scan_data->scans[i].nav_size;
            serialized_gnss_scan_data->timestamp = SID_STM32_UTIL_SWAP_BYTES_32(scan_data->scans[i].timestamp); /* Store timestamp in Big Endian format */

            serialized_gnss_scan_data = get_next_serialized_gnss_scan_data_ptr(serialized_gnss_scan_data);
        }

        *out_serialized_len = serialized_data_length;
        ret = SID_ERROR_NONE;
    } while (0);

    return ret;
}
#endif /* GEOLOCATION_DEMO_USE_GNSS */

/*----------------------------------------------------------------------------*/

static sid_error_t serialize_demo_counter_data(const uint8_t counter_value, uint8_t * buffer, const size_t buffer_size, size_t * const out_serialized_len)
{
    sid_error_t ret;

    /**
     * Serialized payload structure:
     * vvv Universal TLV Header vvvvvvvvvvvv
     * Byte  0: tag (Demo Counter)
     * Byte  1: payload length (excluding Tag and Payload Length bytes)
     * ^^^ End of Universal TLV Header ^^^^^
     * vvv Scan Data (Payload) vvvvvvvvvvvvv
     * Byte 2: Counter value reported by the application
     * ^^^ End of Scan Data (Payload) ^^^^^^
     * 
     * Note: the number of the discovered WiFi networks can be calculated based on the overall payload length in TLV header and a fixed length of a single record (7 bytes)
     */
    do
    {
        sid_uplink_tlv_record_t * serialized_payload = (sid_uplink_tlv_record_t *)(void *)buffer;

        /* Validate inputs */
        if ((NULL == buffer) || (0u == buffer_size) || (NULL == out_serialized_len))
        {
            ret = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Calculate the required buffer size and ensure we have enough space */
        const size_t serialized_data_length = sizeof(serialized_payload->header) + sizeof(sid_uplink_demo_counter_record_t);
        if (buffer_size < serialized_data_length)
        {
            ret = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* Fill-in the header */
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Warray-bounds"
        serialized_payload->header.tag = GEOLOCATION_DEMO_TLV_TAG_COUNTER;
        serialized_payload->header.length = (uint8_t)(serialized_data_length - sizeof(serialized_payload->header));

        /* Populate data */
        sid_uplink_demo_counter_record_t * const serialized_demo_counter_record = &serialized_payload->payload.demo_counter;
        serialized_demo_counter_record->counter_value = counter_value;
        #pragma GCC diagnostic pop

        *out_serialized_len = serialized_data_length;
        ret = SID_ERROR_NONE;
    } while (0);

    return ret;
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
        sid_hal_reset(SID_HAL_RESET_NORMAL);
    }
    else
    {
        SID_PAL_LOG_DEBUG("Wait for SID API to notify to proceed with factory reset!");
    }
}

/*----------------------------------------------------------------------------*/

static void print_subghz_link_profile(const app_context_t * const context)
{
    sid_error_t ret;
    struct sid_device_profile dev_cfg;

    do
    {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        dev_cfg.unicast_params.device_profile_id = SID_LINK3_PROFILE_A;
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        dev_cfg.unicast_params.device_profile_id = SID_LINK2_PROFILE_2;
#else
        ret = SID_ERROR_NOSUPPORT;
        break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

        ret = sid_option(context->sidewalk_handle, SID_OPTION_900MHZ_GET_DEVICE_PROFILE, &dev_cfg, sizeof(dev_cfg));
        if (ret != SID_ERROR_NONE)
        {
            break;
        }

        SID_PAL_LOG_INFO("Current Sidewalk SubGHz mode: %s, rx_wndw_cnt: %u, wkup_type: %s, rx_interv: %ums",
                         sid_subghz_profile_code_to_str(dev_cfg.unicast_params.device_profile_id),
                         (uint32_t)dev_cfg.unicast_params.rx_window_count,
                         dev_cfg.unicast_params.wakeup_type < SID_STM32_UTIL_ARRAY_SIZE(sid_subghz_wakeup_type_strings) ? sid_subghz_wakeup_type_strings[dev_cfg.unicast_params.wakeup_type] : "Unknown",
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
                         (uint32_t)dev_cfg.unicast_params.unicast_window_interval.async_rx_interval_ms
#else
                         (uint32_t)dev_cfg.unicast_params.unicast_window_interval.sync_rx_interval_ms
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
        );

        ret = SID_ERROR_NONE;
    } while (0);

    if (ret != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to get Sidewalk SubGHz link profile info. Error %d", (int32_t)ret);
    }
}

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
static void adjust_sidewalk_link_log_level(const uint32_t link_mask)
{
    if (SID_PAL_LOG_LEVEL > SID_PAL_LOG_SEVERITY_INFO)
    {
        struct sid_log_control_severity sid_log_settings;
        sid_log_control_get_severity(&sid_log_settings);
        if ((link_mask & SID_LINK_TYPE_2) != 0u)
        {
            sid_log_settings.level = SID_PAL_LOG_SEVERITY_INFO;
        }
        else
        {
            sid_log_settings.level = SID_PAL_LOG_LEVEL;
        }
        sid_log_control_set_severity(&sid_log_settings);
    }
}
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

/*----------------------------------------------------------------------------*/

static sid_error_t init_link_and_check_sid_registration(app_context_t * const context, struct sid_config * const config)
{
    sid_error_t ret = SID_ERROR_GENERIC;
    struct sid_handle *sid_handle = NULL;
    struct sid_status sid_status;

    do
    {
        /* Validate inputs */
        if (NULL == context)
        {
            SID_PAL_LOG_ERROR("Sidewalk context cannot be null");
            ret = SID_ERROR_NULL_POINTER;
            break;
        }

        if (NULL == config)
        {
            SID_PAL_LOG_ERROR("Sidewalk config cannot be null");
            ret = SID_ERROR_NULL_POINTER;
            break;
        }

        /* Make sure Sidewalk is not initialized already */
        if (context->sidewalk_handle != NULL)
        {
            SID_PAL_LOG_ERROR("Sidewalk is initialized already");
            ret = SID_ERROR_ALREADY_INITIALIZED;
            break;
        }

        /* Initialize state indication to the user app */
        context->state = STATE_SIDEWALK_NOT_READY;

#if SID_FSK_ONLY_LINK
        /* For FSK-only mode start with FSK mode and handle the device registration over the FSK link */
        config->link_mask = SID_COMM_LINK_TYPE;
#else
        /* Initialize Sidewalk in BLE or FSK mode to check device registration status */
        /* WARNING: starting in LoRa-only mode is not allowed if the device is not registered. Calling sid_init() will always return an error */
        config->link_mask = SID_REGISTRATION_LINK_TYPE;
#endif /* SID_FSK_ONLY_LINK */
        ret = sid_init(config, &sid_handle);
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to initialize Sidewalk stack, link_mask:%x, err:%d", (int32_t)config->link_mask, (int32_t)ret);
            break;
        }

        /* Register sidewalk handle to the application context */
        context->sidewalk_handle = sid_handle;

        ret = sid_get_status(sid_handle, &sid_status);
        if (ret != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to get Sidewalk stack status, err:%d", (int32_t)ret);
            break;
        }

        SID_PAL_LOG_INFO("Sidewalk registration status: %s", sid_status.detail.registration_status == SID_STATUS_REGISTERED ? "Registered" : "Not registered");

#if (SID_SDK_CONFIG_ENABLE_LINK_TYPE_2)
        /* Reduce log level for FSK connection since it produces excessive logs that affect communication timings */
        adjust_sidewalk_link_log_level(config->link_mask);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

        if (SID_STATUS_REGISTERED == sid_status.detail.registration_status)
        {
            /* Indicate to the app that Sidewalk device registration is completed */
            queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_REGISTRATION_COMPLETED);
        }
        else
        {
            /* Indicate registration process is pending */
            registration_pending = true;

            /* Start the Sidewalk stack with the selected link type for registration */
            ret = sid_start(sid_handle, config->link_mask);
            if (ret != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to start Sidewalk stack, link_mask:%x, err:%d", (int32_t)config->link_mask, (int32_t)ret);
                break;
            }
        }

        /* Done */
        ret = SID_ERROR_NONE;
    } while (0);

    if (ret != SID_ERROR_NONE)
    {
        context->sidewalk_handle = NULL;
        config->link_mask = 0;
    }

    return ret;
}

/*----------------------------------------------------------------------------*/

static sid_error_t set_device_profile(app_context_t * const context, struct sid_device_profile * const set_dp_cfg)
{
    sid_error_t ret;
    struct sid_device_profile dev_cfg;

    do
    {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        dev_cfg.unicast_params.device_profile_id = SID_LINK3_PROFILE_A;
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        dev_cfg.unicast_params.device_profile_id = SID_LINK2_PROFILE_2;
#else
        ret = SID_ERROR_NOSUPPORT;
        break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

        ret = sid_option(context->sidewalk_handle, SID_OPTION_900MHZ_GET_DEVICE_PROFILE, &dev_cfg, sizeof(dev_cfg));
        if (ret != SID_ERROR_NONE)
        {
            break;
        }

        if ((set_dp_cfg->unicast_params.device_profile_id != dev_cfg.unicast_params.device_profile_id)
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
         || (set_dp_cfg->unicast_params.unicast_window_interval.async_rx_interval_ms != dev_cfg.unicast_params.unicast_window_interval.async_rx_interval_ms)
#else
         || (set_dp_cfg->unicast_params.unicast_window_interval.sync_rx_interval_ms != dev_cfg.unicast_params.unicast_window_interval.sync_rx_interval_ms)
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
         || (set_dp_cfg->unicast_params.rx_window_count   != dev_cfg.unicast_params.rx_window_count))
        {
            ret = sid_option(context->sidewalk_handle, SID_OPTION_900MHZ_SET_DEVICE_PROFILE, set_dp_cfg, sizeof(dev_cfg));
        }
        else
        {
            ret = SID_ERROR_NONE;
            SID_PAL_LOG_INFO("Sidewalk SubGHz mode is already set to the desired configuration");
        }
    } while (0);

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
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 || SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        /**
         * FIXME: sid_pal_radio_deinit wasn't called by sid_deinit function
         *        It's a Sidewalk SDK issue.
         *        workaround is to call sid_pal_radio_deinit explicitly
         */
        ret = sid_pal_radio_deinit();
        if(ret != SID_ERROR_NONE)
        {
            break;
        }
#endif
        app_context->sidewalk_handle = NULL;
#if CFG_LED_SUPPORTED
        (void)led_indication_set(LED_INDICATE_IDLE);
#endif
    } while (0);

    return ret;
}

/*----------------------------------------------------------------------------*/

static void lbm_process_scheduler_timer_cb(void * arg, sid_pal_timer_t * originator)
{
    (void)arg;
    (void)originator;

    queue_event(g_event_queue, EVENT_TYPE_SMTC_LBM_PROCESS);
}

/*----------------------------------------------------------------------------*/

static void schedule_smtc_modem_engine_run(const uint32_t delay_ms)
{
    sid_pal_enter_critical_region();

    /* Ensure any previously scheduled execution is cancelled */
    (void)sid_pal_timer_cancel(&lbm_process_scheduler_timer);

    /* Schedule new modem engine run */
    if (0u == delay_ms)
    {
        /* Add processing request to the event queue immediately */
        queue_event(g_event_queue, EVENT_TYPE_SMTC_LBM_PROCESS);
    }
    else
    {
        struct sid_timespec engine_run_ts;
        sid_error_t err;

        /* Postpone processing for the requested duration */
        err = sid_pal_uptime_now(&engine_run_ts);
        SID_PAL_ASSERT(SID_ERROR_NONE == err);
        sid_add_ms_to_timespec(&engine_run_ts, delay_ms);
        err = sid_pal_timer_arm(&lbm_process_scheduler_timer, SID_PAL_TIMER_PRIO_CLASS_PRECISE, &engine_run_ts, NULL);
        SID_PAL_ASSERT(SID_ERROR_NONE == err);
    }

    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

static void smtc_lbm_event_callback(void)
{
    smtc_modem_return_code_t                                    lbm_err;
    smtc_modem_event_t                                          lbm_current_event;
    uint8_t                                                     lbm_pending_event_count;
#if GEOLOCATION_DEMO_USE_GNSS
    smtc_modem_almanac_demodulation_event_data_almanac_update_t almanac_update_data;
#endif /* GEOLOCATION_DEMO_USE_GNSS */

    /* Retrieve LBM events until all of them are processed */
    do
    {
        /* Read modem event */
        lbm_err = smtc_modem_get_event(&lbm_current_event, &lbm_pending_event_count);
        if (lbm_err != SMTC_MODEM_RC_OK)
        {
            SID_PAL_LOG_ERROR("LBM: Failed to fetch active event. Error %u", (uint32_t)lbm_err);
            break;
        }

        switch (lbm_current_event.event_type)
        {
            case SMTC_MODEM_EVENT_RESET:
                SID_PAL_LOG_DEBUG("LBM: Event received: RESET");

                do
                {
#if GEOLOCATION_DEMO_USE_GNSS
                    /* Configure almanac demodulation service */
                    lbm_err = smtc_modem_almanac_demodulation_set_constellations(GEOLOCATION_DEMO_LBM_STACK_ID, GEOLOCATION_DEMO_GNSS_CONSTELLATIONS_TO_USE);
                    if (lbm_err != SMTC_MODEM_RC_OK)
                    {
                        SID_PAL_LOG_ERROR("Failed to configure constellations for almanac demodulation. Error %u", (uint32_t)lbm_err);
                        break;
                    }

                    /* Configure GNSS scan */
                    lbm_err = smtc_modem_gnss_send_mode(GEOLOCATION_DEMO_LBM_STACK_ID, SMTC_MODEM_SEND_MODE_BYPASS);
                    if (lbm_err != SMTC_MODEM_RC_OK)
                    {
                        SID_PAL_LOG_ERROR("Failed to configure GNSS scan result send mode. Error %u", (uint32_t)lbm_err);
                        break;
                    }

                    lbm_err = smtc_modem_gnss_set_constellations(GEOLOCATION_DEMO_LBM_STACK_ID, GEOLOCATION_DEMO_GNSS_CONSTELLATIONS_TO_USE);
                    if (lbm_err != SMTC_MODEM_RC_OK)
                    {
                        SID_PAL_LOG_ERROR("Failed to configure constellations for GNSS scan. Error %u", (uint32_t)lbm_err);
                        break;
                    }
#endif /* GEOLOCATION_DEMO_USE_GNSS */

#if GEOLOCATION_DEMO_USE_WIFI
                    /* Configure Wi-Fi scan */
                    lbm_err = smtc_modem_wifi_send_mode(GEOLOCATION_DEMO_LBM_STACK_ID, SMTC_MODEM_SEND_MODE_BYPASS);
                    if (lbm_err != SMTC_MODEM_RC_OK)
                    {
                        SID_PAL_LOG_ERROR("Failed to configure WiFi scan result send mode. Error %u", (uint32_t)lbm_err);
                        break;
                    }
#endif /* GEOLOCATION_DEMO_USE_WIFI */

                    /* Put the radio into sleep mode - can't use sid_pal_radio_sleep() here due to LBM bridge being active */
                    lbm_err = smtc_modem_ext_sleep(0u);
                    if (lbm_err != SMTC_MODEM_RC_OK)
                    {
                        SID_PAL_LOG_ERROR("Failed to put radio to sleep via LBM. Error %u", (uint32_t)lbm_err);
                        break;
                    }

                    /* Done */
                    lbm_err = SMTC_MODEM_RC_OK;
                } while (0);

                /* Notify LBM is initialized now */
                lbm_init_error = lbm_err;
                (void)osSemaphoreRelease(lbm_ready_semaphore);
                break;

#if GEOLOCATION_DEMO_USE_GNSS
            case SMTC_MODEM_EVENT_GNSS_SCAN_DONE:
                /* Nothing to do here */
                SID_PAL_LOG_DEBUG("LBM: Event received: GNSS_SCAN_DONE");
                break;

            case SMTC_MODEM_EVENT_GNSS_TERMINATED:
                SID_PAL_LOG_DEBUG("LBM: Event received: GNSS_TERMINATED");
                lbm_err = smtc_modem_gnss_get_event_data_scan_done(GEOLOCATION_DEMO_LBM_STACK_ID, &lbm_gnss_scan_done_data);
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    SID_PAL_LOG_ERROR("LBM: Failed to get GNSS scan results from LBM. Error %u", (uint32_t)lbm_err);
                    /* Don't terminate here, we may need to relaunch GNSS scan */
                }
                else
                {
                    SID_PAL_LOG_INFO("LBM: GNSS scan finished, number of valid NAV messages: %u ", lbm_gnss_scan_done_data.nb_scans_valid);
                    if (lbm_gnss_scan_done_data.indoor_detected != false)
                    {
                        SID_PAL_LOG_WARNING("LBM: the device is possibly located indoors, GNSS signals are obstructed");
                    }
                }

                if ((lbm_err != SMTC_MODEM_RC_OK) || (false == lbm_gnss_scan_done_data.is_valid))
                {
                    /* Unsatisfactory outcomes, let's relaunch the scan */
                    if (lbm_gnss_scan_retry_counter < GEOLOCATION_DEMO_GNSS_SCAN_RETRY_LIMIT)
                    {
                        lbm_gnss_scan_retry_counter++;

                        lbm_err = smtc_modem_gnss_scan(GEOLOCATION_DEMO_LBM_STACK_ID, GEOLOCATION_DEMO_GNSS_SCAN_MODE_TO_USE, GEOLOCATION_DEMO_GNSS_SCAN_RETRY_INTERVAL_S);
                        if (lbm_err != SMTC_MODEM_RC_OK)
                        {
                            SID_PAL_LOG_ERROR("LBM: Failed to restart GNSS scan. Error %u", (uint32_t)lbm_err);
                            /* Let the flow proceed and release lbm_gnss_scan_ready_semaphore as this situation is non-recoverable from here */
                        }
                        else
                        {
                            SID_PAL_LOG_INFO("LBM: Unsatisfactory GNSS scan results. Retry attempt #%u (out of %u) will start in %u seconds",
                                             lbm_gnss_scan_retry_counter,
                                             GEOLOCATION_DEMO_GNSS_SCAN_RETRY_LIMIT,
                                             GEOLOCATION_DEMO_GNSS_SCAN_RETRY_INTERVAL_S);
                            /* Jump out here and don't release the semaphore, we are waiting for the new GNSS scan results */
                            break;
                        }
                    }
                    else
                    {
                        SID_PAL_LOG_ERROR("LBM: GNSS scan finished unsuccessfully, retry limit reached");
                        lbm_err = SMTC_MODEM_RC_FAIL;
                    }
                }

                /* Reset the retry counter */
                lbm_gnss_scan_retry_counter = 0u;
                /* Invalidate GNSS scan data if scan was not completed successfully */
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    lbm_gnss_scan_done_data.is_valid = false;
                    lbm_gnss_scan_done_data.nb_scans_valid = 0u;
                }
                (void)osSemaphoreRelease(lbm_gnss_scan_ready_semaphore);
                break;

            case SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE:
                SID_PAL_LOG_DEBUG("LBM: Event received: GNSS_ALMANAC_DEMOD_UPDATE");
                lbm_err = smtc_modem_almanac_demodulation_get_event_data_almanac_update(GEOLOCATION_DEMO_LBM_STACK_ID, &almanac_update_data);
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    SID_PAL_LOG_ERROR("LBM: Failed to get almanac demodulation results from LBM. Error %u", (uint32_t)lbm_err);
                    break;
                }

                if (almanac_update_data.status_gps != SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_UNKNOWN)
                {
                    SID_PAL_LOG_INFO("LBM: GPS almanac update progress: %u%%", almanac_update_data.update_progress_gps);
                }
                if (almanac_update_data.status_beidou != SMTC_MODEM_GNSS_ALMANAC_UPDATE_STATUS_UNKNOWN)
                {
                    SID_PAL_LOG_INFO("LBM: Beidou almanac update progress: %u%%", almanac_update_data.update_progress_beidou);
                }
                break;
#endif /* GEOLOCATION_DEMO_USE_GNSS */

#if GEOLOCATION_DEMO_USE_WIFI
            case SMTC_MODEM_EVENT_WIFI_SCAN_DONE:
                /* Nothing to do here */
                SID_PAL_LOG_DEBUG("LBM: Event received: WIFI_SCAN_DONE");
                break;

            case SMTC_MODEM_EVENT_WIFI_TERMINATED:
                SID_PAL_LOG_DEBUG("LBM: Event received: WIFI_TERMINATED");
                lbm_err = smtc_modem_wifi_get_event_data_scan_done(GEOLOCATION_DEMO_LBM_STACK_ID, &lbm_wifi_scan_done_data);
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    SID_PAL_LOG_ERROR("LBM: Failed to get WiFi scan results from LBM. Error %u", (uint32_t)lbm_err);
                    /* Don't terminate here, we may need to relaunch WiFi scan */
                }
                else
                {
#  if GEOLOCATION_DEMO_USE_GNSS
                    if ((0u == lbm_wifi_scan_done_data.nbr_results) && (0u == lbm_wifi_scan_done_data.scan_duration_ms))
                    {
                        /* Most probably WiFi task was preempted by GNSS or almanac tasks. Don't increment retry counter, just relaunch the WiFi scan ASAP */
                        lbm_err = smtc_modem_wifi_scan(GEOLOCATION_DEMO_LBM_STACK_ID, 0u);
                        if (lbm_err != SMTC_MODEM_RC_OK)
                        {
                            SID_PAL_LOG_ERROR("LBM: Failed to restart WiFi scan. Error %u", (uint32_t)lbm_err);
                            /* Let the flow proceed and release lbm_wifi_scan_ready_semaphore as this situation is non-recoverable from here */
                        }
                        else
                        {
                            /* Wait for the new scan iteration to complete */
                            SID_PAL_LOG_INFO("LBM: WiFi scan rescheduled due to preemption");
                            break;
                        }
                    }
#  endif /* GEOLOCATION_DEMO_USE_GNSS */
                    SID_PAL_LOG_INFO("LBM: WiFi scan finished, discovered %u BSSIDs", lbm_wifi_scan_done_data.nbr_results);
                }

                if ((lbm_err != SMTC_MODEM_RC_OK) || (lbm_wifi_scan_done_data.nbr_results < GEOLOCATION_DEMO_WIFI_SCANS_MIN))
                {
                    /* Unsatisfactory outcomes, let's relaunch the scan */
                    if (lbm_wifi_scan_retry_counter < GEOLOCATION_DEMO_WIFI_SCAN_RETRY_LIMIT)
                    {
                        lbm_wifi_scan_retry_counter++;

                        lbm_err = smtc_modem_wifi_scan(GEOLOCATION_DEMO_LBM_STACK_ID, GEOLOCATION_DEMO_WIFI_SCAN_RETRY_INTERVAL_S);
                        if (lbm_err != SMTC_MODEM_RC_OK)
                        {
                            SID_PAL_LOG_ERROR("LBM: Failed to restart WiFi scan. Error %u", (uint32_t)lbm_err);
                            /* Let the flow proceed and release lbm_wifi_scan_ready_semaphore as this situation is non-recoverable from here */
                        }
                        else
                        {
                            SID_PAL_LOG_INFO("LBM: Unsatisfactory WiFi scan results. Retry attempt #%u (out of %u) will start in %u seconds",
                                             lbm_wifi_scan_retry_counter,
                                             GEOLOCATION_DEMO_WIFI_SCAN_RETRY_LIMIT,
                                             GEOLOCATION_DEMO_WIFI_SCAN_RETRY_INTERVAL_S);
                            /* Jump out here and don't release the semaphore, we are waiting for the new WiFi scan results */
                            break;
                        }
                    }
                    else
                    {
                        SID_PAL_LOG_ERROR("LBM: WiFi scan finished unsuccessfully, retry limit reached");
                        lbm_err = SMTC_MODEM_RC_FAIL;
                    }
                }

                /* Reset the retry counter */
                lbm_wifi_scan_retry_counter = 0u;
                /* Invalidate WiFi scan data if scan was not completed successfully */
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    lbm_wifi_scan_done_data.nbr_results = 0u;
                }
                (void)osSemaphoreRelease(lbm_wifi_scan_ready_semaphore);
                break;
#endif /* GEOLOCATION_DEMO_USE_WIFI */

            default:
                SID_PAL_LOG_ERROR("LBM: Unknown LBM event type (%u)", lbm_current_event.event_type);
                break;
        }
    } while (lbm_pending_event_count != 0u);
}

/*----------------------------------------------------------------------------*/

#if (CFG_BUTTON_SUPPORTED == 1)
#  if !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
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
                /* Nothing to do here */
            }
            else
            {
                /* Long-press event */
                queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_FACTORY_RESET);
            }
        }
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to read B1 state. Error %d", (int32_t)ret);
    }
}
#  endif /* STM32WBA6x */

/*----------------------------------------------------------------------------*/

#  if !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
static void button2_irq_handler(uint32_t pin, void * callback_arg)
{
    (void)pin;
    (void)callback_arg;

    queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_SET_DEVICE_PROFILE);
}
#  endif /* !STM32WBA5x */

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

#  if defined(STM32WBA5x) /* Semtech shields occupy B2 GPIO pin for BUSY signal, use extra-long B3 press instead */
            if((uEnd - uStart) >= (4u * OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS)))
            {
                /* Extra-long-press event */
                queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_SET_DEVICE_PROFILE);
            }
            else
#  endif /* STM32WBA5x */
            if((uEnd - uStart) < OS_MS_TO_TICKS(BUTTON_LONG_PRESS_MS))
            {
                /* Short-press event */
                queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_SEND_DEMO_COUNTER);
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
    SID_PAL_LOG_INFO("Sidewalk LR11xx geolocation demo started");

    app_context_t *app_context = (app_context_t *)context;
    sid_error_t ret;
    osStatus_t os_status;

    struct sid_event_callbacks event_callbacks = {
        .context = app_context,
        .on_event = on_sidewalk_event, /* Called from ISR context */
        .on_msg_received = on_sidewalk_msg_received, /* Called from sid_process() */
        .on_msg_sent = on_sidewalk_msg_sent,  /* Called from sid_process() */
        .on_send_error = on_sidewalk_send_error, /* Called from sid_process() */
        .on_status_changed = on_sidewalk_status_changed, /* Called from sid_process() */
        .on_factory_reset = on_sidewalk_factory_reset, /* Called from sid_process */
    };

    struct sid_config config = {
        .link_mask = 0,
        .callbacks = &event_callbacks,
        .link_config = app_get_sidewalk_ble_config(),
        .sub_ghz_link_config = app_get_sub_ghz_config(),
    };

    /* Performing a cold start of the Sidewalk since device registration status is unknown at this point */
    queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_INIT);

    while (1)
    {
        enum event_type event;

        if (osMessageQueueGet (app_context->event_queue, &event, 0u, osWaitForever) == osOK)
        {
            switch (event)
            {
                case EVENT_TYPE_SIDEWALK_INIT:
                    {
                        /* Performing a cold start of the Sidewalk since device registration status is unknown at this point */
                        ret = init_link_and_check_sid_registration(app_context, &config);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Failed to initialize Sidewalk. Error code: %d", (int32_t)ret);
                            goto error;
                        }
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_START_LINK:
                    {
                        /* Start selected Sidewalk link */
                        ret = sid_start(app_context->sidewalk_handle, config.link_mask);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Failed to start Sidewalk link, link_mask:%x, err:%d", (int32_t)config.link_mask, (int32_t)ret);
                            goto error;
                        }
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_STOP_LINK:
                    {
                        /* Start selected Sidewalk link */
                        ret = sid_stop(app_context->sidewalk_handle, config.link_mask);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Failed to stop Sidewalk link, link_mask:%x, err:%d", (int32_t)config.link_mask, (int32_t)ret);
                            goto error;
                        }
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_PROCESS:
                    if ((NULL == app_context) || (NULL == app_context->sidewalk_handle))
                    {
                        /*  This may happen if Sidewalk link was terminated but the event queue already contained some events - just ignore them */
                        break;
                    }
                    ret = sid_process(app_context->sidewalk_handle);
                    if ((ret != SID_ERROR_NONE) && (ret != SID_ERROR_STOPPED))
                    {
                        SID_PAL_LOG_ERROR("Error processing Sidewalk event, err:%d", (int32_t)ret);
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

                case EVENT_TYPE_SIDEWALK_SEND_DEMO_COUNTER:
                    SID_PAL_LOG_INFO("EVENT_TYPE_SIDEWALK_SEND_DEMO_COUNTER");

                    if (app_context->state == STATE_SIDEWALK_READY)
                    {
                        uint8_t serialized_data_buf[SID_STM32_UTIL_STRUCT_MEMBER_SIZE(sid_uplink_tlv_record_t, header) + sizeof(sid_uplink_demo_counter_record_t)];
                        size_t serialized_data_len;

                        SID_PAL_LOG_INFO("Sending counter update: %d", app_context->counter);
                        ret = serialize_demo_counter_data(app_context->counter, serialized_data_buf, sizeof(serialized_data_buf), &serialized_data_len);
                        if (ret != SID_ERROR_NONE)
                        {
                            break;
                        }
                        ret = send_sidewalk_uplink(app_context, serialized_data_buf, serialized_data_len);
                        if (SID_ERROR_NONE == ret)
                        {
                            app_context->counter++;
                        }
                    }
                    else
                    {
                        SID_PAL_LOG_WARNING("Can't send counter update - Sidewalk is not ready");;
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_SEND_GEOLOCATION_DATA:
                    SID_PAL_LOG_INFO("EVENT_TYPE_SIDEWALK_SEND_GEOLOCATION_DATA");

                    if (app_context->state == STATE_SIDEWALK_READY)
                    {
                        // uint8_t serialized_data_buf[SID_STM32_UTIL_STRUCT_MEMBER_SIZE(sid_uplink_tlv_record_t, header) + sizeof(sid_uplink_demo_counter_record_t)];
                        size_t serialized_data_len = 0u;
                        size_t remaining_buf_space = sizeof(geodata_serialization_buffer);
                        uint8_t * working_buf_ptr = geodata_serialization_buffer;

                        /* Run in a critical section to ensure scan data is not modified in the background */
                        sid_pal_enter_critical_region();
#if GEOLOCATION_DEMO_USE_WIFI
                        if (lbm_wifi_scan_done_data.nbr_results > 0u)
                        {
                            size_t serialized_wifi_data_len;
                            ret = serialize_wifi_scan_data(&lbm_wifi_scan_done_data, working_buf_ptr, remaining_buf_space, &serialized_wifi_data_len);
                            if (SID_ERROR_NONE == ret)
                            {
                                SID_PAL_LOG_DEBUG("Serialized %u WiFi scan result(s) into %u bytes of payload", lbm_wifi_scan_done_data.nbr_results, serialized_wifi_data_len);
                                serialized_data_len += serialized_wifi_data_len;
                                remaining_buf_space -= serialized_wifi_data_len;
                                working_buf_ptr += serialized_wifi_data_len;
                            }
                            else
                            {
                                SID_PAL_LOG_ERROR("Failed to serialize WiFi scan data, sending out WiFi data is skipped. Error %d", (int32_t)ret);
                                /* Don't terminate, we still need to leave the critical section */
                            }
                        }
                        else
                        {
                            SID_PAL_LOG_WARNING("No valid WiFi scan results available, sending out WiFi data is skipped");
                        }
#endif /* GEOLOCATION_DEMO_USE_WIFI */

#if GEOLOCATION_DEMO_USE_GNSS
                        if (lbm_gnss_scan_done_data.nb_scans_valid > 0u)
                        {
                            size_t serialized_gnss_data_len;
                            ret = serialize_gnss_scan_data(&lbm_gnss_scan_done_data, working_buf_ptr, remaining_buf_space, &serialized_gnss_data_len);
                            if (SID_ERROR_NONE == ret)
                            {
                                SID_PAL_LOG_DEBUG("Serialized %u GNSS scan result(s) into %u bytes of payload", lbm_gnss_scan_done_data.nb_scans_valid, serialized_gnss_data_len);
                                serialized_data_len += serialized_gnss_data_len;
                                remaining_buf_space -= serialized_gnss_data_len;
                                working_buf_ptr += serialized_gnss_data_len;
                            }
                            else
                            {
                                SID_PAL_LOG_ERROR("Failed to serialize GNSS scan data, sending out GNSS data is skipped. Error %d", (int32_t)ret);
                                /* Don't terminate, we still need to leave the critical section */
                            }
                        }
                        else
                        {
                            SID_PAL_LOG_WARNING("No valid GNSS scan results available, sending out GNSS data is skipped");
                        }
#endif /* GEOLOCATION_DEMO_USE_GNSS */
                        sid_pal_exit_critical_region();

                        if (0u == serialized_data_len)
                        {
                            /* Terminate if serialization failed */
                            SID_PAL_LOG_ERROR("Failed to send out geolocation scan data - no serialized payload available");
                            break;
                        }

                        SID_PAL_LOG_INFO("Sending out %u bytes of aggregated geolocation data", serialized_data_len);
                        ret = send_sidewalk_uplink(app_context, geodata_serialization_buffer, serialized_data_len);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Geolocation update sendout failed. Error %d", (int32_t)ret);
                        }
                    }
                    else
                    {
                        SID_PAL_LOG_WARNING("Can't send geolocation data - Sidewalk is not ready");;
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_SET_DEVICE_PROFILE:
                    {
                        SID_PAL_LOG_INFO("EVENT_TYPE_SIDEWALK_SET_DEVICE_PROFILE");
                        struct sid_device_profile set_dp_cfg = {};
                        struct sid_device_profile dev_cfg;

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
                        dev_cfg.unicast_params.device_profile_id = SID_LINK3_PROFILE_A;
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
                        dev_cfg.unicast_params.device_profile_id = SID_LINK2_PROFILE_2;
#else
                        SID_PAL_LOG_ERROR("SubGHz profile switch failed - unknown link mode");
                        break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

                        sid_option(app_context->sidewalk_handle, SID_OPTION_900MHZ_GET_DEVICE_PROFILE, &dev_cfg, sizeof(dev_cfg));

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
                        if (SID_LINK3_PROFILE_A == dev_cfg.unicast_params.device_profile_id)
                        {
                            set_dp_cfg.unicast_params.device_profile_id = SID_LINK3_PROFILE_B;
                            set_dp_cfg.unicast_params.rx_window_count   = SID_RX_WINDOW_CNT_INFINITE;
                            set_dp_cfg.unicast_params.wakeup_type       = SID_TX_AND_RX_WAKEUP;
                            set_dp_cfg.unicast_params.unicast_window_interval.async_rx_interval_ms = SID_LINK3_RX_WINDOW_SEPARATION_3;
                        }
                        else if (SID_LINK3_PROFILE_B == dev_cfg.unicast_params.device_profile_id)
                        {
                            set_dp_cfg.unicast_params.device_profile_id = SID_LINK3_PROFILE_A;
                            set_dp_cfg.unicast_params.rx_window_count   = SID_RX_WINDOW_CNT_2;
                            set_dp_cfg.unicast_params.wakeup_type       = SID_TX_AND_RX_WAKEUP;
                            set_dp_cfg.unicast_params.unicast_window_interval.async_rx_interval_ms = SID_LINK3_RX_WINDOW_SEPARATION_3;
                        }
                        else
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
                        if (SID_LINK2_PROFILE_1 == dev_cfg.unicast_params.device_profile_id)
                        {
                            set_dp_cfg.unicast_params.device_profile_id = SID_LINK2_PROFILE_2;
                            set_dp_cfg.unicast_params.rx_window_count   = SID_RX_WINDOW_CNT_INFINITE;
                            set_dp_cfg.unicast_params.wakeup_type       = SID_TX_AND_RX_WAKEUP;
                            set_dp_cfg.unicast_params.unicast_window_interval.sync_rx_interval_ms = SID_LINK2_RX_WINDOW_SEPARATION_2;
                        }
                        else if (SID_LINK2_PROFILE_2 == dev_cfg.unicast_params.device_profile_id)
                        {
                            set_dp_cfg.unicast_params.device_profile_id = SID_LINK2_PROFILE_1;
                            set_dp_cfg.unicast_params.rx_window_count   = SID_RX_WINDOW_CNT_INFINITE;
                            set_dp_cfg.unicast_params.wakeup_type       = SID_TX_AND_RX_WAKEUP;
                            set_dp_cfg.unicast_params.unicast_window_interval.sync_rx_interval_ms = SID_LINK2_RX_WINDOW_SEPARATION_1;
                        }
                        else
#else
                        {
                            SID_PAL_LOG_ERROR("Unsupported SubGHz link profile - %s (%u)",
                                              sid_subghz_profile_code_to_str(dev_cfg.unicast_params.device_profile_id),
                                              (uint32_t)dev_cfg.unicast_params.device_profile_id);
                            break;
                        }
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

                        SID_PAL_LOG_INFO("Changing Sidewalk SubGHz link mode: %s -> %s, rx_interval %u",
                                         sid_subghz_profile_code_to_str(dev_cfg.unicast_params.device_profile_id),
                                         sid_subghz_profile_code_to_str(set_dp_cfg.unicast_params.device_profile_id),
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
                                         (uint32_t)set_dp_cfg.unicast_params.unicast_window_interval.async_rx_interval_ms
#else
                                         (uint32_t)set_dp_cfg.unicast_params.unicast_window_interval.sync_rx_interval_ms
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */
                        );

                        ret = set_device_profile(app_context, &set_dp_cfg);
                        if (SID_ERROR_NONE == ret)
                        {
                            SID_PAL_LOG_INFO("Successfully switched Sidewalk SubGHz mode to %s",
                                             sid_subghz_profile_code_to_str(set_dp_cfg.unicast_params.device_profile_id));
                        }
                        else
                        {
                            SID_PAL_LOG_ERROR("Failed to switch Sidewalk SubGHz link mode to %s. Error %d",
                                              sid_subghz_profile_code_to_str(set_dp_cfg.unicast_params.device_profile_id),
                                              (int32_t)ret);
                        }
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_REGISTRATION_COMPLETED:
                    {
                        registration_pending = false;

#if SID_FSK_ONLY_LINK
                        /* Stack re-initialization is not required, we are in the FSK mode already */
                        /* Stop any running links */
                        ret = sid_stop(app_context->sidewalk_handle, config.link_mask);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Failed to stop Sidewalk link, link_mask:%x, err:%d", (int32_t)config.link_mask, (int32_t)ret);
                            goto error;
                        }
#else
                        /* Switch to SubGHz-only configuration since device registration is done */
                        ret = sid_deinit(app_context->sidewalk_handle);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Failed to deinitialize Sidewalk, link_mask:%x, err:%d", (int32_t)config.link_mask, (int32_t)ret);
                            break;
                        }

                        config.link_mask = SID_COMM_LINK_TYPE;
                        ret = sid_init(&config, &app_context->sidewalk_handle);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Failed to initialize Sidewalk link_mask:%x, err:%d", (int32_t)config.link_mask, (int32_t)ret);
                            break;
                        }
#endif /* SID_FSK_ONLY_LINK */

                        /* Indicate to the app that Sidewalk device registration is completed */
                        os_status = osSemaphoreRelease(sidewalk_registration_ready_semaphore);
                        if (os_status != osOK)
                        {
                            SID_PAL_LOG_ERROR("Failed to indicate Sidewalk registration completion. Error code : %d", (int32_t)os_status);
                            goto error;
                        }
                    }
                    break;

                case EVENT_TYPE_SIDEWALK_FACTORY_RESET:
                    factory_reset(app_context);
                    break;

#if (CFG_BUTTON_SUPPORTED != 0) && (CFG_LPM_STDBY_SUPPORTED != 0)
                case EVENT_TYPE_STANDBY:
                    if ((app_context != NULL) && (app_context->sidewalk_handle != NULL))
                    {
                        ret = destroy_link(app_context, &config);
                        if (ret != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("Error entering in standby mode: unable to terminate Sidewalk link. Error code: %d", (int32_t)ret);
                            goto error;
                        }
                    }

                    ret = sid_pal_gpio_ext_ifc_set_as_wakeup(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_IRQ_TRIGGER_RISING);
                    if (ret != SID_ERROR_NONE)
                    {
                        SID_PAL_LOG_ERROR("Error entering in standby mode: unable to configure wakeup pin, err:%d", (int32_t)ret);
                        goto error;
                    }

#  if CFG_LED_SUPPORTED
                    led_indication_set(LED_INDICATE_OFF);
#  endif /* CFG_LED_SUPPORTED */
                    UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);
                    SID_PAL_LOG_INFO("Entering Standby mode...");
                    break;
#endif /* CFG_BUTTON_SUPPORTED && CFG_LPM_STDBY_SUPPORTED */

                case EVENT_TYPE_SMTC_LBM_INIT:
                    /* Handle LoRa Basics Modem stack initialization */
                    if (false == app_context->lbm_initialized)
                    {
                        /* Ensure LBM bridge is activated in LR11xx Sidewalk driver */
                        const radio_lr11xx_lbm_bridge_state_t lbm_bridge_state = sid_pal_radio_lr11xx_get_lbm_bridge_mode();
                        if (lbm_bridge_state != RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE)
                        {
                            SID_PAL_LOG_ERROR("LoRa Basics Modem bridge should be in exclusive mode before LBM initialization");
                            goto error;
                        }

                        /* Initialize RALF layer to link it with Sidewalk radio driver */
                        const ralf_t * const board_ralf = smtc_modem_ext_ral_bsp_init();
                        if (NULL == board_ralf)
                        {
                            SID_PAL_LOG_ERROR("LBM BSP initialization failed");
                            goto error;
                        }

                        /* Initialize LBM library */
                        smtc_modem_init(&smtc_lbm_event_callback, false);
                        app_context->lbm_initialized = true;
                    }
                    else
                    {
                        /* Notify LBM is initialized already */
                        (void)osSemaphoreRelease(lbm_ready_semaphore);
                    }
                    break;

                case EVENT_TYPE_SMTC_LBM_PROCESS:
                    /* Process LoRa Basics Modem event(s) */
                    if (app_context->lbm_initialized != false)
                    {
                        uint32_t lbm_next_event_sleep_time_ms;

                        do
                        {
                            lbm_next_event_sleep_time_ms = smtc_modem_run_engine();
                        } while ((0u == lbm_next_event_sleep_time_ms) || (smtc_modem_is_irq_flag_pending() != false));

                        /* Schedule next invocation of LBM event processing */
                        schedule_smtc_modem_engine_run(lbm_next_event_sleep_time_ms);
                    }
                    break;

                default:
                    /* Ignore unknown events */
                    break;
            }
        }
    }

error:
    if (geolocation_demo_task != NULL)
    {
        (void)osThreadTerminate(geolocation_demo_task);
        geolocation_demo_task = NULL;
    }

    if (app_context->sidewalk_handle != NULL)
    {
        sid_stop(app_context->sidewalk_handle, config.link_mask);
        sid_deinit(app_context->sidewalk_handle);
        app_context->sidewalk_handle = NULL;
    }

#if CFG_LED_SUPPORTED
    (void)led_indication_set(LED_INDICATE_OFF);
#endif

    SID_PAL_LOG_INFO("Sidewalk demo terminated due to error");
    SID_PAL_LOG_FLUSH();
    Error_Handler();

    osThreadExit(); /* Normally this line is not reachable, but keep it if Error_Handler somehow returns */
}

/*----------------------------------------------------------------------------*/

static void geolocation_demo_task_entry(void *context)
{
    app_context_t *          app_context = (app_context_t *)context;
    osStatus_t               os_status;
    int32_t                  sid_radio_err;
    smtc_modem_return_code_t lbm_err;
#if ((GEOLOCATION_DEMO_FULL_LBM_SID_CONCURRENCY == 0) || (GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING))
    bool                     lbm_radio_comm_suspended = false;
#endif /* GEOLOCATION_DEMO_FULL_LBM_SID_CONCURRENCY || LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */
#if (GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING)
    bool                     gnss_scan_allow_sid_blanking = false;
#endif /* GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */

    (void)app_context; /* Not used in this demo, but may be potentially usefull if Sidewalk API is called from this thread */

    /* 1. Wait for Sidewalk device registration process to be completed - this is required to ensure Sidewalk stack won't reset the radio anymore */
    SID_PAL_LOG_INFO("Geolocation Demo: waiting for the Sidewalk registration to be completed...");
    os_status = osSemaphoreAcquire(sidewalk_registration_ready_semaphore, osWaitForever);
    if (os_status != osOK)
    {
        SID_PAL_LOG_ERROR("Geolocation Demo Thread - failed to wait for Sidewalk device registration to be completed");
        goto error;
    }
    SID_PAL_LOG_INFO("Geolocation Demo: Sidewalk device is registered, proceeding with the demo...");
    /*----------------------------------------------------------------------------*/

    /* 2. Switch from Sidewalk mode to LBM exclusive mode ------------------------*/
    sid_radio_err = sid_pal_radio_lr11xx_start_lbm_bridge_mode(RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE);
    if (sid_radio_err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Demo Counter Thread: Failed to enable LBM bridge mode in Sidewalk radio driver. Error %d", sid_radio_err);
        goto error;
    }

#if (SID_SDK_CONFIG_ENABLE_LINK_TYPE_2)
    /* For FSK link only: switch back log level to the original app setting */
    adjust_sidewalk_link_log_level(SID_LINK_TYPE_NONE);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
    /*----------------------------------------------------------------------------*/

    /* 3. Initialize LoRa Basics Modem middleware - it is crucial to perform this in LBM bridge exclusive mode because smtc_modem_init() talks to LR11xx via SPI */
    queue_event(g_event_queue, EVENT_TYPE_SMTC_LBM_INIT); /* Queue event to the communication stack task to minimize task stack size requirements for this thread */
    os_status = osSemaphoreAcquire(lbm_ready_semaphore, osWaitForever);
    if (os_status != osOK)
    {
        SID_PAL_LOG_ERROR("Geolocation Demo Thread - failed to wait for LoRa Basics Modem initialization to be completed");
        goto error;
    }
    if (lbm_init_error != SMTC_MODEM_RC_OK)
    {
        goto error;
    }
    SID_PAL_LOG_INFO("Geolocation Demo: LoRa Basics Modem middleware initialized, proceeding with the demo...");
    /*----------------------------------------------------------------------------*/

    /* 4. Establish initial Sidewalk connection ----------------------------------*/
    sid_radio_err = sid_pal_radio_lr11xx_stop_lbm_bridge_mode();
    if (sid_radio_err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Demo Counter Thread: Failed to disable LBM bridge mode in Sidewalk radio driver. Error %d", sid_radio_err);
        goto error;
    }

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    /* For FSK link only: limit debug logging during Sidewalk FSK connection since the amount of message is large and it may affect radio timings */
    adjust_sidewalk_link_log_level(SID_COMM_LINK_TYPE);
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

    /* Start the Sidewalk stack with the selected link type */
    queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_START_LINK);
    /*----------------------------------------------------------------------------*/

    /* 5. Wait for Sidewalk connection readiness ---------------------------------*/
    SID_PAL_LOG_INFO("Geolocation Demo: waiting for the Sidewalk connection...");
    os_status = osSemaphoreAcquire(sidewalk_connection_ready_semaphore, osWaitForever);
    if (os_status != osOK)
    {
        SID_PAL_LOG_ERROR("Geolocation Demo: unable to establish Sidewalk connection. Error code: %d", os_status);
        goto error;
    }

    /* Send a test message to verify the device and Sidewalk link are both ok. Getting the geolocation data, especially from cold start, can take 20-30 minutes or even longer */
    SID_PAL_LOG_INFO("Geolocation Demo: send Demo Counter value to test Sidewalk link...");
    queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_SEND_DEMO_COUNTER);
    os_status = osSemaphoreAcquire(sidewalk_message_delivered_semaphore, osWaitForever);
    if (os_status != osOK)
    {
        SID_PAL_LOG_ERROR("Geolocation Demo: unable to establish Sidewalk connection. Error code: %d", os_status);
        goto error;
    }
    SID_PAL_LOG_INFO("Geolocation Demo: Demo Counter value acknowledged by the cloud, proceeding to geolocation scan");
    /*----------------------------------------------------------------------------*/

    do
    {
#if ((GEOLOCATION_DEMO_FULL_LBM_SID_CONCURRENCY == 0) || (GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING))
        /* 6. Pause Sidewalk connection and switch back to LBM mode */
        /**
         * Note: It's perfectly fine to call sid_stop() API from here, but this requires current task to have
         *       enough stack allocated in RAM to handle everything happening inside sid_stop() call. That's
         *       why this example offloads sid_stop() to connectivity stack task - to save some RAM
         */
#  if (GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING)
        if ((lbm_gnss_scan_done_data.is_valid != false) && (lbm_gnss_scan_done_data.nb_scans_valid > 0u))
        {
            gnss_scan_allow_sid_blanking = true;
            for (uint32_t i = 0u; i < lbm_gnss_scan_done_data.nb_scans_valid; i++)
            {
                if (lbm_gnss_scan_done_data.scans[i].scan_mode_launched != LR11XX_GNSS_LAST_SCAN_MODE_ASSISTED)
                {
                    /* At least one scan was not performed in the Assisted mode - enforce LBM bridge to use exclusive mode since upcoming GNSS scan make take long to complete */
                    gnss_scan_allow_sid_blanking = false;
                    SID_PAL_LOG_WARNING("Geolocation Demo: Using exclusive LBM mode for next GNSS scan since assisted GNSS mode is not reached");
                    break;
                }
            }
        }
        else
        {
            /* No valid GNSS scan is available yet - enforce LBM bridge to use exclusive mode since upcoming GNSS scan make take long to complete */
            gnss_scan_allow_sid_blanking = false;
            SID_PAL_LOG_WARNING("Geolocation Demo: Using exclusive LBM mode for next GNSS scan since no valid position is available");
        }

        /* Stop Sidewalk link and switch into exclusive LBM bridge mode if a prolonged GNSS scan is anticipated */
        if (false == gnss_scan_allow_sid_blanking)
#  endif /* GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */
        {
            SID_PAL_LOG_INFO("Geolocation Demo: suspending Sidewalk connection...");
            queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_STOP_LINK);
            os_status = osSemaphoreAcquire(sidewalk_connection_stopped_semaphore, osWaitForever);
            if (os_status != osOK)
            {
                SID_PAL_LOG_ERROR("Geolocation Demo: unable to stop Sidewalk connection. Error code: %d", os_status);
                break;
            }

            sid_radio_err = sid_pal_radio_lr11xx_start_lbm_bridge_mode(RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE);
            if (sid_radio_err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Geolocation Demo: Failed to enable LBM bridge mode in Sidewalk radio driver. Error %d", sid_radio_err);
                break;
            }
        }
#  if (GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING)
        else
        {
            SID_PAL_LOG_INFO("Geolocation Demo: LBM bridge runs in concurrency mode, Sidewalk link remains active throughout the scan");
        }
#  endif /* GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */

        if (lbm_radio_comm_suspended != false)
        {
            /* Resume LBM's Radio Planner if it was suspended previously */
            lbm_err = smtc_modem_suspend_radio_communications(false);
            if (lbm_err != SMTC_MODEM_RC_OK)
            {
                break;
            }
            lbm_radio_comm_suspended = false;
        }

#if  SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        /* For FSK link only: switch back log level to the original app setting if Sidewalk is not going to output any FSK logs */
        if (sid_pal_radio_lr11xx_get_lbm_bridge_mode() == RADIO_LR11XX_LBM_BRIDGE_STATE_EXCLUSIVE_MODE)
        {
            adjust_sidewalk_link_log_level(SID_LINK_TYPE_NONE);
        }
#  endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
        /*----------------------------------------------------------------------------*/
#endif /* GEOLOCATION_DEMO_FULL_LBM_SID_CONCURRENCY || LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */

        /* 7. Run GNSS scan and almanac updates --------------------------------------*/
        SID_PAL_LOG_INFO("Geolocation Demo: Launching geolocation scans...");

#if GEOLOCATION_DEMO_USE_GNSS
        /* Initiate GNSS scan and almanac demodulation */
        if (false == lbm_almanac_demod_started)
        {
            /* Almanac demodulation is a persistent LBM task, it should be started only once */
            lbm_err = smtc_modem_almanac_demodulation_start(GEOLOCATION_DEMO_LBM_STACK_ID);
            if (lbm_err != SMTC_MODEM_RC_OK)
            {
                SID_PAL_LOG_ERROR("Geolocation Demo: Failed to start almanac demodulation. Error %u", (uint32_t)lbm_err);
                break;
            }

            lbm_almanac_demod_started = true;
        }

        lbm_err = smtc_modem_gnss_scan(GEOLOCATION_DEMO_LBM_STACK_ID, GEOLOCATION_DEMO_GNSS_SCAN_MODE_TO_USE, 0u);
        if (lbm_err != SMTC_MODEM_RC_OK)
        {
            SID_PAL_LOG_ERROR("Geolocation Demo: Failed to start GNSS scan. Error %u", (uint32_t)lbm_err);
            break;
        }
#endif /* GEOLOCATION_DEMO_USE_GNSS */

#if GEOLOCATION_DEMO_USE_WIFI
        /* Initiate WiFi scanning */
        /**
         * Note: This demo showcases the capabilities of LBM's Radio Planner to manage concurrent radio tasks (GNSS scan, almanac demodulation, WiFi scan).
         *       However, in real-life applications it may be more energy-efficient to run such tasks one by one to avoid additional wait times caused by
         *       radio task rescheduling caused by LBM's radio planning managing the priorities
         */
        lbm_err = smtc_modem_wifi_scan(GEOLOCATION_DEMO_LBM_STACK_ID, 0u);
        if (lbm_err != SMTC_MODEM_RC_OK)
        {
            SID_PAL_LOG_ERROR("Geolocation Demo: Failed to start WiFi scan. Error %u", (uint32_t)lbm_err);
            break;
        }
#endif /* #if GEOLOCATION_DEMO_USE_WIFI */

        /* Since LBM may not had any active tasks we need to force an engine run to process the above calls */
        schedule_smtc_modem_engine_run(0u);
        /*----------------------------------------------------------------------------*/

        /* 8. Wait for scan results availability -------------------------------------*/
#if GEOLOCATION_DEMO_USE_GNSS
        /* Wait for GNSS scan results first since they are more probable to come first due to GNSS task having higher priority over WiFi task in LBM */
        SID_PAL_LOG_INFO("Geolocation Demo: Waiting for GNSS scan results...");
        os_status = osSemaphoreAcquire(lbm_gnss_scan_ready_semaphore, osWaitForever);
        if (os_status != osOK)
        {
            SID_PAL_LOG_ERROR("Geolocation Demo: unable to retrieve WiFi scan results. Error code: %d", os_status);
            break;
        }
        SID_PAL_LOG_INFO("Geolocation Demo: GNSS scan results are available");
#endif /* GEOLOCATION_DEMO_USE_GNSS */

#if GEOLOCATION_DEMO_USE_WIFI
        SID_PAL_LOG_INFO("Geolocation Demo: Waiting for WiFi scan results...");
        os_status = osSemaphoreAcquire(lbm_wifi_scan_ready_semaphore, osWaitForever);
        if (os_status != osOK)
        {
            SID_PAL_LOG_ERROR("Geolocation Demo: unable to retrieve WiFi scan results. Error code: %d", os_status);
            break;
        }
        SID_PAL_LOG_INFO("Geolocation Demo: WiFi scan results are available");
#endif /* #if GEOLOCATION_DEMO_USE_WIFI */
        /*----------------------------------------------------------------------------*/

#if ((GEOLOCATION_DEMO_FULL_LBM_SID_CONCURRENCY == 0) || (GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING))
#  if (GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING)
        if (false == gnss_scan_allow_sid_blanking)
#  endif /* GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */
        {
            /* 9. Postpone all LBM activities and switch back to Sidewalk mode -----------*/
            do
            {
                /**
                 * Note: You need to call smtc_modem_suspend_radio_communications(true) if LBM runs some persistent tasks that never terminate
                 *       (e.g. almanac demodulation). If your application does not initiate any such tasks in LBM and all the other LBM tasks (e.g.
                 *       WiFi scan, GNSS scan, etc.) are finished by this point you may safely skip smtc_modem_suspend_radio_communications(true)
                 *       call here as well as smtc_modem_suspend_radio_communications(false) to resume LBM scheduler at a later point
                 */
                lbm_err = smtc_modem_suspend_radio_communications(true);
                if (lbm_err != SMTC_MODEM_RC_OK)
                {
                    /* LBM operations may not be suspended immediately (e.g. if the radio runs a scan), give it some cooldown time and retry */
                    osDelay(500u);
                }
            } while (lbm_err != SMTC_MODEM_RC_OK);
            lbm_radio_comm_suspended = true;

            /* Turn off LBM bridge mode and switch back to Sidewalk mode */
            sid_radio_err = sid_pal_radio_lr11xx_stop_lbm_bridge_mode();
            if (sid_radio_err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Demo Counter Thread: Failed to disable LBM bridge mode in Sidewalk radio driver. Error %d", sid_radio_err);
                break;
            }
            /*----------------------------------------------------------------------------*/

            /* 10. Re-establish Sidewalk connection --------------------------------------*/
            SID_PAL_LOG_INFO("Geolocation Demo: Re-establishing Sidewalk connection...");

#  if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
            /* For FSK link only: limit debug logging during Sidewalk FSK connection since the amount of message is large and it may affect radio timings */
            adjust_sidewalk_link_log_level(SID_COMM_LINK_TYPE);
#  endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

            queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_START_LINK);
            osStatus_t status = osSemaphoreAcquire(sidewalk_connection_ready_semaphore, osWaitForever);
            if (status != osOK)
            {
                SID_PAL_LOG_ERROR("Geolocation Demo: Unable to establish Sidewalk connection. Error code: %d", status);
                break;
            }
            /*----------------------------------------------------------------------------*/
        }
#  if (GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING)
        else
        {
            /* LBM bridge runs in Concurrency mode, let's just make sure Sidewalk connection is still alive before we proceed. If not, it will be re-established by SIdewalk stack automatically */
            if (app_context->state != STATE_SIDEWALK_READY)
            {
                SID_PAL_LOG_INFO("Geolocation Demo: waiting for the Sidewalk connection...");
                os_status = osSemaphoreAcquire(sidewalk_connection_ready_semaphore, osWaitForever);
                if (os_status != osOK)
                {
                    SID_PAL_LOG_ERROR("Geolocation Demo: unable to establish Sidewalk connection. Error code: %d", os_status);
                    break;
                }
            }
        }
#  endif /* GEOLOCATION_DEMO_USE_GNSS && LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */
#endif /* GEOLOCATION_DEMO_FULL_LBM_SID_CONCURRENCY || LR11XX_RADIO_CFG_GNSS_SCAN_USE_BLANKING */

        /* 11. Send out geolocation scan results via Sidewalk link -------------------*/
        if ((FALSE) /* using dummy condition to support preprocessor options below */
#if GEOLOCATION_DEMO_USE_GNSS
            || (lbm_gnss_scan_done_data.nb_scans_valid > 0u)
#endif /* GEOLOCATION_DEMO_USE_GNSS */

#if GEOLOCATION_DEMO_USE_WIFI
            || (lbm_wifi_scan_done_data.nbr_results > 0u)
#endif /* #if GEOLOCATION_DEMO_USE_WIFI */
        )
        {
            SID_PAL_LOG_INFO("Geolocation Demo: Sending out collected geolocation data");
            queue_event(g_event_queue, EVENT_TYPE_SIDEWALK_SEND_GEOLOCATION_DATA);
        }
        else
        {
            SID_PAL_LOG_INFO("Geolocation Demo: No valid geolocation scan results available, sending out geolocation data is skipped");
        }
        /*----------------------------------------------------------------------------*/

        /* 12. Idle time, do nothing till the next location update cycle -------------*/
        // TODO: for a production-ready solution it's better to expliitly check that all messages were sent rather than just waiting for a set delay
        SID_PAL_LOG_INFO("Geolocation Demo: Idling for %u seconds while Sidewalk stack processes uplinks", GEOLOCATION_DEMO_UPDATE_INTERVAL_S);
        os_status = osDelay(GEOLOCATION_DEMO_UPDATE_INTERVAL_S * 1000u);
        if (os_status != osOK)
        {
            SID_PAL_LOG_ERROR("Geolocation Demo: Failed to idle. Error code: %d", os_status);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* New scan iteration starts from top */
        SID_PAL_LOG_INFO("Geolocation Demo: New location update cycle started");
    } while (1);

error:
    SID_PAL_LOG_INFO("Geolocation Demo: Terminated due to error");
    SID_PAL_LOG_FLUSH();
    Error_Handler();

    /* Normally the lines below are not reachable, but keep them if Error_Handler somehow returns */
    geolocation_demo_task = NULL;
    osThreadExit();
}

/*----------------------------------------------------------------------------*/

#if SID_PAL_LOG_ENABLED
static const char * sid_subghz_profile_code_to_str(enum sid_device_profile_id code)
{
    const char * profile_str = NULL;

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
    assert_param(code >= SID_LINK3_PROFILE_A);
    assert_param(code < SID_LINK3_PROFILE_LAST);
#elif SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    assert_param(code >= SID_LINK2_PROFILE_1);
    assert_param(code < SID_LINK2_PROFILE_LAST);
#else
    assert_param(0); /* Invalid configuration */
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

    /*Converting from sid_profile_id to index*/
    switch (code)
    {
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
        case SID_LINK3_PROFILE_A:
            profile_str = sid_subghz_profile_strings[SID_LORA_PROFILE_A];
            break;

        case SID_LINK3_PROFILE_B:
            profile_str = sid_subghz_profile_strings[SID_LORA_PROFILE_B];
            break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_3 */

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
        case SID_LINK2_PROFILE_1:
            profile_str = sid_subghz_profile_strings[SID_FSK_PROFILE_1];
            break;

        case SID_LINK2_PROFILE_2:
            profile_str = sid_subghz_profile_strings[SID_FSK_PROFILE_2];
            break;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */

        default:
            profile_str = sid_subghz_profile_strings[SID_SUBGHZ_PROFILE_UNKNOWN];
            break;
    }

    return profile_str;
}
#endif /* SID_PAL_LOG_ENABLED */

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
    SID_PAL_LOG_INFO("LoRa Basics Modem version %u.%u.%u", LORA_BASICS_MODEM_FW_VERSION_MAJOR, LORA_BASICS_MODEM_FW_VERSION_MINOR, LORA_BASICS_MODEM_FW_VERSION_PATCH);
    SID_PAL_LOG_INFO("LR11xx driver version: %s", LR11XX_DRIVER_VERSION);
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
        .lbm_initialized = false,
    };

#if (CFG_BUTTON_SUPPORTED == 1)
#  if !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* !STM32WBA6x */

#  if !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
    ret_code = sid_pal_gpio_pull_mode(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_PULL_UP);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
    ret_code = sid_pal_gpio_set_direction(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_DIRECTION_INPUT);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* !STM32WBA5x */

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

#  if !defined(STM32WBA6x) /* Semtech shields share the same EXTI line for BUSY signal */
    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B1_GPIO_PORT, B1_PIN), SID_PAL_GPIO_IRQ_TRIGGER_EDGE, button1_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* !STM32WBA6x */

#  if !defined(STM32WBA5x) /* Semtech shields use the same GPIO pin for BUSY signal */
    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B2_GPIO_PORT, B2_PIN), SID_PAL_GPIO_IRQ_TRIGGER_EDGE, button2_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#  endif /* !STM32WBA5x */

    ret_code = sid_pal_gpio_set_irq(GPIO_PORT_PIN_TO_NUM(B3_GPIO_PORT, B3_PIN), SID_PAL_GPIO_IRQ_TRIGGER_EDGE, button3_irq_handler, NULL);
    SID_PAL_ASSERT(ret_code == SID_ERROR_NONE);
#endif

    ret_code = sid_pal_timer_init(&lbm_process_scheduler_timer, lbm_process_scheduler_timer_cb, NULL);
    if (ret_code != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("LBM engine run scheduler timer init err: %d", ret_code);
        SID_PAL_ASSERT(0);
    }

    app_context.event_queue = g_event_queue;
    app_context.main_task = osThreadNew(sidewalk_stack_task_entry, &app_context, &sidewalk_stack_task_attributes);
    if (NULL == app_context.main_task)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk processing thread. No memory");
        SID_PAL_ASSERT(0);
    }

    /* Create a semaphore to monitor LoRa Basics Modem readiness status */
    lbm_ready_semaphore = osSemaphoreNew(1u, 0u, &lbm_ready_sem_attributes);
    if (NULL == lbm_ready_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create LoRa Basics Modem Readiness Status semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

#if GEOLOCATION_DEMO_USE_WIFI
    /* Create a semaphore to monitor readiness of WiFi scan results */
    lbm_wifi_scan_ready_semaphore = osSemaphoreNew(1u, 0u, &lbm_wifi_scan_ready_sem_attributes);
    if (NULL == lbm_wifi_scan_ready_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create LoRa Basics Modem WiFi Scan Readiness semaphore. No memory");
        SID_PAL_ASSERT(0);
    }
#endif /* GEOLOCATION_DEMO_USE_WIFI */

#if GEOLOCATION_DEMO_USE_GNSS
    /* Create a semaphore to monitor readiness of GNSS scan results */
    lbm_gnss_scan_ready_semaphore = osSemaphoreNew(1u, 0u, &lbm_gnss_scan_ready_sem_attributes);
    if (NULL == lbm_gnss_scan_ready_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create LoRa Basics Modem GNSS Scan Readiness semaphore. No memory");
        SID_PAL_ASSERT(0);
    }
#endif /* GEOLOCATION_DEMO_USE_GNSS */

    /* Create a semaphore to monitor Sidewalk registration status */
    sidewalk_registration_ready_semaphore = osSemaphoreNew(1u, 0u, &sidewalk_registration_ready_sem_attributes);
    if (NULL == sidewalk_registration_ready_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk Registration Status semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

    /* Create a semaphore to monitor Sidewalk connection status */
    sidewalk_connection_ready_semaphore = osSemaphoreNew(1, 0, &sidewalk_connection_ready_sem_attributes);
    if (NULL == sidewalk_connection_ready_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk connection status semaphore. No memory");
        SID_PAL_ASSERT(0);
    }
    sidewalk_connection_stopped_semaphore = osSemaphoreNew(1, 0, &sidewalk_connection_stopped_sem_attributes);
    if (NULL == sidewalk_connection_stopped_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk connection status semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

    /* Create a semaphore to control Sidewalk message delivery */
    sidewalk_message_delivered_semaphore = osSemaphoreNew(1, 0, &sidewalk_msg_delivered_sem_attributes);
    if (NULL == sidewalk_message_delivered_semaphore)
    {
        SID_PAL_LOG_ERROR("Can't create Sidewalk Message Delivered semaphore. No memory");
        SID_PAL_ASSERT(0);
    }

   geolocation_demo_task = osThreadNew(geolocation_demo_task_entry, &app_context, &geolocation_demo_task_attributes);
   if (NULL == geolocation_demo_task)
   {
       SID_PAL_LOG_ERROR("Can't create Sidewalk demo thread. No memory");
       SID_PAL_ASSERT(0);
   }
}

/*----------------------------------------------------------------------------*/

void smtc_modem_hal_user_lbm_irq(void)
{
    /* Schedule LBM event processing ASAP */
    schedule_smtc_modem_engine_run(0u);
}
