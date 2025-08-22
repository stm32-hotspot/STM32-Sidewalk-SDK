/**
  ******************************************************************************
  * @file    sid_pal_ble_adapter_stm32wba_sidewalk_gap_gatt.c
  * @brief   Sidewalk-specific GAP and GATT handling
  * 
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

#include <stdlib.h>
#include <stdio.h>

#include "bluetooth_hci_defs.h"
#include "sid_pal_ble_adapter_stm32wba.h"
#include "sid_pal_ble_adapter_stm32wba_private_defs.h"
#include "sid_pal_ble_adapter_stm32wba_sidewalk_gap_gatt.h"
#include "sid_pal_ble_adapter_stm32wba_utils.h"

/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>

/* BLE stack */
#include <ble_core.h>
#include <ble_types.h>
#include <ll_sys_if.h>

/* Platform-specific headers */
#include "app_conf.h"
#include <cmsis_os2.h>
#include "stm32_rtos.h"

/* Utilities and helpers */
#include <sid_stm32_common_utils.h>
#include <stm32_timer.h>

/* Private defines -----------------------------------------------------------*/

#define BLE_AMA_SERVICE_CHAR_COUNT                                  (2u)
#ifndef SIDEWALK_BLE_AMA_SEND_OUT_MSG_QUEUE_LEN
#  define SIDEWALK_BLE_AMA_SEND_OUT_MSG_QUEUE_LEN                   (10u) /*!< Uplink message queue size for Sidewalk-over-BLE link */
#endif /* SIDEWALK_BLE_AMA_SEND_OUT_MSG_QUEUE_LEN */

#define SID_BLE_AMAZON_COMPANY_ID                                   (0x0171u)

#define SID_BLE_SPEC_CONN_INTERVAL_MIN                              (SID_STM32_BLE_MS_TO_CONN_INTERVAL_UNITS(30u))      /*!< The absolute minimum acceptable connection interval as per Sidewalk specification */
#define SID_BLE_SPEC_CONN_INTERVAL_MAX                              (SID_STM32_BLE_MS_TO_CONN_INTERVAL_UNITS(4000u))    /*!< The absolute maximum acceptable connection interval as per Sidewalk specification */
#define SID_BLE_REASONABLE_CONN_TIMEOUT                             (SID_STM32_BLE_MS_TO_CONN_SUPERVISION_UNITS(5000u)) /*!< Some reasonable time for BLE connection timeout. This value is used only when it exceeds the bare minimum for given connection interval and latency */
#define SID_BLE_UNACCEPTABLE_CONN_PARAMS_UPDATE_LIMIT               (5u)

/* Private macro -------------------------------------------------------------*/

#ifndef SID_BLE_SIDEWALK_EXTRA_LOGGING
/* Set SID_BLE_SIDEWALK_EXTRA_LOGGING to 1 to enable extended logs */
#  define SID_BLE_SIDEWALK_EXTRA_LOGGING (0)
#endif

#if SID_BLE_SIDEWALK_EXTRA_LOGGING
#  define SID_BLE_SIDEWALK_LOG_ERROR(...)         SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SID_BLE_SIDEWALK_LOG_WARNING(...)       SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SID_BLE_SIDEWALK_LOG_INFO(...)          SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SID_BLE_SIDEWALK_LOG_DEBUG(...)         SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SID_BLE_SIDEWALK_LOG_TRACE(...)         SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SID_BLE_SIDEWALK_LOG_ERROR(...)         ((void)0u)
#  define SID_BLE_SIDEWALK_LOG_WARNING(...)       ((void)0u)
#  define SID_BLE_SIDEWALK_LOG_INFO(...)          ((void)0u)
#  define SID_BLE_SIDEWALK_LOG_DEBUG(...)         ((void)0u)
#  define SID_BLE_SIDEWALK_LOG_TRACE(...)         ((void)0u)
#endif

/* Private typedef -----------------------------------------------------------*/

typedef struct {
    uint8_t * data;
    uint32_t  length;
} AMAMessageQueueElement_t;

/* Private variables ---------------------------------------------------------*/

sid_pal_ble_sidewalk_link_ctx_t sid_ble_sidewalk_ctx = {
    .conn_ctx                       = NULL,
    .ble_cfg                        = NULL,
    .adv_set = {
        .Advertising_Handle              = SID_STM32_BLE_SIDEWALK_VIRT_DEV_ID,
        .Duration                        = 0u,
        .Max_Extended_Advertising_Events = 0u,
    },
    .event_callbacks                = NULL,
    .advertising_state              = BADVS_ADVERTISEMENT_OFF,

    .ama_ctx = {
        .inbox_char_handle          = SID_STM32_BLE_HANDLE_INVALID_VALUE,
        .outbox_char_handle         = SID_STM32_BLE_HANDLE_INVALID_VALUE,
        .service_context = {
            .num_characteristics    = 0u,
            .characteristic_context = NULL,
            .service_handle         = SID_STM32_BLE_HANDLE_INVALID_VALUE,
        },
    },
};

/* Application-specific and ACI related resources */
static osThreadId_t       ama_sendout_task = NULL;
static osMessageQueueId_t ama_sendout_queue = NULL;

/* Private constants ---------------------------------------------------------*/

static const osThreadAttr_t ama_sendout_task_attributes = {
    .name       = "Sidewalk AMA Sendout Task",
    .priority   = TASK_PRIO_BLE_HOST + 1u, /* Keep the priority above BLE Host task so AMA won't be preempted immediately on triggering first BLE API call */
    .stack_size = RTOS_STACK_SIZE_NORMAL,
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
    .stack_mem  = TASK_DEFAULT_STACK_MEM,
};

static const osMessageQueueAttr_t ama_sendout_queue_attributes = {
    .name       =  "Sidewalk AMA Sendout Queue",
    .attr_bits  = QUEUE_DEFAULT_ATTR_BITS,
    .cb_mem     = QUEUE_DEFAULT_CB_MEM,
    .cb_size    = QUEUE_DEFAULT_CB_SIZE,
    .mq_mem     = QUEUE_DEFAULT_MQ_MEM,
    .mq_size    = QUEUE_DEFAULT_MQ_SIZE,
};

static const sid_ble_ext_conn_params_limits_t sidewalk_conn_limits ={
    .interval_min       = SID_BLE_SPEC_CONN_INTERVAL_MIN,
    .interval_max       = SID_BLE_SPEC_CONN_INTERVAL_MAX,
    .latency_max        = SID_BLE_HCI_CONN_LATENCY_UPPER_LIMIT,
    .reasonable_timeout = SID_BLE_REASONABLE_CONN_TIMEOUT,
    .ce_length_min      = SID_BLE_HCI_CONN_EVENT_LENGTH_LOWER_LIMIT,
    .ce_length_max      = SID_BLE_HCI_CONN_EVENT_LENGTH_UPPER_LIMIT,
};

/* Imported variables --------------------------------------------------------*/

extern sid_pal_ble_adapter_ctx_t sid_ble_drv_ctx;

/* Private function prototypes -----------------------------------------------*/

static inline void                  _ble_adapter_sidewalk_invalidate_ama_svc_ctx(void);
static inline SVCCTL_EvtAckStatus_t _ble_adapter_sidewalk_process_gatt_attr_modified_evt(const aci_gatt_attribute_modified_event_rp0 * const evt);
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
static inline SVCCTL_EvtAckStatus_t _ble_adapter_sidewalk_process_gap_char_read_req(const uint16_t att_handle);
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
static inline SVCCTL_EvtAckStatus_t _ble_adapter_sidewalk_process_hci_le_meta_evt(const evt_le_meta_event * const evt);
static        SVCCTL_EvtAckStatus_t _ble_adapter_sidewalk_svc_evt_handler(void *p_Event);
static        sid_error_t           _ble_adapter_sidewalk_ama_gatt_send_out(uint8_t * const data, const uint32_t length);
static        void                  _ble_adapter_sidewalk_ama_process_send_out(void* thread_input);

static        sid_error_t           _ble_adapter_sidewalk_service_init(sid_ble_adapter_sid_gatt_svc_ctx_t * const out_svc_ctx, const sid_ble_adapter_sid_gatt_svc_init_t * const ble_svc_init);
static        sid_error_t           _ble_adapter_sidewalk_add_characteristics(sid_ble_adapter_sid_gatt_svc_ctx_t * const out_svc_ctx, const sid_ble_cfg_gatt_profile_t * const service_def);
static        sid_error_t           _ble_adapter_sidewalk_services_init(const sid_ble_config_t * const sid_ble_cfg);
static        sid_error_t           _ble_adapter_sidewalk_service_deinit(sid_ble_adapter_sid_gatt_svc_ctx_t * const svc_ctx);
static        sid_error_t           _ble_adapter_sidewalk_services_deinit(void);

static        sid_error_t           _ble_adapter_sidewalk_init(const sid_ble_config_t * cfg);
static        sid_error_t           _ble_adapter_sidewalk_start_service(void);
static inline sid_error_t           _ble_adapter_sidewalk_set_adv_params(const uint32_t adv_interval);
static        sid_error_t           _ble_adapter_sidewalk_set_adv_data(uint8_t *mfg_data, uint8_t mfg_length);
static        sid_error_t           _ble_adapter_sidewalk_start_advertisement(void);
static        sid_error_t           _ble_adapter_sidewalk_stop_advertisement(void);
static        sid_error_t           _ble_adapter_sidewalk_get_rssi(int8_t * rssi);
static        sid_error_t           _ble_adapter_sidewalk_get_tx_pwr(int8_t * tx_power);
static        sid_error_t           _ble_adapter_sidewalk_send_data(sid_ble_cfg_service_identifier_t id, uint8_t * data, uint16_t length);
static        sid_error_t           _ble_adapter_sidewalk_set_callbacks(const sid_pal_ble_adapter_callbacks_t * cb);
static        sid_error_t           _ble_adapter_sidewalk_set_tx_pwr(int8_t tx_power);
static        sid_error_t           _ble_adapter_sidewalk_disconnect(void);
static        sid_error_t           _ble_adapter_sidewalk_deinit(void);

/* Private constants ---------------------------------------------------------*/

/**
 * @brief Sidewalk-specific BLE interface
 * @warning This interface shall be used by Sidewalk stack only. Using it from the user app will result in undefined behavior
 */
static const struct sid_pal_ble_adapter_interface sidwalk_ble_ifc = {
    .init          = _ble_adapter_sidewalk_init,
    .start_service = _ble_adapter_sidewalk_start_service,
    .set_adv_data  = _ble_adapter_sidewalk_set_adv_data,
    .start_adv     = _ble_adapter_sidewalk_start_advertisement,
    .stop_adv      = _ble_adapter_sidewalk_stop_advertisement,
    .get_rssi      = _ble_adapter_sidewalk_get_rssi,
    .get_tx_pwr    = _ble_adapter_sidewalk_get_tx_pwr,
    .send          = _ble_adapter_sidewalk_send_data,
    .set_callback  = _ble_adapter_sidewalk_set_callbacks,
    .set_tx_pwr    = _ble_adapter_sidewalk_set_tx_pwr,
    .disconnect    = _ble_adapter_sidewalk_disconnect,
    .deinit        = _ble_adapter_sidewalk_deinit,
};

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _ble_adapter_sidewalk_invalidate_ama_svc_ctx(void)
{
    sid_pal_enter_critical_region();

    sid_ble_sidewalk_ctx.ama_ctx.service_context.service_handle      = SID_STM32_BLE_HANDLE_INVALID_VALUE;
    sid_ble_sidewalk_ctx.ama_ctx.service_context.num_characteristics = 0u;

    sid_ble_sidewalk_ctx.ama_ctx.inbox_char_handle                   = SID_STM32_BLE_HANDLE_INVALID_VALUE;
    sid_ble_sidewalk_ctx.ama_ctx.inbox_char_index                    = 0u;
    sid_ble_sidewalk_ctx.ama_ctx.outbox_char_handle                  = SID_STM32_BLE_HANDLE_INVALID_VALUE;
    sid_ble_sidewalk_ctx.ama_ctx.outbox_char_index                   = 0u;

    if (sid_ble_sidewalk_ctx.ama_ctx.service_context.characteristic_context != NULL)
    {
        free(sid_ble_sidewalk_ctx.ama_ctx.service_context.characteristic_context);
        sid_ble_sidewalk_ctx.ama_ctx.service_context.characteristic_context = NULL;
    }

    if (sid_ble_sidewalk_ctx.ama_ctx.inbox_buf != NULL)
    {
        free(sid_ble_sidewalk_ctx.ama_ctx.inbox_buf);
        sid_ble_sidewalk_ctx.ama_ctx.inbox_buf = NULL;
    }
    sid_ble_sidewalk_ctx.ama_ctx.inbox_valid_length = 0u;
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline SVCCTL_EvtAckStatus_t _ble_adapter_sidewalk_process_gatt_attr_modified_evt(const aci_gatt_attribute_modified_event_rp0 * const evt)
{
    SVCCTL_EvtAckStatus_t evt_ack_status = SVCCTL_EvtNotAck;

    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.conn_ctx != NULL);

    do
    {
        /* Do not process access to the General Attribute Service, this is handled by the generic part of the driver */
        if (evt->Attr_Handle == (SID_STM32_BLE_GATT_SVC_SERVICE_CHANGED_CHAR_HANDLE + SID_STM32_BLE_CHARACTERISTIC_CCCD_ATTRIBUTE_OFFSET))
        {
            break;
        }

        /* AMA Inbox characteristic access */
        if (evt->Attr_Handle == (sid_ble_sidewalk_ctx.ama_ctx.inbox_char_handle + SID_STM32_BLE_CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
        {
            /* Ensure the connection comes from Sidewalk link */
            if (evt->Connection_Handle != sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id)
            {
                /* Terminate the violating connection */
                SID_PAL_LOG_WARNING("Access to Sidewalk GATT characteristic from non-Sidewalk BLE connection (0x%04X). Access denied", evt->Connection_Handle);
                aci_gap_terminate(evt->Connection_Handle, SID_BLE_HCI_STATUS_AUTH_FAILURE);
                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                break;
            }

            const size_t buffer_offset = (evt->Offset & (~SID_STM32_BLE_ACI_GATT_ATTRIBUTE_MODIFIED_MORE_DATA_PENDING_FLAG));
            const uint32_t more_data_pending = (evt->Offset & SID_STM32_BLE_ACI_GATT_ATTRIBUTE_MODIFIED_MORE_DATA_PENDING_FLAG) != 0u;

            /* Ensure there are no systematic SW failures */
            SID_PAL_ASSERT(sid_ble_sidewalk_ctx.ama_ctx.inbox_buf != NULL);

            /* Check the received data does not exceed the receiving buffer size */
            if ((buffer_offset + evt->Attr_Data_Length) > SID_STM32_BLE_ATT_LEN_LIMIT(sid_ble_drv_ctx.cfg->sidewalk_profile.max_att_mtu))
            {
                if (FALSE == more_data_pending) /* Print out the warning only once */
                {
                    SID_PAL_LOG_WARNING("AMA data received exceeds processing buffer size. Data discarded");
                }
                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                break;
            }

            SID_PAL_LOG_DEBUG("==>>AMA Inbox: received %u bytes%s", evt->Attr_Data_Length, more_data_pending ? ". More data pending" : "");
            SID_STM32_UTIL_fast_memcpy(&(sid_ble_sidewalk_ctx.ama_ctx.inbox_buf[buffer_offset]), evt->Attr_Data, evt->Attr_Data_Length);
            sid_ble_sidewalk_ctx.ama_ctx.inbox_valid_length += evt->Attr_Data_Length;

            /* Call Sidewalk stack if all the data read out from the BLE stack */
            if (more_data_pending == FALSE)
            {
                SID_PAL_LOG_DEBUG("==>>AMA Inbox: RX finished, totally received %u bytes", sid_ble_sidewalk_ctx.ama_ctx.inbox_valid_length);

                if ((sid_ble_sidewalk_ctx.event_callbacks != NULL) && (sid_ble_sidewalk_ctx.event_callbacks->data_callback != NULL))
                {
                    sid_ble_sidewalk_ctx.event_callbacks->data_callback(AMA_SERVICE, sid_ble_sidewalk_ctx.ama_ctx.inbox_buf, sid_ble_sidewalk_ctx.ama_ctx.inbox_valid_length);
                }
                else
                {
                    SID_PAL_LOG_WARNING("No callback set for Sidewalk AMA inbox. Data discarded");
                }

                /* Reset the data accumulator offset */
                sid_ble_sidewalk_ctx.ama_ctx.inbox_valid_length = 0u;
            }

            /* This event is processed, no further actions are required */
            evt_ack_status = SVCCTL_EvtAckFlowEnable;
            break;
        }
        /* AMA Outbox characteristic CCCD access */
        else if (evt->Attr_Handle == (sid_ble_sidewalk_ctx.ama_ctx.outbox_char_handle + SID_STM32_BLE_CHARACTERISTIC_CCCD_ATTRIBUTE_OFFSET))
        {
            /* Ensure the connection comes from Sidewalk link */
            if (evt->Connection_Handle != sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id)
            {
                /* Terminate the violating connection */
                SID_PAL_LOG_WARNING("Access to Sidewalk GATT characteristic from non-Sidewalk BLE connection (0x%04X). Access denied", evt->Connection_Handle);
                aci_gap_terminate(evt->Connection_Handle, SID_BLE_HCI_STATUS_AUTH_FAILURE);
                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                break;
            }

            /* Ensure the correct amount of data is written */
            if (evt->Attr_Data_Length != SID_STM32_BLE_CHARACTERISTIC_CCCD_ATTRIBUTE_SIZE)
            {
                /* Terminate the violating connection */
                SID_PAL_LOG_WARNING("Wrong CCCD data length write attempt. Connection handle 0x%04X", evt->Connection_Handle);
                aci_gap_terminate(evt->Connection_Handle, SID_BLE_HCI_STATUS_AUTH_FAILURE);
                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                break;
            }

            const uint16_t cccd_value = (uint16_t)((evt->Attr_Data[0] & 0xFFu) | ((evt->Attr_Data[1] << 8) & 0xFF00u));
            SID_PAL_LOG_DEBUG("  AMA Outbox service CCCD attribute changed to 0x%04x", cccd_value);

            sid_ble_sidewalk_ctx.ama_ctx.is_notification_enabled = (cccd_value & SID_STM32_BLE_CHARACTERISTIC_CCCD_NOTIFY_BIT_MASK) != 0u ? TRUE : FALSE;
            if (sid_ble_sidewalk_ctx.ama_ctx.is_notification_enabled != FALSE)
            {
                SID_PAL_LOG_DEBUG("==>>Remote client subscribed to AMA Outbox notifications");
            }
            else
            {
                SID_PAL_LOG_DEBUG("==>>Remote client unsubscribed from AMA Outbox notifications");
            }
            if ((sid_ble_sidewalk_ctx.event_callbacks != NULL) && (sid_ble_sidewalk_ctx.event_callbacks->notify_callback != NULL))
            {
                sid_ble_sidewalk_ctx.event_callbacks->notify_callback(AMA_SERVICE, sid_ble_sidewalk_ctx.ama_ctx.is_notification_enabled);
            }

            /* This event is processed, no further actions are required */
            evt_ack_status = SVCCTL_EvtAckFlowEnable;
        }
        else
        {
            /* Do nothing, let the other handlers to process the event */
        }
    } while (0);

    return evt_ack_status;
}

/*----------------------------------------------------------------------------*/

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
SID_STM32_SPEED_OPTIMIZED static inline SVCCTL_EvtAckStatus_t _ble_adapter_sidewalk_process_gap_char_read_req(const uint16_t att_handle)
{
    SVCCTL_EvtAckStatus_t evt_ack_status = SVCCTL_EvtNotAck;

    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.conn_ctx != NULL);

    do
    {
        /* Handle GAP Device Name characteristic access */
        if (att_handle == (sid_ble_drv_ctx.gap_dev_name_char_handle + SID_STM32_BLE_CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
        {
            tBleStatus ble_status;
            const char * const gap_device_name = sid_ble_drv_ctx.cfg->sidewalk_profile.device_name;
            uint16_t gap_device_name_len = (NULL == gap_device_name) ? 0u : strlen(gap_device_name);

            if (gap_device_name_len > SID_STM32_BLE_GAP_DEVICE_NAME_MAX_LEN)
            {
                gap_device_name_len = SID_STM32_BLE_GAP_DEVICE_NAME_MAX_LEN;
            }

            /* Update GAP device name */
            ble_status = aci_gatt_update_char_value(sid_ble_drv_ctx.gap_service_handle,
                                                    sid_ble_drv_ctx.gap_dev_name_char_handle,
                                                    0u,
                                                    gap_device_name_len,
                                                    (uint8_t *)gap_device_name);
            if (ble_status != BLE_STATUS_SUCCESS)
            {
                /* Failed to update GAP Device Name - prohibit reading since the characterstic will have wrong value */
                SID_BLE_SIDEWALK_LOG_ERROR("  Fail   : aci_gatt_update_char_value - Device Name, result: 0x%02X", ble_status);
                evt_ack_status = SVCCTL_EvtAckFlowDisable;
                break;
            }
            else
            {
                /* Successfully updated GAP Device Name characterstic with the name of the current Virtual BLE device */
                SID_BLE_SIDEWALK_LOG_DEBUG("  Success: aci_gatt_update_char_value - Device Name");
                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                break; /* Exit from here since we don't need to invoke user callback for GAP characteristics */
            }
        }

        /* Handle GAP Appearance characteristic access */
        if (att_handle == (sid_ble_drv_ctx.gap_appearance_char_handle + SID_STM32_BLE_CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
        {
            tBleStatus ble_status;
            const uint16_t gap_appearance = sid_ble_drv_ctx.cfg->sidewalk_profile.appearance;

            /* Update GAP appearance */
            ble_status = aci_gatt_update_char_value(sid_ble_drv_ctx.gap_service_handle,
                                                    sid_ble_drv_ctx.gap_appearance_char_handle,
                                                    0u,
                                                    sizeof(gap_appearance),
                                                    (uint8_t *)(void *)&gap_appearance);
            if (ble_status != BLE_STATUS_SUCCESS)
            {
                SID_BLE_SIDEWALK_LOG_ERROR("  Fail   : aci_gatt_update_char_value - Appearance, result: 0x%02X", ble_status);
                evt_ack_status = SVCCTL_EvtAckFlowDisable;
                break;
            }
            else
            {
                SID_BLE_SIDEWALK_LOG_DEBUG("  Success: aci_gatt_update_char_value - Appearance");
                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                break;
            }
        }

        /* Handle GAP Preferred Connection Parameters characteristic access */
        if (att_handle == (sid_ble_drv_ctx.gap_ppcp_char_handle + SID_STM32_BLE_CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
        {
            const sid_ble_cfg_conn_param_t * const conn_param = &sid_ble_sidewalk_ctx.ble_cfg->conn_param;

            if (conn_param != NULL)
            {
                (void)sid_stm32wba_ble_adapter_util_set_gap_ppcp_char(conn_param);
            }

            evt_ack_status = SVCCTL_EvtAckFlowEnable;
            break;
        }
    } while (0);

    return evt_ack_status;
}
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline SVCCTL_EvtAckStatus_t _ble_adapter_sidewalk_process_hci_le_meta_evt(const evt_le_meta_event * const evt)
{
    SVCCTL_EvtAckStatus_t evt_ack_status = SVCCTL_EvtNotAck;

    do
    {
        switch (evt->subevent)
        {
            case HCI_LE_DATA_LENGTH_CHANGE_SUBEVT_CODE:
                {
                    const hci_le_data_length_change_event_rp0 * const p_le_data_length_change = (hci_le_data_length_change_event_rp0 *)evt->data;

                    if (p_le_data_length_change->Connection_Handle == sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id)
                    {
                        SID_PAL_LOG_INFO("Sidewalk BLE DLE: MaxTxOctets: %u, MaxTxTime: %uus, MaxRxOctets: %u, MaxRxTime: %uus",
                                         p_le_data_length_change->MaxTxOctets,
                                         p_le_data_length_change->MaxTxTime,
                                         p_le_data_length_change->MaxRxOctets,
                                         p_le_data_length_change->MaxRxTime);

                        /* Event processing is finished */
                        evt_ack_status = SVCCTL_EvtAckFlowEnable;
                    }
                }
                break;

            default:
                /* Do nothing, allow the event flow to continue */
                break;
        }
    } while (0);

    return evt_ack_status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static SVCCTL_EvtAckStatus_t _ble_adapter_sidewalk_svc_evt_handler(void *p_Event)
{
    SVCCTL_EvtAckStatus_t evt_ack_status = SVCCTL_EvtNotAck;

    do
    {
        const hci_event_pckt * const p_event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)p_Event)->data);
        const evt_blecore_aci *      p_blecore_evt;

        /* Check if Sidewalk link is up */
        if (NULL == sid_ble_sidewalk_ctx.conn_ctx)
        {
            /* No active connection, nothing to process */
            evt_ack_status = SVCCTL_EvtNotAck;
            break;
        }

        switch(p_event_pckt->evt)
        {
            case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
                p_blecore_evt = (evt_blecore_aci*)p_event_pckt->data;

                switch(p_blecore_evt->ecode)
                {
                    case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
                        evt_ack_status = _ble_adapter_sidewalk_process_gatt_attr_modified_evt((aci_gatt_attribute_modified_event_rp0 *)p_blecore_evt->data);
                        break;

                    case ACI_GATT_NOTIFICATION_COMPLETE_VSEVT_CODE:
                        {
                            const aci_gatt_notification_complete_event_rp0 * const p_notification_complete = (aci_gatt_notification_complete_event_rp0 *)p_blecore_evt->data;

                            if (p_notification_complete->Attr_Handle == (sid_ble_sidewalk_ctx.ama_ctx.outbox_char_handle + SID_STM32_BLE_CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
                            {
                                /* Invoke TX completion callback */
                                SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks != NULL);
                                SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks->ind_callback != NULL);
                                sid_ble_sidewalk_ctx.event_callbacks->ind_callback(TRUE);

                                SID_PAL_LOG_DEBUG("==>> AMA Outbox upload completed");

                                /* Event processing is finished */
                                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                            }
                        }
                        break;

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                    case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE:
                        {
                            const aci_gatt_read_permit_req_event_rp0 * const p_read_permit_req = (aci_gatt_read_permit_req_event_rp0 *)p_blecore_evt->data;
                            uint8_t resp_error_code = SID_STM32_BLE_ATT_STATUS_APP_ERROR_GENERIC;
                            tBleStatus ble_status;

                            /* Ensure we are processing only Sidewalk-related access */
                            if (p_read_permit_req->Connection_Handle != sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id)
                            {
                                break;
                            }

                            evt_ack_status = _ble_adapter_sidewalk_process_gap_char_read_req(p_read_permit_req->Attribute_Handle);

                            /* Send the response */
                            if (SVCCTL_EvtAckFlowEnable == evt_ack_status)
                            {
                                ble_status = aci_gatt_allow_read(p_read_permit_req->Connection_Handle);
                                if (ble_status != BLE_STATUS_SUCCESS)
                                {
                                    SID_PAL_LOG_ERROR("Failed to allow read for BLE connection 0x%04X, attribute 0x%04X. ACI error 0x%02X", p_read_permit_req->Connection_Handle, p_read_permit_req->Attribute_Handle, ble_status);
                                    (void)aci_gap_terminate(p_read_permit_req->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
                                }
                            }
                            else if (SVCCTL_EvtAckFlowDisable == evt_ack_status)
                            {
                                SID_PAL_LOG_WARNING("Rejected read request for BLE connection 0x%04X, attribute 0x%04X. Reason code 0x%02X", p_read_permit_req->Connection_Handle, p_read_permit_req->Attribute_Handle, resp_error_code);

                                ble_status = aci_gatt_deny_read(p_read_permit_req->Connection_Handle, resp_error_code);
                                if (ble_status != BLE_STATUS_SUCCESS)
                                {
                                    SID_PAL_LOG_ERROR("Failed to deny read for BLE connection 0x%04X, attribute 0x%04X. ACI error 0x%02X", p_read_permit_req->Connection_Handle, p_read_permit_req->Attribute_Handle, ble_status);
                                    (void)aci_gap_terminate(p_read_permit_req->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
                                }

                                /* Done with processing of this event */
                                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                            }
                            else
                            {
                                /* Nothing to do here */
                            }
                        }
                        break;

                    case ACI_GATT_READ_MULTI_PERMIT_REQ_VSEVT_CODE:
                        {
                            const aci_gatt_read_multi_permit_req_event_rp0 * const p_read_multi_permit_req = (aci_gatt_read_multi_permit_req_event_rp0 *)p_blecore_evt->data;
                            uint8_t resp_error_code = SID_STM32_BLE_ATT_STATUS_APP_ERROR_GENERIC;
                            tBleStatus ble_status;

                            /* Ensure we are processing only Sidewalk-related access */
                            if (p_read_multi_permit_req->Connection_Handle != sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id)
                            {
                                break;
                            }

                            for (uint32_t i = 0u; i < p_read_multi_permit_req->Number_of_Handles; i++)
                            {
                                evt_ack_status = _ble_adapter_sidewalk_process_gap_char_read_req(p_read_multi_permit_req->Handle_Item[i].Handle);
                                if (evt_ack_status != SVCCTL_EvtAckFlowEnable)
                                {
                                    break;
                                }
                            }

                            /* Send the response */
                            if (SVCCTL_EvtAckFlowEnable == evt_ack_status)
                            {
                                ble_status = aci_gatt_allow_read(p_read_multi_permit_req->Connection_Handle);
                                if (ble_status != BLE_STATUS_SUCCESS)
                                {
                                    SID_PAL_LOG_ERROR("Failed to allow multiple attribute read for BLE connection 0x%04X. ACI error 0x%02X", p_read_multi_permit_req->Connection_Handle, ble_status);
                                    (void)aci_gap_terminate(p_read_multi_permit_req->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
                                }
                            }
                            else if (SVCCTL_EvtAckFlowDisable == evt_ack_status)
                            {
                                SID_PAL_LOG_WARNING("Rejected read multiple request for BLE connection 0x%04X. Reason code 0x%02X", p_read_multi_permit_req->Connection_Handle, resp_error_code);

                                ble_status = aci_gatt_deny_read(p_read_multi_permit_req->Connection_Handle, resp_error_code);
                                if (ble_status != BLE_STATUS_SUCCESS)
                                {
                                    SID_PAL_LOG_ERROR("Failed to deny multiple attribute read for BLE connection 0x%04X. ACI error 0x%02X", p_read_multi_permit_req->Connection_Handle, ble_status);
                                    (void)aci_gap_terminate(p_read_multi_permit_req->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
                                }

                                /* Done with processing of this event */
                                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                            }
                            else
                            {
                                /* Nothing to do here */
                            }
                        }
                        break;
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

                    default:
                        /* Allow other handlers to process this event */
                        break;
                }
                break;

            case HCI_LE_META_EVT_CODE:
                evt_ack_status = _ble_adapter_sidewalk_process_hci_le_meta_evt((evt_le_meta_event *)p_event_pckt->data);
                break;

            default:
                /* Allow other handlers to process this event */
                break;
        }
    } while (0);

    return evt_ack_status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_ama_gatt_send_out(uint8_t * const data, const uint32_t length)
{
    sid_error_t status = SID_ERROR_GENERIC;
    tBleStatus ble_ret = BLE_STATUS_ERROR;

    if ((NULL == sid_ble_sidewalk_ctx.conn_ctx)
         || (FALSE == sid_ble_sidewalk_ctx.ama_ctx.is_notification_enabled)
         || (SID_STM32_BLE_HANDLE_INVALID_VALUE == sid_ble_sidewalk_ctx.ama_ctx.service_context.service_handle)
         || (SID_STM32_BLE_HANDLE_INVALID_VALUE == sid_ble_sidewalk_ctx.ama_ctx.outbox_char_handle))
    {
        return SID_ERROR_INVALID_STATE;
    }

    /* Use aci_gatt_update_char_value_ext() instead of aci_gatt_update_char_value() to support MTU sizes greater than 251 bytes */
    if (length <= SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT)
    {
        ble_ret = aci_gatt_update_char_value_ext(sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id,
                                                 sid_ble_sidewalk_ctx.ama_ctx.service_context.service_handle,
                                                 sid_ble_sidewalk_ctx.ama_ctx.outbox_char_handle,
                                                 SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_TYPE_NOTIFY,
                                                 (uint16_t)length,
                                                 0u, /* value offset */
                                                 (uint8_t)length,
                                                 data);
    }
    else
    {
        /* Use several subsequent calls to pass all the data to BLE stack. Since maximum MTU size is 512 bytes we may need up to 3 calls */
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> aci_gatt_update_char_value_ext - sending out   long data (%u bytes)", length);

        uint32_t remaining_data_size = length;
        uint32_t current_offset = 0u;

        while (remaining_data_size > SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT)
        {
            /* First call just pushes the data but does not trigger the actual value change notification */
            ble_ret = aci_gatt_update_char_value_ext(sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id,
                                                     sid_ble_sidewalk_ctx.ama_ctx.service_context.service_handle,
                                                     sid_ble_sidewalk_ctx.ama_ctx.outbox_char_handle,
                                                     SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_TYPE_DO_NOT_NOTIFY,
                                                     (uint16_t)length, /* total length */
                                                     (uint16_t)current_offset, /* value offset */
                                                     SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT, /* size of the partial data to push */
                                                     data + current_offset);
            if (BLE_STATUS_SUCCESS == ble_ret)
            {
                SID_BLE_SIDEWALK_LOG_DEBUG("==>> aci_gatt_update_char_value_ext - updated partial data, start offset: %u, bytes written: %u, remaining size: %u", current_offset, SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT, remaining_data_size - SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT);
                current_offset += SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT;
                remaining_data_size -= SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT;
            }
            else
            {
                SID_BLE_SIDEWALK_LOG_DEBUG("==>> aci_gatt_update_char_value_ext - fail on putting partial data, start offset: %u, total length: %u, result: 0x%02X", current_offset, length, ble_ret);
                break;
            }
        }

        if (BLE_STATUS_SUCCESS == ble_ret)
        {
            /* The last call uploads the rest of the data and triggers BLE notification indication */
            ble_ret = aci_gatt_update_char_value_ext(sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id,
                                                     sid_ble_sidewalk_ctx.ama_ctx.service_context.service_handle,
                                                     sid_ble_sidewalk_ctx.ama_ctx.outbox_char_handle,
                                                     SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_TYPE_NOTIFY,
                                                     (uint16_t)length, /* total length */
                                                     (uint16_t)current_offset, /* value offset */
                                                     (uint8_t)remaining_data_size, /* size of the remaining partial data to push */
                                                     data + current_offset);
        }
    }

    if (BLE_STATUS_SUCCESS == ble_ret)
    {
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> aci_gatt_update_char_value_ext - success, %u bytes written", length);
        status = SID_ERROR_NONE;
    }
    else if (BLE_STATUS_INSUFFICIENT_RESOURCES == ble_ret)
    {
        SID_BLE_SIDEWALK_LOG_WARNING("==>> aci_gatt_update_char_value_ext - recoverable failure, insufficient resources");
        status = SID_ERROR_BUSY;
    }
    else
    {
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> aci_gatt_update_char_value_ext - fail, result: 0x%02X", ble_ret);
        status = SID_ERROR_IO_ERROR;
    }

    return status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _ble_adapter_sidewalk_ama_process_send_out(void* thread_input)
{
    UNUSED(thread_input);

    while (1)
    {
        osStatus_t status;
        AMAMessageQueueElement_t msg_descriptor;
        sid_error_t gatt_status;
        uint32_t retry_counter = 0u;

        status = osMessageQueueGet(ama_sendout_queue, &msg_descriptor, NULL, osWaitForever);
        if (status != osOK)
        {
            SID_PAL_LOG_WARNING("Unable to fetch outgoing AMA message from queue. Status code: %d", status);
            continue;
        }

        if ((NULL == msg_descriptor.data) || (0u == msg_descriptor.length))
        {
            SID_PAL_LOG_WARNING("Empty message fetched from outgoing AMA message from queue. Send out will be ignored");
            continue;
        }

        SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks != NULL);
        SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks->ind_callback != NULL);

        do
        {
            gatt_status = _ble_adapter_sidewalk_ama_gatt_send_out(msg_descriptor.data, msg_descriptor.length);

            if (SID_ERROR_NONE == gatt_status)
            {
                /* Notification sent */
                SID_PAL_LOG_DEBUG("==>> Successfully put %d bytes into AMA Outbox", msg_descriptor.length);
                break;
            }
            else if (SID_ERROR_BUSY == gatt_status)
            {
                /*  Recoverable state, try again a bit later */
                retry_counter++;
                osDelay(2);
            }
            else
            {
                /* Unrecoverable error, stop processing */
                switch (gatt_status)
                {
                    case SID_ERROR_INVALID_STATE:
                        SID_PAL_LOG_ERROR("==>> Unable to send out AMA message. No valid connection opened. Message will be lost");
                        break;
                    
                    default:
                        SID_PAL_LOG_ERROR("==>> Unable to write AMA message to outbox. Status code: %d", gatt_status);
                        break;
                }

                /* Invoke TX completion callback */
                sid_ble_sidewalk_ctx.event_callbacks->ind_callback(FALSE);
                break;
            }
        } while ((gatt_status != SID_ERROR_NONE) && (retry_counter <= 10));

        /* Finally release the RAM occupied by the message */
        free(msg_descriptor.data);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_service_init(sid_ble_adapter_sid_gatt_svc_ctx_t * const out_svc_ctx, const sid_ble_adapter_sid_gatt_svc_init_t * const ble_svc_init)
{
    sid_error_t err;
    tBleStatus  ble_status;

    do
    {
        /* Validate input params */
        if ((NULL == out_svc_ctx) || (NULL == ble_svc_init) || (NULL == ble_svc_init->service_def))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Copy service UUID and ID */
        out_svc_ctx->service_info = ble_svc_init->service_def->service;

        /* Allocate memory for characteristics of the service */
        out_svc_ctx->num_characteristics = ble_svc_init->service_def->char_count;
        out_svc_ctx->characteristic_context = (sid_ble_adapter_sid_gatt_char_ctx_t*)malloc(sizeof(*out_svc_ctx->characteristic_context) * out_svc_ctx->num_characteristics);
        if (NULL == out_svc_ctx->characteristic_context)
        {
            SID_PAL_LOG_ERROR("  Failed to allocate memory for GATT service context storage");
            err = SID_ERROR_OOM;
            break;
        }

        /* Add service */
        uint8_t stm_uuid_type;
        Service_UUID_t stm_uuid;
        char msg_buf[100];

        err = sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_svc_uuid(&stm_uuid_type, &stm_uuid, &out_svc_ctx->service_info.id);
        if (err != SID_ERROR_NONE)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("  Fail   : sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_svc_uuid command, error code: %d", err);
            break;
        }

        ble_status = aci_gatt_add_service(stm_uuid_type,
                                          &stm_uuid,
                                          PRIMARY_SERVICE,
                                          (2u * (ble_svc_init->service_def->char_count + ble_svc_init->service_def->desc_count)) + 2u, /* Max_Attribute_Records = 2*no_of_char + 2*num_of_desc + 2 for svc */
                                          &(out_svc_ctx->service_handle));
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            int pos = sprintf(msg_buf, "  Fail   : aci_gatt_add_service command for service ");
            pos += sid_stm32wba_ble_adapter_util_sprintf_uuid(&msg_buf[pos], &out_svc_ctx->service_info.id);
            pos += sprintf(&msg_buf[pos], ", error code: 0x%02X", ble_status);
            SID_BLE_SIDEWALK_LOG_ERROR("%s", msg_buf);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        else
        {
            int pos = sprintf(msg_buf, "  Success: aci_gatt_add_service command for service ");
            pos += sid_stm32wba_ble_adapter_util_sprintf_uuid(&msg_buf[pos], &out_svc_ctx->service_info.id);
            SID_BLE_SIDEWALK_LOG_DEBUG("%s", msg_buf);
        }

        /* Add characteristics */
        err = _ble_adapter_sidewalk_add_characteristics(out_svc_ctx, ble_svc_init->service_def);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Add event handler */
        out_svc_ctx->event_handler = ble_svc_init->svc_event_handler;
        if (out_svc_ctx->event_handler != NULL)
        {
            err = sid_stm32wba_ble_adapter_prv_generic_register_svc_handler(out_svc_ctx->event_handler);
            if (err != SID_ERROR_NONE)
            {
                int pos = sprintf(msg_buf, "  Fail   : unable to register event handler for service ");
                (void)sid_stm32wba_ble_adapter_util_sprintf_uuid(&msg_buf[pos], &out_svc_ctx->service_info.id);
                SID_BLE_SIDEWALK_LOG_ERROR("%s, error %d", msg_buf, (int32_t)err);

                out_svc_ctx->event_handler = NULL;
                break;
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_add_characteristics(sid_ble_adapter_sid_gatt_svc_ctx_t * const out_svc_ctx, const sid_ble_cfg_gatt_profile_t * const service_def)
{
    sid_error_t err = SID_ERROR_GENERIC;

    SID_PAL_ASSERT(sid_ble_drv_ctx.cfg != NULL);
    SID_PAL_ASSERT(out_svc_ctx);
    SID_PAL_ASSERT(service_def);
    SID_PAL_ASSERT(out_svc_ctx->num_characteristics == service_def->char_count);

    for (uint32_t i = 0u; i < service_def->char_count; i++)
    {
        Char_UUID_t stm_uuid;
        uint8_t stm_uuid_type;
        tBleStatus ble_status;
        char msg_buf[100];
        sid_ble_adapter_sid_gatt_char_ctx_t * const current_char_ctx = &(out_svc_ctx->characteristic_context[i]);
        const sid_ble_cfg_characteristics_t * const char_def = &(service_def->characteristic[i]);

        /* Copy UUID */
        current_char_ctx->characteristic_info = *char_def;

        /* Convert UUID to STM BLE stack format */
        err = sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_char_uuid(&stm_uuid_type, &stm_uuid, &current_char_ctx->characteristic_info.id);
        if (err != SID_ERROR_NONE)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("  Fail   : sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_char_uuid command, error code: %d", err);
            break;
        }

        /* Compute characteristic settings */
        uint8_t char_props = CHAR_PROP_NONE;
        uint8_t event_mask = GATT_DONT_NOTIFY_EVENTS;
        if (char_def->properties.is_read != false)
        {
            char_props |= CHAR_PROP_READ;
            event_mask |= GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
        }
        if (char_def->properties.is_write != false)
        {
            char_props |= CHAR_PROP_WRITE;
            event_mask |= (GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP);
        }
        if (char_def->properties.is_write_no_resp != false)
        {
            char_props |= CHAR_PROP_WRITE_WITHOUT_RESP;
            event_mask |= (GATT_NOTIFY_ATTRIBUTE_WRITE);
        }
        if (char_def->properties.is_notify != false)
        {
            char_props |= CHAR_PROP_NOTIFY;
            event_mask |= (GATT_NOTIFY_NOTIFICATION_COMPLETION);
        }

        uint8_t char_perms = ATTR_PERMISSION_NONE;
        if (char_def->perm.is_none == false)
        {
            if (char_def->perm.is_read != false)
            {
                char_perms |= ATTR_PERMISSION_AUTHEN_READ;
            }
            if (char_def->perm.is_write != false)
            {
                char_perms |= ATTR_PERMISSION_AUTHEN_WRITE;
            }
        }

        ble_status = aci_gatt_add_char(out_svc_ctx->service_handle,
                                       stm_uuid_type,
                                       &stm_uuid,
                                       SID_STM32_BLE_ATT_LEN_LIMIT(sid_ble_drv_ctx.cfg->sidewalk_profile.max_att_mtu),
                                       char_props,
                                       char_perms,
                                       event_mask,
                                       SID_STM32_BLE_CHAR_ENC_KEY_MINIMUM_LENGTH,
                                       CHAR_VALUE_LEN_VARIABLE,
                                       &(current_char_ctx->characteristic_handle));
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            int pos = sprintf(msg_buf, "  Fail   : aci_gatt_add_char command for characteristic ");
            pos += sid_stm32wba_ble_adapter_util_sprintf_uuid(&msg_buf[pos], &(current_char_ctx->characteristic_info.id));
            pos += sprintf(&msg_buf[pos], ", error code: 0x%02X", ble_status);
            SID_BLE_SIDEWALK_LOG_ERROR("%s", msg_buf);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        else
        {
            int pos = sprintf(msg_buf, "  Success: aci_gatt_add_char ");
            pos += sid_stm32wba_ble_adapter_util_sprintf_uuid(&msg_buf[pos], &(current_char_ctx->characteristic_info.id));
            SID_BLE_SIDEWALK_LOG_DEBUG("%s", msg_buf);
        }

        err = SID_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_services_init(const sid_ble_config_t * const sid_ble_cfg)
{
    sid_error_t err = SID_ERROR_GENERIC;

    SID_BLE_SIDEWALK_LOG_DEBUG("==>> Start ble_adapter_sidewalk_services_init function");

    do
    {
        /* Validate inputs */
        if (NULL == sid_ble_cfg)
        {
            SID_PAL_LOG_ERROR("  Fail   : Cannot initialize Sidewalk BLE GATT services. Sidewalk BLE config is NULL");
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (NULL == sid_ble_cfg->profile)
        {
            SID_PAL_LOG_ERROR("  Fail   : Cannot initialize Sidewalk BLE GATT services. Profile specification is NULL");
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (sid_ble_cfg->num_profile > SID_STM32_BLE_SIDEWALK_NUM_GATT_SERVICES)
        {
            SID_PAL_LOG_ERROR("  Fail   : Invalid WPAN config. Number of the GATT services in profile exceeds SID_STM32_BLE_SIDEWALK_NUM_GATT_SERVICES parameter");
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        // TODO: calculate total attributes count and compare against CFG_BLE_NUM_GATT_ATTRIBUTES

        /* Invalidate AMA service context */
        _ble_adapter_sidewalk_invalidate_ama_svc_ctx();

        for (uint32_t i = 0u; i < sid_ble_cfg->num_profile; i++)
        {
            const sid_ble_cfg_gatt_profile_t * const profile = &sid_ble_cfg->profile[i];
            sid_ble_adapter_sid_gatt_svc_init_t ble_svc_init = {0};

            switch (profile->service.type)
            {
                case AMA_SERVICE:
                    ble_svc_init.svc_event_handler = _ble_adapter_sidewalk_svc_evt_handler;
                    break;

                case VENDOR_SERVICE:
                case LOGGING_SERVICE:
                default:
                    {
                        char buf[90];
                        int pos = sprintf(buf, "Adding GATT service with UUID ");
                        pos += sid_stm32wba_ble_adapter_util_sprintf_uuid(&buf[pos], &profile->service.id);
                        pos += sprintf(&buf[pos]," without event handler");
                        SID_PAL_LOG_WARNING("%s", buf);
                    }
                    break;
            }

            sid_ble_adapter_sid_gatt_svc_ctx_t svc_ctx;
            ble_svc_init.service_def = (sid_ble_cfg_gatt_profile_t *)profile;
            err = _ble_adapter_sidewalk_service_init(&svc_ctx, &ble_svc_init);
            if (err != SID_ERROR_NONE)
            {
                char buf[90];
                int pos = sprintf(buf, "  Failed to initialize GATT service ");
                pos += sid_stm32wba_ble_adapter_util_sprintf_uuid(&buf[pos], &profile->service.id);
                SID_PAL_LOG_ERROR("%s, error %d", buf, (int32_t)err);
                break;
            }

            if (AMA_SERVICE == profile->service.type)
            {
                /* Allocate memory for AMA Inbox buffer */
                SID_PAL_ASSERT(NULL == sid_ble_sidewalk_ctx.ama_ctx.inbox_buf);
                sid_ble_sidewalk_ctx.ama_ctx.inbox_valid_length = 0u;
                sid_ble_sidewalk_ctx.ama_ctx.inbox_buf = malloc(SID_STM32_BLE_ATT_LEN_LIMIT(sid_ble_drv_ctx.cfg->sidewalk_profile.max_att_mtu));
                if (NULL == sid_ble_sidewalk_ctx.ama_ctx.inbox_buf)
                {
                    SID_PAL_LOG_ERROR("Unable to allocate memory for Sidewalk AMA Inbox - no memory");
                    err = SID_ERROR_OOM;
                    break;
                }

                /* Store context for quick access */
                sid_ble_sidewalk_ctx.ama_ctx.service_context = svc_ctx;
                SID_PAL_ASSERT(BLE_AMA_SERVICE_CHAR_COUNT == sid_ble_sidewalk_ctx.ama_ctx.service_context.num_characteristics);
                for (uint32_t i = 0u; i < sid_ble_sidewalk_ctx.ama_ctx.service_context.num_characteristics; i++)
                {
                    const sid_ble_adapter_sid_gatt_char_ctx_t * const char_ctx = &(sid_ble_sidewalk_ctx.ama_ctx.service_context.characteristic_context[i]);
                    if ((char_ctx->characteristic_info.properties.is_read != false) || (char_ctx->characteristic_info.properties.is_notify != false))
                    {
                        sid_ble_sidewalk_ctx.ama_ctx.outbox_char_index = i;
                        sid_ble_sidewalk_ctx.ama_ctx.outbox_char_handle = char_ctx->characteristic_handle;
                        continue;
                    }
                    if ((char_ctx->characteristic_info.properties.is_write != false) || (char_ctx->characteristic_info.properties.is_write_no_resp != false))
                    {
                        sid_ble_sidewalk_ctx.ama_ctx.inbox_char_index = i;
                        sid_ble_sidewalk_ctx.ama_ctx.inbox_char_handle = char_ctx->characteristic_handle;
                        continue;
                    }
                }

                SID_PAL_ASSERT(sid_ble_sidewalk_ctx.ama_ctx.inbox_char_handle != SID_STM32_BLE_HANDLE_INVALID_VALUE);
                SID_PAL_ASSERT(sid_ble_sidewalk_ctx.ama_ctx.outbox_char_handle != SID_STM32_BLE_HANDLE_INVALID_VALUE);
            }
        }

        /* Check if the initialization loop terminated with error */
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    SID_BLE_SIDEWALK_LOG_DEBUG("==>> End ble_adapter_sidewalk_services_init function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_service_deinit(sid_ble_adapter_sid_gatt_svc_ctx_t * const svc_ctx)
{
    sid_error_t err = SID_ERROR_NONE;
    tBleStatus  ble_status;
    char        uuid_str[40];

    SID_PAL_ASSERT(svc_ctx != NULL);

    SID_BLE_SIDEWALK_LOG_DEBUG("==>> Start ble_adapter_sidewalk_service_deinit function");

    (void)sid_stm32wba_ble_adapter_util_sprintf_uuid(uuid_str, &svc_ctx->service_info.id);
    SID_BLE_SIDEWALK_LOG_DEBUG("  Removing GATT service %s with %u characteristics", uuid_str, svc_ctx->num_characteristics);

    /* Remove the service from GATT database */
    ble_status = aci_gatt_del_service(svc_ctx->service_handle);
    if (ble_status != BLE_STATUS_SUCCESS)
    {
        SID_BLE_SIDEWALK_LOG_ERROR("  Fail   : aci_gatt_del_service command for service %s, error code: 0x%02X", uuid_str, ble_status);
        err = SID_ERROR_IO_ERROR;
    }

    svc_ctx->service_handle = SID_STM32_BLE_HANDLE_INVALID_VALUE;

    /* Deregister the event handler */
    if (svc_ctx->event_handler != NULL)
    {
        (void)sid_stm32wba_ble_adapter_prv_generic_deregister_svc_handler(svc_ctx->event_handler);
        svc_ctx->event_handler = NULL;
    }

    /* Clean up characteristic contexts */
    if (svc_ctx->characteristic_context != NULL)
    {
        free(svc_ctx->characteristic_context);
        svc_ctx->characteristic_context = NULL;
    }
    svc_ctx->num_characteristics = 0u;

    SID_BLE_SIDEWALK_LOG_DEBUG("==>> End ble_adapter_sidewalk_service_deinit function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_services_deinit(void)
{
    sid_error_t err = SID_ERROR_NONE;

    SID_BLE_SIDEWALK_LOG_DEBUG("==>> Start ble_adapter_sidewalk_services_deinit function");

    sid_pal_enter_critical_region();

    /* Delete AMA service from the GATT database */
    err = _ble_adapter_sidewalk_service_deinit(&sid_ble_sidewalk_ctx.ama_ctx.service_context);

    /* Invalidate AMA service context */
    _ble_adapter_sidewalk_invalidate_ama_svc_ctx();

    sid_pal_exit_critical_region();

    SID_BLE_SIDEWALK_LOG_DEBUG("==>> End ble_adapter_sidewalk_services_deinit function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_init(const sid_ble_config_t * cfg)
{
    sid_error_t err = SID_ERROR_GENERIC;
    osStatus_t  os_status;
    uint32_t    ll_lock_acquired;
    const sid_pal_ble_prv_operating_mode_t init_type =
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                                                  SPBP_OPERATING_MODE_CONCURRENT;
#else
                                                  SPBP_OPERATING_MODE_SIDEWALK;
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY */

    SID_BLE_SIDEWALK_LOG_DEBUG("==>> Start ble_adapter_sidewalk_init function");

    if (sid_ble_sidewalk_ctx.init_done != FALSE)
    {
        SID_PAL_LOG_WARNING("Sidewalk BLE is initialized already. Requesting deinitialization");

        /* Deinitialize any hardware that may have been partially initialized to bring it to the known state */
        err = _ble_adapter_sidewalk_deinit();
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Unable to initialize Sidewalk BLE radio - driver is initialized already and deinitialization failed. Error %d", (int32_t)err);
            return err;
        }
    }

    if (LinkLayerMutex != NULL)
    {
        /* Wait for any potentially ongoing BLE operations to stop */
        os_status = osMutexAcquire(LinkLayerMutex, SID_STM32_BLE_LL_MUTEX_WAIT_TIMEOUT_ON_INIT_TICKS);

        /* Print out warning but proceed */
        if (os_status != osOK)
        {
            SID_PAL_LOG_WARNING("Failed to acquire BLE LL mutex, race conditions are possible during Sidewalk BLE init");
            ll_lock_acquired = FALSE;
        }
        else
        {
            ll_lock_acquired = TRUE;
        }
    }
    else
    {
        /* This is the very first initialization, the mutex is not created yet */
        ll_lock_acquired = FALSE;
    }

    do
    {
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING)
        if (sid_ble_drv_ctx.operating_mode != SPBP_OPERATING_MODE_OFF)
        {
            SID_PAL_LOG_ERROR("Sidewalk BLE mode cannot be initialized because BLE radio is busy with another mode (%u)", sid_ble_drv_ctx.operating_mode);
            err = SID_ERROR_BUSY;
            break;
        }
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING */

        /* Validate BLE config for Sidewalk */
        if (cfg->conn_param.min_conn_interval < SID_BLE_SPEC_CONN_INTERVAL_MIN)
        {
            SID_PAL_LOG_ERROR("Invalid Sidewalk BLE config. Min connection interval is below the Sidewalk specification");
            err = SID_ERROR_PARAM_OUT_OF_RANGE;
            break;
        }
        if (cfg->conn_param.max_conn_interval > SID_BLE_SPEC_CONN_INTERVAL_MAX)
        {
            SID_PAL_LOG_ERROR("Invalid Sidewalk BLE config. Mac connection interval is above the Sidewalk specification");
            err = SID_ERROR_PARAM_OUT_OF_RANGE;
            break;
        }
        if (cfg->conn_param.conn_sup_timeout < sid_stm32wba_ble_adapter_util_calc_min_conn_timeout(cfg->conn_param.max_conn_interval, cfg->conn_param.slave_latency))
        {
            SID_PAL_LOG_ERROR("Invalid Sidewalk BLE config. Connection supervision timeout is too short for given latency and interval");
            err = SID_ERROR_PARAM_OUT_OF_RANGE;
            break;
        }

        /* Invalidate AMA Inbox buffer */
        sid_ble_sidewalk_ctx.ama_ctx.inbox_valid_length = 0u;

        /* Do the generic BLE stack init */
        err = sid_stm32wba_ble_adapter_prv_generic_init(init_type, cfg);
        if (err != SID_ERROR_NONE)
        {
            /* Logs are provided by sid_stm32wba_ble_adapter_prv_generic_init() */
            break;
        }

        SID_PAL_LOG_INFO("Sidewalk BLE max ATT MTU: %u bytes", sid_ble_drv_ctx.cfg->sidewalk_profile.max_att_mtu);

        /* Initialize the associated RTOS resources for Sidewalk GATT profile */
        if ((ama_sendout_queue != NULL) || (ama_sendout_task != NULL))
        {
            err = SID_ERROR_ALREADY_INITIALIZED;
            break;
        }
        ama_sendout_queue = osMessageQueueNew(SIDEWALK_BLE_AMA_SEND_OUT_MSG_QUEUE_LEN, sizeof(AMAMessageQueueElement_t), &ama_sendout_queue_attributes);
        ama_sendout_task = osThreadNew(_ble_adapter_sidewalk_ama_process_send_out, NULL, &ama_sendout_task_attributes);
        if ((NULL == ama_sendout_queue) || (NULL == ama_sendout_task))
        {
            err = SID_ERROR_OOM;
            break;
        }

        /* Set advertisement parameters - this will create an extended advertisement set for Sidewalk (if used) */
        err = _ble_adapter_sidewalk_set_adv_params(sid_ble_sidewalk_ctx.ble_cfg->adv_param.fast_interval);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_sidewalk_set_adv_params() */
            break;
        }

        /* Add Sidewalk GATT profile */
        err = _ble_adapter_sidewalk_services_init(sid_ble_sidewalk_ctx.ble_cfg);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_sidewalk_services_init() */
            break;
        }

        SID_PAL_LOG_INFO("Sidewalk BLE init done");
        sid_ble_sidewalk_ctx.init_done = TRUE;
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    if (ll_lock_acquired != FALSE)
    {
        os_status = osMutexRelease(LinkLayerMutex);
        if (os_status != osOK)
        {
            SID_PAL_LOG_ERROR("Failed to release BLE LL mutex. BLE stack will be blocked till reset");
            err = SID_ERROR_UNRECOVERABLE;
        }
    }

    /* Release any partially allocated resources if the initialization has failed */
    if (err != SID_ERROR_NONE)
    {
        (void)_ble_adapter_sidewalk_deinit();
    }

    SID_BLE_SIDEWALK_LOG_DEBUG("==>> End ble_adapter_sidewalk_init function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_start_service(void)
{
    return SID_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_error_t _ble_adapter_sidewalk_set_adv_params(const uint32_t adv_interval)
{
    sid_error_t       err;
    tBleStatus        ble_status;
    const uint32_t    adv_interval_min = adv_interval - SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(((float)(SID_STM32_BLE_SIDEWALK_ADV_WINDOW_MS) / 2.f));
    const uint32_t    adv_interval_max = adv_interval + SID_STM32_BLE_MS_TO_ADV_INTERVAL_UNITS(((float)(SID_STM32_BLE_SIDEWALK_ADV_WINDOW_MS) / 2.f));

#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
    SID_PAL_ASSERT(adv_interval > SID_BLE_HCI_EXTENDED_ADV_INTERVAL_MIN);
    SID_PAL_ASSERT(adv_interval < SID_BLE_HCI_EXTENDED_ADV_INTERVAL_MAX);
    SID_PAL_ASSERT(adv_interval_min >= SID_BLE_HCI_EXTENDED_ADV_INTERVAL_MIN);
    SID_PAL_ASSERT(adv_interval_max <= SID_BLE_HCI_EXTENDED_ADV_INTERVAL_MAX);
#else
    SID_PAL_ASSERT(adv_interval > SID_BLE_HCI_ADV_INTERVAL_MIN);
    SID_PAL_ASSERT(adv_interval < SID_BLE_HCI_ADV_INTERVAL_MAX);
    SID_PAL_ASSERT(adv_interval_min >= SID_BLE_HCI_ADV_INTERVAL_MIN);
    SID_PAL_ASSERT(adv_interval_max <= SID_BLE_HCI_ADV_INTERVAL_MAX);
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */
    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.ble_cfg != NULL);

    do
    {
        uint8_t own_addr_type = sid_stm32wba_ble_adapter_util_convert_sid_ble_addr_type_to_hci_own_adv_addr_type(sid_ble_sidewalk_ctx.ble_cfg->mac_addr_type);

        /* Configure Identity Address for RPA */
        uint8_t                          ia_addr_type;
        sid_pal_ble_prv_bt_addr_buffer_t ia_addr;

#if SID_STM32_BLE_USE_RANDOM_IDENTITY_ADDRESS
        ia_addr_type = SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RANDOM;
        err = sid_stm32wba_ble_adapter_util_get_host_static_random_addr(&ia_addr);
#else
        ia_addr_type = SID_BLE_HCI_ADV_PEER_ADDR_TYPE_PUBLIC;
        err = sid_stm32wba_ble_adapter_util_get_host_public_addr(&ia_addr);
#endif /* SID_STM32_BLE_USE_RANDOM_IDENTITY_ADDRESS */
        if (err != SID_ERROR_NONE)
        {
            break;
        }

#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
        int8_t selected_adv_tx_pwr;

        /* Configure extended advertisement set for Sidewalk */
        ble_status = hci_le_set_extended_advertising_parameters(
                        sid_ble_sidewalk_ctx.adv_set.Advertising_Handle,
                        SID_STM32_BLE_LEGACY_ADV_TYPE_TO_EXT_ADV_EVT_PROPS(HCI_ADV_TYPE_ADV_IND), /* Sidewalk requires legacy advertisement PDU */
                        (uint8_t *)(void *)&adv_interval_min,
                        (uint8_t *)(void *)&adv_interval_max,
                        (ADV_CH_37 | ADV_CH_38 | ADV_CH_39),                    /* Use all channels for advertisement */
                        own_addr_type,                                          /* Typically a non-resolvable private address is used for Sidewalk advertisement, but allow the user to override this */
                        ia_addr_type,                                           /* Point to local host peer address for RPA, unused for other host address types */
                        (uint8_t *)ia_addr.bytes,                               /* Point to local host peer address for RPA, unused for other host address types */
                        HCI_ADV_FILTER_NO,                                      /* Don't use any filters for Sidewalk connections */
                        127u,                                                   /* Tx power preference - no preference */
                        HCI_PRIMARY_ADV_PHY_LE_1M,
                        0x00u,                                                  /* Secondary_Advertising_Max_Skip - irrelevant for legacy Adv PDU */
                        HCI_TX_PHY_LE_2M,                                       /* Secondary advertisement PHY - irrelevant for legacy Adv PDU */
                        0x00u,                                                  /* Advertising_SID - irrelevant for legacy Adv PDU */
                        0x01u,                                                  /* Scan request notifications enabled */
                        (uint8_t *)&selected_adv_tx_pwr);                       /* Advertisement Tx power selected by the Host Controller */
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("==>> hci_le_set_extended_advertising_parameters - fail, result: 0x%02X, set: 0x%02X", ble_status, sid_ble_sidewalk_ctx.adv_set.Advertising_Handle);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        else
        {
            SID_BLE_SIDEWALK_LOG_DEBUG("==>> hci_le_set_extended_advertising_parameters - Success, set: 0x%02X, Tx power: %s%ddBm", sid_ble_sidewalk_ctx.adv_set.Advertising_Handle, selected_adv_tx_pwr > 0 ? "+" : "", selected_adv_tx_pwr);
            err = SID_ERROR_NONE;
        }
#else
        ble_status = hci_le_set_advertising_parameters(
                        (uint16_t)adv_interval_min,
                        (uint16_t)adv_interval_max,
                        HCI_ADV_TYPE_ADV_IND,
                        own_addr_type,                                          /* Typically a non-resolvable private address is used for Sidewalk advertisement, but allow the user to override this */
                        ia_addr_type,                                           /* Point to local host peer address for RPA, unused for other host address types */
                        (uint8_t *)ia_addr.bytes,                               /* Point to local host peer address for RPA, unused for other host address types */
                        (ADV_CH_37 | ADV_CH_38 | ADV_CH_39),                    /* Use all channels for advertisement */
                        HCI_ADV_FILTER_NO);                                     /* Don't use any filters for Sidewalk connections */
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("==>> hci_le_set_advertising_parameters - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        else
        {
            SID_BLE_SIDEWALK_LOG_DEBUG("==>> hci_le_set_advertising_parameters - Success");
            err = SID_ERROR_NONE;
        }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_set_adv_data(uint8_t * mfg_data, uint8_t mfg_length)
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus  ble_status;

    SID_PAL_ASSERT(sid_ble_drv_ctx.cfg != NULL);
    SID_PAL_ASSERT(sid_ble_drv_ctx.cfg->sidewalk_profile.device_name != NULL);

    do
    {
        uint8_t adv_bin_data[SID_STM32_BLE_MAX_ADVERTISING_BUFFER_SIZE];
        uint8_t sr_bin_data[SID_STM32_BLE_MAX_ADVERTISING_BUFFER_SIZE];

        /* Validate inputs */
        if (SID_STM32_BLE_HANDLE_INVALID_VALUE == sid_ble_sidewalk_ctx.ama_ctx.service_context.service_handle)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        /* Cleanup any previous data */
        SID_STM32_UTIL_fast_memset(adv_bin_data, 0u, sizeof(adv_bin_data));
        SID_STM32_UTIL_fast_memset(sr_bin_data, 0u, sizeof(sr_bin_data));

        /* Build advertisement data */
        uint32_t adv_pos = 0u;

        /* Add advertisement flags */
        adv_bin_data[adv_pos] = SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + SID_BLE_HCI_ADV_FLAGS_RECORD_SIZE;
        adv_pos += SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE;
        adv_bin_data[adv_pos] = AD_TYPE_FLAGS;
        adv_pos += SID_BLE_HCI_ADV_TYPE_RECORD_SIZE;
        adv_bin_data[adv_pos] = (FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE | FLAG_BIT_BR_EDR_NOT_SUPPORTED);
        adv_pos += SID_BLE_HCI_ADV_FLAGS_RECORD_SIZE;

        /* Add AMA service UUID */
        uint8_t stm_uuid_type;
        Service_UUID_t stm_uuid;

        /* Convert from Sidewalk SDK format to STM32 WPAN format */
        err = sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_svc_uuid(&stm_uuid_type, &stm_uuid, &(sid_ble_sidewalk_ctx.ama_ctx.service_context.service_info.id));
        if (err != SID_ERROR_NONE)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("  Fail   : sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_svc_uuid command, error code: %d", (int32_t)err);
            break;
        }

        /* Add converted data to the buffer */
        if (STM32_WPAN_BLE_UUID_TYPE_16 == stm_uuid_type)
        {
            adv_bin_data[adv_pos] = SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + sizeof(stm_uuid.Service_UUID_16);
            adv_pos += SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE;
            adv_bin_data[adv_pos] = AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST;
            adv_pos += SID_BLE_HCI_ADV_TYPE_RECORD_SIZE;
            SID_STM32_UTIL_fast_memcpy(&adv_bin_data[adv_pos], &stm_uuid.Service_UUID_16, sizeof(stm_uuid.Service_UUID_16));
            adv_pos += sizeof(stm_uuid.Service_UUID_16);
        }
        else if (STM32_WPAN_BLE_UUID_TYPE_128 == stm_uuid_type)
        {
            adv_bin_data[adv_pos] = SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + sizeof(stm_uuid.Service_UUID_128);
            adv_pos += SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE;
            adv_bin_data[adv_pos] = AD_TYPE_128_BIT_SERV_UUID;
            adv_pos += SID_BLE_HCI_ADV_TYPE_RECORD_SIZE;
            SID_STM32_UTIL_fast_memcpy(&adv_bin_data[adv_pos], &stm_uuid.Service_UUID_128, sizeof(stm_uuid.Service_UUID_128));
            adv_pos += sizeof(stm_uuid.Service_UUID_128);
        }
        else
        {
            SID_PAL_LOG_ERROR("  Fail   : Unsupported UUID type for advertisement");
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Add manufacturing data supplied by Sidewalk stack */
        adv_bin_data[adv_pos] = SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + SID_BLE_HCI_ADV_MFG_DATA_MANUFACTURER_ID_LENGTH + mfg_length;
        adv_pos += SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE;
        adv_bin_data[adv_pos] = AD_TYPE_MANUFACTURER_SPECIFIC_DATA;
        adv_pos += SID_BLE_HCI_ADV_TYPE_RECORD_SIZE;
        adv_bin_data[adv_pos] = (uint8_t)(SID_BLE_AMAZON_COMPANY_ID & 0xFFu);
        adv_bin_data[adv_pos + 1] = (uint8_t)((SID_BLE_AMAZON_COMPANY_ID >> 8) & 0xFFu);
        adv_pos += SID_BLE_HCI_ADV_MFG_DATA_MANUFACTURER_ID_LENGTH;
        if ((mfg_data != NULL) && (mfg_length > 0u))
        {
            SID_STM32_UTIL_fast_memcpy(&adv_bin_data[adv_pos], mfg_data, mfg_length);
            adv_pos += mfg_length;
        }

        SID_PAL_ASSERT(adv_pos <= MAX_ADV_DATA_LEN);

        /* Build scan response data and put the Sidewalk device name there */
        uint32_t sr_pos = 0u;

        const uint32_t name_len = strlen(sid_ble_drv_ctx.cfg->sidewalk_profile.device_name);
        sr_bin_data[sr_pos] = SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + name_len;
        sr_pos += SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE;
        sr_bin_data[sr_pos] = AD_TYPE_COMPLETE_LOCAL_NAME;
        sr_pos += SID_BLE_HCI_ADV_TYPE_RECORD_SIZE;
        SID_STM32_UTIL_fast_memcpy(&sr_bin_data[sr_pos], sid_ble_drv_ctx.cfg->sidewalk_profile.device_name, name_len);
        sr_pos += name_len;

        SID_PAL_ASSERT(sr_pos <= MAX_ADV_DATA_LEN);

        /* Put advertisement data into the host controller */
#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
        ble_status = hci_le_set_extended_advertising_data(sid_ble_sidewalk_ctx.adv_set.Advertising_Handle,
                                                          HCI_SET_ADV_DATA_OPERATION_COMPLETE,
                                                          SID_BLE_HCI_SET_ADV_DATA_FRAGMENT_PREF_NO_FRAGMENT,
                                                          sizeof(adv_bin_data), (uint8_t *)adv_bin_data);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("==>> hci_le_set_extended_advertising_data - fail, result: 0x%02X, set: 0x%02X", ble_status, sid_ble_sidewalk_ctx.adv_set.Advertising_Handle);
            err = SID_BLE_HCI_STATUS_UNKNOWN_ADV_IDENTIFIER == ble_status ? SID_ERROR_UNINITIALIZED : SID_ERROR_IO_ERROR; /* Advertisement set shall be created first via calling hci_le_set_extended_advertising_parameters */
            break;
        }
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> Success: hci_le_set_extended_advertising_data, set: 0x%02X", sid_ble_sidewalk_ctx.adv_set.Advertising_Handle);
#else
        /* Use legacy advertisement API */
        ble_status = hci_le_set_advertising_data(adv_pos, (uint8_t *)adv_bin_data);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("==>> hci_le_set_advertising_data - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> Success: hci_le_set_advertising_data");
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("==>> hci_le_set_scan_response_data - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> hci_le_set_scan_response_data - Success");

        /* Put scan response data into the host controller */
#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
        ble_status = hci_le_set_extended_scan_response_data(sid_ble_sidewalk_ctx.adv_set.Advertising_Handle,
                                                            HCI_SET_ADV_DATA_OPERATION_COMPLETE,
                                                            SID_BLE_HCI_SET_ADV_DATA_FRAGMENT_PREF_NO_FRAGMENT,
                                                            sizeof(sr_bin_data), (uint8_t *)sr_bin_data);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("==>> hci_le_set_extended_scan_response_data - fail, result: 0x%02X, set: 0x%02X", ble_status, sid_ble_sidewalk_ctx.adv_set.Advertising_Handle);
            err = SID_BLE_HCI_STATUS_UNKNOWN_ADV_IDENTIFIER == ble_status ? SID_ERROR_UNINITIALIZED : SID_ERROR_IO_ERROR; /* Advertisement set shall be created first via calling hci_le_set_extended_advertising_parameters */
            break;
        }
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> Success: hci_le_set_extended_scan_response_data, set: 0x%02X", sid_ble_sidewalk_ctx.adv_set.Advertising_Handle);
#else
        /* Use legacy scan response API */
        ble_status = hci_le_set_scan_response_data(sr_pos, (uint8_t*)sr_bin_data);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("==>> hci_le_set_scan_response_data - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> hci_le_set_scan_response_data - Success");
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_start_advertisement(void)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        sid_pal_ble_prv_adv_state_t new_adv_state;
        uint32_t                    adv_interval;

        SID_PAL_ASSERT(sid_ble_sidewalk_ctx.ble_cfg != NULL);

        sid_pal_enter_critical_region();
        err = SID_ERROR_NONE;

        /* Validate inputs */
        if (SID_STM32_BLE_HANDLE_INVALID_VALUE == sid_ble_sidewalk_ctx.ama_ctx.service_context.service_handle)
        {
            SID_PAL_LOG_ERROR("Sidewalk BLE advertisement request discarded - GATT is not initialized")
            err = SID_ERROR_UNINITIALIZED;
            /* Don't terminate yet */
        }

        /* Validate Sidewalk connection state */
        if (sid_ble_sidewalk_ctx.conn_ctx != NULL)
        {
            SID_PAL_LOG_WARNING("Sidewalk BLE advertisement request discarded - active Sidewalk connection present, handle: 0x%04X", sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id);
            err = SID_ERROR_INVALID_STATE;
            /* Don't terminate yet */
        }

        if (sid_ble_sidewalk_ctx.advertising_state != BADVS_ADVERTISEMENT_OFF)
        {
            SID_PAL_LOG_WARNING("Sidewalk BLE advertisement request discarded - advertising already");
            err = SID_ERROR_INVALID_STATE;
            /* Don't terminate yet */
        }
        sid_pal_exit_critical_region();

        /* Terminate if pre-launch validations failed */
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        if (sid_ble_sidewalk_ctx.ble_cfg->adv_param.fast_enabled != FALSE)
        {
            new_adv_state                         = BADVS_ADVERTISEMENT_FAST;
            adv_interval                          = sid_ble_sidewalk_ctx.ble_cfg->adv_param.fast_interval;
            sid_ble_sidewalk_ctx.adv_set.Duration = sid_ble_sidewalk_ctx.ble_cfg->adv_param.fast_timeout;
        }
        else if (sid_ble_sidewalk_ctx.ble_cfg->adv_param.slow_enabled != FALSE)
        {
            new_adv_state                         = BADVS_ADVERTISEMENT_SLOW;
            adv_interval                          = sid_ble_sidewalk_ctx.ble_cfg->adv_param.slow_interval;
            sid_ble_sidewalk_ctx.adv_set.Duration = sid_ble_sidewalk_ctx.ble_cfg->adv_param.slow_timeout;
        }
        else
        {
            SID_PAL_LOG_ERROR("Cannot start advertisement. Both fast and slow modes are disabled in configuration");
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Set advertisement parameters */
        err = _ble_adapter_sidewalk_set_adv_params(adv_interval);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_sidewalk_set_adv_params() */
            break;
        }

#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
        /* Update random address if required */
        if (SID_BLE_CFG_MAC_ADDRESS_TYPE_PUBLIC != sid_ble_sidewalk_ctx.ble_cfg->mac_addr_type)
        {
            sid_pal_ble_prv_bt_addr_buffer_t adv_bd_addr;

            switch (sid_ble_sidewalk_ctx.ble_cfg->mac_addr_type)
            {
                case SID_BLE_CFG_MAC_ADDRESS_TYPE_STATIC_RANDOM:
                    /* Load static random address for this device */
                    err = sid_stm32wba_ble_adapter_util_get_host_static_random_addr(&adv_bd_addr);
                    break;

                case SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_RESOLVABLE:
                case SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:
                default:
                    /* Generate a unique private random non-resolvable address for advertisement set. If RPA is used, this NRPA will be a back up address */
                    err = sid_stm32wba_ble_adapter_util_generate_private_nonresolvable_random_mac_address(&adv_bd_addr);
                    break;
            }

            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("  Fail   : Unable to set random device address, error code: %d", (int32_t)err);
                break;
            }

            /* Set address to be used by advertisement set */
            tBleStatus ble_status = hci_le_set_advertising_set_random_address(sid_ble_sidewalk_ctx.adv_set.Advertising_Handle, (uint8_t *)adv_bd_addr.bytes);
            if (ble_status != BLE_STATUS_SUCCESS)
            {
                SID_PAL_LOG_ERROR("==>> hci_le_set_advertising_set_random_address - fail, result: 0x%02X, set: 0x%02X", ble_status, sid_ble_sidewalk_ctx.adv_set.Advertising_Handle);
                err = SID_ERROR_IO_ERROR;
                break;
            }
            SID_BLE_SIDEWALK_LOG_DEBUG("==>> hci_le_set_advertising_set_random_address - Success, set: 0x%02X", sid_ble_sidewalk_ctx.adv_set.Advertising_Handle);
            SID_BLE_SIDEWALK_LOG_DEBUG("   Random Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x", adv_bd_addr.bytes[5],
                                                                                                     adv_bd_addr.bytes[4],
                                                                                                     adv_bd_addr.bytes[3],
                                                                                                     adv_bd_addr.bytes[2],
                                                                                                     adv_bd_addr.bytes[1],
                                                                                                     adv_bd_addr.bytes[0]);
        }
#else
        /* Update random address if required */
        if (SID_BLE_CFG_MAC_ADDRESS_TYPE_PUBLIC != sid_ble_sidewalk_ctx.ble_cfg->mac_addr_type)
        {
            err = sid_stm32wba_ble_adapter_prv_rotate_random_mac_address(NULL, sid_ble_sidewalk_ctx.ble_cfg->mac_addr_type);
            if (err != SID_ERROR_NONE)
            {
                SID_BLE_SIDEWALK_LOG_ERROR("  Fail   : unable to rotate BLE random address, error %d", (int32_t)err);
                break;
            }
            SID_BLE_SIDEWALK_LOG_DEBUG("  Success: rotate_random_mac_address");
        }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        /* Start advertising */
        err = sid_stm32wba_ble_adapter_prv_generic_start_advertisement(&sid_ble_sidewalk_ctx.adv_set);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by sid_stm32wba_ble_adapter_prv_generic_start_advertisement */
            break;
        }

        /* Indicate that we are advertising */
        sid_ble_sidewalk_ctx.advertising_state = new_adv_state;

        SID_PAL_LOG_INFO("Sidewalk BLE advertisement started in %s mode", BADVS_ADVERTISEMENT_FAST == new_adv_state ? "fast" : "slow");
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_stop_advertisement(void)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        if (BADVS_ADVERTISEMENT_OFF == sid_ble_sidewalk_ctx.advertising_state)
        {
            /* Advertising is not active */
            err = SID_ERROR_NONE;
            break;
        }

        err = sid_stm32wba_ble_adapter_prv_generic_stop_advertisement(sid_ble_sidewalk_ctx.adv_set.Advertising_Handle);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Advertisement is stopped */
        sid_ble_sidewalk_ctx.advertising_state = BADVS_ADVERTISEMENT_OFF;
#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0)
        if (osThreadGetId() != sid_ble_drv_ctx.advertising_cmd_task)
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */
        {
            SID_PAL_LOG_INFO("Sidewalk BLE advertisement stopped on request");
        }

        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_get_rssi(int8_t * rssi)
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus ble_status;
    int8_t raw_rssi;

    SID_PAL_ASSERT(rssi != NULL);

    do
    {
        uint16_t conn_handle = SID_STM32_BLE_HANDLE_INVALID_VALUE;

        sid_pal_enter_critical_region();
        if (sid_ble_sidewalk_ctx.conn_ctx != NULL)
        {
            /* Store a local copy of the connection handle because sid_ble_sidewalk_ctx.conn_ctx can be modified asynchronously at any point */
            conn_handle = sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id;
        }
        sid_pal_exit_critical_region();

        if (SID_STM32_BLE_HANDLE_INVALID_VALUE == conn_handle)
        {
            err = SID_ERROR_PORT_NOT_OPEN;
            break;
        }

        /* Read last known RSSI for Sidewalk connection */
        ble_status = hci_read_rssi(conn_handle, (uint8_t *)&raw_rssi);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Check RSSI raw value is valid */
        if (SID_BLE_HCI_INVALID_RSSI_VALUE == raw_rssi)
        {
            /* RSSI value cannot be read from HCI */
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Process raw value */
        *rssi = raw_rssi; /* If raw RSSI is valid it's already in dBm units and can be used directly */
        err = SID_ERROR_NONE;
    } while (0);

    if (err != SID_ERROR_NONE)
    {
        /* Set RSSI to invalid value for any reported error */
        *rssi = (int8_t)SID_BLE_HCI_INVALID_RSSI_VALUE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_get_tx_pwr(int8_t * tx_power)
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus ble_status;
    int8_t current_pwr;

    do
    {
        uint16_t conn_handle = SID_STM32_BLE_HANDLE_INVALID_VALUE;

        sid_pal_enter_critical_region();
        if (sid_ble_sidewalk_ctx.conn_ctx != NULL)
        {
            /* Store a local copy of the connection handle because sid_ble_sidewalk_ctx.conn_ctx can be modified asynchronously at any point */
            conn_handle = sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id;
        }
        sid_pal_exit_critical_region();

        if (SID_STM32_BLE_HANDLE_INVALID_VALUE == conn_handle)
        {
            err = SID_ERROR_PORT_NOT_OPEN;
            break;
        }

        ble_status = hci_read_transmit_power_level(conn_handle, 0x00u, (uint8_t *)&current_pwr);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("Failed to read current tx power 0x%x", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        SID_PAL_LOG_WARNING("Current Sidewalk Tx pwr: %d", (int8_t)current_pwr);
        *tx_power = current_pwr;
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_send_data(sid_ble_cfg_service_identifier_t id, uint8_t * data, uint16_t length)
{
    sid_error_t err = SID_ERROR_GENERIC;

    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks != NULL);
    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks->ind_callback != NULL);

    do
    {
        /* Ensure there's a valid connection */
        if (NULL == sid_ble_sidewalk_ctx.conn_ctx)
        {
            /* There's no active BLE connection */
            err = SID_ERROR_PORT_NOT_OPEN;
            break;
        }

        /* Validate inputs */
        if ((NULL == data) || (0u == length) || (length > SID_STM32_BLE_ATT_LEN_LIMIT(sid_ble_sidewalk_ctx.conn_ctx->public_ctx.mtu_size)))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        switch (id)
        {
            case AMA_SERVICE:
                {
                    /* Ensure proper state */
                    if ((NULL == sid_ble_sidewalk_ctx.conn_ctx)
                     || (FALSE == sid_ble_sidewalk_ctx.ama_ctx.is_notification_enabled)
                     || (SID_STM32_BLE_HANDLE_INVALID_VALUE == sid_ble_sidewalk_ctx.ama_ctx.service_context.service_handle)
                     || (SID_STM32_BLE_HANDLE_INVALID_VALUE == sid_ble_sidewalk_ctx.ama_ctx.outbox_char_handle))
                    {
                        err = SID_ERROR_PORT_NOT_OPEN;
                        break;
                    }

                    /* Allocate RAM buffer to temporarily store characteristic data */
                    uint8_t * databuf = (uint8_t *)malloc(length);
                    if (NULL == databuf)
                    {
                        err = SID_ERROR_OOM;
                        break;
                    }
                    SID_STM32_UTIL_fast_memcpy(databuf, data, length);

                    /* Enqueue characteristic data */
                    AMAMessageQueueElement_t msg_descriptor = {
                        .data   = databuf,
                        .length = length,
                    };
                    osStatus_t enqueue_status = osMessageQueuePut(ama_sendout_queue, &msg_descriptor, osPriorityNone, 0u);
                    if (enqueue_status != osOK)
                    {
                        SID_PAL_LOG_ERROR("==>> Unable to enqueue %d bytes message into AMA Send Out queue. Message will be lost", length);
                        free(databuf);

                        /* Notify Sidewalk stack about the failure */
                        sid_ble_sidewalk_ctx.event_callbacks->ind_callback(FALSE);

                        err = SID_ERROR_IO_ERROR;
                        break;
                    }

                    /* Done */
                    SID_PAL_LOG_DEBUG("==>> Successfully enqueued %d bytes message into AMA Send Out queue", length);
                    err = SID_ERROR_NONE;
                }
                break;

            case VENDOR_SERVICE:
                SID_PAL_LOG_ERROR("Vendor GATT services are not supported by this Sidewalk driver");
                err = SID_ERROR_NOSUPPORT;
                break;

            case LOGGING_SERVICE:
                SID_PAL_LOG_ERROR("Logging GATT service is not supported by this Sidewalk driver");
                err = SID_ERROR_NOSUPPORT;
                break;

            default:
                SID_PAL_LOG_ERROR("Unknown Sidewalk BLE service id: %u", (uint32_t)id);
                err = SID_ERROR_INVALID_ARGS;
                break;
        }

        /* Check if service-specific processing ended with an error */
        if (err != SID_ERROR_NONE)
        {
            if (AMA_SERVICE == id)
            {
                SID_PAL_LOG_ERROR("==>> Failed to write data to AMA Outbox. Error code: %d", (int32_t)err);
            }
            break;
        }

        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_set_callbacks(const sid_pal_ble_adapter_callbacks_t * cb)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Validate inputs */
        if (NULL == cb)
        {
            err = SID_ERROR_NULL_POINTER;
            break;
        }

        if ((NULL == cb->data_callback)
         || (NULL == cb->notify_callback)
         || (NULL == cb->conn_callback)
         || (NULL == cb->ind_callback)
         || (NULL == cb->mtu_callback)
         || (NULL == cb->adv_start_callback))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Set new callbacks */
        sid_pal_enter_critical_region();
        sid_ble_sidewalk_ctx.event_callbacks = cb;
        sid_pal_exit_critical_region();

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_set_tx_pwr(int8_t tx_power)
{
    // TODO: implement
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_sidewalk_disconnect(void)
{
    sid_error_t  err = SID_ERROR_GENERIC;
    tBleStatus   ble_status;

    /**
     * Store pointer to the connection context storage since sid_ble_sidewalk_ctx.conn_ctx can be invalidated right after aci_gap_terminate()
     * before sid_stm32wba_ble_adapter_prv_gap_cmd_resp_wait() is reached
     */
    const sid_pal_ble_prv_connection_ctx_t * const tmp_conn_ctx = sid_ble_sidewalk_ctx.conn_ctx;

    sid_pal_enter_critical_region();
    do
    {
        /* Validate state */
        if (SID_STM32_BLE_HANDLE_INVALID_VALUE == sid_ble_sidewalk_ctx.ama_ctx.service_context.service_handle)
        {
            err = SID_ERROR_UNINITIALIZED;
            break;
        }

        if (NULL == sid_ble_sidewalk_ctx.conn_ctx)
        {
            /* Already disconnected */
            err = SID_ERROR_PORT_NOT_OPEN;
            break;
        }

        /* Prepare to wait for disconnection completion */
        sid_stm32wba_ble_adapter_prv_gap_cmd_arm_resp_wait(sid_ble_sidewalk_ctx.conn_ctx);

        /* Initiate disconnect */
        ble_status = aci_gap_terminate(sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("==>> aci_gap_terminate - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> aci_gap_terminate - Success");

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);
    sid_pal_exit_critical_region();

    if (SID_ERROR_NONE == err)
    {
        /* Wait for disconnect to complete */
        err = sid_stm32wba_ble_adapter_prv_gap_cmd_resp_wait(tmp_conn_ctx);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to wait for Sidewalk BLE disconnect completion. Error %d", (int32_t)err);
        }
    }

    /* For Sidewalk stack invoking this method if there's no active connection is not an error */
    if (SID_ERROR_PORT_NOT_OPEN == err)
    {
        err = SID_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _ble_adapter_sidewalk_deinit(void)
{
    sid_error_t err;
    sid_error_t gatt_deinit_err;
    sid_error_t rtos_resource_deinit_err;
    sid_error_t generic_deinit_err;
    osStatus_t  os_status;
    uint32_t    ll_lock_acquired;

    SID_BLE_SIDEWALK_LOG_DEBUG("==>> Start ble_adapter_sidewalk_deinit function");

    /* Trigger disconnect. We don't care if there's anything connected or if this call will succeed */
    (void)_ble_adapter_sidewalk_disconnect();

    /* Wait for any potentially ongoing BLE operations to stop */
    os_status = osMutexAcquire(LinkLayerMutex, SID_STM32_BLE_LL_MUTEX_WAIT_TIMEOUT_ON_DEINIT_TICKS);

    /* Print out warning but proceed */
    if (os_status != osOK)
    {
        SID_PAL_LOG_WARNING("Failed to acquire BLE LL mutex, race conditions are possible during Sidewalk BLE deinit");
        ll_lock_acquired = FALSE;
    }
    else
    {
        ll_lock_acquired = TRUE;
    }

    /*----------------------------------------------------------------------------*/

#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
    /* Delete Sidewalk advertisement set */
    tBleStatus ble_status = hci_le_remove_advertising_set(sid_ble_sidewalk_ctx.adv_set.Advertising_Handle);
    if (ble_status != BLE_STATUS_SUCCESS)
    {
        SID_PAL_LOG_WARNING("Failed to remove Sidewalk advertising set. Status 0x%02X", ble_status);
    }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

    /* Clean up AMA GATT service resources - this will also render AMA service unusable, preventing any further connections to it */
    gatt_deinit_err = _ble_adapter_sidewalk_services_deinit();
    if (gatt_deinit_err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_WARNING("Failed to deinitialize Sidewalk BLE GATT. Memory leak is possible");
    }

    /*----------------------------------------------------------------------------*/

    /* Now it is safe to deinitialize the associated RTOS resources */
    rtos_resource_deinit_err = SID_ERROR_NONE;

    if (ama_sendout_task != NULL)
    {
        os_status = osThreadTerminate(ama_sendout_task);
        ama_sendout_task = NULL;
        if (os_status != osOK)
        {
            SID_PAL_LOG_ERROR("Failed to terminate Sidewalk AMA Thread. Error 0x%08X", (uint32_t)os_status);
            rtos_resource_deinit_err = SID_ERROR_UNINITIALIZED;
            /* Continue deinitialization because we still need to deallocate the other resources */
        }
    }

    if (ama_sendout_queue != NULL)
    {
        os_status = osMessageQueueDelete(ama_sendout_queue);
        ama_sendout_queue = NULL;
        if (os_status != osOK)
        {
            SID_PAL_LOG_ERROR("Failed to delete Sidewalk AMA Message Queue. Error 0x%08X", (uint32_t)os_status);
            rtos_resource_deinit_err = SID_ERROR_UNINITIALIZED;
            /* Continue deinitialization because we still need to deallocate the other resources */
        }
    }

    if (rtos_resource_deinit_err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_WARNING("Failed to deinitialize Sidewalk BLE RTOS resources. Memory leak is possible");
    }

    /*----------------------------------------------------------------------------*/

    generic_deinit_err = sid_stm32wba_ble_adapter_prv_generic_deinit();
    /* Logs provided by sid_stm32wba_ble_adapter_prv_generic_deinit */

    /* Release LL lock */
    if ((ll_lock_acquired != FALSE) && (LinkLayerMutex != NULL))
    {
        os_status = osMutexRelease(LinkLayerMutex);
        if (os_status != osOK)
        {
            SID_PAL_LOG_ERROR("Failed to release BLE LL mutex. BLE stack will be blocked till reset");
            rtos_resource_deinit_err = SID_ERROR_UNRECOVERABLE;
        }
    }

    /**
     * Select error with following priorities (highest to lowest)
     * 1. Generic deinitialization errors
     * 2. BLE GATT deinit errors
     * 3. RTOS resource deallocation errors
     */
    err = generic_deinit_err != SID_ERROR_NONE ? generic_deinit_err : (gatt_deinit_err != SID_ERROR_NONE ? gatt_deinit_err : rtos_resource_deinit_err);

    sid_ble_sidewalk_ctx.init_done = FALSE;

    SID_BLE_SIDEWALK_LOG_DEBUG("==>> End ble_adapter_sidewalk_deinit function");

    return err;
}

/* Private shared function definitions ---------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_ble_adapter_create(sid_pal_ble_adapter_interface_t * handle)
{
    sid_error_t err = SID_ERROR_GENERIC;

    /**
     * WARNING: This method is intended to be used by the Sidewalk stack only. Using it in the user application
     *          directly may result in undefined behavior.
     * 
     */

    do
    {
        if (NULL == handle)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        *handle = (sid_pal_ble_adapter_interface_t)&sidwalk_ble_ifc;
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_sidewalk_on_adv_timeout(void)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Check if restart is needed */
        if ((BADVS_ADVERTISEMENT_FAST == sid_ble_sidewalk_ctx.advertising_state) && (sid_ble_sidewalk_ctx.ble_cfg->adv_param.slow_enabled != FALSE))
        {
            /* Fast advertisement timed out and slow advertisement is allowed - proceed with slow advertisement */

            /* Indicate the advertisement is stopped in case restart will fail */
            sid_ble_sidewalk_ctx.advertising_state = BADVS_ADVERTISEMENT_OFF;

            /* Update advertisement duration */
            sid_ble_sidewalk_ctx.adv_set.Duration = sid_ble_sidewalk_ctx.ble_cfg->adv_param.slow_timeout;

            /* Apply advertisement parameters for slow advertisement */
            err = _ble_adapter_sidewalk_set_adv_params(sid_ble_sidewalk_ctx.ble_cfg->adv_param.slow_interval);
            if (err != SID_ERROR_NONE)
            {
                /* Logs provided by _ble_adapter_sidewalk_set_adv_params() */
                break;
            }

            /* Restart advertisement */
            err = sid_stm32wba_ble_adapter_prv_generic_start_advertisement(&sid_ble_sidewalk_ctx.adv_set);
            if (err != SID_ERROR_NONE)
            {
                /* Logs provided by sid_stm32wba_ble_adapter_prv_generic_start_advertisement */
                break;
            }

            /* Indicate that we are advertising */
            sid_ble_sidewalk_ctx.advertising_state = BADVS_ADVERTISEMENT_SLOW;

            SID_PAL_LOG_INFO("Sidewalk BLE advertisement switched to slow mode");
        }
        else
        {
            /* No more advertisements - final timeout */
            sid_ble_sidewalk_ctx.advertising_state = BADVS_ADVERTISEMENT_OFF;
            SID_PAL_LOG_INFO("Sidewalk BLE advertisement terminated due to timeout");
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_sidewalk_on_ble_connected(sid_pal_ble_prv_connection_ctx_t * const conn_ctx)
{
    sid_error_t err;
    tBleStatus ble_status;

    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks != NULL);
    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks->conn_callback != NULL);
    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks->mtu_callback != NULL);

    do
    {
        sid_pal_enter_critical_region();

        /* Store connection context pointer */
        sid_ble_sidewalk_ctx.conn_ctx = conn_ctx;

        /* Set the MTU limit specific to the Sidewalk link */
        sid_ble_sidewalk_ctx.conn_ctx->public_ctx.max_mtu_size = sid_ble_drv_ctx.cfg->sidewalk_profile.max_att_mtu;

        /* Update logical statuses */
        sid_ble_sidewalk_ctx.advertising_state = BADVS_ADVERTISEMENT_OFF;

        /* Notify Sidewalk stack about the established connection */
        sid_ble_sidewalk_ctx.event_callbacks->conn_callback(TRUE, sid_ble_sidewalk_ctx.conn_ctx->public_ctx.peer_identity);

        /* Set initial MTU in the Sidewalk stack */
        sid_ble_sidewalk_ctx.event_callbacks->mtu_callback(sid_ble_sidewalk_ctx.conn_ctx->public_ctx.mtu_size);

        sid_pal_exit_critical_region();

#if SID_STM32_BLE_GATT_EXCHANGE_CONFIG_SUPPORTED
        /* Exchange peripheral config to negotiate Sidewalk-specific MTU size */
        ble_status = aci_gatt_exchange_config(sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("aci_gatt_exchange_config - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> aci_gatt_exchange_config - Success");
#else
        /* This command is not supported by the peripheral-only version of the stack */
#endif /* SID_STM32_BLE_GATT_EXCHANGE_CONFIG_SUPPORTED */

        /* Update connection parameters */
        err = sid_stm32wba_ble_adapter_sidewalk_on_ble_conn_params_changed();
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Enable data length extensions (DLE) for Sidewalk link */
        ble_status = hci_le_set_data_length(sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id, SID_STM32_BLE_DLE_MAX_TX_OCTETS, SID_STM32_BLE_DLE_MAX_TX_TIME_US);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("hci_le_set_data_length - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> hci_le_set_data_length - Success");

        /* Request switch to 2Mbit PHY (if supported by the peer) */
        ble_status = hci_le_set_phy(sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id, 0x00, HCI_TX_PHYS_LE_2M_PREF, HCI_RX_PHYS_LE_2M_PREF, 0x00u);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_SIDEWALK_LOG_ERROR("hci_le_set_default_phy - fail, result: 0x%02X", ble_status);
        }
        SID_BLE_SIDEWALK_LOG_DEBUG("==>> hci_le_set_default_phy - Success");

        /* Done */
        SID_PAL_LOG_INFO("Sidewalk BLE connection established (0x%04X)", sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id);
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_stm32wba_ble_adapter_sidewalk_on_ble_disconnected(const uint8_t reason)
{
    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks != NULL);
    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks->conn_callback != NULL);

    sid_pal_enter_critical_region();

    /* Notify Sidewalk stack about the disconnection */
    sid_ble_sidewalk_ctx.event_callbacks->conn_callback(FALSE, sid_ble_sidewalk_ctx.conn_ctx->public_ctx.peer_identity);

    /* Invalidate context */
    sid_ble_sidewalk_ctx.conn_ctx = NULL;

    sid_pal_exit_critical_region();

    SID_PAL_LOG_INFO("Sidewalk BLE connection terminated with reason 0x%02X", reason);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_stm32wba_ble_adapter_sidewalk_on_ble_mtu_changed(void)
{
    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks != NULL);
    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.event_callbacks->mtu_callback != NULL);

    /* Notify Sidewalk stack about the MTU change */
    sid_ble_sidewalk_ctx.event_callbacks->mtu_callback(sid_ble_sidewalk_ctx.conn_ctx->public_ctx.mtu_size);

    SID_PAL_LOG_INFO("Sidewalk BLE MTU changed to %u bytes", sid_ble_sidewalk_ctx.conn_ctx->public_ctx.mtu_size);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_sidewalk_on_ble_conn_params_update_req(const sid_ble_ext_proposed_conn_params_t * const proposed_params,
                                                                                                      sid_ble_ext_accepted_conn_params_t * const out_accepted_params, tBleStatus * const out_le_resp_code)
{
    sid_error_t err;

    do
    {
        err = sid_stm32wba_ble_adapter_util_validate_proposed_conn_params(proposed_params, &sidewalk_conn_limits, out_accepted_params, out_le_resp_code);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_sidewalk_on_ble_conn_params_changed(void)
{
    sid_error_t err;
    tBleStatus  ble_status;
    uint32_t    conn_params_ok;
    uint16_t    req_conn_interval_min;
    uint16_t    req_conn_interval_max;
    uint16_t    req_conn_latency;
    uint16_t    req_conn_timeout;

    static uint32_t unacceptable_params_counter = 0u;

    SID_PAL_ASSERT(sid_ble_sidewalk_ctx.conn_ctx != NULL);

    /* Use new connection parameters as defaults for possible connection update request */
    sid_ble_ext_proposed_conn_params_t proposed_params = {
        .interval_min = sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_interval,
        .interval_max = sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_interval,
        .latency_max  = sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_latency,
        .timeout      = sid_ble_sidewalk_ctx.conn_ctx->public_ctx.supervision_timeout,
    };

    /* Storage for the negotiated connection parameters */
    sid_ble_ext_accepted_conn_params_t accepted_params;

    do
    {
        /* Validate new connection parameters */
        err = sid_stm32wba_ble_adapter_util_validate_proposed_conn_params(&proposed_params, &sidewalk_conn_limits, &accepted_params, &ble_status);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        if (ble_status != BLE_STATUS_SUCCESS)
        {
            /* New parameters are unacceptable at all */
            SID_BLE_SIDEWALK_LOG_WARNING("Unacceptable Sidewalk BLE connection parameters - requesting parameter update");

            conn_params_ok        = FALSE;
            req_conn_interval_min = sid_ble_sidewalk_ctx.ble_cfg->conn_param.min_conn_interval;
            req_conn_interval_max = sid_ble_sidewalk_ctx.ble_cfg->conn_param.max_conn_interval;
            req_conn_latency      = sid_ble_sidewalk_ctx.ble_cfg->conn_param.slave_latency;
            req_conn_timeout      = sid_ble_sidewalk_ctx.ble_cfg->conn_param.conn_sup_timeout;
        }
        else
        {
            if ((proposed_params.timeout != accepted_params.timeout) || (proposed_params.interval_min != accepted_params.interval_min)
             || (proposed_params.interval_max != accepted_params.interval_max) || (proposed_params.latency_max != accepted_params.latency_max))
            {
                /* New connection parameters are generally ok, but some adjustments are required */
                SID_BLE_SIDEWALK_LOG_DEBUG("Suboptimal Sidewalk BLE connection parameters - requesting parameter update");

                conn_params_ok         = FALSE;
                req_conn_interval_min = accepted_params.interval_min;
                req_conn_interval_max = accepted_params.interval_max;
                req_conn_latency      = accepted_params.latency_max;
                req_conn_timeout      = accepted_params.timeout;
            }
            else
            {
                /* We are fine with the update params */
                conn_params_ok = TRUE;
            }
        }

        if (FALSE == conn_params_ok)
        {

            /* Check how many times in a row we got unacceptable params - terminate the connection if we are stuck in a parameter update loop */
            unacceptable_params_counter++;
            if (unacceptable_params_counter > SID_BLE_UNACCEPTABLE_CONN_PARAMS_UPDATE_LIMIT)
            {
                /* Limit reached, we were provided with unacceptable connection params too many times in a row */
                SID_PAL_LOG_ERROR("BLE central enforces unacceptable conn params for Sidewalk link - terminating the connection");
                unacceptable_params_counter = 0u;
                err = SID_ERROR_INCOMPATIBLE_PARAMS;
                break;
            }

            ble_status = aci_l2cap_connection_parameter_update_req(sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id,
                                                                   req_conn_interval_min,
                                                                   req_conn_interval_max,
                                                                   req_conn_latency,
                                                                   req_conn_timeout);
            if (ble_status != BLE_STATUS_SUCCESS)
            {
                SID_BLE_SIDEWALK_LOG_ERROR("aci_l2cap_connection_parameter_update_req - fail, result: 0x%02X", ble_status);
                err = SID_ERROR_IO_ERROR;
                break;
            }
            SID_BLE_SIDEWALK_LOG_DEBUG("==>> aci_l2cap_connection_parameter_update_req - Success");
        }
        else
        {
            /* Parameters are within an acceptable range, reset the counter */
            unacceptable_params_counter = 0u;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}
