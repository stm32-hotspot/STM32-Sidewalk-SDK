/**
  ******************************************************************************
  * @file    sid_pal_ble_adapter_stm32wba.c
  * @brief   Sidewalk BLE driver for STM32WBA host MCU
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

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "bluetooth_hci_defs.h"
#include "host_ble.h"
#include "sid_pal_ble_adapter_stm32wba.h"
#include "sid_pal_ble_adapter_stm32wba_config.h"
#include "sid_pal_ble_adapter_stm32wba_ext_ifc.h"
#include "sid_pal_ble_adapter_stm32wba_private_defs.h"
#include "sid_pal_ble_adapter_stm32wba_sidewalk_gap_gatt.h"
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
#  include "sid_pal_ble_adapter_stm32wba_user_gap_gatt.h"
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
#include "sid_pal_ble_adapter_stm32wba_utils.h"

/* Sidewalk interfaces */
#include <sid_pal_ble_adapter_ifc.h>
#include <sid_ble_config_ifc.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>

/* BLE stack */
#include <advanced_memory_manager.h>
#include <ble_core.h>
#include <ble_const.h>
#include <blestack.h>
#include <host_stack_if.h>
#include <ll_sys.h>
#include "ll_sys_if.h"
#include <ll_sys_startup.h>

/* BLE stack dependencies  */
#include <ble_timer.h>
#include <flash_driver.h>
#include <hw.h>
#include <simple_nvm_arbiter.h>

/* Platform-specific headers */
#include "stm32_rtos.h"
#include <stm32wbaxx_hal.h>

/* Utilities and helpers */
#include <sid_stm32_common_utils.h>
#include <stm_list.h>
#include <stm32_timer.h>

/* Private defines -----------------------------------------------------------*/

#define BLE_HAL_ENABLE_RADIO_ACTIVITY_EVENT_ADVERTISING             (0x0002u)
#define BLE_HAL_ENABLE_RADIO_ACTIVITY_EVENT_CONNECTION              (0x0004u)

#define SID_STM32_BLE_PRV_CI_GAP_SECURITY_CFG_FIXED_PIN_ENABLED     (0x00u)
#define SID_STM32_BLE_PRV_CI_GAP_SECURITY_CFG_FIXED_PIN_DISABLED    (0x01u)

#ifndef SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING
#  error "SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING is not defined. Please set it explicitly"
#endif /* SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING */

#if SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING
#  define SID_STM32_BLE_PRV_L2CAP_PDU_SZ(_PDU_)                     (sizeof(sid_pal_ble_prv_l2cap_pdu_t) - SID_STM32_UTIL_STRUCT_MEMBER_SIZE(sid_pal_ble_prv_att_pdu_t, raw) + SID_STM32_UTIL_STRUCT_MEMBER_SIZE(sid_pal_ble_prv_att_pdu_t, _PDU_))
#  define SID_STM32_BLE_PRV_ATT_EXCHANGE_MTU_REQ_SZ                 (SID_STM32_BLE_PRV_L2CAP_PDU_SZ(exchange_mtu_req))
#  define SID_STM32_BLE_PRV_ATT_EXCHANGE_MTU_RSP_SZ                 (SID_STM32_BLE_PRV_L2CAP_PDU_SZ(exchange_mtu_rsp))
#  define SID_STM32_BLE_PRV_ATT_EXCHANGE_MTU_REQ_OPCODE             (0x02u) /* As per ATT protocol specification in Core Specification Volume 3, Part F */
#  define SID_STM32_BLE_PRV_ATT_EXCHANGE_MTU_RSP_OPCODE             (0x03u) /* As per ATT protocol specification in Core Specification Volume 3, Part F */
#endif /* SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING */

#define SID_STM32_BLE_PRV_RANDOM_ADDR_SPECIFIER_MASK                (0xC0u)
#define SID_STM32_BLE_PRV_RANDOM_ADDR_TYPE_RPA_MASK                 (0x40u)

#define SID_STM32_BLE_PRV_ADVERTISING_CMD_QUEUE_LEN                 (2u)

#define SID_STM32_WPAN_STACK_EXPECTED_BUILD_NUMBER                  (0x100Fu) /* Version information reported by the STM32_WPAN stack v2.7.0 */

/* Private macro -------------------------------------------------------------*/

#ifndef SID_BLE_GENERIC_EXTRA_LOGGING
/* Set SID_BLE_GENERIC_EXTRA_LOGGING to 1 to enable extended logs */
#  define SID_BLE_GENERIC_EXTRA_LOGGING                             (0)
#endif

#if SID_BLE_GENERIC_EXTRA_LOGGING
#  define SID_BLE_GENERIC_LOG_ERROR(...)                            SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SID_BLE_GENERIC_LOG_WARNING(...)                          SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SID_BLE_GENERIC_LOG_INFO(...)                             SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SID_BLE_GENERIC_LOG_DEBUG(...)                            SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SID_BLE_GENERIC_LOG_TRACE(...)                            SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SID_BLE_GENERIC_LOG_ERROR(...)                            ((void)0u)
#  define SID_BLE_GENERIC_LOG_WARNING(...)                          ((void)0u)
#  define SID_BLE_GENERIC_LOG_INFO(...)                             ((void)0u)
#  define SID_BLE_GENERIC_LOG_DEBUG(...)                            ((void)0u)
#  define SID_BLE_GENERIC_LOG_TRACE(...)                            ((void)0u)
#endif

#define BLE_ADAPTER_DBM_TO_INT(__X__)                               ((int16_t)((float)(__X__) * 100.f))

#define BLE_ADAPTER_MS_TO_OS_TICKS(__MS__)                          ((((__MS__) * osKernelGetTickFreq()) + 999u) / 1000u)                            /*!< Convert milliseconds to RTOS ticks with math ceiling */
#define BLE_ADAPTER_OS_TICKS_TO_MS(__TICKS__)                       ((((__TICKS__) * 1000u) + (osKernelGetTickFreq() - 1u)) / osKernelGetTickFreq()) /*!< Convert RTOS ticks to milliseconds with math ceiling */

#define BLE_ADAPTER_CALCULATE_DISCONNECT_TIMEOUT_MS(_CONN_CTX_)     (SID_STM32_BLE_CONN_SUPERVISION_UNITS_TO_MS((_CONN_CTX_)->public_ctx.supervision_timeout) + SID_STM32_BLE_DISCONNECT_TIMEOUT_ALLOWANCE_MS)

/* Private typedef -----------------------------------------------------------*/

#if SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING

/* L2CAP header definition */
typedef __PACKED_STRUCT {
    uint8_t  code;
    uint8_t  identifier;
    uint16_t length;
} sid_pal_ble_prv_l2cap_header_t;

/* ATT protocol data for ATT_EXCHANGE_MTU_REQ PDU */
typedef __PACKED_STRUCT {
    uint16_t client_rx_mtu;
} sid_pal_ble_prv_att_exchange_mtu_req_t;

/* ATT protocol data for ATT_EXCHANGE_MTU_RSP PDU */
typedef __PACKED_STRUCT {
    uint16_t server_rx_mtu;
} sid_pal_ble_prv_att_exchange_mtu_rsp_t;

/* Generic definition of ATT protocol PDU */
typedef __PACKED_STRUCT {
    uint8_t opcode;
    __PACKED_UNION {
        uint8_t                                raw[246]; /* 22 bytes without data length extensions */
        sid_pal_ble_prv_att_exchange_mtu_req_t exchange_mtu_req;
        sid_pal_ble_prv_att_exchange_mtu_rsp_t exchange_mtu_rsp;
    };
} sid_pal_ble_prv_att_pdu_t;

/* L2CAP PDU definition */
typedef __PACKED_STRUCT {
    sid_pal_ble_prv_l2cap_header_t header;
    sid_pal_ble_prv_att_pdu_t      att_pdu;
} sid_pal_ble_prv_l2cap_pdu_t;

/* Mock of the BLE stack-specific data frame object */
typedef __PACKED_STRUCT {
    uint32_t                      __dummy0;
    uint16_t                      frame_len;   /*!< Full length of the L2CAP frame, including L2CAP header */
    uint16_t                      __dummy1;
    uint32_t                      __dummy2[6];
    sid_pal_ble_prv_l2cap_pdu_t * l2cap_frame;
} sid_pal_ble_prv_client_pdu_mock_t;

/* Mock of the BLE stack-specific connection context */
typedef __PACKED_STRUCT {
    uint16_t conn_handle; /*!< BLE connection handle for this packet*/
    uint16_t mtu_size;    /*!< Currently applied MTU */
} sid_pal_ble_prv_conn_ctx_mock_t;

#endif /* SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING */

/* Characteristic definition (BLE stack internal format) */
typedef __PACKED_STRUCT
{
    uint16_t    Service_Handle;
    uint8_t     Char_UUID_Type;
    uint8_t     : 8; /* Padding */
    Char_UUID_t Char_UUID;
    uint16_t    Char_Value_Length;
    uint8_t     Char_Properties;
    uint8_t     Security_Permissions;
    uint8_t     GATT_Evt_Mask;
    uint8_t     Enc_Key_Size;
    uint8_t     Is_Variable;
} gatt_add_char_core_params_t;

/* Exported variables --------------------------------------------------------*/

osSemaphoreId_t BleHostSemaphore = NULL; /* The name of this semaphore shall be aligned with the BLE stack / CubeMX FW pack (host_stack_if.c) */

/* Private variables ---------------------------------------------------------*/

sid_pal_ble_adapter_ctx_t sid_ble_drv_ctx                       = {0};

static osThreadId_t       ble_host_task                         = NULL;
static osThreadId_t       hci_async_evt_task                    = NULL;
static osSemaphoreId_t    hci_async_evt_semaphore               = NULL;
static tListNode          BleAsynchEventQueue                   = {0};
static uint64_t           host_nvm_buffer[CFG_BLE_NVM_SIZE_MAX] = {0};

static AMM_VirtualMemoryCallbackFunction_t ble_amm_cb = {0};

/* Private constants ---------------------------------------------------------*/

static const osSemaphoreAttr_t ble_host_semaphore_atributes = {
    .name       = "BLE Host Semaphore",
    .attr_bits  = SEMAPHORE_DEFAULT_ATTR_BITS,
    .cb_mem     = SEMAPHORE_DEFAULT_CB_MEM,
    .cb_size    = SEMAPHORE_DEFAULT_CB_SIZE,
};

static const osThreadAttr_t ble_host_task_atributes = {
    .name       = "BLE Host Task",
    .priority   = TASK_PRIO_BLE_HOST,
    .stack_size = TASK_STACK_SIZE_BLE_HOST,
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
    .stack_mem  = TASK_DEFAULT_STACK_MEM,
};

static const osThreadAttr_t hci_async_evt_task_attributes = {
    .name       = "HCI async Event Task",
    .priority   = TASK_PRIO_HCI_ASYNC_EVENT,
    .stack_size = TASK_STACK_SIZE_HCI_ASYNC_EVENT,
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
    .stack_mem  = TASK_DEFAULT_STACK_MEM,
};

static const osSemaphoreAttr_t hci_async_evt_semaphore_attributes = {
    .name       = "HCI async Event Semaphore",
    .attr_bits  = SEMAPHORE_DEFAULT_ATTR_BITS,
    .cb_mem     = SEMAPHORE_DEFAULT_CB_MEM,
    .cb_size    = SEMAPHORE_DEFAULT_CB_SIZE,
};

#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0)
static const osThreadAttr_t advertising_cmd_task_attributes = {
    .name       = "Advertising Cmd Task",
    .priority   = osPriorityBelowNormal,
    .stack_size = 1216u,
    .attr_bits  = TASK_DEFAULT_ATTR_BITS,
    .cb_mem     = TASK_DEFAULT_CB_MEM,
    .cb_size    = TASK_DEFAULT_CB_SIZE,
    .stack_mem  = TASK_DEFAULT_STACK_MEM,
};

static const osMessageQueueAttr_t advertising_cmd_queue_attributes = {
    .name       = "Advertising Cmd Queue",
    .attr_bits  = QUEUE_DEFAULT_ATTR_BITS,
    .cb_mem     = QUEUE_DEFAULT_CB_MEM,
    .cb_size    = QUEUE_DEFAULT_CB_SIZE,
    .mq_mem     = QUEUE_DEFAULT_MQ_MEM,
    .mq_size    = QUEUE_DEFAULT_MQ_SIZE,
};
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

/* Imported variables --------------------------------------------------------*/

extern sid_pal_ble_sidewalk_link_ctx_t sid_ble_sidewalk_ctx;

/* Imported function prototypes ----------------------------------------------*/

extern tBleStatus GAP_Set_Own_Address(uint8_t own_addr_type_req, uint8_t * const out_own_addr_type, uint8_t addr[6]);

/* Private function prototypes -----------------------------------------------*/

static inline const char *                       _ble_adapter_generic_aci_warning_display_str(const uint8_t warning_type);
static inline const char *                       _ble_adapter_generic_peer_addr_type_display_str(const sid_pal_ble_ext_peer_addr_type_t addr_type, const uint8_t addr_msb);
static inline const char *                       _ble_adapter_generic_phy_type_display_str(const sid_pal_ble_ext_le_phy_t phy_type);

static        void                               _ble_adapter_generic_resume_flow_process(void);
static        void                               BLE_NvmCallback(SNVMA_Callback_Status_t CbkStatus);
static inline void                               _gap_cmd_resp_release(const sid_pal_ble_prv_connection_ctx_t * const conn_ctx);
static        void                               BleStack_Process_BG_Entry(void* thread_input);
static inline void                               BleStack_Process_BG(void);
static        void                               Ble_UserEvtRx_Entry(void* thread_input);
static        void                               Ble_UserEvtRx(void);

static inline void                               _ble_adapter_generic_svc_event_handling_init(void);
static        SVCCTL_EvtAckStatus_t              _ble_adapter_generic_svc_event_router(void * p_evt);
static inline SVCCTL_UserEvtFlowStatus_t         _ble_adapter_generic_process_hci_le_meta_evt(const evt_le_meta_event * const evt);
static inline SVCCTL_UserEvtFlowStatus_t         _ble_adapter_generic_process_aci_evt(const evt_blecore_aci * const evt);

#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0)
static        void                               _ble_adapter_generic_adv_duration_timer_cb(void *arg);
static        void                               _ble_adapter_generic_adv_cmd_task_entry(void* thread_input);
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

static inline sid_error_t                        _ble_adapter_generic_conn_ctxs_init(void);
static inline void                               _ble_adapter_generic_conn_ctxs_deinit(void);
static inline void                               _ble_adapter_generic_invalidate_conn_ctx(sid_pal_ble_prv_connection_ctx_t * const ctx);
static inline sid_pal_ble_prv_connection_ctx_t * _ble_adapter_generic_get_free_conn_ctx(const uint16_t conn_handle);
static inline sid_pal_ble_prv_connection_ctx_t * _ble_adapter_generic_get_conn_ctx_for_handle(const uint16_t conn_handle);

static        sid_error_t                        _ble_stack_init(const sid_pal_ble_prv_operating_mode_t init_type);
static        sid_error_t                        _reload_bonded_devices(void);
static        sid_error_t                        _gap_and_gatt_params_init(const sid_pal_ble_prv_operating_mode_t init_type);

/* Private constants ---------------------------------------------------------*/

/**
 * @brief Look-up table to convert HW-speciifc Tx power indexes to the physical values in dBm
 */
static const int16_t ble_tx_pwr_to_dbm_lut[] = {
    BLE_ADAPTER_DBM_TO_INT(-20.4),
    BLE_ADAPTER_DBM_TO_INT(-20.4),
    BLE_ADAPTER_DBM_TO_INT(-20.4),
    BLE_ADAPTER_DBM_TO_INT(-20.4),
    BLE_ADAPTER_DBM_TO_INT(-20.4),
    BLE_ADAPTER_DBM_TO_INT(-20.4),
    BLE_ADAPTER_DBM_TO_INT(-19.4),
    BLE_ADAPTER_DBM_TO_INT(-18.3),
    BLE_ADAPTER_DBM_TO_INT(-17.3),
    BLE_ADAPTER_DBM_TO_INT(-16.4),
    BLE_ADAPTER_DBM_TO_INT(-15.4),
    BLE_ADAPTER_DBM_TO_INT(-14.3),
    BLE_ADAPTER_DBM_TO_INT(-13.3),
    BLE_ADAPTER_DBM_TO_INT(-12.3),
    BLE_ADAPTER_DBM_TO_INT(-11.4),
    BLE_ADAPTER_DBM_TO_INT(-10.4),
    BLE_ADAPTER_DBM_TO_INT(-9.5),
    BLE_ADAPTER_DBM_TO_INT(-8.4),
    BLE_ADAPTER_DBM_TO_INT(-7.5),
    BLE_ADAPTER_DBM_TO_INT(-6.5),
    BLE_ADAPTER_DBM_TO_INT(-5.4),
    BLE_ADAPTER_DBM_TO_INT(-4.5),
    BLE_ADAPTER_DBM_TO_INT(-3.5),
    BLE_ADAPTER_DBM_TO_INT(-2.4),
    BLE_ADAPTER_DBM_TO_INT(-1.5),
    BLE_ADAPTER_DBM_TO_INT(-0.3),
    BLE_ADAPTER_DBM_TO_INT(+0.9),
    BLE_ADAPTER_DBM_TO_INT(+2.3),
    BLE_ADAPTER_DBM_TO_INT(+2.8),
    BLE_ADAPTER_DBM_TO_INT(+3.9),
    BLE_ADAPTER_DBM_TO_INT(+4.8),
    BLE_ADAPTER_DBM_TO_INT(+5.6),
    BLE_ADAPTER_DBM_TO_INT(+6.9),
    BLE_ADAPTER_DBM_TO_INT(+7.5),
    BLE_ADAPTER_DBM_TO_INT(+8.5),
    BLE_ADAPTER_DBM_TO_INT(+10),
};

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline const char * _ble_adapter_generic_aci_warning_display_str(const uint8_t warning_type)
{
    const char * warning_type_str;

    switch (warning_type)
    {
        case WARNING_L2CAP_RECOMBINATION_FAILURE:
            warning_type_str = "L2CAP recombination failure";
            break;

        case WARNING_GATT_UNEXPECTED_PEER_MESSAGE:
            warning_type_str = "GATT unexpected peer message";
            break;

        case WARNING_NVM_ALMOST_FULL:
            warning_type_str = "NVM almost full";
            break;

        case WARNING_COC_RX_DATA_LENGTH_TOO_LARGE:
            warning_type_str = "COC RX data length too large";
            break;

        case WARNING_COC_ALREADY_ASSIGNED_DCID:
            warning_type_str = "COC already assigned DCID";
            break;

        case WARNING_SMP_UNEXPECTED_LTK_REQUEST:
            warning_type_str = "SMP unexpected LTK request";
            break;

        case WARNING_GATT_BEARER_NOT_ALLOCATED:
            warning_type_str = "GATT bearer not allocated";

        default:
            warning_type_str = "Unknown";
            break;
    }

    return warning_type_str;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline const char * _ble_adapter_generic_peer_addr_type_display_str(const sid_pal_ble_ext_peer_addr_type_t addr_type, const uint8_t addr_msb)
{
    const char * addr_type_str;

    switch (addr_type)
    {
        case SPBP_PEER_ADDR_TYPE_PUBLIC:
            addr_type_str = "Public";
            break;

        case SPBP_PEER_ADDR_TYPE_RANDOM:
            if ((addr_msb & 0xC0u) == 0u)
            {
                /* As per BLE specification , the two upper bits shall be set to 00b for non-resolvable private address */
                addr_type_str = "Non-resolvable Private";
            }
            else if ((addr_msb & 0xC0u) == 0xC0u)
            {
                /* As per BLE specification , the two upper bits shall be set to 11b for static random address */
                addr_type_str = "Static Random";
            }
            else if ((addr_msb & 0xC0u) == 0x40u) /* This brach covers connections to the RPA peers before pairing is done, so RPA cannot be resolved at this stage and stack reports it as a generic random address */
            {
                /* For RPA address the upper two bits are 01b */
                addr_type_str = "RPA";
            }
            else
            {
                /* This is out of BLE spec and normally should not happen */
                addr_type_str = "Malformed Random";
            }
            break;

        case SPBP_PEER_ADDR_TYPE_RPA_PUB_ID:
        case SPBP_PEER_ADDR_TYPE_RPA_RAND_ID:
            /* This branch covers connections with known RPA peers after pairing, so RPA is resolved and recognized properly by the stack */
            addr_type_str = "RPA";
            break;

        default:
            /* Should never happen */
            addr_type_str = "Unknown";
            break;
    }

    return addr_type_str;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline const char * _ble_adapter_generic_phy_type_display_str(const sid_pal_ble_ext_le_phy_t phy_type)
{
    const char * phy_type_str;

    switch (phy_type)
    {
        case SPBP_LE_PHY_1M:
            phy_type_str = "1M";
            break;

        case SPBP_LE_PHY_2M:
            phy_type_str = "2M";
            break;

        case SPBP_LE_PHY_CODED:
            phy_type_str = "LE-Coded";
            break;

        case SPBP_LE_PHY_UNKNOWN:
        default:
            phy_type_str = "Unknown";
            break;
    }

    return phy_type_str;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _ble_adapter_generic_resume_flow_process(void)
{
    /* Receive any events from the LL */
    change_state_options_t notify_options = {
        .bitfield = {
            .allow_generic_event = 1u,
            .allow_acl_data      = 1u,
            .allow_iso_data      = 1u,
            .allow_reports       = 1u,
            .allow_sync_event    = 1u,
            .allow_eoa_event     = 1u,
        },
    };

    ll_intf_chng_evnt_hndlr_state(notify_options);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void BLE_NvmCallback(SNVMA_Callback_Status_t CbkStatus)
{
    if (CbkStatus != SNVMA_OPERATION_COMPLETE)
    {
        SNVMA_Cmd_Status_t snvma_status;

        SID_PAL_LOG_ERROR("NVM Callback 0x%02X", CbkStatus);

        /* Retry the write operation */
        snvma_status = SNVMA_Write(APP_BLE_NvmBuffer, BLE_NvmCallback);
        if (snvma_status != SNVMA_ERROR_OK)
        {
            SID_PAL_LOG_ERROR("BLE NVMA Store failed with error 0x%02X", snvma_status);
        }
    }
    else
    {
        SID_BLE_GENERIC_LOG_DEBUG("BLE NVM Store Success");
    }
}

/*----------------------------------------------------------------------------*/

#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0)
SID_STM32_SPEED_OPTIMIZED static void _ble_adapter_generic_adv_duration_timer_cb(void *arg)
{
    uint32_t adv_set_id = (uint32_t)arg;

    /**
     * The code shall be executed in the background as aci command may be sent
     * The background is the only place where the application can make sure a new aci command
     * is not sent if there is a pending one
     */
    (void)osMessageQueuePut(sid_ble_drv_ctx.advertising_cmd_queue, &adv_set_id, 0u, 0u);

    return;
}
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _gap_cmd_resp_release(const sid_pal_ble_prv_connection_ctx_t * const conn_ctx)
{
    SID_PAL_ASSERT(conn_ctx != NULL);
    SID_PAL_ASSERT(conn_ctx->gap_cmd_lock != NULL);

    (void)osSemaphoreRelease(conn_ctx->gap_cmd_lock);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void BleStack_Process_BG_Entry(void* thread_input)
{
    (void)(thread_input);

    while (1)
    {
        osSemaphoreAcquire(BleHostSemaphore, osWaitForever);
        BleStack_Process_BG();
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void BleStack_Process_BG(void)
{
    osMutexAcquire(LinkLayerMutex, osWaitForever);
    const uint32_t Running = BleStack_Process();
    osMutexRelease(LinkLayerMutex);
    if (BLE_SLEEPMODE_RUNNING == Running)
    {
        BleStackCB_Process();
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void Ble_UserEvtRx_Entry(void* thread_input)
{
    (void)(thread_input);

    while(1)
    {
        osSemaphoreAcquire(hci_async_evt_semaphore, osWaitForever);
        osMutexAcquire(LinkLayerMutex, osWaitForever);
        Ble_UserEvtRx();
        osMutexRelease(LinkLayerMutex);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void Ble_UserEvtRx(void)
{
    SVCCTL_UserEvtFlowStatus_t svctl_return_status;
    BleEvtPacket_t *phcievt;

    LST_remove_head(&BleAsynchEventQueue, (tListNode **)&phcievt);

    svctl_return_status = SVCCTL_UserEvtRx((void *)&(phcievt->evtserial));

    if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
    {
        AMM_Free((uint32_t *)phcievt);
    }
    else
    {
        LST_insert_head(&BleAsynchEventQueue, (tListNode *)phcievt);
    }

    if ((LST_is_empty(&BleAsynchEventQueue) == FALSE) && (svctl_return_status != SVCCTL_UserEvtFlowDisable))
    {
        osSemaphoreRelease(hci_async_evt_semaphore);
    }

    /* Trigger BLE Host stack to process */
    osSemaphoreRelease(BleHostSemaphore);
}

/*----------------------------------------------------------------------------*/

#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0)
SID_STM32_SPEED_OPTIMIZED static void _ble_adapter_generic_adv_cmd_task_entry(void* thread_input)
{
  (void)(thread_input);

  while(1)
  {
    osStatus_t os_status;
    uint32_t   adv_set_id;

    /* Wait for a notification from a timer */
    os_status = osMessageQueueGet(sid_ble_drv_ctx.advertising_cmd_queue, &adv_set_id, NULL, osWaitForever);

    /* Check if an advertisement set id was received successfully */
    if (os_status != osOK)
    {
        /* Skip processing and wait for the next notification */
        continue;
    }

    /* Acquire LL lock to safely process advertisement timeout */
    os_status = osMutexAcquire(LinkLayerMutex, osWaitForever);
    SID_PAL_ASSERT(osOK == os_status);

    /* Stop the advertisement since the planned duration has elapsed */
    (void)sid_stm32wba_ble_adapter_prv_generic_stop_advertisement(adv_set_id);
    /* Logs are provided by sid_stm32wba_ble_adapter_prv_generic_stop_advertisement(), in terms of the error reaction there's not so much we can do from here */

    /* Process advertisement timeout */
    if (SID_STM32_BLE_SIDEWALK_VIRT_DEV_ID == adv_set_id)
    {
        (void)sid_stm32wba_ble_adapter_sidewalk_on_adv_timeout();
        /* Logs are provided by sid_stm32wba_ble_adapter_sidewalk_on_adv_timeout(), in terms of the error reaction there's not so much we can do from here */
    }
#  if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
    else
    {
        (void)sid_stm32wba_ble_adapter_user_on_adv_timeout(adv_set_id);
        /* Logs are provided by sid_stm32wba_ble_adapter_user_on_adv_timeout(), in terms of the error reaction there's not so much we can do from here */
    }
#  endif /* SID_STM32_BLE_COEXISTENCE_MODE */

    (void)osMutexRelease(LinkLayerMutex);
  }
}
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _ble_adapter_generic_svc_event_handling_init(void)
{
    sid_pal_enter_critical_region();

    /* Clear the local register of handlers */
    for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(sid_ble_drv_ctx.svc_event_handlers); i++)
    {
        sid_ble_drv_ctx.svc_event_handlers[i].handler = NULL;
        sid_ble_drv_ctx.svc_event_handlers[i].ref_count = 0u;
    }

    /* Register local event router */
    SVCCTL_RegisterSvcHandler(_ble_adapter_generic_svc_event_router); /* Handles SVCCTL_GATT_EVT_TYPE events only */
    SVCCTL_RegisterHandler(_ble_adapter_generic_svc_event_router);    /* Handles all the other events */

    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static SVCCTL_EvtAckStatus_t _ble_adapter_generic_svc_event_router(void * p_evt)
{
    SVCCTL_EvtAckStatus_t evt_ack_status = SVCCTL_EvtNotAck;

    for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(sid_ble_drv_ctx.svc_event_handlers); i++)
    {
        sid_pal_ble_prv_evt_hndlr_desc_t handler_descriptor;

        /* Store a local copy of the function pointer to avoid race conditions */
        sid_pal_enter_critical_region();
        handler_descriptor = sid_ble_drv_ctx.svc_event_handlers[i];
        sid_pal_exit_critical_region();

        if (handler_descriptor.handler != NULL)
        {
            evt_ack_status = handler_descriptor.handler(p_evt);
            if (evt_ack_status != SVCCTL_EvtNotAck)
            {
                /* One of the handlers processed the event, stop further processing */
                break;
            }
        }
    }

    return evt_ack_status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline SVCCTL_UserEvtFlowStatus_t _ble_adapter_generic_process_hci_le_meta_evt(const evt_le_meta_event * const evt)
{
    SVCCTL_UserEvtFlowStatus_t evt_flow_status = SVCCTL_UserEvtFlowEnable;

    do
    {
        switch (evt->subevent)
        {
            case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
                {
                    const hci_le_connection_update_complete_event_rp0 * const p_conn_update_complete = (hci_le_connection_update_complete_event_rp0 *)evt->data;

                    SID_BLE_GENERIC_LOG_DEBUG(">>== HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT - status: 0x%02X, conn: 0x%04X", p_conn_update_complete->Status, p_conn_update_complete->Connection_Handle);

                    if (SID_BLE_HCI_STATUS_SUCCESS == p_conn_update_complete->Status)
                    {
                        /* Locate the associated generic connection context for the connection */
                        sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_conn_ctx_for_handle(p_conn_update_complete->Connection_Handle);

                        if (NULL == conn_ctx)
                        {
                            SID_PAL_LOG_ERROR("No context found for BLE connection 0x%04X", p_conn_update_complete->Connection_Handle);
                            (void)aci_gap_terminate(p_conn_update_complete->Connection_Handle, SID_BLE_HCI_STATUS_UNKNOWN_CONN_ID);
                            break;
                        }

                        /* Update stored data in the connection context */
                        sid_pal_enter_critical_region();
                        conn_ctx->public_ctx.conn_interval       = p_conn_update_complete->Conn_Interval;
                        conn_ctx->public_ctx.conn_latency        = p_conn_update_complete->Conn_Latency;
                        conn_ctx->public_ctx.supervision_timeout = p_conn_update_complete->Supervision_Timeout;
                        sid_pal_exit_critical_region();

                        SID_PAL_LOG_INFO("BLE conn params updated for handle 0x%04X, interval: %ums, latency: %u, supervision timeout: %ums",
                                         p_conn_update_complete->Connection_Handle,
                                         (uint32_t)SID_STM32_BLE_CONN_INTERVAL_UNITS_TO_MS(p_conn_update_complete->Conn_Interval),
                                         p_conn_update_complete->Conn_Latency, /* Latency is measured in link events */
                                         SID_STM32_BLE_CONN_SUPERVISION_UNITS_TO_MS(p_conn_update_complete->Supervision_Timeout)
                                        );

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                        if ((NULL == sid_ble_sidewalk_ctx.conn_ctx) || (p_conn_update_complete->Connection_Handle != sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id))
                        {
                            sid_error_t sid_err;

                            sid_err = sid_stm32wba_ble_adapter_user_on_conn_param_changed(p_conn_update_complete->Connection_Handle);
                            if (sid_err != SID_ERROR_NONE)
                            {
                                if (SID_ERROR_NOT_FOUND == sid_err)
                                {
                                    SID_PAL_LOG_ERROR("Cannot associate BLE connection 0x%04X with Sidewalk or User Mode", p_conn_update_complete->Connection_Handle);
                                    (void)aci_gap_terminate(p_conn_update_complete->Connection_Handle, SID_BLE_HCI_STATUS_UNKNOWN_CONN_ID);
                                }
                                else
                                {
                                    SID_PAL_LOG_ERROR("BLE connection parameter update for connection 0x%04X failed. Error %d", p_conn_update_complete->Connection_Handle, (int32_t)sid_err);
                                    (void)aci_gap_terminate(p_conn_update_complete->Connection_Handle, SID_BLE_HCI_STATUS_UNACCEPTABLE_CONN_PARAMS);
                                }
                            }
                        }
                        else
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
                        {
                            sid_error_t sid_err;

                            sid_err = sid_stm32wba_ble_adapter_sidewalk_on_ble_conn_params_changed();
                            if (sid_err != SID_ERROR_NONE)
                            {
                                SID_PAL_LOG_ERROR("BLE connection parameter update for Sidewalk failed. Error %d", (int32_t)sid_err);
                                (void)aci_gap_terminate(p_conn_update_complete->Connection_Handle, SID_BLE_HCI_STATUS_UNACCEPTABLE_CONN_PARAMS);
                            }
                        }
                    }
                    else
                    {
                        SID_PAL_LOG_ERROR("BLE connection parameter update failed. Status: 0x%02X, conn: 0x%04X", p_conn_update_complete->Status, p_conn_update_complete->Connection_Handle);
                    }
                }
                break;

            case HCI_LE_REMOTE_CONNECTION_PARAMETER_REQUEST_SUBEVT_CODE:
                {
                    const hci_le_remote_connection_parameter_request_event_rp0 * const p_remote_connection_parameter_request = (hci_le_remote_connection_parameter_request_event_rp0 *)evt->data;
                    sid_ble_ext_proposed_conn_params_t proposed_params;
                    sid_ble_ext_accepted_conn_params_t accepted_params;
                    tBleStatus reason, ble_status;

                    SID_BLE_GENERIC_LOG_DEBUG(">>== HCI_LE_REMOTE_CONNECTION_PARAMETER_REQUEST_EVENT- conn: 0x%04X", p_remote_connection_parameter_request->Connection_Handle);

                    /* Locate the associated generic connection context for the connection */
                     sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_conn_ctx_for_handle(p_remote_connection_parameter_request->Connection_Handle);

                    if (NULL == conn_ctx)
                    {
                        SID_PAL_LOG_ERROR("No context found for BLE connection 0x%04X", p_remote_connection_parameter_request->Connection_Handle);
                        (void)aci_gap_terminate(p_remote_connection_parameter_request->Connection_Handle, SID_BLE_HCI_STATUS_UNKNOWN_CONN_ID);
                        break;
                    }

                    proposed_params.interval_min = p_remote_connection_parameter_request->Interval_Min;
                    proposed_params.interval_max = p_remote_connection_parameter_request->Interval_Max;
                    proposed_params.latency_max  = p_remote_connection_parameter_request->Max_Latency;
                    proposed_params.timeout      = p_remote_connection_parameter_request->Timeout;

                    SID_PAL_LOG_INFO("BLE conn params update request for handle 0x%04X, interval min: %ums, interval max: %ums, latency max: %u, timeout: %ums",
                                     p_remote_connection_parameter_request->Connection_Handle,
                                     (uint32_t)SID_STM32_BLE_CONN_INTERVAL_UNITS_TO_MS(proposed_params.interval_min),
                                     (uint32_t)SID_STM32_BLE_CONN_INTERVAL_UNITS_TO_MS(proposed_params.interval_max),
                                     proposed_params.latency_max, /* Latency is measured in link events */
                                     SID_STM32_BLE_CONN_SUPERVISION_UNITS_TO_MS(proposed_params.timeout)
                                    );

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                    if ((NULL == sid_ble_sidewalk_ctx.conn_ctx) || (p_remote_connection_parameter_request->Connection_Handle != sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id))
                    {
                        sid_error_t sid_err;

                        sid_err = sid_stm32wba_ble_adapter_user_on_conn_params_update_req(p_remote_connection_parameter_request->Connection_Handle, &proposed_params, &accepted_params, &reason);
                        if (sid_err != SID_ERROR_NONE)
                        {
                            if (SID_ERROR_NOT_FOUND == sid_err)
                            {
                                SID_PAL_LOG_ERROR("Cannot associate BLE connection 0x%04X with Sidewalk or User Mode", p_remote_connection_parameter_request->Connection_Handle);
                                (void)aci_gap_terminate(p_remote_connection_parameter_request->Connection_Handle, SID_BLE_HCI_STATUS_UNKNOWN_CONN_ID);
                                break;
                            }
                            else
                            {
                                SID_PAL_LOG_ERROR("BLE connection parameter update request for connection 0x%04X failed. Error %d", p_remote_connection_parameter_request->Connection_Handle, (int32_t)sid_err);
                                (void)aci_gap_terminate(p_remote_connection_parameter_request->Connection_Handle, SID_BLE_HCI_STATUS_UNACCEPTABLE_CONN_PARAMS);
                                break;
                            }
                        }
                    }
                    else
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
                    {
                        sid_error_t sid_err;

                        sid_err = sid_stm32wba_ble_adapter_sidewalk_on_ble_conn_params_update_req(&proposed_params, &accepted_params, &reason);
                        if (sid_err != SID_ERROR_NONE)
                        {
                            SID_PAL_LOG_ERROR("BLE connection parameter update request for Sidewalk failed. Error %d", (int32_t)sid_err);
                            (void)aci_gap_terminate(p_remote_connection_parameter_request->Connection_Handle, SID_BLE_HCI_STATUS_UNACCEPTABLE_CONN_PARAMS);
                            break;
                        }
                    }

                    if (BLE_STATUS_SUCCESS == reason)
                    {
                        ble_status = hci_le_remote_connection_parameter_request_reply(p_remote_connection_parameter_request->Connection_Handle,
                                                                                      accepted_params.interval_min,
                                                                                      accepted_params.interval_max,
                                                                                      accepted_params.latency_max,
                                                                                      accepted_params.timeout,
                                                                                      accepted_params.ce_length_min,
                                                                                      accepted_params.ce_length_max);
                    }
                    else
                    {
                        ble_status = hci_le_remote_connection_parameter_request_negative_reply(p_remote_connection_parameter_request->Connection_Handle, reason);
                    }

                    if (ble_status != BLE_STATUS_SUCCESS)
                    {
                        SID_PAL_LOG_ERROR("Failed to respond to BLE connection parameter update request for handle 0x%04X, status 0x%02X", p_remote_connection_parameter_request->Connection_Handle, ble_status);
                        (void)aci_gap_terminate(p_remote_connection_parameter_request->Connection_Handle, SID_BLE_HCI_STATUS_UNACCEPTABLE_CONN_PARAMS);
                    }
                }
                break;

            case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:
                {
                    const hci_le_phy_update_complete_event_rp0 * const p_phy_update_complete = (hci_le_phy_update_complete_event_rp0 *)evt->data;

                    SID_BLE_GENERIC_LOG_DEBUG(">>== HCI_LE_PHY_UPDATE_COMPLETE_EVENT - status: 0x%02X, conn: 0x%04X, tx_phy: 0x%1X, rx_phy: 0x%1X",
                                      p_phy_update_complete->Status,
                                      p_phy_update_complete->Connection_Handle,
                                      p_phy_update_complete->TX_PHY,
                                      p_phy_update_complete->RX_PHY
                    );

                    sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_conn_ctx_for_handle(p_phy_update_complete->Connection_Handle);
                    if (conn_ctx != NULL)
                    {
                        static_assert(SPBP_LE_PHY_1M    == HCI_RX_PHY_LE_1M);
                        static_assert(SPBP_LE_PHY_2M    == HCI_RX_PHY_LE_2M);
                        static_assert(SPBP_LE_PHY_CODED == HCI_RX_PHY_LE_CODED);
                        static_assert(SPBP_LE_PHY_1M    == HCI_TX_PHY_LE_1M);
                        static_assert(SPBP_LE_PHY_2M    == HCI_TX_PHY_LE_2M);
                        static_assert(SPBP_LE_PHY_CODED == HCI_TX_PHY_LE_CODED);

                        if (SID_BLE_HCI_STATUS_SUCCESS == p_phy_update_complete->Status)
                        {
                            conn_ctx->public_ctx.tx_phy = (sid_pal_ble_ext_le_phy_t)p_phy_update_complete->TX_PHY;
                            conn_ctx->public_ctx.rx_phy = (sid_pal_ble_ext_le_phy_t)p_phy_update_complete->RX_PHY;

                            SID_PAL_LOG_INFO("PHY updated for BLE connection 0x%04X, Tx PHY: %s, Rx PHY: %s",
                                             conn_ctx->public_ctx.conn_id,
                                             _ble_adapter_generic_phy_type_display_str(conn_ctx->public_ctx.tx_phy),
                                             _ble_adapter_generic_phy_type_display_str(conn_ctx->public_ctx.rx_phy));
                        }
                        _gap_cmd_resp_release(conn_ctx);
                    }
                }
                break;

#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
            case HCI_LE_ADVERTISING_SET_TERMINATED_SUBEVT_CODE:
                {
                    const hci_le_advertising_set_terminated_event_rp0 * const p_advertising_set_terminated = (hci_le_advertising_set_terminated_event_rp0 *)evt->data;

                    SID_BLE_GENERIC_LOG_DEBUG(">>== HCI_LE_ADVERTISING_SET_TERMINATED_EVENT - set: 0x%02X, status: 0x%02X, conn handle: 0x%04X",
                                      p_advertising_set_terminated->Advertising_Handle,
                                      p_advertising_set_terminated->Status,
                                      (SID_BLE_HCI_STATUS_SUCCESS == p_advertising_set_terminated->Status) ? p_advertising_set_terminated->Connection_Handle : SID_STM32_BLE_HANDLE_INVALID_VALUE);

                    /* If the advertisement resulted in a connection */
                    if (SID_BLE_HCI_STATUS_SUCCESS == p_advertising_set_terminated->Status)
                    {
                        sid_error_t sid_err;

                        /* Locate the associated generic connection context for the connection */
                        sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_conn_ctx_for_handle(p_advertising_set_terminated->Connection_Handle);

                        if (NULL == conn_ctx)
                        {
                            SID_PAL_LOG_ERROR("No context found for BLE connection 0x%04X", p_advertising_set_terminated->Connection_Handle);
                            (void)aci_gap_terminate(p_advertising_set_terminated->Connection_Handle, SID_BLE_HCI_STATUS_UNKNOWN_CONN_ID);
                            break;
                        }

                        /* Store the related advertising handle */
                        conn_ctx->adv_set_id = p_advertising_set_terminated->Advertising_Handle;

                        /* Process Sidewalk-related event */
                        if (p_advertising_set_terminated->Advertising_Handle == sid_ble_sidewalk_ctx.adv_set.Advertising_Handle)
                        {
                            /* This connection comes from the advertisement set associated with Sidewalk advertisement - start Sidewalk link */
                            sid_err = sid_stm32wba_ble_adapter_sidewalk_on_ble_connected(conn_ctx);
                        }
                        else
                        {
                            /* Established a user mode connection with a Central peer */
                            sid_err = sid_stm32wba_ble_adapter_user_on_ble_connected_to_central(p_advertising_set_terminated->Advertising_Handle, conn_ctx);
                        }

                        if (sid_err != SID_ERROR_NONE)
                        {
                            /* Something went wrong during connection completion (e.g. user-mode part of the driver refused to accept the connection) */
                            SID_PAL_LOG_ERROR("Can't accept BLE connection with @:%02x:%02x:%02x:%02x:%02x:%02x",
                                              conn_ctx->public_ctx.peer_identity[5],
                                              conn_ctx->public_ctx.peer_identity[4],
                                              conn_ctx->public_ctx.peer_identity[3],
                                              conn_ctx->public_ctx.peer_identity[2],
                                              conn_ctx->public_ctx.peer_identity[1],
                                              conn_ctx->public_ctx.peer_identity[0]);
                            (void)aci_gap_terminate(p_advertising_set_terminated->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
                            break;
                        }
                    }
                    else if (SID_BLE_HCI_STATUS_ADV_TIMEOUT == p_advertising_set_terminated->Status)
                    {
                        /* Advertisement set was terminated due to timeout - restart if needed */
                        if (p_advertising_set_terminated->Advertising_Handle == sid_ble_sidewalk_ctx.adv_set.Advertising_Handle)
                        {
                            (void)sid_stm32wba_ble_adapter_sidewalk_on_adv_timeout();
                        }
                        else
                        {
                            (void)sid_stm32wba_ble_adapter_user_on_adv_timeout(p_advertising_set_terminated->Advertising_Handle);
                        }
                    }
                    else
                    {
                        SID_PAL_LOG_ERROR("BLE advertisement terminated with error. Set: 0x%02X, status: %02X", p_advertising_set_terminated->Advertising_Handle, p_advertising_set_terminated->Status);
                    }
                }
                break;
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

            case HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE:
                {
                    const hci_le_enhanced_connection_complete_event_rp0 * const p_enhanced_conn_complete = (hci_le_enhanced_connection_complete_event_rp0 *)evt->data;

                    SID_BLE_GENERIC_LOG_DEBUG(">>== HCI_LE_ENHANCED_CONNECTION_COMPLETE_EVENT - Status: 0x%02X, Connection handle: 0x%04X", p_enhanced_conn_complete->Status, p_enhanced_conn_complete->Connection_Handle);

                    /* Run in a critical section because we are about to setup connection context */
                    sid_pal_enter_critical_region();

                    if (p_enhanced_conn_complete->Status != SID_BLE_HCI_STATUS_SUCCESS)
                    {
                        /* Connection attempt failed - ignore it and restart advertisement */
#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0)
                        /* Restart the advertisement automatically. Duration timer is still running if it was armed */
                        tBleStatus ble_status = hci_le_set_advertising_enable(SID_BLE_HCI_SET_ADV_ENABLE);
                        if (ble_status != BLE_STATUS_SUCCESS)
                        {
                            SID_PAL_LOG_ERROR("==>> hci_le_set_advertising_enable - fail, result: 0x%02X", ble_status);
                        }
                        else
                        {
                            SID_PAL_LOG_DEBUG("==>> hci_le_set_advertising_enable - Success");
                        }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */
                        sid_pal_exit_critical_region();
                        break;
                    }

                    /* Create generic connection context for further connection management */
                    sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_free_conn_ctx(p_enhanced_conn_complete->Connection_Handle);

                    /* Check if we are not out of resources */
                    if (NULL == conn_ctx)
                    {
                        SID_PAL_LOG_WARNING("Cannot accept BLE connection - no slots for ctx storage");
                        (void)aci_gap_terminate(p_enhanced_conn_complete->Connection_Handle, SID_BLE_HCI_STATUS_CONN_LIMIT_EXCEEDED);
                        sid_pal_exit_critical_region();
                        break;
                    }

#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0)
                    /* Stop the related advertisement timer */
                    (void)UTIL_TIMER_Stop(&sid_ble_drv_ctx.adv_timer);
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

                    /* Populate connection context */
                    /* conn_ctx->public_ctx.conn_id is set already by _ble_adapter_generic_get_free_conn_ctx() */
                    /* conn_ctx->adv_set_id will be set in HCI_LE_ADVERTISING_SET_TERMINATED (if relevant) */
                    conn_ctx->public_ctx.is_secure           = FALSE;
                    conn_ctx->public_ctx.mtu_size            = SID_BLE_HCI_DEFAULT_ATT_MTU;         /* Initial MTU size is mandated by the Core Specification - 23 bytes */
                    conn_ctx->public_ctx.max_mtu_size        = sid_ble_drv_ctx.global_att_mtu_size; /* Maximum acceptable MTU by the controller */
                    conn_ctx->role                           = p_enhanced_conn_complete->Role;
                    conn_ctx->public_ctx.peer_addr_type      = p_enhanced_conn_complete->Peer_Address_Type;
                    conn_ctx->public_ctx.conn_interval       = p_enhanced_conn_complete->Conn_Interval;
                    conn_ctx->public_ctx.conn_latency        = p_enhanced_conn_complete->Conn_Latency;
                    conn_ctx->public_ctx.supervision_timeout = p_enhanced_conn_complete->Supervision_Timeout;
                    SID_STM32_UTIL_fast_memcpy(conn_ctx->public_ctx.peer_identity, p_enhanced_conn_complete->Peer_Address,                     sizeof(conn_ctx->public_ctx.peer_identity));
                    SID_STM32_UTIL_fast_memcpy(conn_ctx->public_ctx.local_rpa,     p_enhanced_conn_complete->Local_Resolvable_Private_Address, sizeof(conn_ctx->public_ctx.local_rpa));
                    SID_STM32_UTIL_fast_memcpy(conn_ctx->public_ctx.peer_rpa,      p_enhanced_conn_complete->Peer_Resolvable_Private_Address,  sizeof(conn_ctx->public_ctx.peer_rpa));

                    /* Get information about the initially used PHYs for Tx and Rx for this connection */
                    uint8_t tx_phy = (uint8_t)SPBP_LE_PHY_UNKNOWN;
                    uint8_t rx_phy = (uint8_t)SPBP_LE_PHY_UNKNOWN;
                    (void)hci_le_read_phy(p_enhanced_conn_complete->Connection_Handle, &tx_phy, &rx_phy);
                    conn_ctx->public_ctx.tx_phy = (sid_pal_ble_ext_le_phy_t)tx_phy;
                    conn_ctx->public_ctx.rx_phy = (sid_pal_ble_ext_le_phy_t)rx_phy;

                    sid_pal_exit_critical_region();

                    /* Log connection info */
                    if (p_enhanced_conn_complete->Peer_Address_Type < SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RPA_PUBLIC_IDENTITY)
                    {
                        /* Non-resolvable address (e.g. public, static random, or NRPA) */
                        SID_PAL_LOG_INFO("Established BLE connection (0x%04X) with %02X:%02X:%02X:%02X:%02X:%02X (%s addr), Tx PHY: %s, Rx PHY: %s",
                                         p_enhanced_conn_complete->Connection_Handle,
                                         p_enhanced_conn_complete->Peer_Address[5],
                                         p_enhanced_conn_complete->Peer_Address[4],
                                         p_enhanced_conn_complete->Peer_Address[3],
                                         p_enhanced_conn_complete->Peer_Address[2],
                                         p_enhanced_conn_complete->Peer_Address[1],
                                         p_enhanced_conn_complete->Peer_Address[0],
                                         _ble_adapter_generic_peer_addr_type_display_str(p_enhanced_conn_complete->Peer_Address_Type, p_enhanced_conn_complete->Peer_Address[5]),
                                         _ble_adapter_generic_phy_type_display_str(conn_ctx->public_ctx.tx_phy),
                                         _ble_adapter_generic_phy_type_display_str(conn_ctx->public_ctx.rx_phy));
                    }
                    else
                    {
                        /* Resolvable address - print RPA and related resolved identity address */
                        SID_PAL_LOG_INFO("Established BLE connection (0x%04X) with %02X:%02X:%02X:%02X:%02X:%02X (RPA, resolved to %02X:%02X:%02X:%02X:%02X:%02X), PHY: %s, Rx PHY: %s",
                                         p_enhanced_conn_complete->Connection_Handle,
                                         p_enhanced_conn_complete->Peer_Resolvable_Private_Address[5],
                                         p_enhanced_conn_complete->Peer_Resolvable_Private_Address[4],
                                         p_enhanced_conn_complete->Peer_Resolvable_Private_Address[3],
                                         p_enhanced_conn_complete->Peer_Resolvable_Private_Address[2],
                                         p_enhanced_conn_complete->Peer_Resolvable_Private_Address[1],
                                         p_enhanced_conn_complete->Peer_Resolvable_Private_Address[0],
                                         p_enhanced_conn_complete->Peer_Address[5],
                                         p_enhanced_conn_complete->Peer_Address[4],
                                         p_enhanced_conn_complete->Peer_Address[3],
                                         p_enhanced_conn_complete->Peer_Address[2],
                                         p_enhanced_conn_complete->Peer_Address[1],
                                         p_enhanced_conn_complete->Peer_Address[0],
                                         _ble_adapter_generic_phy_type_display_str(conn_ctx->public_ctx.tx_phy),
                                         _ble_adapter_generic_phy_type_display_str(conn_ctx->public_ctx.rx_phy));
                    }
                    SID_PAL_LOG_INFO("BLE conn params for handle 0x%04X: interval: %ums, latency: %u, supervision timeout: %ums",
                                     p_enhanced_conn_complete->Connection_Handle,
                                     (uint32_t)SID_STM32_BLE_CONN_INTERVAL_UNITS_TO_MS(p_enhanced_conn_complete->Conn_Interval),
                                     p_enhanced_conn_complete->Conn_Latency, /* Latency is measured in link events */
                                     (uint32_t)SID_STM32_BLE_CONN_SUPERVISION_UNITS_TO_MS(p_enhanced_conn_complete->Supervision_Timeout)
                                    );

#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
                    /* If extended advertisement is supported the processing will continue in HCI_LE_ADVERTISING_SET_TERMINATED event to decide if this connection belongs to Sidewalk or user domain */
#else
                    sid_error_t sid_err;
                    if ((SPBP_OPERATING_MODE_SIDEWALK == sid_ble_drv_ctx.operating_mode) && (SID_BLE_HCI_CONN_COMPLETE_ROLE_PERIPHERAL == p_enhanced_conn_complete->Role))
                    {
                        /* Driver is in the Sidewalk mode, treat this connection as Sidewalk connection */
                        sid_err = sid_stm32wba_ble_adapter_sidewalk_on_ble_connected(conn_ctx);
                    }
#  if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING)
                    else if (SPBP_OPERATING_MODE_USER == sid_ble_drv_ctx.operating_mode)
                    {
                        /* Driver is in the User BLE mode, treat this connection as user connection */
                        if (SID_BLE_HCI_CONN_COMPLETE_ROLE_PERIPHERAL == p_enhanced_conn_complete->Role)
                        {
                            /* Established a user mode connection with a Central peer */
                            sid_err = sid_stm32wba_ble_adapter_user_on_ble_connected_to_central(SID_STM32_BLE_VIRTUAL_DEVICE_ID_INVALID, conn_ctx);
                        }
                        else
                        {
                            /* Established a user mode connection with a Peripheral peer */
                            sid_err = sid_stm32wba_ble_adapter_user_on_ble_connected_to_peripheral(conn_ctx);
                        }
                    }
#  endif /* SID_STM32_BLE_COEXISTENCE_MODE */
                    else
                    {
                        /* Normally we shouldn't get here - there's a mismatch between the logical state of the driver and the actual events */
                        SID_PAL_LOG_ERROR("Inconsistent BLE driver state. Operating mode: %u, coexistence mode: %u", sid_ble_drv_ctx.operating_mode, SID_STM32_BLE_COEXISTENCE_MODE);
                        (void)aci_gap_terminate(p_enhanced_conn_complete->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
                        break;
                    }

                    if (sid_err != SID_ERROR_NONE)
                    {
                        /* Something went wrong during connection completion (e.g. user-mode part of the driver refused to accept the connection) */
                        SID_PAL_LOG_DEBUG("Can't accept BLE connection with @:%02x:%02x:%02x:%02x:%02x:%02x",
                                          conn_ctx->public_ctx.peer_identity[5],
                                          conn_ctx->public_ctx.peer_identity[4],
                                          conn_ctx->public_ctx.peer_identity[3],
                                          conn_ctx->public_ctx.peer_identity[2],
                                          conn_ctx->public_ctx.peer_identity[1],
                                          conn_ctx->public_ctx.peer_identity[0]);
                        (void)aci_gap_terminate(p_enhanced_conn_complete->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
                        break;
                    }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */
                }
                break;

            default:
                /* Do nothing, allow the event flow to continue */
                break;
        }
    } while (0);

    return evt_flow_status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline SVCCTL_UserEvtFlowStatus_t _ble_adapter_generic_process_aci_evt(const evt_blecore_aci * const evt)
{
    SVCCTL_UserEvtFlowStatus_t evt_flow_status = SVCCTL_UserEvtFlowEnable;
    tBleStatus                 ret;

    do
    {
        switch (evt->ecode)
        {
            case ACI_L2CAP_CONNECTION_UPDATE_RESP_VSEVT_CODE:
                {
                    const aci_l2cap_connection_update_resp_event_rp0 * const p_l2cap_conn_update_resp = (aci_l2cap_connection_update_resp_event_rp0 *)evt->data;
                    UNUSED(p_l2cap_conn_update_resp);
                    /* USER CODE BEGIN EVT_L2CAP_CONNECTION_UPDATE_RESP */

                    /* USER CODE END EVT_L2CAP_CONNECTION_UPDATE_RESP */
                }
                break;

            case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
                {
                    SID_BLE_GENERIC_LOG_DEBUG(">>== ACI_GAP_PROC_COMPLETE_EVENT");
                    const aci_gap_proc_complete_event_rp0 * const p_gap_proc_complete = (aci_gap_proc_complete_event_rp0 *)evt->data;
                    UNUSED(p_gap_proc_complete);
                    /* USER CODE BEGIN EVT_GAP_PROCEDURE_COMPLETE */

                    /* USER CODE END EVT_GAP_PROCEDURE_COMPLETE */
                }
                break;

            case ACI_GAP_KEYPRESS_NOTIFICATION_VSEVT_CODE:
                {
                    SID_BLE_GENERIC_LOG_DEBUG(">>== ACI_GAP_KEYPRESS_NOTIFICATION_EVENT");
                    const aci_gap_keypress_notification_event_rp0 * const p_keypress_notification = (aci_gap_keypress_notification_event_rp0 *)evt->data;

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                    /* Notify the respective GATT processors */
                    if ((NULL == sid_ble_sidewalk_ctx.conn_ctx) || (p_keypress_notification->Connection_Handle != sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id))
                    {
                        (void)sid_stm32wba_ble_adapter_user_on_pairing_keypress_notification(p_keypress_notification->Connection_Handle, p_keypress_notification->Notification_Type);
                    }
#else
                    (void)p_keypress_notification;
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
                }
                break;

            case ACI_GAP_PASS_KEY_REQ_VSEVT_CODE:
                {
                    SID_BLE_GENERIC_LOG_DEBUG(">>== ACI_GAP_PASS_KEY_REQ_EVENT");
                    const aci_gap_pass_key_req_event_rp0 * const p_pass_key_req = (aci_gap_pass_key_req_event_rp0 *)evt->data;
                    sid_error_t sid_err;

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                    /* Notify the respective GATT processors */
                    if ((NULL == sid_ble_sidewalk_ctx.conn_ctx) || (p_pass_key_req->Connection_Handle != sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id))
                    {
                        uint32_t passkey;

                        sid_err = sid_stm32wba_ble_adapter_user_on_pairing_pass_key_request(p_pass_key_req->Connection_Handle, &passkey);

                        if (SID_ERROR_NONE == sid_err)
                        {
                            ret = aci_gap_pass_key_resp(p_pass_key_req->Connection_Handle, passkey);
                            if (ret != BLE_STATUS_SUCCESS)
                            {
                                SID_BLE_GENERIC_LOG_ERROR("==>> aci_gap_pass_key_resp : Fail, reason: 0x%02X", ret);
                            }
                            else
                            {
                                SID_BLE_GENERIC_LOG_DEBUG("==>> aci_gap_pass_key_resp : Success");
                            }
                        }
                    }
                    else
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
                    {
                        sid_err = SID_ERROR_NO_PERMISSION;
                    }

                    if (sid_err != SID_ERROR_NONE)
                    {
                        (void)aci_gap_terminate(p_pass_key_req->Connection_Handle, SID_BLE_HCI_STATUS_PAIRING_NOT_ALLOWED);
                    }
                }
                break;

            case ACI_GAP_NUMERIC_COMPARISON_VALUE_VSEVT_CODE:
                {
                    SID_BLE_GENERIC_LOG_DEBUG(">>== ACI_GAP_NUMERIC_COMPARISON_VALUE_EVENT");
                    const aci_gap_numeric_comparison_value_event_rp0 * const p_numeric_comparison_value = (aci_gap_numeric_comparison_value_event_rp0 *)evt->data;
                    sid_error_t sid_err;

                    SID_BLE_GENERIC_LOG_DEBUG("     - numeric_value = %ld", p_numeric_comparison_value->Numeric_Value);
                    SID_BLE_GENERIC_LOG_DEBUG("     - Hex_value = %lx",p_numeric_comparison_value->Numeric_Value);

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                    /* Notify the respective GATT processors */
                    if ((NULL == sid_ble_sidewalk_ctx.conn_ctx) || (p_numeric_comparison_value->Connection_Handle != sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id))
                    {
                        uint32_t numeric_value_matches;

                        sid_err = sid_stm32wba_ble_adapter_user_on_pairing_numeric_comparison_value(p_numeric_comparison_value->Connection_Handle, p_numeric_comparison_value->Numeric_Value, &numeric_value_matches);
                        if (SID_ERROR_NONE == sid_err)
                        {
                            ret = aci_gap_numeric_comparison_value_confirm_yesno(p_numeric_comparison_value->Connection_Handle, numeric_value_matches != FALSE ? NUMERIC_COMPARISON_CONFIRM_YES : NUMERIC_COMPARISON_CONFIRM_NO);
                            if (ret != BLE_STATUS_SUCCESS)
                            {
                                SID_BLE_GENERIC_LOG_ERROR("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Fail, reason: 0x%02X", ret);
                            }
                            else
                            {
                                SID_BLE_GENERIC_LOG_DEBUG("==>> aci_gap_numeric_comparison_value_confirm_yesno-->YES : Success");
                            }
                        }
                    }
                    else
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
                    {
                        sid_err = SID_ERROR_NO_PERMISSION;
                    }

                    if (sid_err != SID_ERROR_NONE)
                    {
                        (void)aci_gap_terminate(p_numeric_comparison_value->Connection_Handle, SID_BLE_HCI_STATUS_PAIRING_NOT_ALLOWED);
                    }
                }
                break;

            case ACI_GAP_PAIRING_COMPLETE_VSEVT_CODE:
                {
                    SID_BLE_GENERIC_LOG_DEBUG(">>== ACI_GAP_PAIRING_COMPLETE_EVENT");
                    const aci_gap_pairing_complete_event_rp0 * const p_pairing_complete = (aci_gap_pairing_complete_event_rp0*)evt->data;
                    sid_error_t sid_err;

                    /* Update connection context if IRKs were exchanged */
                    if (BLE_STATUS_SUCCESS == p_pairing_complete->Status)
                    {
                        sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_conn_ctx_for_handle(p_pairing_complete->Connection_Handle);
                        if ((conn_ctx != NULL) && (SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RANDOM == conn_ctx->public_ctx.peer_addr_type) && ((conn_ctx->public_ctx.peer_identity[5] & SID_STM32_BLE_PRV_RANDOM_ADDR_SPECIFIER_MASK) == SID_STM32_BLE_PRV_RANDOM_ADDR_TYPE_RPA_MASK))
                        {
                            /* Move random address to RPA storage */
                            SID_STM32_UTIL_fast_memcpy(conn_ctx->public_ctx.peer_rpa, conn_ctx->public_ctx.peer_identity, sizeof(conn_ctx->public_ctx.peer_rpa));

                            /* Resolve address and update peer_identity storage */
                            ret = aci_gap_check_bonded_device(conn_ctx->public_ctx.peer_addr_type, conn_ctx->public_ctx.peer_rpa, &conn_ctx->public_ctx.peer_addr_type, conn_ctx->public_ctx.peer_identity);

                            /* Revert to SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RANDOM as we cannot resolve the address or bonding was not performed */
                            if (ret != BLE_STATUS_SUCCESS)
                            {
                                SID_STM32_UTIL_fast_memcpy(conn_ctx->public_ctx.peer_identity, conn_ctx->public_ctx.peer_rpa, sizeof(conn_ctx->public_ctx.peer_identity));
                                SID_STM32_UTIL_fast_memset(conn_ctx->public_ctx.peer_rpa, 0u, sizeof(conn_ctx->public_ctx.peer_rpa));
                            }
                            else
                            {
                                /* Resolvable address - print RPA and related resolved identity address */
                                SID_PAL_LOG_INFO("BLE connection (0x%04X) RPA configured: %02X:%02X:%02X:%02X:%02X:%02X (%02X:%02X:%02X:%02X:%02X:%02X), addr type 0x%02X",
                                                 p_pairing_complete->Connection_Handle,
                                                 conn_ctx->public_ctx.peer_rpa[5],
                                                 conn_ctx->public_ctx.peer_rpa[4],
                                                 conn_ctx->public_ctx.peer_rpa[3],
                                                 conn_ctx->public_ctx.peer_rpa[2],
                                                 conn_ctx->public_ctx.peer_rpa[1],
                                                 conn_ctx->public_ctx.peer_rpa[0],
                                                 conn_ctx->public_ctx.peer_identity[5],
                                                 conn_ctx->public_ctx.peer_identity[4],
                                                 conn_ctx->public_ctx.peer_identity[3],
                                                 conn_ctx->public_ctx.peer_identity[2],
                                                 conn_ctx->public_ctx.peer_identity[1],
                                                 conn_ctx->public_ctx.peer_identity[0],
                                                 conn_ctx->public_ctx.peer_addr_type);
                            }
                        }
                    }

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                    /* Notify the respective GATT processors */
                    if ((NULL == sid_ble_sidewalk_ctx.conn_ctx) || (p_pairing_complete->Connection_Handle != sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id))
                    {
                        sid_err = sid_stm32wba_ble_adapter_user_on_pairing_complete(p_pairing_complete->Connection_Handle, p_pairing_complete->Status, p_pairing_complete->Reason);
                    }
                    else
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
                    {
                        sid_err = SID_ERROR_NO_PERMISSION;
                    }

                    if (sid_err != SID_ERROR_NONE)
                    {
                        /* Try to clean up the device from the bonding list */
                        sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_conn_ctx_for_handle(p_pairing_complete->Connection_Handle);
                        if (conn_ctx != NULL)
                        {
                            uint8_t peer_identity_addr_type;
                            switch (conn_ctx->public_ctx.peer_addr_type)
                            {
                                case SID_BLE_HCI_ADV_PEER_ADDR_TYPE_PUBLIC:
                                case SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RPA_PUBLIC_IDENTITY:
                                    peer_identity_addr_type = SID_BLE_HCI_ADV_PEER_ADDR_TYPE_PUBLIC;
                                    break;

                                case SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RANDOM:
                                case SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RPA_RANDOM_IDENTITY:
                                default:
                                    peer_identity_addr_type = SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RANDOM;
                                    break;
                            }
                            (void)aci_gap_remove_bonded_device(peer_identity_addr_type, conn_ctx->public_ctx.peer_identity);
                        }

                        (void)aci_gap_terminate(p_pairing_complete->Connection_Handle, SID_BLE_HCI_STATUS_AUTH_FAILURE);
                    }
                }
                break;

            case ACI_GAP_BOND_LOST_VSEVT_CODE:
                {
                    SID_BLE_GENERIC_LOG_DEBUG(">>== ACI_GAP_BOND_LOST_EVENT");
                    const aci_gap_bond_lost_event_rp0 * const p_bond_lost = (aci_gap_bond_lost_event_rp0*)evt->data;

                    ret = aci_gap_allow_rebond(p_bond_lost->Connection_Handle);
                    if (ret != BLE_STATUS_SUCCESS)
                    {
                        SID_BLE_GENERIC_LOG_ERROR("==>> aci_gap_allow_rebond : Fail, reason: 0x%02X\n", ble_status);
                    }
                    else
                    {
                        SID_BLE_GENERIC_LOG_DEBUG("==>> aci_gap_allow_rebond : Success\n");
                    }
                }
                break;

            case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
                {
                    const aci_att_exchange_mtu_resp_event_rp0 * const p_exchange_mtu = (aci_att_exchange_mtu_resp_event_rp0 *) evt->data;

                    SID_BLE_GENERIC_LOG_DEBUG(">>== ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE - conn: 0x%04X, mtu: %u", p_exchange_mtu->Connection_Handle, p_exchange_mtu->Server_RX_MTU);

                    /* Locate the associated generic connection context for the connection */
                    sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_conn_ctx_for_handle(p_exchange_mtu->Connection_Handle);

                    if (NULL == conn_ctx)
                    {
                        SID_PAL_LOG_ERROR("No context found for BLE connection 0x%04X", p_exchange_mtu->Connection_Handle);
                        (void)aci_gap_terminate(p_exchange_mtu->Connection_Handle, SID_BLE_HCI_STATUS_UNKNOWN_CONN_ID);
                        break;
                    }

                    sid_pal_enter_critical_region();
                    /* Store the related advertising handle */
                    conn_ctx->public_ctx.mtu_size = p_exchange_mtu->Server_RX_MTU;

                    /* Notify the respective GATT processors */
                    if ((sid_ble_sidewalk_ctx.conn_ctx != NULL) && (p_exchange_mtu->Connection_Handle == sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id))
                    {
                        sid_stm32wba_ble_adapter_sidewalk_on_ble_mtu_changed();
                    }
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                    else
                    {
                        (void)sid_stm32wba_ble_adapter_user_on_ble_mtu_changed(p_exchange_mtu->Connection_Handle);
                    }
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
                    sid_pal_exit_critical_region();
                }
                break;

            /* Manage ACI_GATT_INDICATION_VSEVT_CODE occurring on Android 12 */
            case ACI_GATT_INDICATION_VSEVT_CODE:
                {
                    const aci_gatt_indication_event_rp0 * const p_indication = (aci_gatt_indication_event_rp0 *)evt->data;
                    ret = aci_gatt_confirm_indication(p_indication->Connection_Handle);
                    if (ret != BLE_STATUS_SUCCESS)
                    {
                        SID_PAL_LOG_ERROR("  Fail   : aci_gatt_confirm_indication command, result: 0x%x", ret);
                    }
                    else
                    {
                        SID_PAL_LOG_DEBUG("  Success: aci_gatt_confirm_indication command");
                    }
                }
                break;

            case ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE:
                {
                    const aci_gatt_tx_pool_available_event_rp0 * const p_tx_pool_available_event = (aci_gatt_tx_pool_available_event_rp0 *)evt->data;

                    /* TODO: this event is currently not used by the driver, but may be used to dynamically limit and restore BLE operations depending on the amount of the available RAM */
                    (void)p_tx_pool_available_event;
                }
                break;

            case ACI_GAP_AUTHORIZATION_REQ_VSEVT_CODE:
                {
                    SID_BLE_GENERIC_LOG_DEBUG(">>== ACI_GAP_AUTHORIZATION_REQ_EVENT");
                    aci_gap_authorization_req_event_rp0 * p_authorization_req = (aci_gap_authorization_req_event_rp0*)evt->data;
                    uint32_t authorize;

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                    if ((NULL == sid_ble_sidewalk_ctx.conn_ctx) || (p_authorization_req->Connection_Handle != sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id))
                    {
                        sid_error_t sid_err;

                        /* Let the user decide if this operation is authorized */
                        sid_err = sid_stm32wba_ble_adapter_user_on_authorization_req(p_authorization_req->Connection_Handle);
                        authorize = (SID_ERROR_NONE == sid_err) ? TRUE : FALSE;
                    }
                    else
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
                    {
                        /* For Sidewalk context authorization request is never expected - reject it */
                        authorize = FALSE;
                    }

                    (void)aci_gap_authorization_resp(p_authorization_req->Connection_Handle, authorize != FALSE ? 0x01u /* authorize */ : 0x02u /* reject */);
                }
                break;

            case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
                {
                    const aci_gatt_write_permit_req_event_rp0 * const p_write_permit_req = (aci_gatt_write_permit_req_event_rp0 *)evt->data;

                    /* Handle CCCD access to the Service Changed characteristic of the GATT Service */
                    if (p_write_permit_req->Attribute_Handle == (SID_STM32_BLE_GATT_SVC_SERVICE_CHANGED_CHAR_HANDLE + SID_STM32_BLE_CHARACTERISTIC_CCCD_ATTRIBUTE_OFFSET))
                    {
                        ret = aci_gatt_write_resp(p_write_permit_req->Connection_Handle, p_write_permit_req->Attribute_Handle,
                                                  0x00u /* value can be written */, SID_STM32_BLE_ATT_STATUS_SUCCESS,
                                                  p_write_permit_req->Data_Length, p_write_permit_req->Data);
                        if (ret != BLE_STATUS_SUCCESS)
                        {
                            SID_PAL_LOG_ERROR("Failed to set BLE write response for connection 0x%04X, attribute 0x%04X. ACI error 0x%02X", p_write_permit_req->Connection_Handle, p_write_permit_req->Attribute_Handle, ret);
                            (void)aci_gap_terminate(p_write_permit_req->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
                            break;
                        }
                    }
                }
                break;

            case ACI_WARNING_VSEVT_CODE:
                {
                    SID_BLE_GENERIC_LOG_DEBUG(">>== ACI_WARNING_EVENT");
                    const aci_warning_event_rp0 * const p_warning = (aci_warning_event_rp0 *)evt->data;

                    SID_PAL_LOG_WARNING("BLE ACI warning (0x%02X): %s", p_warning->Warning_Type, _ble_adapter_generic_aci_warning_display_str(p_warning->Warning_Type));
                    SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_DEBUG, p_warning->Data, p_warning->Data_Length);
                }
                break;

            default:
                /* Do nothing, allow the event flow to continue */
                break;
        }
    } while (0);

    return evt_flow_status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_error_t _ble_adapter_generic_conn_ctxs_init(void)
{
    sid_error_t err = SID_ERROR_NONE;
    static const osSemaphoreAttr_t gap_cmd_semaphore_attributes = {
        .name = "GAPCmdSemaphore"
    };

    for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(sid_ble_drv_ctx.conn_ctxs); i++)
    {
        SID_PAL_ASSERT(NULL == sid_ble_drv_ctx.conn_ctxs[i].gap_cmd_lock);

        sid_ble_drv_ctx.conn_ctxs[i].gap_cmd_lock = osSemaphoreNew(1u, 0u, &gap_cmd_semaphore_attributes);
        if (NULL == sid_ble_drv_ctx.conn_ctxs[i].gap_cmd_lock)
        {
            SID_PAL_LOG_ERROR("Failed to allocate GAP cmd lock - no memory");
            err = SID_ERROR_OOM;
            break;
        }
    }

    /* Invalidate all connection contexts */
    _ble_adapter_generic_invalidate_conn_ctx(NULL);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _ble_adapter_generic_conn_ctxs_deinit(void)
{
    for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(sid_ble_drv_ctx.conn_ctxs); i++)
    {
        if (sid_ble_drv_ctx.conn_ctxs[i].public_ctx.conn_id != SID_STM32_BLE_HANDLE_INVALID_VALUE)
        {
            SID_PAL_LOG_WARNING("Deleting BLE connection context for an active connection 0x%04X", sid_ble_drv_ctx.conn_ctxs[i].public_ctx.conn_id);
        }

        /* Delete the Semaphore used to wait for GAP commands completion */
        if (sid_ble_drv_ctx.conn_ctxs[i].gap_cmd_lock != NULL)
        {
            osStatus_t os_status;
            os_status = osSemaphoreDelete(sid_ble_drv_ctx.conn_ctxs[i].gap_cmd_lock);
            if (os_status != osOK)
            {
                SID_PAL_LOG_WARNING("Failed to delete GAP cmd lock for slot %u. Memory leak is possible", i);
            }
            sid_ble_drv_ctx.conn_ctxs[i].gap_cmd_lock = NULL;
        }

        /* Invalidate the connection context */
        _ble_adapter_generic_invalidate_conn_ctx(&sid_ble_drv_ctx.conn_ctxs[i]);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _ble_adapter_generic_invalidate_conn_ctx(sid_pal_ble_prv_connection_ctx_t * const ctx)
{
    sid_pal_ble_prv_connection_ctx_t * proc_ptr;
    sid_pal_ble_prv_connection_ctx_t * stop_ptr;

    /* Ensure ctx is either NULL or within the memory range of sid_ble_drv_ctx.conn_ctxs */
    SID_PAL_ASSERT((NULL == ctx) || ((ctx >= &sid_ble_drv_ctx.conn_ctxs[0]) && (ctx <= &sid_ble_drv_ctx.conn_ctxs[SID_STM32_UTIL_ARRAY_SIZE(sid_ble_drv_ctx.conn_ctxs) - 1])));

    if (NULL == ctx)
    {
        /* Invalidate all the contexts */
        proc_ptr = &sid_ble_drv_ctx.conn_ctxs[0];
        stop_ptr = &sid_ble_drv_ctx.conn_ctxs[SID_STM32_UTIL_ARRAY_SIZE(sid_ble_drv_ctx.conn_ctxs) - 1];
    }
    else
    {
        /* Invalidate only one specific context */
        proc_ptr = ctx;
        stop_ptr = ctx;
    }

    sid_pal_enter_critical_region();
    while (proc_ptr <= stop_ptr)
    {
        /* Store the pointer to the lock object */
        osSemaphoreId_t gap_cmd_lock = proc_ptr->gap_cmd_lock;

        /* Invalidate context */
        SID_STM32_UTIL_fast_memset(proc_ptr, 0u, sizeof(* proc_ptr));
        proc_ptr->public_ctx.conn_id = SID_STM32_BLE_HANDLE_INVALID_VALUE;
        proc_ptr->adv_set_id         = SID_STM32_BLE_HANDLE_INVALID_VALUE;

        /* Restore lock object pointer */
        proc_ptr->gap_cmd_lock = gap_cmd_lock;

        /* Move to the next connection context */
        proc_ptr++;
    }
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_pal_ble_prv_connection_ctx_t * _ble_adapter_generic_get_free_conn_ctx(const uint16_t conn_handle)
{
    sid_pal_ble_prv_connection_ctx_t * free_ctx = NULL;

    sid_pal_enter_critical_region();
    for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(sid_ble_drv_ctx.conn_ctxs); i++)
    {
        if (SID_STM32_BLE_HANDLE_INVALID_VALUE == sid_ble_drv_ctx.conn_ctxs[i].public_ctx.conn_id)
        {
            free_ctx = &sid_ble_drv_ctx.conn_ctxs[i];
            sid_ble_drv_ctx.conn_ctxs[i].public_ctx.conn_id = conn_handle;
            break;
        }
    }
    sid_pal_exit_critical_region();

    return free_ctx;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_pal_ble_prv_connection_ctx_t * _ble_adapter_generic_get_conn_ctx_for_handle(const uint16_t conn_handle)
{
    sid_pal_ble_prv_connection_ctx_t * associated_ctx = NULL;

    sid_pal_enter_critical_region();
    for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(sid_ble_drv_ctx.conn_ctxs); i++)
    {
        if (conn_handle == sid_ble_drv_ctx.conn_ctxs[i].public_ctx.conn_id)
        {
            associated_ctx = &sid_ble_drv_ctx.conn_ctxs[i];
            break;
        }
    }
    sid_pal_exit_critical_region();

    return associated_ctx;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_stack_init(const sid_pal_ble_prv_operating_mode_t init_type)
{
    sid_error_t         err = SID_ERROR_GENERIC;
    SNVMA_Cmd_Status_t  snvma_err;
    tBleStatus          ble_err;

    SID_PAL_LOG_DEBUG("==>> Start ble_stack_init function");

    do
    {
        /* Ensure we have a valid config to proceed with initialization */
        if (NULL == sid_ble_drv_ctx.cfg)
        {
            SID_PAL_LOG_ERROR("  Fail   : Cannot initialize BLE stack, config is NULL");
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Initialize stack event queue */
        SID_PAL_ASSERT(((NULL == BleAsynchEventQueue.next) && (NULL == BleAsynchEventQueue.prev)) || (LST_is_empty(&BleAsynchEventQueue) != FALSE));
        LST_init_head(&BleAsynchEventQueue);

        /* Initialize RTOS threads and associated resources */
        if ((BleHostSemaphore != NULL) || (ble_host_task != NULL))
        {
            err = SID_ERROR_ALREADY_INITIALIZED;
            break;
        }
        BleHostSemaphore = osSemaphoreNew(1u, 0u, &ble_host_semaphore_atributes);
        ble_host_task = osThreadNew(BleStack_Process_BG_Entry, NULL, &ble_host_task_atributes);
        if ((NULL == BleHostSemaphore) || (NULL == ble_host_task))
        {
            err = SID_ERROR_OUT_OF_RESOURCES;
            break;
        }

        /*----------------------------------------------------------------------------*/

        if ((hci_async_evt_semaphore != NULL) || (hci_async_evt_task != NULL))
        {
            err = SID_ERROR_ALREADY_INITIALIZED;
            break;
        }
        hci_async_evt_semaphore = osSemaphoreNew(1u, 0u, &hci_async_evt_semaphore_attributes);
        hci_async_evt_task = osThreadNew(Ble_UserEvtRx_Entry, NULL, &hci_async_evt_task_attributes);
        if ((NULL == hci_async_evt_semaphore) || (NULL == hci_async_evt_task))
        {
            err = SID_ERROR_OUT_OF_RESOURCES;
            break;
        }

        /*----------------------------------------------------------------------------*/

#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0)
        UTIL_TIMER_Status_t timer_err;

        if ((sid_ble_drv_ctx.advertising_cmd_queue != NULL) || (sid_ble_drv_ctx.advertising_cmd_task != NULL))
        {
            err = SID_ERROR_ALREADY_INITIALIZED;
            break;
        }
        sid_ble_drv_ctx.advertising_cmd_queue = osMessageQueueNew(SID_STM32_BLE_PRV_ADVERTISING_CMD_QUEUE_LEN, SID_STM32_UTIL_STRUCT_MEMBER_SIZE(UTIL_TIMER_Object_t, argument), &advertising_cmd_queue_attributes);
        sid_ble_drv_ctx.advertising_cmd_task = osThreadNew(_ble_adapter_generic_adv_cmd_task_entry, NULL, &advertising_cmd_task_attributes);
        if ((NULL == sid_ble_drv_ctx.advertising_cmd_queue) || (NULL == sid_ble_drv_ctx.advertising_cmd_task))
        {
            err = SID_ERROR_OUT_OF_RESOURCES;
            break;
        }

        /* Create timer to enter Low Power Advertising */
        timer_err = UTIL_TIMER_Create(&(sid_ble_drv_ctx.adv_timer),
                                      0u, /* Timer period is updated dynamically on BLE advertisement start */
                                      UTIL_TIMER_ONESHOT,
                                      &_ble_adapter_generic_adv_duration_timer_cb, NULL);
        if (timer_err != UTIL_TIMER_OK)
        {
            SID_PAL_LOG_ERROR("  Fail   : Advertisement timer create, result: %d", (int32_t)timer_err);
            err = SID_ERROR_OUT_OF_RESOURCES;
            break;
        }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        /*----------------------------------------------------------------------------*/

        /* Initialize NVM RAM buffer, invalidate it's content before restore */
        SID_STM32_UTIL_fast_memset(host_nvm_buffer, 0u, sizeof(host_nvm_buffer));

        /* Register the APP BLE buffer */
        snvma_err = SNVMA_Register(APP_BLE_NvmBuffer,
                                   (uint32_t *)host_nvm_buffer,
                                   (CFG_BLE_NVM_SIZE_MAX * 2u));
        if (snvma_err != SNVMA_ERROR_OK)
        {
            SID_PAL_LOG_ERROR("  Fail   : SNVMA_Register command, result: 0x%02X", snvma_err);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Restore buffer from flash */
        snvma_err = SNVMA_Restore(APP_BLE_NvmBuffer);
        if ((snvma_err != SNVMA_ERROR_OK) && (snvma_err != SNVMA_ERROR_NVM_BANK_EMPTY) && (snvma_err != SNVMA_ERROR_NVM_BANK_CORRUPTED))
        {
            SID_PAL_LOG_ERROR("  Fail   : SNVMA_Restore command, result: 0x%02X", snvma_err);
            err = SID_ERROR_STORAGE_READ_FAIL;
            break;
        }

        /* Initialize the BLE Host */
        ble_err = HOST_BLE_Init(sid_ble_drv_ctx.cfg, init_type, host_nvm_buffer, &sid_ble_drv_ctx.global_att_mtu_size);
        if (ble_err != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : HOST_BLE_Init command, result: 0x%02X", ble_err);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /**
         * Workaround across lack of de-initialization support in LL - as LL will call ll_sys_dependencies_init() only at first startup, we additionally
         * call it here every time BLE stack is (re)initialized. This won't cause any troubles with LL since after ll_sys_dependencies_init() is completed
         * it sets an indication that initialization is performed and it skips repetitive initialization till ll_sys_dependencies_deinit() call
         *
         * Call to ll_sys_dependencies_init() shall follow the call to HOST_BLE_Init() to guarantee that LL's memory manager is ready
         */
        ll_sys_dependencies_init();

#ifndef SID_PAL_ASSERT_DISABLED
        /* Verify the actual WPAN stack version matches the expected one. This is the protection from systematic failures when incompatible static libraries were linked */
        uint32_t stack_version[2], options[1], debug_info[3];
        ble_err = aci_get_information(stack_version, options, debug_info);
        SID_PAL_ASSERT(BLE_STATUS_SUCCESS == ble_err);

        const uint16_t stack_build_number = (uint16_t)(stack_version[1] & 0xFFFFu);
        SID_PAL_ASSERT(SID_STM32_WPAN_STACK_EXPECTED_BUILD_NUMBER == stack_build_number);
#endif /* SID_PAL_ASSERT_DISABLED */
  
        err = SID_ERROR_NONE;
    } while (0);

    SID_PAL_LOG_DEBUG("==>> End ble_stack_init function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _reload_bonded_devices(void)
{
    sid_error_t err = SID_ERROR_GENERIC;
    sid_error_t ble_list_error = SID_ERROR_NONE;
    tBleStatus ble_status;

    do
    {
        /* Check if bonding is used at all */
        if (sid_ble_drv_ctx.security_cfg.enable_bonding != FALSE)
        {
            /* Load bonded devices from the controller/NVM */
            Bonded_Device_Entry_t bonded_list[SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES + SID_STM32_BLE_BONDING_LIST_OVERHEAD_ENTRIES];
            List_Entry_t * resolving_list_entries;
            uint8_t bond_cnt;

            /* Compile-time checks to ensure List_Entry_t and Bonded_Device_Entry_t definitions are interchangeable */
            static_assert(sizeof(List_Entry_t) == sizeof(Bonded_Device_Entry_t));
            static_assert(SID_STM32_UTIL_STRUCT_MEMBER_SIZE(List_Entry_t, Address_Type) == SID_STM32_UTIL_STRUCT_MEMBER_SIZE(Bonded_Device_Entry_t, Address_Type));
            static_assert(SID_STM32_UTIL_STRUCT_MEMBER_SIZE(List_Entry_t, Address) == SID_STM32_UTIL_STRUCT_MEMBER_SIZE(Bonded_Device_Entry_t, Address));
            static_assert(offsetof(List_Entry_t, Address_Type) == offsetof(Bonded_Device_Entry_t, Address_Type));
            static_assert(offsetof(List_Entry_t, Address) == offsetof(Bonded_Device_Entry_t, Address));

            /* Get the list of the bonded devices */
            ble_status = aci_gap_get_bonded_devices(&bond_cnt, bonded_list);
            if (ble_status != BLE_STATUS_SUCCESS)
            {
              SID_PAL_LOG_ERROR("  Fail   : aci_gap_get_bonded_devices command, result: 0x%02X", ble_status);
                err = SID_ERROR_IO_ERROR;
                break;
            }

            /* Manage the quantity of the bonded devices */
            if (bond_cnt > SID_STM32_UTIL_ARRAY_SIZE(bonded_list))
            {
                /* This is a fatal error, we don't know what data got corrupted as a result */
                SID_PAL_LOG_ERROR("Buffer overflow on loading bonded BLE devices list. Buffer size: %u, actual devices: %u", SID_STM32_UTIL_ARRAY_SIZE(bonded_list), bond_cnt);
                err = SID_ERROR_BUFFER_OVERFLOW;
                break;
            }
#if SID_STM32_BLE_AUTO_MANAGE_OLD_BONDS
            else if (bond_cnt > SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES)
            {
                /* The number of bondings exceed the limit - delete the oldest records */
                for (uint32_t i = 0u; i < (bond_cnt - SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES); i++)
                {
                    ble_status = aci_gap_remove_bonded_device(bonded_list[i].Address_Type, bonded_list[i].Address);
                    if (ble_status != BLE_STATUS_SUCCESS)
                    {
                        SID_BLE_GENERIC_LOG_ERROR("  Fail   : aci_gap_remove_bonded_device command, result: 0x%02X", ble_status);
                        /* Don't react on the error since this is not critical */
                    }
                }

                SID_PAL_LOG_WARNING("Max number of BLE bondings (%u) exceeded. %u oldest bond(s) removed", SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES, (bond_cnt - SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES));

                /* Use only the SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES latest entries, skip the oldest ones in the RAM buffer */
                resolving_list_entries = (List_Entry_t *)&bonded_list[bond_cnt - SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES];
                bond_cnt = SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES; /* The list was trimmed to contain exactly SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES records */
            }
#endif /* SID_STM32_BLE_AUTO_MANAGE_OLD_BONDS */
            else
            {
                /* Nothing to do, there are no excessive bondings in the list */
                resolving_list_entries = (List_Entry_t *)bonded_list;
            }

            /* Add existing bonded devices to the address resolution list and access filter list */
            ble_status = aci_gap_add_devices_to_list(bond_cnt, resolving_list_entries, SID_STM32_BLE_ADD_TO_LIST_MODE_CLEAR_SET_BOTH);
            if (ble_status != BLE_STATUS_SUCCESS)
            {
                /* This is fine - BLE controller is advertising or a new connection is pending, proceed normally and indicate that lists should be updated at a later time */
                ble_list_error = SID_ERROR_BUSY;
                SID_BLE_GENERIC_LOG_ERROR("  Fail   : aci_gap_add_devices_to_list command, result: 0x%02X", ble_status);
            }
            else
            {
                SID_PAL_LOG_INFO("(Re)loaded %u BLE bonds", bond_cnt);
            }
        }

        /* Add local host to the address resolution database to enable RPA in advertisements */
        List_Entry_t own_identity_addr;
#if SID_STM32_BLE_USE_RANDOM_IDENTITY_ADDRESS
        sid_pal_ble_prv_bt_addr_buffer_t static_rand_addr;
        err = sid_stm32wba_ble_adapter_util_get_host_static_random_addr(&static_rand_addr);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("  Fail   : unable to load BLE static random address");
            break;
        }

        own_identity_addr.Address_Type = SID_BLE_HCI_ADV_PEER_ADDR_TYPE_RANDOM;
        SID_STM32_UTIL_fast_memcpy(own_identity_addr.Address, static_rand_addr.bytes, sizeof(own_identity_addr.Address));
#else
        own_identity_addr.Address_Type = SID_BLE_HCI_ADV_PEER_ADDR_TYPE_PUBLIC;
        SID_STM32_UTIL_fast_memcpy(own_identity_addr.Address, pub_addr.bytes, sizeof(own_identity_addr.Address));
#endif /* SID_STM32_BLE_USE_RANDOM_IDENTITY_ADDRESS */

        ble_status = aci_gap_add_devices_to_list(1u, &own_identity_addr, SID_STM32_BLE_ADD_TO_LIST_MODE_APPEND_RESOLVING_ONLY);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            /* This is fine - BLE controller is advertising or a new connection is pending, proceed normally and indicate that lists should be updated at a later time */
            ble_list_error = SID_ERROR_BUSY;
            SID_BLE_GENERIC_LOG_ERROR("  Fail   : aci_gap_add_devices_to_list command, result: 0x%02X", ble_status);
        }
        else
        {
        	SID_BLE_GENERIC_LOG_DEBUG("  Success: aci_gap_add_devices_to_list command");
        }

        /* Done */
        err = ble_list_error;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _gap_and_gatt_params_init(const sid_pal_ble_prv_operating_mode_t init_type)
{
    sid_error_t err = SID_ERROR_GENERIC;

    SID_PAL_LOG_DEBUG("==>> Start gap_and_gatt_params_init function");

    do
    {
        uint8_t         gap_role = 0u;
        uint16_t        gap_appearance = CFG_GAP_APPEARANCE;
        int16_t         tx_power;
        tBleStatus      ret = BLE_STATUS_INVALID_PARAMS;

        /* Validate inputs */
        if (NULL == sid_ble_drv_ctx.cfg)
        {
            SID_PAL_LOG_ERROR("  Fail   : Cannot initialize BLE GAP and GATT params. BLE config is NULL");
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Ensure init_type is aligned with the static driver config */
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_NONE)
        if (init_type != SPBP_OPERATING_MODE_SIDEWALK)
#elif (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING)
        if (init_type == SPBP_OPERATING_MODE_CONCURRENT)
#elif (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
        if (init_type != SPBP_OPERATING_MODE_CONCURRENT)
#else
        /* Unconditionally report the error */
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */
        {
            SID_PAL_LOG_ERROR("  Fail   : Mismatch between the BLE driver cfg and requested init type. Cfg mode: %u, init type: %u", SID_STM32_BLE_COEXISTENCE_MODE, (uint32_t)init_type);
        }

        /*----------------------------------------------------------------------------*/

        /* Configure LE event mask - stack default is 0x000000C7FFF7F85F */
        const uint64_t hci_le_event_mask = (
                            /* SID_BLE_HCI_LE_EVENT_MASK_CONN_COMPLETE - not used, driver relies on SID_BLE_HCI_LE_EVENT_MASK_ENHANCED_CONN_COMPLETE event */
                               SID_BLE_HCI_LE_EVENT_MASK_ADV_REPORT
                             | SID_BLE_HCI_LE_EVENT_MASK_CONN_UPDATE_COMPLETE
                             | SID_BLE_HCI_LE_EVENT_MASK_READ_REMOTE_FEATURES_COMPLETE
                             | SID_BLE_HCI_LE_EVENT_MASK_LTK_REQUEST
                             | SID_BLE_HCI_LE_EVENT_MASK_REMOTE_CONN_PARAMETER_REQ
                             | SID_BLE_HCI_LE_EVENT_MASK_DATA_LENGTH_CHANGE
                            /* SID_BLE_HCI_LE_EVENT_MASK_READ_LOCAL_P256_KEY_COMPLETE - not used */
                            /* SID_BLE_HCI_LE_EVENT_MASK_GENERATE_DHKEY_COMPLETE - not used */
                             | SID_BLE_HCI_LE_EVENT_MASK_ENHANCED_CONN_COMPLETE
                            /* SID_BLE_HCI_LE_EVENT_MASK_DIRECTED_ADV_REPORT - not used */
                             | SID_BLE_HCI_LE_EVENT_MASK_PHY_UPDATE_COMPLETE
                             | SID_BLE_HCI_LE_EVENT_MASK_EXTENDED_ADV_REPORT
                             | SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SYNC_ESTABLISHED
                             | SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_REPORT
                             | SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SYNC_LOST
                             | SID_BLE_HCI_LE_EVENT_MASK_SCAN_TIMEOUT
                             | SID_BLE_HCI_LE_EVENT_MASK_ADV_SET_TERMINATED
                             | SID_BLE_HCI_LE_EVENT_MASK_SCAN_REQUEST_RECEIVED
                            /* SID_BLE_HCI_LE_EVENT_MASK_CHANNEL_SELECTION_ALGO - not used */
                             | SID_BLE_HCI_LE_EVENT_MASK_CONNECTIONLESS_IQ_REPORT
                             | SID_BLE_HCI_LE_EVENT_MASK_CONN_IQ_REPORT
                             | SID_BLE_HCI_LE_EVENT_MASK_CTE_REQUEST_FAILED
                             | SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SYNC_TRANSFER_RECEIVED
                             | SID_BLE_HCI_LE_EVENT_MASK_CIS_ESTABLISHED
                             | SID_BLE_HCI_LE_EVENT_MASK_CIS_REQUEST
                             | SID_BLE_HCI_LE_EVENT_MASK_CREATE_BIG_COMPLETE
                             | SID_BLE_HCI_LE_EVENT_MASK_TERMINATE_BIG_COMPLETE
                             | SID_BLE_HCI_LE_EVENT_MASK_BIG_SYNC_ESTABLISHED
                             | SID_BLE_HCI_LE_EVENT_MASK_BIG_SYNC_LOST
                             | SID_BLE_HCI_LE_EVENT_MASK_REQUEST_PEER_SCA_COMPLETE
                             | SID_BLE_HCI_LE_EVENT_MASK_PATH_LOSS_THRESHOLD
                             | SID_BLE_HCI_LE_EVENT_MASK_TRANSMIT_POWER_REPORTING
                             | SID_BLE_HCI_LE_EVENT_MASK_BIGINFO_ADV_REPORT
                             | SID_BLE_HCI_LE_EVENT_MASK_SUBRATE_CHANGE
                            /* SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SYNC_ESTABLISHED_V2 - not used */
                            /* SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_REPORT_V2 */
                            /* SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SYNC_TRANSFER_RECEIVED_V2 - not used */
                             | SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_SUBEVT_DATA_REQUEST
                             | SID_BLE_HCI_LE_EVENT_MASK_PERIODIC_ADV_RESPONSE_REPORT
                            /* SID_BLE_HCI_LE_EVENT_MASK_ENHANCED_CONN_COMPLETE_V2 - not used */
        );

        ret = hci_le_set_event_mask((void*)&hci_le_event_mask);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : hci_le_set_event_mask command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: hci_le_set_event_mask command");

        /* Load public address into the controller */
        sid_pal_ble_prv_bt_addr_buffer_t pub_addr;
        err = sid_stm32wba_ble_adapter_util_get_host_public_addr(&pub_addr);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("  Fail   : unable to load BLE public address");
            break;
        }
        ret = aci_hal_write_config_data(CONFIG_DATA_PUBLIC_ADDRESS_OFFSET,
                                        CONFIG_DATA_PUBLIC_ADDRESS_LEN,
                                        (uint8_t*)pub_addr.bytes);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_PUBLIC_ADDRESS_OFFSET, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_hal_write_config_data command - CONFIG_DATA_PUBLIC_ADDRESS_OFFSET");
        SID_PAL_LOG_DEBUG("   Public Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x",pub_addr.bytes[5],pub_addr.bytes[4],pub_addr.bytes[3],pub_addr.bytes[2],pub_addr.bytes[1],pub_addr.bytes[0]);

        /* Configure random address  */
        err = sid_stm32wba_ble_adapter_prv_rotate_random_mac_address(NULL, SID_BLE_CFG_MAC_ADDRESS_TYPE_STATIC_RANDOM);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("  Fail   : unable to configure BLE random address");
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: rotate_random_mac_address");

        /* Write local Identity Root Key (IRK) */
        sid_pal_ble_prv_irk_buffer_t irk_seed;
        err = sid_stm32wba_ble_adapter_util_get_local_irk_seed(&irk_seed);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("  Fail   : unable to load local BLE IRK");
            break;
        }
        ret = aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)irk_seed.bytes);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET");

        /* Write Encryption Root Key (ERK) used to derive LTK and CSRK */
        sid_pal_ble_prv_erk_buffer_t erk;
        err = sid_stm32wba_ble_adapter_util_get_erk(&erk);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("  Fail   : unable to load BLE ERK");
            break;
        }
        ret = aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)erk.bytes);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET");

        /* Set TX Power limit */
        ret = aci_hal_set_tx_power_level(0x01u, CFG_BLE_MAX_TX_POWER);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_hal_set_tx_power_level command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_hal_set_tx_power_level command");

        /* Print the configure Tx power to logs */
        tx_power = CFG_BLE_MAX_TX_POWER < SID_STM32_UTIL_ARRAY_SIZE(ble_tx_pwr_to_dbm_lut) ? ble_tx_pwr_to_dbm_lut[CFG_BLE_MAX_TX_POWER] : INT16_MIN;
        /* Compute integer and fractional parts since tiny_vsnprintf may not support %f printout */
        const uint32_t pwr_int_db   = tx_power > 0 ? (uint32_t)(tx_power / 100) : (uint32_t)((-tx_power) / 100);
        const uint32_t pwr_fract_db = tx_power > 0 ? (uint32_t)(tx_power % 100 / 10 ) : (uint32_t)((-tx_power) % 100 / 10);
        SID_PAL_LOG_INFO("BLE max Tx power: %s%u.%udBm", tx_power < 0 ? "-" : "+", pwr_int_db, pwr_fract_db);

        /* Initialize GATT interface */
        ret = aci_gatt_init();
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_gatt_init command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_gatt_init command");

        /* Initialize GAP interface */
        if ((SPBP_OPERATING_MODE_SIDEWALK == init_type) || (SPBP_OPERATING_MODE_CONCURRENT == init_type))
        {
            /* Peripheral role is always required for Sidewalk link */
            gap_role = GAP_PERIPHERAL_ROLE;
        }

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
        if ((SPBP_OPERATING_MODE_USER == init_type) || (SPBP_OPERATING_MODE_CONCURRENT == init_type))
        {
            SID_PAL_ASSERT(sid_ble_drv_ctx.cfg->custom_profiles.virtual_devices != NULL);
            SID_PAL_ASSERT(sid_ble_drv_ctx.cfg->custom_profiles.num_virtual_devices != 0u);

            /* Enrich GAP role with the user-selected options */
            for (uint32_t i = 0u; i < sid_ble_drv_ctx.cfg->custom_profiles.num_virtual_devices; i++)
            {
                const sid_ble_ext_virtual_device_t * const virt_dev_desc = &sid_ble_drv_ctx.cfg->custom_profiles.virtual_devices[i];
                SID_PAL_ASSERT(virt_dev_desc != NULL);

                switch (virt_dev_desc->device_type)
                {
                    case SBEVDR_BLE_PERIPHERAL:
#if SID_STM32_BLE_GAP_ROLE_PERIPHERAL_SUPPORTED
                        gap_role |= GAP_PERIPHERAL_ROLE;
#else
                        SID_PAL_LOG_ERROR("BLE GAP peripheral role is not supported by BLE stack");
                        err = SID_ERROR_NOSUPPORT;
#endif /* SID_STM32_BLE_GAP_ROLE_PERIPHERAL_SUPPORTED */
                        break;

                    case SBEVDR_BLE_BROADCASTER:
#if SID_STM32_BLE_GAP_ROLE_BROADCASTER_SUPPORTED
                        gap_role |= GAP_BROADCASTER_ROLE;
#else
                        SID_PAL_LOG_ERROR("BLE GAP broadcaster role is not supported by BLE stack");
                        err = SID_ERROR_NOSUPPORT;
#endif /* SID_STM32_BLE_GAP_ROLE_BROADCASTER_SUPPORTED */
                        break;

                    case SBEVDR_BLE_CENTRAL:
#if SID_STM32_BLE_GAP_ROLE_CENTRAL_SUPPORTED
                        gap_role |= GAP_CENTRAL_ROLE;
#else
                        SID_PAL_LOG_ERROR("BLE GAP central role is not supported by BLE stack");
                        err = SID_ERROR_NOSUPPORT;
#endif /* SID_STM32_BLE_GAP_ROLE_CENTRAL_SUPPORTED */
                        break;

                    case SBEVDR_BLE_OBSERVER:
#if SID_STM32_BLE_GAP_ROLE_OBSERVER_SUPPORTED
                        gap_role |= GAP_OBSERVER_ROLE;
#else
                        SID_PAL_LOG_ERROR("BLE GAP observer role is not supported by BLE stack");
                        err = SID_ERROR_NOSUPPORT;
#endif /* SID_STM32_BLE_GAP_ROLE_OBSERVER_SUPPORTED */
                        break;

                    default:
                        SID_PAL_LOG_ERROR("Unknown BLE GAP role selected: %u", (uint32_t)virt_dev_desc->device_type);
                        err = SID_ERROR_INVALID_ARGS;
                        break;
                }

                /* Terminate loop if error occurred */
                if (err != SID_ERROR_NONE)
                {
                    break;
                }
            }

            /* Terminate if GAP role selection failed */
            if (err != SID_ERROR_NONE)
            {
                break;
            }
        }
#else
        SID_PAL_ASSERT(SPBP_OPERATING_MODE_SIDEWALK == init_type);
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

        SID_PAL_ASSERT(gap_role != 0u);
        SID_PAL_LOG_DEBUG("   Selected GAP roles: 0x%02X", gap_role);

        /* Prepare GAP device name value */
        const char * gap_device_name = sid_ble_drv_ctx.cfg->sidewalk_profile.device_name;
        uint32_t     gap_device_name_len;

        if (gap_device_name != NULL)
        {
            gap_device_name_len = strlen(gap_device_name);
            if (gap_device_name_len > SID_STM32_BLE_GAP_DEVICE_NAME_MAX_LEN)
            {
                gap_device_name_len = SID_STM32_BLE_GAP_DEVICE_NAME_MAX_LEN;
            }
        }
        else
        {
            gap_device_name_len = 0u;
        }

        /* Initialize GAP */
        ret = aci_gap_init(gap_role,
                           PRIVACY_ENABLED,
                           SID_STM32_BLE_GAP_DEVICE_NAME_MAX_LEN,
                           &sid_ble_drv_ctx.gap_service_handle,
                           &sid_ble_drv_ctx.gap_dev_name_char_handle,
                           &sid_ble_drv_ctx.gap_appearance_char_handle);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_gap_init command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_gap_init command");

        /* Update GAP device name */
        if (gap_device_name_len > 0u)
        {
            ret = aci_gatt_update_char_value(sid_ble_drv_ctx.gap_service_handle,
                                             sid_ble_drv_ctx.gap_dev_name_char_handle,
                                             0u,
                                             gap_device_name_len,
                                             (uint8_t *)gap_device_name);
            if (ret != BLE_STATUS_SUCCESS)
            {
                SID_PAL_LOG_ERROR("  Fail   : aci_gatt_update_char_value - Device Name, result: 0x%02X", ret);
                err = SID_ERROR_IO_ERROR;
                break;
            }
            SID_PAL_LOG_DEBUG("  Success: aci_gatt_update_char_value - Device Name");
        }

        /* Update GAP appearance */
        ret = aci_gatt_update_char_value(sid_ble_drv_ctx.gap_service_handle,
                                         sid_ble_drv_ctx.gap_appearance_char_handle,
                                         0u,
                                         sizeof(gap_appearance),
                                         (uint8_t *)(void *)&gap_appearance);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_gatt_update_char_value - Appearance, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_gatt_update_char_value - Appearance");

        /* Set preferred connection parameters */
        SID_PAL_ASSERT(sid_ble_sidewalk_ctx.ble_cfg != NULL);
        sid_ble_drv_ctx.gap_ppcp_char_handle = sid_ble_drv_ctx.gap_appearance_char_handle + SID_STM32_BLE_PPCP_CHAR_HANDLE_OFFSET;
        err = sid_stm32wba_ble_adapter_util_set_gap_ppcp_char(&sid_ble_sidewalk_ctx.ble_cfg->conn_param);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Initialize Default PHY */
        ret = hci_le_set_default_phy(0x00, HCI_TX_PHYS_LE_2M_PREF, HCI_RX_PHYS_LE_2M_PREF);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : hci_le_set_default_phy command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: hci_le_set_default_phy command");

#if SID_STM32_BLE_DLE_ENABLE
        /* Enable data length extension (DLE) */
        ret = hci_le_write_suggested_default_data_length(SID_STM32_BLE_DLE_MAX_TX_OCTETS, SID_STM32_BLE_DLE_MAX_TX_TIME_US);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : hci_le_write_suggested_default_data_length command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: hci_le_write_suggested_default_data_length command");
#endif /* SID_STM32_BLE_DLE_ENABLE */

        /* Initialize IO capability */
        if (SPBP_OPERATING_MODE_SIDEWALK == init_type)
        {
            /* Sidewalk uses JustWorks method, so we don't need to allocate any resources to support IO capabilities */
            sid_ble_drv_ctx.security_cfg.io_capability = IO_CAP_NO_INPUT_NO_OUTPUT;
        }
        else
        {
            /* Use actual I/O capabilities of the host platform */
            sid_ble_drv_ctx.security_cfg.io_capability = CFG_IO_CAPABILITY;
        }
        ret = aci_gap_set_io_capability(sid_ble_drv_ctx.security_cfg.io_capability);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_gap_set_io_capability command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_gap_set_io_capability command");

        /* Initialize authentication */
        if (SPBP_OPERATING_MODE_SIDEWALK == init_type)
        {
            /* Sidewalk-only config - rely on JustWorks, encryption is implemented on the Sidewalk link layer above the non-secure BLE connection */
            sid_ble_drv_ctx.security_cfg.keypress_support      = KEYPRESS_NOT_SUPPORTED;
            sid_ble_drv_ctx.security_cfg.mitm_mode             = MITM_PROTECTION_NOT_REQUIRED;
            sid_ble_drv_ctx.security_cfg.sc_mode               = SC_PAIRING_UNSUPPORTED;
            sid_ble_drv_ctx.security_cfg.encryptionKeySizeMin  = CFG_ENCRYPTION_KEY_SIZE_MIN;
            sid_ble_drv_ctx.security_cfg.encryptionKeySizeMax  = CFG_ENCRYPTION_KEY_SIZE_MAX;
            sid_ble_drv_ctx.security_cfg.use_fixed_pin         = SID_STM32_BLE_PRV_CI_GAP_SECURITY_CFG_FIXED_PIN_DISABLED;
            sid_ble_drv_ctx.security_cfg.fixed_pin             = CFG_FIXED_PIN;
            sid_ble_drv_ctx.security_cfg.enable_bonding        = FALSE;
            sid_ble_drv_ctx.security_cfg.identity_address_type = GAP_STATIC_RANDOM_ADDR;
        }
        else
        {
            sid_ble_drv_ctx.security_cfg.keypress_support      = CFG_KEYPRESS_NOTIFICATION_SUPPORT;
            sid_ble_drv_ctx.security_cfg.mitm_mode             = CFG_MITM_PROTECTION;
            sid_ble_drv_ctx.security_cfg.sc_mode               = (CFG_SC_SUPPORT == SC_PAIRING_ONLY) && (SPBP_OPERATING_MODE_CONCURRENT == init_type) ? SC_PAIRING_OPTIONAL : CFG_SC_SUPPORT; /* Override SC_PAIRING_ONLY if Sidewalk will run concurrently */
            sid_ble_drv_ctx.security_cfg.encryptionKeySizeMin  = CFG_ENCRYPTION_KEY_SIZE_MIN;
            sid_ble_drv_ctx.security_cfg.encryptionKeySizeMax  = CFG_ENCRYPTION_KEY_SIZE_MAX;
            sid_ble_drv_ctx.security_cfg.use_fixed_pin         = (CFG_ENABLE_FIXED_PIN_PAIRING != 0u) ? SID_STM32_BLE_PRV_CI_GAP_SECURITY_CFG_FIXED_PIN_ENABLED : SID_STM32_BLE_PRV_CI_GAP_SECURITY_CFG_FIXED_PIN_DISABLED;
            sid_ble_drv_ctx.security_cfg.fixed_pin             = CFG_FIXED_PIN;
            sid_ble_drv_ctx.security_cfg.enable_bonding        = CFG_BONDING_MODE;
#if SID_STM32_BLE_USE_RANDOM_IDENTITY_ADDRESS
            sid_ble_drv_ctx.security_cfg.identity_address_type = GAP_STATIC_RANDOM_ADDR;
#else
            sid_ble_drv_ctx.security_cfg.identity_address_type = GAP_PUBLIC_ADDR;
#endif /* SID_STM32_BLE_USE_RANDOM_IDENTITY_ADDRESS */
        }

        ret = aci_gap_set_authentication_requirement(sid_ble_drv_ctx.security_cfg.enable_bonding,
                                                     sid_ble_drv_ctx.security_cfg.mitm_mode,
                                                     sid_ble_drv_ctx.security_cfg.sc_mode,
                                                     sid_ble_drv_ctx.security_cfg.keypress_support,
                                                     sid_ble_drv_ctx.security_cfg.encryptionKeySizeMin,
                                                     sid_ble_drv_ctx.security_cfg.encryptionKeySizeMax,
                                                     sid_ble_drv_ctx.security_cfg.use_fixed_pin,
                                                     sid_ble_drv_ctx.security_cfg.fixed_pin,
                                                     sid_ble_drv_ctx.security_cfg.identity_address_type);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_gap_set_authentication_requirement command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_gap_set_authentication_requirement command");

        /* Always enable RPA resolution in the Controller since any peer may use RPA address */
        ret = hci_le_set_address_resolution_enable(0x01u);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : hci_le_set_address_resolution_enable command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: hci_le_set_address_resolution_enable command");

        /* Configure RPA rotation timeout */
        ret = hci_le_set_resolvable_private_address_timeout(SID_STM32_BLE_RPA_ROTATION_TIMEOUT);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : hci_le_set_resolvable_private_address_timeout command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: hci_le_set_resolvable_private_address_timeout command");


        err = _reload_bonded_devices();
        if (err != SID_ERROR_NONE)
        {
            /* Print out a warning message, but proceed as lost bondings are not a blocking point */
            SID_PAL_LOG_WARNING("Failed to load BLE bondings. Error %d", (int32_t)err);
        }

        /* Configure own address type for ACI layer */
        uint8_t effective_own_addr_type;
        ret = GAP_Set_Own_Address(GAP_RESOLVABLE_PRIVATE_ADDR, &effective_own_addr_type, NULL);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : GAP_Set_Own_Address command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Configure ACI GAP event mask */
        const uint16_t aci_gap_event_mask = (
                            /* SID_STM32_BLE_ACI_GAP_LIMITED_DISCOVERABLE_EVENT - not used */
                               SID_STM32_BLE_ACI_GAP_PAIRING_COMPLETE_EVENT
                             | SID_STM32_BLE_ACI_GAP_PASS_KEY_REQ_EVENT
                             | SID_STM32_BLE_ACI_GAP_AUTHORIZATION_REQ_EVENT
                             | SID_STM32_BLE_ACI_GAP_BOND_LOST_EVENT
                             | SID_STM32_BLE_ACI_GAP_PROC_COMPLETE_EVENT
                             | SID_STM32_BLE_ACI_L2CAP_CONNECTION_UPDATE_REQ_EVENT
                             | SID_STM32_BLE_ACI_L2CAP_CONNECTION_UPDATE_RESP_EVENT
                             | SID_STM32_BLE_ACI_L2CAP_PROC_TIMEOUT_EVENT
                             | SID_STM32_BLE_ACI_GAP_ADDR_NOT_RESOLVED_EVENT
        );
        ret = aci_gap_set_event_mask(aci_gap_event_mask);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_gap_set_event_mask command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_gap_set_event_mask command");

        /* Configure ACI GATT event mask */
        const uint32_t aci_gatt_event_mask = (
                               SID_STM32_BLE_ACI_GATT_ATTRIBUTE_MODIFIED_EVENT_MASK
                             | SID_STM32_BLE_ACI_GATT_PROC_TIMEOUT_EVENT_MASK
                             | SID_STM32_BLE_ACI_ATT_EXCHANGE_MTU_RESP_EVENT_MASK
                            /* | SID_STM32_BLE_ACI_ATT_FIND_INFO_RESP_EVENT_MASK */
                            /* | SID_STM32_BLE_ACI_ATT_FIND_BY_TYPE_VALUE_RESP_EVENT_MASK */
                            /* | SID_STM32_BLE_ACI_ATT_READ_BY_TYPE_RESP_EVENT_MASK */
                            /* | SID_STM32_BLE_ACI_ATT_READ_RESP_EVENT_MASK - use ACI_GATT_READ_EXT_EVENT instead */
                            /* | SID_STM32_BLE_ACI_ATT_READ_BLOB_RESP_EVENT_MASK - use ACI_GATT_READ_EXT_EVENT instead */
                            /* | SID_STM32_BLE_ACI_ATT_READ_MULTIPLE_RESP_EVENT_MASK - use ACI_GATT_READ_EXT_EVENT instead */
                            /* | SID_STM32_BLE_ACI_ATT_READ_BY_GROUP_TYPE_RESP_EVENT_MASK */
                             | SID_STM32_BLE_ACI_ATT_PREPARE_WRITE_RESP_EVENT_MASK
                             | SID_STM32_BLE_ACI_ATT_EXEC_WRITE_RESP_EVENT_MASK
                            /* | SID_STM32_BLE_ACI_GATT_INDICATION_EVENT_MASK - use ACI_GATT_INDICATION_EXT_EVENT instead */
                            /* | SID_STM32_BLE_ACI_GATT_NOTIFICATION_EVENT_MASK - use ACI_GATT_NOTIFICATION_EXT instead */
                             | SID_STM32_BLE_ACI_GATT_ERROR_RESP_EVENT_MASK
                             | SID_STM32_BLE_ACI_GATT_PROC_COMPLETE_EVENT_MASK
                            /* | SID_STM32_BLE_ACI_GATT_DISC_READ_CHAR_BY_UUID_RESP_EVENT_MASK */
                             | SID_STM32_BLE_ACI_GATT_TX_POOL_AVAILABLE_EVENT_MASK
                             | SID_STM32_BLE_ACI_GATT_READ_EXT_EVENT_MASK
                             | SID_STM32_BLE_ACI_GATT_INDICATION_EXT_EVENT_MASK
                             | SID_STM32_BLE_ACI_GATT_NOTIFICATION_EXT_EVENT_MASK
        );
        ret = aci_gatt_set_event_mask(aci_gatt_event_mask);
        if (ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_gatt_set_event_mask command, result: 0x%02X", ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_gatt_set_event_mask command");

        err = SID_ERROR_NONE;
    } while (0);

    SID_PAL_LOG_DEBUG("==>> End gap_and_gatt_params_init function");

    return err;
}

/* Private shared function definitions ---------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_stm32wba_ble_adapter_prv_gap_cmd_arm_resp_wait(const sid_pal_ble_prv_connection_ctx_t * const conn_ctx)
{
    SID_PAL_ASSERT(conn_ctx != NULL);
    SID_PAL_ASSERT(conn_ctx->gap_cmd_lock != NULL);

    /* Ensure Semaphore is locked */
    (void)osSemaphoreAcquire(conn_ctx->gap_cmd_lock, 0u);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_prv_gap_cmd_resp_wait(const sid_pal_ble_prv_connection_ctx_t * const conn_ctx)
{
    sid_error_t    err;
    osStatus_t     os_status;
    const uint32_t t0 = osKernelGetTickCount();
    uint32_t       elapsed_ticks;
    uint32_t       timeout_ticks;

    SID_PAL_ASSERT(conn_ctx != NULL);
    SID_PAL_ASSERT(conn_ctx->gap_cmd_lock != NULL);

    /* Compute maximum wait time for the BLE connection to actually disconnect after GAP command was sent - used to avoid deadlocks */
    const uint32_t initial_disconnect_timeout_ms = BLE_ADAPTER_CALCULATE_DISCONNECT_TIMEOUT_MS(conn_ctx);

    /* Convert wait time to OS ticks */
    timeout_ticks = BLE_ADAPTER_MS_TO_OS_TICKS(initial_disconnect_timeout_ms);

    do
    {
        /* Wait till GAP cmd completion */
        os_status = osSemaphoreAcquire(conn_ctx->gap_cmd_lock, timeout_ticks);

        /* Capture elapsed wait time */
        elapsed_ticks = osKernelGetTickCount() - t0;

        /* Evaluate wait results */
        switch (os_status)
        {
            case osOK:
                /* Everything is fine, BLE disconnected within the timeout */
                SID_BLE_GENERIC_LOG_DEBUG("Actual BLE disconnect time for conn handle 0x%04X: %ums",conn_ctx->public_ctx.conn_id, BLE_ADAPTER_OS_TICKS_TO_MS(elapsed_ticks));
                err = SID_ERROR_NONE;
                break;

            case osErrorParameter:
                /* Most probably this function was called in an IRQ context */
                err = SID_ERROR_INVALID_STATE;
                break;

            case osErrorTimeout:
                /* Timeout on wait, let's check a few things */
                {
                    /* BLE connection parameters, including the supervision interval might have changed during the wait time -> compute the applicable disconnection wait timeout now with the actual connection parameters */
                    const uint32_t new_disconnect_timeout_ms = BLE_ADAPTER_CALCULATE_DISCONNECT_TIMEOUT_MS(conn_ctx);

                    if (BLE_ADAPTER_MS_TO_OS_TICKS(new_disconnect_timeout_ms) > elapsed_ticks)
                    {
                        /* Indeed, the new timeout value is longer -> re-adjust to the new realm and continue to wait */
                        timeout_ticks = BLE_ADAPTER_MS_TO_OS_TICKS(new_disconnect_timeout_ms) - elapsed_ticks;
                        SID_BLE_GENERIC_LOG_DEBUG("Extended BLE disconnect timeout for conn handle 0x%04X by %ums. Original timeout: %u, new: %u",
                                                  conn_ctx->public_ctx.conn_id,
                                                  BLE_ADAPTER_OS_TICKS_TO_MS(timeout_ticks),
                                                  initial_disconnect_timeout_ms,
                                                  new_disconnect_timeout_ms
                                                );
                        err = SID_ERROR_TRY_AGAIN;
                    }
                    else
                    {
                        err = SID_ERROR_TIMEOUT;
                    }
                }
                break;

            default:
                /* Normally this should not ever happen */
                err = SID_ERROR_GENERIC;
                break;
        }
    } while (SID_ERROR_TRY_AGAIN == err);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_prv_rotate_random_mac_address(sid_pal_ble_prv_bt_addr_buffer_t * const out_addr, const sid_ble_cfg_mac_address_type_t addr_type)
{
    sid_pal_ble_prv_bt_addr_buffer_t rand_bd_addr;
    tBleStatus ble_ret;
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Generate the requested random address type */
        switch (addr_type)
        {
            case SID_BLE_CFG_MAC_ADDRESS_TYPE_STATIC_RANDOM:
                err = sid_stm32wba_ble_adapter_util_get_host_static_random_addr(&rand_bd_addr);
                break;

            case SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE:
                err = sid_stm32wba_ble_adapter_util_generate_private_nonresolvable_random_mac_address(&rand_bd_addr);
                break;

            case SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_RESOLVABLE:
                /* For RPA address updates are managed by the BLE Controller, non-resolvable address is used as a fall-back, so rotate non-resolvable address here */
                err = sid_stm32wba_ble_adapter_util_generate_private_nonresolvable_random_mac_address(&rand_bd_addr);
                break;

            case SID_BLE_CFG_MAC_ADDRESS_TYPE_PUBLIC:
            default:
                err = SID_ERROR_INVALID_ARGS;
                break;
        }

        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("  Fail   : Unable to generate random device address, error code: %d", (int32_t)err);
            break;
        }

        /* Store random address value to configuration */
        ble_ret = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)rand_bd_addr.bytes);
        if (ble_ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET, result: 0x%02X", ble_ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET");

        /* Store random address value to host controller */
        ble_ret = hci_le_set_random_address((uint8_t*)rand_bd_addr.bytes);
        if (ble_ret != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : hci_le_set_random_address command, result: 0x%02X", ble_ret);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("  Success: hci_le_set_random_address command");

        SID_PAL_LOG_DEBUG("   Random Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x", rand_bd_addr.bytes[5],
                                                                                        rand_bd_addr.bytes[4],
                                                                                        rand_bd_addr.bytes[3],
                                                                                        rand_bd_addr.bytes[2],
                                                                                        rand_bd_addr.bytes[1],
                                                                                        rand_bd_addr.bytes[0]);

        /* Store the generated address if requested */
        if (out_addr != NULL)
        {
            SID_STM32_UTIL_fast_memcpy(out_addr, &rand_bd_addr, sizeof(out_addr));
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_prv_platform_init(void)
{
    sid_error_t err;

    do
    {
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
        err = sid_stm32wba_ble_adapter_user_platform_init();
        if (err != SID_ERROR_NONE)
        {
            break;
        }
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_prv_generic_init(const sid_pal_ble_prv_operating_mode_t init_type, const sid_ble_config_t * const sid_ble_cfg)
{
    sid_error_t err = SID_ERROR_GENERIC;

    SID_PAL_LOG_DEBUG("==>> Start ble_adapter_generic_init function");

    do
    {
        /* Run in a critical section to avoid race conditions during stack initialization */
        sid_pal_enter_critical_region();
        if (sid_ble_drv_ctx.init_counter != 0u)
        {
            /* BLE hardware and stack is initialized already, no actions required */
            sid_ble_drv_ctx.init_counter++;
            err = SID_ERROR_NONE;
            __COMPILER_BARRIER();
            sid_pal_exit_critical_region();
            break;
        }
        else
        {
            /* Increment the counter now since we have to leave the critical section to allow RTOS resources creation (CMSIS cannot allocate resource in a critical section) */
            sid_ble_drv_ctx.init_counter++;
            __COMPILER_BARRIER();
        }
        sid_pal_exit_critical_region();

        /* Invalidate all connection contexts now, even if there are not initialized yet */
        _ble_adapter_generic_invalidate_conn_ctx(NULL);

        /* Get the extended config from the user app */
        const sid_ble_adapter_ext_cfg_t * const ext_cfg = sid_pal_ble_adapter_ext_get_link_config();

        /* Validate the config */
        if (NULL == ext_cfg)
        {
            SID_PAL_LOG_ERROR("Extended BLE adapter config is NULL");
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Store extended config for future use */
        sid_ble_drv_ctx.cfg = ext_cfg;

        /* Load Sidewalk-specific BLE parameters from the fixed driver config */
        sid_ble_sidewalk_ctx.ble_cfg = sid_ble_cfg;
        SID_PAL_ASSERT(sid_ble_sidewalk_ctx.ble_cfg != NULL);
        SID_PAL_ASSERT((sid_ble_sidewalk_ctx.ble_cfg->adv_param.fast_enabled != FALSE) || (sid_ble_sidewalk_ctx.ble_cfg->adv_param.slow_enabled != FALSE)); /* At least one advertisement option shall be enabled */

        /* Init stack library and associated OS resources */
        err = _ble_stack_init(init_type);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Initialize BLE Host Controller */
        err = _gap_and_gatt_params_init(init_type);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
        uint8_t    supported_adv_sets_num = 0u;
        tBleStatus ble_status;

        /* Check the host controller supports the required number of advertisement sets */
        ble_status = hci_le_read_number_of_supported_advertising_sets(&supported_adv_sets_num);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("==>> hci_le_read_number_of_supported_advertising_sets - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        if (SID_STM32_BLE_GAP_MAX_ADV_SETS_NUM > supported_adv_sets_num)
        {
            SID_PAL_LOG_ERROR("BLE config mismatch. Software needs %u advertisement sets, but hardware supports only %u", SID_STM32_BLE_GAP_MAX_ADV_SETS_NUM, supported_adv_sets_num);
            err = SID_ERROR_INVALID_STATE;
            break;
        }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        /* Generic init before any GATT services can be added */
        SVCCTL_Init();

        /* Setup local SVC event handlers management */
        _ble_adapter_generic_svc_event_handling_init();

        err = _ble_adapter_generic_conn_ctxs_init();
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_generic_conn_ctxs_init() */
            break;
        }

        /* Store the activated driver operating mode */
        sid_ble_drv_ctx.operating_mode = init_type;

        /* Disable RFTS Bypass for flash operation - Since LL is about to start */
        FD_SetStatus(FD_FLASHACCESS_RFTS_BYPASS, LL_FLASH_DISABLE);

        /* From here on, all initialization is BLE application specific */
        SID_PAL_LOG_DEBUG("  Generic BLE init done");

        err = SID_ERROR_NONE;
    } while (0);

    /* Release partially allocated resources if init has failed */
    if (err != SID_ERROR_NONE)
    {
        (void)sid_stm32wba_ble_adapter_prv_generic_deinit();
    }

    SID_PAL_LOG_DEBUG("==>> End ble_adapter_generic_init function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_prv_generic_deinit(void)
{
    sid_error_t err = SID_ERROR_GENERIC;
    sid_error_t rtos_resource_deinit_err = SID_ERROR_NONE;
    osStatus_t  os_status;
    tBleStatus  ble_staus;
    uint32_t    ll_lock_acquired = FALSE;

    SID_PAL_LOG_DEBUG("==>> Start ble_adapter_generic_deinit function");

    /* Ensure LL lock is acqwuired by trhe caller */
    if (osMutexGetOwner(LinkLayerMutex) != osThreadGetId())
    {
        /* Wait for any potentially ongoing BLE operations to stop */
        os_status = osMutexAcquire(LinkLayerMutex, SID_STM32_BLE_LL_MUTEX_WAIT_TIMEOUT_ON_DEINIT_TICKS);

        /* Print out warning but proceed */
        if (os_status != osOK)
        {
            SID_PAL_LOG_WARNING("Failed to acquire BLE LL mutex, race conditions are possible during BLE deinit");
        }
        else
        {
            ll_lock_acquired = TRUE;
        }
    }

    do
    {
        sid_pal_enter_critical_region();
        if (0u == sid_ble_drv_ctx.init_counter)
        {
            /* Deinitialized already */
            sid_pal_exit_critical_region();
            err = SID_ERROR_NONE;
            break;
        }
        else
        {
            /* Decrement initialization counter */
            sid_ble_drv_ctx.init_counter--;
            __DSB();
        }

        /* Check if any driver references left */
        if (sid_ble_drv_ctx.init_counter > 0u)
        {
            /* There are still some active driver users, keep  the driver initialized */
            sid_pal_exit_critical_region();
            err = SID_ERROR_NONE;
            break;
        }

        /* No active users left, proceed with full deinitialization */
        sid_pal_exit_critical_region();

        /*----------------------------------------------------------------------------*/

        /*Force HCI into standby mode*/
        if (ll_sys_dp_slp_get_state() == LL_SYS_DP_SLP_DISABLED)
        {
            /* No next radio event scheduled */
            (void)ll_sys_dp_slp_enter(LL_DP_SLP_NO_WAKEUP);
        }

        /* Deallocate RTOS resources */
#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0)
        sid_pal_enter_critical_region();
        /* Don't care if the timer was actually stopped */
        (void)UTIL_TIMER_Stop(&sid_ble_drv_ctx.adv_timer);
        sid_ble_drv_ctx.adv_timer.Callback = NULL;
        sid_ble_drv_ctx.adv_timer.argument = NULL;
        sid_pal_exit_critical_region();

        os_status = osThreadTerminate(sid_ble_drv_ctx.advertising_cmd_task);
        sid_ble_drv_ctx.advertising_cmd_task = NULL;
        os_status != osOK ? rtos_resource_deinit_err = SID_ERROR_UNINITIALIZED : (void)0;
        os_status = osMessageQueueDelete(sid_ble_drv_ctx.advertising_cmd_queue);
        sid_ble_drv_ctx.advertising_cmd_queue = NULL;
        os_status != osOK ? rtos_resource_deinit_err = SID_ERROR_UNINITIALIZED : (void)0;
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        os_status = osThreadTerminate(hci_async_evt_task);
        hci_async_evt_task = NULL;
        os_status != osOK ? rtos_resource_deinit_err = SID_ERROR_UNINITIALIZED : (void)0;
        os_status = osSemaphoreDelete(hci_async_evt_semaphore);
        os_status != osOK ? rtos_resource_deinit_err = SID_ERROR_UNINITIALIZED : (void)0;
        hci_async_evt_semaphore = NULL;

        os_status = osThreadTerminate(ble_host_task);
        ble_host_task = NULL;
        os_status != osOK ? rtos_resource_deinit_err = SID_ERROR_UNINITIALIZED : (void)0;
        os_status = osSemaphoreDelete(BleHostSemaphore);
        BleHostSemaphore = NULL;
        os_status != osOK ? rtos_resource_deinit_err = SID_ERROR_UNINITIALIZED : (void)0;

        ble_staus = BLE_TIMER_Deinit();
        ble_staus != osOK ? rtos_resource_deinit_err = SID_ERROR_UNINITIALIZED : (void)0;

        /* Free up connection contexts */
        _ble_adapter_generic_conn_ctxs_deinit();

        sid_ble_drv_ctx.operating_mode = SPBP_OPERATING_MODE_OFF;

        /* Clear any residual unprocessed BLE events */
        sid_pal_enter_critical_region();
        while (LST_is_empty(&BleAsynchEventQueue) == FALSE)
        {
            BleEvtPacket_t *phcievt;

            LST_remove_head (&BleAsynchEventQueue, (tListNode **)&phcievt);
            AMM_Free((uint32_t *)phcievt);
        }
        sid_pal_exit_critical_region();

        /**
         * Workaround across lack of de-initialization support in LL - call custom de-initalization code to release LL task, Temperature Measurement task,
         * and associated resources (timers, mutexes, semaphores, etc.)
         *
         * WARNING: LinkLayerMutex becomes invalid after this point
         */
        ll_sys_dependencies_deinit();

        /* Enabble RFTS Bypass for flash operation - since LL is terminated */
        FD_SetStatus(FD_FLASHACCESS_RFTS_BYPASS, LL_FLASH_ENABLE);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if ((ll_lock_acquired != FALSE) && (LinkLayerMutex != NULL))
    {
        os_status = osMutexRelease(LinkLayerMutex);
        if (os_status != osOK)
        {
            SID_PAL_LOG_ERROR("Failed to release BLE LL mutex. BLE stack will be blocked till reset");
            err = SID_ERROR_UNRECOVERABLE;
        }
    }

    /* Select error to report */
    err = (err != SID_ERROR_NONE ? err : rtos_resource_deinit_err);

    SID_PAL_LOG_DEBUG("==>> End ble_adapter_generic_deinit function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_prv_generic_start_advertisement(Adv_Set_t * const adv_set)
#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus  ble_status;

    do
    {
        /* Validate inputs */
        if (NULL == adv_set)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (SPBP_OPERATING_MODE_OFF == sid_ble_drv_ctx.operating_mode)
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Start advertising */
        ble_status = hci_le_set_extended_advertising_enable(SID_BLE_HCI_SET_ADV_ENABLE, 1u, adv_set);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("==>> hci_le_set_extended_advertising_enable - fail, result: 0x%02X, set: 0x%02X", ble_status, adv_set->Advertising_Handle);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("==>> hci_le_set_extended_advertising_enable - Success, set: 0x%02X", adv_set->Advertising_Handle);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}
#else
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus  ble_status;

    /** 
     * Concurrency mode is not supported without the extended advertising. Normally this should never happen
     * unless there's systematic failure in the configuration.
     */
    SID_PAL_ASSERT(sid_ble_drv_ctx.operating_mode != SPBP_OPERATING_MODE_CONCURRENT);

    do
    {
        /* Validate inputs */
        if (NULL == adv_set)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (SPBP_OPERATING_MODE_OFF == sid_ble_drv_ctx.operating_mode)
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        const uint32_t adv_duration = adv_set->Duration;

        /* Start advertising */
        ble_status = hci_le_set_advertising_enable(SID_BLE_HCI_SET_ADV_ENABLE);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("==>> hci_le_set_advertising_enable - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("==>> hci_le_set_advertising_enable - Success");

        /* Check if this advertisement is limited in time */
        if (adv_duration != 0u)
        {
            /* Set reference to the advertised link type */
            sid_ble_drv_ctx.adv_timer.argument = (void *)(uint32_t)adv_set->Advertising_Handle;

            /* Start the timer */
            UTIL_TIMER_Status_t timer_ret = UTIL_TIMER_StartWithPeriod(&sid_ble_drv_ctx.adv_timer, SID_STM32_BLE_ADV_DURATION_UNITS_TO_MS(adv_duration));
            if (timer_ret != UTIL_TIMER_OK)
            {
                SID_PAL_LOG_ERROR("Cannot start advertisement timer. Advertisement will run indefinitely. Error %d", timer_ret);
                err = SID_ERROR_IO_ERROR;
                break;
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_prv_generic_stop_advertisement(const uint32_t adv_set_handle)
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus  ble_status;

    do
    {
#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
        /* Validate inputs */
        if (adv_set_handle > SID_BLE_HCI_ADV_HANDLE_MAX)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Stop advertisement for the specified set */
        const Adv_Set_t adv_set = {
            .Advertising_Handle              = (uint8_t)adv_set_handle,
            .Duration                        = 0u,
            .Max_Extended_Advertising_Events = 0u,
        };

        ble_status = hci_le_set_extended_advertising_enable(SID_BLE_HCI_SET_ADV_DISABLE, 1u, &adv_set);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("hci_le_set_extended_advertising_disable - fail, result: 0x%02X, set: 0x%02X", ble_status, adv_set_handle);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_PAL_LOG_DEBUG("==>> hci_le_set_extended_advertising_disable - Success, set: 0x%02X", adv_set_handle);

        /* Done */
        err = SID_ERROR_NONE;
#else
        /* Extended advertising is not supported, fall back to the legacy API */
        UTIL_TIMER_Status_t timer_err;

        (void)adv_set_handle;

        /* Ensure the advertisement timer is stopped */
        timer_err = UTIL_TIMER_Stop(&sid_ble_drv_ctx.adv_timer);
        /* Don't terminate here - we still need to send HCI command to stop the advertisement */

        /* Stop the actual advertisement */
        ble_status = hci_le_set_advertising_enable(SID_BLE_HCI_SET_ADV_DISABLE);

        /* Process the results */
        err = SID_ERROR_NONE;
        if (timer_err != UTIL_TIMER_OK)
        {
            SID_PAL_LOG_ERROR("Failed to stop BLE advertisement timer. Error %u", timer_err);
            err = SID_ERROR_IO_ERROR;
            /* Don't terminate, process the HCI return value */
        }
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("hci_le_set_advertising_disable - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
        }
        SID_PAL_LOG_DEBUG("==>> hci_le_set_advertising_disable - Success");
        if (err != SID_ERROR_NONE)
        {
            /* Terminate from here */
            break;
        }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

#if (SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING && SID_STM32_BLE_GATT_EXCHANGE_CONFIG_SUPPORTED)
/**
 * If the enhanced MTU exchange handling is enabled and GCC toolchain is used, the ATT_Build_Exchg_Mtu_Req() and
 * ATT_Build_Exchg_Mtu_Rsp() functions should be wrapped by linker using -Wl,--wrap=ATT_Build_Exchg_Mtu_Req and
 * -Wl,--wrap=ATT_Build_Exchg_Mtu_Rsp flags.
 */
SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(SID_STM32_SPEED_OPTIMIZED tBleStatus, ATT_Build_Exchg_Mtu_Req(sid_pal_ble_prv_conn_ctx_mock_t * const bearer, sid_pal_ble_prv_client_pdu_mock_t * const out_exch_mtu_req))
{
    tBleStatus ble_status;

    SID_PAL_ASSERT(bearer != NULL);
    SID_PAL_ASSERT(out_exch_mtu_req != NULL);

    do
    {
        /* Call the original method to build Exchange MTU Request PDU */
        ble_status = SID_STM32_FUNC_WRAPPER_CALL_ORIGINAL_FUNCTION(ATT_Build_Exchg_Mtu_Req(bearer, out_exch_mtu_req));
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            break;
        }

        /* Checks to detect if local definitions are not aligned with the BLE stack */
        SID_PAL_ASSERT(SID_STM32_BLE_PRV_ATT_EXCHANGE_MTU_REQ_SZ == out_exch_mtu_req->frame_len);
        SID_PAL_ASSERT(out_exch_mtu_req->l2cap_frame != NULL);
        SID_PAL_ASSERT(SID_STM32_BLE_PRV_ATT_EXCHANGE_MTU_REQ_OPCODE == out_exch_mtu_req->l2cap_frame->att_pdu.opcode);

        /* Locate the associated generic connection context for the connection */
        const sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_conn_ctx_for_handle(bearer->conn_handle);

        /* Apply connection-specific MTU restrictions if the context exists */
        if (conn_ctx != NULL)
        {
            /* Ensure the requested connection-specific ATT MTU size does not exceed the one set in the controller */
            SID_PAL_ASSERT(conn_ctx->public_ctx.max_mtu_size <= out_exch_mtu_req->l2cap_frame->att_pdu.exchange_mtu_req.client_rx_mtu);
            out_exch_mtu_req->l2cap_frame->att_pdu.exchange_mtu_req.client_rx_mtu = conn_ctx->public_ctx.max_mtu_size;
        }

        ble_status = BLE_STATUS_SUCCESS;
    } while (0);

    return ble_status;
}
#endif /* SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING && SID_STM32_BLE_GATT_EXCHANGE_CONFIG_SUPPORTED */

/*----------------------------------------------------------------------------*/

#if SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING
/**
 * If the enhanced MTU exchange handling is enabled and GCC toolchain is used, the ATT_Build_Exchg_Mtu_Req() and
 * ATT_Build_Exchg_Mtu_Rsp() functions should be wrapped by linker using -Wl,--wrap=ATT_Build_Exchg_Mtu_Req and
 * -Wl,--wrap=ATT_Build_Exchg_Mtu_Rsp flags.
 */
SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(SID_STM32_SPEED_OPTIMIZED tBleStatus, ATT_Build_Exchg_Mtu_Rsp(sid_pal_ble_prv_conn_ctx_mock_t * const bearer, sid_pal_ble_prv_client_pdu_mock_t * const in_exch_mtu_req, sid_pal_ble_prv_client_pdu_mock_t * const out_exch_mtu_rsp))
{
    tBleStatus ble_status;

    SID_PAL_ASSERT(bearer != NULL);
    SID_PAL_ASSERT(in_exch_mtu_req != NULL);
    SID_PAL_ASSERT(out_exch_mtu_rsp != NULL);

    /* Use critical section since we are about to modify connection context for one of the BLE connections */
    sid_pal_enter_critical_region();

    do
    {
        /* Checks to detect if local definitions are not aligned with the BLE stack */
        SID_PAL_ASSERT(SID_STM32_BLE_PRV_ATT_EXCHANGE_MTU_REQ_SZ == in_exch_mtu_req->frame_len);
        SID_PAL_ASSERT(in_exch_mtu_req->l2cap_frame != NULL);
        SID_PAL_ASSERT(SID_STM32_BLE_PRV_ATT_EXCHANGE_MTU_REQ_OPCODE == in_exch_mtu_req->l2cap_frame->att_pdu.opcode);

        /* Locate the associated generic connection context for the connection */
        const sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_conn_ctx_for_handle(bearer->conn_handle);

        /* Apply connection-specific MTU restrictions if the context exists */
        if (conn_ctx != NULL)
        {
            /* Limit the requested MTU to the connection-specific ATT MTU size */
            if (in_exch_mtu_req->l2cap_frame->att_pdu.exchange_mtu_req.client_rx_mtu > conn_ctx->public_ctx.max_mtu_size)
            {
                in_exch_mtu_req->l2cap_frame->att_pdu.exchange_mtu_req.client_rx_mtu = conn_ctx->public_ctx.max_mtu_size;
            }
        }

        /* Call the original method to update MTU size and build Exchange MTU Reply PDU */
        ble_status = SID_STM32_FUNC_WRAPPER_CALL_ORIGINAL_FUNCTION(ATT_Build_Exchg_Mtu_Rsp(bearer, in_exch_mtu_req, out_exch_mtu_rsp));
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            break;
        }

        /* Checks to detect if local definitions are not aligned with the BLE stack */
        SID_PAL_ASSERT(SID_STM32_BLE_PRV_ATT_EXCHANGE_MTU_RSP_SZ == out_exch_mtu_rsp->frame_len);
        SID_PAL_ASSERT(out_exch_mtu_rsp->l2cap_frame != NULL);
        SID_PAL_ASSERT(SID_STM32_BLE_PRV_ATT_EXCHANGE_MTU_RSP_OPCODE == out_exch_mtu_rsp->l2cap_frame->att_pdu.opcode);

        /* Ensure the reply does not exceed the connection-specific ATT MTU size */
        if (conn_ctx != NULL)
        {
            if (out_exch_mtu_rsp->l2cap_frame->att_pdu.exchange_mtu_rsp.server_rx_mtu > conn_ctx->public_ctx.max_mtu_size)
            {
                /* Reply with connection-specific limit instead of the global limit */
                out_exch_mtu_rsp->l2cap_frame->att_pdu.exchange_mtu_rsp.server_rx_mtu = conn_ctx->public_ctx.max_mtu_size;
            }
        }

        /* Done */
        ble_status = BLE_STATUS_SUCCESS;
    } while (0);

    sid_pal_exit_critical_region();

    return ble_status;
}
#endif /* SID_STM32_BLE_ENHANCED_MTU_EXCHANGE_HANDLING */

/*----------------------------------------------------------------------------*/

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
/**
 * If GCC toolchain is used, the GATT_Add_Char_Core() function should be wrapped by linker using -Wl,--wrap=GATT_Add_Char_Core.
 */
SID_STM32_FUNC_WRAPPER_DECLARE_WRAPPED_FUNCTION(tBleStatus, GATT_Add_Char_Core(uint16_t * const out_char_handle, const gatt_add_char_core_params_t * const char_def))
{
    tBleStatus ble_status;

    /* Create a local copy of the charactersitic definition because we may need to modify it */
    gatt_add_char_core_params_t char_def_refined = *char_def;

    /* Extend GAP charactersitc with GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP event notification */
    if ((STM32_WPAN_BLE_UUID_TYPE_16 == char_def_refined.Char_UUID_Type) &&
        (  (DEVICE_NAME_UUID == char_def_refined.Char_UUID.Char_UUID_16)
        || (APPEARANCE_UUID == char_def_refined.Char_UUID.Char_UUID_16)
        || (PERIPHERAL_PREFERRED_CONN_PARAMS_UUID == char_def_refined.Char_UUID.Char_UUID_16)
        )
    )
    {
        char_def_refined.GATT_Evt_Mask |= GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
    }

    /* Call the original function */
    ble_status = SID_STM32_FUNC_WRAPPER_CALL_ORIGINAL_FUNCTION(GATT_Add_Char_Core(out_char_handle, &char_def_refined));

    return ble_status;
}
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_prv_generic_register_svc_handler(const SVC_CTL_p_EvtHandler_t gatt_svc_event_handler)
{
    sid_error_t err = SID_ERROR_GENERIC;

    SID_PAL_ASSERT(gatt_svc_event_handler != NULL);

    sid_pal_enter_critical_region();

    do
    {
        sid_pal_ble_prv_evt_hndlr_desc_t * free_slot = NULL;

        /* Find a free slot and register the handler */
        err = SID_ERROR_NONE;
        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(sid_ble_drv_ctx.svc_event_handlers); i++)
        {
            /* Locate the first available free slot */
            if ((NULL == free_slot) && (NULL == sid_ble_drv_ctx.svc_event_handlers[i].handler))
            {
                /* Found - store the free slot location and continue to check the handler is not registered already */
                free_slot = &sid_ble_drv_ctx.svc_event_handlers[i];
            }

            /* Check the handler is not registered */
            if (gatt_svc_event_handler == sid_ble_drv_ctx.svc_event_handlers[i].handler)
            {
                /* Just increment the reference counter */
                sid_ble_drv_ctx.svc_event_handlers[i].ref_count++;
                err = SID_ERROR_ALREADY_EXISTS;
                break;
            }
        }

        if (SID_ERROR_NONE == err)
        {
            if (free_slot != NULL)
            {
                free_slot->handler = gatt_svc_event_handler;
                free_slot->ref_count = 1u;
            }
            else
            {
                /* No free slots found in the registry */
                err = SID_ERROR_OUT_OF_RESOURCES;
            }
        }
        else if (SID_ERROR_ALREADY_EXISTS == err)
        {
            /* Reference counter is incremented already, report no error */
            err = SID_ERROR_NONE;
        }
        else
        {
            /* Nothing to do, report the error */
        }
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_prv_generic_deregister_svc_handler(const SVC_CTL_p_EvtHandler_t gatt_svc_event_handler)
{
    sid_error_t err = SID_ERROR_GENERIC;

    sid_pal_enter_critical_region();

    do
    {
        /* Find the handler in the local registry and free up the slot */
        err = SID_ERROR_NOT_FOUND;
        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(sid_ble_drv_ctx.svc_event_handlers); i++)
        {
            if (gatt_svc_event_handler == sid_ble_drv_ctx.svc_event_handlers[i].handler)
            {
                /* Found - decrement reference counter */
                if (sid_ble_drv_ctx.svc_event_handlers[i].ref_count > 0u)
                {
                    sid_ble_drv_ctx.svc_event_handlers[i].ref_count--;
                }

                /* Release the slot if no references left */
                if (0u == sid_ble_drv_ctx.svc_event_handlers[i].ref_count)
                {
                    sid_ble_drv_ctx.svc_event_handlers[i].handler = NULL;
                }

                err = SID_ERROR_NONE;
                break;
            }
        }
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/* Global function definitions -----------------------------------------------*/

#if SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID
__WEAK SVCCTL_UserEvtFlowStatus_t USER_SVCCTL_App_Notification(void *p_Pckt)
{
    /* When Routing co-existence mode is used - redefine this function in your user app */
    SID_PAL_LOG_WARNING("User variant of the SVCCTL_App_Notification function is not defined");
    return SVCCTL_UserEvtFlowEnable;
}
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *p_Pckt)
{
#if SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID
    if (SPBP_OPERATING_MODE_OFF == sid_ble_drv_ctx.operating_mode)
    {
        /* This BLE driver is not in user, forward BLE stack events to the user-defined handler */
        return USER_SVCCTL_App_Notification(p_Pckt);
    }
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID */

    SVCCTL_UserEvtFlowStatus_t evt_flow_status = SVCCTL_UserEvtFlowEnable;
    const hci_event_pckt *     p_event_pckt;

    p_event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) p_Pckt)->data;

    switch (p_event_pckt->evt)
    {
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
        case HCI_ENCRYPTION_CHANGE_EVT_CODE:
            {
                const hci_encryption_change_event_rp0 * const p_encryption_change_event = (hci_encryption_change_event_rp0 *)p_event_pckt->data;

                SID_BLE_GENERIC_LOG_DEBUG(">>== HCI_ENCRYPTION_CHANGE_EVT_CODE");

                if (SID_BLE_HCI_STATUS_SUCCESS == p_encryption_change_event->Status)
                {
                    sid_pal_enter_critical_region();

                    /* Locate the associated connection context */
                    sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_conn_ctx_for_handle(p_encryption_change_event->Connection_Handle);

                    if (conn_ctx != NULL)
                    {
                        conn_ctx->public_ctx.is_secure = p_encryption_change_event->Encryption_Enabled != 0x00u ? TRUE : FALSE;
                        SID_BLE_GENERIC_LOG_DEBUG("     - Connection 0x%04X encryption changed to 0x%02X", p_encryption_change_event->Connection_Handle, p_encryption_change_event->Encryption_Enabled);

                        (void)sid_stm32wba_ble_adapter_user_on_encryption_changed(p_encryption_change_event->Connection_Handle);
                    }

                    sid_pal_exit_critical_region();
                }
                else
                {
                    SID_PAL_LOG_ERROR("BLE connection 0x%04X security change failed with status 0x%02X", p_encryption_change_event->Connection_Handle, p_encryption_change_event->Status);
                    //TODO: consider terminating the connection at this point
                }
            }
            break;
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

        case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
            {
                const hci_disconnection_complete_event_rp0 * const p_disconnection_complete_event = (hci_disconnection_complete_event_rp0 *)p_event_pckt->data;

                SID_BLE_GENERIC_LOG_DEBUG(">>== HCI_DISCONNECTION_COMPLETE_EVT_CODE");
                SID_BLE_GENERIC_LOG_DEBUG("     - Connection Handle:   0x%02X", p_disconnection_complete_event->Connection_Handle);
                SID_BLE_GENERIC_LOG_DEBUG("     - Reason:    0x%02X", p_disconnection_complete_event->Reason);

                /* Manage the quantity of the bonded devices. This may generate an error if another Virtual BLE device is advertising or a connection is pending. That's normal, ignore errors at this point */
                (void)_reload_bonded_devices();

                sid_pal_enter_critical_region();

                /* Locate the associated connection context */
                sid_pal_ble_prv_connection_ctx_t * const conn_ctx = _ble_adapter_generic_get_conn_ctx_for_handle(p_disconnection_complete_event->Connection_Handle);
                
                /* Invoke Sidewalk disconnection callback if this event belongs to the Sidewalk link */
                if ((sid_ble_sidewalk_ctx.conn_ctx != NULL) && (p_disconnection_complete_event->Connection_Handle == sid_ble_sidewalk_ctx.conn_ctx->public_ctx.conn_id))
                {
                    sid_stm32wba_ble_adapter_sidewalk_on_ble_disconnected(p_disconnection_complete_event->Reason);
                }
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                else
                {
                    /* Invoke user disconnection callback */
                    (void)sid_stm32wba_ble_adapter_user_on_ble_disconnected(p_disconnection_complete_event->Connection_Handle, p_disconnection_complete_event->Reason);
                }
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

                if (conn_ctx != NULL)
                {
                    /* Release GAP cmd lock - do it before the context is invalidated */
                    _gap_cmd_resp_release(conn_ctx);

                    /* Invalidate the associated generic connection context */
                    _ble_adapter_generic_invalidate_conn_ctx(conn_ctx);
                }
                sid_pal_exit_critical_region();
            }
            break;

#ifdef DEBUG
        case HCI_HARDWARE_ERROR_EVT_CODE:
            {
                const hci_hardware_error_event_rp0 * const p_hardware_error_event = (hci_hardware_error_event_rp0 *)p_event_pckt->data;

                SID_PAL_LOG_ERROR("BLE HCI hardware error 0x%02X", p_hardware_error_event->Hardware_Code);
            }
            break;
#endif /* DEBUG */

        case HCI_LE_META_EVT_CODE:
            evt_flow_status = _ble_adapter_generic_process_hci_le_meta_evt((evt_le_meta_event *)p_event_pckt->data);
            break;

        case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
            evt_flow_status = _ble_adapter_generic_process_aci_evt((evt_blecore_aci *)p_event_pckt->data);
            break;

        default:
            /* Allow event flow to continue by default */
            break;
    }

    return evt_flow_status;
}

/*----------------------------------------------------------------------------*/

#if SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID
__WEAK uint8_t USER_BLECB_Indication(const uint8_t* data, uint16_t length, const uint8_t* ext_data, uint16_t ext_length)
{
    /* When Routing co-existence mode is used - redefine this function in your user app */
    SID_PAL_LOG_WARNING("User variant of the BLECB_Indication function is not defined");
    return 0u;
}
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint8_t BLECB_Indication(const uint8_t * data, uint16_t length, const uint8_t * ext_data, uint16_t ext_length)
{
#if SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID
    if (SPBP_OPERATING_MODE_OFF == sid_ble_drv_ctx.operating_mode)
    {
        /* This BLE driver is not in user, forward BLE stack events to the user-defined handler */
        return USER_BLECB_Indication(data, length, ext_data, ext_length);
    }
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID */

    uint8_t              status = BLE_STATUS_FAILED;
    AMM_Function_Error_t amm_error;
    BleEvtPacket_t *     phcievt;
    const uint32_t       total_length = ((uint32_t)length + (uint32_t)ext_length);

    (void)ext_data;

    switch (data[0])
    {
        case HCI_EVENT_PKT_TYPE:
            {
                const uint32_t alloc_size = DIVC((sizeof(BleEvtPacketHeader_t) + total_length), sizeof (uint32_t));
                
                ble_amm_cb.Callback = _ble_adapter_generic_resume_flow_process;
                amm_error = AMM_Alloc(CFG_AMM_VIRTUAL_APP_BLE, alloc_size, (uint32_t **)(void *)&phcievt, &ble_amm_cb);
                if (amm_error != AMM_ERROR_OK)
                {
                    SID_PAL_LOG_ERROR("AMM failed to allocate %u bytes to store BLE event. Error %d", alloc_size, (int32_t)amm_error);
                    status = BLE_STATUS_FAILED;
                    break;
                }
                SID_PAL_ASSERT(phcievt != NULL); /* Should never happen if AMM_ERROR_OK is returned, but just in case... */

                phcievt->evtserial.type        = HCI_EVENT_PKT_TYPE;
                phcievt->evtserial.evt.evtcode = data[1];
                phcievt->evtserial.evt.plen    = data[2];
                SID_STM32_UTIL_fast_memcpy((void*)&phcievt->evtserial.evt.payload, &data[3], phcievt->evtserial.evt.plen);

                LST_insert_tail(&BleAsynchEventQueue, (tListNode *)phcievt);
                osSemaphoreRelease(hci_async_evt_semaphore);
                status = BLE_STATUS_SUCCESS;
            }
            break;

        case HCI_ACLDATA_PKT_TYPE:
            status = BLE_STATUS_SUCCESS;
            break;

        default:
            status = BLE_STATUS_FAILED;
            break;
    }

    return status;
}

/*----------------------------------------------------------------------------*/

#if SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID
__WEAK void USER_APP_BLE_HostNvmStore(void)
{
    /* When Routing co-existence mode is used - redefine this function in your user app */
    SID_PAL_LOG_WARNING("User variant of the APP_BLE_HostNvmStore function is not defined");
}
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID */

/*----------------------------------------------------------------------------*/

void APP_BLE_HostNvmStore(void)
{
#if SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID
    if (SPBP_OPERATING_MODE_OFF == sid_ble_drv_ctx.operating_mode)
    {
        /* This BLE driver is not in user, forward BLE stack events to the user-defined handler */
        USER_APP_BLE_HostNvmStore();
        return;
    }
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID */

    SNVMA_Cmd_Status_t snvma_status;

    /* Call SNVMA for storing */
    SID_BLE_GENERIC_LOG_DEBUG("BLE NVM Store");
    snvma_status = SNVMA_Write(APP_BLE_NvmBuffer, BLE_NvmCallback);
    if (snvma_status != SNVMA_ERROR_OK)
    {
        SID_PAL_LOG_ERROR("BLE NVMA Store failed with error 0x%02X", snvma_status);
    }
}
