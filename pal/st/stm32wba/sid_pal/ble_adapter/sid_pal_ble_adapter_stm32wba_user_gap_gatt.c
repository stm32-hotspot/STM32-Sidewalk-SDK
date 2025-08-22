/**
  ******************************************************************************
  * @file    sid_pal_ble_adapter_stm32wba_user_gap_gatt.c
  * @brief   User GAP and GATT (not related to Sidewalk operation) handling
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024-2025 STMicroelectronics.
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

#include "sid_pal_ble_adapter_stm32wba.h"
#include "sid_pal_ble_adapter_stm32wba_config.h"
#include "sid_pal_ble_adapter_stm32wba_ext_ifc.h"
#include "sid_pal_ble_adapter_stm32wba_private_defs.h"
#include "sid_pal_ble_adapter_stm32wba_user_gap_gatt.h"
#include "sid_pal_ble_adapter_stm32wba_utils.h"

/* Sidewalk interfaces */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_storage_kv_ifc.h>

/* BLE stack */
#include <ble_core.h>
#include <ble_types.h>
#include <ll_sys_if.h>

/* Utils */
#include "stm32_rtos.h"
#include <flash_driver.h>
#include <flash_manager.h>
#include <sid_stm32_common_utils.h>
#include <stm_list.h>

/* Private defines -----------------------------------------------------------*/

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_NONE) || (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID)
#  error "User-defined BLE profiles are not supported by the current config. Please exclude this file from build"
#endif

/* Private macro -------------------------------------------------------------*/

#ifndef SID_BLE_USER_EXTRA_LOGGING
/* Set SID_BLE_USER_EXTRA_LOGGING to 1 to enable extended logs */
#  define SID_BLE_USER_EXTRA_LOGGING (0)
#endif

#if SID_BLE_USER_EXTRA_LOGGING
#  define SID_BLE_USER_LOG_ERROR(...)         SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SID_BLE_USER_LOG_WARNING(...)       SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SID_BLE_USER_LOG_INFO(...)          SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SID_BLE_USER_LOG_DEBUG(...)         SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SID_BLE_USER_LOG_TRACE(...)         SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SID_BLE_USER_LOG_ERROR(...)         ((void)0u)
#  define SID_BLE_USER_LOG_WARNING(...)       ((void)0u)
#  define SID_BLE_USER_LOG_INFO(...)          ((void)0u)
#  define SID_BLE_USER_LOG_DEBUG(...)         ((void)0u)
#  define SID_BLE_USER_LOG_TRACE(...)         ((void)0u)
#endif

#ifndef containerof
#  define containerof(ptr, type, member)      ((type *)((uintptr_t)(ptr) - offsetof(type, member)))
#endif

/* Private typedef -----------------------------------------------------------*/

typedef struct {
    tListNode node;
    sid_ble_ext_virtual_device_ctx_t * ctx;
} sid_ble_ext_virtual_device_node_t;
static_assert(offsetof(sid_ble_ext_virtual_device_node_t, node) == 0u);

/* Private variables ---------------------------------------------------------*/

static sid_pal_ble_user_mode_ctx_t       sid_ble_user_mode_ctx = {0};
static sid_ble_ext_virtual_device_node_t active_virt_devs_list;

/* Flash access management */
static osSemaphoreId_t     ble_adapter_user_fm_semaphore = NULL;
static FM_CallbackNode_t   ble_adapter_user_fm_callback_node;
static FM_FlashOp_Status_t ble_adapter_user_last_fm_status;

/* Private constants ---------------------------------------------------------*/

/**
 * @brief Generic definition of a CCCD
 */
static const sid_ble_ext_char_desc_t cccd_def = {
    .desc_name         = "CCCD",
    .uuid = {
        .type          = UUID_TYPE_16,
        .uu            = {0x29u, 0x02u},
    },
    .max_length        = SID_STM32_BLE_CHARACTERISTIC_CCCD_ATTRIBUTE_SIZE,
    .access = {
        .read          = TRUE,
        .write_wo_resp = TRUE,
    },
};

static const osSemaphoreAttr_t ble_adapter_user_fm_semaphore_attributes = {
    .name      = "User BLE FM Semaphore",
    .attr_bits = SEMAPHORE_DEFAULT_ATTR_BITS,
    .cb_mem    = SEMAPHORE_DEFAULT_CB_MEM,
    .cb_size   = SEMAPHORE_DEFAULT_CB_SIZE,
};

/* Private function prototypes -----------------------------------------------*/

static        void                                 _ble_adapter_user_fm_callback(FM_FlashOp_Status_t Status);
static inline uint32_t                             _ble_adapter_user_acquire_ll_lock(const char * const error_msg);
static inline sid_error_t                          _ble_adapter_user_release_ll_lock(const uint32_t ll_lock_acquired ,const char * const error_msg);
static inline uint32_t                             _ble_adapter_user_is_desc_cccd(const sid_ble_ext_gatt_server_char_desc_ctx_t * const desc_ctx);
static inline uint16_t                             _ble_adapter_user_get_storage_kv_bond_pair_key(const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint16_t slot_id);
static        sid_error_t                          _ble_adapter_user_search_bond_pair_kv_record(const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint8_t peer_identity_addr_type, const uint8_t peer_identity[SID_STM32_BLE_ADDRESS_LENGTH], uint16_t * const out_slot_id);
static inline uint32_t                             _ble_adapter_user_are_uuids_equal(const sid_ble_cfg_uuid_info_t * const uuid_a, const sid_ble_cfg_uuid_info_t * const uuid_b);
static inline const sid_ble_ext_virtual_device_t * _ble_adapter_user_search_device_definition(const sid_ble_ext_virtual_device_id_t virt_dev_id);
static inline sid_ble_ext_virtual_device_node_t *  _ble_adapter_user_search_device_node(const sid_ble_ext_virtual_device_id_t virt_dev_id);
#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0u)
static inline sid_ble_ext_virtual_device_node_t *  _ble_adapter_user_search_first_node_by_type(const sid_ble_ext_virtual_device_role_t device_type);
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */
static inline sid_ble_ext_virtual_device_node_t *  _ble_adapter_user_search_device_node_by_conn_handle(const uint16_t conn_handle);
static inline sid_ble_ext_conn_list_node_t *       _ble_adapter_user_search_connection_node_by_conn_handle(const sid_ble_ext_virtual_device_ctx_t * const virt_dev_ctx, const uint16_t conn_handle);
static inline void                                 _ble_adapter_user_search_att_ctx_by_handle(sid_ble_ext_gatt_server_svc_ctx_t ** const out_svc_ctx, sid_ble_ext_gatt_server_char_ctx_t ** out_char_ctx, sid_ble_ext_gatt_server_char_desc_ctx_t ** const out_desc_ctx,
                                                                                              const sid_ble_ext_virtual_device_ctx_t * const virt_dev_ctx, const uint16_t att_handle);
static inline void                                 _ble_adapter_user_search_char_ctx_by_uuid(const sid_ble_ext_virtual_device_ctx_t * const virt_dev_ctx, const sid_ble_cfg_uuid_info_t * const char_uuid, sid_ble_ext_gatt_server_svc_ctx_t ** const out_svc_ctx, sid_ble_ext_gatt_server_char_ctx_t ** out_char_ctx);
static        uint32_t                             _ble_adapter_user_check_device_requires_pairing(const sid_ble_ext_virtual_device_t * const device_cfg);
static inline sid_error_t                          _ble_adapter_user_set_adv_params(const sid_ble_ext_virtual_device_ctx_t * const device_ctx, const uint32_t adv_interval_min, const uint32_t adv_interval_max);
static        sid_error_t                          _ble_adapter_user_init_advertising_settings(sid_ble_ext_virtual_device_ctx_t * const node_ctx, const sid_ble_ext_adv_param_t * const adv_param);
static        sid_error_t                          _ble_adapter_user_init_peripheral_gatt_char_descriptor(sid_ble_ext_gatt_server_char_desc_ctx_t * const out_desc_ctx, const uint16_t service_handle, const uint16_t char_handle, const sid_ble_ext_char_desc_t * const desc_def, const uint16_t max_att_mtu);
static        sid_error_t                          _ble_adapter_user_init_peripheral_gatt_characteristic(sid_ble_ext_gatt_server_char_ctx_t * const out_char_ctx, const uint16_t service_handle, const sid_ble_ext_char_def_t * const char_def, const uint16_t max_att_mtu);
static        sid_error_t                          _ble_adapter_user_init_peripheral_gatt_service(sid_ble_ext_gatt_server_svc_ctx_t * const svc_ctxs, const sid_ble_ext_gatt_svc_def_t * const svc_definition, const uint16_t max_att_mtu);
static        sid_error_t                          _ble_adapter_user_init_peripheral_gatt_services(sid_ble_ext_virtual_device_ctx_t * const node_ctx, const sid_ble_ext_gatt_profile_def_t * const gatt_profile);
static        sid_error_t                          _ble_adapter_user_init_peripheral(sid_ble_ext_virtual_device_node_t * const node);
static        sid_error_t                          _ble_adapter_user_init_broadcaster(sid_ble_ext_virtual_device_node_t * const node);
static        sid_error_t                          _ble_adapter_user_init_central(sid_ble_ext_virtual_device_node_t * const node);
static        sid_error_t                          _ble_adapter_user_init_observer(sid_ble_ext_virtual_device_node_t * const node);
static        sid_error_t                          _ble_adapter_user_deinit_peripheral(sid_ble_ext_virtual_device_node_t * const node);
static        sid_error_t                          _ble_adapter_user_deinit_broadcaster(sid_ble_ext_virtual_device_node_t * const node);
static        sid_error_t                          _ble_adapter_user_deinit_central(sid_ble_ext_virtual_device_node_t * const node);
static        sid_error_t                          _ble_adapter_user_deinit_observer(sid_ble_ext_virtual_device_node_t * const node);
static        sid_error_t                          _ble_adapter_user_terminate_device_node(sid_ble_ext_virtual_device_node_t * const node);
static        sid_error_t                          _ble_adapter_user_set_node_adv_data(sid_ble_ext_virtual_device_node_t * const node, const uint8_t * const raw_data, const uint32_t raw_data_length);
static        sid_error_t                          _ble_adapter_user_set_node_scan_resp_data(sid_ble_ext_virtual_device_node_t * const node, const uint8_t * const raw_data, const uint32_t raw_data_length);
static        sid_error_t                          _ble_adapter_user_node_start_advertisement(sid_ble_ext_virtual_device_node_t * const node, const sid_ble_ext_adv_param_t * const adv_param);
static        sid_error_t                          _ble_adapter_user_node_stop_advertisement(sid_ble_ext_virtual_device_node_t * const node);
static        sid_error_t                          _ble_adapter_user_node_disconnect(sid_ble_ext_virtual_device_node_t * const node, sid_ble_ext_conn_list_node_t * const conn_node);

static        SVCCTL_EvtAckStatus_t                _ble_adapter_user_gatt_event_handler(void * p_Event);
static inline SVCCTL_EvtAckStatus_t                _ble_adapter_user_process_gatt_attr_rw_permit_req(const uint16_t conn_handle, const uint16_t att_handle, uint8_t * const out_att_status_code, const sid_ble_ext_att_access_type_t access_mode, const uint32_t full_data_length,
                                                                                                     sid_ble_ext_virtual_device_node_t ** const out_dev_node ,sid_ble_ext_conn_list_node_t ** const out_conn_node,
                                                                                                     sid_ble_ext_gatt_server_svc_ctx_t ** const out_svc_ctx, sid_ble_ext_gatt_server_char_ctx_t ** const out_char_ctx, sid_ble_ext_gatt_server_char_desc_ctx_t ** const out_desc_ctx);
static inline SVCCTL_EvtAckStatus_t                _ble_adapter_user_process_gatt_attr_modified_evt(const aci_gatt_attribute_modified_event_rp0 * const evt);
#if SID_STM32_BLE_DLE_ENABLE
static inline SVCCTL_EvtAckStatus_t                _ble_adapter_user_process_hci_le_meta_evt(const evt_le_meta_event * const evt);
#endif /* SID_STM32_BLE_DLE_ENABLE */

static        sid_error_t                          _ble_adapter_user_init(void);
static        sid_error_t                          _ble_adapter_user_deinit(void);
static        sid_error_t                          _ble_adapter_user_factory_reset(void);
static        sid_error_t                          _ble_adapter_user_virt_dev_activate(const sid_ble_ext_virtual_device_id_t virt_dev_id, sid_ble_ext_virtual_device_ctx_t * * const out_ctx);
static        sid_error_t                          _ble_adapter_user_virt_dev_terminate(const sid_ble_ext_virtual_device_id_t virt_dev_id);
static        sid_error_t                          _ble_adapter_user_virt_dev_set_adv_data(const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint8_t * const adv_data, const uint32_t adv_data_length);
static        sid_error_t                          _ble_adapter_user_virt_dev_set_scan_resp_data(const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint8_t * const scan_resp_data, const uint32_t scan_resp_data_length);
static        sid_error_t                          _ble_adapter_user_virt_dev_adv_start(const sid_ble_ext_virtual_device_id_t virt_dev_id);
static        sid_error_t                          _ble_adapter_user_virt_dev_adv_stop(const sid_ble_ext_virtual_device_id_t virt_dev_id);
static        sid_error_t                          _ble_adapter_user_virt_dev_get_bonded_peers(const sid_ble_ext_virtual_device_id_t virt_dev_id, sid_pal_ble_ext_peer_identity_addr_t * const out_bonded_peers_list, uint32_t * const out_bonded_peers_count, const uint32_t bonded_peers_list_size_limit);
static        sid_error_t                          _ble_adapter_user_virt_dev_remove_bond(const sid_ble_ext_virtual_device_id_t virt_dev_id, const sid_pal_ble_ext_peer_identity_addr_t * const peer_identity);
static        sid_error_t                          _ble_adapter_user_virt_dev_disconnect(const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint16_t conn_id);
static        sid_error_t                          _ble_adapter_user_virt_dev_disconnect_all(const sid_ble_ext_virtual_device_id_t virt_dev_id);
static        sid_error_t                          _ble_adapter_user_virt_dev_suspend(const sid_ble_ext_virtual_device_id_t virt_dev_id);
static        sid_error_t                          _ble_adapter_user_virt_dev_resume(const sid_ble_ext_virtual_device_id_t virt_dev_id);
static        sid_error_t                          _ble_adapter_user_virt_dev_update_char(const sid_ble_ext_virtual_device_id_t virt_dev_id, const sid_ble_cfg_uuid_info_t * const char_uuid, const uint8_t * const data, const uint32_t data_length, uint8_t notify);

/* Private constants ---------------------------------------------------------*/

#if (SID_STM32_BLE_COEXISTENCE_MODE != SID_STM32_BLE_COEXISTENCE_MODE_NONE) && (SID_STM32_BLE_COEXISTENCE_MODE != SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID)
static const sid_pal_ble_adapter_extended_interface_t user_ble_ifc = {
    /* Global Initialization and Maintenance */
    .init                           = _ble_adapter_user_init,
    .deinit                         = _ble_adapter_user_deinit,
    .factory_reset                  = _ble_adapter_user_factory_reset,

    /* Per-Device Initialization and Maintenance */
    .virtual_dev_activate           = _ble_adapter_user_virt_dev_activate,
    .virtual_dev_terminate          = _ble_adapter_user_virt_dev_terminate,
    .virtual_dev_suspend            = _ble_adapter_user_virt_dev_suspend,
    .virtual_dev_resume             = _ble_adapter_user_virt_dev_resume,

    /* BLE Advertising Management (for Peripherals and Broadcasters) */
    .virtual_dev_set_adv_data       = _ble_adapter_user_virt_dev_set_adv_data,
    .virtual_dev_set_scan_resp_data = _ble_adapter_user_virt_dev_set_scan_resp_data,
    .virtual_dev_adv_start          = _ble_adapter_user_virt_dev_adv_start,
    .virtual_dev_adv_stop           = _ble_adapter_user_virt_dev_adv_stop,

    /* BLE Per-Device Connection Management */
    .virtual_dev_get_bonded_peers   = _ble_adapter_user_virt_dev_get_bonded_peers,
    .virtual_dev_remove_bond        = _ble_adapter_user_virt_dev_remove_bond,
    .virtual_dev_disconnect         = _ble_adapter_user_virt_dev_disconnect,
    .virtual_dev_disconnect_all     = _ble_adapter_user_virt_dev_disconnect_all,

    /* BLE Peripheral Management */
    .virtual_dev_update_char        = _ble_adapter_user_virt_dev_update_char,
};
#endif /* SID_STM32_BLE_COEXISTENCE_MODE */

/* Imported variables --------------------------------------------------------*/

extern sid_pal_ble_adapter_ctx_t sid_ble_drv_ctx;

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _ble_adapter_user_fm_callback(FM_FlashOp_Status_t Status)
{
	ble_adapter_user_last_fm_status = Status;
    SID_PAL_ASSERT(ble_adapter_user_fm_semaphore != NULL);
    (void)osSemaphoreRelease(ble_adapter_user_fm_semaphore);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _ble_adapter_user_acquire_ll_lock(const char * const error_msg)
{
    osStatus_t os_status;
    uint32_t   ll_lock_acquired;

    if (LinkLayerMutex != NULL)
    {
        /* Wait for any potentially ongoing BLE operations to stop */
        os_status = osMutexAcquire(LinkLayerMutex, SID_STM32_BLE_LL_MUTEX_WAIT_TIMEOUT_ON_INIT_TICKS);

        /* Print out warning but proceed */
        if (os_status != osOK)
        {
            SID_PAL_LOG_WARNING(error_msg);
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

    return ll_lock_acquired;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_error_t _ble_adapter_user_release_ll_lock(const uint32_t ll_lock_acquired ,const char * const error_msg)
{
    sid_error_t err;
    osStatus_t os_status;

    if ((ll_lock_acquired != FALSE) && (LinkLayerMutex != NULL))
    {
        os_status = osMutexRelease(LinkLayerMutex);
        if (os_status != osOK)
        {
            SID_PAL_LOG_ERROR(error_msg);
            err = SID_ERROR_UNRECOVERABLE;
        }
        else
        {
            err = SID_ERROR_NONE;
        }
    }
    else
    {
        err = SID_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _ble_adapter_user_is_desc_cccd(const sid_ble_ext_gatt_server_char_desc_ctx_t * const desc_ctx)
{
    uint32_t ret;

    /* Simple check if the desciptor is CCCD - compare definition address to the generic definition address. Since all CCCD contexts are pointing to cccd_def this can be a reliable way to identify CCCD */
    if (desc_ctx->desc_def == &cccd_def)
    {
        ret = TRUE;
    }
    else
    {
        ret = FALSE;
    }

    return ret;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint16_t _ble_adapter_user_get_storage_kv_bond_pair_key(const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint16_t slot_id)
{
    uint16_t kv_key_value = 0u;
    const uint16_t slot_id_mask = (((uint16_t)1u << STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_KEY_VIRT_DEV_ID_OFFSET) - (uint16_t)1u);

    kv_key_value = (uint16_t)(((uint16_t)virt_dev_id << STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_KEY_VIRT_DEV_ID_OFFSET) | (slot_id & slot_id_mask));

    return kv_key_value;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_search_bond_pair_kv_record(const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint8_t peer_identity_addr_type, const uint8_t peer_identity[SID_STM32_BLE_ADDRESS_LENGTH], uint16_t * const out_slot_id)
{
    sid_error_t err = SID_ERROR_NOT_FOUND;
    sid_error_t kv_err = SID_ERROR_NONE;

    for (uint32_t i = 0u; i < SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES; i++)
    {
        sid_pal_ble_ext_peer_identity_addr_t kv_bond_pair;
        const uint16_t bond_pair_kv_key = _ble_adapter_user_get_storage_kv_bond_pair_key(virt_dev_id, (uint16_t)i);

        kv_err = sid_pal_storage_kv_record_get(STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP, bond_pair_kv_key, &kv_bond_pair, sizeof(kv_bond_pair));
        if (SID_ERROR_NONE == kv_err)
        {
            /* Found a valid record - check if it describes the current bond pair */
            if ((kv_bond_pair.identity_address_type == peer_identity_addr_type)
                      && (SID_STM32_UTIL_fast_memcmp(peer_identity, kv_bond_pair.identity_address, sizeof(kv_bond_pair.identity_address)) == 0u))
            {
                /* Found the matching identity record - connection is allowed */
                err = SID_ERROR_NONE;
                if (out_slot_id != NULL)
                {
                    *out_slot_id = (uint16_t)i;
                }
                break;
            }
            else
            {
                /* Non-matching record - proceed to the next one */
                continue;
            }
        }
        else if (SID_ERROR_NOT_FOUND == kv_err)
        {
            /* Record does not exists, proceed to the next slot */
            continue;
        }
        else
        {
            /* Generic KV storage failure - terminate */
            err = kv_err;
            break;
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _ble_adapter_user_are_uuids_equal(const sid_ble_cfg_uuid_info_t * const uuid_a, const sid_ble_cfg_uuid_info_t * const uuid_b)
{
    uint32_t ret;

    do
    {
        uint32_t compare_len;

        if ((NULL == uuid_a) || (NULL == uuid_b))
        {
            /* Can't compare */
            ret = FALSE;
            break;
        }

        if (uuid_a->type != uuid_b->type)
        {
            /* UUIDs are not equal because they have different length */
            ret = FALSE;
            break;
        }

        switch (uuid_a->type)
        {
            case UUID_TYPE_16:
                compare_len = 2u;
                break;

            case UUID_TYPE_32:
                compare_len = 4u;
                break;

            case UUID_TYPE_128:
            default:
                compare_len = 16u;
                break;
        }

        ret = (SID_STM32_UTIL_fast_memcmp(uuid_a->uu, uuid_b->uu, compare_len) == 0u) ? TRUE : FALSE;
    } while (0);

    return ret;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline const sid_ble_ext_virtual_device_t * _ble_adapter_user_search_device_definition(const sid_ble_ext_virtual_device_id_t virt_dev_id)
{
    const sid_ble_ext_virtual_device_t * dev_definition = NULL;

    if (NULL == sid_ble_drv_ctx.cfg)
    {
        return NULL;
    }

    for (uint32_t i = 0u; i < sid_ble_drv_ctx.cfg->custom_profiles.num_virtual_devices; i++)
    {
        const sid_ble_ext_virtual_device_t * const dev = &sid_ble_drv_ctx.cfg->custom_profiles.virtual_devices[i];
        if (virt_dev_id == dev->device_id)
        {
            dev_definition = dev;
            break;
        }
    }

    return dev_definition;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_ble_ext_virtual_device_node_t * _ble_adapter_user_search_device_node(const sid_ble_ext_virtual_device_id_t virt_dev_id)
{
    sid_ble_ext_virtual_device_node_t * dev_node = NULL;

    sid_pal_enter_critical_region();

    sid_ble_ext_virtual_device_node_t * current_node = (sid_ble_ext_virtual_device_node_t *)(((tListNode *)&active_virt_devs_list)->next);

    while (current_node != &active_virt_devs_list)
    {
        if (virt_dev_id == current_node->ctx->device_cfg->device_id)
        {
            /* Found existing device node */
            dev_node = current_node;
            break;
        }

        /* Advance to the next node */
        LST_get_next_node((tListNode *)current_node, (tListNode **)&current_node);
    }

    sid_pal_exit_critical_region();

    return dev_node;
}

/*----------------------------------------------------------------------------*/

#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0u)
SID_STM32_SPEED_OPTIMIZED static inline sid_ble_ext_virtual_device_node_t * _ble_adapter_user_search_first_node_by_type(const sid_ble_ext_virtual_device_role_t device_type)
{
    sid_ble_ext_virtual_device_node_t * dev_node = NULL;

    sid_pal_enter_critical_region();

    sid_ble_ext_virtual_device_node_t * current_node = (sid_ble_ext_virtual_device_node_t *)(((tListNode *)&active_virt_devs_list)->next);

    while (current_node != &active_virt_devs_list)
    {
        if ((current_node->ctx->device_cfg->device_type & device_type) != 0u)
        {
            /* Found existing device node */
            dev_node = current_node;
            break;
        }

        /* Advance to the next node */
        LST_get_next_node((tListNode *)current_node, (tListNode **)&current_node);
    }

    sid_pal_exit_critical_region();

    return dev_node;
}
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_ble_ext_virtual_device_node_t * _ble_adapter_user_search_device_node_by_conn_handle(const uint16_t conn_handle)
{
    sid_ble_ext_virtual_device_node_t * dev_node = NULL;

    sid_pal_enter_critical_region();

    sid_ble_ext_virtual_device_node_t * current_node = (sid_ble_ext_virtual_device_node_t *)(((tListNode *)&active_virt_devs_list)->next);

    /* Iterate over all virtual device node */
    while (current_node != &active_virt_devs_list)
    {
        /* This search is valid for connectable devices only */
        if ((SBEVDR_BLE_PERIPHERAL == current_node->ctx->device_cfg->device_type) || (SBEVDR_BLE_CENTRAL == current_node->ctx->device_cfg->device_type))
        {
            sid_ble_ext_conn_list_node_t * current_conn_node = (sid_ble_ext_conn_list_node_t *)(((tListNode *)&current_node->ctx->conn_list)->next);

            /* For each device node iterate over the list of connections */
            while (current_conn_node != &current_node->ctx->conn_list)
            {
                if (conn_handle == current_conn_node->ctx->conn_id)
                {
                    /* Found the associated device node for this connection */
                    dev_node = current_node;
                    break;
                }

                /* Advance to the next connection node */
                LST_get_next_node((tListNode *)current_conn_node, (tListNode **)&current_conn_node);
            }

            /* Jump out of main loop if the device node was found */
            if (dev_node != NULL)
            {
                break;
            }
        }

        /* Advance to the next node */
        LST_get_next_node((tListNode *)current_node, (tListNode **)&current_node);
    }

    sid_pal_exit_critical_region();

    return dev_node;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_ble_ext_conn_list_node_t * _ble_adapter_user_search_connection_node_by_conn_handle(const sid_ble_ext_virtual_device_ctx_t * const virt_dev_ctx, const uint16_t conn_handle)
{
    sid_ble_ext_conn_list_node_t * conn_node = NULL;

    sid_pal_enter_critical_region();

    sid_ble_ext_conn_list_node_t * current_conn_node = (sid_ble_ext_conn_list_node_t *)(((tListNode *)&virt_dev_ctx->conn_list)->next);
    while (current_conn_node != &virt_dev_ctx->conn_list)
    {
        if (conn_handle == current_conn_node->ctx->conn_id)
        {
            /* Found the associated connection node */
            conn_node = current_conn_node;
            break;
        }

        /* Advance to the next node */
        LST_get_next_node((tListNode *)current_conn_node, (tListNode **)&current_conn_node);
    }

    sid_pal_exit_critical_region();

    return conn_node;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Search for the attribute-related contexts by attribute handle
 *
 * @note This method is valid only for BLE peripheral devices. Search is performed in the context of the virtual peripheral node
 *
 * @param [out] out_svc_ctx  Context of the GATT service that is associated with the attribute. This pointer is populated whenever the attribute in question turns to belong to one of the characteristics or characteristic descriptors of the service
 * @param [out] out_char_ctx Context of the GATT characteristic that is associated with the attribute. This pointer is set whenever the attribute is the value of this characteristic or belongs to the one of the characteristic descriptors within this characteristic
 * @param [out] out_dec_ctx  Context of the GATT characteristic descriptor that is associated with the attribute. Can be NULL if the attribute corresponds to the value of characteristic itself
 */
SID_STM32_SPEED_OPTIMIZED static inline void _ble_adapter_user_search_att_ctx_by_handle(sid_ble_ext_gatt_server_svc_ctx_t ** const out_svc_ctx, sid_ble_ext_gatt_server_char_ctx_t ** out_char_ctx, sid_ble_ext_gatt_server_char_desc_ctx_t ** const out_desc_ctx,
                                                                                        const sid_ble_ext_virtual_device_ctx_t * const virt_dev_ctx, const uint16_t att_handle)
{
    *out_svc_ctx = NULL;
    *out_char_ctx = NULL;
    *out_desc_ctx = NULL;

    sid_pal_enter_critical_region();
    do
    {
        /* This method is valid for BLE peripherals only */
        if (virt_dev_ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL)
        {
            break;
        }

        /* Go through the list of services */
        for (uint32_t i = 0u; ((i < virt_dev_ctx->periph.svc_ctxs_count) && (NULL == *out_svc_ctx)); i++)
        {
            sid_ble_ext_gatt_server_svc_ctx_t * const current_svc_ctx = &virt_dev_ctx->periph.svc_ctxs[i];

            /* Move to the next service if this one has no characteristics */
            if ((NULL == current_svc_ctx->char_ctxs) || (0u == current_svc_ctx->char_ctxs_count))
            {
                continue;
            }

            /* Iterate over the list of characteristics */
            for (uint32_t j = 0u; ((j < current_svc_ctx->char_ctxs_count) && (NULL == *out_char_ctx)); j++)
            {
                sid_ble_ext_gatt_server_char_ctx_t * const current_char_ctx = &current_svc_ctx->char_ctxs[j];
                if (att_handle == (current_char_ctx->handle + SID_STM32_BLE_CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
                {
                    /* The attribute in question corresponds to the value attribute of the current characteristic. Search is finished */
                    *out_svc_ctx  = current_svc_ctx;
                    *out_char_ctx = current_char_ctx;
                    break;
                }

                /* Move to the next characteristic if this one has no descriptors */
                if ((NULL == current_char_ctx->desc_ctxs) || (0u == current_char_ctx->desc_ctxs_count))
                {
                    continue;
                }

                /* Iterate over the list of characteristic descriptors */
                for (uint32_t k = 0u; k < current_char_ctx->desc_ctxs_count; k++)
                {
                    sid_ble_ext_gatt_server_char_desc_ctx_t * const current_desc_ctx = &current_char_ctx->desc_ctxs[k];
                    if (att_handle == current_desc_ctx->handle)
                    {
                        /* The attribute in question corresponds to one of the characteristic descriptors of the current characteristic. Search is finished */
                        *out_svc_ctx  = current_svc_ctx;
                        *out_char_ctx = current_char_ctx;
                        *out_desc_ctx  = current_desc_ctx;
                        break;
                    }
                }
            }
        }
    } while (0);
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Search for the service and characteristic contexts by characteristic UUID
 *
 * @note This method is valid only for BLE peripheral devices. Search is performed in the context of the virtual peripheral node
 *
 * @param [in] virt_dev_ctx Context of the virtual BLE device for which the search is performed
 * @param [in] char_uuid Characteristic UUID to look for
 * @param [out] out_svc_ctx  Context of the GATT service that is associated with the characteristic
 * @param [out] out_char_ctx Context of the GATT characteristic (if found)
 */
SID_STM32_SPEED_OPTIMIZED static inline void _ble_adapter_user_search_char_ctx_by_uuid(const sid_ble_ext_virtual_device_ctx_t * const virt_dev_ctx, const sid_ble_cfg_uuid_info_t * const char_uuid, sid_ble_ext_gatt_server_svc_ctx_t ** const out_svc_ctx, sid_ble_ext_gatt_server_char_ctx_t ** out_char_ctx)
{
    *out_svc_ctx = NULL;
    *out_char_ctx = NULL;

    sid_pal_enter_critical_region();
    do
    {
        /* This method is valid for BLE peripherals only */
        if (virt_dev_ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL)
        {
            break;
        }

        /* Go through the list of services */
        for (uint32_t i = 0u; ((i < virt_dev_ctx->periph.svc_ctxs_count) && (NULL == *out_svc_ctx)); i++)
        {
            sid_ble_ext_gatt_server_svc_ctx_t * const current_svc_ctx = &virt_dev_ctx->periph.svc_ctxs[i];

            /* Move to the next service if this one has no characteristics */
            if ((NULL == current_svc_ctx->char_ctxs) || (0u == current_svc_ctx->char_ctxs_count))
            {
                continue;
            }

            /* Iterate over the list of characteristics */
            for (uint32_t j = 0u; ((j < current_svc_ctx->char_ctxs_count) && (NULL == *out_char_ctx)); j++)
            {
                sid_ble_ext_gatt_server_char_ctx_t * const current_char_ctx = &current_svc_ctx->char_ctxs[j];

                if (_ble_adapter_user_are_uuids_equal(&current_char_ctx->char_def->uuid, char_uuid) != FALSE)
                {
                    /* Found the characteristic */
                    *out_svc_ctx = current_svc_ctx;
                    *out_char_ctx = current_char_ctx;
                    break;
                }
            }
        }
    } while (0);
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static uint32_t _ble_adapter_user_check_device_requires_pairing(const sid_ble_ext_virtual_device_t * const device_cfg)
{
    uint32_t needs_pairing = FALSE;

    do
    {
        /* Non-connectable devices do not need pairing by definition */
        if ((SBEVDR_BLE_BROADCASTER == device_cfg->device_type) || (SBEVDR_BLE_OBSERVER == device_cfg->device_type))
        {
            needs_pairing = FALSE;
            break;
        }

        if (SBEVDR_BLE_PERIPHERAL == device_cfg->device_type)
        {
            /* Iterate over the GATT profile and check if there's any characteristic that requires pairing for access */
            const sid_ble_ext_gatt_profile_def_t * const gatt_profile = &device_cfg->peripheral_cfg.gatt_profile;
            needs_pairing = FALSE;

            for (uint32_t svc_idx = 0u; ((svc_idx < gatt_profile->svc_count) && (FALSE == needs_pairing)); svc_idx++)
            {
                const sid_ble_ext_gatt_svc_def_t * const svc_def = &gatt_profile->services[svc_idx];

                /* Check characteristics within the service */
                for (uint32_t char_idx = 0u; ((char_idx < svc_def->char_count) && (FALSE == needs_pairing)); char_idx++)
                {
                    const sid_ble_ext_char_def_t * const char_def = &svc_def->characteristics[char_idx];

                    if ((char_def->permissions.authen_read != FALSE)
                      || (char_def->permissions.encry_read != FALSE)
                      || (char_def->permissions.authen_write != FALSE)
                      || (char_def->permissions.encry_write != FALSE))
                    {
                        /* Characteristic itself needs pairing */
                        needs_pairing = TRUE;
                        break;
                    }

                    /* Check characteristic descriptors within characteristic */
                    for (uint32_t desc_idx = 0u; desc_idx < char_def->desc_count; desc_idx++)
                    {
                        const sid_ble_ext_char_desc_t * const desc_def = &char_def->descriptors[desc_idx];

                        if ((desc_def->security.authen_read != FALSE)
                          || (desc_def->security.encry_read != FALSE)
                          || (desc_def->security.authen_write != FALSE)
                          || (desc_def->security.encry_write != FALSE))
                        {
                            /* Characteristic itself needs pairing */
                            needs_pairing = TRUE;
                            break;
                        }
                    }
                }
            }
        }

        if (SBEVDR_BLE_CENTRAL == device_cfg->device_type)
        {
            //TODO: implement support for Central role
            needs_pairing = FALSE;
            break;
        }
    } while (0);

    return needs_pairing;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_error_t _ble_adapter_user_set_adv_params(const sid_ble_ext_virtual_device_ctx_t * const device_ctx, const uint32_t adv_interval_min, const uint32_t adv_interval_max)
{
    sid_error_t       err;
    tBleStatus        ble_status;

    SID_PAL_ASSERT(adv_interval_min <= adv_interval_max);
#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
    SID_PAL_ASSERT(adv_interval_min >= SID_BLE_HCI_EXTENDED_ADV_INTERVAL_MIN);
    SID_PAL_ASSERT(adv_interval_max <= SID_BLE_HCI_EXTENDED_ADV_INTERVAL_MAX);
#else
    SID_PAL_ASSERT(adv_interval_min >= SID_BLE_HCI_ADV_INTERVAL_MIN);
    SID_PAL_ASSERT(adv_interval_max <= SID_BLE_HCI_ADV_INTERVAL_MAX);
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */
    SID_PAL_ASSERT(device_ctx != NULL);
    SID_PAL_ASSERT(device_ctx->device_cfg != NULL);

    do
    {
        uint8_t own_addr_type = sid_stm32wba_ble_adapter_util_convert_sid_ble_addr_type_to_hci_own_adv_addr_type(device_ctx->device_cfg->mac_addr_type);
        uint8_t adv_type;
        int8_t  adv_tx_power;
        uint8_t scan_resp_en;

        /* Select advertisement type based on the device role */
        switch (device_ctx->device_cfg->device_type)
        {
            case SBEVDR_BLE_PERIPHERAL:
                SID_PAL_ASSERT(device_ctx->device_cfg->peripheral_cfg.adv_param != NULL);
                adv_tx_power = device_ctx->device_cfg->peripheral_cfg.adv_param->adv_tx_power;
                scan_resp_en = device_ctx->device_cfg->peripheral_cfg.adv_param->scan_resp_en != FALSE ? 0x01u : 0x00u;
                adv_type     = HCI_ADV_TYPE_ADV_IND;
                err          = SID_ERROR_NONE;
                break;

            case SBEVDR_BLE_BROADCASTER:
                SID_PAL_ASSERT(device_ctx->device_cfg->broadcaster_cfg.adv_param != NULL);
                adv_tx_power = device_ctx->device_cfg->broadcaster_cfg.adv_param->adv_tx_power;
                scan_resp_en = device_ctx->device_cfg->broadcaster_cfg.adv_param->scan_resp_en != FALSE ? 0x01u : 0x00u;
                adv_type     = scan_resp_en ? HCI_ADV_TYPE_ADV_SCAN_IND : HCI_ADV_TYPE_ADV_NONCONN_IND;
                err          = SID_ERROR_NONE;
                break;

            default:
                /* Other roles can't advertise */
                err = SID_ERROR_INVALID_ARGS;
                break;
        }

        /* Check if device role is ok */
        if (err != SID_ERROR_NONE)
        {
            break;
        }

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
        if ((adv_tx_power > SID_BLE_HCI_ADV_TX_POWER_MAX) && (adv_tx_power != SID_BLE_HCI_ADV_TX_POWER_NO_PREFERENCE))
        {
            adv_tx_power = SID_BLE_HCI_ADV_TX_POWER_MAX;
        }

        /* Configure extended advertisement set for current virtual device */
        ble_status = hci_le_set_extended_advertising_parameters(
                        device_ctx->device_cfg->device_id,
                        SID_STM32_BLE_LEGACY_ADV_TYPE_TO_EXT_ADV_EVT_PROPS(adv_type), /* Use legacy advertisement PDU */
                        (uint8_t *)(void *)&adv_interval_min,
                        (uint8_t *)(void *)&adv_interval_max,
                        (ADV_CH_37 | ADV_CH_38 | ADV_CH_39),                          /* Use all channels for advertisement */
                        own_addr_type,
                        ia_addr_type,                                                 /* Point to local host peer address for RPA, unused for other host address types */
                        (uint8_t *)ia_addr.bytes,                                     /* Point to local host peer address for RPA, unused for other host address types */
                        HCI_ADV_FILTER_NO,                                            /* Don't use any filters */
                        (uint8_t)adv_tx_power,                                        /* Tx power preference - no guarantees here */
                        HCI_PRIMARY_ADV_PHY_LE_1M,
                        0x00u,                                                        /* Secondary_Advertising_Max_Skip - irrelevant for legacy Adv PDU */
                        HCI_TX_PHY_LE_2M,                                             /* Secondary advertisement PHY - irrelevant for legacy Adv PDU */
                        0x00u,                                                        /* Advertising_SID - irrelevant for legacy Adv PDU */
                        scan_resp_en,                                                 /* Scan request notifications enabled */
                        (uint8_t *)&selected_adv_tx_pwr);                             /* Advertisement Tx power selected by the Host Controller */
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_USER_LOG_ERROR("==>> hci_le_set_extended_advertising_parameters - fail, result: 0x%02X, set: 0x%02X", ble_status, device_ctx->device_cfg->device_id);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        SID_BLE_USER_LOG_DEBUG("==>> hci_le_set_extended_advertising_parameters - Success, set: 0x%02X, Tx power: %s%ddBm", device_ctx->device_cfg->device_id, selected_adv_tx_pwr > 0 ? "+" : "", selected_adv_tx_pwr);
#else
        (void)adv_tx_power;
        (void)scan_resp_en;

        ble_status = hci_le_set_advertising_parameters(
                        (uint16_t)adv_interval_min,
                        (uint16_t)adv_interval_max,
                        adv_type,
                        own_addr_type,
                        ia_addr_type,                                           /* Point to local host peer address for RPA, unused for other host address types */
                        (uint8_t *)ia_addr.bytes,                               /* Point to local host peer address for RPA, unused for other host address types */
                        (ADV_CH_37 | ADV_CH_38 | ADV_CH_39),                    /* Use all channels for advertisement */
                        HCI_ADV_FILTER_NO);                                     /* Don't use any filters */
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_USER_LOG_ERROR("==>> hci_le_set_advertising_parameters - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }

        SID_BLE_USER_LOG_DEBUG("==>> hci_le_set_advertising_parameters - Success");
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_init_advertising_settings(sid_ble_ext_virtual_device_ctx_t * const node_ctx, const sid_ble_ext_adv_param_t * const adv_param)
{
    sid_error_t err;

    do
    {
        uint32_t adv_interval_min;
        uint32_t adv_interval_max;

        if (adv_param->fast_enabled != FALSE)
        {
            adv_interval_min = adv_param->fast_interval_min;
            adv_interval_max = adv_param->fast_interval_max;
        }
        else if (adv_param->slow_enabled != FALSE)
        {
            adv_interval_min = adv_param->slow_interval_min;
            adv_interval_max = adv_param->slow_interval_max;
        }
        else
        {
            SID_PAL_LOG_ERROR("Cannot initialize virtual BLE device \"%s\" (0x%02X). Both fast and slow advertising modes are disabled in config", node_ctx->device_cfg->device_name != NULL ? node_ctx->device_cfg->device_name : "<unnamed>", node_ctx->device_cfg->device_id);
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        err = _ble_adapter_user_set_adv_params(node_ctx, adv_interval_min, adv_interval_max);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to apply advertising params for virtual BLE device \"%s\" (0x%02X). Error %d", node_ctx->device_cfg->device_name != NULL ? node_ctx->device_cfg->device_name : "<unnamed>", node_ctx->device_cfg->device_id, (int32_t)err);
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_init_peripheral_gatt_char_descriptor(sid_ble_ext_gatt_server_char_desc_ctx_t * const out_desc_ctx, const uint16_t service_handle, const uint16_t char_handle,
                                                                                                    const sid_ble_ext_char_desc_t * const desc_def, const uint16_t max_att_mtu)
{
    sid_error_t err;

    do
    {
    	Char_Desc_Uuid_t stm_uuid;
        uint8_t stm_uuid_type;
        tBleStatus ble_status;

        /* Check if characteristic descriptor length and MTU size for the virtual device are aligned */
        if (desc_def->max_length > SID_STM32_BLE_ATT_LEN_LIMIT(max_att_mtu))
        {
            char uuid_text[40];
            const char * desc_display_name;

            if (desc_def->desc_name != NULL)
            {
                desc_display_name = desc_def->desc_name;
            }
            else
            {
                (void)sid_stm32wba_ble_adapter_util_sprintf_uuid(uuid_text, &desc_def->uuid);
                desc_display_name = uuid_text;
            }
            SID_PAL_LOG_ERROR("GATT characteristic descriptor \"%s\" length (%u) exceeds limit (%u) for selected max ATT MTU (%U). HCI error 0x%02X", desc_display_name, desc_def->max_length, SID_STM32_BLE_ATT_LEN_LIMIT(max_att_mtu), max_att_mtu);

            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Store the pointer to char definition for quick access */
        out_desc_ctx->desc_def = desc_def;

        /* Convert UUID to STM BLE stack format */
        err = sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_char_desc_uuid(&stm_uuid_type, &stm_uuid, &desc_def->uuid);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("  Fail   : sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_char_uuid command, error code: %d", err);
            break;
        }

        /* Compute descriptor access permissions */
        uint8_t desc_access = ATTR_NO_ACCESS;
        uint8_t event_mask = GATT_DONT_NOTIFY_EVENTS;
        if (desc_def->access.read != FALSE)
        {
            desc_access |= ATTR_ACCESS_READ_ONLY;
            event_mask |= GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
        }
        if (desc_def->access.write != FALSE)
        {
            desc_access |= ATTR_ACCESS_WRITE_REQ_ONLY;
            event_mask |= (GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP | GATT_NOTIFY_ATTRIBUTE_WRITE);
        }
        if (desc_def->access.write_wo_resp != FALSE)
        {
            desc_access |= ATTR_ACCESS_WRITE_WITHOUT_RESPONSE;
            event_mask |= GATT_NOTIFY_ATTRIBUTE_WRITE;
        }

        /* Compute characteristic descriptor security permissions */
        uint8_t desc_security = ATTR_PERMISSION_NONE;
        if (desc_def->security.authen_read != FALSE)
        {
            desc_security |= ATTR_PERMISSION_AUTHEN_READ;
        }
        if (desc_def->security.author_read != FALSE)
        {
            desc_security |= ATTR_PERMISSION_AUTHOR_READ;
        }
        if (desc_def->security.encry_read != FALSE)
        {
            desc_security |= ATTR_PERMISSION_ENCRY_READ;
        }
        if (desc_def->security.authen_write != FALSE)
        {
            desc_security |= ATTR_PERMISSION_AUTHEN_WRITE;
        }
        if (desc_def->security.author_write != FALSE)
        {
            desc_security |= ATTR_PERMISSION_AUTHOR_WRITE;
        }
        if (desc_def->security.encry_write != FALSE)
        {
            desc_security |= ATTR_PERMISSION_ENCRY_WRITE;
        }

        /* Add characteristic to GATT database */
        ble_status = aci_gatt_add_char_desc(service_handle,
                                            char_handle,
                                            stm_uuid_type,
                                            &stm_uuid,
                                            desc_def->max_length,
                                            0u,
                                            NULL,
                                            desc_security,
                                            desc_access,
                                            event_mask,
                                            SID_STM32_BLE_CHAR_ENC_KEY_MINIMUM_LENGTH,
                                            CHAR_VALUE_LEN_VARIABLE,
                                            &out_desc_ctx->handle);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            char uuid_text[40];
            const char * desc_display_name;

            if (desc_def->desc_name != NULL)
            {
                desc_display_name = desc_def->desc_name;
            }
            else
            {
                (void)sid_stm32wba_ble_adapter_util_sprintf_uuid(uuid_text, &desc_def->uuid);
                desc_display_name = uuid_text;
            }
            SID_PAL_LOG_ERROR("Failed to add GATT characteristic descriptor \"%s\". HCI error 0x%02X", desc_display_name, ble_status);

            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_init_peripheral_gatt_characteristic(sid_ble_ext_gatt_server_char_ctx_t * const out_char_ctx, const uint16_t service_handle, const sid_ble_ext_char_def_t * const char_def, const uint16_t max_att_mtu)
{
    sid_error_t err;

    do
    {
        Char_UUID_t stm_uuid;
        uint8_t stm_uuid_type;
        tBleStatus ble_status;

        /* Check if characteristic length and MTU size for the virtual device are aligned */
        if (char_def->max_length > SID_STM32_BLE_ATT_LEN_LIMIT(max_att_mtu))
        {
            char uuid_text[40];
            const char * char_display_name;

            if (char_def->char_name != NULL)
            {
                char_display_name = char_def->char_name;
            }
            else
            {
                (void)sid_stm32wba_ble_adapter_util_sprintf_uuid(uuid_text, &char_def->uuid);
                char_display_name = uuid_text;
            }
            SID_PAL_LOG_ERROR("GATT characteristic \"%s\" length (%u) exceeds limit (%u) for selected max ATT MTU (%U). HCI error 0x%02X", char_display_name, char_def->max_length, SID_STM32_BLE_ATT_LEN_LIMIT(max_att_mtu), max_att_mtu);

            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Store the pointer to char definition for quick access */
        out_char_ctx->char_def = char_def;

        /* Convert UUID to STM BLE stack format */
        err = sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_char_uuid(&stm_uuid_type, &stm_uuid, &char_def->uuid);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("  Fail   : sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_char_uuid command, error code: %d", err);
            break;
        }

        /* Compute characteristic settings */
        uint8_t char_props = CHAR_PROP_NONE;
        uint8_t event_mask = GATT_DONT_NOTIFY_EVENTS;
        if (char_def->properties.read != FALSE)
        {
            char_props |= CHAR_PROP_READ;
            event_mask |= GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
        }
        if (char_def->properties.write != FALSE)
        {
            char_props |= CHAR_PROP_WRITE;
            event_mask |= (GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP | GATT_NOTIFY_ATTRIBUTE_WRITE);
        }
        if (char_def->properties.write_wo_resp != FALSE)
        {
            char_props |= CHAR_PROP_WRITE_WITHOUT_RESP;
            event_mask |= GATT_NOTIFY_ATTRIBUTE_WRITE;
        }
        if (char_def->properties.notify != FALSE)
        {
            char_props |= CHAR_PROP_NOTIFY;
            event_mask |= GATT_NOTIFY_NOTIFICATION_COMPLETION;
        }

        /* Compute characteristic security permissions */
        uint8_t char_perms = ATTR_PERMISSION_NONE;
        if (char_def->permissions.authen_read != FALSE)
        {
            char_perms |= ATTR_PERMISSION_AUTHEN_READ;
        }
        if (char_def->permissions.author_read != FALSE)
        {
            char_perms |= ATTR_PERMISSION_AUTHOR_READ;
        }
        if (char_def->permissions.encry_read != FALSE)
        {
            char_perms |= ATTR_PERMISSION_ENCRY_READ;
        }
        if (char_def->permissions.authen_write != FALSE)
        {
            char_perms |= ATTR_PERMISSION_AUTHEN_WRITE;
        }
        if (char_def->permissions.author_write != FALSE)
        {
            char_perms |= ATTR_PERMISSION_AUTHOR_WRITE;
        }
        if (char_def->permissions.encry_write != FALSE)
        {
            char_perms |= ATTR_PERMISSION_ENCRY_WRITE;
        }

        /* Add characteristic to GATT database */
        ble_status = aci_gatt_add_char(service_handle,
                                       stm_uuid_type,
                                       &stm_uuid,
                                       char_def->max_length,
                                       char_props,
                                       char_perms,
                                       event_mask,
                                       SID_STM32_BLE_CHAR_ENC_KEY_MINIMUM_LENGTH,
                                       CHAR_VALUE_LEN_VARIABLE,
                                       &out_char_ctx->handle);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            char uuid_text[40];
            const char * char_display_name;

            if (char_def->char_name != NULL)
            {
                char_display_name = char_def->char_name;
            }
            else
            {
                (void)sid_stm32wba_ble_adapter_util_sprintf_uuid(uuid_text, &char_def->uuid);
                char_display_name = uuid_text;
            }
            SID_PAL_LOG_ERROR("Failed to add GATT characteristic \"%s\". HCI error 0x%02X", char_display_name, ble_status);

            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Clear characteristic value */
        (void)aci_gatt_update_char_value(service_handle, out_char_ctx->handle, 0u, 0u, NULL);

        /* Add characteristics descriptors to controller -------------------------*/

        /* Check if this characteristic contains any special descriptors (e.g. SCCD, CCCD, etc.) */
        uint32_t special_desc_cnt = char_def->desc_count;
        uint32_t add_cccd = FALSE;
        if (char_def->properties.notify != FALSE)
        {
            special_desc_cnt++;
            add_cccd = TRUE;
        }

        /* Process special descriptors */
        uint32_t desc_idx_offset = 0u;
        if (add_cccd != FALSE)
        {
            sid_ble_ext_gatt_server_char_desc_ctx_t * const cccd_ctx = &out_char_ctx->desc_ctxs[desc_idx_offset];

            /* Update CCCD handle in the context */
            cccd_ctx->handle = out_char_ctx->handle + SID_STM32_BLE_CHARACTERISTIC_CCCD_ATTRIBUTE_OFFSET;

            desc_idx_offset++;
        }

        for (uint32_t i = 0u; i < char_def->desc_count; i++)
        {
            err = _ble_adapter_user_init_peripheral_gatt_char_descriptor(&out_char_ctx->desc_ctxs[i + desc_idx_offset], service_handle, out_char_ctx->handle, &char_def->descriptors[i], max_att_mtu);
            if (err != SID_ERROR_NONE)
            {
                break;
            }
        }

        /* Jump out if characteristic creation failed */
        if (err != SID_ERROR_NONE)
        {
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_init_peripheral_gatt_service(sid_ble_ext_gatt_server_svc_ctx_t * const out_svc_ctx, const sid_ble_ext_gatt_svc_def_t * const svc_definition, const uint16_t max_att_mtu)
{
    sid_error_t err;
    tBleStatus  ble_status;

    do
    {
        /* Allocate memory for characteristic contexts */
        out_svc_ctx->char_ctxs = calloc(svc_definition->char_count, sizeof(*out_svc_ctx->char_ctxs));
        if (NULL == out_svc_ctx->char_ctxs)
        {
            err = SID_ERROR_OOM;
            break;
        }

        /* Store the total number of service contexts in use */
        out_svc_ctx->char_ctxs_count = svc_definition->char_count;

        /* Set invalid handles before any further actions */
        for (uint32_t i = 0u; i < svc_definition->char_count; i++)
        {
            out_svc_ctx->char_ctxs[i].handle = SID_STM32_BLE_HANDLE_INVALID_VALUE;
        }

        /* Allocate memory for all characteristic descriptors */
        uint32_t total_descriptors = 0u;
        err = SID_ERROR_NONE;
        for (uint32_t i = 0u; i < svc_definition->char_count; i++)
        {
            const sid_ble_ext_char_def_t * const char_def = &svc_definition->characteristics[i];
            sid_ble_ext_gatt_server_char_ctx_t * const char_ctx = &out_svc_ctx->char_ctxs[i];

            /* Allocate characteristic value buffer */
            char_ctx->data = malloc(char_def->max_length);
            if (NULL == char_ctx->data)
            {
                err = SID_ERROR_OOM;
                break;
            }

            /* Check if this characteristic contains any special descriptors (e.g. SCCD, CCCD, etc.) */
            uint32_t special_desc_cnt = char_def->desc_count;
            uint32_t add_cccd = FALSE;
            if (char_def->properties.notify != FALSE)
            {
                special_desc_cnt++;
                add_cccd = TRUE;
            }

            /* If this characteristic has any descriptors at all */
            if ((char_def->desc_count + special_desc_cnt) > 0u)
            {
                /* Allocate memory for descriptor contexts */
                char_ctx->desc_ctxs = calloc((char_def->desc_count + special_desc_cnt), sizeof(*char_ctx->desc_ctxs));
                if (NULL == char_ctx->desc_ctxs)
                {
                    err = SID_ERROR_OOM;
                    break;
                }

                /* Store the total number of descriptor contexts in use */
                char_ctx->desc_ctxs_count = (char_def->desc_count + special_desc_cnt);
                total_descriptors += char_def->desc_count; /* special_desc_cnt is out of scope for this counter as special descriptors are managed by ACI */

                /* Process special descriptors */
                uint32_t desc_idx_offset = 0u;
                if (add_cccd != FALSE)
                {
                    sid_ble_ext_gatt_server_char_desc_ctx_t * const cccd_ctx = &char_ctx->desc_ctxs[desc_idx_offset];

                    /* Set invalid handle value for the characteristic descriptor */
                    cccd_ctx->handle = SID_STM32_BLE_HANDLE_INVALID_VALUE;

                    cccd_ctx->desc_def = &cccd_def;

                    /* Allocate characteristic value buffer */
                    cccd_ctx->data = malloc(cccd_ctx->desc_def->max_length);
                    if (NULL == cccd_ctx->data)
                    {
                        err = SID_ERROR_OOM;
                        break;
                    }

                    desc_idx_offset++;
                }

                /* Set invalid handles before any further actions */
                for (uint32_t j = 0u; j < char_def->desc_count; j++)
                {
                    const sid_ble_ext_char_desc_t * const desc_def = &char_def->descriptors[j];
                    sid_ble_ext_gatt_server_char_desc_ctx_t * const desc_ctx = &char_ctx->desc_ctxs[j + desc_idx_offset];

                    /* Set invalid handle value for the characteristic descriptor */
                    desc_ctx->handle = SID_STM32_BLE_HANDLE_INVALID_VALUE;

                    desc_ctx->desc_def = desc_def;

                    /* Allocate characteristic value buffer */
                    desc_ctx->data = malloc(desc_def->max_length);
                    if (NULL == desc_ctx->data)
                    {
                        err = SID_ERROR_OOM;
                        break;
                    }
                }

                /* Jump out if memory allocation failed */
                if (err != SID_ERROR_NONE)
                {
                    break;
                }
            }
        }

        /* Jump out if memory allocation failed */
        if (err != SID_ERROR_NONE)
        {
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Add service to the controller ---------------------------------------------*/
        uint8_t        stm_uuid_type;
        Service_UUID_t stm_uuid;

        /* Convert UUID to the controller byte order */
        err = sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_svc_uuid(&stm_uuid_type, &stm_uuid, &svc_definition->uuid);
        if (err != SID_ERROR_NONE)
        {
            SID_BLE_USER_LOG_ERROR("  Fail   : sid_stm32wba_ble_adapter_util_convert_sid_uuid_to_stm32_svc_uuid command, error code: %d", err);
            break;
        }

        /* Add service to GATT database and retrieve its handle */
        ble_status = aci_gatt_add_service(stm_uuid_type,
                                          &stm_uuid,
                                          PRIMARY_SERVICE,
                                          (2u * (svc_definition->char_count + total_descriptors)) + 2u, /* Max_Attribute_Records = 2*no_of_char + 2*num_of_desc + 2 for svc */
                                          &out_svc_ctx->handle);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            char uuid_text[40];
            const char * svc_display_name;

            if (svc_definition->service_name != NULL)
            {
                svc_display_name = svc_definition->service_name;
            }
            else
            {
                (void)sid_stm32wba_ble_adapter_util_sprintf_uuid(uuid_text, &svc_definition->uuid);
                svc_display_name = uuid_text;
            }
            SID_PAL_LOG_ERROR("Failed to create GATT service \"%s\". HCI error 0x%02X", svc_display_name, ble_status);

            err = SID_ERROR_IO_ERROR;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Add characteristics and descriptors to controller -------------------------*/
        for (uint32_t i = 0u; i < svc_definition->char_count; i++)
        {
            err = _ble_adapter_user_init_peripheral_gatt_characteristic(&out_svc_ctx->char_ctxs[i], out_svc_ctx->handle, &svc_definition->characteristics[i], max_att_mtu);
            if (err != SID_ERROR_NONE)
            {
                break;
            }
        }

        /* Jump out if characteristic creation failed */
        if (err != SID_ERROR_NONE)
        {
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_init_peripheral_gatt_services(sid_ble_ext_virtual_device_ctx_t * const node_ctx, const sid_ble_ext_gatt_profile_def_t * const gatt_profile)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Allocate memory for service contexts */
        node_ctx->periph.svc_ctxs = calloc(gatt_profile->svc_count, sizeof(*node_ctx->periph.svc_ctxs));
        if (NULL == node_ctx->periph.svc_ctxs)
        {
            err = SID_ERROR_OOM;
            break;
        }

        /* Store the total number of service contexts in use */
        node_ctx->periph.svc_ctxs_count = gatt_profile->svc_count;

        /* Set invalid handles before any further actions */
        for (uint32_t i = 0u; i < gatt_profile->svc_count; i++)
        {
            node_ctx->periph.svc_ctxs[i].handle = SID_STM32_BLE_HANDLE_INVALID_VALUE;
        }

        /* Instantiate each service */
        for (uint32_t i = 0u; i < gatt_profile->svc_count; i++)
        {
            err = _ble_adapter_user_init_peripheral_gatt_service(&node_ctx->periph.svc_ctxs[i], &gatt_profile->services[i], node_ctx->device_cfg->peripheral_cfg.max_att_mtu);
            if (err != SID_ERROR_NONE)
            {
                break;
            }
        }

        /* Jump out if service creation failed */
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to load GATT profile for virtual BLE device \"%s\" (0x%02X). Error %d", node_ctx->device_cfg->device_name != NULL ? node_ctx->device_cfg->device_name : "<unnamed>", node_ctx->device_cfg->device_id, err);
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_init_peripheral(sid_ble_ext_virtual_device_node_t * const node)
{
    sid_error_t err;

    SID_PAL_ASSERT(node != NULL);
    SID_PAL_ASSERT(node->ctx != NULL);
    SID_PAL_ASSERT(node->ctx->device_cfg != NULL);
    SID_PAL_ASSERT(node->ctx->device_cfg->peripheral_cfg.adv_param != NULL);
    SID_PAL_ASSERT(SBEVDR_BLE_PERIPHERAL == node->ctx->device_cfg->device_type);

    do
    {
        const char * dev_name;
        uint32_t dev_name_len, dev_name_len_limit;
        uint8_t adv_name_type;
        uint8_t adv_bin_data[SID_STM32_BLE_MAX_ADVERTISING_BUFFER_SIZE];
        const sid_ble_ext_adv_param_t * const adv_param = node->ctx->device_cfg->peripheral_cfg.adv_param;

        /* Validate inputs */
        if ((0u == node->ctx->device_cfg->peripheral_cfg.max_conn) || (node->ctx->device_cfg->peripheral_cfg.max_conn > CFG_BLE_NUM_LINK))
        {
            SID_PAL_LOG_ERROR("Invalid max connection number for virtual BLE device \"%s\" (0x%02X)", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id);
            err = SID_ERROR_INVALID_ARGS;
            break;
        }
        if ((node->ctx->device_cfg->peripheral_cfg.max_att_mtu < SID_BLE_HCI_DEFAULT_ATT_MTU) || (node->ctx->device_cfg->peripheral_cfg.max_att_mtu > CFG_BLE_ATT_MTU_MAX))
        {
            SID_PAL_LOG_ERROR("Invalid max MTU size for virtual BLE device \"%s\" (0x%02X)", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id);
            err = SID_ERROR_INVALID_ARGS;
            break;
        }
        if ((0u == node->ctx->device_cfg->peripheral_cfg.gatt_profile.svc_count) || (NULL == node->ctx->device_cfg->peripheral_cfg.gatt_profile.services))
        {
            SID_PAL_LOG_ERROR("Empty GATT profile for virtual BLE device \"%s\" (0x%02X)", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id);
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Instantiate GATT services */
        err = _ble_adapter_user_init_peripheral_gatt_services(node->ctx, &node->ctx->device_cfg->peripheral_cfg.gatt_profile);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_user_init_peripheral_gatt_services() */
            break;
        }

        /* Init advertising settings (e.g. create extended advertising set if supported) */
        err = _ble_adapter_user_init_advertising_settings(node->ctx, adv_param);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_user_init_advertising_settings() */
            break;
        }

        /* Build default advertising/scan response data */
        dev_name           = node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>";
        dev_name_len       = strlen(dev_name);
        dev_name_len_limit = sizeof(adv_bin_data) - (SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE + SID_BLE_HCI_ADV_TYPE_RECORD_SIZE); /* Space taken by device name header */

        /* Truncate device name if necessary */
        if (dev_name_len > dev_name_len_limit)
        {
            /* Determine the name type depending on if it fits or not */
            dev_name_len  = dev_name_len_limit;
            adv_name_type = AD_TYPE_SHORTENED_LOCAL_NAME;
        }
        else
        {
            adv_name_type = AD_TYPE_COMPLETE_LOCAL_NAME;
        }

        /* Copy as much as fits into the buffer */
        adv_bin_data[0] = SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + dev_name_len;
        adv_bin_data[1] = adv_name_type;
        SID_STM32_UTIL_fast_memcpy(&adv_bin_data[2], dev_name, dev_name_len);

        /* Set default scan response data if scan responses are enabled */
        if (adv_param->scan_resp_en != FALSE)
        {
            err = _ble_adapter_user_set_node_scan_resp_data(node, adv_bin_data, (adv_bin_data[0] + SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE));
            if (err != SID_ERROR_NONE)
            {
                /* Logs provided by _ble_adapter_user_set_node_scan_resp_data() */
                break;
            }
        }

        /* Prepare default advertisement data - it fits a bit less bytes */
        dev_name_len_limit -= (SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE + SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + SID_BLE_HCI_ADV_FLAGS_RECORD_SIZE); /* Space taken by mandatory advedrtising flags */
        
        /* Truncate device name even more if necessary */
        if (dev_name_len > dev_name_len_limit)
        {
            /* Determine the name type depending on if it fits or not */
            dev_name_len  = dev_name_len_limit;
            adv_name_type = AD_TYPE_SHORTENED_LOCAL_NAME;

            /* Update header */
            adv_bin_data[0] = SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + dev_name_len;
            adv_bin_data[1] = adv_name_type;
        }

        /* Set default advertising data */
        err = _ble_adapter_user_set_node_adv_data(node, adv_bin_data, (adv_bin_data[0] + SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE));
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_user_set_node_adv_data() */
            break;
        }

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING)
        tBleStatus ble_status;

        /* Update GAP device name */
        if (dev_name_len > SID_STM32_BLE_GAP_DEVICE_NAME_MAX_LEN)
        {
            dev_name_len = SID_STM32_BLE_GAP_DEVICE_NAME_MAX_LEN;
        }
        ble_status = aci_gatt_update_char_value(sid_ble_drv_ctx.gap_service_handle,
                                                sid_ble_drv_ctx.gap_dev_name_char_handle,
                                                0u,
                                                dev_name_len,
                                                (uint8_t *)dev_name);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_ERROR("  Fail   : aci_gatt_update_char_value - Device Name, result: 0x%02X", ble_status);
            /* Do not fail the entire bootstrap process since this error is not a blocker */
        }
        else
        {
            SID_PAL_LOG_DEBUG("  Success: aci_gatt_update_char_value - Device Name");
        }

        /* Update preferred connection parameters */
        const sid_ble_cfg_conn_param_t * const conn_param = node->ctx->device_cfg->peripheral_cfg.conn_param;
        if (conn_param != NULL)
        {
            (void)sid_stm32wba_ble_adapter_util_set_gap_ppcp_char(conn_param);

            /* Do not fail the entire bootstrap process since PPCP error is not a blocker */
        }
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING */

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_init_broadcaster(sid_ble_ext_virtual_device_node_t * const node)
{
    sid_error_t err;

    SID_PAL_ASSERT(node != NULL);
    SID_PAL_ASSERT(node->ctx != NULL);
    SID_PAL_ASSERT(node->ctx->device_cfg != NULL);
    SID_PAL_ASSERT(node->ctx->device_cfg->broadcaster_cfg.adv_param != NULL);
    SID_PAL_ASSERT(SBEVDR_BLE_BROADCASTER == node->ctx->device_cfg->device_type);

    do
    {
        const char * dev_name;
        uint32_t dev_name_len, dev_name_len_limit;
        uint8_t adv_name_type;
        uint8_t adv_bin_data[SID_STM32_BLE_MAX_ADVERTISING_BUFFER_SIZE];
        const sid_ble_ext_adv_param_t * const adv_param = node->ctx->device_cfg->broadcaster_cfg.adv_param;

        err = _ble_adapter_user_init_advertising_settings(node->ctx, adv_param);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_user_init_advertising_settings() */
            break;
        }

        /* Build default advertising/scan response data */
        dev_name           = node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>";
        dev_name_len       = strlen(dev_name);
        dev_name_len_limit = sizeof(adv_bin_data) - (SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE + SID_BLE_HCI_ADV_TYPE_RECORD_SIZE); /* Space taken by device name header */

        /* Truncate device name if necessary */
        if (dev_name_len > dev_name_len_limit)
        {
            /* Determine the name type depending on if it fits or not */
            dev_name_len  = dev_name_len_limit;
            adv_name_type = AD_TYPE_SHORTENED_LOCAL_NAME;
        }
        else
        {
            adv_name_type = AD_TYPE_COMPLETE_LOCAL_NAME;
        }

        /* Copy as much as fits into the buffer */
        adv_bin_data[0] = SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + dev_name_len;
        adv_bin_data[1] = adv_name_type;
        SID_STM32_UTIL_fast_memcpy(&adv_bin_data[2], dev_name, dev_name_len);

        /* Set default scan response data if scan responses are enabled */
        if (adv_param->scan_resp_en != FALSE)
        {
            err = _ble_adapter_user_set_node_scan_resp_data(node, adv_bin_data, (adv_bin_data[0] + SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE));
            if (err != SID_ERROR_NONE)
            {
                /* Logs provided by _ble_adapter_user_set_node_scan_resp_data() */
                break;
            }
        }

        /* Prepare default advertisement data - it fits a bit less bytes */
        dev_name_len_limit -= (SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE + SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + SID_BLE_HCI_ADV_FLAGS_RECORD_SIZE); /* Space taken by mandatory advedrtising flags */
        
        /* Truncate device name even more if necessary */
        if (dev_name_len > dev_name_len_limit)
        {
            /* Determine the name type depending on if it fits or not */
            dev_name_len  = dev_name_len_limit;
            adv_name_type = AD_TYPE_SHORTENED_LOCAL_NAME;

            /* Update header */
            adv_bin_data[0] = SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + dev_name_len;
            adv_bin_data[1] = adv_name_type;
        }

        /* Set default advertising data */
        err = _ble_adapter_user_set_node_adv_data(node, adv_bin_data, (adv_bin_data[0] + SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE));
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_user_set_node_adv_data() */
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _ble_adapter_user_init_central(sid_ble_ext_virtual_device_node_t * const node)
{
    (void)node;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _ble_adapter_user_init_observer(sid_ble_ext_virtual_device_node_t * const node)
{
    (void)node;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_deinit_peripheral(sid_ble_ext_virtual_device_node_t * const node)
{
    sid_error_t err;
    sid_error_t svc_del_err;
    tBleStatus  ble_status;

    SID_PAL_ASSERT(node != NULL);
    SID_PAL_ASSERT(node->ctx != NULL);
    SID_PAL_ASSERT(node->ctx->device_cfg != NULL);

    do
    {
        /* Ensure advertising is stopped */
        (void)_ble_adapter_user_node_stop_advertisement(node);

#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
        /* Delete associated advertisement set */
        ble_status = hci_le_remove_advertising_set(node->ctx->device_cfg->device_id);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_WARNING("Failed to remove advertising set for virtual BLE device \"%s\" (0x%02X). Status 0x%02X", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id, ble_status);
        }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        /* Terminate all the connections */
        while (LST_is_empty((tListNode *)&node->ctx->conn_list) == FALSE)
        {
            sid_ble_ext_conn_list_node_t * conn_node;

            LST_get_next_node((tListNode *)&node->ctx->conn_list, (tListNode **)&conn_node);
            err = _ble_adapter_user_node_disconnect(node, conn_node);
            if (err != SID_ERROR_NONE)
            {
                /* We can't disconnect - just skip this connection, remove it from list and deallocate memory */
                LST_remove_node((tListNode *)conn_node);
                free(conn_node);
            }
        }
        /*----------------------------------------------------------------------------*/

        /* Remove associated service(s) from GATT database ---------------------------*/
        /* Clear the root pointer to the contexts tree */
        sid_pal_enter_critical_region();
        sid_ble_ext_gatt_server_svc_ctx_t * const svc_ctxs       = node->ctx->periph.svc_ctxs;
        const uint32_t                            svc_ctxs_count = node->ctx->periph.svc_ctxs_count;
        node->ctx->periph.svc_ctxs_count = 0u;
        node->ctx->periph.svc_ctxs = NULL;
        sid_pal_exit_critical_region();

        /* Delete entire services from the controller */
        svc_del_err = SID_ERROR_NONE;
        for (uint32_t i = 0u; i < svc_ctxs_count; i++)
        {
            sid_ble_ext_gatt_server_svc_ctx_t * const svc_ctx = &svc_ctxs[i];

            ble_status = aci_gatt_del_service(svc_ctx->handle);
            if (ble_status != BLE_STATUS_SUCCESS)
            {
                char uuid_text[40];
                const char * svc_display_name;

                if (svc_ctx->svc_def->service_name != NULL)
                {
                    svc_display_name = svc_ctx->svc_def->service_name;
                }
                else
                {
                    (void)sid_stm32wba_ble_adapter_util_sprintf_uuid(uuid_text, &svc_ctx->svc_def->uuid);
                    svc_display_name = uuid_text;
                }
                SID_PAL_LOG_ERROR("Failed to delete GATT service \"%s\". HCI error 0x%02X", svc_display_name, ble_status);

                /* Set the error, but proceed with deleting the other services */
                svc_del_err = SID_ERROR_IO_ERROR;
            }

            if (NULL == svc_ctx->char_ctxs)
            {
                svc_ctx->char_ctxs_count = 0u;
                continue;
            }

            /* Iterate over the list of characteristics and deallocate memory */
            for (uint32_t j = 0u; j < svc_ctx->char_ctxs_count; j++)
            {
                sid_ble_ext_gatt_server_char_ctx_t * const char_ctx = &svc_ctx->char_ctxs[j];

                /* Deallocate characterstic value buffer */
                if (char_ctx->data != NULL)
                {
                    free(char_ctx->data);
                }
                char_ctx->data = NULL;
                char_ctx->data_length = 0u;

                /* Skip if there are no characteristic descriptors */
                if (NULL == char_ctx->desc_ctxs)
                {
                    svc_ctx->char_ctxs_count = 0u;
                    char_ctx->desc_ctxs_count = 0u;
                    continue;
                }

                /* Iterate over the list of characteristics and deallocate memory */
                for (uint32_t k = 0u; k < char_ctx->desc_ctxs_count; k++)
                {
                    sid_ble_ext_gatt_server_char_desc_ctx_t * const desc_ctx = &char_ctx->desc_ctxs[k];

                    if (desc_ctx->data != NULL)
                    {
                        free(desc_ctx->data);
                    }
                    desc_ctx->data = NULL;
                    desc_ctx->data_length = 0u;
                }

                /* Deallocate char descriptor contexts */
                char_ctx->desc_ctxs_count = 0u;
                free(char_ctx->desc_ctxs);
                char_ctx->desc_ctxs = NULL;
            }

            /* Deallocate characteristic contexts for all chars in the service */
            svc_ctx->char_ctxs_count = 0u;
            free(svc_ctx->char_ctxs);
            svc_ctx->char_ctxs = NULL;
        }

        /* Deallocate contexts for all the services in the GATT profile */
        free(svc_ctxs);

        /* Done */
        err = svc_del_err;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _ble_adapter_user_deinit_broadcaster(sid_ble_ext_virtual_device_node_t * const node)
{
    sid_error_t err;

    SID_PAL_ASSERT(node != NULL);
    SID_PAL_ASSERT(node->ctx != NULL);
    SID_PAL_ASSERT(node->ctx->device_cfg != NULL);

    do
    {
        /* Ensure advertising is stopped */
        (void)_ble_adapter_user_node_stop_advertisement(node);

#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
        /* Delete associated advertisement set */
        tBleStatus ble_status = hci_le_remove_advertising_set(node->ctx->device_cfg->device_id);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_PAL_LOG_WARNING("Failed to remove advertising set for virtual BLE device \"%s\" (0x%02X). Status 0x%02X", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id, ble_status);
        }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _ble_adapter_user_deinit_central(sid_ble_ext_virtual_device_node_t * const node)
{
    (void)node;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _ble_adapter_user_deinit_observer(sid_ble_ext_virtual_device_node_t * const node)
{
    (void)node;
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_terminate_device_node(sid_ble_ext_virtual_device_node_t * const node)
{
    sid_error_t err;

    SID_PAL_ASSERT(node != NULL);

    do
    {
        /* Ensure the node is marked as terminating */
        sid_pal_enter_critical_region();
        /* Update device state */
        node->ctx->state = SBEVD_STATE_TERMINATING;
        __COMPILER_BARRIER();
        sid_pal_exit_critical_region();

        /* Perform role-specific tear down (e.g. remove adv set, terminate all connections, etc.) */
        switch (node->ctx->device_cfg->device_type)
        {
            case SBEVDR_BLE_PERIPHERAL:
                err = _ble_adapter_user_deinit_peripheral(node);
                break;

            case SBEVDR_BLE_BROADCASTER:
                err = _ble_adapter_user_deinit_broadcaster(node);
                break;

            case SBEVDR_BLE_CENTRAL:
                err = _ble_adapter_user_deinit_central(node);
                break;

            case SBEVDR_BLE_OBSERVER:
                err = _ble_adapter_user_deinit_observer(node);
                break;

            default:
                err = SID_ERROR_INVALID_ARGS;
                break;
        }

        /* Exclude node from the list of active nodes */
        LST_remove_node((tListNode * )node);

        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to terminate virtual BLE device \"%s\" (0x%02X). Error %d", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id, (int32_t)err);
            /* Don't jump out of loop here as we still need to deallocate memory */
        }
        else
        {
            /* Print out status message now as we are about to destroy node context, it won't be possible to print device name & ID after th memory is deallocated */
            SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X) terminated", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id);
        }

        /* Deallocate memory for the node */
        free(node->ctx);
        free(node);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_set_node_adv_data(sid_ble_ext_virtual_device_node_t * const node, const uint8_t * const raw_data, const uint32_t raw_data_length)
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus  ble_status;

    SID_PAL_ASSERT(node != NULL);
    SID_PAL_ASSERT(node->ctx != NULL);
    SID_PAL_ASSERT(node->ctx->device_cfg != NULL);

    do
    {
        uint8_t adv_bin_data[SID_STM32_BLE_MAX_ADVERTISING_BUFFER_SIZE];

        /* Validate inputs */
        if ((NULL == raw_data) || (0u == raw_data_length))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if ((node->ctx->device_cfg->device_type != SBEVDR_BLE_BROADCASTER) && (node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL))
        {
            SID_PAL_LOG_ERROR("Advertising data can be set for Broadcaster and Peripheral roles only. Virtual BLE device \"%s\" (0x%02X)", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id);
            err = SID_ERROR_NOSUPPORT;
            break;
        }

        /* Cleanup any previous data */
        SID_STM32_UTIL_fast_memset(adv_bin_data, 0u, sizeof(adv_bin_data));

        /* Build advertisement data */
        uint32_t adv_pos = 0u;

        /* Add advertisement flags */
        adv_bin_data[adv_pos] = SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + SID_BLE_HCI_ADV_FLAGS_RECORD_SIZE;
        adv_pos += SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE;
        adv_bin_data[adv_pos] = AD_TYPE_FLAGS;
        adv_pos += SID_BLE_HCI_ADV_TYPE_RECORD_SIZE;
        adv_bin_data[adv_pos] = (FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE | FLAG_BIT_BR_EDR_NOT_SUPPORTED);
        adv_pos += SID_BLE_HCI_ADV_FLAGS_RECORD_SIZE;

        /* Verify the supplied raw data fits into the buffer */
        if (raw_data_length > (sizeof(adv_bin_data) - adv_pos))
        {
            SID_PAL_LOG_ERROR("Supplied advertising data is too long. Virtual BLE device \"%s\" (0x%02X)", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id);
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Pre-validate raw data */
        uint32_t raw_data_pos = 0u;
        while (raw_data_pos < raw_data_length)
        {
            if (raw_data[raw_data_pos] < (SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + SID_BLE_HCI_ADV_RECORD_SIZE_MIN))
            {
                /* That's invalid, this byte should keep the length of the advertising data record which is at least two bytes */
                raw_data_pos = UINT32_MAX;
                break;
            }

            /* Advance position by the length of the current advertising data record */
            raw_data_pos += (raw_data[raw_data_pos] + SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE);
        }

        if (raw_data_pos != raw_data_length)
        {
            /* Mismatch between the supplied adv data length and the actual computed length based on the adv data format */
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Copy user-supplied data into the buffer */
        SID_STM32_UTIL_fast_memcpy(&adv_bin_data[adv_pos], raw_data, raw_data_length);
        adv_pos += raw_data_length;

        SID_PAL_ASSERT(adv_pos <= MAX_ADV_DATA_LEN);

        /* Put advertisement data into the host controller */
#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
        ble_status = hci_le_set_extended_advertising_data(node->ctx->device_cfg->device_id,
                                                          HCI_SET_ADV_DATA_OPERATION_COMPLETE,
                                                          SID_BLE_HCI_SET_ADV_DATA_FRAGMENT_PREF_NO_FRAGMENT,
                                                          sizeof(adv_bin_data), (uint8_t *)adv_bin_data);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_USER_LOG_ERROR("==>> hci_le_set_extended_advertising_data - fail, result: 0x%02X, set: 0x%02X", ble_status, node->ctx->device_cfg->device_id);
            err = SID_BLE_HCI_STATUS_UNKNOWN_ADV_IDENTIFIER == ble_status ? SID_ERROR_UNINITIALIZED : SID_ERROR_IO_ERROR; /* Advertisement set shall be created first via calling hci_le_set_extended_advertising_parameters */
            break;
        }
        SID_BLE_USER_LOG_DEBUG("==>> Success: hci_le_set_extended_advertising_data, set: 0x%02X", node->ctx->device_cfg->device_id);
#else
        /* Use legacy advertisement API */
        ble_status = hci_le_set_advertising_data(adv_pos, (uint8_t *)adv_bin_data);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_USER_LOG_ERROR("==>> hci_le_set_advertising_data - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_USER_LOG_DEBUG("==>> Success: hci_le_set_advertising_data");
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_set_node_scan_resp_data(sid_ble_ext_virtual_device_node_t * const node, const uint8_t * const raw_data, const uint32_t raw_data_length)
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus  ble_status;

    SID_PAL_ASSERT(node != NULL);
    SID_PAL_ASSERT(node->ctx != NULL);
    SID_PAL_ASSERT(node->ctx->device_cfg != NULL);

    do
    {
        uint8_t sr_bin_data[SID_STM32_BLE_MAX_ADVERTISING_BUFFER_SIZE];

        /* Validate inputs */
        if ((NULL == raw_data) || (0u == raw_data_length))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if ((node->ctx->device_cfg->device_type != SBEVDR_BLE_BROADCASTER) && (node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL))
        {
            SID_PAL_LOG_ERROR("Scan response data can be set for Broadcaster and Peripheral roles only. Virtual BLE device \"%s\" (0x%02X)", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id);
            err = SID_ERROR_NOSUPPORT;
            break;
        }

        /* Build scan response data */
        uint32_t sr_pos = 0u;

        /* Verify the supplied raw data fits into the buffer */
        if (raw_data_length > (sizeof(sr_bin_data) - sr_pos))
        {
            SID_PAL_LOG_ERROR("Supplied scan response data is too long. Virtual BLE device \"%s\" (0x%02X)", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id);
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Pre-validate raw data */
        uint32_t raw_data_pos = 0u;
        while (raw_data_pos < raw_data_length)
        {
            if (raw_data[raw_data_pos] < (SID_BLE_HCI_ADV_TYPE_RECORD_SIZE + SID_BLE_HCI_ADV_RECORD_SIZE_MIN))
            {
                /* That's invalid, this byte should keep the length of the scan response data record which is at least two bytes */
                raw_data_pos = UINT32_MAX;
                break;
            }

            /* Advance position by the length of the current advertising data record */
            raw_data_pos += (raw_data[raw_data_pos] + SID_BLE_HCI_ADV_RECORD_LENGTH_SIZE);
        }

        if (raw_data_pos != raw_data_length)
        {
            /* Mismatch between the supplied scan response data length and the actual computed length based on the scan response data format */
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Cleanup any previous data */
        SID_STM32_UTIL_fast_memset(sr_bin_data, 0u, sizeof(sr_bin_data));

        /* Copy user-supplied data into the buffer */
        SID_STM32_UTIL_fast_memcpy(&sr_bin_data[sr_pos], raw_data, raw_data_length);
        sr_pos += raw_data_length;

        SID_PAL_ASSERT(sr_pos <= MAX_ADV_DATA_LEN);

        /* Put scan response data into the host controller */
#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
        ble_status = hci_le_set_extended_scan_response_data(node->ctx->device_cfg->device_id,
                                                            HCI_SET_ADV_DATA_OPERATION_COMPLETE,
                                                            SID_BLE_HCI_SET_ADV_DATA_FRAGMENT_PREF_NO_FRAGMENT,
                                                            sizeof(sr_bin_data), (uint8_t *)sr_bin_data);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_USER_LOG_ERROR("==>> hci_le_set_extended_scan_response_data - fail, result: 0x%02X, set: 0x%02X", ble_status, node->ctx->device_cfg->device_id);
            err = SID_BLE_HCI_STATUS_UNKNOWN_ADV_IDENTIFIER == ble_status ? SID_ERROR_UNINITIALIZED : SID_ERROR_IO_ERROR; /* Advertisement set shall be created first via calling hci_le_set_extended_advertising_parameters */
            break;
        }
        SID_BLE_USER_LOG_DEBUG("==>> Success: hci_le_set_extended_scan_response_data, set: 0x%02X", node->ctx->device_cfg->device_id);
#else
        /* Use legacy scan response API */
        ble_status = hci_le_set_scan_response_data(sr_pos, (uint8_t*)sr_bin_data);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_USER_LOG_ERROR("==>> hci_le_set_scan_response_data - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_USER_LOG_DEBUG("==>> hci_le_set_scan_response_data - Success");
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_node_start_advertisement(sid_ble_ext_virtual_device_node_t * const node, const sid_ble_ext_adv_param_t * const adv_param)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        uint32_t                           adv_interval_min;
        uint32_t                           adv_interval_max;
        uint32_t                           adv_duration;
        sid_ble_ext_virtual_device_state_t targeted_state;
        sid_ble_ext_adv_state_change_t     adv_notify_event;

        /* Ensure the node is in the acceptable state for advertising */
        if ((node->ctx->state & (SBEVD_STATE_ADV_FAST | SBEVD_STATE_ADV_SLOW)) != 0u)
        {
            /* Node is advertising already */
            SID_BLE_USER_LOG_ERROR("Virtual BLE device \"%s\" (0x%02X) is advertising already", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id);
            err = SID_ERROR_INVALID_STATE;
            break;
        }
        else if (SBEVD_STATE_CONNECTED == node->ctx->state)
        {
            /* Node is connected, but if it allows multiple connection we can start advertising */
            if (SBEVDR_BLE_PERIPHERAL == node->ctx->device_cfg->device_type)
            {
                /* Check if the connection limit is reached */
                if (node->ctx->conn_cnt >= node->ctx->device_cfg->peripheral_cfg.max_conn)
                {
                    err = SID_ERROR_BUSY;
                    break;
                }
                else
                {
                    /* There are still free connection slots, we can proceed with advertising */
                }
            }
            else
            {
                /* Invalid node type */
                err = SID_ERROR_INVALID_STATE;
                break;
            }
        }
        else if (node->ctx->state != SBEVD_STATE_IDLE)
        {
            /* Node can be uninitialized, suspended, in error state, etc. Anyway, we can't proceed with advertising */
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Select advertising parameters to use */
        if (adv_param->fast_enabled != FALSE)
        {
            adv_interval_min = adv_param->fast_interval_min;
            adv_interval_max = adv_param->fast_interval_max;
            adv_duration     = adv_param->fast_timeout;
            targeted_state   = SBEVD_STATE_ADV_FAST;
            adv_notify_event = SBEASC_ADV_STARTED_FAST;
        }
        else if (adv_param->slow_enabled != FALSE)
        {
            adv_interval_min = adv_param->slow_interval_min;
            adv_interval_max = adv_param->slow_interval_max;
            adv_duration     = adv_param->slow_timeout;
            targeted_state   = SBEVD_STATE_ADV_SLOW;
            adv_notify_event = SBEASC_ADV_STARTED_SLOW;
        }
        else
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* (Re)initialize advertising parameters */
        err = _ble_adapter_user_set_adv_params(node->ctx, adv_interval_min, adv_interval_max);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to apply advertising params for virtual BLE device \"%s\" (0x%02X). Error %d", node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>", node->ctx->device_cfg->device_id, (int32_t)err);
            break;
        }

#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED
        /* Update random address if required */
        if (SID_BLE_CFG_MAC_ADDRESS_TYPE_PUBLIC != node->ctx->device_cfg->mac_addr_type)
        {
            sid_pal_ble_prv_bt_addr_buffer_t adv_bd_addr;

            switch (node->ctx->device_cfg->mac_addr_type)
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
            tBleStatus ble_status = hci_le_set_advertising_set_random_address(node->ctx->device_cfg->device_id, (uint8_t *)adv_bd_addr.bytes);
            if (ble_status != BLE_STATUS_SUCCESS)
            {
                SID_PAL_LOG_ERROR("==>> hci_le_set_advertising_set_random_address - fail, result: 0x%02X, set: 0x%02X", ble_status, node->ctx->device_cfg->device_id);
                err = SID_ERROR_IO_ERROR;
                break;
            }
            SID_BLE_USER_LOG_DEBUG("==>> hci_le_set_advertising_set_random_address - Success, set: 0x%02X", node->ctx->device_cfg->device_id);
            SID_BLE_USER_LOG_DEBUG("   Random Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x", adv_bd_addr.bytes[5],
                                                                                                 adv_bd_addr.bytes[4],
                                                                                                 adv_bd_addr.bytes[3],
                                                                                                 adv_bd_addr.bytes[2],
                                                                                                 adv_bd_addr.bytes[1],
                                                                                                 adv_bd_addr.bytes[0]);
        }
#else
        /* Update random address if required */
        if (SID_BLE_CFG_MAC_ADDRESS_TYPE_PUBLIC != node->ctx->device_cfg->mac_addr_type)
        {
            err = sid_stm32wba_ble_adapter_prv_rotate_random_mac_address(NULL, node->ctx->device_cfg->mac_addr_type);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("  Fail   : unable to rotate BLE random address, error %d", (int32_t)err);
                break;
            }
            SID_PAL_LOG_DEBUG("  Success: rotate_random_mac_address");
        }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        /* Build advertising set descriptor for HCI interface */
        Adv_Set_t hci_adv_set = {
            .Advertising_Handle              = node->ctx->device_cfg->device_id,
            .Duration                        = adv_duration,
            .Max_Extended_Advertising_Events = 0u,
        };

        /* Start advertising */
        err = sid_stm32wba_ble_adapter_prv_generic_start_advertisement(&hci_adv_set);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by sid_stm32wba_ble_adapter_prv_generic_start_advertisement */
            break;
        }

        /* Indicate that we are advertising */
        node->ctx->state = targeted_state;

        /* Notify user about the advertising start */
        sid_ble_ext_on_adv_state_changed_cb_t on_adv_state_changed;
        if (SBEVDR_BLE_PERIPHERAL == node->ctx->device_cfg->device_type)
        {
            on_adv_state_changed = node->ctx->device_cfg->peripheral_cfg.callbacks.on_adv_state_changed;
        }
        else if (SBEVDR_BLE_BROADCASTER == node->ctx->device_cfg->device_type)
        {
            on_adv_state_changed = node->ctx->device_cfg->broadcaster_cfg.callbacks.on_adv_state_changed;
        }
        else
        {
            on_adv_state_changed = NULL;
        }

        if (on_adv_state_changed != NULL)
        {
            on_adv_state_changed(node->ctx, adv_notify_event);
        }

        SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X) started advertising in %s mode",
                         node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>",
                         node->ctx->device_cfg->device_id,
                         SBEVD_STATE_ADV_FAST == targeted_state ? "fast" : "slow");
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _ble_adapter_user_node_stop_advertisement(sid_ble_ext_virtual_device_node_t * const node)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Ensure the node is in the acceptable state for advertising */
        if ((node->ctx->state < SBEVD_STATE_FATAL_ERROR) && (node->ctx->state >= SBEVD_STATE_IDLE) && ((node->ctx->state & (SBEVD_STATE_ADV_FAST | SBEVD_STATE_ADV_SLOW)) == 0u))
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Stop advertising */
        err = sid_stm32wba_ble_adapter_prv_generic_stop_advertisement(node->ctx->device_cfg->device_id);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Advertisement is stopped */
        if ((node->ctx->state < SBEVD_STATE_FATAL_ERROR) && (node->ctx->state >= SBEVD_STATE_IDLE))
        {
            /* Clear advertising status bits */
            node->ctx->state &= ~(SBEVD_STATE_ADV_FAST | SBEVD_STATE_ADV_SLOW);

            if ((node->ctx->state & SBEVD_STATE_CONNECTED) == 0u)
            {
                node->ctx->state = SBEVD_STATE_IDLE;
            }
        }

        sid_ble_ext_on_adv_state_changed_cb_t on_adv_state_changed;
        if (SBEVDR_BLE_PERIPHERAL == node->ctx->device_cfg->device_type)
        {
            on_adv_state_changed = node->ctx->device_cfg->peripheral_cfg.callbacks.on_adv_state_changed;
        }
        else if (SBEVDR_BLE_BROADCASTER == node->ctx->device_cfg->device_type)
        {
            on_adv_state_changed = node->ctx->device_cfg->broadcaster_cfg.callbacks.on_adv_state_changed;
        }
        else
        {
            on_adv_state_changed = NULL;
        }

        if ((on_adv_state_changed != NULL) && (node->ctx->state < SBEVD_STATE_FATAL_ERROR) && (node->ctx->state >= SBEVD_STATE_IDLE))
        {
            on_adv_state_changed(node->ctx, SBEASC_ADV_SWITCHED_TERMINATED_ON_REQUEST);
        }

        if ((node->ctx->state < SBEVD_STATE_FATAL_ERROR) && (node->ctx->state >= SBEVD_STATE_IDLE))
        {
            SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X) stopped advertising on request",
                             node->ctx->device_cfg->device_name != NULL ? node->ctx->device_cfg->device_name : "<unnamed>",
                             node->ctx->device_cfg->device_id);
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_node_disconnect(sid_ble_ext_virtual_device_node_t * const node, sid_ble_ext_conn_list_node_t * const conn_node)
{
    sid_error_t                        err = SID_ERROR_GENERIC;
    tBleStatus                         ble_status;
    uint32_t                           ll_lock_was_acquired;
    sid_pal_ble_prv_connection_ctx_t * prv_conn_ctx = NULL;

    /* Check if LL is locked by the current thread */
    if (osThreadGetId() == osMutexGetOwner(LinkLayerMutex))
    {
        /* Yes, we will have temporarily release the LL mutex */
        ll_lock_was_acquired = TRUE;
    }
    else
    {
        /* Nope, we don't have to deal with the LL mutex */
        ll_lock_was_acquired = FALSE;
    }

    do
    {
        /* Validate state */
        if ((NULL == node) || (NULL == conn_node))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        if (SID_STM32_BLE_HANDLE_INVALID_VALUE == conn_node->ctx->conn_id)
        {
            /* Already disconnected */
            err = SID_ERROR_PORT_NOT_OPEN;
            break;
        }

        /* Prepare to wait for disconnection completion */
        prv_conn_ctx = containerof(conn_node->ctx, sid_pal_ble_prv_connection_ctx_t, public_ctx);
        sid_stm32wba_ble_adapter_prv_gap_cmd_arm_resp_wait(prv_conn_ctx);

        if (ll_lock_was_acquired != FALSE)
        {
            /* Temporarily release mutex to allow BLE stack to process disconnect */
            (void)osMutexRelease(LinkLayerMutex);
        }
        __COMPILER_BARRIER();

        /* Initiate disconnect */
        ble_status = aci_gap_terminate(conn_node->ctx->conn_id, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_USER_LOG_ERROR("==>> aci_gap_terminate - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_USER_LOG_DEBUG("==>> aci_gap_terminate - Success");

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    if (SID_ERROR_NONE == err)
    {
        /* Wait for disconnect to complete */
        err = sid_stm32wba_ble_adapter_prv_gap_cmd_resp_wait(prv_conn_ctx);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to wait for BLE disconnect completion for connection 0x%04X. Error %d", conn_node->ctx->conn_id, (int32_t)err);
        }

        /* Re-acquire LL mutex if it was previously acquired by this thread */
        if (ll_lock_was_acquired != FALSE)
        {
            (void)osMutexAcquire(LinkLayerMutex, osWaitForever);
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

SID_STM32_SPEED_OPTIMIZED static SVCCTL_EvtAckStatus_t _ble_adapter_user_gatt_event_handler(void * p_Event)
{
    SVCCTL_EvtAckStatus_t evt_ack_status = SVCCTL_EvtNotAck;

    do
    {
        const hci_event_pckt * const p_event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)p_Event)->data);
        const evt_blecore_aci *      p_blecore_evt;
        tBleStatus                   ble_status;

        switch(p_event_pckt->evt)
        {
            case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
                p_blecore_evt = (evt_blecore_aci*)p_event_pckt->data;

                switch(p_blecore_evt->ecode)
                {
                    case ACI_GATT_PREPARE_WRITE_PERMIT_REQ_VSEVT_CODE:
                        {
                            const aci_gatt_prepare_write_permit_req_event_rp0 * const p_prepare_write_permit_req = (aci_gatt_prepare_write_permit_req_event_rp0 *)p_blecore_evt->data;
                            uint8_t resp_error_code;
                            sid_ble_ext_virtual_device_node_t       * dev_node;
                            sid_ble_ext_conn_list_node_t            * conn_node;
                            sid_ble_ext_gatt_server_svc_ctx_t       * svc_ctx;
                            sid_ble_ext_gatt_server_char_ctx_t      * char_ctx;
                            sid_ble_ext_gatt_server_char_desc_ctx_t * desc_ctx;

                            evt_ack_status = _ble_adapter_user_process_gatt_attr_rw_permit_req(p_prepare_write_permit_req->Connection_Handle, p_prepare_write_permit_req->Attribute_Handle, &resp_error_code, SBEAAT_WRITE_ACCESS, (p_prepare_write_permit_req->Offset + p_prepare_write_permit_req->Data_Length),
                                                                                               &dev_node, &conn_node, &svc_ctx, &char_ctx, &desc_ctx);

                            if (SVCCTL_EvtAckFlowEnable == evt_ack_status)
                            {
                                ble_status = aci_gatt_write_resp(p_prepare_write_permit_req->Connection_Handle, p_prepare_write_permit_req->Attribute_Handle,
                                                                 0x00u /* value can be written */, SID_STM32_BLE_ATT_STATUS_SUCCESS,
                                                                 p_prepare_write_permit_req->Data_Length, p_prepare_write_permit_req->Data);
                                if (ble_status != BLE_STATUS_SUCCESS)
                                {
                                    SID_PAL_LOG_ERROR("Failed to set BLE write response for connection 0x%04X, attribute 0x%04X. ACI error 0x%02X", p_prepare_write_permit_req->Connection_Handle, p_prepare_write_permit_req->Attribute_Handle, ble_status);
                                    (void)aci_gap_terminate(p_prepare_write_permit_req->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
                                    break;
                                }
                            }
                            else if (SVCCTL_EvtAckFlowDisable == evt_ack_status)
                            {
                                SID_PAL_LOG_WARNING("Rejected write request for BLE connection 0x%04X, attribute 0x%04X. Reason code 0x%02X", p_prepare_write_permit_req->Connection_Handle, p_prepare_write_permit_req->Attribute_Handle, resp_error_code);

                                ble_status = aci_gatt_write_resp(p_prepare_write_permit_req->Connection_Handle, p_prepare_write_permit_req->Attribute_Handle,
                                                                 0x01u /* write req rejected */, resp_error_code,
                                                                 p_prepare_write_permit_req->Data_Length, p_prepare_write_permit_req->Data);
                                if (ble_status != BLE_STATUS_SUCCESS)
                                {
                                    SID_PAL_LOG_ERROR("Failed to set BLE write response for connection 0x%04X, attribute 0x%04X. ACI error 0x%02X", p_prepare_write_permit_req->Connection_Handle, p_prepare_write_permit_req->Attribute_Handle, ble_status);
                                    (void)aci_gap_terminate(p_prepare_write_permit_req->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
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

                    case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
                        {
                            const aci_gatt_write_permit_req_event_rp0 * const p_write_permit_req = (aci_gatt_write_permit_req_event_rp0 *)p_blecore_evt->data;
                            uint8_t resp_error_code;
                            sid_ble_ext_virtual_device_node_t       * dev_node;
                            sid_ble_ext_conn_list_node_t            * conn_node;
                            sid_ble_ext_gatt_server_svc_ctx_t       * svc_ctx;
                            sid_ble_ext_gatt_server_char_ctx_t      * char_ctx;
                            sid_ble_ext_gatt_server_char_desc_ctx_t * desc_ctx;

                            /* Do not process access to the General Attribute Service, this is handled by the generic part of the driver */
                            if (p_write_permit_req->Attribute_Handle == (SID_STM32_BLE_GATT_SVC_SERVICE_CHANGED_CHAR_HANDLE + SID_STM32_BLE_CHARACTERISTIC_CCCD_ATTRIBUTE_OFFSET))
                            {
                                break;
                            }

                            evt_ack_status = _ble_adapter_user_process_gatt_attr_rw_permit_req(p_write_permit_req->Connection_Handle, p_write_permit_req->Attribute_Handle, &resp_error_code, SBEAAT_WRITE_ACCESS, p_write_permit_req->Data_Length,
                                                                                               &dev_node, &conn_node, &svc_ctx, &char_ctx, &desc_ctx);

                            if (SVCCTL_EvtAckFlowEnable == evt_ack_status)
                            {
                                ble_status = aci_gatt_write_resp(p_write_permit_req->Connection_Handle, p_write_permit_req->Attribute_Handle,
                                                                 0x00u /* value can be written */, SID_STM32_BLE_ATT_STATUS_SUCCESS,
                                                                 p_write_permit_req->Data_Length, p_write_permit_req->Data);
                                if (ble_status != BLE_STATUS_SUCCESS)
                                {
                                    SID_PAL_LOG_ERROR("Failed to set BLE write response for connection 0x%04X, attribute 0x%04X. ACI error 0x%02X", p_write_permit_req->Connection_Handle, p_write_permit_req->Attribute_Handle, ble_status);
                                    (void)aci_gap_terminate(p_write_permit_req->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
                                    break;
                                }
                            }
                            else if (SVCCTL_EvtAckFlowDisable == evt_ack_status)
                            {
                                SID_PAL_LOG_WARNING("Rejected write request for BLE connection 0x%04X, attribute 0x%04X. Reason code 0x%02X", p_write_permit_req->Connection_Handle, p_write_permit_req->Attribute_Handle, resp_error_code);

                                ble_status = aci_gatt_write_resp(p_write_permit_req->Connection_Handle, p_write_permit_req->Attribute_Handle,
                                                                 0x01u /* write req rejected */, resp_error_code,
                                                                 p_write_permit_req->Data_Length, p_write_permit_req->Data);
                                if (ble_status != BLE_STATUS_SUCCESS)
                                {
                                    SID_PAL_LOG_ERROR("Failed to set BLE write response for connection 0x%04X, attribute 0x%04X. ACI error 0x%02X", p_write_permit_req->Connection_Handle, p_write_permit_req->Attribute_Handle, ble_status);
                                    (void)aci_gap_terminate(p_write_permit_req->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
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

                    case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE:
                        {
                            const aci_gatt_read_permit_req_event_rp0 * const p_read_permit_req = (aci_gatt_read_permit_req_event_rp0 *)p_blecore_evt->data;
                            uint8_t resp_error_code;
                            sid_ble_ext_virtual_device_node_t       * dev_node;
                            sid_ble_ext_conn_list_node_t            * conn_node;
                            sid_ble_ext_gatt_server_svc_ctx_t       * svc_ctx;
                            sid_ble_ext_gatt_server_char_ctx_t      * char_ctx;
                            sid_ble_ext_gatt_server_char_desc_ctx_t * desc_ctx;

                            evt_ack_status = _ble_adapter_user_process_gatt_attr_rw_permit_req(p_read_permit_req->Connection_Handle, p_read_permit_req->Attribute_Handle, &resp_error_code, SBEAAT_READ_ACCESS, (p_read_permit_req->Offset + 1u),
                                                                                               &dev_node, &conn_node, &svc_ctx, &char_ctx, &desc_ctx);

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
                            uint8_t resp_error_code;

                            for (uint32_t i = 0u; i < p_read_multi_permit_req->Number_of_Handles; i++)
                            {
                                sid_ble_ext_virtual_device_node_t       * dev_node;
                                sid_ble_ext_conn_list_node_t            * conn_node;
                                sid_ble_ext_gatt_server_svc_ctx_t       * svc_ctx;
                                sid_ble_ext_gatt_server_char_ctx_t      * char_ctx;
                                sid_ble_ext_gatt_server_char_desc_ctx_t * desc_ctx;

                                evt_ack_status = _ble_adapter_user_process_gatt_attr_rw_permit_req(p_read_multi_permit_req->Connection_Handle, p_read_multi_permit_req->Handle_Item[i].Handle, &resp_error_code, SBEAAT_READ_ACCESS, 0u,
                                                                                                   &dev_node, &conn_node, &svc_ctx, &char_ctx, &desc_ctx);

                                /* Jump out if this connection handle is not related to the User BLE mode or if the access is explicitly prohibited */
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

                    case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
                        {
                            const aci_gatt_attribute_modified_event_rp0 * const p_attribute_modified = (aci_gatt_attribute_modified_event_rp0 *)p_blecore_evt->data;

                            /* Do not process access to the General Attribute Service, this is handled by the generic part of the driver */
                            if (p_attribute_modified->Attr_Handle == (SID_STM32_BLE_GATT_SVC_SERVICE_CHANGED_CHAR_HANDLE + SID_STM32_BLE_CHARACTERISTIC_CCCD_ATTRIBUTE_OFFSET))
                            {
                                break;
                            }

                            evt_ack_status = _ble_adapter_user_process_gatt_attr_modified_evt(p_attribute_modified);

                            if (SVCCTL_EvtAckFlowDisable == evt_ack_status)
                            {
                                (void)aci_gap_terminate(p_attribute_modified->Connection_Handle, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);

                                /* Done with processing of this event */
                                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                            }
                        }
                        break;

                    default:
                        /* Allow other handlers to process this event */
                        break;
                }
                break;

#if SID_STM32_BLE_DLE_ENABLE
            case HCI_LE_META_EVT_CODE:
                evt_ack_status = _ble_adapter_user_process_hci_le_meta_evt((evt_le_meta_event *)p_event_pckt->data);
                break;
#endif /* SID_STM32_BLE_DLE_ENABLE */

            default:
                /* Allow other handlers to process this event */
                break;
        }
    } while (0);

    return evt_ack_status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline SVCCTL_EvtAckStatus_t _ble_adapter_user_process_gatt_attr_rw_permit_req(const uint16_t conn_handle, const uint16_t att_handle, uint8_t * const out_att_status_code, const sid_ble_ext_att_access_type_t access_mode, const uint32_t full_data_length,
                                                                                                                sid_ble_ext_virtual_device_node_t ** const out_dev_node ,sid_ble_ext_conn_list_node_t ** const out_conn_node,
                                                                                                                sid_ble_ext_gatt_server_svc_ctx_t ** const out_svc_ctx, sid_ble_ext_gatt_server_char_ctx_t ** const out_char_ctx, sid_ble_ext_gatt_server_char_desc_ctx_t ** const out_desc_ctx)
{
    SVCCTL_EvtAckStatus_t evt_ack_status;

    do
    {
        *out_dev_node  = NULL;
        *out_conn_node = NULL;
        *out_svc_ctx   = NULL;
        *out_char_ctx  = NULL;
        *out_desc_ctx  = NULL;

        /* Locate associated virtual device node */
        *out_dev_node = _ble_adapter_user_search_device_node_by_conn_handle(conn_handle);
        if (NULL == *out_dev_node)
        {
            /* No associated virtual device found -> this connection does not belong to the user mode */
            evt_ack_status = SVCCTL_EvtNotAck;
            break;
        }

        if ((*out_dev_node)->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL)
        {
            /* Normally should not happen, this event is relevant for Peripherals only */
            *out_att_status_code = SID_STM32_BLE_ATT_STATUS_APP_ERROR_INVALID_STATE;
            evt_ack_status = SVCCTL_EvtAckFlowDisable;
            break;
        }

        /* Locate associated connection node */
        *out_conn_node = _ble_adapter_user_search_connection_node_by_conn_handle((*out_dev_node)->ctx, conn_handle);
        if (NULL == *out_conn_node)
        {
            /* Normally should not happen */
            *out_att_status_code = SID_STM32_BLE_ATT_STATUS_APP_ERROR_INVALID_STATE;
            evt_ack_status = SVCCTL_EvtAckFlowDisable;
            break;
        }

        /* Handle GAP Device Name characteristic access */
        if ((SBEAAT_READ_ACCESS == access_mode) && (att_handle == (sid_ble_drv_ctx.gap_dev_name_char_handle + SID_STM32_BLE_CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)))
        {
            tBleStatus ble_status;
            const char * const gap_device_name = (*out_dev_node)->ctx->device_cfg->device_name;
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
                SID_BLE_USER_LOG_ERROR("  Fail   : aci_gatt_update_char_value - Device Name, result: 0x%02X", ble_status);
                *out_att_status_code = SID_STM32_BLE_ATT_STATUS_APP_ERROR_GENERIC;
                evt_ack_status = SVCCTL_EvtAckFlowDisable;
                break;
            }
            else
            {
                /* Successfully updated GAP Device Name characterstic with the name of the current Virtual BLE device */
                SID_BLE_USER_LOG_DEBUG("  Success: aci_gatt_update_char_value - Device Name");
                *out_att_status_code = SID_STM32_BLE_ATT_STATUS_SUCCESS;
                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                break; /* Exit from here since we don't need to invoke user callback for GAP characteristics */
            }
        }

        /* Handle GAP Appearance characteristic access */
        if ((SBEAAT_READ_ACCESS == access_mode) && (att_handle == (sid_ble_drv_ctx.gap_appearance_char_handle + SID_STM32_BLE_CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)))
        {
            tBleStatus ble_status;
            const uint16_t gap_appearance = (*out_dev_node)->ctx->device_cfg->peripheral_cfg.appearance;

            /* Update GAP appearance */
            ble_status = aci_gatt_update_char_value(sid_ble_drv_ctx.gap_service_handle,
                                                    sid_ble_drv_ctx.gap_appearance_char_handle,
                                                    0u,
                                                    sizeof(gap_appearance),
                                                    (uint8_t *)(void *)&gap_appearance);
            if (ble_status != BLE_STATUS_SUCCESS)
            {
                SID_BLE_USER_LOG_ERROR("  Fail   : aci_gatt_update_char_value - Appearance, result: 0x%02X", ble_status);
                *out_att_status_code = SID_STM32_BLE_ATT_STATUS_APP_ERROR_GENERIC;
                evt_ack_status = SVCCTL_EvtAckFlowDisable;
                break;
            }
            else
            {
                SID_BLE_USER_LOG_DEBUG("  Success: aci_gatt_update_char_value - Appearance");
                *out_att_status_code = SID_STM32_BLE_ATT_STATUS_SUCCESS;
                evt_ack_status = SVCCTL_EvtAckFlowEnable;
                break; /* Exit from here since we don't need to invoke user callback for GAP characteristics */
            }
        }

        /* Handle GAP Preferred Connection Parameters characteristic access */
        if ((SBEAAT_READ_ACCESS == access_mode) && (att_handle == (sid_ble_drv_ctx.gap_ppcp_char_handle + SID_STM32_BLE_CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET)))
        {
            const sid_ble_cfg_conn_param_t * const conn_param = (*out_dev_node)->ctx->device_cfg->peripheral_cfg.conn_param;

            if (conn_param != NULL)
            {
                (void)sid_stm32wba_ble_adapter_util_set_gap_ppcp_char(conn_param);
            }

            *out_att_status_code = SID_STM32_BLE_ATT_STATUS_SUCCESS;
            evt_ack_status = SVCCTL_EvtAckFlowEnable;
            break; /* Exit from here since we don't need to invoke user callback for GAP characteristics */
        }

        /* Locate associated attribute context */
        _ble_adapter_user_search_att_ctx_by_handle(out_svc_ctx, out_char_ctx, out_desc_ctx, (*out_dev_node)->ctx, att_handle);

        if ((NULL == *out_svc_ctx) || (NULL == *out_char_ctx))
        {
            /* Corresponding attribute was not found - reject the request. This can happen if the attribute belongs to the virtual device A and access is performed by the virtual device B */
            *out_att_status_code = SID_STM32_BLE_ATT_STATUS_APP_ERROR_WRONG_VIRT_DEV;
            evt_ack_status = SVCCTL_EvtAckFlowDisable;
            break;
        }

        /* Ensure the operation is not exceeding the limits */
        uint32_t data_len_limit;
        if (*out_desc_ctx != NULL)
        {
            /* This attribute is characteristic descriptor, select corresponding length limit */
            data_len_limit = (*out_desc_ctx)->desc_def->max_length;
        }
        else
        {
            /* This attribute is characteristic value, select corresponding length limit */
            data_len_limit = (*out_char_ctx)->char_def->max_length;
        }
        
        if (full_data_length > data_len_limit)
        {
            /* Too much data -> reject request to avoid buffer overflow */
            *out_att_status_code = SID_STM32_BLE_ATT_STATUS_APP_ERROR_EXCEEDS_LIMIT;
            evt_ack_status = SVCCTL_EvtAckFlowDisable;
            break;
        }

        /* Invoke user callback to decide if access permission should be granted */
        if ((*out_dev_node)->ctx->device_cfg->peripheral_cfg.callbacks.on_att_access_request != NULL)
        {
            *out_att_status_code = (*out_dev_node)->ctx->device_cfg->peripheral_cfg.callbacks.on_att_access_request((*out_dev_node)->ctx, (*out_conn_node)->ctx, *out_svc_ctx, *out_char_ctx, *out_desc_ctx, access_mode);
            if (SID_STM32_BLE_ATT_STATUS_SUCCESS == *out_att_status_code)
            {
                evt_ack_status = SVCCTL_EvtAckFlowEnable;
            }
            else
            {
                evt_ack_status = SVCCTL_EvtAckFlowDisable;

                /* Ensure the response code is within the valid range for the read request */
                if (SBEAAT_READ_ACCESS == access_mode)
                {
                    if ((*out_att_status_code != SID_STM32_BLE_ATT_STATUS_INSUFFICIENT_AUTHOR) && ((*out_att_status_code < SID_STM32_BLE_ATT_STATUS_APP_ERROR_MIN) || (*out_att_status_code > SID_STM32_BLE_ATT_STATUS_APP_ERROR_MAX)))
                    {
                        SID_PAL_LOG_WARNING("Invalid ATT read reply status code - 0x%02X. Connection 0x%04X, attribute 0x%04X", *out_att_status_code, conn_handle, att_handle);
                        *out_att_status_code = SID_STM32_BLE_ATT_STATUS_APP_ERROR_GENERIC;
                    }
                }
            }
        }
        else
        {
            /* Grant permission if no user callback is set */
            *out_att_status_code = SID_STM32_BLE_ATT_STATUS_SUCCESS;
            evt_ack_status = SVCCTL_EvtAckFlowEnable;
        }
    } while (0);

    return evt_ack_status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline SVCCTL_EvtAckStatus_t _ble_adapter_user_process_gatt_attr_modified_evt(const aci_gatt_attribute_modified_event_rp0 * const evt)
{
    SVCCTL_EvtAckStatus_t evt_ack_status = SVCCTL_EvtNotAck;

    do
    {
        sid_ble_ext_virtual_device_node_t       * dev_node;
        sid_ble_ext_conn_list_node_t            * conn_node;
        sid_ble_ext_gatt_server_svc_ctx_t       * svc_ctx;
        sid_ble_ext_gatt_server_char_ctx_t      * char_ctx;
        sid_ble_ext_gatt_server_char_desc_ctx_t * desc_ctx;

        /* Locate associated virtual device node */
        dev_node = _ble_adapter_user_search_device_node_by_conn_handle(evt->Connection_Handle);
        if (NULL == dev_node)
        {
            /* No associated virtual device found -> this connection does not belong to the user mode */
            evt_ack_status = SVCCTL_EvtNotAck;
            break;
        }

        if (dev_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL)
        {
            /* Normally should not happen, this event is relevant for Peripherals only */
            evt_ack_status = SVCCTL_EvtAckFlowDisable;
            break;
        }

        /* Locate associated connection node */
        conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(dev_node->ctx, evt->Connection_Handle);
        if (NULL == conn_node)
        {
            /* Normally should not happen */
            evt_ack_status = SVCCTL_EvtAckFlowDisable;
            break;
        }

        /* Locate associated attribute context */
        _ble_adapter_user_search_att_ctx_by_handle(&svc_ctx, &char_ctx, &desc_ctx, dev_node->ctx, evt->Attr_Handle);

        if ((NULL == svc_ctx) || (NULL == char_ctx))
        {
            /* Corresponding attribute was not found - reject the request. This can happen if the attribute belongs to the virtual device A and access is performed by the virtual device B */
            evt_ack_status = SVCCTL_EvtAckFlowDisable;
            break;
        }

        /* Ensure the operation is not exceeding the limits */
        uint8_t  * rx_buf_ptr;
        uint16_t * valid_len_ptr;
        uint16_t * accum_len_ptr;
        uint32_t   data_len_limit;
        if (desc_ctx != NULL)
        {
            /* This attribute is characteristic descriptor, select corresponding length limit */
            rx_buf_ptr     = desc_ctx->data;
            valid_len_ptr  = &desc_ctx->data_length;
            accum_len_ptr  = &desc_ctx->upd_accum_len;
            data_len_limit = desc_ctx->desc_def->max_length;
        }
        else
        {
            /* This attribute is characteristic value, select corresponding length limit */
            rx_buf_ptr     = char_ctx->data;
            valid_len_ptr  = &char_ctx->data_length;
            accum_len_ptr  = &char_ctx->upd_accum_len;
            data_len_limit = char_ctx->char_def->max_length;
        }

        /* Ensure there are no systematic SW failures */
        SID_PAL_ASSERT(rx_buf_ptr != NULL);

        /* Check the received data does not exceed the receiving buffer size */
        const uint32_t store_offset     = (evt->Offset & (~SID_STM32_BLE_ACI_GATT_ATTRIBUTE_MODIFIED_MORE_DATA_PENDING_FLAG));
        const uint32_t more_data_pending = (evt->Offset & SID_STM32_BLE_ACI_GATT_ATTRIBUTE_MODIFIED_MORE_DATA_PENDING_FLAG) != 0u;

        /* Check the received data does not exceed the receiving buffer size */
        if ((store_offset + evt->Attr_Data_Length) > data_len_limit)
        {
            if (FALSE == more_data_pending) /* Print out the warning only once */
            {
                SID_PAL_LOG_WARNING("Received GATT attribute data exceeds processing buffer size (%u vs %u bytes). Data discarded", (store_offset + evt->Attr_Data_Length), data_len_limit);
            }
            evt_ack_status = SVCCTL_EvtAckFlowEnable;
            break;
        }

        SID_BLE_USER_LOG_DEBUG("==>>ATT received %u bytes%s", evt->Attr_Data_Length, more_data_pending ? ". More data pending" : "");
        SID_STM32_UTIL_fast_memcpy(&rx_buf_ptr[store_offset], evt->Attr_Data, evt->Attr_Data_Length);
        *accum_len_ptr += evt->Attr_Data_Length;

        /* Invoke user callback if all the data read out from the BLE stack */
        if (more_data_pending == FALSE)
        {
            SID_BLE_USER_LOG_DEBUG("==>>ATT: RX finished, totally received %u bytes", *accum_len_ptr);

            /* Reset the data accumulator offset */
            *valid_len_ptr = *accum_len_ptr;
            *accum_len_ptr = 0u;

            /* Data received from the GATT client in full - call the user callback */
            if ((desc_ctx != NULL) && (_ble_adapter_user_is_desc_cccd(desc_ctx) != FALSE))
            {
                /* This attribute is CCCD of a characteristic - notify the user about CCCD change */
                if (dev_node->ctx->device_cfg->peripheral_cfg.callbacks.on_cccd_changed != NULL)
                {
                    const sid_ble_ext_cccd_val_t * const cccd_val_ptr = (sid_ble_ext_cccd_val_t *)rx_buf_ptr;
                    dev_node->ctx->device_cfg->peripheral_cfg.callbacks.on_cccd_changed(dev_node->ctx, conn_node->ctx, svc_ctx, char_ctx, *cccd_val_ptr);
                }
            }
            else if (dev_node->ctx->device_cfg->peripheral_cfg.callbacks.on_data_received != NULL)
            {
                /* This just an ordinary attribute modification - call Data Received callback */
                dev_node->ctx->device_cfg->peripheral_cfg.callbacks.on_data_received(dev_node->ctx, conn_node->ctx, svc_ctx, char_ctx, desc_ctx, rx_buf_ptr, *valid_len_ptr);
            }
            else
            {
                /* Nothing to do */
            }
        }
        else
        {
            /* Indicate there's no valid data till we receive full update */
            *valid_len_ptr = 0u;
        }

        /* This event is processed, no further actions are required */
        evt_ack_status = SVCCTL_EvtAckFlowEnable;
    } while (0);

    return evt_ack_status;
}

/*----------------------------------------------------------------------------*/

#if SID_STM32_BLE_DLE_ENABLE
SID_STM32_SPEED_OPTIMIZED static inline SVCCTL_EvtAckStatus_t _ble_adapter_user_process_hci_le_meta_evt(const evt_le_meta_event * const evt)
{
    SVCCTL_EvtAckStatus_t evt_ack_status = SVCCTL_EvtNotAck;

    do
    {
        switch (evt->subevent)
        {
            case HCI_LE_DATA_LENGTH_CHANGE_SUBEVT_CODE:
                {
                    const hci_le_data_length_change_event_rp0 * const p_le_data_length_change = (hci_le_data_length_change_event_rp0 *)evt->data;
                    sid_ble_ext_virtual_device_node_t * const dev_node = _ble_adapter_user_search_device_node_by_conn_handle(p_le_data_length_change->Connection_Handle);

                    if (dev_node != NULL)
                    {
                        const sid_ble_ext_virtual_device_t * const virt_dev_cfg = dev_node->ctx->device_cfg;
                        SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X) DLE: MaxTxOctets: %u, MaxTxTime: %uus, MaxRxOctets: %u, MaxRxTime: %uus",
                                         virt_dev_cfg->device_name != NULL ? virt_dev_cfg->device_name : "<unnamed>",
                                         virt_dev_cfg->device_id,
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
#endif /* SID_STM32_BLE_DLE_ENABLE */

/* Private shared function definitions ---------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_init(void)
{
    sid_error_t err = SID_ERROR_GENERIC;
    uint32_t    ll_lock_acquired;
    const sid_pal_ble_prv_operating_mode_t init_type =
#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY)
                                                  SPBP_OPERATING_MODE_CONCURRENT;
#else
                                                  SPBP_OPERATING_MODE_USER;
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_CONCURRENCY */

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_init function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible during User BLE init");

    do
    {
        sid_pal_enter_critical_region();

#if (SID_STM32_BLE_COEXISTENCE_MODE == SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING)
        if (sid_ble_drv_ctx.operating_mode != SPBP_OPERATING_MODE_OFF)
        {
            SID_PAL_LOG_ERROR("User BLE mode cannot be initialized because BLE radio is busy with another mode (%u)", sid_ble_drv_ctx.operating_mode);
            err = SID_ERROR_BUSY;
            sid_pal_exit_critical_region();
            break;
        }
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_INTERLEAVING */

        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_UNINITIALIZED)
        {
            /* Either User mode is activated already or the driver is in a transitional state, the user must wait till it reaches a stable state */
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        sid_ble_user_mode_ctx.state = SPBUM_STATE_INITIALIZING;
        __COMPILER_BARRIER();

        sid_pal_exit_critical_region();

        /* Obtain Sidewalk BLE config - it is required to compute the common BLE stack settings properly */
        const sid_ble_link_config_t * const sid_cfg = app_get_sidewalk_ble_config();
        SID_PAL_ASSERT(sid_cfg != NULL);

        err = sid_stm32wba_ble_adapter_prv_generic_init(init_type, sid_cfg->config);
        if (err != SID_ERROR_NONE)
        {
            /* Logs are provided by sid_stm32wba_ble_adapter_prv_generic_init() */
            break;
        }

        /* Create list of active virtual device contexts */
        sid_pal_enter_critical_region();
        LST_init_head((tListNode *)&active_virt_devs_list);
        sid_pal_exit_critical_region();

        /* Register GATT event handler for user mode */
        err = sid_stm32wba_ble_adapter_prv_generic_register_svc_handler(_ble_adapter_user_gatt_event_handler);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to register GATT handler for user-mode BLE. Error %d", (int32_t)err);
            break;
        }

        SID_PAL_LOG_INFO("User BLE init done");
        sid_ble_user_mode_ctx.state = SPBUM_STATE_INITIALIZED;
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_init function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_deinit(void)
{
    sid_error_t err;
    sid_error_t gatt_deinit_err;
    sid_error_t rtos_resource_deinit_err;
    sid_error_t generic_deinit_err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_deinit function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible during User BLE deinit");

    do
    {
        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        sid_ble_user_mode_ctx.state = SPBUM_STATE_DEINITIALIZING;
        __COMPILER_BARRIER();

        sid_pal_exit_critical_region();

        /*----------------------------------------------------------------------------*/

        /* Iterate over the list of active devices and terminate every of them */
        sid_ble_ext_virtual_device_node_t * current_node = (sid_ble_ext_virtual_device_node_t *)(((tListNode *)&active_virt_devs_list)->next);
        while (current_node != &active_virt_devs_list)
        {
            /* Kill the node */
            (void)_ble_adapter_user_terminate_device_node(current_node);

            /* Advance to the next node */
            current_node = (sid_ble_ext_virtual_device_node_t *)(((tListNode *)&active_virt_devs_list)->next);
        }

        /* De-register GATT event handler */
        gatt_deinit_err = sid_stm32wba_ble_adapter_prv_generic_deregister_svc_handler(_ble_adapter_user_gatt_event_handler);

        rtos_resource_deinit_err = SID_ERROR_NONE;

        /*----------------------------------------------------------------------------*/

        generic_deinit_err = sid_stm32wba_ble_adapter_prv_generic_deinit();
        /* Logs provided by sid_stm32wba_ble_adapter_prv_generic_deinit */

        /**
         * Select error with following priorities (highest to lowest)
         * 1. Generic deinitialization errors
         * 2. BLE GATT deinit errors
         * 3. RTOS resource deallocation errors
         */
        err = generic_deinit_err != SID_ERROR_NONE ? generic_deinit_err : (gatt_deinit_err != SID_ERROR_NONE ? gatt_deinit_err : rtos_resource_deinit_err);
    } while (0);

    if (SID_ERROR_NONE == err)
    {
        sid_ble_user_mode_ctx.state = SPBUM_STATE_UNINITIALIZED;
    }

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_deinit function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_factory_reset(void)
{
    sid_error_t err;

    do
    {
        /* Ensure BLE is non-functional at the moment */
        if (sid_ble_drv_ctx.operating_mode != SPBP_OPERATING_MODE_OFF)
        {
            SID_PAL_LOG_ERROR("BLE Factory Reset cannot be performed because BLE driver is initialized. Oeprating mode %u", sid_ble_drv_ctx.operating_mode);
            err = SID_ERROR_INVALID_STATE;
            break;
        }

#if SID_STM32_BLE_CLEAR_CFG_ON_FACTORY_RESET
        /* Clear config data in the KV storage */
        err = sid_pal_storage_kv_group_delete(STORAGE_KV_SID_BLE_ADAPTER_CONFIG_GROUP);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("  Fail   : Unable to delete sid ble adapter config group from kv storage, error code: %d", err);
            break;
        }
#endif /* SID_STM32_BLE_CLEAR_CFG_ON_FACTORY_RESET */

        /* Clear non-volatile bonding contexts in the KV storage */
        err = sid_pal_storage_kv_group_delete(STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("  Fail   : Unable to delete sid ble adapter non-volatile bonding contexts from kv storage, error code: %d", err);
            break;
        }

        /* Erase NVM flash area */
        err = SID_ERROR_NONE;
        if (osKernelGetState() == osKernelRunning)
        {
            FM_Cmd_Status_t fm_status;
            osStatus_t os_status;

            /* RTOS is running, we can use Flash Manager module for async operations and to avoid conflicts with 2.4GHz radio operations */
            do
            {
                fm_status = FM_Erase(CFG_SNVMA_START_SECTOR_ID, (CFG_SNVMA_END_SECTOR_ID - CFG_SNVMA_START_SECTOR_ID) + 1u, &ble_adapter_user_fm_callback_node);
                if (fm_status != FM_OK)
                {
                    SID_PAL_LOG_ERROR("BLE NVM erase failed - Flash Manager error");
                    err = SID_ERROR_STORAGE_ERASE_FAIL;
                    break;
                }

                os_status = osSemaphoreAcquire(ble_adapter_user_fm_semaphore, osWaitForever);
                if (os_status != osOK)
                {
                    SID_PAL_LOG_ERROR("Failed to wait for BLE NVM earse completion. Error %d", (int32_t)os_status);
                    err = SID_ERROR_STORAGE_ERASE_FAIL;
                    break;
                }
            } while (ble_adapter_user_last_fm_status != FM_OPERATION_COMPLETE);
        }
        else
        {
            /* RTOS not running, fall back to direct calls to Flash Driver */
            (void)HAL_FLASH_Unlock();

            /* Data writing loop */
            for (uint32_t sector_id = CFG_SNVMA_START_SECTOR_ID; sector_id <= CFG_SNVMA_END_SECTOR_ID; sector_id++)
            {
                FD_FlashOp_Status_t fd_status = FD_EraseSectors(sector_id);
                if (fd_status != FD_FLASHOP_SUCCESS)
                {
                    SID_PAL_LOG_ERROR("Failed to erase BLE NVM page %u", sector_id);
                    err = SID_ERROR_STORAGE_ERASE_FAIL;
                    break;
                }
            }

            (void)HAL_FLASH_Lock();
        }
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        SID_PAL_LOG_INFO("User mode BLE factory reset completed");
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_activate(const sid_ble_ext_virtual_device_id_t virt_dev_id, sid_ble_ext_virtual_device_ctx_t * * const out_ctx)
{
    sid_error_t err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_activate function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        /* Entering danger zone - we are about to access and potentially modify the list of active devices */
        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_UNINITIALIZED;
            sid_pal_exit_critical_region();
            break;
        }

        /* Validate inputs */
        if ((virt_dev_id < SID_STM32_BLE_CUSTOM_VIRT_DEV_ID_MIN) || (virt_dev_id > SID_STM32_BLE_CUSTOM_VIRT_DEV_ID_MAX))
        {
            SID_PAL_LOG_ERROR("Virtual BLE device ID 0x%02X is out of valid range (0x%02X - 0x%02X)", virt_dev_id, SID_STM32_BLE_CUSTOM_VIRT_DEV_ID_MIN, SID_STM32_BLE_CUSTOM_VIRT_DEV_ID_MAX);
            err = SID_ERROR_INVALID_ARGS;
            sid_pal_exit_critical_region();
            break;
        }

        if (NULL == out_ctx)
        {
            SID_PAL_LOG_ERROR("No storage provided for virtual BLE device 0x%02X context", virt_dev_id);
            err = SID_ERROR_INVALID_ARGS;
            sid_pal_exit_critical_region();
            break;
        }

        /* Check the configuration for this device exists */
        const sid_ble_ext_virtual_device_t * virt_dev_cfg = _ble_adapter_user_search_device_definition(virt_dev_id);
        if (NULL == virt_dev_cfg)
        {
            SID_PAL_LOG_ERROR("No definition of the virtual BLE device 0x%02X found", virt_dev_id);
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Verify the device is not activated already */
        sid_ble_ext_virtual_device_node_t * dev_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (dev_node != NULL)
        {
            SID_PAL_LOG_ERROR("Virtual BLE device \"%s\" (0x%02X) is activated already", virt_dev_cfg->device_name != NULL ? virt_dev_cfg->device_name : "<unnamed>", virt_dev_id);
            *out_ctx = dev_node->ctx;
            err = SID_ERROR_ALREADY_INITIALIZED;
            sid_pal_exit_critical_region();
            break;
        }

        if (_ble_adapter_user_check_device_requires_pairing(virt_dev_cfg) != FALSE)
        {
            /* This device requires pairing - ensure it has compatible BLE address setting */
#if SID_STM32_BLE_USE_RANDOM_IDENTITY_ADDRESS
            /* If Static Random address is used as Identity Address virtual devices can use only Static Random and RPA addresses for pairing */
            if (SID_BLE_CFG_MAC_ADDRESS_TYPE_PUBLIC == virt_dev_cfg->mac_addr_type)
            {
                SID_PAL_LOG_ERROR("BLE pairing requires either Static Random or RPA address, but virtual BLE device \"%s\" (0x%02X) uses Public address", virt_dev_cfg->device_name != NULL ? virt_dev_cfg->device_name : "<unnamed>", virt_dev_id);
                err = SID_ERROR_NOSUPPORT;
                break;
            }
#else
            /* If Public address is used as Identity Address virtual devices can use only Public and RPA addresses for pairing */
            if (SID_BLE_CFG_MAC_ADDRESS_TYPE_STATIC_RANDOM == virt_dev_cfg->mac_addr_type)
            {
                SID_PAL_LOG_ERROR("BLE pairing requires either Public or RPA address, but virtual BLE device \"%s\" (0x%02X) uses Static Random address", virt_dev_cfg->device_name != NULL ? virt_dev_cfg->device_name : "<unnamed>", virt_dev_id);
                err = SID_ERROR_NOSUPPORT;
                break;
            }
#endif /* SID_STM32_BLE_USE_RANDOM_IDENTITY_ADDRESS */

            if (SID_BLE_CFG_MAC_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE == virt_dev_cfg->mac_addr_type)
            {
                SID_PAL_LOG_ERROR("Virtual BLE device \"%s\" (0x%02X): BLE pairing is not supported for non-resolvable addresses", virt_dev_cfg->device_name != NULL ? virt_dev_cfg->device_name : "<unnamed>", virt_dev_id);
                err = SID_ERROR_NOSUPPORT;
                break;
            }
        }

#if SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0
        /* Check the device can be activated - with no support for extended advertising we can only run one advertiser at a time */
        if ((SBEVDR_BLE_PERIPHERAL == virt_dev_cfg->device_type) || (SBEVDR_BLE_BROADCASTER == virt_dev_cfg->device_type))
        {
            sid_ble_ext_virtual_device_node_t * current_node = (sid_ble_ext_virtual_device_node_t *)(((tListNode *)&active_virt_devs_list)->next);
            uint32_t active_advertisers_exist = FALSE;

            while (current_node != &active_virt_devs_list)
            {
                if ((SBEVDR_BLE_PERIPHERAL == current_node->ctx->device_cfg->device_type) || (SBEVDR_BLE_BROADCASTER == current_node->ctx->device_cfg->device_type))
                {
                    /* Found an existing advertiser node */
                    active_advertisers_exist = TRUE;
                    break;
                }

                /* Advance to the next node */
                LST_get_next_node((tListNode *)current_node, (tListNode **)&current_node);
            }

            if (active_advertisers_exist != FALSE)
            {
                SID_PAL_LOG_ERROR("Can't bootstrap virtual BLE device \"%s\" (0x%02X) - another advetiser is currently active", virt_dev_cfg->device_name != NULL ? virt_dev_cfg->device_name : "<unnamed>", virt_dev_id);
                err = SID_ERROR_OUT_OF_RESOURCES;
                sid_pal_exit_critical_region();
                break;
            }
        }
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        /* Create new device node */
        dev_node = malloc(sizeof(*dev_node));
        if (NULL == dev_node)
        {
            SID_PAL_LOG_ERROR("Can't bootstrap virtual BLE device \"%s\" (0x%02X) - out of RAM", virt_dev_cfg->device_name != NULL ? virt_dev_cfg->device_name : "<unnamed>", virt_dev_id);
            err = SID_ERROR_OOM;
            sid_pal_exit_critical_region();
            break;
        }

        /* Allocate memory for device context */
        dev_node->ctx = calloc(1u, sizeof(*dev_node->ctx));
        if (NULL == dev_node)
        {
            SID_PAL_LOG_ERROR("Can't bootstrap virtual BLE device \"%s\" (0x%02X) - out of RAM", virt_dev_cfg->device_name != NULL ? virt_dev_cfg->device_name : "<unnamed>", virt_dev_id);
            err = SID_ERROR_OOM;
            sid_pal_exit_critical_region();
            break;
        }

        /* Set initial virtual device state */
        dev_node->ctx->state = SBEVD_STATE_BOOTSTRAPPING;

        /* Initialize connection list in the context */
        LST_init_head((tListNode *)&dev_node->ctx->conn_list);
        *((sid_ble_ext_connection_ctx_t **)&dev_node->ctx->conn_list.ctx) = NULL;
        dev_node->ctx->conn_cnt = 0u;

        /* Store device config pointer for faster access */
        dev_node->ctx->device_cfg = virt_dev_cfg;

        /* Add device node to the to list */
        LST_insert_tail((tListNode *)&active_virt_devs_list, (tListNode *)dev_node);

        /* Done with the critical stuff */
        sid_pal_exit_critical_region();

        /* Proceed with the role-specific initialization */
        SID_PAL_LOG_DEBUG("Bootstrapping virtual BLE device \"%s\" (0x%02X)", virt_dev_cfg->device_name != NULL ? virt_dev_cfg->device_name : "<unnamed>", virt_dev_id);

        switch (virt_dev_cfg->device_type)
        {
            case SBEVDR_BLE_PERIPHERAL:
                err = _ble_adapter_user_init_peripheral(dev_node);
                break;

            case SBEVDR_BLE_BROADCASTER:
                err = _ble_adapter_user_init_broadcaster(dev_node);
                break;

            case SBEVDR_BLE_CENTRAL:
                err = _ble_adapter_user_init_central(dev_node);
                break;

            case SBEVDR_BLE_OBSERVER:
                err = _ble_adapter_user_init_observer(dev_node);
                break;

            default:
                SID_PAL_LOG_ERROR("Failed to bootstrap virtual BLE device 0x%02X. %u is not a valid virtual BLE device type", virt_dev_cfg->device_id, (uint32_t)virt_dev_cfg->device_type);
                err = SID_ERROR_INVALID_ARGS;
                break;
        }

        /* Check if the BLE initialization was successful */
        if (err != SID_ERROR_NONE)
        {
            /* Something went wrong, deallocate resources and terminate */
            dev_node->ctx->state = SBEVD_STATE_FATAL_ERROR;
            (void)_ble_adapter_user_terminate_device_node(dev_node);
            break;
        }
        else
        {
            /* Device is ready now */
            dev_node->ctx->state = SBEVD_STATE_IDLE;
        }

        /* Done */
        SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X) is ready", virt_dev_cfg->device_name != NULL ? virt_dev_cfg->device_name : "<unnamed>", virt_dev_id);
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_activate function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_terminate(const sid_ble_ext_virtual_device_id_t virt_dev_id)
{
    sid_error_t err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_terminate function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        /* Ensure the node is in the acceptable state for termination */
        if ((associated_node->ctx->state < SBEVD_STATE_SUSPENDED) || (associated_node->ctx->state >= SBEVD_STATE_TERMINATING))
        {
            /* Either device is in the process of termination already or its initialization has not finished */
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Update device state */
        associated_node->ctx->state = SBEVD_STATE_TERMINATING;
        __COMPILER_BARRIER();

        /* Done with critical stuff */
        sid_pal_exit_critical_region();

        /* Terminate the virtual device */
        err = _ble_adapter_user_terminate_device_node(associated_node);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_user_terminate_device_node() */
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_terminate function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_set_adv_data(const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint8_t * const adv_data, const uint32_t adv_data_length)
{
    sid_error_t err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_set_adv_data function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        /* Ensure the node is capable of advertising at all */
        switch (associated_node->ctx->device_cfg->device_type)
        {
            case SBEVDR_BLE_PERIPHERAL:
                err = SID_ERROR_NONE;
                break;

            case SBEVDR_BLE_BROADCASTER:
                err = SID_ERROR_NONE;
                break;

            default:
                err = SID_ERROR_NOSUPPORT;
                break;
        }

        /* This virtual device is not capable of advertising */
        if (err != SID_ERROR_NONE)
        {
            sid_pal_exit_critical_region();
            break;
        }

        /* Set advertising data */
        err = _ble_adapter_user_set_node_adv_data(associated_node, adv_data, adv_data_length);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_user_set_node_adv_data() */
            sid_pal_exit_critical_region();
            break;
        }

        /* Done with critical stuff */
        sid_pal_exit_critical_region();

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_set_adv_data function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_set_scan_resp_data(const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint8_t * const scan_resp_data, const uint32_t scan_resp_data_length)
{
    sid_error_t err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_set_scan_resp_data function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        /* Ensure the node is capable of advertising at all */
        const sid_ble_ext_adv_param_t * adv_param;
        switch (associated_node->ctx->device_cfg->device_type)
        {
            case SBEVDR_BLE_PERIPHERAL:
                adv_param = associated_node->ctx->device_cfg->peripheral_cfg.adv_param;
                err = SID_ERROR_NONE;
                break;

            case SBEVDR_BLE_BROADCASTER:
                adv_param = associated_node->ctx->device_cfg->broadcaster_cfg.adv_param;
                err = SID_ERROR_NONE;
                break;

            default:
                adv_param = NULL;
                err = SID_ERROR_NOSUPPORT;
                break;
        }

        /* This virtual device is not capable of advertising */
        if (err != SID_ERROR_NONE)
        {
            sid_pal_exit_critical_region();
            break;
        }

        if ((NULL == adv_param) || (FALSE == adv_param->scan_resp_en))
        {
            /* Scan responses are not allowed in the configuration */
            sid_pal_exit_critical_region();
            err = SID_ERROR_NOSUPPORT;
            break;
        }

        /* Set advertising data */
        err = _ble_adapter_user_set_node_scan_resp_data(associated_node, scan_resp_data, scan_resp_data_length);
        if (err != SID_ERROR_NONE)
        {
            /* Logs provided by _ble_adapter_user_set_node_adv_data() */
            sid_pal_exit_critical_region();
            break;
        }

        /* Done with critical stuff */
        sid_pal_exit_critical_region();

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_set_scan_resp_data function");

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _ble_adapter_user_virt_dev_adv_start(const sid_ble_ext_virtual_device_id_t virt_dev_id)
{
    sid_error_t err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_adv_start function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        /* Ensure the node is capable of advertising at all */
        const sid_ble_ext_adv_param_t * adv_param;
        switch (associated_node->ctx->device_cfg->device_type)
        {
            case SBEVDR_BLE_PERIPHERAL:
                adv_param = associated_node->ctx->device_cfg->peripheral_cfg.adv_param;
                err = SID_ERROR_NONE;
                break;

            case SBEVDR_BLE_BROADCASTER:
                adv_param = associated_node->ctx->device_cfg->broadcaster_cfg.adv_param;
                err = SID_ERROR_NONE;
                break;

            default:
                err = SID_ERROR_NOSUPPORT;
                break;
        }

        /* This virtual device is not capable of advertising */
        if (err != SID_ERROR_NONE)
        {
            sid_pal_exit_critical_region();
            break;
        }

        /* Initiate advertising */
        err = _ble_adapter_user_node_start_advertisement(associated_node, adv_param);
        if (err != SID_ERROR_NONE)
        {
            /* Logs supplied by _ble_adapter_user_node_start_advertisement() */
            sid_pal_exit_critical_region();
            break;
        }

        /* Done with critical stuff */
        sid_pal_exit_critical_region();

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_adv_start function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_adv_stop(const sid_ble_ext_virtual_device_id_t virt_dev_id)
{
    sid_error_t err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_adv_stop function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        /* Ensure the node is capable of advertising at all */
        switch (associated_node->ctx->device_cfg->device_type)
        {
            case SBEVDR_BLE_PERIPHERAL:
                err = SID_ERROR_NONE;
                break;

            case SBEVDR_BLE_BROADCASTER:
                err = SID_ERROR_NONE;
                break;

            default:
                err = SID_ERROR_NOSUPPORT;
                break;
        }

        /* This virtual device is not capable of advertising */
        if (err != SID_ERROR_NONE)
        {
            sid_pal_exit_critical_region();
            break;
        }

        /* Initiate advertising */
        err = _ble_adapter_user_node_stop_advertisement(associated_node);
        if (err != SID_ERROR_NONE)
        {
            /* Logs supplied by _ble_adapter_user_node_start_advertisement() */
            sid_pal_exit_critical_region();
            break;
        }

        /* Done with critical stuff */
        sid_pal_exit_critical_region();

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_adv_stop function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_get_bonded_peers(const sid_ble_ext_virtual_device_id_t virt_dev_id, sid_pal_ble_ext_peer_identity_addr_t * const out_bonded_peers_list, uint32_t * const out_bonded_peers_count, const uint32_t bonded_peers_list_size_limit)
{
    sid_error_t err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_get_bonded_peers function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        /* Validate inputs */
        if ((NULL == out_bonded_peers_list) || (NULL == out_bonded_peers_count) || (0u == bonded_peers_list_size_limit))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        *out_bonded_peers_count = 0u;

        /* Run in a critical section to avoid mid-flight alterations of the KV data */
        sid_pal_enter_critical_region();

        err = SID_ERROR_NONE;

        for (uint32_t i = 0u; i < SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES; i++)
        {
            sid_pal_ble_ext_peer_identity_addr_t kv_bond_pair;
            const uint16_t bond_pair_kv_key = _ble_adapter_user_get_storage_kv_bond_pair_key(virt_dev_id, (uint16_t)i);

            err = sid_pal_storage_kv_record_get(STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP, bond_pair_kv_key, &kv_bond_pair, sizeof(kv_bond_pair));
            if (SID_ERROR_NONE == err)
            {
                /* Found a valid record */
                if (*out_bonded_peers_count < bonded_peers_list_size_limit)
                {
                    SID_STM32_UTIL_fast_memcpy(&out_bonded_peers_list[*out_bonded_peers_count], &kv_bond_pair, sizeof(kv_bond_pair));
                }

                (*out_bonded_peers_count)++;
            }
            else if (SID_ERROR_NOT_FOUND == err)
            {
                /* Record does not exists, proceed to the next slot */
                continue;
            }
            else
            {
                /* Generic KV storage failure - terminate */
                break;
            }
        }

        if (err != SID_ERROR_NONE)
        {
            sid_pal_exit_critical_region();
            break;
        }

        /* Done with critical stuff */
        sid_pal_exit_critical_region();

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_get_bonded_peers function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_remove_bond(const sid_ble_ext_virtual_device_id_t virt_dev_id, const sid_pal_ble_ext_peer_identity_addr_t * const peer_identity)
{
    sid_error_t err;
    tBleStatus ble_status;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_remove_bond function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        uint16_t storage_kv_slot_id;

        /* Validate inputs */
        if (NULL == peer_identity)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        const sid_ble_ext_virtual_device_t * const virt_dev_cfg = _ble_adapter_user_search_device_definition(virt_dev_id);

        /* Done with critical stuff */
        sid_pal_exit_critical_region();

        /* Check if the virtual device is found */
        if (NULL == virt_dev_cfg)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Check the virtual device is of the acceptable type */
        if (virt_dev_cfg->device_type != SBEVDR_BLE_PERIPHERAL)
        {
            err = SID_ERROR_NOSUPPORT;
            break;
        }

        /* Locate the non-volatile bonding context */
        err = _ble_adapter_user_search_bond_pair_kv_record(virt_dev_id, peer_identity->identity_address_type, peer_identity->identity_address, &storage_kv_slot_id);
        if (SID_ERROR_NOT_FOUND == err)
        {
            /* Failed to locate the non-volatile bonding context of the peer identity that is associated with the specified virtual device */
            SID_PAL_LOG_ERROR("Failed to remove bonding info for peer %02X:%02X:%02X:%02X:%02X:%02X: it's not associated with the Virtual BLE device \"%s\" (0x%02X)",
                              peer_identity->identity_address[5],
                              peer_identity->identity_address[4],
                              peer_identity->identity_address[3],
                              peer_identity->identity_address[2],
                              peer_identity->identity_address[1],
                              peer_identity->identity_address[0],
                              virt_dev_cfg->device_name != NULL ? virt_dev_cfg->device_name : "<unnamed>",
                              virt_dev_id);
            break;
        }
        if (err != SID_ERROR_NONE)
        {
            /* Some other error */
            break;
        }

        /* Remove bonding from the controller */ 
        ble_status = aci_gap_remove_bonded_device(peer_identity->identity_address_type, peer_identity->identity_address);
        if (SID_BLE_HCI_STATUS_CMD_DISALLOWED == ble_status)
        {
            err = SID_ERROR_BUSY;
            break;
        }
        else if (ble_status != BLE_STATUS_SUCCESS)
        {
            err = SID_ERROR_IO_ERROR;
            break;
        }
        else
        {
            /* Nothing to do, verything is ok */
        }

        /* Erase the non-volatile bonding context */
        const uint16_t bond_pair_kv_key = _ble_adapter_user_get_storage_kv_bond_pair_key(virt_dev_id, storage_kv_slot_id);
        err = sid_pal_storage_kv_record_delete(STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP, bond_pair_kv_key);
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_remove_bond function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_disconnect(const sid_ble_ext_virtual_device_id_t virt_dev_id, const uint16_t conn_id)
{
    sid_error_t err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_disconnect function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        /* Ensure the node is capable of advertising at all */
        switch (associated_node->ctx->device_cfg->device_type)
        {
            case SBEVDR_BLE_PERIPHERAL:
                err = SID_ERROR_NONE;
                break;

            case SBEVDR_BLE_CENTRAL:
                err = SID_ERROR_NONE;
                break;

            default:
                err = SID_ERROR_NOSUPPORT;
                break;
        }

        /* This virtual device is not capable of connections */
        if (err != SID_ERROR_NONE)
        {
            sid_pal_exit_critical_region();
            break;
        }

        /* Try to locate the specified connection */
        sid_ble_ext_conn_list_node_t * const conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(associated_node->ctx, conn_id);
        if (NULL == conn_node)
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        /* Done with critical stuff */
        sid_pal_exit_critical_region();

        /* Terminate the connection */
        err = _ble_adapter_user_node_disconnect(associated_node, conn_node);
        if (err != SID_ERROR_NONE)
        {
            /* Logs supplied by _ble_adapter_user_node_disconnect() */
            break;
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_disconnect function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_disconnect_all(const sid_ble_ext_virtual_device_id_t virt_dev_id)
{
    sid_error_t err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_disconnect_all function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        /* Ensure the node is capable of advertising at all */
        switch (associated_node->ctx->device_cfg->device_type)
        {
            case SBEVDR_BLE_PERIPHERAL:
                err = SID_ERROR_NONE;
                break;

            case SBEVDR_BLE_CENTRAL:
                err = SID_ERROR_NONE;
                break;

            default:
                err = SID_ERROR_NOSUPPORT;
                break;
        }

        /* This virtual device is not capable of connections */
        if (err != SID_ERROR_NONE)
        {
            sid_pal_exit_critical_region();
            break;
        }

        /* Done with critical stuff */
        sid_pal_exit_critical_region();

        while (LST_is_empty((tListNode *)&associated_node->ctx->conn_list) == FALSE)
        {
            sid_ble_ext_conn_list_node_t * conn_node;

            LST_get_next_node((tListNode *)&associated_node->ctx->conn_list, (tListNode **)&conn_node);
            err = _ble_adapter_user_node_disconnect(associated_node, conn_node);
            if (err != SID_ERROR_NONE)
            {
                /* We can't disconnect - just skip this connection, remove it from list and deallocate memory */
                LST_remove_node((tListNode *)conn_node);
                free(conn_node);
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_disconnect_all function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_suspend(const sid_ble_ext_virtual_device_id_t virt_dev_id)
{
    sid_error_t err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_resume function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        if ((associated_node->ctx->state < SBEVD_STATE_IDLE) || (associated_node->ctx->state >= SBEVD_STATE_FATAL_ERROR))
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Update device state; */
        associated_node->ctx->state = SBEVD_STATE_SUSPENDED;

        /* Done with critical stuff */
        sid_pal_exit_critical_region();

        /* Terminate all advertisements */
        if ((SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type) || (SBEVDR_BLE_BROADCASTER == associated_node->ctx->device_cfg->device_type))
        {
            (void)_ble_adapter_user_node_stop_advertisement(associated_node);
        }

        /* Terminate all connections */
        if ((SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type) || (SBEVDR_BLE_CENTRAL == associated_node->ctx->device_cfg->device_type))
        {
            while (LST_is_empty((tListNode *)&associated_node->ctx->conn_list) == FALSE)
            {
                sid_ble_ext_conn_list_node_t * conn_node;

                LST_get_next_node((tListNode *)&associated_node->ctx->conn_list, (tListNode **)&conn_node);
                err = _ble_adapter_user_node_disconnect(associated_node, conn_node);
                if (err != SID_ERROR_NONE)
                {
                    /* We can't disconnect - just skip this connection, remove it from list and deallocate memory */
                    LST_remove_node((tListNode *)conn_node);
                    free(conn_node);
                }
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_resume function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_resume(const sid_ble_ext_virtual_device_id_t virt_dev_id)
{
    sid_error_t err;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_resume function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        if (SBEVD_STATE_SUSPENDED == associated_node->ctx->state)
        {
            associated_node->ctx->state = SBEVD_STATE_IDLE;
            err = SID_ERROR_NONE;
        }
        else
        {
            err = SID_ERROR_INVALID_STATE;
        }

        /* Done with critical stuff */
        sid_pal_exit_critical_region();
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_resume function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _ble_adapter_user_virt_dev_update_char(const sid_ble_ext_virtual_device_id_t virt_dev_id, const sid_ble_cfg_uuid_info_t * const char_uuid, const uint8_t * const data, const uint32_t data_length, uint8_t notify)
{
    sid_error_t err;
    tBleStatus  ble_ret;
    uint32_t    ll_lock_acquired;

    SID_BLE_USER_LOG_DEBUG("==>> Start ble_adapter_user_virt_dev_update_char function");

    /* Acquire BLE LL lock */
    ll_lock_acquired = _ble_adapter_user_acquire_ll_lock("Failed to acquire BLE LL mutex, race conditions are possible");

    do
    {
        if ((NULL == char_uuid) || (NULL == data) || (0u == data_length))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        sid_pal_enter_critical_region();

        /* Check if the user part is initialized at all */
        if (sid_ble_user_mode_ctx.state != SPBUM_STATE_INITIALIZED)
        {
            err = SID_ERROR_INVALID_STATE;
            sid_pal_exit_critical_region();
            break;
        }

        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        /* Ensure the device is a Peripheral */
        if (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL)
        {
            err = SID_ERROR_NOSUPPORT;
            sid_pal_exit_critical_region();
            break;
        }

        /* Search characteristic context by UUID */
        sid_ble_ext_gatt_server_svc_ctx_t * svc_ctx;
        sid_ble_ext_gatt_server_char_ctx_t * char_ctx;
        _ble_adapter_user_search_char_ctx_by_uuid(associated_node->ctx, char_uuid, &svc_ctx, &char_ctx);

        if ((NULL == svc_ctx) || (NULL == char_ctx))
        {
            err = SID_ERROR_NOT_FOUND;
            sid_pal_exit_critical_region();
            break;
        }

        /* Done with critical stuff */
        sid_pal_exit_critical_region();

        /* Use aci_gatt_update_char_value_ext() instead of aci_gatt_update_char_value() to support MTU sizes greater than 251 bytes */
        if (data_length <= SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT)
        {
            ble_ret = aci_gatt_update_char_value_ext(0x0000u, /* Notify all connections if enabled */
                                                     svc_ctx->handle,
                                                     char_ctx->handle,
                                                     (notify != FALSE) ? SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_TYPE_NOTIFY : SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_TYPE_DO_NOT_NOTIFY,
                                                     (uint16_t)data_length,
                                                     0u, /* value offset */
                                                     (uint8_t)data_length,
                                                     data);
        }
        else
        {
            /* Use several subsequent calls to pass all the data to BLE stack. Since maximum MTU size is 512 bytes we may need up to 3 calls */
            SID_BLE_USER_LOG_DEBUG("==>> aci_gatt_update_char_value_ext - sending out   long data (%u bytes)", length);

            uint32_t remaining_data_size = data_length;
            uint32_t current_offset = 0u;

            while (remaining_data_size > SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT)
            {
                /* First call just pushes the data but does not trigger the actual value change notification */
                ble_ret = aci_gatt_update_char_value_ext(0x0000u, /* Notify all connections if enabled */
                                                         svc_ctx->handle,
                                                         char_ctx->handle,
                                                         SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_TYPE_DO_NOT_NOTIFY,
                                                         (uint16_t)data_length, /* total length */
                                                         (uint16_t)current_offset, /* value offset */
                                                         SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT, /* size of the partial data to push */
                                                         data + current_offset);
                if (BLE_STATUS_SUCCESS == ble_ret)
                {
                    SID_BLE_USER_LOG_DEBUG("==>> aci_gatt_update_char_value_ext - updated partial data, start offset: %u, bytes written: %u, remaining size: %u", current_offset, SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT, remaining_data_size - SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT);
                    current_offset += SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT;
                    remaining_data_size -= SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_SIZE_LIMIT;
                }
                else
                {
                    SID_BLE_USER_LOG_ERROR("==>> aci_gatt_update_char_value_ext - fail on putting partial data, start offset: %u, total length: %u, result: 0x%02X", current_offset, length, ble_ret);
                    break;
                }
            }

            if (BLE_STATUS_SUCCESS == ble_ret)
            {
                /* The last call uploads the rest of the data and triggers BLE notification indication */
                ble_ret = aci_gatt_update_char_value_ext(0x0000u, /* Notify all connections if enabled */
                                                         svc_ctx->handle,
                                                         char_ctx->handle,
                                                         (notify != FALSE) ? SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_TYPE_NOTIFY : SID_STM32_BLE_ACI_CHARACTERISTIC_VALUE_UPDATE_TYPE_DO_NOT_NOTIFY,
                                                         (uint16_t)data_length, /* total length */
                                                         (uint16_t)current_offset, /* value offset */
                                                         (uint8_t)remaining_data_size, /* size of the remaining partial data to push */
                                                         data + current_offset);
            }
        }

        if (BLE_STATUS_SUCCESS == ble_ret)
        {
            SID_BLE_USER_LOG_DEBUG("==>> aci_gatt_update_char_value_ext - success, %u bytes written", length);
            err = SID_ERROR_NONE;
        }
        else if (BLE_STATUS_INSUFFICIENT_RESOURCES == ble_ret)
        {
            SID_BLE_USER_LOG_WARNING("==>> aci_gatt_update_char_value_ext - recoverable failure, insufficient resources");
            err = SID_ERROR_BUSY;
        }
        else
        {
            SID_BLE_USER_LOG_ERROR("==>> aci_gatt_update_char_value_ext - fail, result: 0x%02X", ble_ret);
            err = SID_ERROR_IO_ERROR;
        }
    } while (0);

    /* Release LL lock */
    (void)_ble_adapter_user_release_ll_lock(ll_lock_acquired, "Failed to release BLE LL mutex. BLE stack will be blocked till reset");

    SID_BLE_USER_LOG_DEBUG("==>> End ble_adapter_user_virt_dev_update_char function");

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_platform_init(void)
{
    sid_error_t err;

    do
    {
        /* Create semaphore to manage flash write and erase operations */
        if (NULL == ble_adapter_user_fm_semaphore)
        {
            ble_adapter_user_fm_semaphore = osSemaphoreNew(1u, 0u, &ble_adapter_user_fm_semaphore_attributes);
            if (NULL == ble_adapter_user_fm_semaphore)
            {
                err = SID_ERROR_OOM;
                break;
            }
        }

        /* Configure callback */
        ble_adapter_user_fm_callback_node.Callback = _ble_adapter_user_fm_callback;

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_adv_timeout(const sid_ble_ext_virtual_device_id_t virt_dev_id)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        sid_ble_ext_adv_state_change_t adv_notify_event;

        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node(virt_dev_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Get advertising parameters */
        const sid_ble_ext_adv_param_t * adv_param;
        switch (associated_node->ctx->device_cfg->device_type)
        {
            case SBEVDR_BLE_PERIPHERAL:
                adv_param = associated_node->ctx->device_cfg->peripheral_cfg.adv_param;
                err = SID_ERROR_NONE;
                break;

            case SBEVDR_BLE_BROADCASTER:
                adv_param = associated_node->ctx->device_cfg->broadcaster_cfg.adv_param;
                err = SID_ERROR_NONE;
                break;

            default:
                err = SID_ERROR_NOSUPPORT;
                break;
        }

        /* This virtual device is not capable of advertising */
        if (err != SID_ERROR_NONE)
        {
            sid_pal_exit_critical_region();
            break;
        }

        /* Check if restart is needed */
        if ((SBEVD_STATE_ADV_FAST == associated_node->ctx->state) && (adv_param->slow_enabled != FALSE))
        {
            /* Fast advertisement timed out and slow advertisement is allowed - proceed with slow advertisement */

            /* Indicate the advertisement is stopped in case restart will fail */
            if ((associated_node->ctx->state & SBEVD_STATE_CONNECTED) != 0u)
            {
                /* Device was connected during advertisement -> clear adv bits and keep Connected status */
                associated_node->ctx->state &= ~(SBEVD_STATE_ADV_FAST | SBEVD_STATE_ADV_SLOW);
            }
            else
            {
                /* No connections were present, now the node is idle */
                associated_node->ctx->state = SBEVD_STATE_IDLE;
            }

            /* Update advertisement duration */
            const uint32_t adv_duration = adv_param->slow_timeout;

            /* Apply advertisement parameters for slow advertisement */
            err = _ble_adapter_user_set_adv_params(associated_node->ctx, adv_param->slow_interval_min, adv_param->slow_interval_max);
            if (err != SID_ERROR_NONE)
            {
                /* Logs provided by _ble_adapter_sidewalk_set_adv_params() */
                break;
            }

            /* Build advertising set descriptor for HCI interface */
            Adv_Set_t hci_adv_set = {
                .Advertising_Handle              = associated_node->ctx->device_cfg->device_id,
                .Duration                        = adv_duration,
                .Max_Extended_Advertising_Events = 0u,
            };

            /* Restart advertisement */
            err = sid_stm32wba_ble_adapter_prv_generic_start_advertisement(&hci_adv_set);
            if (err != SID_ERROR_NONE)
            {
                /* Logs provided by sid_stm32wba_ble_adapter_prv_generic_start_advertisement */
                break;
            }

            /* Indicate that we are advertising */
            if (SBEVD_STATE_IDLE == associated_node->ctx->state)
            {
                associated_node->ctx->state = SBEVD_STATE_ADV_SLOW;
            }
            else
            {
                /* Keep Connected bit and add Adv Slow bit */
                associated_node->ctx->state |= SBEVD_STATE_ADV_SLOW;
            }
            adv_notify_event = SBEASC_ADV_SWITCHED_FAST_TO_SLOW;
            SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X) switched advertisement to slow mode",
                         associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                         associated_node->ctx->device_cfg->device_id);
        }
        else
        {
            /* No more advertisements - final timeout */
            if ((associated_node->ctx->state & SBEVD_STATE_CONNECTED) != 0u)
            {
                /* Device was connected during advertisement -> clear adv bits and keep Connected status */
                associated_node->ctx->state &= ~(SBEVD_STATE_ADV_FAST | SBEVD_STATE_ADV_SLOW);
            }
            else
            {
                /* No connections were present, now the node is idle */
                associated_node->ctx->state = SBEVD_STATE_IDLE;
            }
            adv_notify_event = SBEASC_ADV_SWITCHED_TERMINATED_BY_TIMEOUT;
            SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X) terminated advertisement due to timeout",
                         associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                         associated_node->ctx->device_cfg->device_id);
        }

        /* Call user callback for advertising state change event */
        sid_ble_ext_on_adv_state_changed_cb_t on_adv_state_changed;
        if (SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type)
        {
            on_adv_state_changed = associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_adv_state_changed;
        }
        else if (SBEVDR_BLE_BROADCASTER == associated_node->ctx->device_cfg->device_type)
        {
            on_adv_state_changed = associated_node->ctx->device_cfg->broadcaster_cfg.callbacks.on_adv_state_changed;
        }
        else
        {
            on_adv_state_changed = NULL;
        }

        if (on_adv_state_changed != NULL)
        {
            on_adv_state_changed(associated_node->ctx, adv_notify_event);
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_ble_connected_to_central(const sid_ble_ext_virtual_device_id_t virt_dev_id, sid_pal_ble_prv_connection_ctx_t * const conn_ctx)
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus ble_status;
    sid_ble_ext_conn_list_node_t * user_conn_ctx_node = NULL;

    do
    {
        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node =
#if (SID_STM32_BLE_EXTENDED_ADV_SUPPORTED == 0u)
                                                                    _ble_adapter_user_search_first_node_by_type(SBEVDR_BLE_PERIPHERAL);
#else
                                                                    _ble_adapter_user_search_device_node(virt_dev_id);
#endif /* SID_STM32_BLE_EXTENDED_ADV_SUPPORTED */

        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL)
        {
            /* Should not be possible, but just in case */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        /* Check we can accept this connection */
        sid_pal_enter_critical_region();
        {
            /* Check if the virtual device state is ok */
            if ((associated_node->ctx->state < SBEVD_STATE_IDLE) || (associated_node->ctx->state >= SBEVD_STATE_FATAL_ERROR))
            {
                err = SID_ERROR_INVALID_STATE;
                sid_pal_exit_critical_region();
                break;
            }

            /* Check if the limit on simultaneous connections is not exceeded */
            if (associated_node->ctx->conn_cnt >= associated_node->ctx->device_cfg->peripheral_cfg.max_conn)
            {
                SID_PAL_LOG_ERROR("Virtual BLE device \"%s\" (0x%02X) can't accept connection 0x%04X. Connections limit (%u) reached",
                         associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                         associated_node->ctx->device_cfg->device_id,
                         conn_ctx->public_ctx.conn_id,
                         associated_node->ctx->device_cfg->peripheral_cfg.max_conn);
                err = SID_ERROR_OUT_OF_RESOURCES;
                sid_pal_exit_critical_region();
                break;
            }
        }
        sid_pal_exit_critical_region();

        /* Allocate memory for the user connection context list node */
        user_conn_ctx_node = malloc(sizeof(*user_conn_ctx_node));
        if (NULL == user_conn_ctx_node)
        {
            /* No memory to store connection context */
            err = SID_ERROR_OOM;
            break;
        }
        user_conn_ctx_node->node.prev = NULL;
        user_conn_ctx_node->node.next = NULL;

        sid_pal_enter_critical_region();
        {
            /* Store connection context pointer */
            *(sid_ble_ext_connection_ctx_t **)&user_conn_ctx_node->ctx = &conn_ctx->public_ctx;
            LST_insert_head((tListNode *)&associated_node->ctx->conn_list, (tListNode *)user_conn_ctx_node);
            associated_node->ctx->conn_cnt++;

            /* Set the MTU limit specific to the virtual BLE device */
            conn_ctx->public_ctx.max_mtu_size = associated_node->ctx->device_cfg->peripheral_cfg.max_att_mtu;

            /* Update logical state - set Connection flag and clear Advertising flags since advertising is terminated automatically upon a successful connection */
            associated_node->ctx->state |= SBEVD_STATE_CONNECTED;
            associated_node->ctx->state &= ~(SBEVD_STATE_ADV_FAST | SBEVD_STATE_ADV_SLOW);

            /* Enable authorization for the connection */
            ble_status = aci_gap_set_authorization_requirement(conn_ctx->public_ctx.conn_id, 0x01u);
            if (ble_status != BLE_STATUS_SUCCESS)
            {
                SID_BLE_USER_LOG_ERROR("aci_gap_set_authorization_requirement - fail, result: 0x%02X", ble_status);
                err = SID_ERROR_IO_ERROR;
                break;
            }
            SID_BLE_USER_LOG_DEBUG("==>> aci_gap_set_authorization_requirement - Success");
        }
        sid_pal_exit_critical_region();

        /* Notify user about the established connection */
        if (associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_connected != NULL)
        {
            associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_connected(associated_node->ctx, &conn_ctx->public_ctx);
        }

        /* Notify user about the initial MTU size (23 bytes) */
        if (associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_mtu_changed != NULL)
        {
            associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_mtu_changed(associated_node->ctx, &conn_ctx->public_ctx);
        }

#if SID_STM32_BLE_GATT_EXCHANGE_CONFIG_SUPPORTED && SID_STM32_BLE_AUTO_NEGOTIATE_MTU_SIZE
        /* Exchange peripheral config to negotiate Sidewalk-specific MTU size */
        ble_status = aci_gatt_exchange_config(conn_ctx->public_ctx.conn_id);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_USER_LOG_ERROR("aci_gatt_exchange_config - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_USER_LOG_DEBUG("==>> aci_gatt_exchange_config - Success");
#else
        /* This command is not supported by the peripheral-only version of the stack */
#endif /* SID_STM32_BLE_GATT_EXCHANGE_CONFIG_SUPPORTED && SID_STM32_BLE_AUTO_NEGOTIATE_MTU_SIZE */

#if SID_STM32_BLE_AUTO_NEGOTIATE_CONN_PARAMS
        /* Update connection parameters */
        if (associated_node->ctx->device_cfg->peripheral_cfg.conn_param != NULL)
        {
            const sid_ble_cfg_conn_param_t * const preferred_conn_param = associated_node->ctx->device_cfg->peripheral_cfg.conn_param;
            ble_status = aci_l2cap_connection_parameter_update_req(
                                                               conn_ctx->public_ctx.conn_id,
                                                               preferred_conn_param->min_conn_interval,
                                                               preferred_conn_param->min_conn_interval,
                                                               preferred_conn_param->slave_latency,
                                                               preferred_conn_param->conn_sup_timeout);
            if (ble_status != BLE_STATUS_SUCCESS)
            {
                SID_BLE_USER_LOG_ERROR("aci_l2cap_connection_parameter_update_req - fail, result: 0x%02X", ble_status);
                err = SID_ERROR_IO_ERROR;
                break;
            }
            SID_BLE_USER_LOG_DEBUG("==>> aci_l2cap_connection_parameter_update_req - Success");
        }
#endif /* SID_STM32_BLE_AUTO_NEGOTIATE_CONN_PARAMS */

#if SID_STM32_BLE_DLE_ENABLE && SID_STM32_BLE_AUTO_NEGOTIATE_DLE_PARAMS
        /* Enable data length extensions (DLE) for Sidewalk link */
        ble_status = hci_le_set_data_length(conn_ctx->public_ctx.conn_id, SID_STM32_BLE_DLE_MAX_TX_OCTETS, SID_STM32_BLE_DLE_MAX_TX_TIME_US);
        if (ble_status != BLE_STATUS_SUCCESS)
        {
            SID_BLE_USER_LOG_ERROR("hci_le_set_data_length - fail, result: 0x%02X", ble_status);
            err = SID_ERROR_IO_ERROR;
            break;
        }
        SID_BLE_USER_LOG_DEBUG("==>> hci_le_set_data_length - Success");
#endif /* SID_STM32_BLE_DLE_ENABLE && SID_STM32_BLE_AUTO_NEGOTIATE_DLE_PARAMS */

        /* Schedule automatic advertisement restart if needed. No need to check here if the node is advertising already because advertising is terminated on connection establishment */
        if (associated_node->ctx->device_cfg->peripheral_cfg.adv_param->auto_restart != FALSE)
        {
            err = _ble_adapter_user_node_start_advertisement(associated_node, associated_node->ctx->device_cfg->peripheral_cfg.adv_param);
            if ((err != SID_ERROR_NONE) && (err != SID_ERROR_BUSY))
            {
                SID_PAL_LOG_ERROR("Failed to automatically restart advertising for virtual BLE device \"%s\" (0x%02X). Error %d",
                         associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                         associated_node->ctx->device_cfg->device_id,
                         (int32_t)err);
            }
        }
        else
        {
            /* Notify user about the advertising termination */
            if (associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_adv_state_changed != NULL)
            {
                associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_adv_state_changed(associated_node->ctx, SBEASC_ADV_SWITCHED_TERMINATED_BY_CONNECTION);
            }
        }

        /* Done */
        SID_PAL_LOG_INFO("Connection (0x%04X) established with the Virtual BLE device \"%s\" (0x%02X). Total connections: %u",
                         conn_ctx->public_ctx.conn_id,
                         associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                         associated_node->ctx->device_cfg->device_id,
                         associated_node->ctx->conn_cnt);
        err = SID_ERROR_NONE;
    } while (0);

    if ((err != SID_ERROR_NONE) && (user_conn_ctx_node != NULL))
    {
        if ((user_conn_ctx_node->node.prev != NULL) && (user_conn_ctx_node->node.next != NULL))
        {
            LST_remove_node((tListNode *)user_conn_ctx_node);
        }
        free(user_conn_ctx_node);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_ble_connected_to_peripheral(sid_pal_ble_prv_connection_ctx_t * const conn_ctx)
{
    sid_error_t err = SID_ERROR_NOSUPPORT;

    //TODO: implement support for BLE Central role

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_ble_disconnected(const uint16_t conn_id, const uint8_t reason)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node_by_conn_handle(conn_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if ((associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL) && (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_CENTRAL))
        {
            /* Should not be possible, but just in case */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        /* Locate connection context */
        sid_ble_ext_conn_list_node_t * const conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(associated_node->ctx, conn_id);
        if (NULL == conn_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        sid_pal_enter_critical_region();
        {
            /* Remove connection from the list */
            LST_remove_node((tListNode *)conn_node);

            /* Update active connections counter */
            if (associated_node->ctx->conn_cnt > 0u)
            {
                associated_node->ctx->conn_cnt--;
            }

            /* Update device state */
            if ((0u == associated_node->ctx->conn_cnt) && (associated_node->ctx->state < SBEVD_STATE_FATAL_ERROR) && (associated_node->ctx->state >= SBEVD_STATE_IDLE))
            {
                if ((associated_node->ctx->state & (SBEVD_STATE_ADV_FAST | SBEVD_STATE_ADV_SLOW)) == 0u)
                {
                    /* Device was not advertising, now it's in Idle state */
                    associated_node->ctx->state = SBEVD_STATE_IDLE;
                }
                else
                {
                    /* Device is currently advertising -> just clear Connected bit and keep adv bits in place */
                    associated_node->ctx->state &= ~SBEVD_STATE_CONNECTED;
                }
            }
            else
            {
                /* Nothing to do - the node is still connected to some other device(s) */
            }
        }
        sid_pal_exit_critical_region();

        /* Notify user about the terminated connection */
        sid_ble_ext_on_disconnected_cb_t on_disconnected;
        if (SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type)
        {
            on_disconnected = associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_disconnected;
        }
        else if (SBEVDR_BLE_CENTRAL == associated_node->ctx->device_cfg->device_type)
        {
            on_disconnected = associated_node->ctx->device_cfg->central_cfg.callbacks.on_disconnected;
        }
        else
        {
            on_disconnected = NULL;
        }

        if ((on_disconnected != NULL) && (associated_node->ctx->state < SBEVD_STATE_FATAL_ERROR))
        {
            on_disconnected(associated_node->ctx, conn_node->ctx);
        }

        /* Deallocate memory for the connection context node */
        free(conn_node);

        SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X terminated (reason 0x%02X). Remaining active connections: %u",
                         associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                         associated_node->ctx->device_cfg->device_id,
                         conn_id,
                         reason,
                         associated_node->ctx->conn_cnt);

        /* If advertisement autorestart is enabled and the node is not advertising already - resume advertising */
        if ((SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type) && (associated_node->ctx->device_cfg->peripheral_cfg.adv_param->auto_restart != FALSE)
          && ((associated_node->ctx->state < SBEVD_STATE_FATAL_ERROR)  && (associated_node->ctx->state >= SBEVD_STATE_IDLE))
          && ((associated_node->ctx->state & (SBEVD_STATE_ADV_FAST | SBEVD_STATE_ADV_SLOW)) == 0u))
        {
            /* Restart advertising for the node */
            err = _ble_adapter_user_node_start_advertisement(associated_node, associated_node->ctx->device_cfg->peripheral_cfg.adv_param);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to automatically restart advertising for virtual BLE device \"%s\" (0x%02X). Error %d",
                         associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                         associated_node->ctx->device_cfg->device_id,
                         (int32_t)err);
            }
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_ble_mtu_changed(const uint16_t conn_id)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node_by_conn_handle(conn_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if ((associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL) && (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_CENTRAL))
        {
            /* Should not be possible, but just in case */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        /* Locate connection context */
        sid_ble_ext_conn_list_node_t * const conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(associated_node->ctx, conn_id);
        if (NULL == conn_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Notify user about the terminated connection */
        sid_ble_ext_on_mtu_changed_cb_t on_mtu_changed;
        if (SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type)
        {
            on_mtu_changed = associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_mtu_changed;
        }
        else if (SBEVDR_BLE_CENTRAL == associated_node->ctx->device_cfg->device_type)
        {
            on_mtu_changed = associated_node->ctx->device_cfg->central_cfg.callbacks.on_mtu_changed;
        }
        else
        {
            on_mtu_changed = NULL;
        }

        if (on_mtu_changed != NULL)
        {
            on_mtu_changed(associated_node->ctx, conn_node->ctx);
        }

        SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: MTU changed to %u bytes",
                         associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                         associated_node->ctx->device_cfg->device_id,
                         conn_id,
                         conn_node->ctx->mtu_size);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_conn_param_changed(const uint16_t conn_id)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node_by_conn_handle(conn_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if ((associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL) && (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_CENTRAL))
        {
            /* Should not be possible, but just in case */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        /* Locate connection context */
        sid_ble_ext_conn_list_node_t * const conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(associated_node->ctx, conn_id);
        if (NULL == conn_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Notify user about the connection parameters update */
        sid_ble_ext_on_conn_param_changed_cb_t on_conn_param_changed;
        if (SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type)
        {
            on_conn_param_changed = associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_conn_param_changed;
        }
        else if (SBEVDR_BLE_CENTRAL == associated_node->ctx->device_cfg->device_type)
        {
            on_conn_param_changed = associated_node->ctx->device_cfg->central_cfg.callbacks.on_conn_param_changed;
        }
        else
        {
            on_conn_param_changed = NULL;
        }

        if (on_conn_param_changed != NULL)
        {
            on_conn_param_changed(associated_node->ctx, conn_node->ctx);
        }

        SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: connection parameters updated:",
                         associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                         associated_node->ctx->device_cfg->device_id,
                         conn_id);
        SID_PAL_LOG_INFO("     - Connection Interval: %u ms", (uint32_t)SID_STM32_BLE_CONN_INTERVAL_UNITS_TO_MS(conn_node->ctx->conn_interval));
        SID_PAL_LOG_INFO("     - Connection latency:  %u",    conn_node->ctx->conn_latency);
        SID_PAL_LOG_INFO("     - Supervision Timeout: %u ms", SID_STM32_BLE_CONN_SUPERVISION_UNITS_TO_MS(conn_node->ctx->supervision_timeout));

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_conn_params_update_req(const uint16_t conn_id, const sid_ble_ext_proposed_conn_params_t * const proposed_params,
                                                                                              sid_ble_ext_accepted_conn_params_t * const out_accepted_params, tBleStatus * const out_le_resp_code)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node_by_conn_handle(conn_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if ((associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL) && (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_CENTRAL))
        {
            /* Should not be possible, but just in case */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        /* Locate connection context */
        sid_ble_ext_conn_list_node_t * const conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(associated_node->ctx, conn_id);
        if (NULL == conn_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Notify user about the connection parameters update request */
        sid_ble_ext_on_conn_param_update_req_cb_t on_conn_param_update_req;
        if (SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type)
        {
            on_conn_param_update_req = associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_conn_param_update_req;
        }
        else if (SBEVDR_BLE_CENTRAL == associated_node->ctx->device_cfg->device_type)
        {
            on_conn_param_update_req = associated_node->ctx->device_cfg->central_cfg.callbacks.on_conn_param_update_req;
        }
        else
        {
            /* Invalid device type - only peripherals and centrals can handle this event */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        if (on_conn_param_update_req != NULL)
        {
            /* Call device-speicifc handler */
            on_conn_param_update_req(associated_node->ctx, conn_node->ctx, proposed_params, out_accepted_params, out_le_resp_code);
        }
        else
        {
            /* Use generic handler */
            const sid_ble_ext_conn_params_limits_t generic_limits ={
                .interval_min       = SID_BLE_HCI_CONN_INTERVAL_LOWER_LIMIT,
                .interval_max       = SID_BLE_HCI_CONN_INTERVAL_UPPER_LIMIT,
                .latency_max        = SID_BLE_HCI_CONN_LATENCY_UPPER_LIMIT,
                .reasonable_timeout = SID_STM32_BLE_REASONABLE_CONN_TIMEOUT,
                .ce_length_min      = SID_BLE_HCI_CONN_EVENT_LENGTH_LOWER_LIMIT,
                .ce_length_max      = SID_BLE_HCI_CONN_EVENT_LENGTH_UPPER_LIMIT,
            };

            err = sid_stm32wba_ble_adapter_util_validate_proposed_conn_params(proposed_params, &generic_limits, out_accepted_params, out_le_resp_code);
            if (err != SID_ERROR_NONE)
            {
                break;
            }
        }

        if (BLE_STATUS_SUCCESS == *out_le_resp_code)
        {
            SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: connection parameters update request accepted:",
                             associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                             associated_node->ctx->device_cfg->device_id,
                             conn_id);
            SID_PAL_LOG_INFO("     - Connection Interval Min: %u ms", (uint32_t)SID_STM32_BLE_CONN_INTERVAL_UNITS_TO_MS(out_accepted_params->interval_min));
            SID_PAL_LOG_INFO("     - Connection Interval Max: %u ms", (uint32_t)SID_STM32_BLE_CONN_INTERVAL_UNITS_TO_MS(out_accepted_params->interval_max));
            SID_PAL_LOG_INFO("     - Connection latency:      %u",    out_accepted_params->latency_max);
            SID_PAL_LOG_INFO("     - Supervision Timeout:     %u ms", SID_STM32_BLE_CONN_SUPERVISION_UNITS_TO_MS(out_accepted_params->timeout));
        }
        else
        {
             SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: connection parameters update request rejected, status 0x%02X",
                associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                associated_node->ctx->device_cfg->device_id,
                conn_id,
                *out_le_resp_code
            );
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_encryption_changed(const uint16_t conn_id)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node_by_conn_handle(conn_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if ((associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL) && (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_CENTRAL))
        {
            /* Should not be possible, but just in case */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        /* Locate connection context */
        sid_ble_ext_conn_list_node_t * const conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(associated_node->ctx, conn_id);
        if (NULL == conn_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Notify user about the encryption state update */
        sid_ble_ext_on_encryption_changed_cb_t on_encryption_changed;
        if (SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type)
        {
            on_encryption_changed = associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_encryption_changed;
        }
        else if (SBEVDR_BLE_CENTRAL == associated_node->ctx->device_cfg->device_type)
        {
            on_encryption_changed = associated_node->ctx->device_cfg->central_cfg.callbacks.on_encryption_changed;
        }
        else
        {
            on_encryption_changed = NULL;
        }

        if (on_encryption_changed != NULL)
        {
            on_encryption_changed(associated_node->ctx, conn_node->ctx);
        }

        if (FALSE == conn_node->ctx->is_secure)
        {
            SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: encryption disabled",
                             associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                             associated_node->ctx->device_cfg->device_id,
                             conn_id);
        }
        else
        {
            uint8_t security_mode = 0u;
            uint8_t security_level = 0u;

            (void)aci_gap_get_security_level(conn_id, &security_mode, &security_level);

            SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: encryption enabled: mode %u, level %u",
                         associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                         associated_node->ctx->device_cfg->device_id,
                         conn_id,
                         security_mode, security_level);
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_pairing_keypress_notification(const uint16_t conn_id, const uint8_t notification_type)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node_by_conn_handle(conn_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if ((associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL) && (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_CENTRAL))
        {
            /* Should not be possible, but just in case */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        /* Locate connection context */
        sid_ble_ext_conn_list_node_t * const conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(associated_node->ctx, conn_id);
        if (NULL == conn_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Notify user about the keypress event */
        sid_ble_ext_on_pairing_keypress_notification_cb_t on_pairing_keypress_notification;
        if (SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type)
        {
            on_pairing_keypress_notification = associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_pairing_keypress_notification;
        }
        else if (SBEVDR_BLE_CENTRAL == associated_node->ctx->device_cfg->device_type)
        {
            on_pairing_keypress_notification = associated_node->ctx->device_cfg->central_cfg.callbacks.on_pairing_keypress_notification;
        }
        else
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        if (on_pairing_keypress_notification != NULL)
        {
            on_pairing_keypress_notification(associated_node->ctx, conn_node->ctx, notification_type);
        }

        SID_BLE_USER_LOG_DEBUG("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: keypress notification 0x%02X",
                               associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                               associated_node->ctx->device_cfg->device_id,
                               conn_id,
                               notification_type);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_pairing_pass_key_request(const uint16_t conn_id, uint32_t * const out_passkey)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node_by_conn_handle(conn_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if ((associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL) && (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_CENTRAL))
        {
            /* Should not be possible, but just in case */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        /* Locate connection context */
        sid_ble_ext_conn_list_node_t * const conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(associated_node->ctx, conn_id);
        if (NULL == conn_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Request the user to generate a passkey */
        sid_ble_ext_on_pairing_pass_key_request_cb_t on_pairing_pass_key_request;
        if (SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type)
        {
            on_pairing_pass_key_request = associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_pairing_pass_key_request;
        }
        else if (SBEVDR_BLE_CENTRAL == associated_node->ctx->device_cfg->device_type)
        {
            on_pairing_pass_key_request = associated_node->ctx->device_cfg->central_cfg.callbacks.on_pairing_pass_key_request;
        }
        else
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        if (on_pairing_pass_key_request != NULL)
        {
            on_pairing_pass_key_request(associated_node->ctx, conn_node->ctx, out_passkey);
        }
        else
        {
            /* Reject the request since there's no user-defined method to generate and display the passkey */
            SID_PAL_LOG_ERROR("Virtual BLE device \"%s\" (0x%02X) - passkey callback is null, pairing aborted",
                               associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                               associated_node->ctx->device_cfg->device_id);
            err = SID_ERROR_NOSUPPORT;
            break;
        }

        /* Check the passkey is in the valid range */
        if (*out_passkey > 999999u)
        {
            SID_PAL_LOG_ERROR("Passkey %u is invalid - it should not exceed 6 digits", *out_passkey);
            err = SID_ERROR_INVALID_RESPONSE;
            break;
        }

        SID_BLE_USER_LOG_DEBUG("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: passkey set to %06u",
                               associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                               associated_node->ctx->device_cfg->device_id,
                               conn_id,
                               *out_passkey);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_pairing_numeric_comparison_value(const uint16_t conn_id, const uint32_t numeric_value, uint32_t * const out_numeric_value_matches)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node_by_conn_handle(conn_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if ((associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL) && (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_CENTRAL))
        {
            /* Should not be possible, but just in case */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        /* Locate connection context */
        sid_ble_ext_conn_list_node_t * const conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(associated_node->ctx, conn_id);
        if (NULL == conn_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Send the numeric comparison value to the user */
        sid_ble_ext_on_pairing_numeric_comparison_value_cb_t on_pairing_numeric_comparison_value;
        if (SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type)
        {
            on_pairing_numeric_comparison_value = associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_pairing_numeric_comparison_value;
        }
        else if (SBEVDR_BLE_CENTRAL == associated_node->ctx->device_cfg->device_type)
        {
            on_pairing_numeric_comparison_value = associated_node->ctx->device_cfg->central_cfg.callbacks.on_pairing_numeric_comparison_value;
        }
        else
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        if (on_pairing_numeric_comparison_value != NULL)
        {
            on_pairing_numeric_comparison_value(associated_node->ctx, conn_node->ctx, numeric_value, out_numeric_value_matches);
        }
        else
        {
            /* Reject the request since there's no user-defined method to generate and display the passkey */
            SID_PAL_LOG_ERROR("Virtual BLE device \"%s\" (0x%02X) - numeric comparison callback is null, pairing aborted",
                               associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                               associated_node->ctx->device_cfg->device_id);
            err = SID_ERROR_NOSUPPORT;
            break;
        }

        SID_BLE_USER_LOG_DEBUG("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: numeric comparison is %s",
                               associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                               associated_node->ctx->device_cfg->device_id,
                               conn_id,
                               *out_numeric_value_matches != FALSE ? "ok" : "rejected");

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_pairing_complete(const uint16_t conn_id, tBleStatus status, uint8_t reason)
{
    sid_error_t err = SID_ERROR_GENERIC;
    tBleStatus ble_status;

    do
    {
        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node_by_conn_handle(conn_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if ((associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL) && (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_CENTRAL))
        {
            /* Should not be possible, but just in case */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        /* Locate connection context */
        sid_ble_ext_conn_list_node_t * const conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(associated_node->ctx, conn_id);
        if (NULL == conn_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Check if the peer is bonded */
        uint8_t peer_identity_addr_type;
        switch (conn_node->ctx->peer_addr_type)
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

        uint8_t resolved_peer_addr_type;
        uint8_t resolved_peer_address[6];
        ble_status = aci_gap_check_bonded_device(peer_identity_addr_type, conn_node->ctx->peer_identity, &resolved_peer_addr_type, resolved_peer_address);
        if (BLE_STATUS_SUCCESS == ble_status)
        {
            /**
             * The peer is bonded - check the bonding belongs to the current Virtual BLE Device. This check is necessary to eliminate the possibility of pairing via Virtual Devie A and then reuse the bonding
             * for unauthorized secure connections to the Virtual Device B.
             */
            err = _ble_adapter_user_search_bond_pair_kv_record(associated_node->ctx->device_cfg->device_id, peer_identity_addr_type, conn_node->ctx->peer_identity, NULL);

            /* Check if the connection can be accepted */
            if (err != SID_ERROR_NONE)
            {
                if (SID_ERROR_NOT_FOUND == err)
                {
                    /* Identity info for the remote peer was not found in the non-volitile context of this virtual device - search in the contexts of the other virtual devices */
                    for (uint32_t i = 0u; i < sid_ble_drv_ctx.cfg->custom_profiles.num_virtual_devices; i++)
                    {
                        const sid_ble_ext_virtual_device_t * const dev = &sid_ble_drv_ctx.cfg->custom_profiles.virtual_devices[i];

                        /* Search for the other peripheral devices */
                        if ((SBEVDR_BLE_PERIPHERAL == dev->device_type) && (dev->device_id != associated_node->ctx->device_cfg->device_id))
                        {
                            sid_error_t kv_search_err = _ble_adapter_user_search_bond_pair_kv_record(dev->device_id, peer_identity_addr_type, conn_node->ctx->peer_identity, NULL);

                            if (SID_ERROR_NONE == kv_search_err)
                            {
                                /* Found current remote peer in the non-volatile context of the other Virtual BLE device */
                                SID_PAL_LOG_WARNING("Connection 0x%04X is bonded to the Virtual BLE device \"%s\" (0x%02X), not to the Virtual BLE device \"%s\" (0x%02X)",
                                                    conn_id,
                                                    dev->device_name != NULL ? dev->device_name : "<unnamed>",
                                                    dev->device_id,
                                                    associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                                                    associated_node->ctx->device_cfg->device_id
                                                    );
                                err = SID_ERROR_AUTHENTICATION_FAIL;
                                break;
                            }
                        }
                    }
                }

                if (SID_ERROR_AUTHENTICATION_FAIL == err)
                {
                    /* Found current remote peer in the non-volatile context of the other Virtual BLE device - reject the connection*/
                    SID_PAL_LOG_ERROR("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: rejected because the bonding was performed with a different Virtual BLE Device",
                                      associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                                      associated_node->ctx->device_cfg->device_id,
                                      conn_id);
                }
                else if (SID_ERROR_NOT_FOUND == err)
                {
                    /* If the error is still SID_ERROR_NOT_FOUND this means the non-volatile context is not stored yet, probably that's the very first connection of this peer */
                    err = SID_ERROR_OUT_OF_RESOURCES;
                    for (uint32_t i = 0u; i < SID_STM32_BLE_BONDING_LIST_MAX_ENTRIES; i++)
                    {
                        const uint16_t bond_pair_kv_key = _ble_adapter_user_get_storage_kv_bond_pair_key(associated_node->ctx->device_cfg->device_id, (uint16_t)i);
                        uint32_t dummy_len;
                        sid_error_t free_slot_err;

                        /* Check if this slot is free */
                        free_slot_err = sid_pal_storage_kv_record_get_len(STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP, bond_pair_kv_key, &dummy_len);

                        if (SID_ERROR_NOT_FOUND == free_slot_err)
                        {
                            /* Slot is free, store the record here */
                            sid_pal_ble_ext_peer_identity_addr_t kv_bond_pair = {
                                .identity_address_type = peer_identity_addr_type,
                            };
                            SID_STM32_UTIL_fast_memcpy(kv_bond_pair.identity_address, conn_node->ctx->peer_identity, sizeof(kv_bond_pair.identity_address));

                            /* Store non-volatile context */
                            err = sid_pal_storage_kv_record_set(STORAGE_KV_SID_BLE_ADAPTER_BOND_PAIR_GROUP, bond_pair_kv_key, &kv_bond_pair, sizeof(kv_bond_pair));
                            if (err != SID_ERROR_NONE)
                            {
                                /* Connection security can be reduced due to that */
                                SID_PAL_LOG_ERROR("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: failed to store non-volatile bonding context. Error %d",
                                                  associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                                                  associated_node->ctx->device_cfg->device_id,
                                                  conn_id,
                                                  (int32_t)err);
                            }
                            else
                            {
                                SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: stored non-volatile bonding context",
                                                  associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                                                  associated_node->ctx->device_cfg->device_id,
                                                  conn_id);
                            }

                            /* Successfull or not, terminate the loop */
                            break;
                        }
                    }
                }
                else
                {
                    /* Nothing to do here */
                }

                /* If there's still an error at this point there's nothing more we can do */
                if (err != SID_ERROR_NONE)
                {
                    /* Terminate the connection */
                    ble_status = aci_gap_terminate(conn_id, SID_BLE_HCI_STATUS_REMOTE_SIDE_TERMINATED);
                    if (ble_status != BLE_STATUS_SUCCESS)
                    {
                        SID_PAL_LOG_ERROR("Failed to terminate BLE connection 0x%04X. Error 0x%02X. Virtual BLE device \"%s\" (0x%02X) is blocked for security reasons",
                                          conn_id,
                                          ble_status,
                                          associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                                          associated_node->ctx->device_cfg->device_id);

                        /* Lock the virtual device to prevent unauthenticated access */
                        associated_node->ctx->state = SBEVD_STATE_FATAL_ERROR;
                    }

                    /* Report back no error, otherwise the generic pairing error handler will erase the bond */
                    err = SID_ERROR_NONE;

                    /* Something went wrong - abort further processing */
                    break;
                }
            }
        }
        else
        {
            /* The device is not bonded, meaning the pairing was performed from scratch - not further actions needed since it's techinically impossible to reuse the generated keys for the other connections */
        }

        /* Notify the user about the pairing outcomes */
        sid_ble_ext_on_pairing_complete_cb_t on_pairing_complete;
        if (SBEVDR_BLE_PERIPHERAL == associated_node->ctx->device_cfg->device_type)
        {
            on_pairing_complete = associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_pairing_complete;
        }
        else if (SBEVDR_BLE_CENTRAL == associated_node->ctx->device_cfg->device_type)
        {
            on_pairing_complete = associated_node->ctx->device_cfg->central_cfg.callbacks.on_pairing_complete;
        }
        else
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        if (on_pairing_complete != NULL)
        {
            on_pairing_complete(associated_node->ctx, conn_node->ctx, status, reason);
        }

        if (SID_BLE_HCI_STATUS_SUCCESS == status)
        {
            SID_PAL_LOG_INFO("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: pairing succeeded",
                             associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                             associated_node->ctx->device_cfg->device_id,
                             conn_id);
        }
        else
        {
            SID_PAL_LOG_WARNING("Virtual BLE device \"%s\" (0x%02X), connection 0x%04X: pairing rejected with status 0x%02X, reason 0x%02X",
                                associated_node->ctx->device_cfg->device_name != NULL ? associated_node->ctx->device_cfg->device_name : "<unnamed>",
                                associated_node->ctx->device_cfg->device_id,
                                conn_id,
                                status,
                                reason);
        }

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_stm32wba_ble_adapter_user_on_authorization_req(const uint16_t conn_id)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* Locate associated node in the list of the active devices */
        sid_ble_ext_virtual_device_node_t * const associated_node = _ble_adapter_user_search_device_node_by_conn_handle(conn_id);
        if (NULL == associated_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if (associated_node->ctx->device_cfg->device_type != SBEVDR_BLE_PERIPHERAL)
        {
            /* Should not be possible, but just in case */
            err = SID_ERROR_NO_PERMISSION;
            break;
        }

        /* Locate connection context */
        sid_ble_ext_conn_list_node_t * const conn_node = _ble_adapter_user_search_connection_node_by_conn_handle(associated_node->ctx, conn_id);
        if (NULL == conn_node)
        {
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Notify the user about the pairing outcomes */
        if (associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_authorization_req != NULL)
        {
            err = associated_node->ctx->device_cfg->peripheral_cfg.callbacks.on_authorization_req(associated_node->ctx, conn_node->ctx);
        }
        else
        {
            /* No user-defined callback exists, grant authorization by default */
            err = SID_ERROR_NONE;
        }

        /* Done */
    } while (0);

    return err;
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_ble_adapter_ext_create_extended_ifc(const sid_pal_ble_adapter_extended_interface_t * * const ext_ifc_handle)
{
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        if (NULL == ext_ifc_handle)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

#if (SID_STM32_BLE_COEXISTENCE_MODE != SID_STM32_BLE_COEXISTENCE_MODE_NONE) && (SID_STM32_BLE_COEXISTENCE_MODE != SID_STM32_BLE_COEXISTENCE_MODE_ROUTE_NON_SID)
        *ext_ifc_handle = &user_ble_ifc;
        err = SID_ERROR_NONE;
#else
        SID_PAL_LOG_ERROR("User-defined BLE profiles are not supported by the current config");
        *ext_ifc_handle = NULL;
        err = SID_ERROR_NOSUPPORT;
#endif /* SID_STM32_BLE_COEXISTENCE_MODE_NONE */

    } while (0);

    return err;
}
