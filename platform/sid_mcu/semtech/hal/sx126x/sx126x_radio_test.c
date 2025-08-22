/*
 * Copyright (c) 2019-2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * This file supports diagnostic functionality specific to FSK
 */

#include "sx126x.h"
#include "sx126x_radio.h"

#include <sid_pal_radio_ifc.h>
#include <semtech_radio_ifc.h>

#include <sid_stm32_common_utils.h>

int32_t semtech_radio_set_trim_cap_val(uint16_t trim)
{
    return set_radio_sx126x_trim_cap_val(trim);
}

uint16_t semtech_radio_get_trim_cap_val(void)
{
    const halo_drv_semtech_ctx_t *drv_ctx = sx126x_get_drv_ctx();
    return drv_ctx->trim;
}

int32_t semtech_radio_get_tx_power_range(int8_t *max_tx_power, int8_t *min_tx_power)
{
    const halo_drv_semtech_ctx_t *drv_ctx = sx126x_get_drv_ctx();

    if (drv_ctx->config->id == SEMTECH_ID_SX1261) {
        *max_tx_power = SX1261_MAX_TX_POWER;
        *min_tx_power = SX1261_MIN_TX_POWER;
         return RADIO_ERROR_NONE;
    }

    if (drv_ctx->config->id == SEMTECH_ID_SX1262) {
        *max_tx_power = SX1262_MAX_TX_POWER;
        *min_tx_power = SX1262_MIN_TX_POWER;
         return RADIO_ERROR_NONE;
    }

    return RADIO_ERROR_INVALID_PARAMS;
}

int32_t semtech_radio_get_pa_config(semtech_radio_pa_cfg_t *cfg)
{
    const halo_drv_semtech_ctx_t *drv_ctx = sx126x_get_drv_ctx();

    if ((NULL == drv_ctx) || (NULL == drv_ctx->config) || (NULL == cfg)) {
        return RADIO_ERROR_INVALID_PARAMS;
    }

    radio_sx126x_pa_dynamic_cfg_t cur_cfg;

    get_radio_sx126x_pa_config(&cur_cfg);

    cfg->pa_duty_cycle = cur_cfg.pa_duty_cycle;
    cfg->hp_max        = cur_cfg.hp_max;
    cfg->pa_lut        = cur_cfg.pa_lut;
    cfg->tx_power      = cur_cfg.tx_power_reg;
    cfg->ramp_time     = cur_cfg.ramp_time;

    switch (drv_ctx->config->id)
    {
        case SEMTECH_ID_SX1261:
            cfg->device_sel = 0x01u;
            break;

        case SEMTECH_ID_SX1262:
            cfg->device_sel = 0x00u;
            break;

        default:
            return RADIO_ERROR_INVALID_PARAMS;
    }

#if SX126X_RADIO_CFG_USE_EXTERNAL_PA
    cfg->enable_ext_pa = drv_ctx->pa_cfg.use_ext_pa;
#else
    cfg->enable_ext_pa = false;
#endif /* SX126X_RADIO_CFG_USE_EXTERNAL_PA */

    return RADIO_ERROR_NONE;
}

const uint8_t lora_sf[] = { SX126X_LORA_SF5,  SX126X_LORA_SF6,  SX126X_LORA_SF7,
                            SX126X_LORA_SF8,  SX126X_LORA_SF9,  SX126X_LORA_SF10,
                            SX126X_LORA_SF11, SX126X_LORA_SF12};

const uint8_t lora_bw[] = { SX126X_LORA_BW_500, SX126X_LORA_BW_250, SX126X_LORA_BW_125,
                            SX126X_LORA_BW_062, SX126X_LORA_BW_041, SX126X_LORA_BW_031,
                            SX126X_LORA_BW_020, SX126X_LORA_BW_015, SX126X_LORA_BW_010,
                            SX126X_LORA_BW_007};

const uint8_t lora_cr[] = { SX126X_LORA_CR_4_5, SX126X_LORA_CR_4_6, SX126X_LORA_CR_4_7,
                            SX126X_LORA_CR_4_8};

const uint8_t lora_cad_symbol[] = { SX126X_CAD_01_SYMB, SX126X_CAD_02_SYMB,
                                    SX126X_CAD_04_SYMB, SX126X_CAD_08_SYMB,
                                    SX126X_CAD_16_SYMB};

const uint8_t lora_cad_exit_mode[] = {SX126X_CAD_ONLY, SX126X_CAD_RX, SX126X_CAD_LBT};

#define LORA_SF_NUM SID_STM32_UTIL_ARRAY_SIZE(lora_sf)
#define LORA_BW_NUM SID_STM32_UTIL_ARRAY_SIZE(lora_bw)
#define LORA_CR_NUM SID_STM32_UTIL_ARRAY_SIZE(lora_cr)
#define LORA_CAD_SYMBOL_NUM SID_STM32_UTIL_ARRAY_SIZE(lora_cad_symbol)
#define LORA_CAD_EXIT_MODE_NUM SID_STM32_UTIL_ARRAY_SIZE(lora_cad_exit_mode)


int32_t semtech_radio_get_lora_sf(uint8_t idx, uint8_t *sf)
{
    if (idx >= LORA_SF_NUM) {
        return RADIO_ERROR_INVALID_PARAMS;
    }
    *sf = lora_sf[idx];
    return RADIO_ERROR_NONE;
}

int16_t semtech_radio_get_lora_sf_idx(uint8_t sf)
{
    for (int i = 0; i < LORA_SF_NUM; i++) {
        if (lora_sf[i] == sf)
            return i;
    }
    return -1;
}


int32_t semtech_radio_get_lora_bw(uint8_t idx, uint8_t *bw)
{
    if (idx >= LORA_BW_NUM) {
        return RADIO_ERROR_INVALID_PARAMS;
    }
    *bw = lora_bw[idx];
    return RADIO_ERROR_NONE;
}

int16_t semtech_radio_get_lora_bw_idx(uint8_t bw)
{
    for (int i = 0; i < LORA_BW_NUM; i++) {
        if (lora_bw[i] == bw) {
            return i;
        }
    }
    return -1;
}

int32_t semtech_radio_get_lora_cr(uint8_t idx, uint8_t *cr)
{
    if (idx >= LORA_CR_NUM) {
        return RADIO_ERROR_INVALID_PARAMS;
    }
    *cr = lora_cr[idx];
    return RADIO_ERROR_NONE;
}

int16_t semtech_radio_get_lora_cr_idx(uint8_t cr)
{
    for (int i = 0; i < LORA_CR_NUM; i++) {
        if (lora_cr[i] == cr) {
            return i;
        }
    }
    return -1;
}

int32_t semtech_radio_get_lora_cad_symbol(uint8_t idx, uint8_t *sym)
{
    if (idx >= LORA_CAD_SYMBOL_NUM) {
        return RADIO_ERROR_INVALID_PARAMS;
    }
    *sym = lora_cad_symbol[idx];
    return RADIO_ERROR_NONE;
}

int16_t semtech_radio_get_lora_cad_symbol_idx(uint8_t sym)
{
    for (int i = 0; i < LORA_CAD_SYMBOL_NUM; i++) {
        if (lora_cad_symbol[i] == sym) {
            return i;
        }
    }
    return -1;
}

int32_t semtech_radio_get_lora_cad_exit_mode(uint8_t idx, uint8_t *em)
{
    if (idx >= LORA_CAD_EXIT_MODE_NUM) {
        return RADIO_ERROR_INVALID_PARAMS;
    }
    *em = lora_cad_exit_mode[idx];
    return RADIO_ERROR_NONE;
}

int16_t semtech_radio_get_lora_cad_exit_mode_idx(uint8_t em)
{
    for (int i = 0; i < LORA_CAD_EXIT_MODE_NUM; i++) {
        if (lora_cad_exit_mode[i] == em) {
            return i;
        }
    }
    return -1;
}



const uint8_t fsk_mod_shaping[] = {SX126X_GFSK_PULSE_SHAPE_OFF, SX126X_GFSK_PULSE_SHAPE_BT_03,
                                   SX126X_GFSK_PULSE_SHAPE_BT_05, SX126X_GFSK_PULSE_SHAPE_BT_07,
                                   SX126X_GFSK_PULSE_SHAPE_BT_1};

const uint8_t fsk_bw[] = {SX126X_GFSK_BW_4800,   SX126X_GFSK_BW_5800,   SX126X_GFSK_BW_7300,
                          SX126X_GFSK_BW_9700,   SX126X_GFSK_BW_11700,  SX126X_GFSK_BW_14600,
                          SX126X_GFSK_BW_19500,  SX126X_GFSK_BW_23400,  SX126X_GFSK_BW_29300,
                          SX126X_GFSK_BW_39000,  SX126X_GFSK_BW_46900,  SX126X_GFSK_BW_58600,
                          SX126X_GFSK_BW_78200,  SX126X_GFSK_BW_93800,  SX126X_GFSK_BW_117300,
                          SX126X_GFSK_BW_156200, SX126X_GFSK_BW_187200, SX126X_GFSK_BW_234300,
                          SX126X_GFSK_BW_312000, SX126X_GFSK_BW_373600, SX126X_GFSK_BW_467000};

const uint8_t fsk_addr_comp[] = {SX126X_GFSK_ADDRESS_FILTERING_DISABLE, SX126X_GFSK_ADDRESS_FILTERING_NODE_ADDRESS,
                                 SX126X_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES};

const uint8_t fsk_preamble_detect[] = {SX126X_GFSK_PREAMBLE_DETECTOR_OFF, SX126X_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
                                       SX126X_GFSK_PREAMBLE_DETECTOR_MIN_16BITS, SX126X_GFSK_PREAMBLE_DETECTOR_MIN_24BITS,
                                       SX126X_GFSK_PREAMBLE_DETECTOR_MIN_32BITS};

const uint8_t fsk_crc_type[] = {SX126X_GFSK_CRC_OFF, SX126X_GFSK_CRC_1_BYTE, SX126X_GFSK_CRC_2_BYTES,
                                SX126X_GFSK_CRC_1_BYTE_INV, SX126X_GFSK_CRC_2_BYTES_INV};

#define FSK_MOD_SHAPING_PARAMS_NUM     SID_STM32_UTIL_ARRAY_SIZE(fsk_mod_shaping)
#define FSK_BW_NUM                     SID_STM32_UTIL_ARRAY_SIZE(fsk_bw)
#define FSK_ADDR_COMP_NUM              SID_STM32_UTIL_ARRAY_SIZE(fsk_addr_comp)
#define FSK_PREAMBLE_DETECT_NUM        SID_STM32_UTIL_ARRAY_SIZE(fsk_preamble_detect)
#define FSK_CRC_TYPES_NUM              SID_STM32_UTIL_ARRAY_SIZE(fsk_crc_type)

int32_t semtech_radio_get_fsk_mod_shaping(uint8_t idx, uint8_t *ms)
{
    if (idx >= FSK_MOD_SHAPING_PARAMS_NUM) {
        return RADIO_ERROR_INVALID_PARAMS;
    }
    *ms = fsk_mod_shaping[idx];
    return RADIO_ERROR_NONE;
}

int16_t semtech_radio_get_fsk_mod_shaping_idx(uint8_t ms)
{
    for (int i = 0; i < FSK_MOD_SHAPING_PARAMS_NUM; i++) {
        if (fsk_mod_shaping[i] == ms) {
            return i;
        }
    }
    return -1;
}

int32_t semtech_radio_get_fsk_bw(uint8_t idx, uint8_t *bw)
{
    if (idx >= FSK_BW_NUM) {
        return RADIO_ERROR_INVALID_PARAMS;
    }
    *bw = fsk_bw[idx];
    return RADIO_ERROR_NONE;
}

int16_t semtech_radio_get_fsk_bw_idx(uint8_t bw)
{
    for (int i = 0; i < FSK_BW_NUM; i++) {
        if (fsk_bw[i] == bw) {
            return i;
        }
    }
    return -1;
}

int32_t semtech_radio_get_fsk_addr_comp(uint8_t idx, uint8_t *ac)
{
    if (idx >= FSK_ADDR_COMP_NUM) {
        return RADIO_ERROR_INVALID_PARAMS;
    }
    *ac = fsk_addr_comp[idx];
    return RADIO_ERROR_NONE;
}


int16_t semtech_radio_get_fsk_addr_comp_idx(uint8_t ac)
{
    for (int i = 0; i < FSK_ADDR_COMP_NUM; i++) {
        if (fsk_addr_comp[i] == ac) {
            return i;
        }
    }
    return -1;
}

int32_t semtech_radio_get_fsk_preamble_detect(uint8_t idx, uint8_t *pd)
{
    if (idx >= FSK_PREAMBLE_DETECT_NUM) {
        return RADIO_ERROR_INVALID_PARAMS;
    }
    *pd = fsk_preamble_detect[idx];
    return RADIO_ERROR_NONE;
}

int16_t semtech_radio_get_fsk_preamble_detect_idx(uint8_t pd)
{
    for (int i = 0; i < FSK_PREAMBLE_DETECT_NUM; i++) {
        if (fsk_preamble_detect[i] == pd) {
            return i;
        }
    }
    return -1;
}

int32_t semtech_radio_get_fsk_crc_type(uint8_t idx, uint8_t *crc)
{
    if (idx >= FSK_CRC_TYPES_NUM) {
        return RADIO_ERROR_INVALID_PARAMS;
    }
    *crc = fsk_crc_type[idx];
    return RADIO_ERROR_NONE;
}


int16_t semtech_radio_get_fsk_crc_type_idx(uint8_t crc)
{
    for (int i = 0; i < FSK_CRC_TYPES_NUM; i++) {
        if (fsk_crc_type[i] == crc) {
            return i;
        }
    }
    return -1;
}

