/**
  ******************************************************************************
  * @file    s2_lp_radio.c
  * @brief   Handling of the S2-LP transceiver for Sidewalk application
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

#include "s2_lp_radio.h"
#include "s2_lp_radio_config.h"
#include "s2_lp_radio_hal.h"

#include <sid_fsk_phy_cfg.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_radio_ifc.h>
#include <sid_pal_timer_ifc.h>
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>

#include <sid_stm32_common_utils.h>

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
#  include "main.h"
#endif

/* LPM handling */
#include "app_conf.h"
#if (CFG_LPM_LEVEL != 0)
#  include <stm32_lpm.h>
#endif /* (CFG_LPM_LEVEL != 0) */

/* Private defines -----------------------------------------------------------*/

#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_3
#  error "S2-LP radio supports FSK link only, LoRa is not supported"
#endif

#define S2LP_RADIO_EXPECTED_DEVICE_PART_NUMBER   (0x03u)

#define S2LP_RADIO_SIDEWALK_FSK_PREAMBLE_SYMB    (0x55u)

#define S2LP_RADIO_ESSENTIAL_FSK_IRQ_MASK        ( S2_LP_IC_IRQ_MASK_RX_DATA_READY        \
                                                 | S2_LP_IC_IRQ_MASK_TX_DATA_SENT         \
                                                 | S2_LP_IC_IRQ_MASK_CRC_ERROR            \
                                                 | S2_LP_IC_IRQ_MASK_TX_FIFO_ERROR        \
                                                 | S2_LP_IC_IRQ_MASK_RX_FIFO_ERROR        \
                                                 | S2_LP_IC_IRQ_MASK_RX_FIFO_ALMOST_FULL  \
                                                 | S2_LP_IC_IRQ_MASK_RX_TIMEOUT           \
                                                 )

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
/* Enable additional IRQs needed for RF operations profiling */
#  define S2LP_RADIO_BASE_FSK_IRQ_MASK           (S2LP_RADIO_ESSENTIAL_FSK_IRQ_MASK \
                                                 | S2_LP_IC_IRQ_MASK_TX_START_TIME  \
                                                 | S2_LP_IC_IRQ_MASK_RX_START_TIME  \
                                                 )
#else
#  define S2LP_RADIO_BASE_FSK_IRQ_MASK           (S2LP_RADIO_ESSENTIAL_FSK_IRQ_MASK)
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */

#define S2LP_RADIO_DEBUG_FSK_IRQ_MASK            ( S2_LP_IC_IRQ_MASK_VALID_PREAMBLE \
                                                 | S2_LP_IC_IRQ_MASK_VALID_SYNC     \
                                                 )

#ifndef S2LP_RADIO_PHY_DEBUG
# define S2LP_RADIO_DEFAULT_FSK_IRQ_MASK         (S2LP_RADIO_BASE_FSK_IRQ_MASK)
#else
/* Enable additional radio IRQs required for phy layer debug */
# define S2LP_RADIO_DEFAULT_FSK_IRQ_MASK         (S2LP_RADIO_BASE_FSK_IRQ_MASK | S2LP_RADIO_DEBUG_FSK_IRQ_MASK)
#endif /* S2LP_RADIO_PHY_DEBUG */

#define S2LP_RADIO_SUPPORTED_SID_IRQS            ( RADIO_IRQ_TX_DONE         \
                                                 | RADIO_IRQ_RX_DONE         \
                                                 | RADIO_IRQ_PREAMBLE_DETECT \
                                                 | RADIO_IRQ_VALID_SYNC_WORD \
                                                 | RADIO_IRQ_ERROR_CRC       \
                                                 | RADIO_IRQ_TXRX_TIMEOUT    \
                                                 )

#define SID_PAL_RADIO_FSK_CRC_4_BYTES_ADCCP      (0xFFu) /*!< Missing definition in the Sidewalk SDK, use a custom value */

#define S2LP_RADIO_FSK_RX_PROCESS_SAFETY_GAP_US  (50u)   /*!< Rx window will be opened earlier by this amount of microseconds to mitigate any SW execution delays and jitter */

#define S2LP_RADIO_MIN_CHANNEL_FREE_DELAY_US     (10u)
#define S2LP_RADIO_MIN_CHANNEL_NOISE_DELAY_US    (50u)
#define S2LP_RADIO_NOISE_SAMPLE_SIZE             (32u)

#define S2LP_RADIO_SIDEWALK_FSK_CS_DURATION_US   (1200u) /*!< Carrier Sense timeout for FSK link as defined by Sidewalk specification */

#define S2LP_RADIO_ERROR_LIMIT                   (5u)     /*!< If the number of errors in a burst exceeds this limit the driver will reset the radio */
#define S2LP_RADIO_ERROR_BURST_TIME_WINDOW_MS    (10000u) /*!< Duration of the burst detection window (in ms) - even if some operations will complete successfully in less than this time after the last error they won't be treated as a recovery indication */

/* Private macros ------------------------------------------------------------*/

#ifndef S2LP_RADIO_EXTRA_LOGGING
/* Set S2LP_RADIO_EXTRA_LOGGING to 1 to enable extended logs */
#  define S2LP_RADIO_EXTRA_LOGGING (0)
#endif

#if S2LP_RADIO_EXTRA_LOGGING
#  define S2LP_RADIO_LOG_ERROR(...)   SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define S2LP_RADIO_LOG_WARNING(...) SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define S2LP_RADIO_LOG_INFO(...)    SID_PAL_LOG_INFO(__VA_ARGS__)
#  define S2LP_RADIO_LOG_DEBUG(...)   SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define S2LP_RADIO_LOG_TRACE(...)   SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define S2LP_RADIO_LOG_ERROR(...)   ((void)0u)
#  define S2LP_RADIO_LOG_WARNING(...) ((void)0u)
#  define S2LP_RADIO_LOG_INFO(...)    ((void)0u)
#  define S2LP_RADIO_LOG_DEBUG(...)   ((void)0u)
#  define S2LP_RADIO_LOG_TRACE(...)   ((void)0u)
#endif

/* Private types -------------------------------------------------------------*/

typedef struct {
    s2_lp_ic_cut_t cut_id;
    char           cut_name[7];
} s2_lp_ic_cut_info_t;

/* Private variables ---------------------------------------------------------*/

static halo_drv_s2_lp_ctx_t drv_ctx = {0};

/* Private constants ---------------------------------------------------------*/

static const uint8_t fsk_sync_word_fec_disabled[] = {0x90u, 0x4Eu}; /*!< Sidewalk's FSK sync word when FEC is deactivated */
static const uint8_t fsk_sync_word_fec_enabled[]  = {0x6Fu, 0x4Eu}; /*!< Sidewalk's FSK sync word when FEC is activated */

/* Ensure sync word sizes are identical */
static_assert(sizeof(fsk_sync_word_fec_disabled) == sizeof(fsk_sync_word_fec_enabled));

static const s2_lp_ic_cut_info_t supported_s2_lp_versions[] = {
    { S2_LP_CUT_2_0, "2.0" },
    { S2_LP_CUT_2_1, "2.1" },
    { S2_LP_CUT_3_0, "3.0" },
    { S2_LP_CUT_3_1, "3.1" },
};

/* Private function prototypes -----------------------------------------------*/

static inline int32_t  _s2lp_radio_platform_init(void);
static inline int32_t  _s2lp_radio_platform_deinit(void);
static inline int32_t  _s2lp_radio_sid_modp_to_s2lp_modp(const sid_pal_radio_fsk_modulation_params_t * const sid_mod_params, s2_lp_hal_mod_params_t * const out_s2lp_mod_params);
static inline int32_t  _s2lp_radio_sid_pcktp_to_s2lp_pcktp(const sid_pal_radio_fsk_packet_params_t * const sid_pckt_params, s2_lp_hal_pckt_params_t * const out_s2lp_pckt_params);
static inline uint32_t _s2lp_radio_get_fsk_crc_len_in_bytes(const uint32_t crc_type);
static inline int32_t  _s2lp_radio_configure_fsk_sync_words(void);
static inline void     _s2lp_radio_error_manager(const int32_t reported_err);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t _s2lp_radio_platform_init(void)
{
    int32_t err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;
    sid_error_t sid_err;

    S2LP_RADIO_LOG_DEBUG("_s2lp_radio_platform_init");

    do
    {
        /* Validate inputs */
        if (NULL == drv_ctx.config->pa_cfg_callback)
        {
           S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_init - null pa_cfg_callback!");
           err = RADIO_ERROR_INVALID_PARAMS;
           break;
        }

        /* Configure essential GPIO pins */
        hal_err = s2_lp_radio_hal_init_gpio(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_init - failed to configure essential GPIO. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Bring up SPI bus */
        sid_err = drv_ctx.config->bus_factory->create(&drv_ctx.bus_iface, drv_ctx.config->bus_factory->config);
        if ((sid_err != SID_ERROR_NONE) || (NULL == drv_ctx.bus_iface))
        {
            S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_init - error in bus_factory->create(). Error %d, iface: 0x%x", (int32_t)sid_err, drv_ctx.bus_iface);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }
        
        /* Release the radio reset line */
        hal_err = s2_lp_radio_hal_reset_radio(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_init - S2-LP reset failed. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Read out device info to check if we have a valid IC attached */
        s2_lp_ic_version_info_t ic_version;
        hal_err = s2_lp_radio_hal_get_ic_version(&drv_ctx, &ic_version);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_init - S2-LP version readout failed. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Check if S2-LP is detected at all and if the chip revision is supported by this driver */
        if (ic_version.part_number.PARTNUM != S2LP_RADIO_EXPECTED_DEVICE_PART_NUMBER)
        {
            SID_PAL_LOG_ERROR("Unable to detect an S2-LP device on SPI bus. Received part number: 0x%02X", ic_version.part_number.PARTNUM);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = RADIO_ERROR_NOT_SUPPORTED;
        const char * ic_revision_name;
        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(supported_s2_lp_versions); i++)
        {
            if (supported_s2_lp_versions[i].cut_id == ic_version.version.VERSION)
            {
                /* That's a revision we support */
                err = RADIO_ERROR_NONE;
                ic_revision_name = supported_s2_lp_versions[i].cut_name;
                break;
            }
        }

        if (RADIO_ERROR_NONE == err)
        {
            SID_PAL_LOG_INFO("Detected an S2-LP transceiver, chip revision: %s", ic_revision_name);
        }
        else
        {
            SID_PAL_LOG_ERROR("Unsupported revision of S2-LP detected, rev ID: 0x%02X", ic_version.version.VERSION);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Initialization is competed */
        S2LP_RADIO_LOG_DEBUG("_s2lp_radio_platform_init - done");
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t _s2lp_radio_platform_deinit(void)
{
    int32_t err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;
    sid_error_t sid_err;

    S2LP_RADIO_LOG_DEBUG("_s2lp_radio_platform_deinit");

    do
    {
        /* Deactivate IRQ pin */
        hal_err = s2_lp_radio_hal_disarm_irq(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
           S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_deinit - error in s2_lp_radio_hal_disarm_irq(). Error %u", (uint32_t)hal_err);
           err = RADIO_ERROR_HARDWARE_ERROR;
           break;
        }

        /* Bring down SPI bus */
        const struct sid_pal_serial_bus_iface * const spi_bus_iface = drv_ctx.bus_iface;
        if (spi_bus_iface != NULL)
        {
            if (NULL == spi_bus_iface->destroy)
            {
                S2LP_RADIO_LOG_WARNING("_s2lp_radio_platform_deinit - spi_bus_iface has no destroy() method");
                err = RADIO_ERROR_NOT_SUPPORTED;
                break;
            }

            sid_err = spi_bus_iface->destroy(spi_bus_iface);
            if (sid_err != SID_ERROR_NONE)
            {
                S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_deinit - error in spi_bus_iface->destroy(). Error %d", (int32_t)sid_err);
                err = RADIO_ERROR_IO_ERROR;
                break;
            }
        }
        else
        {
            SID_PAL_LOG_WARNING("_s2lp_radio_platform_deinit - SPI bus interface is null, bus deinitialization skipped");
        }

        /* Release GPIO pins */
        hal_err = s2_lp_radio_hal_deinit_gpio(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_deinit - failed to configure essential GPIO. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Deinitialization is competed */
        S2LP_RADIO_LOG_DEBUG("_s2lp_radio_platform_deinit - done");
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t _s2lp_radio_sid_modp_to_s2lp_modp(const sid_pal_radio_fsk_modulation_params_t * const sid_mod_params, s2_lp_hal_mod_params_t * const out_s2lp_mod_params)
{
    int32_t err = RADIO_ERROR_GENERIC;

    SID_PAL_ASSERT(sid_mod_params != NULL);
    SID_PAL_ASSERT(out_s2lp_mod_params != NULL);

    do
    {
        /* Direct copy whatever possible */
        out_s2lp_mod_params->bit_rate_bps = sid_mod_params->bit_rate;
        out_s2lp_mod_params->fdev_in_hz   = sid_mod_params->freq_dev; /* Sidewalk specifies 2-FSK as Carrier +/- (dev/2), while S2-LP defines 2-FSK as Carrier +/- Fdev */

        /* Convert SID shaping selection to the modulation type of S2-LP */
        switch (sid_mod_params->mod_shaping)
        {
            case SID_PAL_RADIO_FSK_MOD_SHAPING_OFF:
                out_s2lp_mod_params->mod_type = S2_LP_MOD_TYPE_2FSK;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_03:
                SID_PAL_LOG_ERROR("GFSK modulation with the Gaussian filter BT product of 0.3 is not supported by S2-LP");
                err = RADIO_ERROR_NOT_SUPPORTED;
                break;

            case SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_05:
                out_s2lp_mod_params->mod_type = S2_LP_MOD_TYPE_2GFSK_BT_05;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_07:
                SID_PAL_LOG_ERROR("GFSK modulation with the Gaussian filter BT product of 0.7 is not supported by S2-LP");
                err = RADIO_ERROR_NOT_SUPPORTED;
                break;

            case SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_1:
                out_s2lp_mod_params->mod_type = S2_LP_MOD_TYPE_2GFSK_BT_1;
                err = RADIO_ERROR_NONE;
                break;

            default:
                SID_PAL_LOG_ERROR("Unknown (G)FSK modulation type requested: 0x%02X", sid_mod_params->mod_shaping);
                err = RADIO_ERROR_INVALID_PARAMS;
                break;
        }

        if (err != RADIO_ERROR_NONE)
        {
            /* Propagate the error */
            break;
        }

        /* Translate Rx filter bandwidth aliases into actual bandwidth in Hz */
        switch (sid_mod_params->bandwidth)
        {
            case SID_PAL_RADIO_FSK_BW_4800:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 4800u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_5800:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 5800u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_7300:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 7300u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_9700:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 9700u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_11700:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 11700u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_14600:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 14600u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_19500:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 19500u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_23400:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 23400u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_29300:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 29300u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_39000:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 39000u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_46900:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 46900u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_58600:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 58600u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_78200:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 78200u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_93800:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 93800u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_117300:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 117300u;
                err = RADIO_ERROR_NONE;
                break;
                
            case SID_PAL_RADIO_FSK_BW_156200:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 156200u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_187200:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 187200u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_234300:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 234300u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_312000:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 312000u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_373600:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 373600u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_BW_467000:
                out_s2lp_mod_params->rx_filter_bandwidth_hz = 467000u;
                err = RADIO_ERROR_NONE;
                break;

            default:
                SID_PAL_LOG_ERROR("Unknown Rx channel filter bandwidth requested. ID: 0x%02X", sid_mod_params->bandwidth);
                err = RADIO_ERROR_INVALID_PARAMS;
                break;
        }

        if (err != RADIO_ERROR_NONE)
        {
            /* Propagate the error */
            break;
        }

        /* Everything is fine */
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t _s2lp_radio_sid_pcktp_to_s2lp_pcktp(const sid_pal_radio_fsk_packet_params_t * const sid_pckt_params, s2_lp_hal_pckt_params_t * const out_s2lp_pckt_params)
{
    int32_t err = RADIO_ERROR_GENERIC;

    SID_PAL_ASSERT(sid_pckt_params != NULL);
    SID_PAL_ASSERT(out_s2lp_pckt_params != NULL);

    do
    {
        /* Convert preamble length */
        out_s2lp_pckt_params->preamble_len_bits = (sid_pckt_params->preamble_length << 3);

        /* Convert preamble minimum bits to detect */
        switch (sid_pckt_params->preamble_min_detect)
        {
            case SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_OFF:
                out_s2lp_pckt_params->pqi_threshold = 0u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_08_BITS:
                out_s2lp_pckt_params->pqi_threshold = (8u / S2_LP_IC_PQI_TH_FACTOR);
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_16_BITS:
                out_s2lp_pckt_params->pqi_threshold = (16u / S2_LP_IC_PQI_TH_FACTOR);
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_24_BITS:
                out_s2lp_pckt_params->pqi_threshold = (24u / S2_LP_IC_PQI_TH_FACTOR);
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_32_BITS:
                out_s2lp_pckt_params->pqi_threshold = (32u / S2_LP_IC_PQI_TH_FACTOR);
                err = RADIO_ERROR_NONE;
                break;

            default:
                SID_PAL_LOG_ERROR("Unknown preamble min length setting. ID: 0x%02X", sid_pckt_params->preamble_min_detect);
                err = RADIO_ERROR_INVALID_PARAMS;
                break;
        }

        if (err != RADIO_ERROR_NONE)
        {
            /* Propagate the error */
            break;
        }

        /* Convert data whitening setting */
        switch (sid_pckt_params->crc_type)
        {
            case SID_PAL_RADIO_FSK_CRC_OFF:
                out_s2lp_pckt_params->crc_mode = S2_LP_CRC_MODE_OFF;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_CRC_1_BYTES:
                SID_PAL_LOG_ERROR("S2-LP does not support CRC_1_BYTES CRC mode");
                err = RADIO_ERROR_NOT_SUPPORTED;
                break;

            case SID_PAL_RADIO_FSK_CRC_1_BYTES_INV:
            	SID_PAL_LOG_ERROR("S2-LP does not support CRC_1_BYTES_INV CRC mode");
                err = RADIO_ERROR_NOT_SUPPORTED;
                break;

            case SID_PAL_RADIO_FSK_CRC_2_BYTES:
            case SID_PAL_RADIO_FSK_CRC_2_BYTES_IBM:
                out_s2lp_pckt_params->crc_mode = S2_LP_CRC_MODE_IBM_16;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_CRC_2_BYTES_INV:
            case SID_PAL_RADIO_FSK_CRC_2_BYTES_CCIT:
                out_s2lp_pckt_params->crc_mode = S2_LP_CRC_MODE_CCITT_16;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_CRC_4_BYTES_ADCCP:
                out_s2lp_pckt_params->crc_mode = S2_LP_CRC_MODE_ADCCP_32;
                err = RADIO_ERROR_NONE;
                break;

            default:
                SID_PAL_LOG_ERROR("Unknown CRC mode setting. ID: 0x%02X", sid_pckt_params->crc_type);
                err = RADIO_ERROR_INVALID_PARAMS;
                break;
        }

        if (err != RADIO_ERROR_NONE)
        {
            /* Propagate the error */
            break;
        }

        /* Convert data whitening setting */
        switch (sid_pckt_params->radio_whitening_mode)
        {
            case SID_PAL_RADIO_FSK_DC_FREE_OFF:
            	out_s2lp_pckt_params->data_whitening_en = 0u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_DC_FREEWHITENING:
            	out_s2lp_pckt_params->data_whitening_en = 1u;
                err = RADIO_ERROR_NONE;
                break;

            default:
                SID_PAL_LOG_ERROR("Unknown data whitening setting. ID: 0x%02X", sid_pckt_params->radio_whitening_mode);
                err = RADIO_ERROR_INVALID_PARAMS;
                break;
        }

        if (err != RADIO_ERROR_NONE)
        {
            /* Propagate the error */
            break;
        }

        /* Convert variable length setting */
        switch (sid_pckt_params->header_type)
        {
            case SID_PAL_RADIO_FSK_RADIO_PACKET_FIXED_LENGTH:
            	out_s2lp_pckt_params->variable_packet_len_en = 0u;
                err = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_FSK_RADIO_PACKET_VARIABLE_LENGTH:
            	out_s2lp_pckt_params->variable_packet_len_en = 1u;
                err = RADIO_ERROR_NONE;
                break;

            default:
                SID_PAL_LOG_ERROR("Unknown Sidewalk FSK packet header type setting. ID: 0x%02X", sid_pckt_params->header_type);
                err = RADIO_ERROR_INVALID_PARAMS;
                break;
        }

        if (err != RADIO_ERROR_NONE)
        {
            /* Propagate the error */
            break;
        }

        /* Convert packet length */
        out_s2lp_pckt_params->packet_length = sid_pckt_params->payload_length;

        /* Everything is fine */
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _s2lp_radio_get_fsk_crc_len_in_bytes(const uint32_t crc_type)
{
    uint32_t crc_len_bytes;

    switch (crc_type)
        {
            case SID_PAL_RADIO_FSK_CRC_OFF:
                crc_len_bytes = 0u;
                break;

            case SID_PAL_RADIO_FSK_CRC_1_BYTES:
            case SID_PAL_RADIO_FSK_CRC_1_BYTES_INV:
                crc_len_bytes = 1u;
                break;

            case SID_PAL_RADIO_FSK_CRC_2_BYTES:
            case SID_PAL_RADIO_FSK_CRC_2_BYTES_INV:
            case SID_PAL_RADIO_FSK_CRC_2_BYTES_IBM:
            case SID_PAL_RADIO_FSK_CRC_2_BYTES_CCIT:
                crc_len_bytes = 2u;
                break;

            default:
                crc_len_bytes = 0u;
                break;
        }

    return crc_len_bytes;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t _s2lp_radio_configure_fsk_sync_words(void)
{
    int32_t            err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    do
    {
        /* Set the sync word for FEC disabled mode */
        hal_err = s2_lp_radio_hal_set_syncword(&drv_ctx, fsk_sync_word_fec_disabled, sizeof(fsk_sync_word_fec_disabled), S2_LP_RADIO_HAL_SYNC_WORD_PRIMARY);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set FSK sync word for FEC-inactive mode. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Set the sync word for FEC disabled mode */
        hal_err = s2_lp_radio_hal_set_syncword(&drv_ctx, fsk_sync_word_fec_enabled, sizeof(fsk_sync_word_fec_enabled), S2_LP_RADIO_HAL_SYNC_WORD_SECONDARY);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set FSK sync word for FEC-enabled mode. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _s2lp_radio_error_manager(const int32_t reported_err)
{
    if (reported_err != RADIO_ERROR_NONE)
    {
        /* A new error was reported - increment error counter and update the timestamp */
        drv_ctx.error_monitor.drv_err_cntr++;
        (void)sid_pal_uptime_now(&drv_ctx.error_monitor.last_err_timestamp);
    }
    else
    {
        /* Operation completed successfully - check if we can reset the error counter */
         struct sid_timespec now;

        /* Store the timestamp as soon as possible */
        (void)sid_pal_uptime_now(&now);

        if (sid_time_gt(&now, &drv_ctx.error_monitor.last_err_timestamp) != FALSE)
        {
            struct sid_timespec time_delta;

            sid_time_delta(&time_delta, &now, &drv_ctx.error_monitor.last_err_timestamp);
            const uint32_t time_from_last_error = sid_timespec_to_ms(&time_delta);

            if (time_from_last_error > S2LP_RADIO_ERROR_BURST_TIME_WINDOW_MS)
            {
                /* Long enough time from the last error, it is safe to clear the error counter */
                drv_ctx.error_monitor.drv_err_cntr = 0u;
            }
            else
            {
                /* There were some errors recently, keep the counter as is since this may be just a transient success in a burst of errors */
            }
        }
    }

    /* Check if we are above the error threshold and react if needed */
    if (drv_ctx.error_monitor.drv_err_cntr > S2LP_RADIO_ERROR_LIMIT)
    {
        int32_t            err;
        s2_lp_hal_status_t hal_err;

        SID_PAL_LOG_WARNING("Too many radio errors in a row. Resetting the radio...");

        do
        {
            /* Ensure S2-LP won't trigger any interrupt if we are re-initializing ----*/
            hal_err = s2_lp_radio_hal_disarm_irq(&drv_ctx);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                /* Logs are provided by the s2_lp_radio_hal_disarm_irq() */
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
            __COMPILER_BARRIER(); /* Ensure IRQ is disarmed right now before any further actions take place */
            /*----------------------------------------------------------------------------*/

            /* Bring down the underlying hardware */
            err = _s2lp_radio_platform_deinit();
            if (err != RADIO_ERROR_NONE)
            {
                S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_deinit() failed. Error %d", err);
                break;
            }

            /* Initialize the underlying hardware */
            err = _s2lp_radio_platform_init();
            if (err != RADIO_ERROR_NONE)
            {
                S2LP_RADIO_LOG_ERROR("_s2lp_radio_error_manager() failed with error %d", err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            /* Low level S2-LP init (e.g. XTAL, SMPS, reference clock, etc.) */
            hal_err = s2_lp_radio_hal_ll_init(&drv_ctx);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                S2LP_RADIO_LOG_ERROR("_s2lp_radio_error_manager - failed to perform low-level S2-LP IC init. HAL error %u", hal_err);
                break;
            }

            /* Apply regional settings */
            err = sid_pal_radio_set_region(drv_ctx.config->regional_config.radio_region);
            if (err != RADIO_ERROR_NONE)
            {
                S2LP_RADIO_LOG_ERROR("sid_pal_radio_set_region() failed with error %d", err);
                break;
            }

            /* Apply modulation parameters that are static though out the operation */
            hal_err = s2_lp_radio_hal_static_mod_params_init(&drv_ctx);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                S2LP_RADIO_LOG_ERROR("_s2lp_radio_error_manager - failed to apply static modulation parameters to S2-LP. HAL error %u", hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            /* FSK sync words use static config - apply now */
            err = _s2lp_radio_configure_fsk_sync_words();
            if (err != RADIO_ERROR_NONE)
            {
                S2LP_RADIO_LOG_ERROR("_s2lp_radio_error_manager() failed with error %d", err);
                break;
            }

            /* Now configure S2-LP IRQ line as a regular IRQ line ------------------------*/
            hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                /* Logs are provided by the s2_lp_radio_hal_set_radio_irq_mask() */
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            hal_err = s2_lp_radio_hal_arm_irq(&drv_ctx);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                /* Logs are provided by the s2_lp_radio_hal_arm_irq() */
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
            /*----------------------------------------------------------------------------*/

            /* Radio re-initialized successfully */
            err = RADIO_ERROR_NONE;
            drv_ctx.error_monitor.drv_err_cntr = 0u;
            SID_PAL_LOG_INFO("Radio re-initialized");
        } while (0);

        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to re-initialize S2-LP radio. This may indicate a permanent failure");
        }
    }
}

/* Global function definitions -----------------------------------------------*/

void s2_lp_radio_set_device_config(const s2_lp_radio_device_config_t * const cfg)
{
    drv_ctx.config = cfg;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_init(sid_pal_radio_event_notify_t notify, sid_pal_radio_irq_handler_t dio_irq_handler, sid_pal_radio_rx_packet_t *rx_packet)
{
    int32_t            err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;
    sid_error_t        sid_err;

    S2LP_RADIO_LOG_DEBUG("sid_pal_radio_init... ");
    do
    {
        /* Validate the inputs -------------------------------------------------------*/
        if ((NULL == notify) || (NULL == dio_irq_handler) || (NULL == rx_packet))
        {
            S2LP_RADIO_LOG_ERROR("sid_pal_radio_init() aborted due to invalid input parameters");
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Check if the driver is initialized already */
        if (drv_ctx.init_done != FALSE)
        {
            SID_PAL_LOG_WARNING("S2-LP driver is initialized already. Requesting deinitialization");

            /* Deinitialize any hardware that may have been partially initialized to bring it to the known state */
            err = sid_pal_radio_deinit();
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Unable to initialize S2-LP radio - driver is initialized already and deinitialization failed. Error %d", err);
                break;
            }
        }
        /*----------------------------------------------------------------------------*/

        /* Ensure S2-LP won't trigger any interrupt if we are re-initializing ----*/
        hal_err = s2_lp_radio_hal_disarm_irq(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Logs are provided by the s2_lp_radio_hal_disarm_irq() */
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
        __COMPILER_BARRIER(); /* Ensure IRQ is disarmed right now before any further actions take place */
        /*----------------------------------------------------------------------------*/

        /* Configure driver context with the supplied parameters ---------------------*/
        drv_ctx.radio_rx_packet       = rx_packet;
        drv_ctx.report_radio_event    = notify;
        drv_ctx.radio_irq_handler     = dio_irq_handler;
        drv_ctx.radio_irq_mask.raw    = S2LP_RADIO_DEFAULT_FSK_IRQ_MASK;

        sid_err = sid_pal_timer_init(&drv_ctx.rco_calibration.timer, s2_lp_radio_hal_rco_calib_timer_cb, &drv_ctx);
        if (sid_err != SID_ERROR_NONE)
        {
            S2LP_RADIO_LOG_ERROR("S2-LP RCO calibration timer init failed with error %d", sid_err);
            err = RADIO_ERROR_IO_ERROR;
            break;
        }

        /* Initialize the underlying hardware ----------------------------------------*/
        err = _s2lp_radio_platform_init();
        if (err != RADIO_ERROR_NONE)
        {
            S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_init() failed with error %d", err);
            break;
        }

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Keep  the status led on while the configuration is being applied */
        (void)s2_lp_radio_hal_tx_led_on(&drv_ctx);
        (void)s2_lp_radio_hal_rx_led_on(&drv_ctx);
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        /* Low level S2-LP init (e.g. XTAL, SMPS, reference clock, etc.) */
        hal_err = s2_lp_radio_hal_ll_init(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_init - failed to perform low-level S2-LP IC init. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Apply regional settings */
        err = sid_pal_radio_set_region(drv_ctx.config->regional_config.radio_region);
        if (err != RADIO_ERROR_NONE)
        {
            S2LP_RADIO_LOG_ERROR("sid_pal_radio_set_region() failed with error %d", err);
            break;
        }

        /* Apply modulation parameters that are static though out the operation */
        hal_err = s2_lp_radio_hal_static_mod_params_init(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_init - failed to apply static modulation parameters to S2-LP. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* FSK sync words use static config - apply now */
        err = _s2lp_radio_configure_fsk_sync_words();
        if (err != RADIO_ERROR_NONE)
        {
            S2LP_RADIO_LOG_ERROR("_s2lp_radio_configure_fsk_sync_words() failed with error %d", err);
            break;
        }

        /* For FSK Rx timeout timer shall be stopped on preamble detection */
        hal_err = s2_lp_radio_hal_configure_packet_engine_for_rx(&drv_ctx, S2_LP_RX_TIMEOUT_STOP_PQI_ABOVE_THRESHOLD);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_init - failed to configure S2-LP Rx timer stop conditions. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Now configure S2-LP IRQ line as a regular IRQ line ------------------------*/
        hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Logs are provided by the s2_lp_radio_hal_set_radio_irq_mask() */
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        hal_err = s2_lp_radio_hal_arm_irq(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Logs are provided by the s2_lp_radio_hal_arm_irq() */
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Initialization sequence is finished with no issues */
#if S2LP_RADIO_CFG_USE_STATUS_LED
        (void)s2_lp_radio_hal_tx_led_off(&drv_ctx);
        (void)s2_lp_radio_hal_rx_led_off(&drv_ctx);
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        drv_ctx.radio_state = SID_PAL_RADIO_STANDBY;

        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Unable to initialize S2-LP radio. Error %d. S2-LP state: 0x%02X%02X", err, drv_ctx.regs_cache.mc_state.raw[0], drv_ctx.regs_cache.mc_state.raw[1]);

        /* Ensure we deinitialize any hardware that may have been partially initialized */
        (void)sid_pal_radio_deinit();
    }
    else
    {
        drv_ctx.init_done = TRUE;
        SID_PAL_LOG_INFO("S2-LP radio initialized");
    }

    return err;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_deinit(void)
{
    int32_t err = RADIO_ERROR_GENERIC;

    S2LP_RADIO_LOG_DEBUG("sid_pal_radio_deinit... ");

    do
    {
        (void)sid_pal_timer_deinit(&drv_ctx.rco_calibration.timer);

        /* Bring down the underlying hardware */
        err = _s2lp_radio_platform_deinit();
        if (err != RADIO_ERROR_NONE)
        {
            S2LP_RADIO_LOG_ERROR("_s2lp_radio_platform_deinit() failed. Error %d", err);
            break;
        }

        /* From here radio IRQs won't be processed */

        /* Invalidate the context */
        drv_ctx.init_done                   = FALSE;
        drv_ctx.radio_irq_handler           = NULL;
        drv_ctx.radio_rx_packet             = NULL;
        drv_ctx.radio_state                 = SID_PAL_RADIO_UNKNOWN;
        drv_ctx.current_bit_rate            = 0u;
        drv_ctx.pa_params.pa_ramp_bit_rate  = 0u;
        drv_ctx.rco_cal_init_err_cnt        = 0u;
#if HALO_ENABLE_DIAGNOSTICS
        drv_ctx.pa_params.pa_cfg_configured = FALSE;
#endif /* HALO_ENABLE_DIAGNOSTICS */

        SID_STM32_UTIL_fast_memset(&drv_ctx.regs_cache, 0u, sizeof(drv_ctx.regs_cache));
        SID_STM32_UTIL_fast_memset(&drv_ctx.error_monitor, 0u, sizeof(drv_ctx.error_monitor));
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Unable to deinitialize S2-LP radio. Error %d", err);
    }
    else
    {
        SID_PAL_LOG_INFO("S2-LP radio deinitialized");
    }

#if (CFG_LPM_LEVEL != 0)
    /* Allow Stop mode after the radio driver is deinitialized */
    UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
    UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_pal_radio_irq_mask_t sid_pal_radio_configure_irq_mask(sid_pal_radio_irq_mask_t irq_mask)
{
    /* Store a copy of the original S2-LP IRQ mask, including the IRQs that cannot be matched to SID IRQs */
    const sl2_lp_ic_irq_mask_t radio_mask_backup = drv_ctx.radio_irq_mask;
    s2_lp_hal_status_t         hal_err;

    if (RADIO_IRQ_ALL == irq_mask)
    {
        /* Enable all relevant IRQs */
        drv_ctx.radio_irq_mask.raw = S2LP_RADIO_DEFAULT_FSK_IRQ_MASK;
    }
    else if (RADIO_IRQ_NONE == irq_mask)
    {
        /* Mask out all IRQs */
        drv_ctx.radio_irq_mask.raw = 0u;
    }
    else
    {
        /* Process each IRQ one by one */
        drv_ctx.radio_irq_mask.raw &= ~(S2LP_RADIO_SUPPORTED_SID_IRQS);

        if ((irq_mask & RADIO_IRQ_TX_DONE) != 0u)
        {
            drv_ctx.radio_irq_mask.IRQ_TX_DATA_SENT = S2_LP_EVENT_ACTIVE;
        }

        if ((irq_mask & RADIO_IRQ_RX_DONE) != 0u)
        {
            drv_ctx.radio_irq_mask.IRQ_RX_DATA_READY = S2_LP_EVENT_ACTIVE;
        }

        if ((irq_mask & RADIO_IRQ_PREAMBLE_DETECT) != 0u)
        {
            drv_ctx.radio_irq_mask.IRQ_VALID_PREAMBLE = S2_LP_EVENT_ACTIVE;
        }

        if ((irq_mask & RADIO_IRQ_VALID_SYNC_WORD) != 0u)
        {
            drv_ctx.radio_irq_mask.IRQ_VALID_SYNC = S2_LP_EVENT_ACTIVE;
        }

        if ((irq_mask & RADIO_IRQ_VALID_HEADER) != 0u)
        {
            SID_PAL_LOG_WARNING("S2-LP does not support RADIO_IRQ_VALID_HEADER. It will be ignored");
        }

        if ((irq_mask & RADIO_IRQ_ERROR_HEADER) != 0u)
        {
            SID_PAL_LOG_WARNING("S2-LP does not support RADIO_IRQ_ERROR_HEADER. It will be ignored");
        }

        if ((irq_mask & RADIO_IRQ_ERROR_CRC) != 0u)
        {
            drv_ctx.radio_irq_mask.IRQ_CRC_ERROR = S2_LP_EVENT_ACTIVE;
        }

        if ((irq_mask & RADIO_IRQ_CAD_DONE) != 0u)
        {
            SID_PAL_LOG_WARNING("S2-LP does not support RADIO_IRQ_CAD_DONE. It will be ignored");
        }

        if ((irq_mask & RADIO_IRQ_CAD_DETECT) != 0u)
        {
            SID_PAL_LOG_WARNING("S2-LP does not support RADIO_IRQ_CAD_DETECT. It will be ignored");
        }

        if ((irq_mask & RADIO_IRQ_TXRX_TIMEOUT) != 0u)
        {
            drv_ctx.radio_irq_mask.IRQ_RX_TIMEOUT = S2_LP_EVENT_ACTIVE;
        }
    }

    /* Now send out new IRQ mask to the radio */
    hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
    if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
    {
        /* Restore the original mask  in driver context since we failed to apply the new one */
        drv_ctx.radio_irq_mask = radio_mask_backup;
    }

    /* Get SID IRQ mask based on the applied config */
    sid_pal_radio_irq_mask_t out_mask = sid_pal_radio_get_current_config_irq_mask();
    return out_mask;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_pal_radio_irq_mask_t sid_pal_radio_get_current_config_irq_mask(void)
{
    sid_pal_radio_irq_mask_t sid_irq_mask = RADIO_IRQ_NONE;

    if (S2LP_RADIO_DEFAULT_FSK_IRQ_MASK == drv_ctx.radio_irq_mask.raw)
    {
        sid_irq_mask = RADIO_IRQ_ALL;
    }
    else if (0u == drv_ctx.radio_irq_mask.raw)
    {
        sid_irq_mask = RADIO_IRQ_NONE;
    }
    else
    {
        if (drv_ctx.radio_irq_mask.IRQ_TX_DATA_SENT != S2_LP_EVENT_NOT_PRESENT)
        {
            sid_irq_mask |= RADIO_IRQ_TX_DONE;
        }

        if (drv_ctx.radio_irq_mask.IRQ_RX_DATA_READY != S2_LP_EVENT_NOT_PRESENT)
        {
            sid_irq_mask |= RADIO_IRQ_RX_DONE;
        }

        if (drv_ctx.radio_irq_mask.IRQ_VALID_PREAMBLE != S2_LP_EVENT_NOT_PRESENT)
        {
            sid_irq_mask |= RADIO_IRQ_PREAMBLE_DETECT;
        }

        if (drv_ctx.radio_irq_mask.IRQ_VALID_SYNC != S2_LP_EVENT_NOT_PRESENT)
        {
            sid_irq_mask |= RADIO_IRQ_VALID_SYNC_WORD;
        }

        if (drv_ctx.radio_irq_mask.IRQ_CRC_ERROR != S2_LP_EVENT_NOT_PRESENT)
        {
            sid_irq_mask |= RADIO_IRQ_ERROR_CRC;
        }

        if (drv_ctx.radio_irq_mask.IRQ_RX_TIMEOUT != S2_LP_EVENT_NOT_PRESENT)
        {
            sid_irq_mask |= RADIO_IRQ_TXRX_TIMEOUT;
        }
    }

    return sid_irq_mask;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_irq_process(void)
{
    int32_t                err         = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t     hal_err;
    sid_pal_radio_events_t radio_event = SID_PAL_RADIO_EVENT_UNKNOWN;

    do
    {
        sl2_lp_ic_irq_status_t irq_status;

        err = s2_lp_radio_hal_get_clear_irq_status(&drv_ctx, &irq_status);
        if (err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read out S2-LP IRQ status. HAL error %u", (uint32_t)err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* S2-LP sets IRQ status flags whenever the related event occurs, regardless of the IRQ mask settings.
         * IRQ mask just disables IRQ indication via a GPIO pin. This means we need to clean up the status
         * reported by s2_lp_radio_hal_get_clear_irq_status() from any irrelevant flags
         */
        irq_status.raw &= drv_ctx.radio_irq_mask.raw;

        /* Proceed with regular IRQ handling -----------------------------------------*/

        /* Most urgent IRQ - Tx FIFO almost empty ------------------------------------*/
        if (irq_status.IRQ_TX_FIFO_ALMOST_EMPTY != S2_LP_EVENT_NOT_PRESENT)
        {
            uint32_t tx_fifo_free_bytes;
            S2LP_RADIO_LOG_DEBUG("IRQ_TX_FIFO_ALMOST_EMPTY");

            /* Check how much space we have in Tx FIFO */
            hal_err = s2_lp_radio_hal_get_tx_fifo_free_space(&drv_ctx, &tx_fifo_free_bytes);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to read S2-LP Tx FIFO status. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            /* Calculate the Tx FIFO upload size */
            const uint8_t * const upload_ptr  = &drv_ctx.config->internal_buffer.p[drv_ctx.processing_buf_offset];
            const uint32_t        upload_size = drv_ctx.processing_buf_total_bytes > tx_fifo_free_bytes ? tx_fifo_free_bytes : drv_ctx.processing_buf_total_bytes;

            /* Write data to the FIFO */
            if (upload_size > 0u)
            {
                hal_err = s2_lp_radio_hal_write_to_tx_fifo(&drv_ctx, upload_ptr, upload_size);
                if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
                {
                    SID_PAL_LOG_ERROR("Failed to write to S2-LP Tx FIFO. HAL error %u", (uint32_t)hal_err);
                    err = RADIO_ERROR_HARDWARE_ERROR;
                    break;
                }

                drv_ctx.processing_buf_total_bytes -= upload_size;
                drv_ctx.processing_buf_offset      += upload_size;
            }

            /* If all the data is uploaded to FIFO we don't need the FIFO Almost Empty IRQ anymore */
            if (0u == drv_ctx.processing_buf_total_bytes)
            {
                /* Disable Tx FIFO Almost Empty IRQ - packet is fully uploaded to the FIFO */
                drv_ctx.radio_irq_mask.IRQ_TX_FIFO_ALMOST_EMPTY = S2_LP_EVENT_NOT_PRESENT;
                hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
                if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
                {
                    /* Logs are provided by the s2_lp_radio_hal_set_radio_irq_mask() */
                    err = RADIO_ERROR_HARDWARE_ERROR;
                    break;
                }
            }

            /* If everything is fine */
            err = RADIO_ERROR_NONE;
        }
        /*----------------------------------------------------------------------------*/

        /* Rx FIFO readout -----------------------------------------------------------*/
        if (irq_status.IRQ_RX_FIFO_ALMOST_FULL != S2_LP_EVENT_NOT_PRESENT)
        {
            S2LP_RADIO_LOG_DEBUG("IRQ_RX_FIFO_ALMOST_FULL");

            uint8_t * const store_ptr = &drv_ctx.config->internal_buffer.p[drv_ctx.processing_buf_offset];
            const uint32_t remaining_buf_space = drv_ctx.config->internal_buffer.size - drv_ctx.processing_buf_offset;
            uint32_t bytes_read;

            if (0u == remaining_buf_space)
            {
                SID_PAL_LOG_ERROR("Run out of Rx processing buffer space, can't read from S2-LP's FIFO");
                radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
                err = RADIO_ERROR_NOMEM;
                break;
            }

            /* Read out up to the remaining buffer space */
            hal_err = s2_lp_radio_hal_readout_from_rx_fifo(&drv_ctx, store_ptr, remaining_buf_space, &bytes_read);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                S2LP_RADIO_LOG_ERROR("Failed to read Rx FIFO bytes. HAL error %u", hal_err);
                radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
            else
            {
                /* Advance counters and pointers */
                drv_ctx.processing_buf_offset      += bytes_read;
                drv_ctx.processing_buf_total_bytes += bytes_read;

                err = RADIO_ERROR_NONE;
                /* Continue processing as this IRQ may arrive together with RX_DATA_READY IRQ */
            }
        }
        /*----------------------------------------------------------------------------*/

        /* Handle Tx Done IRQ --------------------------------------------------------*/
        if (irq_status.IRQ_TX_DATA_SENT != S2_LP_EVENT_NOT_PRESENT)
        {
            S2LP_RADIO_LOG_DEBUG("IRQ_TX_DATA_SENT");

            drv_ctx.radio_state = SID_PAL_RADIO_STANDBY; /* Radio automatically goes to READY state upon Tx completion */

            radio_event = SID_PAL_RADIO_EVENT_TX_DONE;
            err = RADIO_ERROR_NONE;
            break; /* Stop further processing */
        }
        /*----------------------------------------------------------------------------*/

        /* Preamble Detect and RSSI Threshold IRQs -----------------------------------*/
        if ((irq_status.IRQ_VALID_PREAMBLE != S2_LP_EVENT_NOT_PRESENT) || (irq_status.IRQ_RSSI_ABOVE_TH != S2_LP_EVENT_NOT_PRESENT))
        {

            if (irq_status.IRQ_VALID_PREAMBLE != S2_LP_EVENT_NOT_PRESENT)
            {
                S2LP_RADIO_LOG_DEBUG("IRQ_VALID_PREAMBLE");
            }
            else
            {
                S2LP_RADIO_LOG_DEBUG("IRQ_RSSI_ABOVE_TH");
            }

            if (SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == drv_ctx.cad_exit_mode)
            {
                /* We were about to start Tx, but CS/CAD detected some activity - channel is busy */
                drv_ctx.radio_rx_packet->fsk_rx_packet_status.rssi_sync = (int8_t)sid_pal_radio_rssi();

                /* Abort CS/CAD */
                err = sid_pal_radio_standby();
                if (err != RADIO_ERROR_NONE)
                {
                    SID_PAL_LOG_ERROR("Failed to abort ongoing carrier sense. Error %d", err);
                    break;
                }

                /* Restore default settings */
                drv_ctx.cad_exit_mode                     = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;
                drv_ctx.radio_irq_mask.IRQ_VALID_PREAMBLE = S2_LP_EVENT_NOT_PRESENT;
                drv_ctx.radio_irq_mask.IRQ_RSSI_ABOVE_TH  = S2_LP_EVENT_NOT_PRESENT;
                hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
                if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
                {
                    /* Logs are provided by the s2_lp_radio_hal_set_radio_irq_mask() */
                    err = RADIO_ERROR_HARDWARE_ERROR;
                    break;
                }

                radio_event = SID_PAL_RADIO_EVENT_CS_DONE;
                err = RADIO_ERROR_NONE;
                break;
            }
            else
            {
                err = RADIO_ERROR_NONE;
                /* Continue processing as this IRQ may arrive together with RX_DATA_READY */
            }
        }
        /*----------------------------------------------------------------------------*/

        /* Rx Timeout IRQ ------------------------------------------------------------*/
        if (irq_status.IRQ_RX_TIMEOUT != S2_LP_EVENT_NOT_PRESENT)
        {
            S2LP_RADIO_LOG_DEBUG("IRQ_RX_TIMEOUT");

            drv_ctx.radio_state = SID_PAL_RADIO_STANDBY; /* Radio automatically goes to READY state upon RX timeout */

            if (SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == drv_ctx.cad_exit_mode)
            {
                /* This was a CS/CAD run and channel is free - we can proceed with Tx */
                drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

                /* Restore IRQ settings */
                drv_ctx.radio_irq_mask.IRQ_VALID_PREAMBLE = S2_LP_EVENT_NOT_PRESENT;
                drv_ctx.radio_irq_mask.IRQ_RSSI_ABOVE_TH  = S2_LP_EVENT_NOT_PRESENT;
                hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
                if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
                {
                    /* Logs are provided by the s2_lp_radio_hal_set_radio_irq_mask() */
                    err = RADIO_ERROR_HARDWARE_ERROR;
                    break;
                }

#if HALO_ENABLE_DIAGNOSTICS
                radio_event = SID_PAL_RADIO_EVENT_CS_TIMEOUT;
                err = RADIO_ERROR_NONE;
#else
                err = sid_pal_radio_start_tx(0u);
#endif
            }
            else
            {
                /* This was a usual Rx operation and it ended with a timeout */
                radio_event = SID_PAL_RADIO_EVENT_RX_TIMEOUT;
                err = RADIO_ERROR_NONE;
            }
            break; /* Stop further processing */
        }
        /*----------------------------------------------------------------------------*/

        /* CRC Error IRQ -------------------------------------------------------------*/
        if (irq_status.IRQ_CRC_ERROR != S2_LP_EVENT_NOT_PRESENT)
        {
            S2LP_RADIO_LOG_DEBUG("IRQ_CRC_ERROR");

            radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
            drv_ctx.radio_state = SID_PAL_RADIO_STANDBY; /* Radio automatically goes to READY state after unsuccessful RX */
            err = RADIO_ERROR_NONE;
            break; /* Stop further processing */
        }
        /*----------------------------------------------------------------------------*/
 
        /* FIFO Error IRQs -----------------------------------------------------------*/
        if ((irq_status.IRQ_TX_FIFO_ERROR != S2_LP_EVENT_NOT_PRESENT) || (irq_status.IRQ_RX_FIFO_ERROR != S2_LP_EVENT_NOT_PRESENT))
        {
            S2LP_RADIO_LOG_DEBUG("IRQ_FIFO_ERROR");

            if (SID_PAL_RADIO_TX == drv_ctx.radio_state)
            {
                /* Sidewalk has no better Tx failure event than Tx timeout - use it to report FIFO error */
                radio_event = SID_PAL_RADIO_EVENT_TX_TIMEOUT;
            }
            else
            {
                radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
            }

            /* Abort any ongoing operation and flush FIFOs to clear the error */
            err = sid_pal_radio_standby();
            if (err != RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to abort ongoing radio operation. Error %d", err);
                break;
            }
            hal_err = s2_lp_radio_hal_flush_fifo(&drv_ctx, S2_LP_RADIO_HAL_FIFO_BOTH);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            err = RADIO_ERROR_NONE;
            break; /* Stop further processing */
        }
        /*----------------------------------------------------------------------------*/

#ifdef S2LP_RADIO_PHY_DEBUG
        /* Valid SYNC IRQ ------------------------------------------------------------*/
        if (irq_status.IRQ_VALID_SYNC != S2_LP_EVENT_NOT_PRESENT)
        {
            SID_PAL_LOG_DEBUG("SYNC");

            /* Don't override radio_event here */
            err = RADIO_ERROR_NONE;
            /* Continue processing as this IRQ may arrive together with RX_DATA_READY */
        }
        /*----------------------------------------------------------------------------*/
#endif /* S2LP_RADIO_PHY_DEBUG */

#if S2LP_RADIO_CFG_USE_STATUS_LED && HALO_ENABLE_DIAGNOSTICS
        /* LDC Wakeup Timeout IRQ ----------------------------------------------------*/
        if (irq_status.IRQ_WKUP_TOUT_LDC != S2_LP_EVENT_NOT_PRESENT)
        {
            S2LP_RADIO_LOG_DEBUG("IRQ_WKUP_TOUT_LDC");

            if (SID_PAL_RADIO_RX_DC == drv_ctx.radio_state)
            {
                /* Turn on the activity LED since the radio was woken up by the LDC timer */
                (void)s2_lp_radio_hal_rx_led_on(&drv_ctx);
            }

            err = RADIO_ERROR_NONE;
            /* Continue processing as this IRQ may arrive together with RX_DATA_READY */
        }
        /*----------------------------------------------------------------------------*/
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED && HALO_ENABLE_DIAGNOSTICS */

        /* Rx Done IRQ ---------------------------------------------------------------*/
        if (irq_status.IRQ_RX_DATA_READY != S2_LP_EVENT_NOT_PRESENT)
        {
            S2LP_RADIO_LOG_DEBUG("IRQ_RX_DATA_READY");

            drv_ctx.radio_state = SID_PAL_RADIO_STANDBY; /* Radio automatically goes to READY state upon RX completion */

            /* Read out Rx FIFO */
            uint8_t * const store_ptr = &drv_ctx.config->internal_buffer.p[drv_ctx.processing_buf_offset];
            const uint32_t remaining_buf_space = drv_ctx.config->internal_buffer.size - drv_ctx.processing_buf_offset;
            uint32_t bytes_read;

            if (0u == remaining_buf_space)
            {
                SID_PAL_LOG_ERROR("Run out of Rx processing buffer space, can't read from S2-LP's FIFO");
                radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
                err = RADIO_ERROR_NOMEM;
                break;
            }

            hal_err = s2_lp_radio_hal_readout_from_rx_fifo(&drv_ctx, store_ptr, remaining_buf_space, &bytes_read);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                S2LP_RADIO_LOG_ERROR("Failed to read Rx FIFO bytes. HAL error %u", hal_err);
                radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            /* Advance counters and pointers */
            drv_ctx.processing_buf_offset      += bytes_read;
            drv_ctx.processing_buf_total_bytes += bytes_read;

            /* Get Rx packet stats */
            s2_lp_hal_rx_packet_info_t rx_packet_info;

            hal_err = s2_lp_radio_hal_get_rx_packet_info(&drv_ctx, &rx_packet_info);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                S2LP_RADIO_LOG_ERROR("Failed to read Rx packet status info. HAL error %u", hal_err);
                radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            /* Check if received data fits into the Sidewalk's internal buffer */
            if (drv_ctx.processing_buf_total_bytes > sizeof(drv_ctx.radio_rx_packet->rcv_payload))
            {
                SID_PAL_LOG_WARNING("Rx size exceeds maximum payload limit (%u vs %u) - packet dropped", drv_ctx.processing_buf_total_bytes, sizeof(drv_ctx.radio_rx_packet->rcv_payload));
                radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
                err = RADIO_ERROR_NOMEM;
                break;
            }

            /* Sanity check */
            uint32_t expected_rx_packet_len = drv_ctx.processing_buf_total_bytes;
            expected_rx_packet_len += S2_LP_FCS_TYPE_0 == rx_packet_info.fcs_type ? 4u : 2u; /* Add expected CRC length depending on the reported FCS type */
            if (expected_rx_packet_len != rx_packet_info.packet_length)
            {
                SID_PAL_LOG_ERROR("Rx size crosscheck failed. Expected: %u, actual: %u", expected_rx_packet_len, rx_packet_info.packet_length);
                radio_event = SID_PAL_RADIO_EVENT_RX_ERROR;
                err = RADIO_ERROR_IO_ERROR;
                break;
            }

            /* Fill-in packet stats */
            drv_ctx.radio_rx_packet->fsk_rx_packet_status.rssi_avg  = (int8_t)rx_packet_info.rssi.adjusted_rssi; /* Measurement not implemented, substitute with RSSI on Sync Word detection */
            drv_ctx.radio_rx_packet->fsk_rx_packet_status.rssi_sync = (int8_t)rx_packet_info.rssi.adjusted_rssi;
            drv_ctx.radio_rx_packet->fsk_rx_packet_status.snr       = 0u;                          /* Measurement not supported by S2-LP */

            /* Store received data as a Rx packet for Sidewalk stack */
            SID_STM32_UTIL_fast_memcpy(drv_ctx.radio_rx_packet->rcv_payload, drv_ctx.config->internal_buffer.p, drv_ctx.processing_buf_total_bytes);
            drv_ctx.radio_rx_packet->payload_len = drv_ctx.processing_buf_total_bytes;

            radio_event = SID_PAL_RADIO_EVENT_RX_DONE;
            err = RADIO_ERROR_NONE;
            break; /* Stop further processing */
        }
        /*----------------------------------------------------------------------------*/
    } while (0u);

    if (SID_PAL_RADIO_EVENT_UNKNOWN != radio_event)
    {
#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
        /* Clear Rx/Tx starting point indication */
        SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */

        /* Handle Rx Duty Cycle mode */
        if (SID_PAL_RADIO_RX_DC == drv_ctx.radio_state)
        {
            /* Put the radio back into sleep mode after IRQ processing */
            hal_err = s2_lp_radio_hal_set_standby(&drv_ctx);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to put S2-LP into Sleep state. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
            }
        }

#if (CFG_LPM_LEVEL != 0)
        if ((SID_PAL_RADIO_EVENT_RX_TIMEOUT == radio_event) || (SID_PAL_RADIO_EVENT_RX_DONE == radio_event) || (SID_PAL_RADIO_EVENT_TX_DONE == radio_event) || (SID_PAL_RADIO_EVENT_TX_TIMEOUT == radio_event)
          || (SID_PAL_RADIO_EVENT_RX_ERROR == radio_event) || (SID_PAL_RADIO_EVENT_HEADER_ERROR == radio_event) || (SID_PAL_RADIO_EVENT_CS_DONE == radio_event) || (SID_PAL_RADIO_EVENT_CS_TIMEOUT == radio_event))
        {
            /* Allow Stop mode after Rx/Tx is finished */
            UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
            UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
            /* Configure the radio front end module for Sleep */
            hal_err = s2_lp_radio_hal_set_fem_mode(&drv_ctx, S2_LP_RADIO_HAL_FEM_MODE_SHUTDOWN);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to turn FEM off. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
            }
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */
        }
#endif /* (CFG_LPM_LEVEL != 0) */

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Turn off the LED to indicate no activity is taking place any longer */
        (void)s2_lp_radio_hal_tx_led_off(&drv_ctx);
        (void)s2_lp_radio_hal_rx_led_off(&drv_ctx);
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        /* Notify the radio about the event - make sure this is the last action after all GPIO handling and LPM management since drv_ctx.report_radio_event() may modify GPIO and LPM states */
        drv_ctx.report_radio_event(radio_event);

        /* Provide some debug outputs. Do it here since it does not affect the timings and processing delays */
        switch (radio_event)
        {
            case SID_PAL_RADIO_EVENT_TX_DONE:
                SID_PAL_LOG_DEBUG("Tx done. Len: %u", drv_ctx.last_tx_payload_size);
                break;

            case SID_PAL_RADIO_EVENT_RX_DONE:
                SID_PAL_LOG_DEBUG("Rx done. Len: %u", drv_ctx.radio_rx_packet->payload_len);
                break;
            
            default:
                /* Do nothing */
                break;
        }
    }

    /* Restore the default IRQ mask if something went wrong with IRQ processing */
    if ((err != RADIO_ERROR_NONE) && (drv_ctx.radio_irq_mask.raw != S2LP_RADIO_DEFAULT_FSK_IRQ_MASK))
    {
        drv_ctx.radio_irq_mask.raw = S2LP_RADIO_DEFAULT_FSK_IRQ_MASK;
        hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Logs are provided by the s2_lp_radio_hal_set_radio_irq_mask() */
            err = RADIO_ERROR_HARDWARE_ERROR;
        }
    }

    /* Re-enable radio IRQ detection */
    hal_err = s2_lp_radio_hal_arm_irq(&drv_ctx);
    if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
    {
        /* Logs are provided by the s2_lp_radio_hal_arm_irq() */
        err = RADIO_ERROR_HARDWARE_ERROR;
    }

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_frequency(uint32_t freq)
{
    int32_t            err;
    s2_lp_hal_status_t hal_err;

    S2LP_RADIO_LOG_DEBUG("sid_pal_radio_set_frequency %uHz...", freq);

    SID_PAL_ASSERT(drv_ctx.regional_radio_params != NULL);
    SID_PAL_ASSERT(drv_ctx.regional_radio_params->channel_spacing_hz != 0u);

    do
    {
        const uint32_t ch_num = (freq - drv_ctx.regional_radio_params->base_frequency_hz) / drv_ctx.regional_radio_params->channel_spacing_hz;
        const uint32_t actual_freq = drv_ctx.regional_radio_params->base_frequency_hz + (drv_ctx.regional_radio_params->channel_spacing_hz * ch_num);

        if (actual_freq != freq)
        {
            SID_PAL_LOG_WARNING("Cannot properly select channel number for %uHz. Base freq: %u, spacing: %u",
                                freq, drv_ctx.regional_radio_params->base_frequency_hz, drv_ctx.regional_radio_params->channel_spacing_hz);
        }

        hal_err = s2_lp_radio_hal_set_rf_channel(&drv_ctx, ch_num);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            const uint32_t freq_khz = freq / 1000u;
            SID_PAL_LOG_ERROR("Failed to set S2-LP frequency to %u.%03uMHz. HAL error %u", (freq_khz / 1000u), (freq_khz % 1000u), (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = RADIO_ERROR_NONE;
    } while (0u);

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_tx_power(int8_t power)
{
    int32_t            err      = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    S2LP_RADIO_LOG_DEBUG("sid_pal_radio_set_tx_power %ddBm...", power);

    do
    {
        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Trigger app callback to adjust Tx power settings before applying them to the radio */
#if HALO_ENABLE_DIAGNOSTICS
        if (FALSE == drv_ctx.pa_params.pa_cfg_configured)
#endif
        {
            err = drv_ctx.config->pa_cfg_callback(power, drv_ctx.regional_radio_params, &drv_ctx.pa_params.pa_dyn_cfg);

            if (err != RADIO_ERROR_NONE)
            {
                break;
            }
        }

        /* Apply the settings to the radio */
        hal_err = s2_lp_radio_hal_configure_tx_power(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set S2-LP Tx power to %ddBm. HAL error %u", power, (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_max_tx_power(sid_pal_radio_data_rate_t data_rate, int8_t *tx_power)
{
    int32_t err;

    SID_PAL_ASSERT(drv_ctx.regional_radio_params != NULL);

    if ((data_rate <= SID_PAL_RADIO_DATA_RATE_INVALID) || (data_rate > SID_PAL_RADIO_DATA_RATE_MAX_NUM))
    {
        err = RADIO_ERROR_INVALID_PARAMS;
    }
    else
    {
        *tx_power = drv_ctx.regional_radio_params->max_tx_power[data_rate - 1];
        err = RADIO_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_region(sid_pal_radio_region_code_t region)
{
    /* By default report the requested region is not supported */
    int32_t err = RADIO_ERROR_NOT_SUPPORTED;

    /* Validate inputs */
    if ((region <= SID_PAL_RADIO_RC_NONE) || (region >= SID_PAL_RADIO_RC_MAX))
    {
        SID_PAL_LOG_ERROR("%d is not a valid radio region ID", (int32_t)region);
        return RADIO_ERROR_INVALID_PARAMS;
    }

    /* Loop through the available regional params to find a matching config */
    for (uint32_t i = 0u; i < drv_ctx.config->regional_config.reg_param_table_size; i++)
    {
        if (drv_ctx.config->regional_config.reg_param_table[i].param_region == region)
        {
            s2_lp_hal_status_t hal_err;

            /* Found the desired regional config */
            drv_ctx.regional_radio_params = &drv_ctx.config->regional_config.reg_param_table[i];

            /* Update base frequency for the S2-LP */
            hal_err = s2_lp_radio_hal_set_base_frequency(&drv_ctx, drv_ctx.regional_radio_params->base_frequency_hz, drv_ctx.regional_radio_params->channel_spacing_hz);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                const uint32_t freq_khz = drv_ctx.regional_radio_params->base_frequency_hz / 1000u;
                SID_PAL_LOG_ERROR("Failed to set S2-LP base frequency to %u.%03uMHz. HAL error %u", (freq_khz / 1000u), (freq_khz % 1000u), (uint32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
            }

            err = RADIO_ERROR_NONE;
            break;
        }
    }

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_sleep(uint32_t sleep_us)
{
    int32_t err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    S2LP_RADIO_LOG_DEBUG("sid_pal_radio_sleep...");

    do
    {
        if (SID_PAL_RADIO_SLEEP == drv_ctx.radio_state)
        {
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Ensure the radio is in READY state because S2-LP does not allow transitions into low power mode directly from Tx/Rx */
        err = sid_pal_radio_standby();
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to set S2-LP to READY state before Sleep. Error %d", (int32_t)err);
            break;
        }

        if (UINT32_MAX == sleep_us)
        {
            /* Special case, keep reporting SID_PAL_RADIO_STANDBY for it */
            err = RADIO_ERROR_NONE;
            break;
        }

        /* Update RCO calibration if the expected sleep time is long enough and calibration process was not initiated already */
        if (sleep_us > (S2LP_RADIO_HAL_RCO_CALIBRATION_TIMEOUT_MS * 1000u))
        {
            /* Initiate RCO calibration to run in the background. Unfortunately, Sidewalk SDK design does not allow us to wait in this method for prolonged
             * periods of time, so we have to trigger the calibration and handle its outcomes asynchronously
             */
            if (sid_pal_timer_is_armed(&drv_ctx.rco_calibration.timer) == FALSE)
            {
                hal_err = s2_lp_radio_hal_calibrate_rco_async(&drv_ctx);
                if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
                {
                    SID_PAL_LOG_ERROR("Failed to start async RCO calibration. HAL error %u", (uint32_t)hal_err);
                    break;
                }
            }
            else
            {
                /* RCO calibration is running already - don't touch anything and allow RCO calibration to complete */
            }

            /* Don't do any further interactions with S2-LP iC here */
        }
        else
        {
            /* Do a dummy IRQ status read to clear any residual IRQ flags */
            hal_err = s2_lp_radio_hal_get_clear_irq_status(&drv_ctx, NULL);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to clear S2-LP IRQ flags. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            /* Transition S2-LP into STANDBY state (don't confuse with Sidewalk's SID_PAL_RADIO_STANDBY - it corresponds to S2-LP's READY state) */
            hal_err = s2_lp_radio_hal_set_standby(&drv_ctx);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to put S2-LP into Sleep state. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
        }

#if (CFG_LPM_LEVEL != 0)
        /* Allow Stop mode after the radio was put into Sleep */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
        UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_ENABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */

        /* Entered radio sleep state */
        drv_ctx.radio_state = SID_PAL_RADIO_SLEEP;
        err = RADIO_ERROR_NONE;
    } while(0);

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_standby(void)
{
    int32_t            err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;
    sid_error_t        sid_err;

    S2LP_RADIO_LOG_DEBUG("sid_pal_radio_standby...");

    do
    {
        /* Ensure RCO calibration is stopped if it was running */
        sid_err = sid_pal_timer_cancel(&drv_ctx.rco_calibration.timer);
        if (sid_err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("S2-LP failed to stop RCO calibration timer. Error %d", (int32_t)sid_err);
            err = RADIO_ERROR_IO_ERROR;
        }

        if (SID_PAL_RADIO_RX_DC == drv_ctx.radio_state)
        {
            /* Disable LDC mode */
            hal_err = s2_lp_radio_hal_set_ldc_mode(&drv_ctx, FALSE);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("S2-LP failed to disable LDC mode. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

#if S2LP_RADIO_CFG_USE_STATUS_LED
            /* Disable LDC Wakeup IRQ, it's not needed any longer */
            drv_ctx.radio_irq_mask.IRQ_WKUP_TOUT_LDC = S2_LP_EVENT_NOT_PRESENT;
            hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                /* Logs are provided by the s2_lp_radio_hal_set_radio_irq_mask() */
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */
        }

        /* S2-LP's READY state corresponds to the Standby state of the Sidewalk driver */
        hal_err = s2_lp_radio_hal_set_ready(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to put S2-LP into Standby state. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        /* Configure the radio front end module for Sleep */
        hal_err = s2_lp_radio_hal_set_fem_mode(&drv_ctx, S2_LP_RADIO_HAL_FEM_MODE_SHUTDOWN);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to turn FEM off. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Turn off the LED to indicate no activity is taking place any longer */
        (void)s2_lp_radio_hal_tx_led_off(&drv_ctx);
        (void)s2_lp_radio_hal_rx_led_off(&drv_ctx);
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        drv_ctx.radio_state = SID_PAL_RADIO_STANDBY;
        err = RADIO_ERROR_NONE;
    } while (0);

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_set_radio_busy(void)
{
    int32_t err = RADIO_ERROR_GENERIC;

    do
    {
        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY)
        {
            SID_PAL_LOG_ERROR("Cannot release the radio, it's not in Standby state")
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        drv_ctx.radio_state = SID_PAL_RADIO_BUSY;
        err = RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_is_cad_exit_mode(sid_pal_radio_cad_param_exit_mode_t exit_mode)
{
    int32_t err;

    if ( (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_CS_ONLY)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_CS_RX)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_ED_ONLY)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_ED_RX)
      && (exit_mode != SID_PAL_RADIO_CAD_EXIT_MODE_ED_LBT))
    {
        err = RADIO_ERROR_INVALID_PARAMS;
    }
    else
    {
        err = RADIO_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_carrier_sense(const sid_pal_radio_fsk_cad_params_t * cad_params, sid_pal_radio_cad_param_exit_mode_t exit_mode)
{
    int32_t err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    do
    {
        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Validate params */
        err = sid_pal_radio_is_cad_exit_mode(exit_mode);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Set Carrier Sense timeout */
        hal_err = s2_lp_radio_hal_set_rx_timeout(&drv_ctx, cad_params->fsk_cs_duration_us);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set S2-LP CS timeout. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Clear Rx FIFO */
        hal_err = s2_lp_radio_hal_flush_fifo(&drv_ctx, S2_LP_RADIO_HAL_FIFO_RX);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to flush S2-LP Rx FIFO. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        /* Configure the radio front end module for Rx */
        hal_err = s2_lp_radio_hal_set_fem_mode(&drv_ctx, S2_LP_RADIO_HAL_FEM_MODE_RX);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure FEM for Rx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

        /* Do a dummy IRQ status read to clear any residual IRQ flags */
        hal_err = s2_lp_radio_hal_get_clear_irq_status(&drv_ctx, NULL);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to clear S2-LP IRQ flags. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* For FSK Carrier Sense timeout timer shall be stopped either on preamble detection or on RSSI noise threshold crossing */
        hal_err = s2_lp_radio_hal_configure_packet_engine_for_rx(&drv_ctx, S2_LP_RX_TIMEOUT_STOP_RSSI_OR_PQI_ABOVE_THRESHOLD);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("Failed to configure S2-LP Rx timer stop conditions. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Enable IRQ on preamble detect or RSSI threshold crossing */
        drv_ctx.radio_irq_mask.IRQ_VALID_PREAMBLE = S2_LP_EVENT_ACTIVE;
        drv_ctx.radio_irq_mask.IRQ_RSSI_ABOVE_TH  = S2_LP_EVENT_ACTIVE;
        hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Logs are provided by the s2_lp_radio_hal_set_radio_irq_mask() */
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Store CAD exit mode */
        drv_ctx.cad_exit_mode = exit_mode;

        /* Go with Rx */
        hal_err = s2_lp_radio_hal_start_rx(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("S2-LP failed to initiate Rx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Rx started */
        drv_ctx.radio_state = SID_PAL_RADIO_RX;

#if (CFG_LPM_LEVEL != 0)
        /* Prohibit Stop mode until CS is done as we need fast response on the radio IRQs */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
            UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Turn on the LED to indicate the ongoing activity */
#  if (HALO_ENABLE_DIAGNOSTICS == 0)
        if (SID_PAL_RADIO_CAD_EXIT_MODE_CS_LBT == exit_mode)
        {
            /* This is carrier sense for FSK Tx */
            (void)s2_lp_radio_hal_tx_led_on(&drv_ctx);
        }
        else
#  endif /* HALO_ENABLE_DIAGNOSTICS */
        {
            (void)s2_lp_radio_hal_rx_led_on(&drv_ctx);
        }
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
    } while(0);

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_rx(uint32_t timeout)
{
    int32_t err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    do
    {
        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Reset internal buffer counters */
        drv_ctx.processing_buf_offset      = 0u;
        drv_ctx.processing_buf_total_bytes = 0u;

        /* Set Rx timeout */
        hal_err = s2_lp_radio_hal_set_rx_timeout(&drv_ctx, timeout);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set S2-LP Rx timeout. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Clear Rx FIFO */
        hal_err = s2_lp_radio_hal_flush_fifo(&drv_ctx, S2_LP_RADIO_HAL_FIFO_RX);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to flush S2-LP Rx FIFO. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        /* Configure the radio front end module for Rx */
        hal_err = s2_lp_radio_hal_set_fem_mode(&drv_ctx, S2_LP_RADIO_HAL_FEM_MODE_RX);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure FEM for Rx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

        /* Do a dummy IRQ status read to clear any residual IRQ flags */
        hal_err = s2_lp_radio_hal_get_clear_irq_status(&drv_ctx, NULL);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to clear S2-LP IRQ flags. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* For FSK Rx timeout timer shall be stopped on preamble detection */
        hal_err = s2_lp_radio_hal_configure_packet_engine_for_rx(&drv_ctx, S2_LP_RX_TIMEOUT_STOP_PQI_ABOVE_THRESHOLD);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("Failed to configure S2-LP Rx timer stop conditions. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure this operation won't be considered as CS/CAD */
        drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

        /* Go with Rx */
        hal_err = s2_lp_radio_hal_start_rx(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("S2-LP failed to initiate Rx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Rx started */
        drv_ctx.radio_state = SID_PAL_RADIO_RX;

#if (CFG_LPM_LEVEL != 0)
        /* Prohibit Stop mode until Rx is done as we need fast response on Rx Done IRQ  */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
        UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Turn on the LED to indicate the ongoing activity */
        (void)s2_lp_radio_hal_rx_led_on(&drv_ctx);
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
    } while(0);

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_continuous_rx(void)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    do
    {
        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Reset internal buffer counters */
        drv_ctx.processing_buf_offset      = 0u;
        drv_ctx.processing_buf_total_bytes = 0u;

        /* Clear Rx FIFO */
        hal_err = s2_lp_radio_hal_flush_fifo(&drv_ctx, S2_LP_RADIO_HAL_FIFO_RX);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to flush S2-LP Rx FIFO. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        /* Configure the radio front end module for Rx */
        hal_err = s2_lp_radio_hal_set_fem_mode(&drv_ctx, S2_LP_RADIO_HAL_FEM_MODE_RX);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure FEM for Rx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

        /* Do a dummy IRQ status read to clear any residual IRQ flags */
        hal_err = s2_lp_radio_hal_get_clear_irq_status(&drv_ctx, NULL);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to clear S2-LP IRQ flags. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Disable Rx timeout completely */
        hal_err = s2_lp_radio_hal_configure_packet_engine_for_rx(&drv_ctx, S2_LP_RX_TIMEOUT_STOP_NO_TIMEOUT);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("Failed to configure S2-LP Rx timer stop conditions. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure this operation won't be considered as CS/CAD */
        drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

        /* Go with Rx */
        hal_err = s2_lp_radio_hal_start_rx(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("S2-LP failed to initiate Rx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Rx started */
        drv_ctx.radio_state = SID_PAL_RADIO_RX;

#if (CFG_LPM_LEVEL != 0)
        /* Prohibit Stop mode until Rx is done as we need fast response on Rx Done IRQ */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
        UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Turn on the LED to indicate the ongoing activity */
        (void)s2_lp_radio_hal_rx_led_on(&drv_ctx);
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
    } while(0);

    _s2lp_radio_error_manager(err);
    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_set_rx_duty_cycle(uint32_t rx_time, uint32_t sleep_time)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    do
    {
        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Reset internal buffer counters */
        drv_ctx.processing_buf_offset      = 0u;
        drv_ctx.processing_buf_total_bytes = 0u;

        /* Set Rx timeout */
        hal_err = s2_lp_radio_hal_set_rx_timeout(&drv_ctx, rx_time);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set S2-LP Rx timeout. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Configure wait time */
        hal_err = s2_lp_radio_hal_set_ldc_timeout(&drv_ctx, sleep_time);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to set S2-LP LDC wait time. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Clear Rx FIFO */
        hal_err = s2_lp_radio_hal_flush_fifo(&drv_ctx, S2_LP_RADIO_HAL_FIFO_RX);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to flush S2-LP Rx FIFO. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        /* Configure the radio front end module for Rx */
        hal_err = s2_lp_radio_hal_set_fem_mode(&drv_ctx, S2_LP_RADIO_HAL_FEM_MODE_RX);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure FEM for Rx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

        /* Do a dummy IRQ status read to clear any residual IRQ flags */
        hal_err = s2_lp_radio_hal_get_clear_irq_status(&drv_ctx, NULL);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to clear S2-LP IRQ flags. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Enable LDC Wakeup IRQ to drive the LED */
        drv_ctx.radio_irq_mask.IRQ_WKUP_TOUT_LDC = S2_LP_EVENT_ACTIVE;
        hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Logs are provided by the s2_lp_radio_hal_set_radio_irq_mask() */
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        /* For FSK Rx timeout timer shall be stopped on preamble detection */
        hal_err = s2_lp_radio_hal_configure_packet_engine_for_rx(&drv_ctx, S2_LP_RX_TIMEOUT_STOP_PQI_ABOVE_THRESHOLD);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("Failed to configure S2-LP Rx timer stop conditions. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Ensure this operation won't be considered as CS/CAD */
        drv_ctx.cad_exit_mode = SID_PAL_RADIO_CAD_EXIT_MODE_NONE;

        /* Enable LDC mode */
        hal_err = s2_lp_radio_hal_set_ldc_mode(&drv_ctx, TRUE);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("S2-LP failed to enable LDC mode. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Go with Rx */
        hal_err = s2_lp_radio_hal_start_rx(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("S2-LP failed to initiate Rx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* LDC Rx started */
        drv_ctx.radio_state = SID_PAL_RADIO_RX_DC;

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Turn on the LED to indicate the ongoing activity */
        (void)s2_lp_radio_hal_rx_led_on(&drv_ctx);
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
    } while(0);

    _s2lp_radio_error_manager(err);
    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_tx_continuous_wave(uint32_t freq, int8_t power)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t            err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    do
    {
        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Update RF frequency */
        err = sid_pal_radio_set_frequency(freq);
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("CW Tx failed, unable to set RF frequency. Error %d", err);
            break;
        }

        /* Adjust Tx power */
        err = sid_pal_radio_set_tx_power(power);
        if (err != RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("CW Tx failed, unable to set Tx power. Error %d", err);
            break;
        }

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        /* Configure the radio front end module for Tx */
        s2_lp_hal_fem_mode_t fem_mode = (FALSE == drv_ctx.pa_params.ext_pa_en) ? S2_LP_RADIO_HAL_FEM_MODE_TX_LP : S2_LP_RADIO_HAL_FEM_MODE_TX_HP;

        hal_err = s2_lp_radio_hal_set_fem_mode(&drv_ctx, fem_mode);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure FEM for Tx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

        /* Do a dummy IRQ status read to clear any residual IRQ flags */
        hal_err = s2_lp_radio_hal_get_clear_irq_status(&drv_ctx, NULL);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to clear S2-LP IRQ flags. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        hal_err = s2_lp_radio_hal_configure_packet_engine_for_cw(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("Failed to configure S2-LP packet engine for CW. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Go with Tx */
        hal_err = s2_lp_radio_hal_start_tx(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("S2-LP failed to initiate CW Tx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Tx started */
        drv_ctx.radio_state = SID_PAL_RADIO_TX;

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Turn on the LED to indicate the ongoing activity */
        (void)s2_lp_radio_hal_tx_led_on(&drv_ctx);
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
    } while(0);

    _s2lp_radio_error_manager(err);
    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_tx_payload(const uint8_t * buffer, uint8_t size)
{
    int32_t            err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    do
    {
        /* Validate inputs */
        if ((NULL == buffer) || (0u == size))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* IMPORTANT: the size here will include CRC bytes. It's not a problem we put these redundant bytes into the Tx FIFO.
         *            Anyway, S2-LP will not touch these CRC dummy bytes, instead, it will calculate the actual CRC and transmit
         *            it. Unused bytes will remain in the Tx FIFO, the FIFO shall be flushed before the next packet can be uploaded to it
         */
        drv_ctx.last_tx_payload_size = size;

        /* Clear Tx FIFO */
        hal_err = s2_lp_radio_hal_flush_fifo(&drv_ctx, S2_LP_RADIO_HAL_FIFO_TX);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to flush S2-LP Tx FIFO. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Reset internal buffer counters */
        drv_ctx.processing_buf_offset      = 0u;
        drv_ctx.processing_buf_total_bytes = 0u;

        /* Upload as much as we can to the Tx FIFO */
        const uint32_t upload_size = size > S2_LP_IC_TX_FIFO_SIZE ? S2_LP_IC_TX_FIFO_SIZE : size;

        hal_err = s2_lp_radio_hal_write_to_tx_fifo(&drv_ctx, buffer, upload_size);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to write to S2-LP Tx FIFO. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Store the remaining data (if any) in the processing buffer */
        const uint32_t remaining_bytes = size - upload_size;
        
        if (remaining_bytes > 0u)
        {
            SID_PAL_ASSERT(remaining_bytes <= drv_ctx.config->internal_buffer.size); /*!< Runtime check takes place in prepare_for_tx() method, so assert here is enough */
            SID_STM32_UTIL_fast_memcpy(drv_ctx.config->internal_buffer.p, &buffer[upload_size], remaining_bytes);
            drv_ctx.processing_buf_total_bytes = remaining_bytes; /* Indicate how many valid bytes are present in the processing buffer */
            drv_ctx.processing_buf_offset      = 0u;              /* Valid data starts from the beginning of the processing buffer */

            /* Do a dummy IRQ status read to clear any residual IRQ flags */
            hal_err = s2_lp_radio_hal_get_clear_irq_status(&drv_ctx, NULL);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to clear S2-LP IRQ flags. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }

            /* Enable Tx FIFO Almost Empty IRQ since we have some data to upload into the FIFO once free space is available */
            drv_ctx.radio_irq_mask.IRQ_TX_FIFO_ALMOST_EMPTY = S2_LP_EVENT_ACTIVE;
            hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                /* Logs are provided by the s2_lp_radio_hal_set_radio_irq_mask() */
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_start_tx(uint32_t timeout)
{
    int32_t            err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    (void)timeout; /* S2-LP does not support Tx timeouts */

    do
    {
        if (drv_ctx.radio_state != SID_PAL_RADIO_STANDBY)
        {
            err = RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Ensure PA ramping is properly configured since S2-LP uses bit time as a reference */
        if (drv_ctx.current_bit_rate != drv_ctx.pa_params.pa_ramp_bit_rate)
        {
            /* PA ramping was configured for the different bit rate, we need to re-adjust the PA ramp settings */
            hal_err = s2_lp_radio_hal_configure_tx_power(&drv_ctx);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                SID_PAL_LOG_ERROR("Failed to adjust PA ramp time. HAL error %u", (uint32_t)hal_err);
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
        }

#if S2LP_RADIO_CFG_USE_EXTERNAL_PA
        /* Configure the radio front end module for Tx */
        s2_lp_hal_fem_mode_t fem_mode = (FALSE == drv_ctx.pa_params.ext_pa_en) ? S2_LP_RADIO_HAL_FEM_MODE_TX_LP : S2_LP_RADIO_HAL_FEM_MODE_TX_HP;

        hal_err = s2_lp_radio_hal_set_fem_mode(&drv_ctx, fem_mode);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to configure FEM for Tx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
#endif /* S2LP_RADIO_CFG_USE_EXTERNAL_PA */

        /* Do a dummy IRQ status read to clear any residual IRQ flags */
        hal_err = s2_lp_radio_hal_get_clear_irq_status(&drv_ctx, NULL);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to clear S2-LP IRQ flags. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        hal_err = s2_lp_radio_hal_configure_packet_engine_for_tx(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            S2LP_RADIO_LOG_ERROR("Failed to configure S2-LP packet engine for Tx. HAL error %u", hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Go with Tx */
        hal_err = s2_lp_radio_hal_start_tx(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("S2-LP failed to initiate Tx. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* Tx started */
        drv_ctx.radio_state = SID_PAL_RADIO_TX;

#if (CFG_LPM_LEVEL != 0)
        /* Prohibit Stop mode until Tx is done as we need fast response on Tx Done IRQ */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  if (CFG_LPM_STDBY_SUPPORTED != 0)
        UTIL_LPM_SetOffMode((1u << CFG_LPM_SIDEWALK), UTIL_LPM_DISABLE);
#  endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* (CFG_LPM_LEVEL != 0) */

#if S2LP_RADIO_CFG_USE_STATUS_LED
        /* Turn on the LED to indicate the ongoing activity */
        (void)s2_lp_radio_hal_tx_led_on(&drv_ctx);
#endif /* S2LP_RADIO_CFG_USE_STATUS_LED */

        err = RADIO_ERROR_NONE;
     } while(0);

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint8_t sid_pal_radio_get_status(void)
{
    return drv_ctx.radio_state;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_pal_radio_modem_mode_t sid_pal_radio_get_modem_mode(void)
{
    return SID_PAL_RADIO_MODEM_MODE_FSK;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_modem_mode(sid_pal_radio_modem_mode_t mode)
{
    int32_t            err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    S2LP_RADIO_LOG_DEBUG("sid_pal_radio_set_modem_mode...");

    do
    {
        /* S2-LP mode is fixed to 2-GFSK */
        if (SID_PAL_RADIO_MODEM_MODE_FSK != mode)
        {
            SID_PAL_LOG_ERROR("Unable to set S2-LP modem mode to %u. Requested mode is not supported", (uint32_t)mode);
            err = RADIO_ERROR_NOT_SUPPORTED;
            break;
        }

        /* Ensure the default IRQ mask is applied */
        if (drv_ctx.radio_irq_mask.raw != S2LP_RADIO_DEFAULT_FSK_IRQ_MASK)
        {
            drv_ctx.radio_irq_mask.raw = S2LP_RADIO_DEFAULT_FSK_IRQ_MASK;
            hal_err = s2_lp_radio_hal_set_radio_irq_mask(&drv_ctx, drv_ctx.radio_irq_mask);
            if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
            {
                /* Logs are provided by the s2_lp_radio_hal_set_radio_irq_mask() */
                err = RADIO_ERROR_HARDWARE_ERROR;
                break;
            }
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_is_channel_free(uint32_t freq, int16_t threshold, uint32_t delay_us, bool * is_channel_free)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t            err, tmp_error;
    sid_error_t        sid_err;
    s2_lp_hal_status_t hal_err;

    *is_channel_free = false;

    do
    {
        /* Set desired RF frequency */
        err = sid_pal_radio_set_frequency(freq);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Disable radio IRQs to avoid unwanted events */
        hal_err = s2_lp_radio_hal_disarm_irq(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
           S2LP_RADIO_LOG_ERROR("sid_pal_radio_is_channel_free - error in s2_lp_radio_hal_disarm_irq(). Error %u", (uint32_t)hal_err);
           err = RADIO_ERROR_HARDWARE_ERROR;
           break;
        }

        /* Initiate continuous Rx to be able to measure real-time RSSI */
        err = sid_pal_radio_start_continuous_rx();
        if (err != RADIO_ERROR_NONE)
        {
            goto enable_irq;
        }

        struct sid_timespec t_start, t_cur, t_threshold;
        int16_t rssi;

        /* Ensure the measurement time is enough for RSSI to settle */
        if (delay_us < S2LP_RADIO_MIN_CHANNEL_FREE_DELAY_US)
        {
            delay_us = S2LP_RADIO_MIN_CHANNEL_FREE_DELAY_US;
        }

        sid_us_to_timespec(delay_us, &t_threshold);

        sid_err = sid_pal_uptime_now(&t_start);
        if (sid_err != SID_ERROR_NONE)
        {
            err = RADIO_ERROR_GENERIC;
            goto enable_irq;
        }

        /* Measurement loop */
        do
        {
            /* Wait */
            sid_pal_delay_us(S2LP_RADIO_MIN_CHANNEL_FREE_DELAY_US);

            /* Read out current RSSI */
            rssi = sid_pal_radio_rssi();

            /* Check if the RSSI is below the threshold */
            if (rssi > threshold)
            {
                /* High RSSI detected - the channel is busy */
                *is_channel_free = false;
                err = RADIO_ERROR_NONE;
                goto enable_irq;
            }

            /* Compute elapsed time */
            sid_err = sid_pal_uptime_now(&t_cur);
            if (sid_err != SID_ERROR_NONE)
            {
                err = RADIO_ERROR_GENERIC;
                goto enable_irq;
            }
            sid_time_sub(&t_cur, &t_start);
        } while (sid_time_gt(&t_threshold, &t_cur));

        /* Measurement period has elapsed and no high RSSI was detected if we got here - the channel is free*/
        *is_channel_free = true;

        /* Re-enable radio IRQs regardless of success or failure */
enable_irq:
        /* 1. Stop continuous Rx by moving the radio into standby mode */
        tmp_error = sid_pal_radio_standby();
        if (tmp_error != RADIO_ERROR_NONE)
        {
            err = tmp_error;
            break;
        }

        /* 2. Clear any residual IRQ indications */
        hal_err = s2_lp_radio_hal_get_clear_irq_status(&drv_ctx, NULL);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read out S2-LP IRQ status. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* 3. Re-enable IRQ line */
        hal_err = s2_lp_radio_hal_arm_irq(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Logs are provided by the s2_lp_radio_hal_arm_irq() */
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
    } while (0);

    _s2lp_radio_error_manager(err);
    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_chan_noise(uint32_t freq, int16_t * noise)
{
#if HALO_ENABLE_DIAGNOSTICS
    int32_t            err, tmp_error;
    s2_lp_hal_status_t hal_err;
    int32_t            meas_accum;

    *noise = 0;

    do
    {
        /* Set desired RF frequency */
        err = sid_pal_radio_set_frequency(freq);
        if (err != RADIO_ERROR_NONE)
        {
            break;
        }

        /* Disable radio IRQs to avoid unwanted events */
        hal_err = s2_lp_radio_hal_disarm_irq(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
           S2LP_RADIO_LOG_ERROR("sid_pal_radio_is_channel_free - error in s2_lp_radio_hal_disarm_irq(). Error %u", (uint32_t)hal_err);
           err = RADIO_ERROR_HARDWARE_ERROR;
           break;
        }

        /* Initiate continuous Rx to be able to measure real-time RSSI */
        err = sid_pal_radio_start_continuous_rx();
        if (err != RADIO_ERROR_NONE)
        {
            goto enable_irq;
        }

        /* Measurement loop */
        meas_accum = 0;
        for (uint32_t i = 0u; i < S2LP_RADIO_NOISE_SAMPLE_SIZE; i++)
        {
            sid_pal_delay_us(S2LP_RADIO_MIN_CHANNEL_NOISE_DELAY_US);
            meas_accum += sid_pal_radio_rssi();
        }

        *noise = meas_accum / S2LP_RADIO_NOISE_SAMPLE_SIZE;

        /* Re-enable radio IRQs regardless of success or failure */
enable_irq:
        /* 1. Stop continuous Rx by moving the radio into standby mode */
        tmp_error = sid_pal_radio_standby();
        if (tmp_error != RADIO_ERROR_NONE)
        {
            err = tmp_error;
            break;
        }

        /* 2. Clear any residual IRQ indications */
        hal_err = s2_lp_radio_hal_get_clear_irq_status(&drv_ctx, NULL);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read out S2-LP IRQ status. HAL error %u", (uint32_t)hal_err);
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        /* 3. Re-enable IRQ line */
        hal_err = s2_lp_radio_hal_arm_irq(&drv_ctx);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            /* Logs are provided by the s2_lp_radio_hal_arm_irq() */
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }
    } while (0);

    _s2lp_radio_error_manager(err);
    return err;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int16_t sid_pal_radio_rssi(void)
{
#if HALO_ENABLE_DIAGNOSTICS
    s2_lp_hal_status_t hal_err;
    s2_lp_hal_rssi_t   rssi = {0};

    hal_err = s2_lp_radio_hal_get_rssi(&drv_ctx, &rssi);
    if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
    {
        SID_PAL_LOG_ERROR("Failed to get real-time RSSI reading. HAL error %u", (uint32_t)hal_err);
    }

    return (int16_t)rssi.adjusted_rssi;
#else
    return RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_random(uint32_t *random)
{
    SID_PAL_LOG_WARNING("sid_pal_radio_random() is not implemented");
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int16_t sid_pal_radio_get_ant_dbi(void)
{
    SID_PAL_ASSERT(drv_ctx.regional_radio_params != NULL);
    return drv_ctx.regional_radio_params->ant_dbi;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_cca_level_adjust(sid_pal_radio_data_rate_t data_rate, int8_t *adj_level)
{
    int32_t err;

    SID_PAL_ASSERT(drv_ctx.regional_radio_params != NULL);

    if ((data_rate <= SID_PAL_RADIO_DATA_RATE_INVALID) || (data_rate > SID_PAL_RADIO_DATA_RATE_MAX_NUM))
    {
        err = RADIO_ERROR_INVALID_PARAMS;
    }
    else
    {
        *adj_level = drv_ctx.regional_radio_params->cca_level_adjust[data_rate - 1];
        err = RADIO_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_get_radio_state_transition_delays(sid_pal_radio_state_transition_timings_t * state_delay)
{
    SID_PAL_ASSERT(drv_ctx.config != NULL);
    static_assert(sizeof(*state_delay) == sizeof(drv_ctx.config->state_timings));

    SID_STM32_UTIL_fast_memcpy(state_delay, &drv_ctx.config->state_timings, sizeof(*state_delay));
    return RADIO_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_set_lora_symbol_timeout(uint8_t num_of_symbols)
{
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_set_lora_sync_word(uint16_t sync_word)
{
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_set_lora_modulation_params(const sid_pal_radio_lora_modulation_params_t *mod_params)
{
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_set_lora_packet_params(const sid_pal_radio_lora_packet_params_t *packet_params)
{
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_set_lora_cad_params(const sid_pal_radio_lora_cad_params_t *cad_params)
{
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_lora_start_cad(void)
{
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

sid_pal_radio_data_rate_t sid_pal_radio_lora_mod_params_to_data_rate(const sid_pal_radio_lora_modulation_params_t *mod_params)
{
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_lora_data_rate_to_mod_params(sid_pal_radio_lora_modulation_params_t *mod_params,
                                           sid_pal_radio_data_rate_t data_rate, uint8_t li_enable)
{
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

uint32_t sid_pal_radio_lora_time_on_air(const sid_pal_radio_lora_modulation_params_t *mod_params,
                               const sid_pal_radio_lora_packet_params_t *packet_params, uint8_t packet_len)
{
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

uint32_t sid_pal_radio_lora_cad_duration(uint8_t symbol, const sid_pal_radio_lora_modulation_params_t *mod_params)
{
    return 0u;
}

/*----------------------------------------------------------------------------*/

uint32_t sid_pal_radio_lora_get_lora_number_of_symbols(const sid_pal_radio_lora_modulation_params_t *mod_params,
                                               uint32_t delay_micro_sec)
{
    return 0u;
}

/*----------------------------------------------------------------------------*/

uint32_t sid_pal_radio_get_lora_rx_done_delay(const sid_pal_radio_lora_modulation_params_t* mod_params,
                                            const sid_pal_radio_lora_packet_params_t* pkt_params)
{
    return 0u;
}

/*----------------------------------------------------------------------------*/

uint32_t sid_pal_radio_get_lora_tx_process_delay(void)
{
    return 0u;
}

/*----------------------------------------------------------------------------*/

uint32_t sid_pal_radio_get_lora_rx_process_delay(void)
{
    return 0u;
}

/*----------------------------------------------------------------------------*/

uint32_t sid_pal_radio_get_lora_symbol_timeout_us(sid_pal_radio_lora_modulation_params_t *mod_params, uint8_t number_of_symbol)
{
    return 0u;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_fsk_sync_word(const uint8_t * sync_word, uint8_t sync_word_length)
{
    /* FSK part of the Sidewalk currently uses 0x904E (when FEC is disabled) and 0x6F4E (when FEC is enabled) sync words only
     * S2-LP allows to set primary and secondary Sync Words at once. Additionally, in 802.15.4g mode secondary sync word is forced
     * for Tx when FEC_EN bit is set in PCKTCTRL1 register. For Rx, FEC configuration is detected automatically by matching the
     * sync word - if the packet start with the sync word stored in SYNC3..0 regs FEC is not used, if the packet starts with the
     * sync word stored in SEC_SYNC3..0 register FEC is automatically activated regardless of the FEC_EN bit state. All of this
     * means we don't need to set sync words here every time, it's enough to configure both primary and secondary sync words at
     * startup and that's it
     */

    int32_t err = RADIO_ERROR_GENERIC;

    do
    {
        if ((NULL == sync_word) || (0u == sync_word_length))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Skip any preamble bytes that Sidewalk SDK may put into the sync word for compatibility and/or legacy reasons */
        uint32_t real_sync_word_start_idx = 0u;
        while ((real_sync_word_start_idx < sync_word_length) && (S2LP_RADIO_SIDEWALK_FSK_PREAMBLE_SYMB == sync_word[real_sync_word_start_idx]))
        {
            real_sync_word_start_idx++;
        }
        

        /* Perform a few checks to detect possible future Sidewalk SDK modifications */
        if ((sync_word_length - real_sync_word_start_idx) != SID_STM32_UTIL_ARRAY_SIZE(fsk_sync_word_fec_disabled))
        {
            SID_PAL_LOG_ERROR("Unexpected FSK sync word length: %u. Probably the Sidewalk stack was changed", sync_word_length);
            err = RADIO_ERROR_NOT_SUPPORTED;
            break;
        }

        err = RADIO_ERROR_NONE;
        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(fsk_sync_word_fec_disabled); i++)
        {
            if ((sync_word[i + real_sync_word_start_idx] != fsk_sync_word_fec_disabled[i]) && (sync_word[i + real_sync_word_start_idx] != fsk_sync_word_fec_enabled[i]))
            {
                SID_PAL_LOG_ERROR("Unexpected FSK sync word byte %u: 0x%02X. Probably the Sidewalk stack was changed", i, sync_word[i]);
                err = RADIO_ERROR_NOT_SUPPORTED;
                break;
            }
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_fsk_whitening_seed(uint16_t seed)
{
    /* S2-LP performs on-chip whitening */
    return RADIO_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_fsk_modulation_params(const sid_pal_radio_fsk_modulation_params_t * mod_params)
{
    int32_t            err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    do
    {
        s2_lp_hal_mod_params_t s2lp_mod_params;

        /* Validate the inputs */
        if (NULL == mod_params)
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Convert Sidewalk modulation parameters to the S2-LP modulation parameters */
        err = _s2lp_radio_sid_modp_to_s2lp_modp(mod_params, &s2lp_mod_params);
        if (err != RADIO_ERROR_NONE)
        {
            /* Logs are provided by _s2lp_radio_sid_modp_to_s2lp_modp() */
            break;
        }

        /* Apply the modulation parameters */
        hal_err = s2_lp_radio_hal_set_mod_params(&drv_ctx, &s2lp_mod_params);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply FSK modulation params. Error %d", (int32_t)err);
    }

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_set_fsk_packet_params(const sid_pal_radio_fsk_packet_params_t * packet_params)
{
    int32_t            err = RADIO_ERROR_GENERIC;
    s2_lp_hal_status_t hal_err;

    do
    {
        s2_lp_hal_pckt_params_t s2lp_packet_params;

        /* Validate the inputs */
        if (NULL == packet_params)
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        /* Convert Sidewalk FSK packet parameters to the S2-LP packet parameters */
        err = _s2lp_radio_sid_pcktp_to_s2lp_pcktp(packet_params, &s2lp_packet_params);
        if (err != RADIO_ERROR_NONE)
        {
            /* Logs are provided by _s2lp_radio_sid_pcktp_to_s2lp_pcktp() */
            break;
        }

        /* Apply the modulation parameters */
        hal_err = s2_lp_radio_hal_set_pckt_params(&drv_ctx, &s2lp_packet_params);
        if (hal_err != S2_LP_RADIO_HAL_STATUS_OK)
        {
            err = RADIO_ERROR_HARDWARE_ERROR;
            break;
        }

        err = RADIO_ERROR_NONE;
    } while (0);

    if (err != RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply FSK packet params. Error %d", (int32_t)err);
    }

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_pal_radio_data_rate_t sid_pal_radio_fsk_mod_params_to_data_rate(const sid_pal_radio_fsk_modulation_params_t * mp)
{
    sid_pal_radio_data_rate_t data_rate;

    if (       (RADIO_FSK_BR_50KBPS  == mp->bit_rate) && (RADIO_FSK_FDEV_25KHZ  == mp->freq_dev) && (SID_PAL_RADIO_FSK_BW_125KHZ == mp->bandwidth))
    {
        data_rate =  SID_PAL_RADIO_DATA_RATE_50KBPS;
    } else if ((RADIO_FSK_BR_150KBPS == mp->bit_rate) && (RADIO_FSK_FDEV_37_5KHZ == mp->freq_dev) && (SID_PAL_RADIO_FSK_BW_250KHZ == mp->bandwidth))
    {
        data_rate = SID_PAL_RADIO_DATA_RATE_150KBPS;
    } else if ((RADIO_FSK_BR_250KBPS == mp->bit_rate) && (RADIO_FSK_FDEV_62_5KHZ == mp->freq_dev) && (SID_PAL_RADIO_FSK_BW_500KHZ == mp->bandwidth))
    {
        data_rate = SID_PAL_RADIO_DATA_RATE_250KBPS;
    }
    else
    {
        data_rate = SID_PAL_RADIO_DATA_RATE_INVALID;
    }

    return data_rate;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_fsk_data_rate_to_mod_params(sid_pal_radio_fsk_modulation_params_t * mod_params, sid_pal_radio_data_rate_t  data_rate)
{
    int32_t err  = RADIO_ERROR_GENERIC;

    do
    {
        if (NULL == mod_params)
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break; 
        }

        switch (data_rate)
        {
            case SID_PAL_RADIO_DATA_RATE_50KBPS:
                mod_params->bit_rate     = RADIO_FSK_BR_50KBPS;
                mod_params->freq_dev     = RADIO_FSK_FDEV_25KHZ;
                mod_params->bandwidth    = SID_PAL_RADIO_FSK_BW_125KHZ;
                mod_params->mod_shaping  = SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_1;
                err                      = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_DATA_RATE_150KBPS:
                mod_params->bit_rate     = RADIO_FSK_BR_150KBPS;
                mod_params->freq_dev     = RADIO_FSK_FDEV_37_5KHZ;
                mod_params->bandwidth    = SID_PAL_RADIO_FSK_BW_250KHZ;
                mod_params->mod_shaping  = SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_05;
                err                      = RADIO_ERROR_NONE;
                break;

            case SID_PAL_RADIO_DATA_RATE_250KBPS:
                mod_params->bit_rate     = RADIO_FSK_BR_250KBPS;
                mod_params->freq_dev     = RADIO_FSK_FDEV_62_5KHZ;
                mod_params->bandwidth    = SID_PAL_RADIO_FSK_BW_500KHZ;
                mod_params->mod_shaping  = SID_PAL_RADIO_FSK_MOD_SHAPING_G_BT_05;
                err                      = RADIO_ERROR_NONE;
                break;

            default:
                err = RADIO_ERROR_INVALID_PARAMS;
                break;
        }
    } while (0); 

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_fsk_time_on_air(const sid_pal_radio_fsk_modulation_params_t * mod_params,
                                                                 const sid_pal_radio_fsk_packet_params_t * packet_params,
                                                                 uint8_t packet_len)
{
    /* Compute the total amount of bits that will be transmitted, from preamble till the very end of the frame */
    const uint32_t preamble_len_bits   = packet_params->preamble_length << 3;
    const uint32_t sync_word_len_bits  = packet_params->sync_word_length << 3;
    const uint32_t packet_length_bytes = S2LP_RADIO_HAL_SIDEWALK_FSK_HEADER_SIZE + packet_len + _s2lp_radio_get_fsk_crc_len_in_bytes(packet_params->crc_type);
    const uint32_t frame_length_bits   = preamble_len_bits + sync_word_len_bits + (packet_length_bytes << 3);

    /* Now calculate the actual time on air in milliseconds with round up (ceil) */
    uint32_t time_on_air = ((1000u * frame_length_bits) + (mod_params->bit_rate - 1u)) / mod_params->bit_rate;

    return time_on_air;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_fsk_get_fsk_number_of_symbols(const sid_pal_radio_fsk_modulation_params_t * mod_params, uint32_t delay_micro_secs)
{
    uint32_t symbols = (uint32_t)((uint64_t)((uint64_t)delay_micro_secs * (uint64_t)mod_params->bit_rate) / (uint64_t)1000000u);

    return symbols;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_get_fsk_tx_process_delay(void)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    SID_PAL_ASSERT(drv_ctx.config != NULL);

    const uint32_t fsk_tx_process_delay =
        (S2LP_RADIO_CFG_USE_DEFAULT_TIMINGS == drv_ctx.config->processing_timings.tx_process_delay_us) ?
        S2LP_RADIO_FSK_TX_DEFAULT_PROCESS_DELAY_US : drv_ctx.config->processing_timings.tx_process_delay_us;

    const uint32_t process_delay = fsk_tx_process_delay - S2LP_RADIO_SIDEWALK_FSK_CS_DURATION_US;
    return process_delay;
#else
    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t sid_pal_radio_get_fsk_rx_process_delay(void)
{
#if SID_SDK_CONFIG_ENABLE_LINK_TYPE_2
    SID_PAL_ASSERT(drv_ctx.config != NULL);

    const uint32_t fsk_rx_process_delay =
        (S2LP_RADIO_CFG_USE_DEFAULT_TIMINGS == drv_ctx.config->processing_timings.rx_process_delay_us) ?
        S2LP_RADIO_FSK_RX_DEFAULT_PROCESS_DELAY_US : drv_ctx.config->processing_timings.rx_process_delay_us;

    const uint32_t process_delay = fsk_rx_process_delay + S2LP_RADIO_FSK_RX_PROCESS_SAFETY_GAP_US;
    return process_delay;
#else
    return 0u;
#endif /* SID_SDK_CONFIG_ENABLE_LINK_TYPE_2 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_prepare_fsk_for_tx(sid_pal_radio_fsk_pkt_cfg_t *tx_pkt_cfg)
{
    int32_t err  = RADIO_ERROR_GENERIC;

    do
    {
        if ((NULL == tx_pkt_cfg) || (NULL == tx_pkt_cfg->phy_hdr) || (NULL == tx_pkt_cfg->packet_params) || (NULL == tx_pkt_cfg->sync_word))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        sid_pal_radio_fsk_packet_params_t * const pckt_params = tx_pkt_cfg->packet_params;
        if ((0u == pckt_params->payload_length) || (0u == pckt_params->preamble_length))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        sid_pal_radio_fsk_phy_hdr_t * const phr = tx_pkt_cfg->phy_hdr;
        if (pckt_params->payload_length > (drv_ctx.config->internal_buffer.size + S2_LP_IC_TX_FIFO_SIZE))
        {
            SID_PAL_LOG_ERROR("Unable to Tx %u bytes - exceeds the size of processing buffer (%u)", pckt_params->payload_length, (drv_ctx.config->internal_buffer.size + S2_LP_IC_TX_FIFO_SIZE));
            err = RADIO_ERROR_NOMEM;
            break;
        }

        /* Process the sync word based on the FEC selection - updating the sync word is done for the Sidewalk stack compatibility. Actual sync words in S2-LP are not affected by this */
        const uint8_t * sw_to_use     = (FALSE ==  phr->is_fec_enabled) ? fsk_sync_word_fec_disabled : fsk_sync_word_fec_enabled;
        uint32_t        preamb_offset = 0u;

        tx_pkt_cfg->sync_word[preamb_offset] = S2LP_RADIO_SIDEWALK_FSK_PREAMBLE_SYMB;
        preamb_offset++;

        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(fsk_sync_word_fec_disabled); i++)
        {
            tx_pkt_cfg->sync_word[i + preamb_offset] = sw_to_use[i];
        }

        /* Determine CRC size */
        uint32_t crc_size = RADIO_FSK_FCS_TYPE_0 == phr->fcs_type ? 4u : 2u;

        /* Adjust packet params */
        pckt_params->preamble_min_detect  = SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_08_BITS;    /* This value is used for CS/CAD - if preamble is detected the Tx operation will be aborted */
        pckt_params->addr_comp            = SID_PAL_RADIO_FSK_ADDRESSCOMP_FILT_OFF;         /* Not actually used by S2-LP */
        pckt_params->header_type          = SID_PAL_RADIO_FSK_RADIO_PACKET_FIXED_LENGTH;    /* Tx size is known */
        pckt_params->payload_length      += crc_size;                                       /* This has to cover CRC bytes as well - S2-LP will read form TX FIFO only the actual payload, without CRC bytes, but packet length register shall cover CRC */
        pckt_params->crc_type             = RADIO_FSK_FCS_TYPE_0 == phr->fcs_type ?         /* Select CRC/FCS type as requested */
                                                SID_PAL_RADIO_FSK_CRC_4_BYTES_ADCCP :
                                                SID_PAL_RADIO_FSK_CRC_2_BYTES_CCIT;
        pckt_params->radio_whitening_mode = FALSE != phr->is_data_whitening_enabled ?       /* Set data whitening mode according to the requested setting */
                                                SID_PAL_RADIO_FSK_DC_FREEWHITENING :
                                                SID_PAL_RADIO_FSK_DC_FREE_OFF;

        /* Update FEC setting in register cache directly, it will be applied with the next packet parameters update */
        drv_ctx.regs_cache.pckt_cfg_regs.pcktctrl1.FEC_EN = (FALSE ==  phr->is_fec_enabled) ? 0u : 1u;

        /* Payload processing is not needed - PHR will be created by S2-LP's packet engine, CRC will be calculated and added by S2-LP as well */

        err = RADIO_ERROR_NONE;
    } while (0);

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int32_t sid_pal_radio_prepare_fsk_for_rx(sid_pal_radio_fsk_pkt_cfg_t *rx_pkt_cfg)
{
    int32_t err  = RADIO_ERROR_GENERIC;

    do
    {
        if ((NULL == rx_pkt_cfg) || (NULL == rx_pkt_cfg->phy_hdr) || (NULL == rx_pkt_cfg->packet_params) || (NULL == rx_pkt_cfg->sync_word))
        {
            err = RADIO_ERROR_INVALID_PARAMS;
            break;
        }

        sid_pal_radio_fsk_packet_params_t * const pckt_params = rx_pkt_cfg->packet_params;
        sid_pal_radio_fsk_phy_hdr_t       * const phr         = rx_pkt_cfg->phy_hdr;

        /* Process the sync word based on the FEC selection - updating the sync word is done for the Sidewalk stack compatibility. Actual sync words in S2-LP are not affected by this */
        const uint8_t * sw_to_use     = (FALSE ==  phr->is_fec_enabled) ? fsk_sync_word_fec_disabled : fsk_sync_word_fec_enabled;
        uint32_t        preamb_offset = 0u;

        rx_pkt_cfg->sync_word[preamb_offset] = S2LP_RADIO_SIDEWALK_FSK_PREAMBLE_SYMB;
        preamb_offset++;

        for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(fsk_sync_word_fec_disabled); i++)
        {
            rx_pkt_cfg->sync_word[i + preamb_offset] = sw_to_use[i];
        }

        /* Adjust packet params */
        pckt_params->preamble_length      = 3u;                                             /* Keep this consistent with the preamble_min_detect setting: this value will determine how many
                                                                                             * BYTES should be received to consider the preamble valid, while preamble_min_detect specifies
                                                                                             * the threshold after which the Rx timeout timer is stopped. So, preamble_length shall be at
                                                                                             * least preamble_min_detect + 1 byte
                                                                                             */
        pckt_params->preamble_min_detect  = SID_PAL_RADIO_FSK_PREAMBLE_DETECTOR_16_BITS;    /* Rx timeout timer is stopped only after this amount of toggling bits is received */
        pckt_params->addr_comp            = SID_PAL_RADIO_FSK_ADDRESSCOMP_FILT_OFF;         /* Not actually used by S2-LP */
        pckt_params->header_type          = SID_PAL_RADIO_FSK_RADIO_PACKET_VARIABLE_LENGTH; /* We don't know the future Rx size */
        pckt_params->payload_length       = 0u;                                             /* Has no effect on S2-LP since dynamic packet length will be extracted from the 802.15.4g header */
        pckt_params->crc_type             = RADIO_FSK_FCS_TYPE_0 == phr->fcs_type ?         /* CRC/FCS type will be detected by S2-LP automatically from the 802.15.4g header, but still let's keep the settings consistent */
                                                SID_PAL_RADIO_FSK_CRC_4_BYTES_ADCCP :
                                                SID_PAL_RADIO_FSK_CRC_2_BYTES_CCIT;
        pckt_params->radio_whitening_mode = SID_PAL_RADIO_FSK_DC_FREEWHITENING;             /* S2-LP automatically detects if whitening is applied from 802.15.4g header, so this setting has
                                                                                             * no effect, but let's keep it consistent with the Tx operations
                                                                                             */

        /* Update FEC setting in register cache directly, it will be applied with the next packet parameters update */
        drv_ctx.regs_cache.pckt_cfg_regs.pcktctrl1.FEC_EN = (FALSE ==  phr->is_fec_enabled) ? 0u : 1u;

        err = RADIO_ERROR_NONE;
    } while (0);

    _s2lp_radio_error_manager(err);
    return err;
}

/*----------------------------------------------------------------------------*/

int32_t sid_pal_radio_set_fsk_crc_polynomial(uint16_t crc_polynomial, uint16_t crc_seed)
{
    SID_PAL_LOG_WARNING("sid_pal_radio_set_fsk_crc_polynomial() is not supported, S2-LP uses built-in packet engine to handle CRC");
    return RADIO_ERROR_NOT_SUPPORTED;
}

/*----------------------------------------------------------------------------*/

#if SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT
SID_STM32_SPEED_OPTIMIZED void sid_pal_radio_rxtx_start_cb(void)
{
    /* Keep this function as short as possible to minimize its contribution to the Rx/Tx processing delays */
    SIDEWALK_RF_TIMINGS_RXTX_START_GPIO_Port->BSRR = SIDEWALK_RF_TIMINGS_RXTX_START_Pin;
}
#endif /* SID_SDK_CONFIG_RF_TIMINGS_MEASUREMENT */
