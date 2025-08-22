/**
  ******************************************************************************
  * @file    sid_radio.c
  * @brief   SubGHz peripheral handling: app commands and IRQ processing
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

/* App-specific headers */
#include "host_comm.h"
#include "serial_bus_spi_pal.h"
#include "sid_pal_log_like.h"
#include "sid_radio.h"
#include "sys_conf.h"
#include "utilities_def.h"
#include SID_APP_VERSION_HEADER
#include <FreeRTOS.h>

/* SPI protocol definitions */
#include <comm_opcodes.h>

/* STM32 Utilities */
#include "sid_pal_log_like.h"
#include <sid_stm32_common_utils.h>
#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
#  include <stm32_lpm.h>
#endif /* LOW_POWER_DISABLE */
#include <stm32_timer.h>
#include <stm32_timer_us.h>

/* Platform definitions */
#include "radio_driver.h"

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief Stores Rx Buffer Status as reported by SubGHz
 * @note This struct uses uint32 values for storage to speed up further processing
 */
typedef struct {
    uint32_t pld_len_in_bytes;      /*!< Number of bytes available in the buffer */
    uint32_t buffer_start_pointer;  /*!< Position of the first byte in the buffer */
} stm32wlxx_rx_buffer_status_t;

/**
 * @brief SubGHz LoRa packet status structure definition
 */
typedef __PACKED_STRUCT {
    int8_t rssi_pkt_in_dbm;         /*!< RSSI of the last packet */
    int8_t snr_pkt_in_db;           /*!< SNR of the last packet */
    int8_t signal_rssi_pkt_in_dbm;  /*!< Estimation of RSSI (after despreading) */
} stm32wlxx_subghz_pkt_status_lora_t;

/**
 * @brief RADIO FSK status enumeration definition
 */
typedef enum {
    STM32WLxx_RADIO_FSK_RX_DONE_STATUS_OK                  = 0,
    STM32WLxx_RADIO_FSK_RX_DONE_STATUS_INVALID_PARAMETER   = 1,
    STM32WLxx_RADIO_FSK_RX_DONE_STATUS_INVALID_LENGTH      = 2,
    STM32WLxx_RADIO_FSK_RX_DONE_STATUS_BAD_CRC             = 3,
    STM32WLxx_RADIO_FSK_RX_DONE_STATUS_TIMEOUT             = 4,
    STM32WLxx_RADIO_FSK_RX_DONE_STATUS_UNKNOWN_ERROR       = 5,
    STM32WLxx_RADIO_FSK_RX_DONE_STATUS_SW_MARK_NOT_PRESENT = 6,
    STM32WLxx_RADIO_FSK_RX_DONE_STATUS_UNDETERMINED        = 7,
} stm32wlxx_radio_fsk_rx_done_status_t;

/**
 * @brief SubGHz fallback modes enumeration definition
 */
typedef enum {
    STM32WLxx_RADIO_FALLBACK_STDBY_RC   = 0x20,
    STM32WLxx_RADIO_FALLBACK_STDBY_XOSC = 0x30,
    STM32WLxx_RADIO_FALLBACK_FS         = 0x40,
} stm32wlxx_subghz_fallback_modes_t;

typedef enum {
    STM32WLxx_RADIO_FRONTEND_MODE_OFF = 0,
    STM32WLxx_RADIO_FRONTEND_MODE_TX  = 1,
    STM32WLxx_RADIO_FRONTEND_MODE_RX  = 2,
} stm32wlxx_subghz_fe_mode_t;

typedef enum {
    STM32WLxx_RADIO_SLEEP_COLD_START = 0, /*!< No register retention in Sleep state */
    STM32WLxx_RADIO_SLEEP_WARM_START = 1, /*!< Start with registers values retained from last sleep */
} stm32wlxx_subghz_sleep_type_t;

/**
 * @brief SubGHz SetSleep command parameters
 *
 * @note This definition is valid for Little Endian architecture only
 */
typedef __PACKED_UNION {
    __PACKED_STRUCT {
        uint8_t                       rtc_wakeup_en: 1; /*!< 1: Get out of sleep mode if wakeup signal received from RTC; 0: RTC won't wake up the radio */
        uint8_t                                    : 1; /*!< Reserved */
        stm32wlxx_subghz_sleep_type_t sleep_type   : 1;
        uint8_t                                    : 5; /*!< Reserved */
    };
    uint8_t raw;
} stm32wlxx_subghz_sleep_params_t;

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
typedef struct {
    osThreadId_t                        task;
    osMessageQueueId_t                  outbound_msg_queue;
    volatile uint32_t                   udt_enabled;
    sid_host_comm_on_incoming_user_data on_user_data_rx;
    UTIL_TIMER_Object_t                 suspend_timer;
} sid_radio_udt_ctx_t;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/* Private defines -----------------------------------------------------------*/

/* Timeouts and retry limits */
#define SID_RADIO_SUBGHZ_RESET_ATTEMPTS                   (5u)
#define SID_RADIO_SUBGHZ_RESET_COOLDOWN_MS                (10u)

#define SID_RADIO_SUBGHZ_RESET_TOGGLE_DELAY_US            (100u)  /*!< Defines the delay time in microseconds for toggling the radio reset signal */
#define SID_RADIO_SUBGHZ_WAIT_FOR_RESET_TIMEOUT_MS        (5u)    /*!< Defines timeout in milliseconds for radio reset to finish */
#define SID_RADIO_SUBGHZ_WAIT_FOR_STARTUP_TIMEOUT_MS      (10u)   /*!< Defines timeout in milliseconds for radio busy signal to release */
#define SID_RADIO_SUBGHZ_WAIT_FOR_WAKEUP_TIMEOUT_MS       (5u)    /*!< Timeout in milliseconds for radio busy signal to be released when leaving Sleep mode */
#define SID_RADIO_HANDSHAKE_LPM_SUSPEND_TIME_MS           (1000u) /*!< LPM entry will be prohibited for this period of time after receiving a handshake request (either internal or external) */
#define SID_RADIO_LPM_RESUME_RETRY_PERIOD_MS              (25u)   /*!< If LPM resume fails for any reason the driver will retry to resume LPM with this period */

#define SID_RADIO_IRQ_PROCESSING_DELAY_STATIC_ERROR_NS    (3120u) /*!< Compensatory time for SubGHz IRQ reaction time */

#ifndef STM32WLxx_SUBGHZ_TXPWR_WORKAROUND
#  define STM32WLxx_SUBGHZ_TXPWR_WORKAROUND 1
#endif

#if STM32WLxx_SUBGHZ_TXPWR_WORKAROUND
#  define STM32WLxx_SUBGHZ_REG_LR_CFG_FREQ                (0x745u)
#  define STM32WLxx_SUBGHZ_BAND_EDGE_LIMIT_FREQ           (903000000u)
#  define STM32WLxx_SUBGHZ_REG_VAL_FREQ_LOW               (0x1Fu)
#  define STM32WLxx_SUBGHZ_REG_VAL_FREQ_HIGH              (0x18u)

#  define STM32WLxx_SUBGHZ_REG_OVP                        (0x08E6u)
#endif /* STM32WLxx_SUBGHZ_TXPWR_WORKAROUND */

#define STM32WLxx_SUBGHZ_REG_OCP_HPA_VAL                  (0x38u) /*!< Current max 160mA for the whole device when using HPA */
#define STM32WLxx_SUBGHZ_REG_OCP_LPA_VAL                  (0x18u) /*!< Current max 80mA for the whole device when using LPA */

/* SMPS clock detection defines */
#define STM32WLxx_SUBGHZ_REG_VAL_SMPSC0R_CLKDE            (0x40u) /*!< SMPS control 0 register, bit SMPS clock detection enable */

/**
 * @brief The address of the register holding the CRC configuration extracted from a received LoRa header
 */
#define STM32WL55xx_SUBGHZ_REG_LR_HEADER_CRC              (0x76Bu)
#define STM32WL55xx_SUBGHZ_REG_LR_HEADER_CRC_POS          (4u)
#define STM32WL55xx_SUBGHZ_REG_LR_HEADER_CRC_MASK         (1u << STM32WL55xx_SUBGHZ_REG_LR_HEADER_CRC_POS)

#define SID_RADIO_POLYNOMIAL_CRC16                        (0x1021u)
#define SID_RADIO_POLYNOMIAL_CRC32                        (0x04C11DB7u)

/** Sidewalk Phy Radio State - copy from sid_pal_radio_ifc.h */
#define SID_PAL_RADIO_UNKNOWN                             (0)
#define SID_PAL_RADIO_STANDBY                             (1)
#define SID_PAL_RADIO_SLEEP                               (2)
#define SID_PAL_RADIO_RX                                  (3)
#define SID_PAL_RADIO_TX                                  (4)
#define SID_PAL_RADIO_CAD                                 (5)
#define SID_PAL_RADIO_STANDBY_XOSC                        (6)
#define SID_PAL_RADIO_RX_DC                               (7)
#define SID_PAL_RADIO_BUSY                                (8)

#define STM32WLxx_SUBGHZ_STDBY_STATE_DELAY_US             (10u) /*!< Delay time to allow for any external PA/FEM turn ON/OFF */

#define SID_RADIO_FSK_WHITENING_SEED                      (0x01FFu)

#define SID_RADIO_RXTX_NO_TIMEOUT_VAL                     (0x000000u)
#define SID_RADIO_RX_CONTINUOUS_VAL                       (0xFFFFFFu)
#define SID_RADIO_INFINITE_TIME                           (0xFFFFFFFFu)

#define SID_RADIO_FSK_SYNC_WORD_VALID_MARKER              ((uint32_t)(0xABBACAFEu))
#define SID_RADIO_FSK_PACKET_TYPE_OFFSET                  (0u)
#define SID_RADIO_FSK_PACKET_LENGTH_OFFSET                (1u)
#define SID_RADIO_FSK_SYNC_WORD_VALID_MARKER_OFFSET       (2u)

#define SID_RADIO_SUBGHZ_XTAL_FREQ                        (HSE_VALUE)
#define SID_RADIO_SUBGHZ_PLL_STEP_POW2_MULT               (14u)         /*!< Used as a multiplier to calculate fractional parts in integer domain: (x << SID_RADIO_SUBGHZ_PLL_STEP_POW2_MULT) = x * 2^SID_RADIO_SUBGHZ_PLL_STEP_POW2_MULT */
#define SID_RADIO_SUBGHZ_PLL_STEP_MULTIPLIED              ((uint32_t)(SID_RADIO_SUBGHZ_XTAL_FREQ) >> (25u - SID_RADIO_SUBGHZ_PLL_STEP_POW2_MULT)) /*!< Single PLL step is (XTAL_FREQ / 2^25), this value is a single step multiplied by 2^SID_RADIO_SUBGHZ_PLL_STEP_POW2_MULT */

#define SID_RADIO_SUBGHZ_PA_SEL_HPA                       (0u) /*!< PaSel register value when High Power PA option is targeted */
#define SID_RADIO_SUBGHZ_PA_SEL_LPA                       (1u) /*!< PaSel register value when Low Power PA option is targeted */

#define SID_RADIO_SUBGHZ_HPA_TX_PWR_UPPER_LIMIT           (22)  /*!< Maximum achievable Tx power when High Power Amplifier (HPA) is used */
#define SID_RADIO_SUBGHZ_HPA_TX_PWR_LOWER_LIMIT           (-9)  /*!< Minimum achievable Tx power when High Power Amplifier (HPA) is used */
#define SID_RADIO_SUBGHZ_LPA_TX_PWR_UPPER_LIMIT           (14)  /*!< Maximum achievable Tx power when Low Power Amplifier (LPA) is used */
#define SID_RADIO_SUBGHZ_LPA_TX_PWR_LOWER_LIMIT           (-17) /*!< Minimum achievable Tx power when Low Power Amplifier (LPA) is used */

#define SID_RADIO_SUBGHZ_DEEP_SLEEP_DISABLE               (0u)  /*!< Radio not in Sleep. Mimics SUBGHZ_DEEP_SLEEP_DISABLE definition from HAL driver */

#define SID_RADIO_LORA_NUM_USED_BW                        (3u)  /* BW125, BW250, BW500 */
#define SID_RADIO_LORA_NUM_USED_SF                        (8u)  /* SF12~SF5 */

#define SID_RADIO_TX_BUFFER_BASE_OFFSET                   (0u)  /* The Tx buffer start offset is fixed to 0x00 to simplify Tx buffer uploading */
#define SID_RADIO_RX_BUFFER_BASE_OFFSET                   (0u)  /* The Rx buffer start offset is fixed to 0x00 to simplify readout */

#define SID_RADIO_FSK_GET_RX_BUFFER_STATUS_RETRIES        (2u)  /*!< Maximum number of retries for GetRxBufferStatus() command whenever workaround is applied */
#define SID_RADIO_LORA_GET_RX_BUFFER_STATUS_RETRIES       (2u)  /*!< Maximum number of retries for GetRxBufferStatus() command whenever workaround is applied */

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
#  define SID_RADIO_UDT_DRIVER_READY_FLAG                 (1u << 0)
#  define SID_RADIO_UDT_TRANSFER_ENABLED_FLAG             (1u << 1)

#  define SID_RADIO_UDT_TASK_STACK_SIZE                   (768u)
#  define SID_RADIO_UDT_TASK_PRIO                         ((osPriority_t) osPriorityRealtime7)
#  define SID_RADIO_UDT_OUT_MSG_QUEUE_LEN                 (10u)
#  define SID_RADIO_UDT_SUSPEND_RESERVED_TIME_US          (7500u) /*!< Amount of time (in us) reserved before the planned radio wakeup event - this safety gap is used to suspend UDT before waking up the radio */
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/* Private macro -------------------------------------------------------------*/

#ifndef SID_RADIO_EXTRA_LOGGING
/* Set SID_RADIO_RADIO_EXTRA_LOGGING to 1 to enable extended logs */
#  define SID_RADIO_EXTRA_LOGGING                (0)
#endif /* SID_RADIO_EXTRA_LOGGING */

#if SID_RADIO_EXTRA_LOGGING
#  define SID_RADIO_LOG_ERROR(...)               SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define SID_RADIO_LOG_WARNING(...)             SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define SID_RADIO_LOG_INFO(...)                SID_PAL_LOG_INFO(__VA_ARGS__)
#  define SID_RADIO_LOG_DEBUG(...)               SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define SID_RADIO_LOG_TRACE(...)               SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define SID_RADIO_LOG_ERROR(...)               ((void)0u)
#  define SID_RADIO_LOG_WARNING(...)             ((void)0u)
#  define SID_RADIO_LOG_INFO(...)                ((void)0u)
#  define SID_RADIO_LOG_DEBUG(...)               ((void)0u)
#  define SID_RADIO_LOG_TRACE(...)               ((void)0u)
#endif

/**
 * @brief Convert frequency in Hz into the number of PLL steps
 */
#define SID_RADIO_FREQ_HZ_TO_PLL_STEP(hz, steps) do\
                                                 {\
                                                     register uint32_t steps_int;\
                                                     register uint32_t steps_frac;\
                                                     steps_int  = (hz) / SID_RADIO_SUBGHZ_PLL_STEP_MULTIPLIED;\
                                                     steps_frac = (hz) - (steps_int * SID_RADIO_SUBGHZ_PLL_STEP_MULTIPLIED);\
                                                     (steps) = (steps_int << SID_RADIO_SUBGHZ_PLL_STEP_POW2_MULT) +\
                                                     ( ( ( steps_frac << SID_RADIO_SUBGHZ_PLL_STEP_POW2_MULT ) + ( SID_RADIO_SUBGHZ_PLL_STEP_MULTIPLIED >> 2 ) ) / SID_RADIO_SUBGHZ_PLL_STEP_MULTIPLIED );\
                                                 } while(0)

/**
 * @brief Macro to reset the radio.
 *
 * This macro enables and disables the RF reset signal with a specified delay time.
 */
#define SID_RADIO_SUBGHZ_RESET_RADIO()           do\
                                                 {\
                                                     LL_RCC_RF_EnableReset();\
                                                     sid_pal_delay_us(SID_RADIO_SUBGHZ_RESET_TOGGLE_DELAY_US);\
                                                     LL_RCC_RF_DisableReset();\
                                                 } while(0)

/*!
 * @brief return the calibration band of the given frequency
 * @param [in] freq_hz Targeted frequency in Hz
 * @return calibration band for the frequency
 */
#define SID_RADIO_GET_FREQ_BAND(freq_hz)         ((freq_hz > 900000000u) ? STM32WLxx_SUBGHZ_BAND_900M :\
                                                  (freq_hz > 850000000u) ? STM32WLxx_SUBGHZ_BAND_850M :\
                                                  (freq_hz > 770000000u) ? STM32WLxx_SUBGHZ_BAND_770M :\
                                                  (freq_hz > 460000000u) ? STM32WLxx_SUBGHZ_BAND_460M :\
                                                  (freq_hz > 430000000u) ? STM32WLxx_SUBGHZ_BAND_430M :\
                                                  STM32WLxx_SUBGHZ_BAND_INVALID)

#define SID_RADIO_GET_FREQ_BAND_STR(_band_)      (STM32WLxx_SUBGHZ_BAND_900M == (_band_) ? "902-928 MHz" : \
                                                  STM32WLxx_SUBGHZ_BAND_850M == (_band_) ? "863-870 MHz" :\
                                                  STM32WLxx_SUBGHZ_BAND_770M == (_band_) ? "779-787 MHz" :\
                                                  STM32WLxx_SUBGHZ_BAND_460M == (_band_) ? "470-510 MHz" :\
                                                  STM32WLxx_SUBGHZ_BAND_430M == (_band_) ? "430-440 MHz" :\
                                                  "<invalid>")

/* External variables --------------------------------------------------------*/

extern SUBGHZ_HandleTypeDef hsubghz;

/* Private variables ---------------------------------------------------------*/

/** Global WL55 radio setup used to minimize data for other commands */
static stm32wlxx_radio_phy_cfg_t         WL55_cfg = {0};
static volatile uint32_t                 valid_subghz_cfg_received = FALSE;

static uint32_t                          subghz_cold_start = TRUE;

/** current standby/sleeping/tx/rx mode */
static uint32_t                          current_radio_state = SID_PAL_RADIO_UNKNOWN;

/** current modem mode */
static stm32wlxx_pal_radio_modem_mode_t  current_modem_mode = STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_UNDEFINED;

/** last frequency set in the radio phy */
static uint32_t                          current_radio_freq_hz = 0u;

/** Storage for IRQ reply follow-up data */
static stm32wlxx_rcp_set_subghz_rx_buf_t radio_irq_followup_frame;

static uint32_t                          radio_tx_buf_upload_offset = 0u;

static stm32wlxx_pal_radio_events_t      last_indicated_radio_event = STM32WLxx_PAL_RADIO_EVENT_UNKNOWN;
static uint32_t                          last_indicated_radio_event_data_len = 0u;
static uint32_t                          mod_params_upd_indication_pending = FALSE;

UTIL_TIMER_Object_t                      radio_timeout_mon;

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
static UTIL_TIMER_Object_t               lpm_suspend_timer;
#endif /* LOW_POWER_DISABLE */

/* User data transfer handling */
#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
static sid_radio_udt_ctx_t udt_ctx = {0u};

static const osThreadAttr_t wlxx_user_data_transfer_thread_attributes = {
    .name       = "WLxx_UDT",
    .priority   = SID_RADIO_UDT_TASK_PRIO,
    .stack_size = SID_RADIO_UDT_TASK_STACK_SIZE,
};

/** Storage for User Data IRQ follow-up data */
static stm32wlxx_rcp_user_data_irq_followup_t udt_irq_followup_frame;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/* Private constants ---------------------------------------------------------*/

/* LUT for LoRa symbol duration for various bandwidth and spreading factor combinations */
static const uint32_t radio_lora_symb_time[SID_RADIO_LORA_NUM_USED_BW][SID_RADIO_LORA_NUM_USED_SF] = {
                                                        { 32768u, 16384u, 8192u, 4096u, 2048u, 1024u, 512u, 256u }, /* 125 KHz, SF12~SF5 */
                                                        { 16384u,  8192u, 4096u, 2048u, 1024u,  512u, 256u, 128u }, /* 250 KHz, SF12~SF5 */
                                                        {  8192u,  4096u, 2048u, 1024u,  512u,  256u, 128u,  64u }, /* 500 KHz, SF12~SF5 */
                                                    };

/* Private function prototypes -----------------------------------------------*/

static inline sid_radio_error_t  _wait_for_subghz_reset_finish(const uint32_t timeout_ms);
static inline sid_radio_error_t  _wait_for_subghz_ready(const uint32_t timeout_ms);
static inline sid_radio_error_t  _subghz_reset(void);
static inline sid_radio_error_t  _sid_radio_indicate_request_processed(const uint16_t request_id, const uint16_t request_status, const uint8_t * const extended_data, const uint32_t extended_data_len);
static inline sid_radio_error_t  _sid_radio_subghz_smps_clk_detect_en(void);
static inline sid_radio_error_t  _sid_radio_subghz_get_gfsk_pkt_status(stm32wlxx_pkt_status_gfsk_t * const pkt_status);
static inline sid_radio_error_t  _sid_radio_subghz_get_lora_pkt_status(stm32wlxx_rcp_lora_rx_packet_info_t * const lora_rx_packet_status);
static inline sid_radio_error_t  _sid_radio_subghz_get_rx_buffer_status(stm32wlxx_rx_buffer_status_t * const rx_buffer_status);
static inline sid_radio_error_t  _sid_radio_subghz_set_rx(const uint32_t timeout_tus);
static inline sid_radio_error_t  _sid_radio_subghz_stop_tmr_on_pbl(const uint32_t enable);
static inline sid_radio_error_t  _sid_radio_subghz_set_tx(const uint32_t timeout);
static inline sid_radio_error_t  _sid_radio_subghz_set_lora_symb_nb_timeout(const uint8_t nb_of_symbs);
static inline sid_radio_error_t  _sid_radio_subghz_set_gfsk_sync_word(const uint8_t * const sync_word, const uint8_t sync_word_len);
static inline sid_radio_error_t  _sid_radio_subghz_set_lora_sync_word(const uint16_t sync_word);
static inline sid_radio_error_t  _sid_radio_subghz_clear_irq_status(const uint16_t irq_mask);
static inline sid_radio_error_t  _sid_radio_subghz_clear_irq_status_all(void);
static inline sid_radio_error_t  _sid_radio_subghz_set_sleep(const stm32wlxx_subghz_sleep_params_t cfg);
static inline sid_radio_error_t  _sid_radio_subghz_wakeup(const uint32_t timeout_ms);
static inline sid_radio_error_t  _sid_radio_subghz_set_standby(const uint8_t cfg);
static inline sid_radio_error_t  _sid_radio_smps_set(const uint8_t level);
static inline sid_radio_error_t  _sid_radio_set_reg_mode(const uint8_t mode);
static inline sid_radio_error_t  _sid_radio_set_buffer_base_addr(const uint8_t tx_base_addr, const uint8_t rx_base_addr);
static inline sid_radio_error_t  _sid_radio_cfg_rx_boosted(const uint32_t enabled);
static inline sid_radio_error_t  _sid_radio_set_tcxo_mode(const RadioTcxoCtrlVoltage_t tcxo_voltage, const uint32_t timeout);
static inline sid_radio_error_t  _sid_radio_clear_subghz_errors(void);
static inline sid_radio_error_t  _sid_radio_get_subghz_errors(uint16_t * const errors);
static inline sid_radio_error_t  _sid_radio_calibrate(const uint8_t param);
static inline sid_radio_error_t  _sid_radio_set_pa_cfg(const stm32wlxx_radio_pa_setup_t * const pa_cfg);
static inline sid_radio_error_t  _sid_radio_set_tx_params(const stm32wlxx_radio_pa_setup_t * const pa_cfg);
static inline sid_radio_error_t  _sid_radio_set_dio_irq_params(const uint16_t irq_mask, const uint16_t dio1_mask, const uint16_t dio2_mask, const uint16_t dio3_mask);
static inline sid_radio_error_t  _sid_radio_set_pkt_type(const uint8_t pkt_type);
static inline sid_radio_error_t  _sid_radio_subghz_set_gfsk_pkt_params(const stm32wlxx_pal_radio_fsk_pkt_params_t * const params);
static inline sid_radio_error_t  _sid_radio_cal_img(const stm32wlxx_freq_cal_band_t freq_band);
static inline sid_radio_error_t  _sid_radio_set_rf_freq(const uint32_t freq_hz);
static inline sid_radio_error_t  _sid_radio_tx_modulation_workaround(const stm32wlxx_pkt_type_t pkt_type, const stm32wlxx_lora_bw_t bw);
static inline sid_radio_error_t  _sid_radio_subghz_set_lora_mod_params(const stm32wlxx_radio_lora_phy_mod_params_t * const params);
static inline sid_radio_error_t  _sid_radio_subghz_set_lora_pkt_params(const stm32wlxx_radio_lora_phy_pkt_params_t * const params);
static inline sid_radio_error_t  _sid_radio_subghz_set_cad_params(const stm32wlxx_lora_cad_params_t * const params);
static inline sid_radio_error_t  _sid_radio_subghz_set_xtal_trim(const uint8_t xta, const uint8_t xtb);
static inline sid_radio_error_t  _sid_radio_subghz_set_gfsk_mod_params(const stm32wlxx_radio_fsk_phy_mod_params_t * const params);
static inline sid_radio_error_t  _sid_radio_set_irq_mask(uint16_t irq_mask);
static inline uint32_t           _sid_radio_get_fsk_crc_len_in_bytes(const stm32wlxx_gfsk_crc_types_t crc_type);
static inline uint32_t           _sid_radio_get_fsk_time_on_air_ms(const stm32wlxx_radio_fsk_phy_mod_params_t * const mod_params, const stm32wlxx_pal_radio_fsk_pkt_params_t * const  packet_params, uint32_t packet_len);
static inline sid_radio_error_t  _sid_radio_fsk_process_sync_word_detected(uint8_t * const rx_storage, const uint32_t rx_completed);
static inline sid_radio_error_t  _sid_radio_enable_default_irqs(void);
static inline sid_radio_error_t  _sid_radio_set_modem_to_fsk(void);
static inline sid_radio_error_t  _sid_radio_set_modem_to_lora(void);
static inline uint8_t            _sid_radio_lora_low_data_rate_optimize(const stm32wlxx_lora_sf_t sf, const stm32wlxx_lora_bw_t bw);
static inline void               _sid_radio_convert_to_phy_lora_mod_params(stm32wlxx_mod_params_lora_t * const sx_m, const stm32wlxx_radio_lora_phy_mod_params_t * const in_mod_params);
static inline RBI_Switch_TypeDef _sid_radio_rbi_get_pa_selector(const uint32_t request_hpa);
static inline sid_radio_error_t  _sid_radio_subghz_set_radio_mode(const stm32wlxx_subghz_fe_mode_t frontend_mode);
static inline sid_radio_error_t  _sid_radio_subghz_sleep(const uint32_t sleep_duration_us);
static inline sid_radio_error_t  _sid_radio_subghz_standby(void);
static        sid_radio_error_t  _sid_radio_set_tx_power(const stm32wlxx_radio_pa_setup_t * const new_pa_cfg);
static inline sid_radio_error_t  _sid_radio_set_modem_mode(const stm32wlxx_rcp_set_modem_mode_req_t * const request);
static inline sid_radio_error_t  _sid_radio_subghz_get_payload(uint8_t * const buffer, const uint32_t buf_size, uint32_t * const actual_size);
static inline sid_radio_error_t  _sid_radio_lora_process_rx_done(stm32wlxx_rcp_lora_rx_packet_info_t * const lora_rx_packet_status, uint8_t * const rx_storage,
                                                                 const uint32_t rx_storage_size, uint32_t * const out_received_len);
static inline void               _sid_radio_fsk_perform_data_whitening(const uint16_t seed, const uint8_t * const buffer_in, uint8_t * const buffer_out, const uint32_t length);
static inline uint16_t           _sid_radio_compute_crc16(const uint8_t * const buffer, const uint32_t length);
static inline uint32_t           _sid_radio_compute_crc32(const uint8_t * const buffer, const uint32_t length);
static inline sid_radio_error_t  _sid_radio_fsk_process_rx_done(stm32wlxx_rcp_fsk_rx_packet_info_t * const fsk_rx_packet_status, uint8_t * const rx_storage,
                                                                const uint32_t rx_storage_size, uint32_t * const out_received_len,
                                                                stm32wlxx_radio_fsk_rx_done_status_t * const out_fsk_rx_done_status);
static inline void               _sid_radio_convert_fsk_packet_params(stm32wlxx_pkt_params_gfsk_t * const fsk_pp, const stm32wlxx_pal_radio_fsk_pkt_params_t * const pkt_params);
static inline sid_radio_error_t  _sid_radio_set_frequency(const stm32wlxx_rcp_set_frequency_req_t * const request);
static inline sid_radio_error_t  _sid_radio_set_sync_word(const stm32wlxx_rcp_set_sync_word_req_t * const request);
static inline sid_radio_error_t  _sid_radio_set_lora_symbol_timeout(const stm32wlxx_rcp_set_lora_symb_timeout_req_t * const request);
static inline uint32_t           _sid_radio_get_lora_symbol_timeout_us(const uint32_t number_of_symbol);
static inline sid_radio_error_t  _sid_radio_process_set_lora_mod_params_req(const stm32wlxx_rcp_set_lora_mod_params_req_t * const request);
static inline sid_radio_error_t  _sid_radio_set_lora_packet_params(const stm32wlxx_rcp_set_lora_pkt_params_req_t * const request);
static inline sid_radio_error_t  _sid_radio_set_lora_cad_params(void);
static inline sid_radio_error_t  _sid_radio_start_tx(const uint32_t timeout);
static inline sid_radio_error_t  _sid_radio_start_rx(const stm32wlxx_rcp_start_rx_params_req_t * const params);
static inline int32_t            _sid_radio_get_adjusted_rssi(const int32_t raw_rrsi);
static inline sid_radio_error_t  _sid_radio_subghz_get_rssi(int8_t * const out_rssi);
static        sid_radio_error_t  _sid_radio_irq_process(const uint32_t reported_irqs, const uint32_t irq_timestamp_s, const uint32_t irq_timestamp_us);
static        sid_radio_error_t  _sid_radio_report_event(const stm32wlxx_pal_radio_events_t radio_event, stm32wlxx_radio_comm_spi_frame_t * const spi_reply_frame, const uint32_t irq_timestamp_s, const uint32_t irq_timestamp_us);
static inline sid_radio_error_t  _sid_radio_process_set_fsk_mod_params_req(const stm32wlxx_rcp_set_fsk_mod_params_req_t * const request);
static inline sid_radio_error_t  _sid_radio_process_set_fsk_packet_params_req(const stm32wlxx_rcp_set_fsk_pkt_params_req_t * const request);
static inline sid_radio_error_t  _sid_radio_start_fsk_carrier_sense(void);

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
static inline sid_radio_error_t  _sid_radio_process_user_data_upload(const uint8_t * const frame_data, const uint32_t available_frame_length, const uint32_t full_frame_length);
static        void               _user_data_transfer_thread(void * context);
static        void               _sid_radio_udt_suspend_timer_cb(void * context);
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

#if HALO_ENABLE_DIAGNOSTICS
static inline sid_radio_error_t  _sid_radio_subghz_set_cw_tx(void);
static inline sid_radio_error_t  _sid_radio_start_cw_tx(const stm32wlxx_rcp_start_cw_tx_params_req_t * const request);
static inline sid_radio_error_t  _sid_radio_start_continuous_rx(void);
#endif /* HALO_ENABLE_DIAGNOSTICS */

static inline sid_radio_error_t  _sid_radio_process_irq_ack(void);
static inline sid_radio_error_t  _sid_radio_process_sunghz_reset_req(const stm32wlxx_rcp_reset_subghz_req_t * const request);
static inline sid_radio_error_t  _sid_radio_process_subghz_sleep_req(const stm32wlxx_rcp_radio_sleep_req_t * const request);
static inline sid_radio_error_t  _sid_radio_process_subghz_standby_req(void);
static inline sid_radio_error_t  _sid_radio_process_apply_cfg_req(const uint8_t * const frame_data, const uint32_t frame_length);
static inline sid_radio_error_t  _sid_radio_process_sunghz_base_init_req(const stm32wlxx_rcp_apply_base_hw_cfg_req_t * const request);
static inline sid_radio_error_t  _sid_radio_process_set_tx_power(const stm32wlxx_rcp_set_tx_power_req_t * const request);
static inline sid_radio_error_t  _sid_radio_process_set_subghz_tx_buf_req(const uint8_t * const frame_data, const uint32_t available_frame_length, const uint32_t full_frame_length);
static inline sid_radio_error_t  _sid_radio_process_start_tx_req(void);
static inline void               _sid_radio_log_mod_params_change(void);
static        void               _sid_radio_on_radio_timeout_event(void * context);
static inline sid_radio_error_t  _sid_radio_start_radio_timeout_mon(const uint32_t timeout_us);
#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
static        void               _sid_radio_lpm_suspend_timer_cb(void * context);
#endif /* LOW_POWER_DISABLE */
#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
static inline sid_radio_error_t  _sid_radio_udt_init(void);
static inline sid_radio_error_t  _sid_radio_udt_setup_session(const uint32_t sleep_duration_us);
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/* Private function definitions ----------------------------------------------*/

/**
 * @brief Waits for the RF module to finish the reset process.
 *
 * This function waits for the RF module to finish the reset process by checking
 * the status of the RF under reset flag. It returns a status code indicating the
 * success or failure of the operation within a specified timeout.
 *
 * @param [in] timeout_ms The timeout value in milliseconds for waiting.
 * @return     Status code indicating success or an error condition.
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _wait_for_subghz_reset_finish(const uint32_t timeout_ms)
{
    register const uint32_t tick_start = HAL_GetTick();
    __COMPILER_BARRIER();

    register const uint32_t timeout_ticks = timeout_ms != osWaitForever ? (timeout_ms / (uint32_t)HAL_GetTickFreq()) : HAL_MAX_DELAY;
    register uint32_t count = timeout_ticks * ((SystemCoreClock * 26U) >> 20U); /* Calculate timeout based on a software loop to avoid blocking issue if SysTick is disabled */
    sid_radio_error_t err = SID_RADIO_ERROR_NONE;

    /* Wait for any ongoing transactions to complete */
    while (LL_RCC_IsRFUnderReset() != 0u)
    {
        if (timeout_ticks != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) >= timeout_ticks) || (0u == count))
            {
                err = SID_RADIO_ERROR_TIMEOUT;
                break;
            }

            /* Count iterations to protect from the SysTick being disabled or not incremented */
            if (count != 0u)
            {
                count--;
            }
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Waits for the RF module's busy pin to be clear
 *
 * This function waits for the RF module's busy pin to be clear after reset by checking the
 * status of the RF busy flags. It returns a status code indicating the success
 * or failure of the operation within a specified timeout.
 *
 * @param [in] timeout_ms The timeout value in milliseconds for waiting.
 * @return     Status code indicating success or an error condition.
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _wait_for_subghz_ready(const uint32_t timeout_ms)
{
    register const uint32_t tick_start = HAL_GetTick();
    __COMPILER_BARRIER();

    register const uint32_t timeout_ticks = timeout_ms != osWaitForever ? (timeout_ms / (uint32_t)HAL_GetTickFreq()) : HAL_MAX_DELAY;
    register uint32_t count = timeout_ticks * ((SystemCoreClock * 26U) >> 20U); /* Calculate timeout based on a software loop to avoid blocking issue if SysTick is disabled */
    sid_radio_error_t err = SID_RADIO_ERROR_NONE;

    /* Wait for any ongoing transactions to complete */
    while (LL_PWR_IsActiveFlag_RFBUSYS() != 0u)
    {
        if (timeout_ticks != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) >= timeout_ticks) || (0u == count))
            {
                err = SID_RADIO_ERROR_TIMEOUT;
                break;
            }

            /* Count iterations to protect from the SysTick being disabled or not incremented */
            if (count != 0u)
            {
                count--;
            }
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _subghz_reset(void)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        /* Toggle internal radio reset line and wait for the reset to complete */
        SID_RADIO_SUBGHZ_RESET_RADIO();
        err = _wait_for_subghz_reset_finish(SID_RADIO_SUBGHZ_WAIT_FOR_RESET_TIMEOUT_MS);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Wait for radio to leave the STARTUP state */
        err = _sid_radio_subghz_wakeup(SID_RADIO_SUBGHZ_WAIT_FOR_STARTUP_TIMEOUT_MS);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* At this point the radio will enter STDBY_RC mode */

        /* Indicate to the HAL driver that the radio is not in Sleep any longer */
        hsubghz.DeepSleep = SID_RADIO_SUBGHZ_DEEP_SLEEP_DISABLE;

        /* Done */
        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_indicate_request_processed(const uint16_t request_id, const uint16_t request_status, const uint8_t * const extended_data, const uint32_t extended_data_len)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    stm32wlxx_radio_comm_spi_frame_t indication_response = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_IRQ_STATUS,
        .payload = {
            .irq_status = {
                .irq_flags = STM32WLxx_APP_IRQ_SPI_REQ_DONE,
                .followup_payload_len = 0u,
                .req_cmpltd_details = {
                    .request_id = request_id,
                    .status = request_status,
                },
            },
        },
    };

    do
    {
        if ((extended_data != NULL) && (extended_data_len > 0u))
        {
            if (extended_data_len > STM32WLxx_RADIO_COMM_MTU_SIZE)
            {
                SID_PAL_LOG_ERROR("Failed to set Request Processed payload - supplied data exceeds MTU size. Req: 0x%x, stat: %u, ext_len: %u", request_id, request_status, extended_data_len);
                err = SID_RADIO_ERROR_EXCEEDS_MTU;
                break;
            }
            else
            {
                indication_response.payload.irq_status.followup_payload_len = extended_data_len;
            }
        }

        sid_host_comm_error_t hc_err = sid_host_comm_enqueue_tx((void *)&indication_response, sizeof(indication_response));
        if (hc_err != SID_HOST_COMM_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("Unable to enqueue Request Processed IRQ response. Host Comm error %u", (uint32_t)hc_err);
            err = SID_RADIO_ERROR_HARDWARE;
        }
        else
        {
            SID_RADIO_LOG_DEBUG("Successfully enqueued Request Processed IRQ response. Req: 0x%x, stat: %u", request_id, request_status);
            err = SID_RADIO_ERROR_NONE;
        }

        if ((extended_data != NULL) && (extended_data_len > 0u))
        {
            hc_err = sid_host_comm_enqueue_tx(extended_data, extended_data_len);
            if (hc_err != SID_HOST_COMM_ERROR_NONE)
            {
                SID_RADIO_LOG_ERROR("Unable to enqueue Request Processed IRQ follow-up data. Host Comm error %u", (uint32_t)hc_err);
                err = SID_RADIO_ERROR_HARDWARE;
            }
            else
            {
                SID_RADIO_LOG_DEBUG("Successfully enqueued Request Processed IRQ follow-up data");
                err = SID_RADIO_ERROR_NONE;
            }
        }
    } while (0);

    if (SID_RADIO_ERROR_NONE == err)
    {
        /* Set IRQ indication if everything is ok */
        SID_HOST_COMM_IRQ_DISABLE_TRIGGER(); /* Disable EXTI on IRQ falling edge since we are going to drive the line, otherwise GPIO Handshake will be triggered */
        __COMPILER_BARRIER();                /* Ensure EXTI trigger is deactivated before the next step */
        SID_HOST_COMM_IRQ_INDICATE_EVENT();  /* Drive the IRQ line to indicate pendingIRQ to the host MCU */
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Enables SMPS clock detection feature to avoid electrical damages if HSE clock is frozen for any reason
 *
 * @returns Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_smps_clk_detect_en(void)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t reg_value;
    uint8_t reg_value_readback;

    do
    {
        hal_err = HAL_SUBGHZ_ReadRegisters(&hsubghz, SUBGHZ_SMPSC0R, &reg_value, sizeof(reg_value));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_SMPSC0R, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* Enable SMPS clock detection feature in SUBGHZ peripheral */
        reg_value |= STM32WLxx_SUBGHZ_REG_VAL_SMPSC0R_CLKDE;

        hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_SMPSC0R, &reg_value, sizeof(reg_value));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_SMPSC0R, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* Read back the register to ensure configuration is applied */
        hal_err = HAL_SUBGHZ_ReadRegisters(&hsubghz, SUBGHZ_SMPSC0R, &reg_value_readback, sizeof(reg_value_readback));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_SMPSC0R, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        if (reg_value != reg_value_readback)
        {
            /* This this a crticial error - improper configuration may lead to the permanent IC damage */
            SID_PAL_LOG_ERROR("Failed to enable SMPS clock detection. Applied config: 0x%02X, readback config: 0x%02X", reg_value, reg_value_readback);
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Get the status of the last GFSK packet received
 *
 * @param [out] pkt_status Pointer to a structure to store the packet status
 *
 * @returns     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_get_gfsk_pkt_status(stm32wlxx_pkt_status_gfsk_t * const pkt_status)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           read_buf[3];

    hal_err = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_PACKETSTATUS, read_buf, sizeof(read_buf));

    if (HAL_OK == hal_err)
    {
        pkt_status->rx_status = read_buf[0];
        pkt_status->rssi_sync = ( int8_t )( -read_buf[1] >> 1 );
        pkt_status->rssi_avg  = ( int8_t )( -read_buf[2] >> 1 );

        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to access SubGHz. HAL_SUBGHZ err: 0x%x", hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Get the status of the last LoRa packet received
 *
 * @param [out] pkt_status Pointer to a structure to store the packet status
 * @returns     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_get_lora_pkt_status(stm32wlxx_rcp_lora_rx_packet_info_t * const lora_rx_packet_status)
{
    sid_radio_error_t                  err;
    HAL_StatusTypeDef                  hal_err;
    stm32wlxx_subghz_pkt_status_lora_t read_buf;

    /* Read raw data */
    hal_err = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_PACKETSTATUS, (uint8_t *)(void *)&read_buf, sizeof(read_buf));

    if (HAL_OK == hal_err)
    {
        /* Calculate actual values as per the datasheet */
        lora_rx_packet_status->rssi        = (int8_t)((-read_buf.rssi_pkt_in_dbm) >> 1);
        lora_rx_packet_status->snr         = (int8_t)((read_buf.snr_pkt_in_db + 2 ) >> 2);
        lora_rx_packet_status->signal_rssi = (int8_t)((-read_buf.signal_rssi_pkt_in_dbm) >> 1);

        /*
         * NOTE: By default STM32WLxx radio chip's rssi saturates at -106 dBm (or some cases -107 dBm)
         * After that the signal deterioration is reflected on the snr change.
         * to make the rssi carry more significance to the upper layer consumers, we need to figure out
         * a derived rssi value combining the rssi and snr in a meaningful way.
         *
         * Based on the empirical data, the formula is: <adjusted rssi = raw rssi + snr (when snr is negative)>.
         * This adjusted value is reported and used in all the ring products.
         */
        if(lora_rx_packet_status->snr < 0)
        {
            lora_rx_packet_status->rssi += lora_rx_packet_status->snr;
        }

        /* Apply adjustments */
        lora_rx_packet_status->rssi = (int8_t)_sid_radio_get_adjusted_rssi(lora_rx_packet_status->rssi);
        lora_rx_packet_status->signal_rssi  = (int8_t)_sid_radio_get_adjusted_rssi(lora_rx_packet_status->signal_rssi);

        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to access SubGHz. HAL_SUBGHZ err: 0x%x", hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Get the current Rx buffer status for both LoRa and GFSK Rx operations
 *
 * @details This function is used to get the length of the received payload and the start address to be used when
 * reading data from the Rx buffer.
 *
 * @param [out] rx_buffer_status Pointer to a structure to store the current status
 * @returns     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_get_rx_buffer_status(stm32wlxx_rx_buffer_status_t * const rx_buffer_status)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           status_buf[2];

    hal_err = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_RXBUFFERSTATUS, status_buf, sizeof(status_buf));

    if (HAL_OK == hal_err)
    {
        rx_buffer_status->pld_len_in_bytes     = status_buf[0];
        rx_buffer_status->buffer_start_pointer = status_buf[1];

        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to access SubGHz. HAL_SUBGHZ err: 0x%x", hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set the chip in reception mode
 *
 * @remark The packet type shall be configured with @ref _sid_radio_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as a packet is received
 * or if no packet has been received before the timeout. This behavior can be altered by @ref
 * _sid_radio_subghz_set_rx_tx_fallback_mode.
 *
 * @remark The timeout duration is obtained by:
 * \f$ timeout\_duration\_ms = timeout \times \frac{1}{64} \f$
 *
 * @remark Maximal timeout value is 0xFFFFFF (i.e. 511 seconds).
 *
 * @remark The timeout argument can have the following special values:
 *
 * | Special values | Meaning                                                                                         |
 * | -----------    | ------------------------------------------------------------------------------------------------|
 * | 0x000000       | RX single: the chip stays in RX mode until a packet is received, then switch to standby RC mode |
 * | 0xFFFFFF       | RX continuous: the chip stays in RX mode even after reception of a packet                       |
 *
 * @param [in] timeout The timeout configuration for Rx operation
 * @returns    Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_rx(const uint32_t timeout_tus)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[3];

    buf[0] = (uint8_t)(timeout_tus >> 16);
    buf[1] = (uint8_t)(timeout_tus >>  8);
    buf[2] = (uint8_t)(timeout_tus >>  0);

#if defined(DEBUG) && defined(NUCLEO_WL55_BOARD)
    /* Quick checks that antenna switch is configured properly - valid for the Nucleo board only */
    const GPIO_PinState ctrl3 = (RF_SW_CTRL3_GPIO_PORT->IDR & RF_SW_CTRL3_PIN) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;
    const GPIO_PinState ctrl1 = (RF_SW_CTRL1_GPIO_PORT->IDR & RF_SW_CTRL1_PIN) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;
    const GPIO_PinState ctrl2 = (RF_SW_CTRL2_GPIO_PORT->IDR & RF_SW_CTRL2_PIN) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if ((ctrl3 != GPIO_PIN_SET) || (ctrl1 != GPIO_PIN_SET) || (ctrl2 != GPIO_PIN_RESET))
    {
        SID_PAL_LOG_ERROR("Wrong antenna switch config for RX: ctrl1: %u, ctrl2: %u, ctrl3: %u", ctrl1, ctrl2, ctrl3);
    }
#endif /* DEBUG && NUCLEO_WL55_BOARD */

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RX, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_RX, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Configure the event on which the Rx timeout is stopped
 *
 * @remark The two options are:
 *   - Syncword / Header detection (default)
 *   - Preamble detection
 *
 * @param [in] enable If true, the timer stops on Syncword / Header detection.
 * @returns    Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_stop_tmr_on_pbl(const uint32_t enable)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf = FALSE == enable ? 0 : 1u;

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STOPRXTIMERONPREAMBLE, &buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_STOPRXTIMERONPREAMBLE, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set the chip in transmission mode
 *
 * @remark The packet type shall be configured with @ref _sid_radio_set_pkt_type before using this command.
 *
 * @remark By default, the chip returns automatically to standby RC mode as soon as the packet is sent or if the packet
 * has not been completely transmitted before the timeout. This behavior can be altered by @ref
 * _sid_radio_subghz_set_rx_tx_fallback_mode.
 *
 * @remark The timeout duration can be computed with the formula:
 * \f$ timeout\_duration\_ms = timeout \times * \frac{1}{64} \f$
 *
 * @remark Maximal value is 0xFFFFFF (i.e. 511 seconds)
 *
 * @remark If the timeout argument is 0, then no timeout is used.
 *
 * @param [in] timeout The timeout configuration for Tx operation
 * @returns    Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_tx(const uint32_t timeout)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[3] = {
                                   (uint8_t)(timeout >> 16),
                                   (uint8_t)(timeout >>  8),
                                   (uint8_t)(timeout >>  0),
                               };

#if defined(DEBUG) && defined(NUCLEO_WL55_BOARD)
    /* Quick checks that antenna switch is configured properly - valid for the Nucleo board only */
    const GPIO_PinState ctrl3 = (RF_SW_CTRL3_GPIO_PORT->IDR & RF_SW_CTRL3_PIN) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;
    const GPIO_PinState ctrl1 = (RF_SW_CTRL1_GPIO_PORT->IDR & RF_SW_CTRL1_PIN) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;
    const GPIO_PinState ctrl2 = (RF_SW_CTRL2_GPIO_PORT->IDR & RF_SW_CTRL2_PIN) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if ((ctrl3 != GPIO_PIN_SET) || (ctrl2 != GPIO_PIN_SET)) /* ctrl1 is don't care */
    {
        SID_PAL_LOG_ERROR("Wrong antenna switch config for TX: ctrl1: %u, ctrl2: %u, ctrl3: %u", ctrl1, ctrl2, ctrl3);
    }
#endif /* DEBUG && NUCLEO_WL55_BOARD */

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TX, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_TX, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Sets LoRa symbol timeout
 *
 * @param nb_of_symbs number of symbol intervals as timeout
 * @returns           Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_lora_symb_nb_timeout(const uint8_t nb_of_symbs)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;

    do
    {
        hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_LORASYMBTIMEOUT, (uint8_t *)&nb_of_symbs, sizeof(nb_of_symbs));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_LORASYMBTIMEOUT, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        if (nb_of_symbs >= 64u)
        {
            uint8_t mant = nb_of_symbs >> 1;
            uint8_t exp  = 0u;
            uint8_t reg  = 0u;

            while (mant > 31u)
            {
                mant >>= 2;
                exp++;
            }

            reg = exp + (mant << 3);

            hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, REG_LR_SYNCH_TIMEOUT, &reg, sizeof(reg));
            if (hal_err != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", REG_LR_SYNCH_TIMEOUT, hal_err);
                err = SID_RADIO_ERROR_HARDWARE;
                break;
            }
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Configure the sync word used in GFSK packet
 *
 * @param [in] sync_word Buffer holding the sync word to be configured
 * @param [in] sync_word_len Sync word length in byte
 *
 * @returns    Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_gfsk_sync_word(const uint8_t * const sync_word, const uint8_t sync_word_len)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[8] = { 0 };

    do
    {
        if (sync_word_len > sizeof(buf))
        {
            SID_PAL_LOG_ERROR("Cannot apply FSK sync word -it's too long (%u vs %u)", sync_word_len, sizeof(buf));
            err = SID_RADIO_ERROR_WRONG_SIZE;
            break;
        }

        /* Copy to a tem buffer since we have to put zero to the unused sync word bytes */
        SID_STM32_UTIL_fast_memcpy(buf, sync_word, sync_word_len);

        hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, REG_LR_SYNCWORDBASEADDRESS, buf, sizeof(buf));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", REG_LR_SYNCWORDBASEADDRESS, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Configure the sync word used in LoRa packet
 *
 * @remark In the case of a LoRaWAN use case, the two following values are specified:
 *   - 0x12 for a private LoRaWAN network (default)
 *   - 0x34 for a public LoRaWAN network
 *
 * @param [in] sync_word Sync word to be configured
 *
 * @returns    Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_lora_sync_word(const uint16_t sync_word)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buffer[2];

    buffer[0] = (uint8_t)(sync_word >> 8);
    buffer[1] = (uint8_t)(sync_word);

    hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, REG_LR_SYNCWORD, buffer, sizeof(buffer));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", REG_LR_SYNCWORD, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_clear_irq_status(const uint16_t irq_mask)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[2];

    buf[0] = (uint8_t)(irq_mask >> 8);
    buf[1] = (uint8_t)(irq_mask);

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CLR_IRQSTATUS, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_CLR_IRQSTATUS, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_clear_irq_status_all(void)
{
    sid_radio_error_t err;

    err = _sid_radio_subghz_clear_irq_status(STM32WLxx_SUBGHZ_IRQ_ALL);
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_sleep(const stm32wlxx_subghz_sleep_params_t cfg)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_SLEEP, (uint8_t *)&cfg.raw, sizeof(cfg.raw));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_SLEEP, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Wakes up SubGHz radio from Sleep state
 *
 * @return Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_wakeup(const uint32_t timeout_ms)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    /* Drive NSS pin to wakeup the radio */
    LL_PWR_SelectSUBGHZSPI_NSS();

    /* Wait for the wakeup to be indicated by Radio Busy signal */
    err = _wait_for_subghz_ready(timeout_ms);

    /* Release NSS after wakeup is completed (regardless if it was successful or not) */
    LL_PWR_UnselectSUBGHZSPI_NSS();

    if (SID_RADIO_ERROR_NONE == err)
    {
        /* Indicate to the HAL driver that the radio is not in Sleep any longer */
        hsubghz.DeepSleep = SID_RADIO_SUBGHZ_DEEP_SLEEP_DISABLE;

        /* The SubGHz radio is in the STDBY_RC state from this point */
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set the chip in stand-by mode
 *
 * @param [in] cfg Stand-by mode configuration
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_standby(const uint8_t cfg)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_STANDBY, (uint8_t *)&cfg, sizeof(cfg));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_STANDBY, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/** Analog of function from WL55_radio.c
 * @param [in] level of SMPS
 * @return     Non-zero value in case of error
 * */

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_smps_set(const uint8_t level)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           reg_value = 0;

    do
    {
        hal_err = HAL_SUBGHZ_ReadRegisters(&hsubghz, SUBGHZ_SMPSC2R, &reg_value, sizeof(reg_value));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_SMPSC2R, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        reg_value &= (~SMPS_DRV_MASK);
        reg_value |= level;

        hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_SMPSC2R, &reg_value, sizeof(reg_value));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_SMPSC2R, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Configure the regulator mode to be used
 *
 * @remark This function shall be called to set the correct regulator mode, depending on the usage of LDO or DC/DC on
 * the PCB implementation.
 *
 * @param [in] mode Regulator mode configuration.
 *
 * @return     Non-zero value in case of error
 * */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_reg_mode(const uint8_t mode)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_REGULATORMODE, (uint8_t *)&mode, sizeof(mode));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_REGULATORMODE, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set buffer start addresses for both Tx and Rx operations
 *
 * @param [in] tx_base_addr The start address used for Tx operations
 * @param [in] rx_base_addr The start address used for Rx operations
 *
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_buffer_base_addr(const uint8_t tx_base_addr, const uint8_t rx_base_addr)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[2] = {tx_base_addr, rx_base_addr};

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_BUFFERBASEADDRESS, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_BUFFERBASEADDRESS, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Configure the boost mode in reception
 *
 * @remark This configuration is not kept in the retention memory. Rx boosted mode shall be enabled each time the chip
 * leaves sleep mode.
 *
 * @param [in] context Chip implementation context.
 * @param [in] state Boost mode activation
 *
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_cfg_rx_boosted(const uint32_t enabled)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           gain_val = enabled != FALSE ? 0x96u : 0x94u;

    hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, REG_RX_GAIN, &gain_val, sizeof(gain_val));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", REG_RX_GAIN, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Configure the embedded TCXO switch control
 *
 * @remark This function shall only be called in standby RC mode.
 *
 * @remark The chip will wait for the timeout to happen before starting any operation that requires the TCXO.
 *
 * @param [in] tcxo_voltage Voltage used to power the TCXO.
 * @param [in] timeout Time needed for the TCXO to be stable.
 *
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_tcxo_mode(const RadioTcxoCtrlVoltage_t tcxo_voltage, const uint32_t timeout)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[4] = {
                                   (uint8_t)tcxo_voltage & 0x07u,
                                   (uint8_t)(timeout >> 16),
                                   (uint8_t)(timeout >> 8),
                                   (uint8_t)(timeout >> 0),
                               };

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TCXOMODE, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_TCXOMODE, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Clear all active errors
 *
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_clear_subghz_errors(void)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[2] = { 0 };

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CLR_ERROR, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_CLR_ERROR, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Get the list of all active errors
 *
 * @param [out] errors Pointer to a variable to store the error list
 *
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_get_subghz_errors(uint16_t * const errors)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           errors_local[2];

    hal_err = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_ERROR , errors_local, sizeof(errors_local));

    if (HAL_OK == hal_err)
    {
        *errors = ( ( uint16_t ) errors_local[0] << 8 ) + ( ( uint16_t ) errors_local[1] << 0 );

        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to access SubGHz. HAL_SUBGHZ err: 0x%x", hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Perform the calibration of the requested blocks
 *
 * @remark This function shall only be called in stand-by RC mode
 *
 * @remark The chip will return to stand-by RC mode on exit. Potential calibration issues can be read out with @ref
 * _sid_radio_get_subghz_errors command.
 *
 * @param [in] param Mask holding the blocks to be calibrated
 *
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_calibrate(const uint8_t param)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CALIBRATE, (uint8_t *)&param, sizeof(param));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_CALIBRATE, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Configure the PA (Power Amplifier)
 *
 * @details This command is used to differentiate the SX1261 from the SX1262 / SX1268. When using this command, the user
 * selects the PA to be used by the device as well as its configuration.
 *
 *
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_pa_cfg(const stm32wlxx_radio_pa_setup_t * const pa_cfg)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;

    do
    {
        /* Check if the requested PA selection is compatibl with current board */
        const RBI_Switch_TypeDef rf_sw_cfg = _sid_radio_rbi_get_pa_selector(SID_RADIO_SUBGHZ_PA_SEL_HPA == pa_cfg->device_sel ? TRUE : FALSE);
        if (RBI_SWITCH_OFF == rf_sw_cfg)
        {
            /* Invalid PA type selection. Error logs are provided by the _sid_radio_rbi_get_pa_selector() */
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
            break;
        }

        uint8_t buf[4] = {
                             pa_cfg->pa_duty_cycle,
                             pa_cfg->hp_max,
                             pa_cfg->device_sel,
                             pa_cfg->pa_lut,
                         };

        hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACONFIG, buf, sizeof(buf));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_PACONFIG, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0u);

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set the parameters for TX power and power amplifier ramp time
 *
 * @param [in] context Chip implementation context.
 * @param [in] pwr_in_dbm The Tx output power in dBm
 * @param [in] ramp_time The ramping time configuration for the PA
 *
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_tx_params(const stm32wlxx_radio_pa_setup_t * const pa_cfg)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;

    /* Ensure the requested Tx power is aligned with the selected power amplifier */
    int32_t tx_pwr_reg = pa_cfg->tx_power_reg;
    if (SID_RADIO_SUBGHZ_PA_SEL_HPA == pa_cfg->device_sel)
    {
        /* HPA will be used */
        if (tx_pwr_reg > SID_RADIO_SUBGHZ_HPA_TX_PWR_UPPER_LIMIT)
        {
            /* Saturate to the highest possible value */
            tx_pwr_reg = SID_RADIO_SUBGHZ_HPA_TX_PWR_UPPER_LIMIT;
        }
        else if (tx_pwr_reg < SID_RADIO_SUBGHZ_HPA_TX_PWR_LOWER_LIMIT)
        {
            /* Saturate to the lowest possible value */
            tx_pwr_reg = SID_RADIO_SUBGHZ_HPA_TX_PWR_LOWER_LIMIT;
        }
        else
        {
            /* Within the range - keep as is */
        }
    }
    else
    {
        /* LPA will be used */
        if (tx_pwr_reg > SID_RADIO_SUBGHZ_LPA_TX_PWR_UPPER_LIMIT)
        {
            /* Saturate to the highest possible value */
            tx_pwr_reg = SID_RADIO_SUBGHZ_LPA_TX_PWR_UPPER_LIMIT;
        }
        else if (tx_pwr_reg < SID_RADIO_SUBGHZ_LPA_TX_PWR_LOWER_LIMIT)
        {
            /* Saturate to the lowest possible value */
            tx_pwr_reg = SID_RADIO_SUBGHZ_LPA_TX_PWR_LOWER_LIMIT;
        }
        else
        {
            /* Within the range - keep as is */
        }
    }

    if (tx_pwr_reg != pa_cfg->tx_power_reg)
    {
        SID_PAL_LOG_WARNING("Tx power saturated from %s%ddBm to %s%ddBm due to HW limitations",
                            pa_cfg->target_tx_power > 0 ? "+" : "", pa_cfg->target_tx_power,
                            pa_cfg->target_tx_power - (pa_cfg->tx_power_reg - tx_pwr_reg)  > 0 ? "+" : "", pa_cfg->target_tx_power - (pa_cfg->tx_power_reg - tx_pwr_reg));
    }

    /* Apply settings */
    uint8_t           buf[2] = {
                                   (uint8_t)tx_pwr_reg,
                                   (uint8_t)pa_cfg->ramp_time,
                               };

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXPARAMS, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_TXPARAMS, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set which interrupt signals are redirected to the dedicated DIO pin
 *
 * @remark By default, no interrupt signal is redirected.
 *
 * @remark An interrupt will not occur until it is enabled system-wide, even if it is redirected to a specific DIO.
 *
 * @remark The DIO pin will remain asserted until all redirected interrupt signals are cleared with a call to @ref
 * _sid_radio_subghz_clear_irq_status.
 *
 * @remark DIO2 and DIO3 are shared with other features.
 *
 * @param [in] context Chip implementation context.
 * @param [in] irq_mask Variable that holds the system interrupt mask
 * @param [in] dio1_mask Variable that holds the interrupt mask for dio1
 * @param [in] dio2_mask Variable that holds the interrupt mask for dio2
 * @param [in] dio3_mask Variable that holds the interrupt mask for dio3
 *
 * @return     Non-zero value in case of error
 *
 * @see _sid_radio_subghz_clear_irq_status, wl55_set_dio2_as_rf_sw_ctrl, _sid_radio_set_tcxo_mode
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_dio_irq_params(const uint16_t irq_mask, const uint16_t dio1_mask, const uint16_t dio2_mask, const uint16_t dio3_mask)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[8];

    /* Global IRQ mask */
    buf[0] = (uint8_t)( irq_mask >> 8 );
    buf[1] = (uint8_t)( irq_mask >> 0 );

    /* IRQ indication assignments for DIO1 pin */
    buf[2] = (uint8_t)( dio1_mask >> 8 );
    buf[3] = (uint8_t)( dio1_mask >> 0 );

    /* IRQ indication assignments for DIO2 pin */
    buf[4] = (uint8_t)( dio2_mask >> 8 );
    buf[5] = (uint8_t)( dio2_mask >> 0 );

    /* IRQ indication assignments for DIO3 pin */
    buf[6] = (uint8_t)( dio3_mask >> 8 );
    buf[7] = (uint8_t)( dio3_mask >> 0 );

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CFG_DIOIRQ, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_CFG_DIOIRQ, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set the packet type
 *
 * @param [in] pkt_type Packet type to set
 * @returns true in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_pkt_type(const uint8_t pkt_type)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETTYPE, (uint8_t *)&pkt_type, sizeof(pkt_type));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_PACKETTYPE, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set the packet parameters for GFSK packets
 *
 * @remark The command @ref _sid_radio_set_pkt_type must be called prior to this one.
 *
 * @param [in] params The structure of GFSK packet configuration
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_gfsk_pkt_params(const stm32wlxx_pal_radio_fsk_pkt_params_t * const params)
{
    sid_radio_error_t           err;
    HAL_StatusTypeDef           hal_err;
    uint8_t                     buf[9];
    stm32wlxx_pkt_params_gfsk_t fsk_pp;

    _sid_radio_convert_fsk_packet_params(&fsk_pp, params);

    buf[0] = (uint8_t)(fsk_pp.pbl_len_in_bits >> 8);
    buf[1] = (uint8_t)(fsk_pp.pbl_len_in_bits >> 0);
    buf[2] = (uint8_t)(fsk_pp.pbl_min_det);
    buf[3] = fsk_pp.sync_word_len_in_bits;
    buf[4] = (uint8_t)(fsk_pp.addr_cmp);
    buf[5] = (uint8_t)(fsk_pp.hdr_type);
    buf[6] = fsk_pp.pld_len_in_bytes;
    buf[7] = (uint8_t)(fsk_pp.crc_type);
    buf[8] = (uint8_t)(fsk_pp.dc_free);

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETPARAMS, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_PACKETPARAMS, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Perform device operating frequency band image rejection calibration
 *
 * @param [in] freq_in_hz Frequency in Hz used for the image calibration
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_cal_img(const stm32wlxx_freq_cal_band_t freq_band)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[2];

    switch (freq_band)
    {
        case STM32WLxx_SUBGHZ_BAND_900M:
            buf[0] = 0xE1u;
            buf[1] = 0xE9u;
            break;

        case STM32WLxx_SUBGHZ_BAND_850M:
            buf[0] = 0xD7u;
            buf[1] = 0xDBu;
            break;

        case STM32WLxx_SUBGHZ_BAND_770M:
            buf[0] = 0xC1u;
            buf[1] = 0xC5u;
            break;

        case STM32WLxx_SUBGHZ_BAND_460M:
            buf[0] = 0x75u;
            buf[1] = 0x81u;
            break;

        case STM32WLxx_SUBGHZ_BAND_430M:
            buf[0] = 0x6Bu;
            buf[1] = 0x6Fu;
            break;

        case STM32WLxx_SUBGHZ_BAND_INVALID:
        default:
            SID_PAL_LOG_ERROR("Invalid frequeny band specified: %u", (uint32_t)freq_band);
            return SID_RADIO_ERROR_INVALID_ARGS;
            break;
    }

    SID_PAL_LOG_DEBUG("Running SubGHz calibration");

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CALIBRATEIMAGE, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_CALIBRATEIMAGE, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set the RF frequency for future radio operations.
 *
 * @remark This commands shall be called only after a packet type is selected.
 *
 * @param [in] freq_hz The frequency in Hz to set for radio operations
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_rf_freq(const uint32_t freq_hz)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[sizeof(uint32_t)];
    uint32_t          freq_pll_step;

    SID_RADIO_FREQ_HZ_TO_PLL_STEP(freq_hz, freq_pll_step);

    buf[0] = (uint8_t)(freq_pll_step >> 24);
    buf[1] = (uint8_t)(freq_pll_step >> 16);
    buf[2] = (uint8_t)(freq_pll_step >>  8);
    buf[3] = (uint8_t)(freq_pll_step >>  0);

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_RFFREQUENCY, (uint8_t *)&buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_RFFREQUENCY, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief 15.1.2 Workaround
 *
 * @remark Before any packet transmission, bit #2 of SUBGHZ_SDCFG0R shall be set to:
 * 0 if the LoRa BW = 500 kHz
 * 1 for any other LoRa BW
 * 1 for any (G)FSK configuration
 *
 * @param [in] pkt_type The modulation type (G)FSK/LoRa
 * @param [in] bw In case of LoRa modulation the bandwith must be specified
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_tx_modulation_workaround(const stm32wlxx_pkt_type_t pkt_type, const stm32wlxx_lora_bw_t bw)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           reg_value;

    do
    {
        hal_err = HAL_SUBGHZ_ReadRegisters(&hsubghz, SUBGHZ_SDCFG0R, &reg_value, sizeof(reg_value));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_SDCFG0R, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        if (pkt_type == STM32WLxx_PKT_TYPE_LORA)
        {
            if (bw == STM32WLxx_LORA_BW_500)
            {
                reg_value &= ~(1u << 2); /* Bit 2 set to 0 if the LoRa BW = 500 kHz */
            }
            else
            {
                reg_value |= (1u << 2); /* Bit 2 set to 1 for any other LoRa BW */
            }
        }
        else
        {
            (void)bw;
            reg_value |= (1u << 2); /* Bit 2 set to 1 for any (G)FSK configuration */
        }

        hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_SDCFG0R, &reg_value, sizeof(reg_value));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_SDCFG0R, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set the modulation parameters for LoRa packets
 *
 * @remark The command @ref _sid_radio_set_pkt_type must be called prior to this one.
 *
 * @param [in] params The structure of LoRa modulation configuration
 * @returns    Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_lora_mod_params(const stm32wlxx_radio_lora_phy_mod_params_t * const params)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        HAL_StatusTypeDef           hal_err;
        uint8_t                     buf[4];
        stm32wlxx_mod_params_lora_t lora_phy_mod_params = {0};

        _sid_radio_convert_to_phy_lora_mod_params(&lora_phy_mod_params, params);

        if (lora_phy_mod_params.cr > STM32WLxx_LORA_CR_4_8)
        {
            SID_PAL_LOG_ERROR("_sid_radio_subghz_set_lora_mod_params: coding rate selectors higher than 0x4 are not supported, requested CR is 0x%x", (uint32_t)lora_phy_mod_params.cr);
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
            break;
        }

        buf[0] = (uint8_t)(lora_phy_mod_params.sf);
        buf[1] = (uint8_t)(lora_phy_mod_params.bw);
        buf[2] = (uint8_t)(lora_phy_mod_params.cr);
        buf[3] = (uint8_t)(lora_phy_mod_params.ldro);

        SID_RADIO_LOG_DEBUG("LoRa mod: sf:0x%02x bw: %u, cr: %u, ldro: %u", lora_phy_mod_params.sf, lora_phy_mod_params.bw, lora_phy_mod_params.cr, lora_phy_mod_params.ldro);

        hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_MODULATIONPARAMS, (uint8_t *)&buf, sizeof(buf));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_MODULATIONPARAMS, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see datasheet DS_SX1261-2_V1.2 §15.1 */
        err = _sid_radio_tx_modulation_workaround(STM32WLxx_PKT_TYPE_LORA, lora_phy_mod_params.bw);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }
        /* WORKAROUND END */

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set the packet parameters for LoRa packets
 *
 * @remark The command @ref _sid_radio_set_pkt_type must be called prior to this one.
 *
 * @param [in] params The structure of LoRa packet configuration
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_lora_pkt_params(const stm32wlxx_radio_lora_phy_pkt_params_t * const params)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        HAL_StatusTypeDef hal_err;
        uint8_t           buf[6] = { 0 };

        buf[0] = (uint8_t)(params->preamble_length >> 8);
        buf[1] = (uint8_t)(params->preamble_length >> 0);
        buf[2] = (uint8_t)(params->header_type);
        buf[3] = params->payload_length;
        buf[4] = (uint8_t)(params->crc_mode ? 1u : 0u);
        buf[5] = (uint8_t)(params->invert_IQ ? 1u : 0u);

        hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_PACKETPARAMS, (uint8_t *)&buf, sizeof(buf));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_PACKETPARAMS, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* WORKAROUND - Optimizing the Inverted IQ Operation, see datasheet DS_SX1261-2_V1.2 §15.4 */
        {
            uint8_t reg_value;

            hal_err = HAL_SUBGHZ_ReadRegisters(&hsubghz, SUBGHZ_LIQPOLR, &reg_value, sizeof(reg_value));
            if (hal_err != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Failed to read SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_LIQPOLR, hal_err);
                err = SID_RADIO_ERROR_HARDWARE;
                break;
            }

            if (params->invert_IQ != 0u)
            {
                reg_value &= ~(1u << 2); /* Bit 2 set to 0 when using inverted IQ polarity */
            }
            else
            {
                reg_value |= (1u << 2);  /* Bit 2 set to 1 when using standard IQ polarity */
            }

            hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_LIQPOLR, &reg_value, sizeof(reg_value));
            if (hal_err != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_LIQPOLR, hal_err);
                err = SID_RADIO_ERROR_HARDWARE;
                break;
            }
        }
        /* WORKAROUND END */

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set the parameters for CAD operation
 *
 * @remark The command @ref _sid_radio_set_pkt_type must be called prior to this one.
 *
 * @param [in] params The structure of CAD configuration
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_cad_params(const stm32wlxx_lora_cad_params_t * const params)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[7];

    buf[0] = (uint8_t)params->cad_symb_nb;
    buf[1] = params->cad_det_peak;
    buf[2] = params->cad_det_min;
    buf[3] = (uint8_t)params->cad_exit_mode;
    buf[4] = (uint8_t)(params->cad_timeout >> 16);
    buf[5] = (uint8_t)(params->cad_timeout >>  8);
    buf[6] = (uint8_t)(params->cad_timeout >>  0);

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_CADPARAMS, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_CADPARAMS, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**  Sets trim values of oscillator
 * @param xta Trim value XTA
 * @param xtb Trim value XTB
 * @return    Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_xtal_trim(const uint8_t xta, const uint8_t xtb)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           buf[2] = {xta, xtb};

    hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, REG_XTA_TRIM, buf, sizeof(buf));
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", REG_XTA_TRIM, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set the modulation parameters for GFSK packets
 *
 * @remark The command @ref _sid_radio_set_pkt_type must be called prior to this one.
 *
 * @param [in] params The structure of GFSK modulation configuration
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_gfsk_mod_params(const stm32wlxx_radio_fsk_phy_mod_params_t * const params)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    HAL_StatusTypeDef hal_err;

    do
    {
        /* check for division by zero */
        if (0u == params->bit_rate)
        {
            err = SID_RADIO_ERROR_INVALID_ARGS;
            break;
        }

        uint8_t  buf[8];
        uint32_t bitrate = (uint32_t)((SID_RADIO_SUBGHZ_XTAL_FREQ << 5) / params->bit_rate);
        uint32_t fdev;

        SID_RADIO_FREQ_HZ_TO_PLL_STEP(params->freq_dev, fdev);

        buf[0] = (uint8_t)(bitrate >> 16);
        buf[1] = (uint8_t)(bitrate >>  8);
        buf[2] = (uint8_t)(bitrate >>  0);

        buf[3] = (uint8_t)(params->shaping);

        buf[4] = params->bandwidth;

        buf[5] = (uint8_t)(fdev >> 16);
        buf[6] = (uint8_t)(fdev >>  8);
        buf[7] = (uint8_t)(fdev >>  0);

        hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_MODULATIONPARAMS, buf, sizeof(buf));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_MODULATIONPARAMS, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* WORKAROUND - Modulation Quality with 500 kHz LoRa Bandwidth, see DS_SX1261-2_V1.2 datasheet chapter 15.1 */
        err = _sid_radio_tx_modulation_workaround(STM32WLxx_PKT_TYPE_GFSK, 0u);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }
        /* WORKAROUND END */

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_irq_mask(uint16_t irq_mask)
{
    return _sid_radio_set_dio_irq_params(irq_mask, irq_mask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _sid_radio_get_fsk_crc_len_in_bytes(const stm32wlxx_gfsk_crc_types_t crc_type)
{
    uint32_t crc_len_bytes;

    switch (crc_type)
    {
        case STM32WLxx_GFSK_CRC_1_BYTE:
        case STM32WLxx_GFSK_CRC_1_BYTE_INV:
            crc_len_bytes = 1u;
            break;

        case STM32WLxx_GFSK_CRC_2_BYTES:
        case STM32WLxx_GFSK_CRC_2_BYTES_INV:
            crc_len_bytes = 2u;
            break;

        case STM32WLxx_GFSK_CRC_OFF:
        default:
            crc_len_bytes = 0u;
            break;
    }

    return crc_len_bytes;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _sid_radio_get_fsk_time_on_air_ms(const stm32wlxx_radio_fsk_phy_mod_params_t * const mod_params, const stm32wlxx_pal_radio_fsk_pkt_params_t * const  packet_params, uint32_t packet_len)
{
    uint32_t time_on_air_ms;
    uint32_t numerator = (packet_params->preamble_length + (STM32WLxx_GFSK_PKT_VAR_LEN == packet_params->header_type ? 8u : 0u) + packet_params->sync_word_length
                          + ((packet_params->payload_length + (STM32WLxx_GFSK_ADDR_CMP_FILT_OFF == packet_params->addr_comp ? 0u : 1u) + _sid_radio_get_fsk_crc_len_in_bytes(packet_params->crc_type)) << 3)
                         ) * 1000u;
    uint32_t denominator = mod_params->bit_rate;

    time_on_air_ms = (numerator + denominator - 1u) / denominator;

    return time_on_air_ms;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_fsk_process_sync_word_detected(uint8_t * const rx_storage, const uint32_t rx_completed)
{
    sid_radio_error_t            err = SID_RADIO_ERROR_GENERIC;
    HAL_StatusTypeDef            hal_err;
    stm32wlxx_rx_buffer_status_t rx_buffer_status;

    /* Block any interrupts since the reaction time here is critical */
    UTILS_ENTER_CRITICAL_SECTION();
    do
    {
        /* Wait for FSK Header reception */
        uint8_t rx_available_len;

        /* FSK modem has no 802.15.4 packet engine - wait till the 2-byte header is received and parse it here to reconfigure the radio */
        if (FALSE == rx_completed)
        {
            /* Rx is still ongoing, let's modify Rx length on the fly */
            uint32_t start_s, start_us;
            uint32_t elapsed_time_us;
            const uint32_t timeout_us = (1000u * _sid_radio_get_fsk_time_on_air_ms(&WL55_cfg.fsk_cfg.mod, &WL55_cfg.fsk_cfg.pkt, STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH)) * 2u;

            start_s = TIMER_IF_GetTimeUs(&start_us);

            do
            {
                /* Readout current Rx position from the radio */
                hal_err = HAL_SUBGHZ_ReadRegisters(&hsubghz, SUBGHZ_RXADRPTR, &rx_available_len, sizeof(rx_available_len));

                /* Compute the elapsed time */
                uint32_t now_s, now_us;
                now_s = TIMER_IF_GetTimeUs(&now_us);
                elapsed_time_us = ((now_s - start_s /* Intentional underflow */) * 1000000u) + (now_us - start_us /* Intentional underflow */);
            } while ((HAL_OK == hal_err) && (rx_available_len < STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH) && (elapsed_time_us <= timeout_us));

            if (hal_err != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Failed to read SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_RXADRPTR, hal_err);
                err = SID_RADIO_ERROR_HARDWARE;
                break;
            }
            else if (elapsed_time_us > timeout_us)
            {
                /* Failed to receive header in time, terminating */
                err = SID_RADIO_ERROR_TIMEOUT;
                break;
            }
            else
            {
                /* All good, proceed with processing */
            }
        }

        /* 802.15.4 header received - read it out */
#if HALO_ENABLE_DIAGNOSTICS
        /* Continuous Rx is enabled in diagnostics mode, meaning buffer start pointer may move - we have to read it out */
        err = _sid_radio_subghz_get_rx_buffer_status(&rx_buffer_status);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }
#  else
        /* In the normal mode only the single Rx operations are allowed, meaning the Rx buffer start pointer is fixed and ther's no need to waste time on reading it out */
        rx_buffer_status.buffer_start_pointer = SID_RADIO_RX_BUFFER_BASE_OFFSET;
#  endif /* HALO_ENABLE_DIAGNOSTICS */

        /* Readout FSK packet header from the SubGHz buffer */
        hal_err = HAL_SUBGHZ_ReadBuffer(&hsubghz, rx_buffer_status.buffer_start_pointer, rx_storage, STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH);
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read %u bytes from SubGHz buffer. HAL_SUBGHZ err: 0x%x", STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* Calculate expected payload len from header */
        uint8_t expected_rx_len = (uint8_t)(rx_storage[SID_RADIO_FSK_PACKET_LENGTH_OFFSET] + STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH);

        /* Update the payload len to be received */
        if (FALSE == rx_completed)
        {
            hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_GRTXPLDLEN, &expected_rx_len, sizeof(expected_rx_len));
            if (hal_err != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_GRTXPLDLEN, hal_err);
                err = SID_RADIO_ERROR_HARDWARE;
                break;
            }
        }

        /* Ensure the Rx packet size is updated ASAP */
        __COMPILER_BARRIER();

        /* Now it is safe to proceed with local arrangements, FSK modem is reconfigured */

        /* Set validity markers */
        uint32_t * const validity_marker_ptr = (uint32_t *)(void *)&rx_storage[SID_RADIO_FSK_SYNC_WORD_VALID_MARKER_OFFSET];
        *validity_marker_ptr                 = SID_RADIO_FSK_SYNC_WORD_VALID_MARKER;

        err = SID_RADIO_ERROR_NONE;
    } while(0);
    UTILS_EXIT_CRITICAL_SECTION();

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_enable_default_irqs(void)
{
    sid_radio_error_t err;

    err = _sid_radio_set_dio_irq_params(WL55_cfg.drv_cfg.irq_mask, WL55_cfg.drv_cfg.irq_mask, STM32WLxx_SUBGHZ_IRQ_NONE, STM32WLxx_SUBGHZ_IRQ_NONE);
    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_RADIO_LOG_ERROR("Failed to restore the deffault SubGHz IRQ mask after processing IRQ. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Sets modem to FSK mode
 *
 * @returns Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_modem_to_fsk(void)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        err = _sid_radio_set_pkt_type(PACKET_TYPE_GFSK);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("Failed to set SubGHz paket type to FSK. Error %u", (uint32_t)err);
            break;
        }

        current_modem_mode = STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK;
        WL55_cfg.drv_cfg.irq_mask = STM32WLxx_RADIO_COMM_DEFAULT_FSK_IRQ_MASK;
        SID_RADIO_LOG_DEBUG("_sid_radio_set_modem_to_fsk: setting IRQ mask 0x%x", WL55_cfg.drv_cfg.irq_mask);

        err = _sid_radio_enable_default_irqs();
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Logs are provided by _sid_radio_enable_default_irqs() */
            break;
        }

        err = _sid_radio_subghz_set_gfsk_sync_word(WL55_cfg.fsk_cfg.sync_word, WL55_cfg.fsk_cfg.sync_word_len);
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Logs are provided by _sid_radio_subghz_set_gfsk_sync_word() */
            break;
        }

        err = _sid_radio_subghz_set_gfsk_mod_params(&WL55_cfg.fsk_cfg.mod);
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Logs are provided by _sid_radio_subghz_set_gfsk_mod_params() */
            break;
        }

        err = _sid_radio_subghz_set_gfsk_pkt_params(&WL55_cfg.fsk_cfg.pkt);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Sets modem to LoRa mode
 *
 * @returns Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_modem_to_lora(void)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        err = _sid_radio_set_pkt_type(PACKET_TYPE_LORA);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("Failed to set SubGHz paket type to LoRa. Error %u", (uint32_t)err);
            break;
        }

        current_modem_mode = STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA;
        WL55_cfg.drv_cfg.irq_mask = STM32WLxx_RADIO_COMM_DEFAULT_LORA_IRQ_MASK;
        SID_RADIO_LOG_DEBUG("_sid_radio_set_modem_to_lora: setting IRQ mask 0x%x", WL55_cfg.drv_cfg.irq_mask);

        err = _sid_radio_enable_default_irqs();
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Logs are provided by _sid_radio_enable_default_irqs() */
            break;
        }

        err = _sid_radio_subghz_set_lora_sync_word(WL55_cfg.lora_cfg.sync_word);
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Logs are provided by _sid_radio_subghz_set_lora_sync_word() */
            break;
        }

        err = _sid_radio_subghz_set_lora_mod_params(&WL55_cfg.lora_cfg.mod);
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Logs are provided by _sid_radio_subghz_set_lora_mod_params() */
            break;
        }

        err = _sid_radio_subghz_set_lora_pkt_params(&WL55_cfg.lora_cfg.pkt);
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Logs are provided by _sid_radio_subghz_set_lora_pkt_params() */
            break;
        }

        //FIXME: set CAD params

         err = _sid_radio_subghz_set_lora_symb_nb_timeout(WL55_cfg.lora_cfg.symbol_timeout);
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Logs are provided by _sid_radio_subghz_set_lora_symb_nb_timeout() */
            break;
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint8_t _sid_radio_lora_low_data_rate_optimize(const stm32wlxx_lora_sf_t sf, const stm32wlxx_lora_bw_t bw)
{
    if (((bw == STM32WLxx_LORA_BW_125) && ((sf == STM32WLxx_LORA_SF11) || (sf == STM32WLxx_LORA_SF12)))
        || ((bw == STM32WLxx_LORA_BW_250) && (sf == STM32WLxx_LORA_SF12)))
    {
        return 1u; /* Low data rates optimizations shall be on */
    }
    else
    {
        return 0u; /* Low data rate optimizations are not required */
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _sid_radio_convert_to_phy_lora_mod_params(stm32wlxx_mod_params_lora_t * const sx_m, const stm32wlxx_radio_lora_phy_mod_params_t * const in_mod_params)
{
    sx_m->sf   = (stm32wlxx_lora_sf_t)in_mod_params->spreading_factor;
    sx_m->bw   = (stm32wlxx_lora_bw_t)in_mod_params->bandwidth;
    sx_m->ldro = _sid_radio_lora_low_data_rate_optimize(sx_m->sf, sx_m->bw);

    /* unfortunately long interleaved mode is not supported by STM32WLxx, so changing it to modes without LI */
    switch (in_mod_params->coding_rate)
    {
        case STM32WLxx_LORA_CR_4_5:
        case STM32WLxx_LORA_CR_4_5_LI:
            sx_m->cr = STM32WLxx_LORA_CR_4_5;
            break;

        case STM32WLxx_LORA_CR_4_6:
        case STM32WLxx_LORA_CR_4_6_LI:
            sx_m->cr = STM32WLxx_LORA_CR_4_6;
            break;

        case STM32WLxx_LORA_CR_4_7:
            sx_m->cr = STM32WLxx_LORA_CR_4_7;
            break;

        case STM32WLxx_LORA_CR_4_8:
        case STM32WLxx_LORA_CR_4_8_LI:
            sx_m->cr = STM32WLxx_LORA_CR_4_8;
            break;

        default:
            SID_PAL_LOG_ERROR("_sid_radio_convert_to_phy_lora_mod_params: coding rate selector is not supported, requested CR is 0x%x", (uint32_t)in_mod_params->coding_rate);
            break;
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline RBI_Switch_TypeDef _sid_radio_rbi_get_pa_selector(const uint32_t request_hpa)
{
    RBI_Switch_TypeDef pa_select;

    int32_t tx_config = RBI_GetTxConfig(); /* Get board capabilities */

    switch (tx_config)
    {
        case RBI_CONF_RFO_LP_HP:
            /* Board provides both external power amp and direct antenna connection - use the one that is requested */
            pa_select = (request_hpa != FALSE) ? RBI_SWITCH_RFO_HP : RBI_SWITCH_RFO_LP;
            break;

        case RBI_CONF_RFO_LP:
            /* The board has only a direct antenna connection */
            if (request_hpa != FALSE)
            {
                SID_PAL_LOG_ERROR("Requesting HPA for SubGHz Tx, but the board does not support it");
                pa_select = RBI_SWITCH_OFF;
            }
            else
            {
                pa_select = RBI_SWITCH_RFO_LP;
            }
            break;

        case RBI_CONF_RFO_HP:
            if (FALSE == request_hpa)
            {
                SID_PAL_LOG_ERROR("Requesting LPA for SubGHz Tx, but the board does not support it");
                pa_select = RBI_SWITCH_OFF;
            }
            else
            {
                pa_select = RBI_SWITCH_RFO_HP;
            }
            break;

        default:
            SID_PAL_LOG_ERROR("Unknown SubGHz RBI Tx capabilities (0x%X)");
            pa_select = RBI_SWITCH_OFF;
            break;
    }

    return pa_select;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Set Radio Mode
 *
 * @param [in] frontend_mode Desired radio frontend mode (e.g. Tx, Rx, Off)
 * @return     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_radio_mode(const stm32wlxx_subghz_fe_mode_t frontend_mode)
{
    sid_radio_error_t  err;
    RBI_Switch_TypeDef rbi_switch_cfg;
    uint8_t smps_drive_lvl = SMPS_DRIVE_SETTING_DEFAULT;

    do
    {
        switch (frontend_mode)
        {
            case STM32WLxx_RADIO_FRONTEND_MODE_OFF:
                rbi_switch_cfg = RBI_SWITCH_OFF;
                break;

            case STM32WLxx_RADIO_FRONTEND_MODE_TX:
                if (SID_RADIO_SUBGHZ_PA_SEL_HPA == WL55_cfg.pa_cfg.device_sel)
                {
                    rbi_switch_cfg = RBI_SWITCH_RFO_HP;
                }
                else
                {
                    rbi_switch_cfg = RBI_SWITCH_RFO_LP;

                    /* If SMPS is used we need to increase its output power since LPA draws current from SMPS */
                    smps_drive_lvl = SMPS_DRIVE_SETTING_MAX;
                }
                break;

            case STM32WLxx_RADIO_FRONTEND_MODE_RX:
                rbi_switch_cfg = RBI_SWITCH_RX;
                break;

            default:
                SID_PAL_LOG_ERROR();
                return SID_RADIO_ERROR_INVALID_ARGS;
        }

        /* Adjust SMPS settings if it is used */
        if (RBI_IsDCDC() != FALSE)
        {
            err = _sid_radio_smps_set(smps_drive_lvl);
            if (err != SID_RADIO_ERROR_NONE)
            {
                SID_RADIO_LOG_ERROR("Failed to set SMPS current drive to 0x%02X. Error %u", smps_drive_lvl, (uint32_t)err);
                break;
            }
        }

        /* Apply antenna switch config */
        const int32_t rbi_err = RBI_ConfigRFSwitch(rbi_switch_cfg);
        if (rbi_err != 0u)
        {
            SID_PAL_LOG_ERROR("Failed to configure SubGHz antenna switch. RBI error %d, switch cfg: %u", rbi_err, rbi_switch_cfg);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to set SubGHz radio mode. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_sleep(const uint32_t sleep_duration_us)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    stm32wlxx_subghz_sleep_params_t sleep_cfg = {
        .raw = 0x00u,
    };
#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
    sid_radio_error_t udt_err;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

    do
    {
        /* Immediately stop software timer for Tx/Rx/CS/CAD timeout */
        (void)UTIL_TIMER_Stop(&radio_timeout_mon);
        __COMPILER_BARRIER();

        if (SID_PAL_RADIO_SLEEP == current_radio_state)
        {
            /* Already in Sleep, no actions required */
            err = SID_RADIO_ERROR_NONE;

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
            /* Reconfigure the UDT cut-off time based on the new expected sleep duration */
            udt_err = _sid_radio_udt_setup_session(sleep_duration_us);
            if (udt_err != SID_RADIO_ERROR_NONE)
            {
                /* UDT failure is not critical for Sidewalk operations - just inform about it and proceed normally */
                SID_PAL_LOG_WARNING("Failed to reconfigure UDT. Error %u", (uint32_t)udt_err);
            }
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

            break;
        }

        /* Sleep can be entered from the Standby states only, so ensure the radio is Standby.
         * This will also cancel any ongoing Tx/Rx operations, turn off the FEM, and configure SMPS to lower the power consumption.
         */
        err = _sid_radio_subghz_standby();
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("SubGHz radio Sleep failed, radio unable to enter Standby. Error %u", (uint32_t)err);
            break;
        }

        /* Configure the desired Sleep parameters - enable register retention for shorter wakeup time */
        sleep_cfg.sleep_type = STM32WLxx_RADIO_SLEEP_WARM_START;

        err = _sid_radio_subghz_set_sleep(sleep_cfg);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("SubGHz Sleep request failed. Unable to send Sleep command to SubGHz. Error %u", (uint32_t)err);
            current_radio_state = SID_PAL_RADIO_UNKNOWN;
            break;
        }

        /* IMPORTANT: do not enable LPM here since radio sleep request may need to be acknowledged */

        current_radio_state = SID_PAL_RADIO_SLEEP;
        err = SID_RADIO_ERROR_NONE;

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
        udt_err = _sid_radio_udt_setup_session(sleep_duration_us);
        if (udt_err != SID_RADIO_ERROR_NONE)
        {
            /* UDT failure is not critical for Sidewalk operations - just inform about it and proceed normally */
            SID_PAL_LOG_WARNING("Failed to enable UDT. Error %u", (uint32_t)udt_err);
        }
#else
        (void)sleep_duration_us;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */
    } while (0);

    SID_RADIO_LOG_DEBUG("_sid_radio_subghz_sleep - done");

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Switch radio and SW to standby state
 *
 * @return Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_standby(void)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        /* Immediately stop software timer for Tx/Rx/CS/CAD timeout */
        (void)UTIL_TIMER_Stop(&radio_timeout_mon);
        __COMPILER_BARRIER();

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
        /* Prohibit the system to enter Stop mode since SubGHz events should be processed ASAP from this point */
        (void)UTIL_TIMER_Stop(&lpm_suspend_timer);
        __COMPILER_BARRIER(); /* Esnure the timer is always processed before the UTIL_LPM_Set<X>Mode() calls */

        UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_RADIO), UTIL_LPM_DISABLE);
        UTIL_LPM_SetOffMode((1u << CFG_LPM_HOST_COMM_RADIO), UTIL_LPM_DISABLE);
#endif /* LOW_POWER_DISABLE */

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
        udt_ctx.udt_enabled = FALSE;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

        if (SID_PAL_RADIO_STANDBY == current_radio_state)
        {
            /* Already in StandBy, no actions required */
            err = SID_RADIO_ERROR_NONE;
            break;
        }

        if ((SID_PAL_RADIO_SLEEP == current_radio_state) || (SID_PAL_RADIO_UNKNOWN == current_radio_state))
        {
            err = _sid_radio_subghz_wakeup(SID_RADIO_SUBGHZ_WAIT_FOR_WAKEUP_TIMEOUT_MS);
            if (err != SID_RADIO_ERROR_NONE)
            {
                SID_RADIO_LOG_ERROR("SubGHz Standby request failed. Unable to wake up SubGHz peripheral. Error %u", (uint32_t)err);
                current_radio_state = SID_PAL_RADIO_UNKNOWN;
                break;
            }

            /* Restore Rx Gain configuration since it is not part of the retained memory */
            err = _sid_radio_cfg_rx_boosted(WL55_cfg.drv_cfg.rx_boost);
            if (err != SID_RADIO_ERROR_NONE)
            {
                SID_RADIO_LOG_ERROR("Failed to set SubGHz Rx boost. Error %u", (uint32_t)err);
                current_radio_state = SID_PAL_RADIO_UNKNOWN;
                break;
            }
        }

        err = _sid_radio_subghz_set_radio_mode(STM32WLxx_RADIO_FRONTEND_MODE_OFF);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Use STDBY_XOSC as Standby-to-Tx/Rx transition time is critical for FSK */
        err = _sid_radio_subghz_set_standby(STDBY_XOSC);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("SubGHz Standby request failed. Unable to send Standby command to SubGHz. Error %u", (uint32_t)err);
            current_radio_state = SID_PAL_RADIO_UNKNOWN;
            break;
        }
        sid_pal_delay_us(STM32WLxx_SUBGHZ_STDBY_STATE_DELAY_US);

        current_radio_state = SID_PAL_RADIO_STANDBY;
        err = SID_RADIO_ERROR_NONE;
    } while (0);

    SID_RADIO_LOG_DEBUG("_sid_radio_subghz_standby - done");

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Set the radio transmit power.
 *
 * @param[in] power tx power in dB
 * @return    Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static sid_radio_error_t _sid_radio_set_tx_power(const stm32wlxx_radio_pa_setup_t * const new_pa_cfg)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    HAL_StatusTypeDef hal_err;

    do
    {
        /* Skip applying the settings if they are applied already. The only exception - re-applying the WL55_cfg itself */
        if ((new_pa_cfg != &WL55_cfg.pa_cfg) && (SID_STM32_UTIL_fast_memcmp(new_pa_cfg, &WL55_cfg.pa_cfg, sizeof(*new_pa_cfg)) == 0u))
        {
            SID_RADIO_LOG_DEBUG("Tx power already set to %s%ddB - Set Tx Power cmd skipped", WL55_cfg.pa_cfg.target_tx_power > 0 ? "+" : "", WL55_cfg.pa_cfg.target_tx_power); /* tiny_vsnprintf() may not support %+d */
            err = SID_RADIO_ERROR_NONE;
            break;
        }

        if (current_radio_state != SID_PAL_RADIO_STANDBY)
        {
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        err = _sid_radio_set_pa_cfg(new_pa_cfg);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("Failed to set SubGHz PA config. Error %u", (uint32_t)err);
            break;
        }

        uint8_t ocp_reg = (SID_RADIO_SUBGHZ_PA_SEL_HPA == new_pa_cfg->device_sel) ? STM32WLxx_SUBGHZ_REG_OCP_HPA_VAL : STM32WLxx_SUBGHZ_REG_OCP_LPA_VAL;
        hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, REG_OCP, &ocp_reg, sizeof(ocp_reg));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", REG_OCP, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

#if STM32WLxx_SUBGHZ_TXPWR_WORKAROUND
        {
            uint8_t ovp_reg;

            hal_err = HAL_SUBGHZ_ReadRegisters(&hsubghz, STM32WLxx_SUBGHZ_REG_OVP, &ovp_reg, sizeof(ovp_reg));
            if (hal_err != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Failed to read SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", STM32WLxx_SUBGHZ_REG_OVP, hal_err);
                err = SID_RADIO_ERROR_HARDWARE;
                break;
            }

            ovp_reg &= 0xF9u; /* Set bits 1 and 2 to 0 */

            hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, STM32WLxx_SUBGHZ_REG_OVP, &ovp_reg, sizeof(ovp_reg));
            if (hal_err != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", STM32WLxx_SUBGHZ_REG_OVP, hal_err);
                err = SID_RADIO_ERROR_HARDWARE;
                break;
            }
        }
#endif /* STM32WLxx_SUBGHZ_TXPWR_WORKAROUND */

        err = _sid_radio_set_tx_params(new_pa_cfg);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("Failed to set SubGHz Tx params. Error %u", (uint32_t)err);
            break;
        }

        /* Store the applied setting */
        if (new_pa_cfg != &WL55_cfg.pa_cfg)
        {
            WL55_cfg.pa_cfg = *new_pa_cfg;
        }
        SID_PAL_LOG_INFO("SubGHz Tx power set to %s%ddB", WL55_cfg.pa_cfg.target_tx_power > 0 ? "+" : "", WL55_cfg.pa_cfg.target_tx_power); /* tiny_vsnprintf() may not support %+d */

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Set radio modem mode.
 *
 *  The driver should configure all the parameters
 *  to operate in the desired mode.
 *  Supported modem modes are LoRa and FSK
 *
 *  @param[in]   mode LoRa = 2 or FSK = 1
 *
 *  @return true in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_modem_mode(const stm32wlxx_rcp_set_modem_mode_req_t * const request)
{
    sid_radio_error_t err;
    const stm32wlxx_pal_radio_modem_mode_t desired_mode = (stm32wlxx_pal_radio_modem_mode_t)request->mode;
    const char * mode_str;

    do
    {
        if (desired_mode == current_modem_mode)
        {
            SID_RADIO_LOG_DEBUG("Set Modem Mode skipped - already in requested mode");
            err = SID_RADIO_ERROR_NONE;
            break;
        }

        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to set modem mode before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        switch(desired_mode)
        {
            case STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA:
                err = _sid_radio_set_modem_to_lora();
                mode_str = "LoRa";
                break;

            case STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK:
                err = _sid_radio_set_modem_to_fsk();
                mode_str = "FSK";
                break;

            case STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_UNDEFINED:
                /* Can get into this mode after a reset, but cannot set it explicitly upon request */
                err = SID_RADIO_ERROR_INVALID_ARGS;
                mode_str = "Undefined";
                break;

            default:
                err = SID_RADIO_ERROR_UNEXPECTED_DATA;
                SID_PAL_LOG_ERROR("Failed to set SubGHz modem mode. Unknown mode requersted (0x%x)", (uint32_t)desired_mode);
                mode_str = "unknown";
                break;
        }

        if (SID_RADIO_ERROR_NONE == err)
        {
            SID_PAL_LOG_INFO("SubGHz radio modem set to %s mode", mode_str);
        }
        else
        {
            SID_PAL_LOG_ERROR("Failed to set SubGHz radio modem to %s mode. Error: %u", mode_str, (uint32_t)err);
            break;
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_get_payload(uint8_t * const buffer, const uint32_t buf_size, uint32_t * const actual_size)
{
    sid_radio_error_t            err = SID_RADIO_ERROR_GENERIC;
    HAL_StatusTypeDef            hal_err;
    stm32wlxx_rx_buffer_status_t rx_buffer_status;

    assert_param(buffer != NULL);
    assert_param(actual_size != NULL);

    do
    {
        *actual_size = 0u;

        /* Readout Rx packet status from SubGHz */
        err = _sid_radio_subghz_get_rx_buffer_status(&rx_buffer_status);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Workaround for sporadic invalid GetRxBufferStatus reports */
        uint32_t buffer_status_retry_counter = 0u;
        while ((0u == rx_buffer_status.pld_len_in_bytes) && (buffer_status_retry_counter < SID_RADIO_LORA_GET_RX_BUFFER_STATUS_RETRIES))
        {
            /* Readout Rx packet status from SX126x again to address internal race condition */
            err = _sid_radio_subghz_get_rx_buffer_status(&rx_buffer_status);
            if (err != SID_RADIO_ERROR_NONE)
            {
                break;
            }

            /* Increment retry counter on successful read */
            buffer_status_retry_counter++;
        }
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Terminate if a fatal error occurred while fetching Rx buffer status */
            break;
        }
        /* Proceed even if the retry limit is reached and the reported packet length is still zero - the generic packet length check below will react on that */

        /* Validate reported packet length */
        if ((0u == rx_buffer_status.pld_len_in_bytes) || (rx_buffer_status.pld_len_in_bytes > buf_size))
        {
            err = SID_RADIO_ERROR_WRONG_SIZE;
            break;
        }

        /* Read out received data from SubGHz */
        hal_err = HAL_SUBGHZ_ReadBuffer(&hsubghz, rx_buffer_status.buffer_start_pointer, buffer, rx_buffer_status.pld_len_in_bytes);
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read %u bytes from SubGHz buffer. HAL_SUBGHZ err: 0x%x", rx_buffer_status.pld_len_in_bytes, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        *actual_size = (uint32_t)rx_buffer_status.pld_len_in_bytes;

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_lora_process_rx_done(stm32wlxx_rcp_lora_rx_packet_info_t * const lora_rx_packet_status, uint8_t * const rx_storage, const uint32_t rx_storage_size, uint32_t * const out_received_len)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    HAL_StatusTypeDef hal_err;

    do
    {
        /* Read out received payload from SubGHz's internal RAM buffer */
        err = _sid_radio_subghz_get_payload(rx_storage, rx_storage_size, out_received_len);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to read SubGHz internal RAM buffer. Error %u", (uint32_t)err);
            break;
        }

        /* Collect LoRa packet info */
        err = _sid_radio_subghz_get_lora_pkt_status(lora_rx_packet_status);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to read SubGHz LoRa packet status. Error %u", (uint32_t)err);
            break;
        }

        /* Determine CRC status */
        uint8_t buffer_header_crc;
        hal_err = HAL_SUBGHZ_ReadRegisters(&hsubghz, STM32WL55xx_SUBGHZ_REG_LR_HEADER_CRC, &buffer_header_crc, sizeof(buffer_header_crc));
        if (HAL_OK != hal_err)
        {
            lora_rx_packet_status->is_crc_present = STM32WLxx_PAL_RADIO_CRC_PRESENT_INVALID;
            SID_PAL_LOG_ERROR("Failed to read SubGHz LoRa header CRC status. Hal error 0x%x", (uint32_t)hal_err);
            if (HAL_BUSY == hal_err)
            {
                err = SID_RADIO_ERROR_RESOURCE_BUSY;
            }
            else
            {
                err = SID_RADIO_ERROR_HARDWARE;
            }
            break;
        }

        lora_rx_packet_status->is_crc_present = ((buffer_header_crc & STM32WL55xx_SUBGHZ_REG_LR_HEADER_CRC_MASK) != 0u) ? STM32WLxx_PAL_RADIO_CRC_PRESENT_ON : STM32WLxx_PAL_RADIO_CRC_PRESENT_OFF;

        err = SID_RADIO_ERROR_NONE;
    } while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * perform data whitening
 *
 * @param [in] seed             whitening seed
 * @param [in] buffer_in        pointer to the buffer that needs to be updated
 * @param [in] buffer_out       pointer to the updated buffer
 * @param [in] length           length of the buffer to perform data whitnening
 *
 */
SID_STM32_SPEED_OPTIMIZED static inline void _sid_radio_fsk_perform_data_whitening(const uint16_t seed, const uint8_t * const buffer_in, uint8_t * const buffer_out, const uint32_t length)
{
    uint16_t lfsr    = seed;
    uint16_t xor_out = 0u;
    uint8_t  ret     = 0u;
    uint8_t  result  = 0u;

    for (uint32_t index = 0u; index < length; index++)
    {
        xor_out = 0u;
        ret     = 0u;
        result  = 0u;

        xor_out  = (uint16_t)( ( lfsr >> 5 ) & 0x0Fu ) ^ ( lfsr & 0x0Fu );
        lfsr     = (uint16_t)( lfsr >> 4 ) | ( xor_out << 5 );
        ret     |= (uint8_t) ( lfsr >> 5 ) & 0x0Fu;

        xor_out  = (uint16_t)( ( lfsr >> 5 ) & 0x0F ) ^ ( lfsr & 0x0F );
        lfsr     = (uint16_t)( lfsr >> 4 ) | ( xor_out << 5 );
        ret     |= (uint8_t) ( ( lfsr >> 1 ) & 0xF0u );

        result |= (uint8_t)( ret & 0x80u ) >> 7;
        result |= (uint8_t)( ret & 0x40u ) >> 5;
        result |= (uint8_t)( ret & 0x20u ) >> 3;
        result |= (uint8_t)( ret & 0x10u ) >> 1;
        result |= (uint8_t)( ret & 0x08u ) << 1;
        result |= (uint8_t)( ret & 0x04u ) << 3;
        result |= (uint8_t)( ret & 0x02u ) << 5;
        result |= (uint8_t)( ret & 0x01u ) << 7;

        buffer_out[index] = (uint8_t)(buffer_in[index] ^ result);
    }
}

/*----------------------------------------------------------------------------*/

/**
 * compute crc16
 *
 *
 * @param [in] buffer          buffer on which crc16 needed to be calculated
 * @param [in] length          length of the buffer that is passed
 *
 * @retval status     calculated crc16 value
 */
SID_STM32_SPEED_OPTIMIZED static inline uint16_t _sid_radio_compute_crc16(const uint8_t * const buffer, const uint32_t length)
{
    if ((NULL == buffer) || (0u == length))
    {
        return 0u;
    }

    register uint16_t crc16 = 0x0000u;

    for (uint32_t index_buffer = 0u; index_buffer < length; index_buffer++)
    {
        crc16 ^= ( ( uint16_t ) buffer[index_buffer] << 8 );

        for (uint32_t i = 0u; i < 8u; i++ )
        {
            crc16 = (uint16_t)(( crc16 & 0x8000u ) ? ( crc16 << 1 ) ^ SID_RADIO_POLYNOMIAL_CRC16 : ( crc16 << 1 ));
        }
    }

    return crc16;
}

/*----------------------------------------------------------------------------*/

/**
 * compute crc32
 *
 *
 * @param [in] buffer          buffer on which crc32 needed to be calculated
 * @param [in] length          length of the buffer that is passed
 *
 * @retval status     calculated crc32 value
 */
SID_STM32_SPEED_OPTIMIZED static inline uint32_t _sid_radio_compute_crc32(const uint8_t * const buffer, const uint32_t length)
{
    if ((NULL == buffer) || (0u == length))
    {
        return 0u;
    }

    uint8_t           temp_buffer[sizeof(uint32_t)] = {0};
    register uint32_t crc32                         = 0xFFFFFFFFu;
    const uint8_t *   buffer_local;
    uint32_t          length_local;

    if (length < sizeof(uint32_t))
    {
        SID_STM32_UTIL_fast_memcpy(temp_buffer, buffer, length);
        length_local = sizeof(uint32_t);
        buffer_local = temp_buffer;
    }
    else
    {
        length_local = length;
        buffer_local = buffer;
    }

    for (uint32_t index_buffer = 0u; index_buffer < length_local; index_buffer++)
    {
        crc32 ^= ( index_buffer < length ) ? ( ( uint32_t ) buffer_local[index_buffer] << 24 ) : 0x00000000u;

        for (uint32_t i = 0u; i < 8u; i++ )
        {
            crc32 = ( crc32 & 0x80000000u ) ? ( crc32 << 1 ) ^ SID_RADIO_POLYNOMIAL_CRC32 : ( crc32 << 1 );
        }
    }

    return ~crc32;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_fsk_process_rx_done(stm32wlxx_rcp_fsk_rx_packet_info_t * const fsk_rx_packet_status, uint8_t * const rx_storage, const uint32_t rx_storage_size, uint32_t * const out_received_len, stm32wlxx_radio_fsk_rx_done_status_t * const out_fsk_rx_done_status)
{
    sid_radio_error_t            err = SID_RADIO_ERROR_GENERIC;
    stm32wlxx_pal_radio_fsk_phy_hdr_t phy_hdr;
    stm32wlxx_pkt_status_gfsk_t  pkt_status;
    uint32_t                     fsk_payload_len;
    uint32_t                     crc;
    uint32_t                     crc_length;
    stm32wlxx_rx_buffer_status_t rx_buffer_status;
    HAL_StatusTypeDef            hal_err;

    do
    {
        *out_received_len       = 0u;
        *out_fsk_rx_done_status = STM32WLxx_RADIO_FSK_RX_DONE_STATUS_UNDETERMINED;

        /* Check the validity marker to determine if the packet header is stored in the buffer already */
        uint32_t * const validity_marker_ptr = (uint32_t *)(void *)&rx_storage[SID_RADIO_FSK_SYNC_WORD_VALID_MARKER_OFFSET];
        if (SID_RADIO_FSK_SYNC_WORD_VALID_MARKER != *validity_marker_ptr)
        {
            *out_fsk_rx_done_status = STM32WLxx_RADIO_FSK_RX_DONE_STATUS_SW_MARK_NOT_PRESENT;
            err = SID_RADIO_ERROR_UNEXPECTED_DATA;
            break;
        }

        /* Clear the validity marker */
        *validity_marker_ptr = 0u;

        /* Parse the header */
        phy_hdr.fcs_type = rx_storage[SID_RADIO_FSK_PACKET_TYPE_OFFSET] >> 4;
        phy_hdr.is_data_whitening_enabled = (rx_storage[SID_RADIO_FSK_PACKET_TYPE_OFFSET] & (1u << 3)) ? true : false;

        /* Determine the expected payload length from the FSK packet header */
        fsk_payload_len = rx_storage[SID_RADIO_FSK_PACKET_LENGTH_OFFSET];

        /* Esnure we have enough space to reedout the payload */
        assert_param((fsk_payload_len) <= rx_storage_size);

        /* Check that expected packet length at least covers the CRC part + 1 byte */
        if ( ((STM32WLxx_RADIO_FSK_FCS_TYPE_0 == phy_hdr.fcs_type) && (fsk_payload_len <= sizeof(uint32_t)))
          || ((STM32WLxx_RADIO_FSK_FCS_TYPE_1 == phy_hdr.fcs_type) && (fsk_payload_len <= sizeof(uint16_t))))
        {
            *out_fsk_rx_done_status = STM32WLxx_RADIO_FSK_RX_DONE_STATUS_BAD_CRC;
            err = SID_RADIO_ERROR_CORRUPTED_DATA;
            break;
        }

        /* Readout Rx packet status from the SubGHz peripheral */
        err = _sid_radio_subghz_get_rx_buffer_status(&rx_buffer_status);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Workaround for sporadic invalid GetRxBufferStatus reports */
        uint32_t buffer_status_retry_counter = 0u;
        while ((0u == rx_buffer_status.pld_len_in_bytes) && (buffer_status_retry_counter < SID_RADIO_FSK_GET_RX_BUFFER_STATUS_RETRIES))
        {
            /* Readout Rx packet status from SX126x again to address internal race condition */
            err = _sid_radio_subghz_get_rx_buffer_status(&rx_buffer_status);
            if (err != SID_RADIO_ERROR_NONE)
            {
                break;
            }

            /* Increment retry counter on successful read */
            buffer_status_retry_counter++;
        }
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Terminate if a fatal error occurred while fetching Rx buffer status */
            break;
        }
        /* Proceed even if the retry limit is reached and the reported packet length is still zero - the generic packet length check below will react on that */

        /* Validity check: actually received data length should match the info from the header */
        if (rx_buffer_status.pld_len_in_bytes != (STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH + fsk_payload_len))
        {
            *out_fsk_rx_done_status = STM32WLxx_RADIO_FSK_RX_DONE_STATUS_INVALID_LENGTH;
            err = SID_RADIO_ERROR_WRONG_SIZE;
            break;
        }

        /* Readout FSK payload only, we don't need the header anymore */
        hal_err = HAL_SUBGHZ_ReadBuffer(&hsubghz, (rx_buffer_status.buffer_start_pointer + STM32WLxx_RADIO_COMM_FSK_PHY_HEADER_LENGTH), rx_storage, fsk_payload_len);
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to read %u bytes from SubGHz buffer. HAL_SUBGHZ err: 0x%x", fsk_payload_len, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* Do data unwhitening if required */
        if (phy_hdr.is_data_whitening_enabled != false)
        {
            _sid_radio_fsk_perform_data_whitening(SID_RADIO_FSK_WHITENING_SEED, rx_storage, rx_storage, fsk_payload_len);
        }

        /* Compute CRC of the received payload */
        switch (phy_hdr.fcs_type)
        {
            case STM32WLxx_RADIO_FSK_FCS_TYPE_0:
                crc_length = sizeof(uint32_t);
                crc        = _sid_radio_compute_crc32(rx_storage, fsk_payload_len - crc_length);
                break;

            case STM32WLxx_RADIO_FSK_FCS_TYPE_1:
                crc_length = sizeof(uint16_t);
                crc        = _sid_radio_compute_crc16(rx_storage, fsk_payload_len - crc_length);
                break;

            default:
                *out_received_len       = 0u;
                *out_fsk_rx_done_status = STM32WLxx_RADIO_FSK_RX_DONE_STATUS_UNKNOWN_ERROR;
                err                     = SID_RADIO_ERROR_UNEXPECTED_DATA;
                break;
        }

        /* Jump out of loop if switch() ended with an error */
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Store the amount of the payload we've got without the CRC part */
        *out_received_len = (uint16_t)(fsk_payload_len - crc_length);

        /* Verify CRC */
        for (uint32_t i = 0u; i < crc_length; i++)
        {
            if (rx_storage[*out_received_len + i] != (uint8_t)(crc >> (8*(crc_length - i - 1 ))))
            {
                *out_fsk_rx_done_status = STM32WLxx_RADIO_FSK_RX_DONE_STATUS_BAD_CRC;
                err                     = SID_RADIO_ERROR_CORRUPTED_DATA;
                break;
            }
        }
        /* Jump out if CRC error detected */
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        err = _sid_radio_subghz_get_gfsk_pkt_status(&pkt_status);
        if (err != SID_RADIO_ERROR_NONE)
        {
            *out_fsk_rx_done_status = STM32WLxx_RADIO_FSK_RX_DONE_STATUS_TIMEOUT;
            break;
        }

        fsk_rx_packet_status->rssi_sync = (int8_t)_sid_radio_get_adjusted_rssi(pkt_status.rssi_sync);
        fsk_rx_packet_status->rssi_avg  = (int8_t)_sid_radio_get_adjusted_rssi(pkt_status.rssi_avg);
        fsk_rx_packet_status->snr       = 0; /* Measurement is not available */

        err = SID_RADIO_ERROR_NONE;
    } while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _sid_radio_convert_fsk_packet_params(stm32wlxx_pkt_params_gfsk_t * const fsk_pp, const stm32wlxx_pal_radio_fsk_pkt_params_t * const pkt_params)
{
    if (pkt_params->preamble_length > 1u)
    {
        fsk_pp->pbl_len_in_bits   = (pkt_params->preamble_length - 1u) << 3;
    }
    else
    {
        fsk_pp->pbl_len_in_bits   = 0u;
    }
    fsk_pp->pbl_min_det           = (stm32wlxx_gfsk_pbl_det_t)pkt_params->preamble_min_detect;
    fsk_pp->sync_word_len_in_bits = pkt_params->sync_word_length << 3;
    fsk_pp->addr_cmp              = (stm32wlxx_gfsk_addr_cmp_t)pkt_params->addr_comp;
    fsk_pp->hdr_type              = (stm32wlxx_gfsk_pkt_len_modes_t)pkt_params->header_type;
    fsk_pp->pld_len_in_bytes      = pkt_params->payload_length;
    fsk_pp->crc_type              = (stm32wlxx_gfsk_crc_types_t)pkt_params->crc_type;
    fsk_pp->dc_free               = (stm32wlxx_gfsk_dc_free_t)pkt_params->radio_whitening_mode;
}

/*----------------------------------------------------------------------------*/

/** @brief Set the frequency for the radio.
 *
 *  @param[in] request Request payload with the settings
 *  @retval    Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_frequency(const stm32wlxx_rcp_set_frequency_req_t * const request)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        if (request->frequency == current_radio_freq_hz)
        {
            SID_RADIO_LOG_DEBUG("Radio frequency is already set to %u - skipping Set Frequency request", current_radio_freq_hz);
            err = SID_RADIO_ERROR_NONE;
            break;
        }

        if (current_radio_state != SID_PAL_RADIO_STANDBY)
        {
           SID_RADIO_LOG_ERROR("Cannot set radio freq to %u, radio is not in standby", request->frequency);
           err = SID_RADIO_ERROR_INVALID_STATE;
           break;
        }

        /* Check if we are going to switch the frequency bands and if calibration is needed due to this */
        const stm32wlxx_freq_cal_band_t current_freq_band = SID_RADIO_GET_FREQ_BAND(current_radio_freq_hz);
        const stm32wlxx_freq_cal_band_t desired_freq_band = SID_RADIO_GET_FREQ_BAND(request->frequency);
        if (current_freq_band != desired_freq_band)
        {
            err = _sid_radio_cal_img(desired_freq_band);
            if (SID_RADIO_ERROR_NONE == err)
            {
                SID_PAL_LOG_INFO("SubGHz frequency band set to %s", SID_RADIO_GET_FREQ_BAND_STR(desired_freq_band));
            }
            else
            {
                /* Logs provided by _sid_radio_cal_img() */
                break;
            }
        }

#if STM32WLxx_SUBGHZ_TXPWR_WORKAROUND
        if (STM32WLxx_SUBGHZ_BAND_900M == desired_freq_band)
        {
            HAL_StatusTypeDef hal_err;
            uint8_t           reg_val = (request->frequency <= STM32WLxx_SUBGHZ_BAND_EDGE_LIMIT_FREQ) ? STM32WLxx_SUBGHZ_REG_VAL_FREQ_LOW : STM32WLxx_SUBGHZ_REG_VAL_FREQ_HIGH;

            hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, STM32WLxx_SUBGHZ_REG_LR_CFG_FREQ, &reg_val, sizeof(reg_val));
            if (hal_err != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", STM32WLxx_SUBGHZ_REG_LR_CFG_FREQ, hal_err);
                err = SID_RADIO_ERROR_HARDWARE;
                break;
            }
        }
#endif /* STM32WLxx_SUBGHZ_TXPWR_WORKAROUND */

        err = _sid_radio_set_rf_freq(request->frequency);
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Logs provided by _sid_radio_set_rf_freq() */
            break;
        }

        /* Store the newly applied frequency */
        current_radio_freq_hz = request->frequency;

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to set SubGHz frequency to %u. Error %u", request->frequency, err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Set sync word
 *
 *  @retval Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_sync_word(const stm32wlxx_rcp_set_sync_word_req_t * const request)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to set modem sync word before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == request->modem)
        {
            if ((request->fsk_sync_word.data_len == WL55_cfg.fsk_cfg.sync_word_len)
              && (SID_STM32_UTIL_fast_memcmp(WL55_cfg.fsk_cfg.sync_word, request->fsk_sync_word.data, WL55_cfg.fsk_cfg.sync_word_len) == 0u))
            {
                SID_RADIO_LOG_DEBUG("Set FSK sync word - skipped, no changes");
                err = SID_RADIO_ERROR_NONE;
                break;
            }

            /* Apply immediately if the modem is currently in FSK mode */
            if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == current_modem_mode)
            {
                err = _sid_radio_subghz_set_gfsk_sync_word(request->fsk_sync_word.data, request->fsk_sync_word.data_len);
                if (err != SID_RADIO_ERROR_NONE)
                {
                    break;
                }
            }

            /* Everything is fine, store the update */
            SID_STM32_UTIL_fast_memcpy(WL55_cfg.fsk_cfg.sync_word, request->fsk_sync_word.data, request->fsk_sync_word.data_len);
            WL55_cfg.fsk_cfg.sync_word_len = request->fsk_sync_word.data_len;
            SID_PAL_LOG_DEBUG("FSK sync word set to 0x%02X%02X%02X, len: %u", WL55_cfg.fsk_cfg.sync_word[0], WL55_cfg.fsk_cfg.sync_word[1], WL55_cfg.fsk_cfg.sync_word[2], WL55_cfg.fsk_cfg.sync_word_len);
            err = SID_RADIO_ERROR_NONE;
        }
        else if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA == request->modem)
        {
            if (request->lora_sync_word == WL55_cfg.lora_cfg.sync_word)
            {
                SID_RADIO_LOG_DEBUG("Set LoRa sync word - skipped, no changes");
                err = SID_RADIO_ERROR_NONE;
                break;
            }

            /* Apply immediately if the modem is currently in LoRa mode */
            if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA == current_modem_mode)
            {
                err = _sid_radio_subghz_set_lora_sync_word(request->lora_sync_word);
                if (err != SID_RADIO_ERROR_NONE)
                {
                    break;
                }
            }

            /* Everything is fine, store the update */
            WL55_cfg.lora_cfg.sync_word = request->lora_sync_word;
            SID_PAL_LOG_DEBUG("LoRa sync word set to 0x%04X", request->lora_sync_word);
            err = SID_RADIO_ERROR_NONE;
        }
        else
        {
            SID_PAL_LOG_WARNING("Invalid modem specified for Set Sync Word cmd: %u", request->modem);
            err = SID_RADIO_ERROR_UNEXPECTED_DATA;
            break;
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Set LoRa symbol timeout.
 *
 *  @param[in] request Request payload from the host MCU
 *  @retval Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_lora_symbol_timeout(const stm32wlxx_rcp_set_lora_symb_timeout_req_t * const request)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to set LoRa symbol timeout before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        if (request->symbol_timeout == WL55_cfg.lora_cfg.symbol_timeout)
        {
            SID_RADIO_LOG_DEBUG("Set LoRa symbol timeout - skipped, no changes");
            err = SID_RADIO_ERROR_NONE;
            break;
        }

        /* This drvier uses software timer to control Tx/Rx timeouts to avoid the radio falling back to STDBY_RC state on timeout. STDBY_XOSC is used instead */

        /* Everything is fine, store the update */
        WL55_cfg.lora_cfg.symbol_timeout = request->symbol_timeout;
        SID_RADIO_LOG_DEBUG("LoRa symbol timeout set to %u", request->symbol_timeout);
        err = SID_RADIO_ERROR_NONE;
    } while (0);

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply LoRa symbol timeout setting. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Calculates the lora symbol timeout in us.
 *
 * In the case of an error, 0 is returned.
 *
 *  @param[in] mod_params Current modulation parameters that phy uses. If null, zero will be returned.
 *  @param[in] number_of_symbol Input number of symbol .
 *  @return lora symbol timeout in us
 *
 */
SID_STM32_SPEED_OPTIMIZED static inline uint32_t _sid_radio_get_lora_symbol_timeout_us(const uint32_t number_of_symbol)
{
    uint32_t symbol_timeout;

    if (valid_subghz_cfg_received != FALSE)
    {
        assert_param(STM32WLxx_LORA_BW_125 == 0x04u);
        assert_param(WL55_cfg.lora_cfg.mod.bandwidth >= STM32WLxx_LORA_BW_125);

        assert_param(STM32WLxx_LORA_SF12 == 0x0Cu);
        assert_param(WL55_cfg.lora_cfg.mod.spreading_factor <= STM32WLxx_LORA_SF12);

        const uint32_t bw_idx = WL55_cfg.lora_cfg.mod.bandwidth - STM32WLxx_LORA_BW_125;
        const uint32_t sf_idx = STM32WLxx_LORA_SF12 - WL55_cfg.lora_cfg.mod.spreading_factor;

        assert_param(bw_idx < SID_STM32_UTIL_ARRAY_SIZE(radio_lora_symb_time));
        assert_param(sf_idx < SID_STM32_UTIL_ARRAY_SIZE(radio_lora_symb_time[0]));

        /* Compute generic symbol duration */
        symbol_timeout = radio_lora_symb_time[bw_idx][sf_idx] * number_of_symbol;
    }
    else
    {
        symbol_timeout = 0u;
    }

    return symbol_timeout;
}

/*----------------------------------------------------------------------------*/

/** @brief Set LoRa modulation parameters.
 *
 *  @retval Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_set_lora_mod_params_req(const stm32wlxx_rcp_set_lora_mod_params_req_t * const request)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to set LoRa modulation params before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        if (SID_STM32_UTIL_fast_memcmp(&WL55_cfg.lora_cfg.mod, &request->mod_params, sizeof(WL55_cfg.lora_cfg.mod)) == 0u)
        {
            SID_RADIO_LOG_DEBUG("Set LoRa modulation params - skipped, no changes");
            err = SID_RADIO_ERROR_NONE;
            break;
        }

        err = _sid_radio_subghz_set_lora_mod_params(&request->mod_params);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Everything is fine, store the update */
        SID_STM32_UTIL_fast_memcpy(&WL55_cfg.lora_cfg.mod, &request->mod_params, sizeof(WL55_cfg.lora_cfg.mod));
        mod_params_upd_indication_pending = TRUE; /* Schedule logging for a later timer */
        err = SID_RADIO_ERROR_NONE;
    } while (0);

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply LoRa modulation params. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Set LoRa packet parameters.
 *
 *  @param[in] packet_params pointer to Sidewalk LoRa packet params.
 *  @retval    Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_lora_packet_params(const stm32wlxx_rcp_set_lora_pkt_params_req_t * const request)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to set LoRa packet params before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        if (SID_STM32_UTIL_fast_memcmp(&WL55_cfg.lora_cfg.pkt, &request->packet_params, sizeof(WL55_cfg.lora_cfg.pkt)) == 0u)
        {
            SID_RADIO_LOG_DEBUG("Set LoRa packet params - skipped, no changes");
            err = SID_RADIO_ERROR_NONE;
            break;
        }

        err = _sid_radio_subghz_set_lora_pkt_params(&request->packet_params);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Everything is fine, store the update */
        SID_STM32_UTIL_fast_memcpy(&WL55_cfg.lora_cfg.pkt, &request->packet_params, sizeof(WL55_cfg.lora_cfg.pkt));
        SID_RADIO_LOG_DEBUG("LoRa pkt params updated. crc_mod:0x%02X hdr_typ:0x%02X inv_iq:%u pload_len:%u preamb_len%u",
                            request->packet_params.crc_mode, request->packet_params.header_type, request->packet_params.invert_IQ,
                            request->packet_params.payload_length, request->packet_params.preamble_length);
        err = SID_RADIO_ERROR_NONE;
    } while (0);

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply LoRa packet params. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Set LoRa CAD parameters.
 *
 *  @param[in] cad_params pointer to Sidewalk CAD params.
 *  @retval   Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_set_lora_cad_params(void)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    stm32wlxx_lora_cad_params_t lora_cad_params;

    do
    {
        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to set LoRa CAD params before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        lora_cad_params.cad_symb_nb = WL55_cfg.lora_cfg.cad_symbol_num;

        lora_cad_params.cad_det_peak = WL55_cfg.lora_cfg.cad_detect_peak;
        lora_cad_params.cad_det_min = WL55_cfg.lora_cfg.cad_detect_min;

        switch (WL55_cfg.lora_cfg.cad_exit_mode)
        {
            case STM32WLxx_RADIO_CAD_EXIT_MODE_CS_ONLY:
                lora_cad_params.cad_exit_mode = STM32WLxx_LORA_CAD_ONLY;
                err = SID_RADIO_ERROR_NONE;
                break;

            case STM32WLxx_RADIO_CAD_EXIT_MODE_CS_RX:
                lora_cad_params.cad_exit_mode = STM32WLxx_LORA_CAD_RX;
                err = SID_RADIO_ERROR_NONE;
                break;

            case STM32WLxx_RADIO_CAD_EXIT_MODE_CS_LBT:
                lora_cad_params.cad_exit_mode = STM32WLxx_LORA_CAD_LBT;
                err = SID_RADIO_ERROR_NONE;
                break;

            default:
                err = SID_RADIO_ERROR_UNEXPECTED_DATA;
                break;
        }

        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        WL55_cfg.drv_cfg.cad_exit_mode = WL55_cfg.lora_cfg.cad_exit_mode;
        lora_cad_params.cad_timeout = WL55_cfg.lora_cfg.cad_timeout;

        err = _sid_radio_subghz_set_cad_params(&lora_cad_params);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply LoRa CAD params. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Start packet transmission
 *
 *  Starts the packet transmission. This needs to be invoked after all
 *  the radio configuration viz modulation params, packet params, freq,
 *  power, payload and payload length are set atleast once.
 *  The radio should be able to transmit the packet within the timeout
 *  specfied through this API. If it fails to transmit the packet within
 *  the stipulated timeout value, the radio driver should generate a
 *  interrupt with tx timeout as the reason
 *
 *  @param [in] timeout for transmission
 *  @retval     Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_start_tx(const uint32_t timeout_us)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        /* Configure antenna switch */
        err = _sid_radio_subghz_set_radio_mode(STM32WLxx_RADIO_FRONTEND_MODE_TX);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        current_radio_state = SID_PAL_RADIO_TX;
        __COMPILER_BARRIER();

        err = _sid_radio_subghz_set_tx(SID_RADIO_RXTX_NO_TIMEOUT_VAL); /* We use a software timer to avoid the radio falling back to STDBY_RC mode on Tx timeout */
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Something went wrong, we don't know if the radio has actually started Tx or remained in Standby */
            current_radio_state = SID_PAL_RADIO_UNKNOWN;
            break;
        }
#if SID_HOST_COMM_PROCESSING_DELAY_PROFILING
        /* Indicate Tx start immediately for more precise results */
        SID_PDP_HOST_COMM_ACTIVITY_GPIO_Port->BRR = SID_PDP_HOST_COMM_ACTIVITY_Pin;
#endif /* SID_HOST_COMM_PROCESSING_DELAY_PROFILING */

        /* Start the software timer if needed */
        err = _sid_radio_start_radio_timeout_mon(timeout_us);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("Failed to start Tx timeout timer. Error %d", err);
            break;
        }
        __COMPILER_BARRIER();

        err = SID_RADIO_ERROR_NONE;
        SID_RADIO_LOG_DEBUG("Started Tx with timeout %uus", timeout_us);
    } while (0);

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to start SubGHz Tx. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Set the radio in receive mode
 *
 *  @param[in]  params: timeout in microseconds for how long radio is in receive mode.
 *  @retval  true in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_start_rx(const stm32wlxx_rcp_start_rx_params_req_t * const params)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    uint32_t timeout = params->timeout_us;

    do
    {
        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to start Rx before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Ensure CS/CAD exit mode is clear */
        WL55_cfg.drv_cfg.cad_exit_mode = STM32WLxx_RADIO_CAD_EXIT_MODE_NONE;

        /* Take into account LoRa symbol timeout if it is set */
        if ((STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA == current_modem_mode) && (WL55_cfg.lora_cfg.symbol_timeout > 0u))
        {
            const uint32_t lora_sync_timeout_us = _sid_radio_get_lora_symbol_timeout_us(WL55_cfg.lora_cfg.symbol_timeout);
            if ((lora_sync_timeout_us < timeout) || (0u == timeout)) /* If symbol time is shorter than Rx timeout or Rx timeout is disabled */
            {
                timeout = lora_sync_timeout_us;
            }
        }

        if  (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == current_modem_mode)
        {
            /* Set expected Rx length to maximum as we don't known the actual length and software may not be fast enough to react on header reception */
            uint8_t expected_rx_len = STM32WLxx_RADIO_COMM_SUBGHZ_RX_MAX_SIZE;
            HAL_StatusTypeDef hal_err;

            hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, SUBGHZ_GRTXPLDLEN, &expected_rx_len, sizeof(expected_rx_len));
            if (hal_err != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", SUBGHZ_GRTXPLDLEN, hal_err);
                err = SID_RADIO_ERROR_HARDWARE;
                break;
            }
        }

        /* Configure antenna switch */
        err = _sid_radio_subghz_set_radio_mode(STM32WLxx_RADIO_FRONTEND_MODE_RX);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Go-go-go */
        current_radio_state = SID_PAL_RADIO_RX;
        __COMPILER_BARRIER();

        err = _sid_radio_subghz_set_rx(SID_RADIO_INFINITE_TIME == timeout ? SID_RADIO_RX_CONTINUOUS_VAL : SID_RADIO_RXTX_NO_TIMEOUT_VAL); /* We use a software timer to avoid the radio falling back to STDBY_RC mode on Rx timeout */
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Something went wrong, we don't know if the radio has actually started Rx or remained in Standby */
            current_radio_state = SID_PAL_RADIO_UNKNOWN;
            break;
        }
#if SID_HOST_COMM_PROCESSING_DELAY_PROFILING
        /* Indicate Rx start immediately for more precise results */
        SID_PDP_HOST_COMM_ACTIVITY_GPIO_Port->BRR = SID_PDP_HOST_COMM_ACTIVITY_Pin;
#endif /* SID_HOST_COMM_PROCESSING_DELAY_PROFILING */

        /* Start the software timer if needed */
        err = _sid_radio_start_radio_timeout_mon(timeout);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        err = SID_RADIO_ERROR_NONE;
    } while(0);

    /* As the timings are not critical anymore provide some postponed logs */
    _sid_radio_log_mod_params_change();

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to start SubGHz Rx. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int32_t _sid_radio_get_adjusted_rssi(const int32_t raw_rrsi)
{
    register int32_t adjusted_rssi = raw_rrsi;

    if (valid_subghz_cfg_received != FALSE)
    {
        adjusted_rssi -= (int32_t)WL55_cfg.drv_cfg.lna_gain;
    }

    if (adjusted_rssi < INT8_MIN)
    {
        adjusted_rssi = INT8_MIN;
    }

    return adjusted_rssi;
}

/*----------------------------------------------------------------------------*/

/** @brief Get RSSI at radio's current configured frequency
 *
 *  The frequency on which RSSI needs to be measured needs to be set before
 *  calling this API.
 *
 *  @retval  signed integer indicating RSSI in dBm
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_get_rssi(int8_t * const out_rssi)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;
    uint8_t           rssi_reg;
    int32_t           rssi;

    assert_param(out_rssi != NULL);

    hal_err = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_RSSIINST, &rssi_reg, sizeof(rssi_reg));

    if (HAL_OK == hal_err)
    {
        /* Compute physical value from the raw register value */
        rssi = ((-(int32_t)rssi_reg) >> 1);

        /* Adjust by LNA gain and Rx boost gain */
        rssi = _sid_radio_get_adjusted_rssi(rssi);

        *out_rssi = (int8_t)rssi;

        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to access SubGHz. HAL_SUBGHZ err: 0x%x", hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Radio irq processing.
 *
 *  The function reads the irq status register and reports radio event to the
 *  phy layer through the callback routine registered as part of sid_pal_radio_init.
 *  The protocol after being notified on receiving a radio interrupt switches
 *  from hardware isr to software isr context to continue with bottom half
 *  processing of the radio interrupt.
 *  _sid_radio_irq_process should determine the cause of interrupt and notify the
 *  protocol of the phy event through the event handler registered as part of
 *  sid_pal_radio_init.
 *  On packet reception, this API has to copy the received packet from radio
 *  buffers to the rx packet registered as part of sid_pal_radio_init
 *
 *  @retval  true in case of error
 */
SID_STM32_SPEED_OPTIMIZED static sid_radio_error_t _sid_radio_irq_process(const uint32_t reported_irqs, const uint32_t irq_timestamp_s, const uint32_t irq_timestamp_us)
{
    sid_radio_error_t                      err                  = SID_RADIO_ERROR_GENERIC;
    stm32wlxx_pal_radio_events_t           radio_event          = STM32WLxx_PAL_RADIO_EVENT_UNKNOWN;
    uint32_t                               remaining_irqs       = reported_irqs;
    stm32wlxx_radio_comm_spi_frame_t       spi_reply_frame;
    stm32wlxx_rcp_irq_status_t * const     irq_status_reply     = &spi_reply_frame.payload.irq_status;
    stm32wlxx_subghz_irq_details_t * const irq_details          = &irq_status_reply->subghz_irq_details;
    uint32_t                               rx_done_received_len = 0u;

    /* Set default follow-up data length to zero */
    irq_status_reply->followup_payload_len = 0u;

    do
    {
        if ((SID_PAL_RADIO_SLEEP == current_radio_state) || (SID_PAL_RADIO_STANDBY == current_radio_state))
        {
            /* This may happen if radio IRQ was pre-empted by timeout monitoring software timer */
            SID_RADIO_LOG_DEBUG("Radio IRQ detected while SubGHz is in Sleep/Standby - ignored");
            remaining_irqs = STM32WLxx_SUBGHZ_IRQ_NONE;
            err = SID_RADIO_ERROR_NONE;
            break;
        }

        /* Handle Tx Done IRQ --------------------------------------------------------*/
        if ((reported_irqs & STM32WLxx_SUBGHZ_IRQ_TX_DONE) != STM32WLxx_SUBGHZ_IRQ_NONE)
        {
            SID_RADIO_LOG_DEBUG("STM32WLxx_SUBGHZ_IRQ_TX_DONE");

            /* Radio automatically enters Standby state after Tx/Rx completion or error (e.g. timeout) */
            current_radio_state = SID_PAL_RADIO_STANDBY;
            /* Turn off FEM as we don't need it any longer */
            (void)_sid_radio_subghz_set_radio_mode(STM32WLxx_RADIO_FRONTEND_MODE_OFF);

            /* Tx Done IRQ is processed successfully */
            remaining_irqs &= ~STM32WLxx_SUBGHZ_IRQ_TX_DONE;
            radio_event = STM32WLxx_PAL_RADIO_EVENT_TX_DONE;
            err = SID_RADIO_ERROR_NONE;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Sync Word Valid IRQ -------------------------------------------------------*/
        if ((reported_irqs & STM32WLxx_SUBGHZ_IRQ_SYNC_WORD_VALID) != STM32WLxx_SUBGHZ_IRQ_NONE)
        {
            SID_RADIO_LOG_DEBUG("STM32WLxx_SUBGHZ_IRQ_SYNC_WORD_VALID");

            const uint32_t rx_done = (reported_irqs & STM32WLxx_SUBGHZ_IRQ_RX_DONE) != STM32WLxx_SUBGHZ_IRQ_NONE ? TRUE : FALSE;

            if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == current_modem_mode)
            {
                err = _sid_radio_fsk_process_sync_word_detected(radio_irq_followup_frame.subghz_rx_buf, rx_done);
                if (err != SID_RADIO_ERROR_NONE)
                {
                    /* Logs provided by _sid_radio_fsk_process_sync_word_detected() */
                    break;
                }
            }
            else
            {
                /* No actions required for LoRa */
                err = SID_RADIO_ERROR_NONE;
            }

            /* Sync Word Valid IRQ is processed successfully */
            remaining_irqs &= ~STM32WLxx_SUBGHZ_IRQ_SYNC_WORD_VALID;

            /* Terminate IRQ processing if no more IRQs are indicated, otherwise proceed (e.g. with Preamble Detected, Rx Done, etc.) */
            if (STM32WLxx_SUBGHZ_IRQ_NONE == remaining_irqs)
            {
                break;
            }
        }
        /*----------------------------------------------------------------------------*/

        /* Preamble Detect IRQ -------------------------------------------------------*/
        if ((reported_irqs & STM32WLxx_SUBGHZ_IRQ_PBL_DET) != STM32WLxx_SUBGHZ_IRQ_NONE)
        {
            SID_RADIO_LOG_DEBUG("STM32WLxx_SUBGHZ_IRQ_PBL_DET");

            if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == current_modem_mode)
            {
                const stm32wlxx_pal_radio_cad_param_exit_mode_t cad_exit_mode_on_entry = WL55_cfg.drv_cfg.cad_exit_mode;

                if (STM32WLxx_RADIO_CAD_EXIT_MODE_CS_LBT == cad_exit_mode_on_entry)
                {
                    /* Carrier sense detected a signal - collision on the channel, cannot transmit */
                    radio_event = STM32WLxx_PAL_RADIO_EVENT_CS_DONE;
                    WL55_cfg.drv_cfg.cad_exit_mode = STM32WLxx_RADIO_CAD_EXIT_MODE_NONE;

                    /* Obtain RSSI of the detected carrier */
                    err = _sid_radio_subghz_get_rssi(&irq_details->received_packet_info.fsk_rx_packet_status.rssi_sync);
                    if (err != SID_RADIO_ERROR_NONE)
                    {
                        SID_RADIO_LOG_ERROR("Failed to get SubGHz RSSI. Error %u", (uint32_t)err);
                        break;
                    }

                    /* Append RSSI debug info to the unsued MSB of the Tx payload length */
                    last_indicated_radio_event_data_len = (last_indicated_radio_event_data_len & 0x00FFFFFFu) |  ((uint32_t)irq_details->received_packet_info.fsk_rx_packet_status.rssi_sync << 24);

                    /* Put radio into Standby */
                    err = _sid_radio_subghz_standby();
                    if (err != SID_RADIO_ERROR_NONE)
                    {
                        /* Logs provided by _sid_radio_subghz_standby() */
                        break;
                    }

                    /* Restore the default IRQ mask since it was modifed for FSK Carrier Sense */
                    err = _sid_radio_enable_default_irqs();
                    if (err != SID_RADIO_ERROR_NONE)
                    {
                        /* Logs are provided by _sid_radio_enable_default_irqs() */
                        break;
                    }
                }
                else
                {
                    err = SID_RADIO_ERROR_NONE;
                }

                /* Restore the default IRQ mask for FSK */
                if (cad_exit_mode_on_entry != STM32WLxx_RADIO_CAD_EXIT_MODE_NONE)
                {
                    err = _sid_radio_enable_default_irqs();
                    if (err != SID_RADIO_ERROR_NONE)
                    {
                        /* Logs are provided by _sid_radio_enable_default_irqs() */
                        break;
                    }
                }
            }
            else
            {
                /* This IRQ is used to stop timeout timer only */
                err = SID_RADIO_ERROR_NONE;
            }

            /* Preamble Detect IRQ is processed successfully */
            remaining_irqs &= ~STM32WLxx_SUBGHZ_IRQ_PBL_DET;

            /* Terminate IRQ processing if no more IRQs are indicated, otherwise proceed (e.g. with Rx Done, etc.) */
            if (STM32WLxx_SUBGHZ_IRQ_NONE == remaining_irqs)
            {
                break;
            }
        }
        /*----------------------------------------------------------------------------*/

        /* CAD Done IRQ --------------------------------------------------------------*/
        if ((reported_irqs & STM32WLxx_SUBGHZ_IRQ_CAD_DONE) != STM32WLxx_SUBGHZ_IRQ_NONE)
        {
            SID_RADIO_LOG_DEBUG("STM32WLxx_SUBGHZ_IRQ_CAD_DONE");

            if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA == current_modem_mode)
            {
#if HALO_ENABLE_DIAGNOSTICS
                radio_event = ((reported_irqs & STM32WLxx_SUBGHZ_IRQ_CAD_DET) != STM32WLxx_SUBGHZ_IRQ_NONE) ? STM32WLxx_PAL_RADIO_EVENT_CAD_DONE : STM32WLxx_PAL_RADIO_EVENT_CAD_TIMEOUT;

                if ((STM32WLxx_RADIO_CAD_EXIT_MODE_CS_LBT == WL55_cfg.drv_cfg.cad_exit_mode) && (STM32WLxx_PAL_RADIO_EVENT_CAD_TIMEOUT == radio_event))
                {
                    /* Radio starts Tx automatically in CAD-LBT mode */
                    radio_event = STM32WLxx_PAL_RADIO_EVENT_UNKNOWN;
                    current_radio_state = SID_PAL_RADIO_TX;

                }
                if ((STM32WLxx_RADIO_CAD_EXIT_MODE_CS_RX == WL55_cfg.drv_cfg.cad_exit_mode) && (STM32WLxx_PAL_RADIO_EVENT_CAD_DONE == radio_event))
                {
                    /* Radio starts Rx automatically in CAD-Rx mode */
                    radio_event = STM32WLxx_PAL_RADIO_EVENT_UNKNOWN;
                    current_radio_state = SID_PAL_RADIO_RX;
                }
                else
                {
                    /* For the other modes report CAD Done/ CAD Timeout event */
                }

                WL55_cfg.drv_cfg.cad_exit_mode = STM32WLxx_RADIO_CAD_EXIT_MODE_NONE;
#endif /* HALO_ENABLE_DIAGNOSTICS */
                err = SID_RADIO_ERROR_NONE;
            }
            else
            {
                SID_RADIO_LOG_ERROR("CAD Done IRQ is expected only for LoRa mode, but modem is in %u mode", (uint32_t)current_modem_mode);
                err = SID_RADIO_ERROR_INVALID_STATE;
                break;
            }

            /* CAD Done IRQ is processed successfully */
            remaining_irqs &= ~(STM32WLxx_SUBGHZ_IRQ_CAD_DONE | STM32WLxx_SUBGHZ_IRQ_CAD_DET);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* CRC Error IRQ -------------------------------------------------------------*/
        if ((reported_irqs & STM32WLxx_SUBGHZ_IRQ_CRC_ERROR) != STM32WLxx_SUBGHZ_IRQ_NONE)
        {
            SID_RADIO_LOG_DEBUG("STM32WLxx_SUBGHZ_IRQ_CRC_ERROR");
            err = SID_RADIO_ERROR_NONE;
            radio_event = STM32WLxx_PAL_RADIO_EVENT_RX_ERROR;
            remaining_irqs &= ~(STM32WLxx_SUBGHZ_IRQ_CRC_ERROR | STM32WLxx_SUBGHZ_IRQ_RX_DONE);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Header Error IRQ ----------------------------------------------------------*/
        if ((reported_irqs & STM32WLxx_SUBGHZ_IRQ_HEADER_ERROR) != STM32WLxx_SUBGHZ_IRQ_NONE)
        {
            SID_RADIO_LOG_DEBUG("STM32WLxx_SUBGHZ_IRQ_HEADER_ERROR");
            err = SID_RADIO_ERROR_NONE;
            radio_event = STM32WLxx_PAL_RADIO_EVENT_HEADER_ERROR;
            remaining_irqs &= ~STM32WLxx_SUBGHZ_IRQ_HEADER_ERROR;
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Rx Done IRQ ---------------------------------------------------------------*/
        if ((reported_irqs & STM32WLxx_SUBGHZ_IRQ_RX_DONE) != STM32WLxx_SUBGHZ_IRQ_NONE)
        {
            /* There's no need to check for CRC and other errors here since all error IRQs are handled before we get here */
            SID_RADIO_LOG_DEBUG("STM32WLxx_SUBGHZ_IRQ_RX_DONE");

            /* Radio automatically enters Standby state after Tx/Rx completion or error (e.g. timeout) */
            current_radio_state = SID_PAL_RADIO_STANDBY;
            /* Turn off FEM as we don't need it any longer */
            (void)_sid_radio_subghz_set_radio_mode(STM32WLxx_RADIO_FRONTEND_MODE_OFF);

            /* Process FSK first as it's more time sensitive */
            if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == current_modem_mode)
            {
                stm32wlxx_radio_fsk_rx_done_status_t fsk_rx_done_status;

                err = _sid_radio_fsk_process_rx_done(&irq_details->received_packet_info.fsk_rx_packet_status, radio_irq_followup_frame.subghz_rx_buf,
                                                     sizeof(radio_irq_followup_frame.subghz_rx_buf), &rx_done_received_len, &fsk_rx_done_status);
                if (err != SID_RADIO_ERROR_NONE)
                {
                    SID_RADIO_LOG_DEBUG("FSK Rx completed with error. Error: %u, rx status: %u", (uint32_t)err, (uint32_t)fsk_rx_done_status);
                    radio_event = STM32WLxx_PAL_RADIO_EVENT_RX_ERROR;
                    last_indicated_radio_event_data_len = 0u;
                    err = SID_RADIO_ERROR_NONE;
                }
                else
                {
                    radio_event = STM32WLxx_PAL_RADIO_EVENT_RX_DONE;
                    last_indicated_radio_event_data_len = rx_done_received_len;
                    radio_irq_followup_frame.opcode = STM32WLxx_RADIO_COMM_OPCODE_RAW_DATA;

                    /* Calculate the actual length of the follow-up data */
                    irq_status_reply->followup_payload_len = (sizeof(radio_irq_followup_frame) - sizeof(radio_irq_followup_frame.subghz_rx_buf)) + rx_done_received_len;
                }
            }
            else if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA == current_modem_mode)
            {
                err = _sid_radio_lora_process_rx_done(&irq_details->received_packet_info.lora_rx_packet_status, radio_irq_followup_frame.subghz_rx_buf,
                                                      sizeof(radio_irq_followup_frame.subghz_rx_buf), &rx_done_received_len);
                if (err != SID_RADIO_ERROR_NONE)
                {
                    SID_RADIO_LOG_DEBUG("LoRa Rx completed with error %u", (uint32_t)err);
                    radio_event = STM32WLxx_PAL_RADIO_EVENT_RX_ERROR;
                    last_indicated_radio_event_data_len = 0u;
                    err = SID_RADIO_ERROR_NONE;
                }
                else
                {
                    radio_event = STM32WLxx_PAL_RADIO_EVENT_RX_DONE;
                    last_indicated_radio_event_data_len = rx_done_received_len;
                    radio_irq_followup_frame.opcode = STM32WLxx_RADIO_COMM_OPCODE_RAW_DATA;

                    /* Calculate the actual length of the follow-up data */
                    irq_status_reply->followup_payload_len = (sizeof(radio_irq_followup_frame) - sizeof(radio_irq_followup_frame.subghz_rx_buf)) + rx_done_received_len;
                }
            }
            else
            {
                SID_PAL_LOG_ERROR("Received SubGHz Rx Done IRQ, but active modem mode (%u) is invalid", (uint32_t)current_modem_mode);
                err = SID_RADIO_ERROR_INVALID_STATE;
                break;
            }

            /* Ensure the indicated received length is valid */
            if ((STM32WLxx_PAL_RADIO_EVENT_RX_DONE == radio_event) && (0u == rx_done_received_len))
            {
                SID_RADIO_LOG_DEBUG("Rx completed with zero received length");
                radio_event = STM32WLxx_PAL_RADIO_EVENT_RX_ERROR;
                last_indicated_radio_event_data_len = 0u;
                irq_status_reply->followup_payload_len = 0u;
                err = SID_RADIO_ERROR_NONE;
            }

            /* Rx Done IRQ is processed successfully */
            remaining_irqs &= ~STM32WLxx_SUBGHZ_IRQ_RX_DONE;
            break;
        }

        /* If we've got here it means none of the above blocks took care of the IRQ. Not an error, but... */
        SID_PAL_LOG_WARNING("Unexpected SubGHz IRQ detected: 0x%04X. No handlers assigned to it. All reported: 0x%04X", remaining_irqs, reported_irqs);
        err = SID_RADIO_ERROR_NONE;
    } while(0);
    /*----------------------------------------------------------------------------*/

    /* Indicate the IRQ to the host MCU if needed --------------------------------*/
    if (SID_RADIO_ERROR_NONE == err)
    {
        if (remaining_irqs != STM32WLxx_SUBGHZ_IRQ_NONE)
        {
            SID_PAL_LOG_WARNING("Unhandled SubGHz IRQs identified. Reported: 0x%04x, unhandled: 0x%04x", reported_irqs, remaining_irqs);
        }

        last_indicated_radio_event = radio_event;

        if (STM32WLxx_PAL_RADIO_EVENT_UNKNOWN != radio_event)
        {
            /* Report the event to the host MCU */
            err = _sid_radio_report_event(radio_event, &spi_reply_frame, irq_timestamp_s, irq_timestamp_us);
        }
        else
        {
            SID_RADIO_LOG_DEBUG("SubGHz IRQ report to host MCU skipped");
        }
    }
    else
    {
        SID_RADIO_LOG_ERROR("SubGHz IRQ processing failed. Error %u, irqs: 0x%x, unhandled: 0x%x", (uint32_t)err, reported_irqs, remaining_irqs);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_radio_error_t _sid_radio_report_event(const stm32wlxx_pal_radio_events_t radio_event, stm32wlxx_radio_comm_spi_frame_t * const spi_reply_frame, const uint32_t irq_timestamp_s, const uint32_t irq_timestamp_us)
{
    sid_radio_error_t                      err                  = SID_RADIO_ERROR_GENERIC;
    stm32wlxx_rcp_irq_status_t * const     irq_status_reply     = &spi_reply_frame->payload.irq_status;
    stm32wlxx_subghz_irq_details_t * const irq_details          = &irq_status_reply->subghz_irq_details;
    sid_host_comm_error_t hc_err;

    do
    {
        /* Report the event to the host MCU */
        spi_reply_frame->opcode      = STM32WLxx_RADIO_COMM_OPCODE_IRQ_STATUS;
        irq_status_reply->irq_flags  = STM32WLxx_APP_IRQ_SUBGHZ;

        irq_details->radio_event     = radio_event;
        irq_details->cad_exit_mode   = WL55_cfg.drv_cfg.cad_exit_mode;
        irq_details->radio_state     = current_radio_state;

        /* Store the location of the SPI Tx buffer to put timestamp to it later */
        stm32wlxx_radio_comm_spi_frame_t * const reply_frame_in_fifo = (stm32wlxx_radio_comm_spi_frame_t *)serial_bus_spi_get_current_enqueue_ptr();

        hc_err = sid_host_comm_enqueue_tx((uint8_t *)(void *)spi_reply_frame, sizeof(*spi_reply_frame));
        if (hc_err != SID_HOST_COMM_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to enqueue SubGHz IRQ status SPI frame. HC error %u", (uint32_t)hc_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        if (irq_status_reply->followup_payload_len != 0u)
        {
            hc_err = sid_host_comm_enqueue_tx((uint8_t *)(void *)&radio_irq_followup_frame, irq_status_reply->followup_payload_len);
            if (hc_err != SID_HOST_COMM_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to enqueue SubGHz IRQ follow-up data. HC error %u", (uint32_t)hc_err);
                err = SID_RADIO_ERROR_HARDWARE;
                break;
            }
        }

        /* Disable all interrupts to calculate IRQ processing time */
        const register uint32_t primask_bit = __get_PRIMASK();
        __disable_irq();
        __COMPILER_BARRIER();

        /* Compute the elapsed time */
        uint32_t now_s, now_us;
        now_s = TIMER_IF_GetTimeUs(&now_us);
        const uint32_t elapsed_time_us = ((now_s - irq_timestamp_s /* Intentional underflow */) * 1000000u) + (now_us - irq_timestamp_us /* Intentional underflow */);

        assert_param(STM32WLxx_RADIO_COMM_OPCODE_IRQ_STATUS == reply_frame_in_fifo->opcode);
        assert_param(STM32WLxx_APP_IRQ_SUBGHZ               == reply_frame_in_fifo->payload.irq_status.irq_flags);
        assert_param(radio_event                            == reply_frame_in_fifo->payload.irq_status.subghz_irq_details.radio_event);

        /* Convert elapsed time to ns and write directlyto the SPI buffer */
        reply_frame_in_fifo->payload.irq_status.subghz_irq_details.compensatory_ns = (elapsed_time_us * 1000u) + SID_RADIO_IRQ_PROCESSING_DELAY_STATIC_ERROR_NS;
        __COMPILER_BARRIER();

        /* If everything is ok indicate the IRQ to the host MCU - this will start SPI transfers */
        SID_HOST_COMM_IRQ_DISABLE_TRIGGER(); /* Avoid self-triggering of the Handshake IRQ */
        __COMPILER_BARRIER();
        SID_HOST_COMM_IRQ_INDICATE_EVENT();

        /* Restore interrupt settings */
        __set_PRIMASK(primask_bit);
        __COMPILER_BARRIER();

#if SID_HOST_COMM_PROCESSING_DELAY_PROFILING
        /* Clear the actual radio IRQ event indication for profiling after the radio IRQ is processed */
        SID_PDP_RADIO_IRQ_ACTIVITY_GPIO_Port->BRR = SID_PDP_RADIO_IRQ_ACTIVITY_Pin;
#endif /* SID_HOST_COMM_PROCESSING_DELAY_PROFILING */

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Set fsk modulation parameters.
 *
 *  @retval Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_set_fsk_mod_params_req(const stm32wlxx_rcp_set_fsk_mod_params_req_t * const request)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to FSK modulation params before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        if (SID_STM32_UTIL_fast_memcmp(&WL55_cfg.fsk_cfg.mod, &request->mod_params, sizeof(WL55_cfg.fsk_cfg.mod)) == 0u)
        {
            SID_RADIO_LOG_DEBUG("Set FSK modulation params - skipped, no changes");
            err = SID_RADIO_ERROR_NONE;
            break;
        }

        err = _sid_radio_subghz_set_gfsk_mod_params(&request->mod_params);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Everything is fine, store the update */
        SID_STM32_UTIL_fast_memcpy(&WL55_cfg.fsk_cfg.mod, &request->mod_params, sizeof(WL55_cfg.fsk_cfg.mod));
        mod_params_upd_indication_pending = TRUE; /* Schedule logs for later time */
        err = SID_RADIO_ERROR_NONE;
    } while (0);

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply FSK modulation params. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Set LoRa packet parameters.
 *
 *  @param[in] packet_params pointer to Sidewalk FSK packet params.
 *  @retval    Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_set_fsk_packet_params_req(const stm32wlxx_rcp_set_fsk_pkt_params_req_t * const request)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to set FSK packet params before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        if (SID_STM32_UTIL_fast_memcmp(&WL55_cfg.fsk_cfg.pkt, &request->packet_params, sizeof(WL55_cfg.fsk_cfg.pkt)) == 0u)
        {
            SID_RADIO_LOG_DEBUG("Set FSK packet params - skipped, no changes");
            err = SID_RADIO_ERROR_NONE;
            break;
        }

        err = _sid_radio_subghz_set_gfsk_pkt_params(&request->packet_params);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Everything is fine, store the update */
        SID_STM32_UTIL_fast_memcpy(&WL55_cfg.fsk_cfg.pkt, &request->packet_params, sizeof(WL55_cfg.fsk_cfg.pkt));
        SID_RADIO_LOG_DEBUG("FSK pkt params updated. crc_t:0x%02X hdr_typ:0x%02X sw:%u wht:0x%02X pload:%u preamb_len%u pream_min_det:%u",
                            request->packet_params.crc_type, request->packet_params.header_type, request->packet_params.sync_word_length,
                            request->packet_params.radio_whitening_mode, request->packet_params.payload_length,
                            request->packet_params.preamble_length, request->packet_params.preamble_min_detect);
        err = SID_RADIO_ERROR_NONE;
    } while (0);

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to apply FSK packet params. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

/** @brief Set the radio in preamble detect mode
 *
 *  @retval Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_start_fsk_carrier_sense(void)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        if (current_modem_mode != STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK)
        {
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to start FSK carrier sense before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Configure FEM for Rx */
        err = _sid_radio_subghz_set_radio_mode(STM32WLxx_RADIO_FRONTEND_MODE_RX);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Enable IRQs relevant to CS */
        err = _sid_radio_set_irq_mask(STM32WLxx_RADIO_COMM_FSK_CARRIER_SENSE_IRQ_MASK);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        /* Update status in the driver */
        current_radio_state = SID_PAL_RADIO_RX;
        WL55_cfg.drv_cfg.cad_exit_mode = STM32WLxx_RADIO_CAD_EXIT_MODE_CS_LBT;
        __COMPILER_BARRIER();

        /* Start Rx to check for radio channel availability */
        err = _sid_radio_subghz_set_rx(SID_RADIO_RXTX_NO_TIMEOUT_VAL); /* We use a software timer to avoid the radio falling back to STDBY_RC mode on Rx timeout */
        if (err != SID_RADIO_ERROR_NONE)
        {
            current_radio_state = SID_PAL_RADIO_UNKNOWN;
            break;
        }

        /* Start the software timer for FSK if needed */
        err = _sid_radio_start_radio_timeout_mon(WL55_cfg.fsk_cfg.fsk_cs_duration_us);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        err = SID_RADIO_ERROR_NONE;
     } while(0);

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to start Carrier Sense. Error %u", (uint32_t)err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_user_data_upload(const uint8_t * const frame_data, const uint32_t available_frame_length, const uint32_t full_frame_length)
{
    /* Check for systematic failures */
    assert_param(frame_data != NULL);
    const stm32wlxx_rcp_user_data_t * const user_data_frame = (stm32wlxx_rcp_user_data_t *)(void *)frame_data;
#if DEBUG
    uint32_t calculated_full_frame_length = user_data_frame->data_length + sizeof(user_data_frame->opcode) + sizeof(user_data_frame->data_length);
    if (calculated_full_frame_length < sizeof(stm32wlxx_radio_comm_spi_frame_t))
    {
        calculated_full_frame_length = sizeof(stm32wlxx_radio_comm_spi_frame_t);
    }
    assert_param(full_frame_length == calculated_full_frame_length);
#endif

    if (available_frame_length == full_frame_length)
    {
        sid_host_comm_on_incoming_user_data cb_func;

        /* All the user data has arrived, trigger the user callback */
        SID_RADIO_LOG_DEBUG("Received %u bytes of user data", user_data_frame->data_length);

        /* Make a local copy of the callback pointer */
        UTILS_ENTER_CRITICAL_SECTION();
        cb_func = udt_ctx.on_user_data_rx;
        UTILS_EXIT_CRITICAL_SECTION();

        if (cb_func != NULL)
        {
            /* Invoke the user callback */
            cb_func(user_data_frame->data, user_data_frame->data_length);
        }
    }
    else
    {
        /* Not enough data to upload, keep waiting for addiiotnal incoming data */
    }

    return SID_RADIO_ERROR_NONE;
}
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/*----------------------------------------------------------------------------*/

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
SID_STM32_SPEED_OPTIMIZED static void _user_data_transfer_thread(void * context)
{
    osStatus_t os_err;

    (void)context;

    do
    {
        assert_param(udt_ctx.outbound_msg_queue != NULL);

        /* Wait for any user app requests to send out data to the STM32WLxx side */
        sid_host_comm_out_udt_msg_desc_t msg_desc;
        os_err = osMessageQueueGet(udt_ctx.outbound_msg_queue, &msg_desc, NULL, osWaitForever);

        if (osOK == os_err)
        {
            if ((msg_desc.data_ptr != NULL) && (msg_desc.data_len > 0u))
            {
                sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

                stm32wlxx_radio_comm_spi_frame_t udt_irq_frame = {
                    .opcode = STM32WLxx_RADIO_COMM_OPCODE_IRQ_STATUS,
                    .payload = {
                        .irq_status = {
                            .irq_flags = STM32WLxx_APP_IRQ_USER_DATA,
                            .followup_payload_len =  (sizeof(udt_irq_followup_frame) - sizeof(udt_irq_followup_frame.data)) + msg_desc.data_len,
                        },
                    },
                };

                /* Prepare IRQ follow-up frame with User Data */
                udt_irq_followup_frame.opcode = STM32WLxx_RADIO_COMM_OPCODE_USER_DATA;
                SID_STM32_UTIL_fast_memcpy(udt_irq_followup_frame.data, msg_desc.data_ptr, msg_desc.data_len);

                /* Automatically free the buffer after processing is done (if requested) */
                if (msg_desc.auto_free != FALSE)
                {
                    vPortFree(msg_desc.data_ptr);
                }

                /* Ensure the radio is fully initialized before any further actions */
                {
                    UTILS_ENTER_CRITICAL_SECTION();
                    const uint32_t local_valid_subghz_cfg_received = valid_subghz_cfg_received;
                    UTILS_EXIT_CRITICAL_SECTION();

                    if (FALSE == local_valid_subghz_cfg_received)
                    {
                        /* Driver is not ready to accept user messages - wait for readiness */
                        const uint32_t event_flags = osThreadFlagsWait(SID_RADIO_UDT_DRIVER_READY_FLAG, osFlagsWaitAny, osWaitForever);
                        if ((event_flags & osFlagsError) != 0u)
                        {
                            /* Unable to send - drop the data and proceed */
                            SID_PAL_LOG_ERROR("UDT - error while waiting driver readiness. Error flags 0x%08X", event_flags);
                            continue;
                        }
                    }
                }

                /* Wait until UDT is allowed */
                {
                    UTILS_ENTER_CRITICAL_SECTION();
                    const uint32_t local_udt_enabled = udt_ctx.udt_enabled;
                    UTILS_EXIT_CRITICAL_SECTION();

                    if (FALSE == local_udt_enabled)
                    {
                        /* UDT is prohibited at the moment -wait for it to get enabled */
                        const uint32_t event_flags = osThreadFlagsWait(SID_RADIO_UDT_TRANSFER_ENABLED_FLAG, osFlagsWaitAny, osWaitForever);
                        if ((event_flags & osFlagsError) != 0u)
                        {
                            /* Unable to send - drop the data and proceed */
                            SID_PAL_LOG_ERROR("UDT - error while waiting UDT enablement. Error flags 0x%08X", event_flags);
                            continue;
                        }
                    }
                }

                UTILS_ENTER_CRITICAL_SECTION();
                do
                {
                    sid_host_comm_error_t hc_err = sid_host_comm_enqueue_tx((void *)&udt_irq_frame, sizeof(udt_irq_frame));
                    if (hc_err != SID_HOST_COMM_ERROR_NONE)
                    {
                        SID_RADIO_LOG_ERROR("Unable to enqueue User Data IRQ frame. Host Comm error %u", (uint32_t)hc_err);
                        err = SID_RADIO_ERROR_HARDWARE;
                        break;
                    }

                    hc_err = sid_host_comm_enqueue_tx((void *)&udt_irq_followup_frame, udt_irq_frame.payload.irq_status.followup_payload_len);
                    if (hc_err != SID_HOST_COMM_ERROR_NONE)
                    {
                        SID_RADIO_LOG_ERROR("Unable to enqueue User Data IRQ follow-up data. Host Comm error %u", (uint32_t)hc_err);
                        err = SID_RADIO_ERROR_HARDWARE;
                        break;
                    }

                    SID_RADIO_LOG_DEBUG("Successfully enqueued User Data IRQ frames");
                    err = SID_RADIO_ERROR_NONE;
                } while (0);

                if (SID_RADIO_ERROR_NONE == err)
                {
                    /* Set IRQ indication if everything is ok */
                    SID_HOST_COMM_IRQ_DISABLE_TRIGGER(); /* Disable EXTI on IRQ falling edge since we are going to drive the line, otherwise GPIO Handshake will be triggered */
                    __COMPILER_BARRIER();                /* Ensure EXTI trigger is deactivated before the next step */
                    SID_HOST_COMM_IRQ_INDICATE_EVENT();  /* Drive the IRQ line to indicate pendingIRQ to the host MCU */
                }
                else
                {
                    SID_PAL_LOG_ERROR("Failed to send UDT message. Error %u", (uint32_t)err);
                }
                UTILS_EXIT_CRITICAL_SECTION();
            }
            else
            {
                SID_PAL_LOG_WARNING("UDT transfer skipped - empty data provided");
            }
        }
        else
        {
            /* Some unexpected CMSIS/OS error occurred */
            SID_PAL_LOG_ERROR("Unexpected error while accessing UDT queue. OS error 0x%08x", (uint32_t)os_err);
        }
    } while (TRUE);

    /* Terminate the thread properly if the code ever gets here */
    SID_PAL_LOG_WARNING("UDT thread terminated");
    osThreadExit();
}
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/*----------------------------------------------------------------------------*/

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
SID_STM32_SPEED_OPTIMIZED static void _sid_radio_udt_suspend_timer_cb(void * context)
{
    (void)context;

    /* Radio is about to wake up, suspend UDT operations */
    udt_ctx.udt_enabled = FALSE;
}
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/*----------------------------------------------------------------------------*/

#if HALO_ENABLE_DIAGNOSTICS
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_subghz_set_cw_tx(void)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;

#if defined(DEBUG) && defined(NUCLEO_WL55_BOARD)
    /* Quick checks that antenna switch is configured properly - valid for the Nucleo board only */
    const GPIO_PinState ctrl3 = (RF_SW_CTRL3_GPIO_PORT->IDR & RF_SW_CTRL3_PIN) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;
    const GPIO_PinState ctrl1 = (RF_SW_CTRL1_GPIO_PORT->IDR & RF_SW_CTRL1_PIN) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;
    const GPIO_PinState ctrl2 = (RF_SW_CTRL2_GPIO_PORT->IDR & RF_SW_CTRL2_PIN) != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if ((ctrl3 != GPIO_PIN_SET) || (ctrl2 != GPIO_PIN_SET)) /* ctrl1 is don't care */
    {
        SID_PAL_LOG_ERROR("Wrong antenna switch config for TX: ctrl1: %u, ctrl2: %u, ctrl3: %u", ctrl1, ctrl2, ctrl3);
    }
#endif /* DEBUG && NUCLEO_WL55_BOARD */

    hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXCONTINUOUSWAVE, NULL, 0u);
    if (HAL_OK == hal_err)
    {
        err = SID_RADIO_ERROR_NONE;
    }
    else
    {
        SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_TXCONTINUOUSWAVE, hal_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }

    return err;
}
#endif /* HALO_ENABLE_DIAGNOSTICS */

/*----------------------------------------------------------------------------*/

#if HALO_ENABLE_DIAGNOSTICS
/**
 * @brief Set the radio to continuous wave transmit
 *
 * @param [in] request CW Tx request data
 * @retval Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_start_cw_tx(const stm32wlxx_rcp_start_cw_tx_params_req_t * const request)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        /* Set the desired radio frequency */
        stm32wlxx_rcp_set_frequency_req_t set_freq_req = {
            .frequency = request->frequecy,
        };
        err = _sid_radio_set_frequency(&set_freq_req);
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Logs are provided by _sid_radio_set_frequency() */
            break;
        }

        /* Configure antenna switch */
        err = _sid_radio_subghz_set_radio_mode(STM32WLxx_RADIO_FRONTEND_MODE_TX);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        current_radio_state = SID_PAL_RADIO_TX;
        __COMPILER_BARRIER();

        err = _sid_radio_subghz_set_cw_tx();
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Something went wrong, we don't know if the radio has actually started Tx or remained in Standby */
            current_radio_state = SID_PAL_RADIO_UNKNOWN;
            break;
        }

        err = SID_RADIO_ERROR_NONE;
        SID_PAL_LOG_DEBUG("Started CW Tx at %u.%uMHz, %s%ddBm", request->frequecy / 1000u, request->frequecy % 1000u, WL55_cfg.pa_cfg.target_tx_power > 0 ? "+" : "", WL55_cfg.pa_cfg.target_tx_power);
    } while (0);

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to start SubGHz CW Tx. Error %u", (uint32_t)err);
    }

    return err;
}
#endif /* HALO_ENABLE_DIAGNOSTICS */

/*----------------------------------------------------------------------------*/

#if HALO_ENABLE_DIAGNOSTICS
/**
 * @brief Set the radio to continuous receive
 *
 * @retval Non-zero value in case of error
 */
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_start_continuous_rx(void)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to start continuous Rx before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        const uint32_t pbl_det_timer = (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == current_modem_mode) ? TRUE: FALSE;

        err = _sid_radio_subghz_stop_tmr_on_pbl(pbl_det_timer);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        err = _sid_radio_subghz_set_radio_mode(STM32WLxx_RADIO_FRONTEND_MODE_RX);
        if (err != SID_RADIO_ERROR_NONE)
        {
            break;
        }

        current_radio_state = SID_PAL_RADIO_RX;
        __COMPILER_BARRIER();

        err = _sid_radio_subghz_set_rx(SID_RADIO_RX_CONTINUOUS_VAL);
        if (err != SID_RADIO_ERROR_NONE)
        {
            current_radio_state = SID_PAL_RADIO_UNKNOWN;
            break;
        }

        err = SID_RADIO_ERROR_NONE;
     } while(0);

    /* As the timings are not critical anymore provide some postponed logs */
    _sid_radio_log_mod_params_change();

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to start continuous Rx. Error %u", (uint32_t)err);
    }

    return err;
}
#endif /* HALO_ENABLE_DIAGNOSTICS */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_irq_ack(void)
{
    SID_RADIO_LOG_DEBUG("Received IRQ readout ack from the host MCU -> clearing IRQ indication");

    // TODO: ensure we actually are in IRQ state before clearing the indication
#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
    /* Prohibit the system to enter Stop mode since SubGHz events should be processed ASAP from this point */
    if (current_radio_state != SID_PAL_RADIO_SLEEP)
    {
        UTILS_ENTER_CRITICAL_SECTION();
        if (UTIL_TIMER_IsRunning(&lpm_suspend_timer) != FALSE)
        {
            (void)UTIL_TIMER_Stop(&lpm_suspend_timer);
        }
        UTILS_EXIT_CRITICAL_SECTION();
        __COMPILER_BARRIER(); /* Esnure the timer is always processed before the UTIL_LPM_Set<X>Mode() calls */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_RADIO), UTIL_LPM_DISABLE);
        UTIL_LPM_SetOffMode((1u << CFG_LPM_HOST_COMM_RADIO), UTIL_LPM_DISABLE);
    }
#endif /* LOW_POWER_DISABLE */
    SID_HOST_COMM_IRQ_CLEAR_EVENT();    /* Release IRQ line, turn off LED (if used) */
    SID_HOST_COMM_IRQ_ENABLE_TRIGGER(); /* Now enable EXTI on IRQ line to catch the GPIO Handshake requests from the host MCU */
    __COMPILER_BARRIER();

    switch (last_indicated_radio_event)
    {
        case STM32WLxx_PAL_RADIO_EVENT_TX_DONE:
            SID_PAL_LOG_DEBUG("Tx done. Len: %u", last_indicated_radio_event_data_len);
            break;

        case STM32WLxx_PAL_RADIO_EVENT_RX_DONE:
            SID_PAL_LOG_DEBUG("Rx done. Len: %u", last_indicated_radio_event_data_len);
            break;

        case STM32WLxx_PAL_RADIO_EVENT_CS_DONE:
#if SID_PAL_LOG_LEVEL >= SID_PAL_LOG_SEVERITY_DEBUG
            {
                int32_t rssi = (int32_t)((int8_t)(last_indicated_radio_event_data_len >> 24));
                uint32_t tx_len = last_indicated_radio_event_data_len & 0x00FFFFFFu;
                uint32_t current_freq_khz = current_radio_freq_hz / 1000u;
                uint32_t mhz_int  = current_freq_khz / 1000u;
                uint32_t mhz_frac = current_freq_khz % 1000u;
                SID_PAL_LOG_DEBUG("Tx of %u bytes aborted, ch busy: %s%ddB, %u.%03uMHz", tx_len, rssi < 0 ? "" : "+", rssi, mhz_int, mhz_frac);
            }
#endif /* SID_PAL_LOG_LEVEL */
            break;

        default:
            /* No logs for other event types */
            break;
    }

    /* Avoid repeated log printouts */
    last_indicated_radio_event = STM32WLxx_PAL_RADIO_EVENT_UNKNOWN;

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
    /* Allow the system to enter LPM after sleep request is acknowledged */
    if (SID_PAL_RADIO_SLEEP == current_radio_state)
    {
        /* Enable the system to enter Stop mode since we don't expect any SubGHz operations anytime soon */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_RADIO), UTIL_LPM_ENABLE);
    }
#endif /* LOW_POWER_DISABLE */

    return SID_RADIO_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_sunghz_reset_req(const stm32wlxx_rcp_reset_subghz_req_t * const request)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    SID_RADIO_LOG_DEBUG("Received SubGHz peipheral reset request");

    do
    {
        if ((request->reset_key_1 != STM32WLxx_RADIO_COMM_SUBGHZ_RESET_KEY_1)
          || (request->reset_key_2 != STM32WLxx_RADIO_COMM_SUBGHZ_RESET_KEY_2))
        {
            SID_PAL_LOG_ERROR("External request to reset SubGHz peripheral rejected - invalid keys (k1: 0x%08x, k2: 0x%08x)", request->reset_key_1, request->reset_key_2);
            err = SID_RADIO_ERROR_FORBIDDEN;
            break;
        }

        err = sid_radio_reset_hardware();
        if (err != SID_RADIO_ERROR_NONE)
        {
            /* Log messages are printed by sid_radio_reset_hardware() */
            break;
        }

        /* err is already set to SID_RADIO_ERROR_NONE by sid_radio_reset_hardware() */
    } while (0);

    /* Indicate reset completion, wether it is successful or not */
    sid_radio_error_t indication_err = _sid_radio_indicate_request_processed(STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_RESET, err, NULL, 0u);
    if (indication_err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to set IRQ indication upon SubGHz Reset completion. Error %u", (uint32_t)indication_err);
    }

    /* If only the indication error took place - report it. Otherwise report the root error */
    err = SID_RADIO_ERROR_NONE == err ? indication_err : err;

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_subghz_sleep_req(const stm32wlxx_rcp_radio_sleep_req_t * const request)
{
    sid_radio_error_t err;

    do
    {
#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
        /* Check if this is a deep sleep request */
        if (request->deep_sleep_en != FALSE)
        {
            if ((request->sleep_duration_us != 0u) || (request->deep_sleep_key_1 != STM32WLxx_RADIO_COMM_DEEP_SLEEP_KEY_1) || (request->deep_sleep_key_2 != STM32WLxx_RADIO_COMM_DEEP_SLEEP_KEY_2))
            {
                SID_PAL_LOG_WARNING("Invalid deep sleep request - ignored");
                err = SID_RADIO_ERROR_INVALID_ARGS;
                break;
            }

            /* Put SubGHz into sleep */
            err = _sid_radio_subghz_sleep(request->sleep_duration_us);
            if (err != SID_RADIO_ERROR_NONE)
            {
                break;
            }

            /* Valid request, enable Standby/Off LPM mode */
            SID_PAL_LOG_INFO("Sidewalk operations ceased");
            UTIL_LPM_SetOffMode((1u << CFG_LPM_HOST_COMM_RADIO), UTIL_LPM_ENABLE);
        }
        else
#endif /* LOW_POWER_DISABLE */
        {
            /* Normal sleep request - just put SubGHz into sleep */
            err = _sid_radio_subghz_sleep(request->sleep_duration_us);
        }
    } while (0);

    /* Indicate reset completion, wether it is successful or not */
    sid_radio_error_t indication_err = _sid_radio_indicate_request_processed(STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SLEEP, err, NULL, 0u);
    if (indication_err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to set IRQ indication upon SubGHz Sleep completion. Error %u", (uint32_t)indication_err);
    }

    /* If only the indication error took place - report it. Otherwise report the root error */
    err = SID_RADIO_ERROR_NONE == err ? indication_err : err;

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_subghz_standby_req(void)
{
    sid_radio_error_t err;

    err = _sid_radio_subghz_standby();

    /* Indicate reset completion, wether it is successful or not */
    sid_radio_error_t indication_err = _sid_radio_indicate_request_processed(STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_STANDBY, err, NULL, 0u);
    if (indication_err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to set IRQ indication upon SubGHz Standby completion. Error %u", (uint32_t)indication_err);
    }

    /* If only the indication error took place - report it. Otherwise report the root error */
    err = SID_RADIO_ERROR_NONE == err ? indication_err : err;

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_apply_cfg_req(const uint8_t * const frame_data, const uint32_t frame_length)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    sid_radio_error_t indication_err;

    SID_RADIO_LOG_DEBUG("Received SubGHz configuration update");

    const stm32wlxx_rcp_apply_cfg_t * const apply_cfg_frame = (stm32wlxx_rcp_apply_cfg_t *)(void *)frame_data;
    assert_param(sizeof(apply_cfg_frame->radio_config) == sizeof(WL55_cfg));
    do
    {
        /* Validity check */
        if (frame_length != sizeof(*apply_cfg_frame))
        {
            SID_PAL_LOG_ERROR("Failed to apply SubGHz Radio configuration. Received %u bytes but expected %u", frame_length, sizeof(*apply_cfg_frame));
            err = SID_RADIO_ERROR_WRONG_SIZE;
            break;
        }

        /* Store the updated config */
        UTILS_ENTER_CRITICAL_SECTION();
        SID_STM32_UTIL_fast_memcpy(&WL55_cfg, &apply_cfg_frame->radio_config, sizeof(WL55_cfg));
        valid_subghz_cfg_received = TRUE;
#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
        (void)osThreadFlagsSet(udt_ctx.task, SID_RADIO_UDT_DRIVER_READY_FLAG);
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */
        UTILS_EXIT_CRITICAL_SECTION();

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    if (apply_cfg_frame->need_ack != FALSE)
    {
        /* Indicate request completion, wether it is successful or not */
        indication_err = _sid_radio_indicate_request_processed(STM32WLxx_RADIO_COMM_OPCODE_SEND_CONFIG, err, NULL, 0u);
        if (indication_err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to acknowledge SubGHz Radio Configuration request. Error %u", (uint32_t)indication_err);
        }
    }
    else
    {
        indication_err = SID_RADIO_ERROR_NONE;
    }

    /* Print log info after acknowledgment is scheduled with SPI to avoid delaying the data transfer by logs */
    if (SID_RADIO_ERROR_NONE == err)
    {
        SID_PAL_LOG_INFO("Received SubGHz configuration");
    }

    /* If only the indication error took place - report it. Otherwise report the root error */
    err = SID_RADIO_ERROR_NONE == err ? indication_err : err;

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_sunghz_base_init_req(const stm32wlxx_rcp_apply_base_hw_cfg_req_t * const request)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    sid_radio_error_t indication_err;

    SID_RADIO_LOG_DEBUG("Got SubGHz base HW init request");
    do
    {
        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_ERROR("SubGHz hardware init requested before host MCU supplied SubGHz config");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        err = sid_radio_subghz_init();
    } while (0);


    if (request->need_ack != FALSE)
    {
        /* Indicate reset completion, whether it is successful or not */
        indication_err = _sid_radio_indicate_request_processed(STM32WLxx_RADIO_COMM_OPCODE_APPLY_BASE_HW_CONFIG, err, NULL, 0u);
        if (indication_err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to set IRQ indication upon SubGHz base HW init completion. Error %u", (uint32_t)indication_err);
        }
    }
    else
    {
        indication_err = SID_RADIO_ERROR_NONE;
    }

    /* If only the indication error took place - report it. Otherwise report the root error */
    err = SID_RADIO_ERROR_NONE == err ? indication_err : err;

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_set_tx_power(const stm32wlxx_rcp_set_tx_power_req_t * const request)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        SID_RADIO_LOG_DEBUG("SubGHz Set Tx Power request: %s%ddB", request->target_tx_power >= 0 ? "+" : "", request->target_tx_power); /* tiny_vsnprintf may not support %+d */

        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to configure SubGHz Tx power before the config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        err = _sid_radio_set_tx_power(request); /* Don't do explicit casting to (const stm32wlxx_radio_pa_setup_t * const) to detect any possible future deviations of stm32wlxx_rcp_set_tx_power_req_t from stm32wlxx_radio_pa_setup_t definition */

        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to set SubGHz Tx power to %s%ddB. Error %u", request->target_tx_power >= 0 ? "+" : "", request->target_tx_power, err); /* tiny_vsnprintf may not support %+d */
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_set_subghz_tx_buf_req(const uint8_t * const frame_data, const uint32_t available_frame_length, const uint32_t full_frame_length)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;

    assert_param(frame_data != NULL);
    const stm32wlxx_rcp_set_subghz_tx_buf_t * const set_tx_buf_frame = (stm32wlxx_rcp_set_subghz_tx_buf_t *)(void *)frame_data;
#if DEBUG
    uint32_t calculated_full_frame_length = set_tx_buf_frame->write_length + sizeof(set_tx_buf_frame->opcode) + sizeof(set_tx_buf_frame->write_length);
    if (calculated_full_frame_length < sizeof(stm32wlxx_radio_comm_spi_frame_t))
    {
        calculated_full_frame_length = sizeof(stm32wlxx_radio_comm_spi_frame_t);
    }
    assert_param(full_frame_length == calculated_full_frame_length);
#endif

    /* See if we can do a (partial) upload to the SubGHz Tx buffer */
    const uint32_t available_subghz_tx_data = available_frame_length - offsetof(stm32wlxx_rcp_set_subghz_tx_buf_t, tx_buf_content);
    const uint32_t can_upload_size = available_subghz_tx_data - radio_tx_buf_upload_offset;
    assert_param(available_subghz_tx_data >= radio_tx_buf_upload_offset);

    if (can_upload_size > 0u)
    {
        do
        {
            SID_RADIO_LOG_DEBUG("Set SubGHz Tx buf: add %u bytes, offset %u", can_upload_size, radio_tx_buf_upload_offset);

            hal_err = HAL_SUBGHZ_WriteBuffer(&hsubghz, (uint8_t)radio_tx_buf_upload_offset, (uint8_t *)&set_tx_buf_frame->tx_buf_content[radio_tx_buf_upload_offset], can_upload_size);
            if (hal_err != HAL_OK)
            {
                SID_RADIO_LOG_ERROR("Failed to write %u bytes to SubGHz buffer. HAL_SUBGHZ err: 0x%x", set_tx_buf_frame->write_length, hal_err);
                err = SID_RADIO_ERROR_HARDWARE;
                break;
            }

            /* Done, advance the upload pointer */
            radio_tx_buf_upload_offset += can_upload_size;

            if (radio_tx_buf_upload_offset == set_tx_buf_frame->write_length)
            {
                /* Tx buffer upload completed */
                SID_RADIO_LOG_DEBUG("Set SubGHz Tx buf: done, total %u bytes", set_tx_buf_frame->write_length);
                radio_tx_buf_upload_offset = 0u; /* Reset for the next uploads */
                last_indicated_radio_event_data_len = set_tx_buf_frame->write_length;
            }

            err = SID_RADIO_ERROR_NONE;
        } while (0);
    }
    else
    {
        /* Not enough data to upload, keep waiting for addiiotnal incoming data */
        err = SID_RADIO_ERROR_NONE;
    }

    if (err != SID_RADIO_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("Failed to set SubGHz Tx buffer. Bytes written: %u, expected: %u, error %u", radio_tx_buf_upload_offset, set_tx_buf_frame->write_length, err);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_process_start_tx_req(void)
{
    sid_radio_error_t err;

    do
    {
        if (FALSE == valid_subghz_cfg_received)
        {
            SID_PAL_LOG_WARNING("Attempt to start Tx before SubGHz config is sent");
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == current_modem_mode)
        {
            err = _sid_radio_start_tx(WL55_cfg.fsk_cfg.tx_timeout);
        }
        else if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA == current_modem_mode)
        {
            err = _sid_radio_start_tx(WL55_cfg.lora_cfg.tx_timeout);
        }
        else
        {
            err = SID_RADIO_ERROR_INVALID_STATE;
        }

        /* As the timings are not critical anymore provide some postponed logs */
        _sid_radio_log_mod_params_change();

        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to start SubGHz Tx. Error %u. Modem mode: %u", (uint32_t)err, current_modem_mode);
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _sid_radio_log_mod_params_change(void)
{
    if (mod_params_upd_indication_pending != FALSE)
    {
        if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == current_modem_mode)
        {
            SID_PAL_LOG_DEBUG("FSK mod params updated. bw:0x%02X shp:0x%02X br:%u fdev: %u",
                              WL55_cfg.fsk_cfg.mod.bandwidth, WL55_cfg.fsk_cfg.mod.shaping, WL55_cfg.fsk_cfg.mod.bit_rate, WL55_cfg.fsk_cfg.mod.freq_dev);
        }
        else if (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_LORA == current_modem_mode)
        {
            SID_PAL_LOG_DEBUG("LoRa mod params updated. bw:0x%02X cr:0x%02X sf:0x%02X",
                              WL55_cfg.lora_cfg.mod.bandwidth, WL55_cfg.lora_cfg.mod.coding_rate, WL55_cfg.lora_cfg.mod.spreading_factor);
        }
        else
        {
            /* Nothing to do */
        }

        mod_params_upd_indication_pending = FALSE;
    }
}

/*----------------------------------------------------------------------------*/

static void _sid_radio_on_radio_timeout_event(void * context)
{
    sid_radio_error_t                      err;
    stm32wlxx_pal_radio_events_t           radio_event = STM32WLxx_PAL_RADIO_EVENT_UNKNOWN;
    stm32wlxx_radio_comm_spi_frame_t       spi_reply_frame;
    stm32wlxx_rcp_irq_status_t * const     irq_status_reply     = &spi_reply_frame.payload.irq_status;
    stm32wlxx_subghz_irq_details_t * const irq_details          = &irq_status_reply->subghz_irq_details;
    uint32_t                               irq_disarmed         = FALSE;
    int8_t                                 rssi_now             = 0;
    uint32_t                               radio_state_on_entry;
    uint32_t                               event_timestamp_s;
    uint32_t                               event_timestamp_us;

    (void)context;

    do
    {
        SID_RADIO_LOG_DEBUG("STM32WLxx_SUBGHZ_SIMULATED_IRQ_TIMEOUT");

        if ((current_radio_state != SID_PAL_RADIO_RX) && (current_radio_state != SID_PAL_RADIO_TX))
        {
            err = SID_RADIO_ERROR_INVALID_STATE;
            break;
        }

        /* Run in a critical section to avoid interference with radio IRQ processing */
        UTILS_ENTER_CRITICAL_SECTION();
        {
            /* Capture the event timestamp */
            event_timestamp_s = TIMER_IF_GetTimeUs(&event_timestamp_us);
            __COMPILER_BARRIER();

            /* Check if NVIC indicates an active SubGHz IRQ */
            if (HAL_NVIC_GetActive(SUBGHZ_Radio_IRQn) != FALSE)
            {
                /* There's an active radio IRQ, terminate processing */
                UTILS_EXIT_CRITICAL_SECTION();
                err = SID_RADIO_ERROR_INVALID_STATE;
                break;
            }

            /* There's no active IRQ at the moment - temporarily deactivate IRQ pin */
            HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);
            irq_disarmed = TRUE;
            __COMPILER_BARRIER();

#if SID_HOST_COMM_PROCESSING_DELAY_PROFILING
            /* Indicate the actual radio IRQ event for profiling */
            SID_PDP_RADIO_IRQ_ACTIVITY_GPIO_Port->BSRR = SID_PDP_RADIO_IRQ_ACTIVITY_Pin;
#endif /* SID_HOST_COMM_PROCESSING_DELAY_PROFILING */

        }
        /* Done with the critical processing */
        UTILS_EXIT_CRITICAL_SECTION();

        /* Set default follow-up data length to zero */
        irq_status_reply->followup_payload_len = 0u;

        /* Capture real-time RSSI before putting the radio to Standby */
        if (WL55_cfg.drv_cfg.cad_exit_mode != STM32WLxx_RADIO_CAD_EXIT_MODE_NONE)
        {
            (void)_sid_radio_subghz_get_rssi(&rssi_now);
        }
        __COMPILER_BARRIER(); /* Ensure RSSI readout takes place before the radio is put to Standby */

        /* Put the radio to Standby state */
        err = _sid_radio_subghz_set_standby(STDBY_XOSC); /* Use STDBY_XOSC to save on oscillator restart time (e.g. when Tx follows right after CS) */
        if (err != SID_RADIO_ERROR_NONE)
        {
            current_radio_state = SID_PAL_RADIO_UNKNOWN;
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* Turn off FEM as we don't need it any longer */
        (void)_sid_radio_subghz_set_radio_mode(STM32WLxx_RADIO_FRONTEND_MODE_OFF);

        radio_state_on_entry = current_radio_state;
        current_radio_state  = SID_PAL_RADIO_STANDBY;
        __COMPILER_BARRIER();

        if (SID_PAL_RADIO_RX == radio_state_on_entry)
        {
            if ((WL55_cfg.drv_cfg.cad_exit_mode != STM32WLxx_RADIO_CAD_EXIT_MODE_NONE) && (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == current_modem_mode))
            {
                /* Restore the default IRQ mask for FSK after any type of Carrier Sense */
                err = _sid_radio_enable_default_irqs();
                if (err != SID_RADIO_ERROR_NONE)
                {
                    SID_RADIO_LOG_ERROR("Failed to set IRQ mask for FSK mode");
                    break;
                }
            }

            /* Process Carrier Sense/CAD events */
            if ((STM32WLxx_RADIO_CAD_EXIT_MODE_CS_LBT == WL55_cfg.drv_cfg.cad_exit_mode) && (STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_FSK == current_modem_mode))
            {
                WL55_cfg.drv_cfg.cad_exit_mode = STM32WLxx_RADIO_CAD_EXIT_MODE_NONE;

                if (rssi_now >= STM32WLxx_RADIO_COMM_FSK_CS_RSSI_THRESHOLD)
                {
                    /* RSSI is above the threshold - radio channel is busy */
                    radio_event = STM32WLxx_PAL_RADIO_EVENT_CS_DONE;
                    err = SID_RADIO_ERROR_NONE;

                    /* Store the RSSI on the channel */
                    irq_details->received_packet_info.fsk_rx_packet_status.rssi_sync = rssi_now;

                    /* Append RSSI debug info to the unsued MSB of the Tx payload length */
                    last_indicated_radio_event_data_len = (last_indicated_radio_event_data_len & 0x00FFFFFFu) |  ((uint32_t)rssi_now << 24);
                }
                else
                {
                    /* CS detected no activity - selected radio channel is free */
#  if HALO_ENABLE_DIAGNOSTICS
                    radio_event = STM32WLxx_PAL_RADIO_EVENT_CS_TIMEOUT;
                    err = SID_RADIO_ERROR_NONE;
#  else
                    /* Carrier Sense for FSK ended with timeout - radio channel is free and we can proceed with Tx */
                    radio_event = STM32WLxx_PAL_RADIO_EVENT_UNKNOWN;

                    /* Start Tx */
                    err = _sid_radio_start_tx(STM32WLxx_RADIO_COMM_CAD_DEFAULT_TX_TIMEOUT);
                    if (err != SID_RADIO_ERROR_NONE)
                    {
                        /* Logs provided by _sid_radio_start_tx() */
                        break;
                    }
#  endif /* HALO_ENABLE_DIAGNOSTICS */
                }
            }
            else if (STM32WLxx_RADIO_CAD_EXIT_MODE_NONE == WL55_cfg.drv_cfg.cad_exit_mode)
            {
                /* This was a normal Rx, not CS */
                radio_event = STM32WLxx_PAL_RADIO_EVENT_RX_TIMEOUT;
                err = SID_RADIO_ERROR_NONE;
            }
            else
            {
                /* Nothing to do here - some sort of CS we're not interested in */
                err = SID_RADIO_ERROR_NONE;
            }
        }
        else /* if (SID_PAL_RADIO_TX == radio_state_on_entry) */
        {
            radio_event = STM32WLxx_PAL_RADIO_EVENT_TX_TIMEOUT;
            err = SID_RADIO_ERROR_NONE;
        }
    } while (0);

    if ((SID_RADIO_ERROR_NONE == err) && (STM32WLxx_PAL_RADIO_EVENT_UNKNOWN != radio_event))
    {
        /* Notify the radio about the event */
        err = _sid_radio_report_event(radio_event, &spi_reply_frame, event_timestamp_s, event_timestamp_us);
    }
    else
    {
#if SID_HOST_COMM_PROCESSING_DELAY_PROFILING
        /* Clear radio IRQ event indication */
        SID_PDP_RADIO_IRQ_ACTIVITY_GPIO_Port->BRR = SID_PDP_RADIO_IRQ_ACTIVITY_Pin;
#endif /* SID_HOST_COMM_PROCESSING_DELAY_PROFILING */
    }

    /* Restore IRQ pin settings */
    if (irq_disarmed != FALSE)
    {
        HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_start_radio_timeout_mon(const uint32_t timeout_us)
{
    sid_radio_error_t err;
    UTIL_TIMER_Status_t timer_err;

    do
    {
        if ((0u == timeout_us) || (SID_RADIO_INFINITE_TIME == timeout_us))
        {
            /* No need to start the countdown */
            err = SID_RADIO_ERROR_NONE;
            break;
        }

        /* Ensure the delay will be at lest the requested amount of the SubGHz ticks */
        const uint32_t rx_timeout = timeout_us + STM32WLxx_SUBGHZ_TICKS_TO_US(1u);

        timer_err = UTIL_TIMER_StartWithPeriodUs(&radio_timeout_mon, rx_timeout);
        if (timer_err != UTIL_TIMER_OK)
        {
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
SID_STM32_SPEED_OPTIMIZED static void _sid_radio_lpm_suspend_timer_cb(void * context)
{
    (void)context;

    UTILS_ENTER_CRITICAL_SECTION();
    if (FALSE == valid_subghz_cfg_received)
    {
        /* Timer has elapsed and still there's no valid config received - put the radio into Sleep to save power */
        sid_radio_error_t err = _sid_radio_subghz_sleep(0u);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_WARNING("Failed to put SubGHz into Sleep. Error %u", (uint32_t)err);

            /* Restart the timer to try to put the radio into sleep a bit later */
            (void)UTIL_TIMER_StartWithPeriod(&lpm_suspend_timer, SID_RADIO_LPM_RESUME_RETRY_PERIOD_MS);
        }
        else
        {
            /* Enable the system to enter LPM as well */
            UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_RADIO), UTIL_LPM_ENABLE);
            UTIL_LPM_SetOffMode((1u << CFG_LPM_HOST_COMM_RADIO), UTIL_LPM_ENABLE);
        }
    }
    else if (SID_PAL_RADIO_SLEEP == current_radio_state)
    {
        /* Enable the system to enter Stop since no activity took place, but driver is initialized */
        UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_RADIO), UTIL_LPM_ENABLE);
    }
    else
    {
        /* Nothing to do here, keep LPM config as is */
    }
    UTILS_EXIT_CRITICAL_SECTION();
}
#endif /* LOW_POWER_DISABLE */

/*----------------------------------------------------------------------------*/

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_udt_init(void)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        /* Create UDT suspend timer */
        UTIL_TIMER_Status_t timer_err = UTIL_TIMER_Create(&udt_ctx.suspend_timer, 0, UTIL_TIMER_ONESHOT, _sid_radio_udt_suspend_timer_cb, NULL);
        if (timer_err != UTIL_TIMER_OK)
        {
            SID_PAL_LOG_ERROR("Can't create UDT suspend timer. Error %d", (int32_t)timer_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* Create UDT message queue */
        if (NULL == udt_ctx.outbound_msg_queue)
        {
            udt_ctx.outbound_msg_queue = osMessageQueueNew(SID_RADIO_UDT_OUT_MSG_QUEUE_LEN, sizeof(sid_host_comm_out_udt_msg_desc_t), NULL);
            if (NULL == udt_ctx.outbound_msg_queue)
            {
                SID_PAL_LOG_ERROR("Can't create UDT message queue. No memory");
                err = SID_RADIO_ERROR_NOMEM;
                break;
            }
        }

        /* Create the task that will handle the UDT */
        if (NULL == udt_ctx.task)
        {
            udt_ctx.task = osThreadNew(_user_data_transfer_thread, NULL, &wlxx_user_data_transfer_thread_attributes);
            if (NULL == udt_ctx.task)
            {
                SID_PAL_LOG_ERROR("Can't create UDT processing thread. No memory");
                err = SID_RADIO_ERROR_NOMEM;
                break;
            }
        }

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/*----------------------------------------------------------------------------*/

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
SID_STM32_SPEED_OPTIMIZED static inline sid_radio_error_t _sid_radio_udt_setup_session(const uint32_t sleep_duration_us)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    UTIL_TIMER_Status_t timer_err;

    do
    {
        /* Ensure UDT cut-off timer is stopped. Run in a crtitical sections to avoid race conditions with timer ineterrupt */
        UTILS_ENTER_CRITICAL_SECTION();
        if (UTIL_TIMER_IsRunning(&udt_ctx.suspend_timer) != FALSE)
        {
            timer_err = UTIL_TIMER_Stop(&udt_ctx.suspend_timer);
        }
        else
        {
            /* No need to stop timer */
            timer_err = UTIL_TIMER_OK;
        }
        UTILS_EXIT_CRITICAL_SECTION();
        if (timer_err != UTIL_TIMER_OK)
        {
            /* Failed to stop the timer, can't proceed */
            udt_ctx.udt_enabled = FALSE;
            err = SID_RADIO_ERROR_HARDWARE;
            SID_PAL_LOG_ERROR("UDT - failed to stop cut-off timer. Error %d", (int32_t)timer_err);
            break;
        }

        /* Determine if UDT operations can be enabled during the planned sleep period */
        if ((0u == sleep_duration_us) || (UINT32_MAX == sleep_duration_us))
        {
            /* This is a special case - radio is put into Sleep for undefined amount of time (e.g. due to sid_stop() call) */
            /* Allow UDT task to run */
            udt_ctx.udt_enabled = TRUE;
            __COMPILER_BARRIER();
            (void)osThreadFlagsSet(udt_ctx.task, SID_RADIO_UDT_TRANSFER_ENABLED_FLAG);
        }
        else if (sleep_duration_us > SID_RADIO_UDT_SUSPEND_RESERVED_TIME_US)
        {
            /* Schedule the UDT cut-off timer */
            uint32_t cutoff_delay_ms = (sleep_duration_us - SID_RADIO_UDT_SUSPEND_RESERVED_TIME_US) / 1000u;

            /* Ensure we can schedule the cut-off event properly */
            if (cutoff_delay_ms >= UTIL_TimerDriver.GetMinimumTimeout())
            {
                /* Configure cut-off timer */
                timer_err =  UTIL_TIMER_SetPeriod(&udt_ctx.suspend_timer, cutoff_delay_ms);
                if (timer_err != UTIL_TIMER_OK)
                {
                    udt_ctx.udt_enabled = FALSE;
                    err = SID_RADIO_ERROR_HARDWARE;
                    SID_PAL_LOG_ERROR("UDT - failed to set cut-off timer period. Error %d", (int32_t)timer_err);
                    break;
                }

                /* Start the cut-off countdown */
                timer_err = UTIL_TIMER_Start(&udt_ctx.suspend_timer);
                if (timer_err != UTIL_TIMER_OK)
                {
                    udt_ctx.udt_enabled = FALSE;
                    err = SID_RADIO_ERROR_HARDWARE;
                    SID_PAL_LOG_ERROR("UDT - failed to start cut-off timer. Error %d", (int32_t)timer_err);
                    break;
                }

                /* Allow UDT task to run */
                udt_ctx.udt_enabled = TRUE;
                __COMPILER_BARRIER();
                (void)osThreadFlagsSet(udt_ctx.task, SID_RADIO_UDT_TRANSFER_ENABLED_FLAG);
            }
            else
            {
                /* Can't meet the constraints of UTIL_TIMER, skipping UDT */
                udt_ctx.udt_enabled = FALSE;
                SID_RADIO_LOG_WARNING("UDT skipped due to constraints of UTIL_TIMER");
            }
        }
        else
        {
            /* Not enough time for UDT */
            udt_ctx.udt_enabled = FALSE;
            SID_RADIO_LOG_WARNING("UDT skipped due to insufficient sleep duration");
        }

        /* Everything is fine if we got here */
        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_radio_error_t sid_radio_process_app_frame(const uint8_t * const frame_data, const uint32_t available_frame_length, const uint32_t full_frame_length)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    if ((NULL == frame_data) || (0u == full_frame_length))
    {
        SID_RADIO_LOG_ERROR("SID Radio App frame cannot be null");
        return SID_RADIO_ERROR_INVALID_ARGS;
    }

    const stm32wlxx_radio_comm_spi_frame_t * const frame = (stm32wlxx_radio_comm_spi_frame_t *)(void *)frame_data; /* Cast pointer to conveniently access frame elements */

    switch (frame->opcode)
    {
        case STM32WLxx_RADIO_COMM_OPCODE_DUMMY_DATA:
            {
#if SID_RADIO_EXTRA_LOGGING
                if (available_frame_length == full_frame_length)
                {
                    uint32_t actual_payload_length = full_frame_length - (sizeof(*frame) + sizeof(frame->payload));
                    SID_RADIO_LOG_DEBUG("Received %u bytes of dummy payload, skipping", actual_payload_length);
                }
#endif /* SID_RADIO_EXTRA_LOGGING */
                err = SID_RADIO_ERROR_NONE;
            }
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_START:
        case STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_CONT:
            /* This should never happen as Long Data Transfer wrapping shall be demangled before the data gets here */
            SID_RADIO_LOG_ERROR("Unexpectedly received Long Data Transfer command (OpCode 0x%x)", (uint32_t)frame->opcode);
            err = SID_RADIO_ERROR_UNEXPECTED_DATA;
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_IRQ_STATUS:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            SID_RADIO_LOG_DEBUG("Host MCU performed IRQ status readout");
            err = SID_RADIO_ERROR_NONE;
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_IRQ_ACK:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_process_irq_ack();
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_RESET:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_process_sunghz_reset_req(&frame->payload.subghz_reset_req);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SLEEP:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_process_subghz_sleep_req(&frame->payload.radio_sleep);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_STANDBY:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_process_subghz_standby_req();
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SEND_CONFIG:
            if (available_frame_length == full_frame_length)
            {
                err = _sid_radio_process_apply_cfg_req(frame_data, full_frame_length);
            }
            else
            {
                /* Just wait for the whole config data to arrive */
#if SID_RADIO_EXTRA_LOGGING
                SID_RADIO_LOG_DEBUG("Received %u out of %u bytes of radio config", available_frame_length, full_frame_length);
#endif /* SID_RADIO_EXTRA_LOGGING */
                err = SID_RADIO_ERROR_NONE;
            }
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_APPLY_BASE_HW_CONFIG:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_process_sunghz_base_init_req(&frame->payload.apply_hw_cfg);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_MODEM_MODE:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_set_modem_mode(&frame->payload.set_modem_mode);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_FREQUENCY:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_set_frequency(&frame->payload.set_frequency);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_TX_POWER:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_process_set_tx_power(&frame->payload.set_tx_power);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_SYNCWORD:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_set_sync_word(&frame->payload.set_sync_word);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_SYMB_TIMEOUT:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_set_lora_symbol_timeout(&frame->payload.set_lora_symb_timeout);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_MOD_PARAMS:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_process_set_lora_mod_params_req(&frame->payload.set_lora_mod_params);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_PKT_PARAMS:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_set_lora_packet_params(&frame->payload.set_lora_pkt_params);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SET_LORA_CAD_PARAMS:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_set_lora_cad_params();
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SET_FSK_MOD_PARAMS:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_process_set_fsk_mod_params_req(&frame->payload.set_fsk_mod_params);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SET_FSK_PKT_PARAMS:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_process_set_fsk_packet_params_req(&frame->payload.set_fsk_pkt_params);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_SET_TX_BUF:
            err = _sid_radio_process_set_subghz_tx_buf_req(frame_data, available_frame_length, full_frame_length);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_START_TX:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_process_start_tx_req();
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SUBGHZ_START_RX:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_start_rx(&frame->payload.start_rx_params);
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_START_CARRIER_SENSE:
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_start_fsk_carrier_sense();
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_USER_DATA:
#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
            err = _sid_radio_process_user_data_upload(frame_data, available_frame_length, full_frame_length);
#else
            SID_PAL_LOG_WARNING("Command USER_DATA is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_START_CW_TX:
#if HALO_ENABLE_DIAGNOSTICS
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_start_cw_tx(&frame->payload.start_cw_tx_params);
#else
            SID_PAL_LOG_WARNING("Command START_CW_TX is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_START_CONTINUOUS_RX:
#if HALO_ENABLE_DIAGNOSTICS
            assert_param(available_frame_length == sizeof(stm32wlxx_radio_comm_spi_frame_t));
            err = _sid_radio_start_continuous_rx();
#else
            SID_PAL_LOG_WARNING("Command START_CONTINIOUS_RX is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_SET_RX_DUTY_CYCLE:
#if HALO_ENABLE_DIAGNOSTICS
            //TODO: implement diagnostic command
            SID_PAL_LOG_WARNING("Command SET_RX_DUTY_CYCLE is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#else
            SID_PAL_LOG_WARNING("Command SET_RX_DUTY_CYCLE is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_START_LORA_CAD:
#if HALO_ENABLE_DIAGNOSTICS
            //TODO: implement diagnostic command
            SID_PAL_LOG_WARNING("Command START_LORA_CAD is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#else
            SID_PAL_LOG_WARNING("Command START_LORA_CAD is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_GET_LIVE_RSSI:
#if HALO_ENABLE_DIAGNOSTICS
            //TODO: implement diagnostic command
            SID_PAL_LOG_WARNING("Command GET_LIVE_RSSI is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#else
            SID_PAL_LOG_WARNING("Command GET_LIVE_RSSI is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_CHECK_CHANNEL_FREE:
#if HALO_ENABLE_DIAGNOSTICS
            //TODO: implement diagnostic command
            SID_PAL_LOG_WARNING("Command CHECK_CHANNEL_FREE is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#else
            SID_PAL_LOG_WARNING("Command CHECK_CHANNEL_FREE is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
            break;

        case STM32WLxx_RADIO_COMM_OPCODE_GET_CHANNEL_NOISE:
#if HALO_ENABLE_DIAGNOSTICS
            //TODO: implement diagnostic command
            SID_PAL_LOG_WARNING("Command GET_CHANNEL_NOISE is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#else
            SID_PAL_LOG_WARNING("Command GET_CHANNEL_NOISE is not supported");
            err = SID_RADIO_ERROR_NOT_SUPPORTED;
#endif /* HALO_ENABLE_DIAGNOSTICS */
            break;

        default:
            SID_PAL_LOG_ERROR("Received an unknown Sidewalk Radio App OpCode: 0x%X, 0x%X", (uint32_t)frame->opcode, frame->payload.raw[0]);
            err = SID_RADIO_ERROR_INVALID_OPCODE;
            break;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_radio_error_t sid_radio_set_handshake_response(void)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    stm32wlxx_radio_comm_spi_frame_t hs_response = {
        .opcode = STM32WLxx_RADIO_COMM_OPCODE_IRQ_STATUS,
        .payload = {
            .irq_status = {
                .irq_flags = STM32WLxx_APP_IRQ_SPI_HANDSHAKE,
                .followup_payload_len = 0u,
                .handshake_details = {
                    .protocol_version = {
                        .major = STM32WLxx_RADIO_COMM_PROTOCOL_MAJOR_VERSION,
                        .minor = STM32WLxx_RADIO_COMM_PROTOCOL_MINOR_VERSION,
                        .patch = STM32WLxx_RADIO_COMM_PROTOCOL_PATCH_VERSION,
                    },
                    .app_version = {
                        .major = SID_APP_PROJECT_MAJOR_VERSION,
                        .minor = SID_APP_PROJECT_MINOR_VERSION,
                        .patch = SID_APP_PROJECT_PATCH_VERSION,
                    },
                },
            },
        },
    };

    const sid_host_comm_error_t hc_err = sid_host_comm_enqueue_tx((void *)&hs_response, sizeof(hs_response));
    if (hc_err != SID_HOST_COMM_ERROR_NONE)
    {
        SID_RADIO_LOG_ERROR("Unable to enqueue Handshake IRQ response. Host Comm error %u", (uint32_t)hc_err);
        err = SID_RADIO_ERROR_HARDWARE;
    }
    else
    {
        SID_RADIO_LOG_DEBUG("Successfully enqueued Handshake IRQ response");
        err = SID_RADIO_ERROR_NONE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

sid_radio_error_t sid_radio_reset_hardware(void)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
    udt_ctx.udt_enabled = FALSE;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

    HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);

    uint32_t remaining_attempts = SID_RADIO_SUBGHZ_RESET_ATTEMPTS;
    do
    {
        err = _subghz_reset();
        if (SID_RADIO_ERROR_TIMEOUT == err)
        {
            /* Recoverable error */
            SID_PAL_LOG_WARNING("Can't reset SubGHz peripheral - operation timeout. A retry will be attempted");
            sid_pal_scheduler_delay_ms(SID_RADIO_SUBGHZ_RESET_COOLDOWN_MS);
        }
        else
        {
            /* Either everything is ok or unrecoverable error happened - jump out in both cases */
            break;
        }

        remaining_attempts--;
    } while (remaining_attempts != 0u);

    /* Inspect the results of SPI communication startup */
    if (SID_RADIO_ERROR_NONE != err)
    {
        SID_PAL_LOG_ERROR("Can't reset SubGHz peripheral. Unrecoverable error %u", (uint32_t)err);
    }
    else
    {
        /* Clear config after reset */
        UTILS_ENTER_CRITICAL_SECTION();
        SID_STM32_UTIL_fast_memset(&WL55_cfg, 0u, sizeof(WL55_cfg));
        subghz_cold_start                   = TRUE;
        current_radio_state                 = SID_PAL_RADIO_UNKNOWN;
        current_modem_mode                  = STM32WLxx_SUBGHZ_RADIO_MODEM_MODE_UNDEFINED;
        current_radio_freq_hz               = 0u;
        last_indicated_radio_event          = STM32WLxx_PAL_RADIO_EVENT_UNKNOWN;
        last_indicated_radio_event_data_len = 0u;
        mod_params_upd_indication_pending   = 0u;
        valid_subghz_cfg_received           = FALSE;
        radio_tx_buf_upload_offset          = 0u;

        /* Initialize the peripheral */
        err = sid_radio_subghz_init();
        UTILS_EXIT_CRITICAL_SECTION();

        /* Restore IRQ settings */
        HAL_NVIC_ClearPendingIRQ(SUBGHZ_Radio_IRQn);
        HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, RADIO_IRQ_PRIO_HIGH, 0u);
        HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);

        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to perform SubGHz hardware initialization. Error %u", (uint32_t)err);
        }
        else
        {
            SID_PAL_LOG_DEBUG("SubGHz peripheral reset and initialized");
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_radio_error_t sid_radio_subghz_init(void)
{
    sid_radio_error_t err;
    HAL_StatusTypeDef hal_err;

    do
    {
        /* Apply essential settings after power on or full SubGHz reset --------------*/
        if (subghz_cold_start != FALSE)
        {
            /* Low level init */
            RADIO_INIT();

            /* Put the radio into the Standby RC state -----------------------------------*/
            err = _sid_radio_subghz_set_standby(STDBY_RC);
            if (err != SID_RADIO_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("SubGHz init failure - unable to put radio into Standby RC state. Error %u", (uint32_t)err);
                break;
            }
            sid_pal_delay_us(STM32WLxx_SUBGHZ_STDBY_STATE_DELAY_US);
            /*----------------------------------------------------------------------------*/

            /* Select regulator mode -----------------------------------------------------*/
            uint8_t regulator_mode;
            if (RBI_IsDCDC() != FALSE)
            {
                /* Use SMPS if the board supports it */
                const uint8_t drive_lvl = SMPS_DRIVE_SETTING_DEFAULT;

                /* Enable SMPS clock detection to avoid permanent damages to the IC if HSE clock is frozen */
                err = _sid_radio_subghz_smps_clk_detect_en();
                if (err != SID_RADIO_ERROR_NONE)
                {
                    /* Logs are provided by _sid_radio_subghz_smps_clk_detect_en() */
                    Error_Handler();
                }

                err = _sid_radio_smps_set(drive_lvl);
                if (err != SID_RADIO_ERROR_NONE)
                {
                    SID_RADIO_LOG_ERROR("Failed to set SMPS current drive to 0x%02X. Error %u", drive_lvl, (uint32_t)err);
                    break;
                }

                /* Configure MCU to use SMPS as well */
                LL_PWR_SMPS_SetMode(LL_PWR_SMPS_STEP_DOWN);
                regulator_mode = USE_DCDC;
            }
            else
            {
                /* Fall back to the LDO */
                LL_PWR_SMPS_SetMode(LL_PWR_SMPS_BYPASS);
                regulator_mode = USE_LDO;
            }

            err = _sid_radio_set_reg_mode(regulator_mode);
            if (err != SID_RADIO_ERROR_NONE)
            {
                SID_RADIO_LOG_ERROR("Failed to set SubGHz regulator mode to %s. Error %u", USE_LDO == regulator_mode ? "LDO" : "DC-DC", (uint32_t)err);
                break;
            }
            /*----------------------------------------------------------------------------*/

            /* Initialize TCXO control ---------------------------------------------------*/
            if (RBI_IsTCXO() != FALSE) /* If the board supports TCXO */
            {
                if (LL_RCC_HSE_IsEnabledTcxo() == FALSE)
                {
                    SID_PAL_LOG_WARNING("HSE TCXO power is disabled, falling back to XTAL");
                    /* Set XTAL trim values from the radio driver config */
                    err = _sid_radio_subghz_set_xtal_trim(XTAL_DEFAULT_CAP_VALUE, XTAL_DEFAULT_CAP_VALUE);
                    if (err != SID_RADIO_ERROR_NONE)
                    {
                        break;
                    }
                }
                else
                {
                    /* Configure TCXO as clock source using the values from the radio driver config */
                    err = _sid_radio_set_tcxo_mode(TCXO_CTRL_VOLTAGE, STM32WLxx_RADIO_COMM_TCXO_TIMEOUT_DURATION_TUS);
                    if (err != SID_RADIO_ERROR_NONE)
                    {
                        break;
                    }

                    /* XTAL trim will be automatically changed to 0x2F (33.4 pF) to filter any glitches */
                }
            }
            else
            {
                /* Set XTAL trim values from the radio driver config */
                err = _sid_radio_subghz_set_xtal_trim(XTAL_DEFAULT_CAP_VALUE, XTAL_DEFAULT_CAP_VALUE);
                if (err != SID_RADIO_ERROR_NONE)
                {
                    break;
                }
            }

            /* Perform calibration calibration */
            err = _sid_radio_calibrate(0x7Fu); /* calibrate everything */
            if (err != SID_RADIO_ERROR_NONE)
            {
                break;
            }
            /*----------------------------------------------------------------------------*/

            /* WORKAROUND - Trimming the output voltage power_ldo to 3.3V ----------------*/
            uint8_t drv_ctrl_val = 0x7u << 1;
            hal_err = HAL_SUBGHZ_WriteRegisters(&hsubghz, REG_DRV_CTRL, &drv_ctrl_val, sizeof(drv_ctrl_val));
            if (hal_err != HAL_OK)
            {
                SID_PAL_LOG_ERROR("Failed to write SubGHz reg. Reg: 0x%04X, HAL_SUBGHZ err: 0x%x", REG_DRV_CTRL, hal_err);
                err = SID_RADIO_ERROR_HARDWARE;
            }
            /*----------------------------------------------------------------------------*/

            SID_PAL_LOG_DEBUG("SubGHz cold start done");
            subghz_cold_start = FALSE;
        }
        /*----------------------------------------------------------------------------*/

        /* Set default Tx/Rx fallback mode -------------------------------------------*/
        uint8_t set_fb_mode_buf = (uint8_t)STM32WLxx_RADIO_FALLBACK_STDBY_XOSC;
        hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_SET_TXFALLBACKMODE, &set_fb_mode_buf, sizeof(set_fb_mode_buf));
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to execute SubGHz cmd. Cmd: 0x%02X, HAL_SUBGHZ err: 0x%x", RADIO_SET_TXFALLBACKMODE, hal_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* Put the radio into the Standby XOSC state ---------------------------------*/
        err = _sid_radio_subghz_set_standby(STDBY_XOSC);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("SubGHz init failure - unable to put radio into Standby XOSC state. Error %u", (uint32_t)err);
            break;
        }
        sid_pal_delay_us(STM32WLxx_SUBGHZ_STDBY_STATE_DELAY_US);

        /* Configure internal buffer base addresses ----------------------------------*/
        err = _sid_radio_set_buffer_base_addr(SID_RADIO_TX_BUFFER_BASE_OFFSET, SID_RADIO_RX_BUFFER_BASE_OFFSET);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("Failed to set base addresses for SubGHz Tx/Rx internal buffers. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* External-only config parameters -------------------------------------------*/
        if (valid_subghz_cfg_received != FALSE)
        {
            /* Configure Rx Boost using the received config if available -----------------*/
            err = _sid_radio_cfg_rx_boosted(WL55_cfg.drv_cfg.rx_boost);
            if (err != SID_RADIO_ERROR_NONE)
            {
                SID_RADIO_LOG_ERROR("Failed to set SubGHz Rx boost. Error %u", (uint32_t)err);
                break;
            }
            /*----------------------------------------------------------------------------*/

            /* Tx power ------------------------------------------------------------------*/
            err = _sid_radio_set_tx_power(&WL55_cfg.pa_cfg);
            if (err != SID_RADIO_ERROR_NONE)
            {
                SID_RADIO_LOG_ERROR("Failed to set SubGHz Tx power. Error %u", (uint32_t)err);
                break;
            }
            /*----------------------------------------------------------------------------*/
        }
        /*----------------------------------------------------------------------------*/

        /* Clear all error flags -----------------------------------------------------*/
        uint16_t errors;
        err = _sid_radio_get_subghz_errors(&errors);
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("Failed to read SubGHz error flags. Error %u", (uint32_t)err);
            break;
        }
        SID_RADIO_LOG_DEBUG("Actie SubGHz error flaghs on init: 0x%04X", errors);
        err = _sid_radio_clear_subghz_errors();
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_RADIO_LOG_ERROR("Failed to clear SubGHz error flags. Error %u", (uint32_t)err);
            break;
        }
        /*----------------------------------------------------------------------------*/

        /* Apply the customized IRQ mask if the external config has arrived ----------*/
        if (valid_subghz_cfg_received != FALSE)
        {
            err = _sid_radio_enable_default_irqs();
            if (err != SID_RADIO_ERROR_NONE)
            {
                /* Logs are provided by _sid_radio_enable_default_irqs() */
                break;
            }
        }
        /*----------------------------------------------------------------------------*/

        /* Init RF Switch GPIO -------------------------------------------------------*/
        RBI_Init();
        /*----------------------------------------------------------------------------*/

        /* Done */
        current_radio_state = SID_PAL_RADIO_STANDBY;
        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_radio_error_t sid_radio_handle_subghz_irq(const uint32_t irq_timestamp_s, const uint32_t irq_timestamp_us)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;
    HAL_StatusTypeDef hal_err;

    //TODO: Consider modifying SubGHz HAL driver to use DMA (or at least do some optimizations) to offload CPU core and speed up data exchange
    do
    {
        uint8_t irq_status_buf[2u];
        uint32_t itsource;

        /* Retrieve Interrupts from SUBGHZ Irq Register */
        hal_err = HAL_SUBGHZ_ExecGetCmd(&hsubghz, RADIO_GET_IRQSTATUS, irq_status_buf, sizeof(irq_status_buf));
        if (hal_err != HAL_OK)
        {
            SID_RADIO_LOG_ERROR("Unable to retrieve SubGHzIRQ status. HAL error 0x%x", (uint32_t)hal_err);
            err = HAL_BUSY == hal_err ? SID_RADIO_ERROR_RESOURCE_BUSY : SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* Convert the received bytes into usable format and add store it*/
        itsource = ((uint32_t)irq_status_buf[0] << 8) | (uint32_t)irq_status_buf[1];

        /** Clear SUBGHZ IRQ register - this will clear the EXTI line and associated IRQ in NVIC if no more radio
         *  radio events are pending. However, should there appear a new radio IRQ while the driver is processing
         *  the ccurent one, the IRQ line will be triggered again and ISR will be re-entered automatically to
         *  handle the new radio event. This is the desired behavior of this driver
         */
        hal_err = HAL_SUBGHZ_ExecSetCmd(&hsubghz, RADIO_CLR_IRQSTATUS, irq_status_buf, sizeof(irq_status_buf));
        if (hal_err != HAL_OK)
        {
            SID_RADIO_LOG_ERROR("Unable to clear SubGHz IRQ status. Active IRQ: 0x%x, HAL error 0x%x", itsource, (uint32_t)hal_err);
            err = HAL_BUSY == hal_err ? SID_RADIO_ERROR_RESOURCE_BUSY : SID_RADIO_ERROR_HARDWARE;
            break;
        }

        /* Clear SubGHz IRQ pending flag in NVIC - if any new radio IRQ occurs during the processing, NVIC flag will be set again by hardware */
        HAL_NVIC_ClearPendingIRQ(SUBGHZ_Radio_IRQn);

        /* Process the received radio events (IRQs) */
        err = _sid_radio_irq_process(itsource, irq_timestamp_s, irq_timestamp_us);

        /* Terminate on failed IRQ handling attempt */
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("SubGHz IRQ process error %u. IRQ: 0x%04X radio state: %u, modem %u", (uint32_t)err, itsource, current_radio_state, current_modem_mode);
            break;
        }

        /* Re-enable SubGHz IRQ after the processing is done */
        HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, RADIO_IRQ_PRIO_HIGH, 0u);
        HAL_NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);
        err = SID_RADIO_ERROR_NONE;
    } while (0);

    /**
     * @note If an error happens during the radio IRQ processing, the code should exit without re-enabling the SUBGHZ IRQ line.
     *       This will prevent entering an infinite ISR loop. Instead, the error is procssed by the HostComm task, which triggers
     *       SUBGHZ internal reset, clears NVIC state, and re-enables the SUBGHZ IRQ line once it is safe to do so.
     * 
     */

#if SID_HOST_COMM_PROCESSING_DELAY_PROFILING
    /* Clear the actual radio IRQ event indication for profiling after the radio IRQ is processed */
    SID_PDP_RADIO_IRQ_ACTIVITY_GPIO_Port->BRR = SID_PDP_RADIO_IRQ_ACTIVITY_Pin;
#endif /* SID_HOST_COMM_PROCESSING_DELAY_PROFILING */

    return err;
}

/*----------------------------------------------------------------------------*/

/* Even though this method logically belongs to host_comm.c, keep it here to avoid cascading function calls and speed up the processing */
SID_STM32_SPEED_OPTIMIZED sid_host_comm_error_t sid_host_comm_set_user_data_received_cb(sid_host_comm_on_incoming_user_data callback)
{
#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
    /* Update the pointer to the user callback in a critical section to avoid mid-flight invocations */
    UTILS_ENTER_CRITICAL_SECTION();
    udt_ctx.on_user_data_rx = callback;
    UTILS_EXIT_CRITICAL_SECTION();

    if (callback != NULL)
    {
        SID_PAL_LOG_DEBUG("STM32WLxx On User Data Received callback set");
    }
    else
    {
        SID_PAL_LOG_DEBUG("STM32WLxx On User Data Received callback cleared");
    }

    return SID_HOST_COMM_ERROR_NONE;
#else
    (void)callback;
    return SID_HOST_COMM_ERROR_NOT_SUPPORTED;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_host_comm_error_t sid_host_comm_send_user_data(const uint8_t * const data, const uint32_t data_len, const uint32_t auto_free)
{
#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
    sid_host_comm_error_t err = SID_HOST_COMM_ERROR_GENERIC;
    osStatus_t os_err;

    do
    {
        /* Validate the inputs */
        if ((NULL == data) || (0u == data_len))
        {
            err = SID_HOST_COMM_INVALID_ARGS;
            break;
        }

        /* This API can be invoked from concurrent thread, use a local copy of the driver status to avoid race conditions */
        UTILS_ENTER_CRITICAL_SECTION();
        const uint32_t local_valid_subghz_cfg_received = valid_subghz_cfg_received;
        UTILS_EXIT_CRITICAL_SECTION();

        if ((FALSE == local_valid_subghz_cfg_received) || (NULL == udt_ctx.outbound_msg_queue))
        {
            /* Driver is not ready to accept user messages */
            err = SID_HOST_COMM_ERROR_INVALID_STATE;
            break;
        }

        /* Put the data in the queue */
        sid_host_comm_out_udt_msg_desc_t msg = {
            .data_ptr =  (uint8_t *)data,
            .data_len  = data_len,
            .auto_free = auto_free,
        };

        os_err = osMessageQueuePut(udt_ctx.outbound_msg_queue, &msg, 0u, 0u);
        if (os_err != osOK)
        {
            /* Failed to put the message in the queue, probably it is full */
            err = SID_HOST_COMM_ERROR_RESOURCE_ALLOC;
            break;
        }

        err = SID_HOST_COMM_ERROR_NONE;
    } while (0);

    return err;
#else
    (void)data;
    (void)data_len;
    (void)auto_free;
    return SID_HOST_COMM_ERROR_NOT_SUPPORTED;
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */
}

/*----------------------------------------------------------------------------*/

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
SID_STM32_SPEED_OPTIMIZED void sid_radio_tmp_lpm_prohibit(void)
{
    /* Temporarily prohibit LPM to allow fast IRQ acknowledgment -----------------*/
    UTIL_TIMER_Status_t timer_err;

    timer_err = UTIL_TIMER_StartWithPeriod(&lpm_suspend_timer, SID_RADIO_HANDSHAKE_LPM_SUSPEND_TIME_MS);
    if (timer_err != UTIL_TIMER_OK)
    {
        /* Something went wrong. That's not critical - notify user and proceed */
        SID_PAL_LOG_WARNING("Failed to arm LPM suspend timer. Error %u", (uint32_t)timer_err);
    }
    /* Prohibit the system to enter Stop mode since SubGHz events should be processed ASAP from this point */
    UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_RADIO), UTIL_LPM_DISABLE);
    UTIL_LPM_SetOffMode((1u << CFG_LPM_HOST_COMM_RADIO), UTIL_LPM_DISABLE);
}
#endif /* LOW_POWER_DISABLE */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_radio_error_t sid_radio_app_init(void)
{
    sid_radio_error_t err = SID_RADIO_ERROR_GENERIC;

    do
    {
        /* Create the timer to process Rx,Tx, and CS/CAD timeouts */
        UTIL_TIMER_Status_t timer_err = UTIL_TIMER_Create(&radio_timeout_mon, 0u, UTIL_TIMER_ONESHOT, _sid_radio_on_radio_timeout_event, NULL);
        if (timer_err != UTIL_TIMER_OK)
        {
            SID_PAL_LOG_ERROR("Failed to create radio timeout monitor. Error %d", (int32_t)timer_err);
        }

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
        /* Create LPM suspend timer for handshake requests */
        timer_err = UTIL_TIMER_Create(&lpm_suspend_timer, 0u, UTIL_TIMER_ONESHOT, _sid_radio_lpm_suspend_timer_cb, NULL);
        if (timer_err != UTIL_TIMER_OK)
        {
            SID_PAL_LOG_ERROR("Can't create LPM suspend timer. Error %d", (int32_t)timer_err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }
#endif /* LOW_POWER_DISABLE */

        /* Initialize the peripheral */
        err = sid_radio_subghz_init();
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to perform SubGHz hardware initialization. Error %u", (uint32_t)err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
        err = _sid_radio_udt_init();
        if (err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to initialize UDT service. Error %u", (uint32_t)err);
            err = SID_RADIO_ERROR_HARDWARE;
            break;
        }
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

        err = SID_RADIO_ERROR_NONE;
    } while (0);

    return err;
}
