/**
  ******************************************************************************
  * @file    sid_pal_serial_bus_spi_config.h
  * @brief   SPI bus abstraction layer for Sidewalk
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

#include <cmsis_os2.h>

/* MCU-specific headers */
#include <stm32wbaxx.h>
#include <stm32wbaxx_hal_gpio.h>
#include <stm32wbaxx_ll_dma.h>

/* Sidewalk SDK includes */
#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_pal_gpio_ifc.h>
#include <sid_pal_gpio_ext_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_pal_serial_bus_ifc.h>
#include <sid_pal_serial_bus_spi_config.h>
#include <sid_pal_uptime_ifc.h>
#include <sid_time_ops.h>

/* STM32 utils */
#include <sid_stm32_common_utils.h>
#include <stm_list.h>
#include <stm32_mem.h>

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief Storage for pre-configured peripheral register values to be written to
 * the corresponding peripherals by DMA upon request
 */
typedef struct {
    /* SPI_CR1 values */
    uint32_t spi_cr1_reg_val_start;

    /* LPTIM_CR values */
    uint32_t lptim_cr_val_en;
    uint32_t lptim_cr_val_start;
    uint32_t lptim_cr_val_dis;

    /* LPTIM_CCRx values */
    uint32_t lptim_ccr1_val_nss_to_nss;
    uint32_t lptim_ccr2_val_nss_to_sck;
    uint32_t lptim_ccrx_val_sck_to_nss;
    uint32_t lptim_ccrx_val_sck_to_nss_dma_rx_compensation;
    uint32_t lptim_ccr1_val_xfer_end;
} dma_regs_storage_t;

/**
 * @brief Client-specific settings for the SPI bus
 */
typedef struct {
    /* SPI phy cfg */
    uint32_t spi_prescaler;
    uint32_t crc_polynomial;
    uint32_t crc_length;
    uint32_t crc_init_pattern;

    /* NSS line */
    GPIO_TypeDef * nss_gpio_port;
    uint32_t nss_gpio_pin;

    /* NSS handling LPTIM */
    uint32_t nss_lptim_clock_hz;
    uint32_t nss_lptim_prescaler;

    /* NSS timings */
    uint32_t nss_to_sck_cycles;
    uint32_t sck_to_nss_cycles;
    uint32_t nss_to_nss_cycles;

    /* Storage for the peripheral register values to be written into peripherals by DMA */
    dma_regs_storage_t dma_regs_storage;
} serial_bus_spi_internal_config_t;

/**
 * @brief Bus-specific settings for the SPI and DMA
 */
typedef struct {
    uint32_t dma_trigger_lptimx_ch1_val;
    uint32_t dma_trigger_lptimx_ch2_val;

    uint32_t dma_trigger_spi_dmatx_ch_val;
    uint32_t dma_trigger_spi_dmarx_ch_val;
} dma_peripheral_config_t;

/**
 * @brief Context of an SPI client - stores client-specific info and settings
 */
typedef struct {
    uint32_t                         client_selector;    /* Unique ID of the client */
    uint32_t *                       dma_lln_buffer;     /* Pointer to the storage space for the DMA LL nodes prototypes */
    uint32_t                         dma_lln_buf_length; /* Length of the storage space for the DMA LL nodes prototypes in words */
    serial_bus_spi_internal_config_t peripherals_cfg;    /* Client-specific peripherals config values */
    tListNode                        node;               /* node element to build a linked list */
} spi_client_context_t;

/**
 * @brief Context of an SPI bus - stores info and settings that are common for all
 * clients of that SPI bus
 */
typedef struct {
    struct sid_pal_serial_bus_iface                       iface;
    const sid_pal_serial_bus_stm32wbaxx_factory_config_t* factory_config;
    dma_peripheral_config_t                               dma_periph_cfg;
    tListNode                                             client_contexts;
} serial_bus_spi_ctx;

/* Private defines -----------------------------------------------------------*/

/**
 * @brief Start address of the DMA linked list nodes RAM section
 */
#define DMA_LLN_REGION_START                                            ((uint32_t)&__start_DMA_LLN)
/**
 * @brief End address of the DMA linked list nodes RAM section
 */
#define DMA_LLN_REGION_END                                              ((uint32_t)&__end_DMA_LLN)

/**
 * @brief Total amount of SPI bus clients that can operate in parallel
 */
#define SPI_CLIENTS_NUMBER_MAX                                          (1u)

/**
 * @brief Length of a per-client buffer to store DMA linked list nodes. Each element is 4 bytes
 */
#define SPI_DMA_LLN_CLIENT_BUF_LENGTH                                   (28u)

/**
 * @brief Watermark to indicate that a per-client DMA LLN buffer is free
 */
#define SPI_DMA_LLN_BUF_FREE_WATERMARK                                  (0xF9EEBEEFu)

/** @defgroup SPI_DMA_LLN_OFFSETS  DMA linked list buffer offsets - used to dynamically update LLN configuration
  * @{
  */
#define SPI_DMA_LLN_INITIAL_CLLR_VALS_OFFSET                            (0u)
#define SPI_DMA_LLN_WRITE_SPI_CSTART_CTR2_OFFSET                        (1u)
#define SPI_DMA_LLN_NSS_EOT_TRIGGER_SRC_OFFSET                          (23u)
#define SPI_DMA_LLN_NSS_EOT_DST_ADDR_OFFSET                             (26u)
/**
  * @}
  */

/** @defgroup SPI_LPTIM_REFERENCE_DELAYS  Reference values for LPTIM timer captured at 12MHz timer clock speed
  * @{
  */

#define SPI_LPTIM_REFERENCE_CLOCK_HZ                                    (24000000u)

#define SPI_LPTIM_OVERSAMPLING_POW_2_EXP                                (2u)

/**
 * @brief Minimum ticks possible between activating the NSS and issuing the very first SCK edge
 * Measured in LPTIM ticks @24MHz LPTIM clock
 *
 * @attention The value of this parameter is mainly defined by the time required for the Transfer
 *            Control DMA channel to fetch the next linked list node and apply the new configuration.
 *            Subsequently, the speed of that operation is defined by PCLK clock speed driving the DMA
 *            and SRAM wait states configuration. The reference value below is valid for 96MHz system
 *            core clock. If you reduce the system core clock or the PCLK driving the DMA peripheral,
 *            you may need to increase the value of this parameter to reliably operate SPI.
 */
#define SPI_LPTIM_NSS_TO_SCK_MIN_DELAY_CYCLES_24MHZ_REFERENCE           (24u)

/**
 * @brief Adjustment to the LPTIM counter since some time is consumed by DMA and timer start
 * delay
 * Measured in LPTIM ticks @24MHz LPTIM clock
 */
#define SPI_LPTIM_NSS_TO_SCK_COMPENSATORY_CYCLES_24MHZ_REFERENCE        (9u)

#define SPI_LPTIM_SCK_TO_NSS_DMA_RX_COMPENSATORY_CYCLES_24MHZ_REFERENCE (11u)

/**
  * @}
  */

#define DMA_CCR_EVENT_SRC_MASK                                                         (DMA_CCR_TCIE   /*!< Transfer complete interrupt             */ | \
                                                                                        DMA_CCR_HTIE   /*!< Half transfer complete interrupt        */ | \
                                                                                        DMA_CCR_DTEIE  /*!< Data transfer error interrupt           */ | \
                                                                                        DMA_CCR_ULEIE  /*!< Update linked-list item error interrupt */ | \
                                                                                        DMA_CCR_USEIE  /*!< User setting error interrupt            */ | \
                                                                                        DMA_CCR_SUSPIE /*!< Completed suspension interrupt          */ | \
                                                                                        DMA_CCR_TOIE   /*!< Trigger overrun interrupt               */)

#define DMA_CFCR_ALL_EVENT_FLAGS                                                       (DMA_CFCR_TCF   /*!< Transfer complete flag             */ | \
                                                                                        DMA_CFCR_HTF   /*!< Half transfer complete flag        */ | \
                                                                                        DMA_CFCR_DTEF  /*!< Data transfer error flag           */ | \
                                                                                        DMA_CFCR_ULEF  /*!< Update linked-list item error flag */ | \
                                                                                        DMA_CFCR_USEF  /*!< User setting error flag            */ | \
                                                                                        DMA_CFCR_SUSPF /*!< Completed suspension flag          */ | \
                                                                                        DMA_CFCR_TOF   /*!< Trigger overrun flag               */)

/**
 * @brief A minimum timeout for an SPI transaction. If the calculated timeout is shorter than this value, it will be overwritten
 */
#define SPI_TRANSACTION_MINIMUM_TIMEOUT_US                              (100u)

/* Private macro -------------------------------------------------------------*/

#ifndef containerof
#  define containerof(ptr, type, member)                         ((type *)((uintptr_t)(ptr) - offsetof(type, member)))
#endif

#define SPI_HAL_PRESCALER_TO_POW_2_EXP_VAL(__hal_presacler__)                          (SPI_BAUDRATEPRESCALER_BYPASS == (__hal_presacler__) ? (0u) : (((__hal_presacler__ ) >> 28) + 1u))
#define SPI_POW_2_EXP_TO_HAL_PRESCALER_VAL(__pow2_exp__)                               (0u == (__pow2_exp__) ? (SPI_BAUDRATEPRESCALER_BYPASS) : ((__pow2_exp__ - 1u) << 28))

#define LPTIM_HAL_PRESCALER_TO_POW_2_EXP_VAL(__hal_presacler__)                        (((__hal_presacler__) & (LPTIM_CFGR_PRESC)) >> (LPTIM_CFGR_PRESC_Pos))
#define LPTIM_POW_2_EXP_TO_HAL_PRESCALER_VAL(__pow2_exp__)                             (((__pow2_exp__) << (LPTIM_CFGR_PRESC_Pos)) & (LPTIM_CFGR_PRESC))

#define LPTIM_RECALCULATE_CYCLES_FROM_REFERENCE(__act_clk__, __ref_clk__, __ref_val__) ((uint32_t)((__act_clk__) == (__ref_clk__) ? \
                                                                                        (__ref_val__) : \
                                                                                        ((uint64_t)(__ref_val__) * (uint64_t)(__act_clk__)) / (uint64_t)(__ref_clk__)) \
                                                                                       )
#define SPI_LPTIM_NSS_TO_SCK_MIN_DELAY_CYCLES(__actual_clock__)                        LPTIM_RECALCULATE_CYCLES_FROM_REFERENCE((__actual_clock__), SPI_LPTIM_REFERENCE_CLOCK_HZ, SPI_LPTIM_NSS_TO_SCK_MIN_DELAY_CYCLES_24MHZ_REFERENCE)
#define SPI_LPTIM_NSS_TO_SCK_COMPENSATORY_CYCLES(__actual_clock__)                     LPTIM_RECALCULATE_CYCLES_FROM_REFERENCE((__actual_clock__), SPI_LPTIM_REFERENCE_CLOCK_HZ, SPI_LPTIM_NSS_TO_SCK_COMPENSATORY_CYCLES_24MHZ_REFERENCE)

#define SPI_HAL_CRC_SIZE_TO_INT_VAL(__hal_crc_size__)                                  ((uint32_t)((((__hal_crc_size__) & SPI_CFG1_CRCSIZE) >> SPI_CFG1_CRCSIZE_Pos) + 1u))

/* External variables --------------------------------------------------------*/

extern uint32_t __start_DMA_LLN;
extern uint32_t __end_DMA_LLN;

/* Private variables ---------------------------------------------------------*/

UTIL_MEM_PLACE_IN_SECTION(".dma_lln")
static uint32_t dma_lln_buf[SPI_CLIENTS_NUMBER_MAX][SPI_DMA_LLN_CLIENT_BUF_LENGTH];

static uint32_t dma_lln_buf_initialized = FALSE;

static serial_bus_spi_ctx bus = {0}; //TODO: handle multiple contexts

/* Private function prototypes -----------------------------------------------*/

static inline HAL_StatusTypeDef _hal_like_start_dma_data_channel(DMA_HandleTypeDef * const hdma, const uint32_t src_addr, const uint32_t dst_addr,
                                                                 const uint32_t src_data_size, const uint32_t use_irqs);
static inline HAL_StatusTypeDef _hal_like_dma_channel_stop(DMA_HandleTypeDef * const hdma);
static void _hal_like_spi_close_transfer(SPI_HandleTypeDef * const hspi, LPTIM_HandleTypeDef * const hlptim);
static void _hal_like_spi_handle_dma_error(DMA_HandleTypeDef *hdma);
static inline HAL_StatusTypeDef _hal_like_lock_spi(SPI_HandleTypeDef * const hspi);
static HAL_StatusTypeDef _hal_like_spi_prepare_transmit_receive_dma(SPI_HandleTypeDef * const hspi, const uint8_t * const tx_buf, uint8_t * const rx_buf,
                                                                    const uint16_t xfer_size, const uint32_t use_irqs, const uint32_t enable_crc, const uint32_t crc_length);
static inline uint32_t _get_lptim_peripheral_clock(const LPTIM_TypeDef * const hlptim_instance);
static inline uint32_t _get_spi_peripheral_clock(const SPI_TypeDef * const hspi_instance);
static sid_error_t _nss_lptim_init(serial_bus_spi_ctx * const out_ctx);
static sid_error_t _spi_peripheral_init(serial_bus_spi_ctx * const out_ctx);
static inline void _connect_adjacent_ll_nodes(uint32_t ** const prev_node, uint32_t * const prev_node_cllr_idx,
                                              uint32_t ** const next_node, const uint32_t next_node_cllr_idx);
static inline void _compute_peripheral_reg_values(const serial_bus_spi_ctx * const bus_ctx, const struct sid_pal_serial_bus_client * const client, spi_client_context_t * const client_ctx);
static inline void _dma_lln_buf_alloc(spi_client_context_t * const client_ctx);
static inline void _dma_lln_buf_free(spi_client_context_t * const client_ctx);
static inline spi_client_context_t * _find_spi_client_ctx(const serial_bus_spi_ctx * const bus_ctx, const struct sid_pal_serial_bus_client * const client);
static inline sid_error_t _create_spi_client_ctx(const serial_bus_spi_ctx * const bus_ctx, const struct sid_pal_serial_bus_client * const client, spi_client_context_t ** const out_client_ctx);
static inline void _delete_spi_client_ctx(spi_client_context_t * const client_ctx);
static inline sid_error_t _set_spi_frame_mode(SPI_HandleTypeDef * const hspi, const sid_pal_serial_bus_stm32wbaxx_spi_mode_t spi_mode);
static inline sid_error_t _handle_spi_xfer_polling(LPTIM_HandleTypeDef * const hlptim, SPI_HandleTypeDef * const hspi, const uint32_t transaction_timeout_us);
static sid_error_t _sid_pal_serial_bus_spi_stm32wbaxx_xfer(const struct sid_pal_serial_bus_iface *iface, const struct sid_pal_serial_bus_client *client,
                                                           uint8_t *tx, uint8_t *rx, size_t xfer_size);
static sid_error_t _sid_pal_serial_bus_spi_stm32wbaxx_xfer_hd(const struct sid_pal_serial_bus_iface *iface, const struct sid_pal_serial_bus_client *client,
                                                              uint8_t *tx, uint8_t *rx, size_t tx_size, size_t rx_size);
static sid_error_t _sid_pal_serial_bus_spi_stm32wbaxx_destroy(const struct sid_pal_serial_bus_iface *iface);

/* Private constants ---------------------------------------------------------*/

static const struct sid_pal_serial_bus_iface bus_ops = {
    .xfer    = _sid_pal_serial_bus_spi_stm32wbaxx_xfer,
    .xfer_hd = _sid_pal_serial_bus_spi_stm32wbaxx_xfer_hd,
    .destroy = _sid_pal_serial_bus_spi_stm32wbaxx_destroy,
};

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline HAL_StatusTypeDef _hal_like_start_dma_data_channel(DMA_HandleTypeDef * const hdma, const uint32_t src_addr, const uint32_t dst_addr,
                                                                                           const uint32_t src_data_size, const uint32_t use_irqs)
{
    /* Check the DMA peripheral handle parameter */
    if (hdma == NULL)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    assert_param(IS_DMA_BLOCK_SIZE(src_data_size));

    /* Process locked */
    __HAL_LOCK(hdma);

    /* Check DMA channel state */
    if (HAL_DMA_STATE_READY == hdma->State)
    {
        /* Update the DMA channel state */
        hdma->State = HAL_DMA_STATE_BUSY;

        /* Update the DMA channel error code */
        hdma->ErrorCode = HAL_DMA_ERROR_NONE;

        /* Configure the DMA channel data size */
        MODIFY_REG(hdma->Instance->CBR1, DMA_CBR1_BNDT, (src_data_size & DMA_CBR1_BNDT));

        /* Configure DMA channel source address */
        hdma->Instance->CSAR = src_addr;

        /* Configure DMA channel destination address */
        hdma->Instance->CDAR = dst_addr;

        /* Disable all DMA interrupts */
        __HAL_DMA_DISABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_DTE | DMA_IT_ULE | DMA_IT_USE | DMA_IT_SUSP | DMA_IT_TO));

        /* Clear all interrupt flags */
        __HAL_DMA_CLEAR_FLAG(hdma, DMA_FLAG_TC | DMA_FLAG_HT | DMA_FLAG_DTE | DMA_FLAG_ULE | DMA_FLAG_USE | DMA_FLAG_SUSP | DMA_FLAG_TO);

        if (use_irqs != FALSE)
        {
            /* Enable only the transfer error IRQ since the other events are not relevant */
            __HAL_DMA_ENABLE_IT(hdma, DMA_IT_DTE);
        }

        /* Enable DMA channel */
        __HAL_DMA_ENABLE(hdma);
    }
    else
    {
        /* Update the DMA channel error code */
        hdma->ErrorCode = HAL_DMA_ERROR_BUSY;

        /* Process unlocked */
        __HAL_UNLOCK(hdma);

        return HAL_ERROR;
    }

    return HAL_OK;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline HAL_StatusTypeDef _hal_like_dma_channel_stop(DMA_HandleTypeDef * const hdma)
{
    sid_error_t uptime_err;
    struct sid_timespec start_ts;
    struct sid_timespec elapsed_ts;

    /* Get start timestamp */
    uptime_err = sid_pal_uptime_now(&start_ts);
    SID_PAL_ASSERT(SID_ERROR_NONE == uptime_err);
    __COMPILER_BARRIER();

     /* Check the DMA peripheral handle parameter */
    if (hdma == NULL)
    {
        return HAL_ERROR;
    }

    /* Check DMA channel state */
    if (hdma->State != HAL_DMA_STATE_BUSY)
    {
        /* Update the DMA channel error code */
        hdma->ErrorCode = HAL_DMA_ERROR_NO_XFER;

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);

        return HAL_ERROR;
    }

    /* Suspend the channel if it is running */
    if ((hdma->Instance->CCR & DMA_CCR_EN) != 0u)
    {
        /* Suspend the channel */
        hdma->Instance->CCR |= DMA_CCR_SUSP;

        /* Update the DMA channel state */
        hdma->State = HAL_DMA_STATE_SUSPEND;

        /* Check if the DMA Channel is suspended */
        while ((hdma->Instance->CSR & DMA_CSR_SUSPF) == 0U)
        {
            /* Check for the Timeout */
            uptime_err = sid_pal_uptime_now(&elapsed_ts);
            SID_PAL_ASSERT(SID_ERROR_NONE == uptime_err);
            sid_time_sub(&elapsed_ts, &start_ts);
            if (sid_timespec_to_ms(&elapsed_ts) > HAL_TIMEOUT_DMA_ABORT)
            {
                /* Update the DMA channel error code */
                hdma->ErrorCode |= HAL_DMA_ERROR_TIMEOUT;

                /* Update the DMA channel state */
                hdma->State = HAL_DMA_STATE_ERROR;

                /* Process Unlocked */
                __HAL_UNLOCK(hdma);

                return HAL_ERROR;
            }
        }
    }

    /* Reset the channel */
    hdma->Instance->CCR |= DMA_CCR_RESET;

    /* Update the DMA channel state */
    hdma->State = HAL_DMA_STATE_ABORT;

    /* Clear all status flags */
    __HAL_DMA_CLEAR_FLAG(hdma, (DMA_FLAG_TC | DMA_FLAG_HT | DMA_FLAG_DTE | DMA_FLAG_ULE | DMA_FLAG_USE | DMA_FLAG_SUSP |
                                DMA_FLAG_TO));

    /* Update the DMA channel state */
    hdma->State = HAL_DMA_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(hdma);

    return HAL_OK;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _hal_like_spi_close_transfer(SPI_HandleTypeDef * const hspi, LPTIM_HandleTypeDef * const hlptim)
{
    uint32_t itflag = hspi->Instance->SR;

    __HAL_SPI_CLEAR_EOTFLAG(hspi);
    __HAL_SPI_CLEAR_TXTFFLAG(hspi);

    /* Disable SPI peripheral */
    __HAL_SPI_DISABLE(hspi);

    /* Disable ITs */
    __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | \
                                SPI_IT_FRE | SPI_IT_MODF));

    /* Disable Tx/Rx DMA Requests */
    CLEAR_BIT(hspi->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

    /* Stop LPTIM and related Transaction Control DMA channel */
    LL_LPTIM_Disable(hlptim->Instance); /* Beware of Errata 2.5.1. Should this implementation ever use LPTIM IRQ in NVIC, the LPTIM should be disabled through RCC reset instead of just clearing the Enable bit in LPTIM's CR */

    /* Clear Compare event flags, otherwise LPTIM may prevent the system from entering the low power mode */
    LL_LPTIM_ClearFlag_CC1(hlptim->Instance);
    LL_LPTIM_ClearFlag_CC2(hlptim->Instance);

    LL_DMA_DisableChannel(LL_DMA_GET_INSTANCE(hlptim->hdma[LPTIM_DMA_ID_CC1]->Instance), LL_DMA_GET_CHANNEL(hlptim->hdma[LPTIM_DMA_ID_CC1]->Instance));
    MODIFY_REG(hlptim->hdma[LPTIM_DMA_ID_CC1]->Instance->CFCR, DMA_CFCR_ALL_EVENT_FLAGS, DMA_CFCR_ALL_EVENT_FLAGS);

    /* Release DMA data channels */
    //TODO: check return codes
    (void)_hal_like_dma_channel_stop(hspi->hdmatx);
    (void)_hal_like_dma_channel_stop(hspi->hdmarx);

    /* Report UnderRun error for non RX Only communication */
    if (hspi->State != HAL_SPI_STATE_BUSY_RX)
    {
        if ((itflag & SPI_FLAG_UDR) != 0UL)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_UDR);
            __HAL_SPI_CLEAR_UDRFLAG(hspi);
        }
    }

    /* Report OverRun error for non TX Only communication */
    if (hspi->State != HAL_SPI_STATE_BUSY_TX)
    {
        if ((itflag & SPI_FLAG_OVR) != 0UL)
        {
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_OVR);
            __HAL_SPI_CLEAR_OVRFLAG(hspi);
        }

#if (USE_SPI_CRC != 0UL)
        /* Check if CRC error occurred */
        const uint32_t cfg1_reg = hspi->Instance->CFG1;
        if ((cfg1_reg & SPI_CFG1_CRCEN) != 0u)
        {
            if ((itflag & SPI_FLAG_CRCERR) != 0UL)
            {
                SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_CRC);
                __HAL_SPI_CLEAR_CRCERRFLAG(hspi);
            }
        }
#endif /* USE_SPI_CRC */
    }

    /* SPI Mode Fault error interrupt occurred -------------------------------*/
    if ((itflag & SPI_FLAG_MODF) != 0UL)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_MODF);
        __HAL_SPI_CLEAR_MODFFLAG(hspi);
    }

    /* SPI Frame error interrupt occurred ------------------------------------*/
    if ((itflag & SPI_FLAG_FRE) != 0UL)
    {
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_FRE);
        __HAL_SPI_CLEAR_FREFLAG(hspi);
    }

    hspi->TxXferCount = (uint16_t)0UL;
    hspi->RxXferCount = (uint16_t)0UL;

    /* Update HAL status */
    hspi->State = HAL_SPI_STATE_READY;
    if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    {
        //TODO: implement error handling
        SID_PAL_LOG_ERROR("SPI Transfer Error");
    }

    //TODO: call TX completed callback if needed
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _hal_like_spi_handle_dma_error(DMA_HandleTypeDef *hdma)
{
    SPI_HandleTypeDef * const hspi = (SPI_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
    const uint32_t dma_err = HAL_DMA_GetError(hdma);

    /* if DMA error is FIFO error ignore it */
    if ((dma_err != HAL_DMA_ERROR_NONE) && (bus.factory_config->hspi == hspi)) //TODO: support multiple bus contexts
    {
        LPTIM_HandleTypeDef * const hlptim = bus.factory_config->hlptim;

        /* Call SPI standard close procedure */
        _hal_like_spi_close_transfer(hspi, hlptim);

        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);
        hspi->State = HAL_SPI_STATE_READY;

        //TODO: invoke error callback from here
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline HAL_StatusTypeDef _hal_like_lock_spi(SPI_HandleTypeDef * const hspi)
{
    /* Lock the process */
    __HAL_LOCK(hspi);
    return HAL_OK;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static HAL_StatusTypeDef _hal_like_spi_prepare_transmit_receive_dma(SPI_HandleTypeDef * const hspi, const uint8_t * const tx_buf, uint8_t * const rx_buf,
                                                                                              const uint16_t xfer_size, const uint32_t use_irqs, const uint32_t enable_crc, const uint32_t crc_length)
{
    static uint32_t dummy_rx_buf;
    HAL_StatusTypeDef errorcode = HAL_ERROR;

    /* Check Direction parameter */
    assert_param(IS_SPI_DIRECTION_2LINES(hspi->Init.Direction));

    /* Ensure data size is fixed at 8 bits */
    assert_param(SPI_DATASIZE_8BIT == hspi->Init.DataSize);

    /* SPI shall be locked before calling this function */
    if(hspi->Lock != HAL_LOCKED)
    {
        errorcode = HAL_ERROR;
        return errorcode;
    }

    if (hspi->State != HAL_SPI_STATE_READY)
    {
        errorcode = HAL_BUSY;
        __HAL_UNLOCK(hspi);
        return errorcode;
    }

    if (((NULL == tx_buf) && (NULL == rx_buf)) || (0u == xfer_size))
    {
        errorcode = HAL_ERROR;
        __HAL_UNLOCK(hspi);
        return errorcode;
    }

    /* Set the transaction information */
    if ((tx_buf != NULL) && (rx_buf != NULL))
    {
        hspi->State   = HAL_SPI_STATE_BUSY_TX_RX;
    }
    else if (tx_buf != NULL)
    {
        hspi->State   = HAL_SPI_STATE_BUSY_TX;
    }
    else
    {
        hspi->State   = HAL_SPI_STATE_BUSY_RX;
    }
    hspi->ErrorCode   = HAL_SPI_ERROR_NONE;

    /* Configure Tx buffer pointer and counters */
    hspi->pTxBuffPtr  = (const uint8_t *)tx_buf;
    if (tx_buf != NULL)
    {
        hspi->TxXferSize  = xfer_size;
        hspi->TxXferCount = xfer_size;
    }
    else
    {
        hspi->TxXferSize  = 0u;
        hspi->TxXferCount = 0u;
    }

    /* Configure Rx buffer pointer and counters */
    if (rx_buf != NULL)
    {
        hspi->pRxBuffPtr  = (uint8_t *)rx_buf;
        hspi->RxXferSize  = xfer_size;
        hspi->RxXferCount = xfer_size;
    }
    else
    {
        hspi->pRxBuffPtr  = (uint8_t *)(void *)&dummy_rx_buf;
        hspi->RxXferSize  = 0u;
        hspi->RxXferCount = 0u;
    }

    /* Init field not used in handle to zero */
    hspi->RxISR       = NULL;
    hspi->TxISR       = NULL;

    /* Configure SPI duplex mode */
    if (hspi->TxXferCount > 0u)
    {
        /* Set Full-Duplex mode. Even if Tx-only transcation is expected, we still need to run the Rx DMA channel with a dummy buffer
         * because automated transaction end detection and NSS timing handling relies on the Transfer Complete event from the Rx DMA
         * channel
         */
        SPI_2LINES(hspi);
    }
    else
    {
        /* No Tx is expected - we can release the MOSI line and we don't need to feed SPI's Tx FIFO
         * IMPORTANT: this configuration physically disconnects the MOSI line from the SPI perihperal. This means that the receiver
         *            side may see random data if the MOSI line is not pulled and Tx CRC (if enabled) will not be calculated - this
         *            may trigger CRC errors on the receiving end. If you need to do a Receive-only transaction with CRC enabled
         *            please do a full Tx/Rx data exchange with a dummy Tx buffer (e.g. all-zeroes)
         */
        SPI_2LINES_RX(hspi);
    }

    /* Reset the Tx/Rx DMA bits */
    CLEAR_BIT(hspi->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

    /* Adjust DMA transaction size based on the xfer size */
    uint32_t dma_data_width;
    if (((xfer_size % sizeof(uint32_t)) == 0u) && ((FALSE == enable_crc) || (crc_length > SPI_CRC_LENGTH_24BIT)))
    {
        /* Amount of transfered bytes (including CRC if used) will be a multiple of 4 - we can use full word as DMA width */
        dma_data_width = LL_DMA_SRC_DATAWIDTH_WORD;

        LL_DMA_SetDestDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmatx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmatx->Instance), LL_DMA_DEST_DATAWIDTH_WORD);
        LL_DMA_SetSrcDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmatx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmatx->Instance), LL_DMA_SRC_DATAWIDTH_WORD);

        LL_DMA_SetDestDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmarx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmarx->Instance), LL_DMA_DEST_DATAWIDTH_WORD);
        LL_DMA_SetSrcDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmarx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmarx->Instance), LL_DMA_SRC_DATAWIDTH_WORD);

        MODIFY_REG(hspi->Instance->CFG1, SPI_CFG1_FTHLV, SPI_FIFO_THRESHOLD_04DATA);
    }
    else if ((((xfer_size % sizeof(uint16_t)) == 0u) && ((FALSE == enable_crc) || ((crc_length > SPI_CRC_LENGTH_8BIT) && (crc_length <= SPI_CRC_LENGTH_16BIT)))))
    {
        /* Amount of transfered bytes (including CRC if used) will be a multiple of 2 - we can use half-word as DMA width */
        dma_data_width = LL_DMA_SRC_DATAWIDTH_HALFWORD;

        LL_DMA_SetDestDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmatx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmatx->Instance), LL_DMA_DEST_DATAWIDTH_HALFWORD);
        LL_DMA_SetSrcDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmatx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmatx->Instance), LL_DMA_SRC_DATAWIDTH_HALFWORD);

        LL_DMA_SetDestDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmarx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmarx->Instance), LL_DMA_DEST_DATAWIDTH_HALFWORD);
        LL_DMA_SetSrcDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmarx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmarx->Instance), LL_DMA_SRC_DATAWIDTH_HALFWORD);

        MODIFY_REG(hspi->Instance->CFG1, SPI_CFG1_FTHLV, SPI_FIFO_THRESHOLD_02DATA);
    }
    else
    {
        /* We have to use one byte as the DMA width */
        dma_data_width = LL_DMA_SRC_DATAWIDTH_BYTE;

        LL_DMA_SetDestDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmatx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmatx->Instance), LL_DMA_DEST_DATAWIDTH_BYTE);
        LL_DMA_SetSrcDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmatx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmatx->Instance), LL_DMA_SRC_DATAWIDTH_BYTE);

        LL_DMA_SetDestDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmarx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmarx->Instance), LL_DMA_DEST_DATAWIDTH_BYTE);
        LL_DMA_SetSrcDataWidth(LL_DMA_GET_INSTANCE(hspi->hdmarx->Instance), LL_DMA_GET_CHANNEL(hspi->hdmarx->Instance), LL_DMA_SRC_DATAWIDTH_BYTE);

        MODIFY_REG(hspi->Instance->CFG1, SPI_CFG1_FTHLV, SPI_FIFO_THRESHOLD_01DATA);
    }

    /* Adjust XferCount according to DMA alignment / Data size */
    if (DMA_SRC_DATAWIDTH_WORD == dma_data_width)
    {
        hspi->TxXferCount = (hspi->TxXferCount + (uint16_t)3u) >> 2;
        hspi->RxXferCount = (hspi->RxXferCount + (uint16_t)3u) >> 2;
    }
    else if (DMA_SRC_DATAWIDTH_HALFWORD == dma_data_width)
    {
        hspi->TxXferCount = (hspi->TxXferCount + (uint16_t)1u) >> 1;
        hspi->RxXferCount = (hspi->RxXferCount + (uint16_t)1u) >> 1;
    }
    else
    {
        /* No adjustment needed */
    }

    /* Configure DMA callbacks - transactions are handled by Transaction Control DMA channel, so for Tx/Rx channels we are only interested in Error IRQs */
    hspi->hdmarx->XferHalfCpltCallback = NULL;
    hspi->hdmarx->XferCpltCallback     = NULL;
    hspi->hdmarx->XferAbortCallback    = NULL;
    hspi->hdmarx->XferErrorCallback    = _hal_like_spi_handle_dma_error;

    hspi->hdmatx->XferHalfCpltCallback = NULL;
    hspi->hdmatx->XferCpltCallback     = NULL;
    hspi->hdmatx->XferAbortCallback    = NULL;
    hspi->hdmatx->XferErrorCallback    = _hal_like_spi_handle_dma_error;

    /* Configure DMA Rx channel even if Rx is not expected - a dummy buffer will be used */
    if (rx_buf != NULL)
    {
        /* Allow Dst address increment since we are using real Rx buffer */
        SET_BIT(hspi->hdmarx->Instance->CTR1, DMA_CTR1_DINC);
    }
    else
    {
        /* Disable Dst address increment since we are using dummy buffer for Rx */
        CLEAR_BIT(hspi->hdmarx->Instance->CTR1, DMA_CTR1_DINC);
    }

    /* Enable the Rx DMA channel  */
    errorcode = _hal_like_start_dma_data_channel(hspi->hdmarx, (uint32_t)&hspi->Instance->RXDR, (uint32_t)hspi->pRxBuffPtr, xfer_size, use_irqs);

    /* Check status */
    if (errorcode != HAL_OK)
    {
        /* Update SPI error code */
        SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);

        /* Unlock the process */
        __HAL_UNLOCK(hspi);

        hspi->State = HAL_SPI_STATE_READY;
        errorcode = HAL_ERROR;
        return errorcode;
    }

    /* Enable Rx DMA Request */
    SET_BIT(hspi->Instance->CFG1, SPI_CFG1_RXDMAEN);

    /* Configure DMA Tx channel if any Tx is expected */
    if (hspi->TxXferCount > 0u)
    {
        /* Enable the Tx DMA channel  */
        errorcode = _hal_like_start_dma_data_channel(hspi->hdmatx, (uint32_t)hspi->pTxBuffPtr, (uint32_t)&hspi->Instance->TXDR, xfer_size, use_irqs);

        /* Check status */
        if (errorcode != HAL_OK)
        {
            /* Update SPI error code */
            SET_BIT(hspi->ErrorCode, HAL_SPI_ERROR_DMA);

            /* Unlock the process */
            __HAL_UNLOCK(hspi);

            hspi->State = HAL_SPI_STATE_READY;
            errorcode = HAL_ERROR;
            return errorcode;
        }

        /* Enable Tx DMA Request */
        SET_BIT(hspi->Instance->CFG1, SPI_CFG1_TXDMAEN);
    }

    /* Configure SPI transaction size - since SPI is guranteed to act with 8bit frame size it will be equal to xfer_size */
    MODIFY_REG(hspi->Instance->CR2, SPI_CR2_TSIZE, xfer_size);

    /* Enable the SPI Error Interrupt Bit */
    if (use_irqs != FALSE)
    {
        if (hspi->RxXferCount > 0u)
        {
            /* TxRx or Rx-only mode */
            __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_OVR | SPI_IT_UDR | SPI_IT_FRE | SPI_IT_MODF));
        }
        else
        {
            /* Tx-only mode */
            __HAL_SPI_ENABLE_IT(hspi, (SPI_IT_UDR | SPI_IT_FRE | SPI_IT_MODF));
        }
    }

    /* Enable SPI peripheral */
    __HAL_SPI_ENABLE(hspi);

    /* Do not start the transaction at this point */
    errorcode = HAL_OK;

    return errorcode;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_lptim_peripheral_clock(const LPTIM_TypeDef * const hlptim_instance)
{
    uint32_t lptim_peripheral_clock;

    switch ((uint32_t)(void *)hlptim_instance)
    {
        case (uint32_t)(void *)LPTIM1:
            lptim_peripheral_clock = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_LPTIM1);
            break;

#if defined (LPTIM2)
        case (uint32_t)(void *)LPTIM2:
            lptim_peripheral_clock = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_LPTIM2);
            break;
#endif

        default:
            SID_PAL_LOG_ERROR("LPTIM clock cfg failed - unknown LPTIM peripheral selected");
            lptim_peripheral_clock = 0u;
            break;
    }

    return lptim_peripheral_clock;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline uint32_t _get_spi_peripheral_clock(const SPI_TypeDef * const hspi_instance)
{
    uint32_t spi_peripheral_clock;

    switch ((uint32_t)(void *)hspi_instance)
    {
#if defined (SPI1)
        case (uint32_t)(void *)SPI1:
            spi_peripheral_clock = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI1);
            break;
#endif /* SPI1 */

#if defined (SPI2)
        case (uint32_t)(void *)SPI2:
            spi_peripheral_clock = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI2);
            break;
#endif /* SPI2 */

#if defined (SPI3)
        case (uint32_t)(void *)SPI3:
            spi_peripheral_clock = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI3);
            break;
#endif /* SPI3 */

        default:
            SID_PAL_LOG_ERROR("SPI clock cfg failed - unknown SPI peripheral selected");
            spi_peripheral_clock = 0u;
            break;
    }

    return spi_peripheral_clock;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _nss_lptim_init(serial_bus_spi_ctx * const out_ctx)
{
    sid_error_t err = SID_ERROR_GENERIC;
    const sid_pal_serial_bus_stm32wbaxx_factory_config_t * const cfg = out_ctx->factory_config;

    do
    {
        /* Validate inputs */
        if ((cfg->hlptim == NULL) || (cfg->mx_lptim_init == NULL))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Ensure LPTIM is not initialized already */
        if (cfg->hlptim->State != HAL_LPTIM_STATE_RESET)
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Call MX initialization function */
        cfg->mx_lptim_init();

        /* Ensure important initialization params are valid
         * IMPORTANT: this check can only be performed after calling the mx_lptim_init() because
         *            the Init structure is populated by that call
         */
        SID_PAL_ASSERT(LPTIM_TRIGSOURCE_SOFTWARE == cfg->hlptim->Init.Trigger.Source);
        SID_PAL_ASSERT(LPTIM_UPDATE_IMMEDIATE == cfg->hlptim->Init.UpdateMode);
        SID_PAL_ASSERT(LPTIM_COUNTERSOURCE_INTERNAL == cfg->hlptim->Init.CounterSource);
        SID_PAL_ASSERT(LPTIM_INPUT1SOURCE_GPIO == cfg->hlptim->Init.Input1Source);
        SID_PAL_ASSERT(LPTIM_INPUT2SOURCE_GPIO == cfg->hlptim->Init.Input2Source);
        SID_PAL_ASSERT(UINT16_MAX == cfg->hlptim->Init.Period); /* ARR register value shall be strictly > greater than any CCx value - ensure it is kept at maximum to eliminate the need to connfigure it dynamically */
        SID_PAL_ASSERT(0u == cfg->hlptim->Init.RepetitionCounter);

        /* Check if DMA channel is linked after calling the mx_lptim_init() */
        if (NULL == cfg->hlptim->hdma[LPTIM_DMA_ID_CC1])
        {
            SID_PAL_LOG_ERROR("LPTIM peripheral has no linked DMA CC1 channel");
            err = SID_ERROR_NULL_POINTER;
            break;
        }

        /* Apply settings that are constant during the operation */
        /* Set WAVE bit to enable the set once mode */
        LL_LPTIM_SetWaveform(cfg->hlptim->Instance, LL_LPTIM_OUTPUT_WAVEFORM_SETONCE);

        /* Enable DMA tirgger for both channels */
        LL_LPTIM_EnableIT_CC1(cfg->hlptim->Instance);
        LL_LPTIM_EnableIT_CC2(cfg->hlptim->Instance);

        /* Enable LPTIM signal on the corresponding output pin */
        LL_LPTIM_CC_EnableChannel(cfg->hlptim->Instance, LL_LPTIM_CHANNEL_CH1);
        LL_LPTIM_CC_EnableChannel(cfg->hlptim->Instance, LL_LPTIM_CHANNEL_CH2);

        /* Everything is ok till now */
        err = SID_ERROR_NONE;

        /* Store values for DMA triggers */
        switch ((uint32_t)cfg->hlptim->Instance)
        {
            case (uint32_t)LPTIM1:
                out_ctx->dma_periph_cfg.dma_trigger_lptimx_ch1_val = LL_GPDMA1_TRIGGER_LPTIM1_CH1;
                out_ctx->dma_periph_cfg.dma_trigger_lptimx_ch2_val = LL_GPDMA1_TRIGGER_LPTIM1_CH2;
                break;

#if defined (LPTIM2)
            case (uint32_t)LPTIM2:
                out_ctx->dma_periph_cfg.dma_trigger_lptimx_ch1_val = LL_GPDMA1_TRIGGER_LPTIM2_CH1;
                out_ctx->dma_periph_cfg.dma_trigger_lptimx_ch2_val = LL_GPDMA1_TRIGGER_LPTIM2_CH2;
                break;
#endif

            default:
                SID_PAL_LOG_ERROR("DMA trigger setup failed - unknown LPTIM peripheral selected");
                out_ctx->dma_periph_cfg.dma_trigger_lptimx_ch1_val = 0u;
                out_ctx->dma_periph_cfg.dma_trigger_lptimx_ch2_val = 0u;
                err = SID_ERROR_INVALID_ARGS;
                break;
        }

        /* Check if DMA triggers were configured successfully */
        if (err != SID_ERROR_NONE)
        {
            break;
        }

        /* Initialize DMA channel with the default settings */
        LL_DMA_InitTypeDef DMA_InitStruct;
        DMA_Channel_TypeDef * const hdma_instance = cfg->hlptim->hdma[LPTIM_DMA_ID_CC1]->Instance;

        SID_PAL_ASSERT((DMA_LLN_REGION_START & 0xFFFF0000u) == (DMA_LLN_REGION_END & 0xFFFF0000u));

        LL_DMA_StructInit(&DMA_InitStruct);
        DMA_InitStruct.SrcAddress = 0u;
        DMA_InitStruct.DestAddress = 0u;
        DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_MEMORY;
        DMA_InitStruct.BlkHWRequest = LL_DMA_HWREQUEST_SINGLEBURST;
        DMA_InitStruct.SrcBurstLength = 1;
        DMA_InitStruct.DestBurstLength = 1;
        DMA_InitStruct.SrcDataWidth = LL_DMA_SRC_DATAWIDTH_WORD;
        DMA_InitStruct.DestDataWidth = LL_DMA_DEST_DATAWIDTH_WORD;
        DMA_InitStruct.Priority = LL_DMA_LOW_PRIORITY_HIGH_WEIGHT;
        DMA_InitStruct.BlkDataLength = 0u;
        DMA_InitStruct.TriggerMode = LL_DMA_TRIGM_BLK_TRANSFER;
        DMA_InitStruct.TriggerPolarity = LL_DMA_TRIG_POLARITY_MASKED;
        DMA_InitStruct.TransferEventMode = LL_DMA_TCEM_LAST_LLITEM_TRANSFER;
        DMA_InitStruct.SrcAllocatedPort = LL_DMA_SRC_ALLOCATED_PORT0;
        DMA_InitStruct.DestAllocatedPort = LL_DMA_DEST_ALLOCATED_PORT0;
        DMA_InitStruct.LinkAllocatedPort = LL_DMA_LINK_ALLOCATED_PORT0;
        DMA_InitStruct.LinkStepMode = LL_DMA_LSM_FULL_EXECUTION;
        DMA_InitStruct.LinkedListBaseAddr = (DMA_LLN_REGION_START & 0xFFFF0000u);
        DMA_InitStruct.LinkedListAddrOffset = 0u;

        uint32_t ll_err = LL_DMA_Init(LL_DMA_GET_INSTANCE(hdma_instance), LL_DMA_GET_CHANNEL(hdma_instance), &DMA_InitStruct);
        if ((uint32_t)SUCCESS != ll_err)
        {
            SID_PAL_LOG_ERROR("Failed to initialize SPI Transaction Control DMA channel. Error code %u", (uint32_t)ll_err);
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _spi_peripheral_init(serial_bus_spi_ctx * const out_ctx)
{
    sid_error_t err = SID_ERROR_GENERIC;
    const sid_pal_serial_bus_stm32wbaxx_factory_config_t * const cfg = out_ctx->factory_config;

    do
    {
        /* Validate inputs */
        if ((out_ctx == NULL) || (cfg->hspi == NULL) || (cfg->mx_spi_init == NULL))
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        /* Ensure SPI peripheral is not initialized already */
        if (cfg->hspi->State != HAL_SPI_STATE_RESET)
        {
            err = SID_ERROR_INVALID_STATE;
            break;
        }

        /* Call MX initialization function */
        cfg->mx_spi_init();

        /* Ensure important initialization params are valid
         * IMPORTANT: this check can only be performed after calling the mx_lptim_init() because
         *            the Init structure is populated by that call
         */
        SID_PAL_ASSERT(SPI_DATASIZE_8BIT == cfg->hspi->Init.DataSize);
        SID_PAL_ASSERT(SPI_DIRECTION_2LINES == cfg->hspi->Init.Direction);
        SID_PAL_ASSERT(SPI_MASTER_RX_AUTOSUSP_DISABLE == cfg->hspi->Init.MasterReceiverAutoSusp);
        SID_PAL_ASSERT(SPI_MODE_MASTER == cfg->hspi->Init.Mode);
        SID_PAL_ASSERT(SPI_NSS_PULSE_DISABLE == cfg->hspi->Init.NSSPMode);
        SID_PAL_ASSERT(SPI_NSS_SOFT == cfg->hspi->Init.NSS);
        SID_PAL_ASSERT(SPI_RDY_MASTER_MANAGEMENT_INTERNALLY == cfg->hspi->Init.ReadyMasterManagement);
        SID_PAL_ASSERT(SPI_TIMODE_DISABLE == cfg->hspi->Init.TIMode);

        /* Check if DMA channels are linked after calling the mx_spi_init() */
        if (NULL == cfg->hspi->hdmatx)
        {
            SID_PAL_LOG_ERROR("SPI peripheral has no linked DMA Tx channel");
            err = SID_ERROR_NULL_POINTER;
            break;
        }
        if (NULL == cfg->hspi->hdmarx)
        {
            SID_PAL_LOG_ERROR("SPI peripheral has no linked DMA Rx channel");
            err = SID_ERROR_NULL_POINTER;
            break;
        }

        /* Validate DMA channel settings */
        if ((cfg->hspi->hdmatx->Mode & DMA_LINKEDLIST) != DMA_NORMAL)
        {
            SID_PAL_LOG_ERROR("SPI DMA Tx channel shall be configured in normal mode, not in a linked-list one");
            err = SID_ERROR_INCOMPATIBLE_PARAMS;
            break;
        }
        if ((cfg->hspi->hdmarx->Mode & DMA_LINKEDLIST) != DMA_NORMAL)
        {
            SID_PAL_LOG_ERROR("SPI DMA Rx channel shall be configured in normal mode, not in a linked-list one");
            err = SID_ERROR_INCOMPATIBLE_PARAMS;
            break;
        }

        /* Compute DMA Tx/Rx channel values for DMA trigger */
        out_ctx->dma_periph_cfg.dma_trigger_spi_dmatx_ch_val = LL_GPDMA1_TRIGGER_GPDMA1_CH0_TCF + LL_DMA_GET_CHANNEL(cfg->hspi->hdmatx->Instance);
        out_ctx->dma_periph_cfg.dma_trigger_spi_dmarx_ch_val = LL_GPDMA1_TRIGGER_GPDMA1_CH0_TCF + LL_DMA_GET_CHANNEL(cfg->hspi->hdmarx->Instance);

        /* Everything is ok */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _connect_adjacent_ll_nodes(uint32_t ** const prev_node, uint32_t * const prev_node_cllr_idx, uint32_t ** const next_node, const uint32_t next_node_cllr_idx)
{
    /* Protect from systematic failures in this module */
    SID_PAL_ASSERT(prev_node != NULL);
    SID_PAL_ASSERT(next_node != NULL);
    SID_PAL_ASSERT(*next_node != NULL);

    if (NULL == *prev_node)
    {
        /* This is the first node in the list - just arrange the pointers */
    }
    else
    {
        /* We have the nodes in the list - proceed with linking */
        LL_DMA_ConnectLinkNode((LL_DMA_LinkNodeTypeDef *)(void *)(*prev_node), *prev_node_cllr_idx,
                               (LL_DMA_LinkNodeTypeDef *)(void *)(*next_node), next_node_cllr_idx);
    }

    /* Advance the pointer to the next node */
    *prev_node = *next_node;
    *prev_node_cllr_idx = next_node_cllr_idx;
    *next_node += (next_node_cllr_idx + 1u);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _build_spi_transaction_dma_ll_skeleton(const serial_bus_spi_ctx * const bus_ctx, spi_client_context_t * const client_ctx)
{
    SPI_TypeDef * const hspi_instance = bus_ctx->factory_config->hspi->Instance;
    LPTIM_TypeDef * const hlptim_instance = bus_ctx->factory_config->hlptim->Instance;
    LL_DMA_InitNodeTypeDef lln_init;
    LL_DMA_LinkNodeTypeDef * ll_node_prev = NULL;
    LL_DMA_LinkNodeTypeDef * ll_node_next = (LL_DMA_LinkNodeTypeDef *)(void *)&client_ctx->dma_lln_buffer[SPI_DMA_LLN_INITIAL_CLLR_VALS_OFFSET + 1u];
    uint32_t prev_ll_node_llr_idx;
    uint32_t modified_regs;
    uint32_t total_regs = (SPI_DMA_LLN_INITIAL_CLLR_VALS_OFFSET + 1u); /* 1 place is reserved for the initial CLLR values */
    sid_error_t err = SID_ERROR_GENERIC;

    do
    {
        /* LLN[0] - Write SPI CSTART bit */
        LL_DMA_NodeStructInit(&lln_init);
        modified_regs = 0u;

        /* CTR1 - no updates */

        /* CTR2 - updated dynamically for each transaction */
        SID_PAL_ASSERT(SPI_DMA_LLN_WRITE_SPI_CSTART_CTR2_OFFSET == (modified_regs + total_regs)); /* Ensure buffer offset is defined properly */
        lln_init.TransferEventMode = LL_DMA_TCEM_LAST_LLITEM_TRANSFER;
        lln_init.TriggerMode       = LL_DMA_TRIGM_BLK_TRANSFER;
        lln_init.TriggerPolarity   = LL_DMA_TRIG_POLARITY_MASKED;
        modified_regs++;

        /* CBR1 */
        lln_init.BlkDataLength = sizeof(hspi_instance->CR1);
        modified_regs++;

        /* CSAR */
        lln_init.SrcAddress = (uint32_t)(void *)&client_ctx->peripherals_cfg.dma_regs_storage.spi_cr1_reg_val_start;
        modified_regs++;

        /* CDAR */
        lln_init.DestAddress = (uint32_t)&(hspi_instance->CR1);
        modified_regs++;

        /* CLLR */
        modified_regs++;

        /* Buffer space check */
        total_regs += modified_regs;
        if (total_regs > client_ctx->dma_lln_buf_length)
        {
            SID_PAL_LOG_ERROR("Not enough space to fit DMA LLN for SPI CSTART write operation");
            err = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* DMA channel regs to be updated */
        lln_init.UpdateRegisters = LL_DMA_UPDATE_CTR2 | LL_DMA_UPDATE_CBR1 | LL_DMA_UPDATE_CSAR | LL_DMA_UPDATE_CDAR | LL_DMA_UPDATE_CLLR;
        client_ctx->dma_lln_buffer[SPI_DMA_LLN_INITIAL_CLLR_VALS_OFFSET] = lln_init.UpdateRegisters; /* Store the list of modified values to automate transaction config */

        /* Create node and advance the pointers */
        LL_DMA_CreateLinkNode(&lln_init, ll_node_next);
        _connect_adjacent_ll_nodes((void*)&ll_node_prev, &prev_ll_node_llr_idx, (void*)&ll_node_next, (modified_regs - 1u));
        /* LL node configured ************************/

        /* LLN[1] - Disable LPTIM after SPI start to reset DMA requests */
        LL_DMA_NodeStructInit(&lln_init);
        modified_regs = 0u;

        /* CTR1 - no updates */

        /* CTR2 */
        lln_init.TransferEventMode = LL_DMA_TCEM_LAST_LLITEM_TRANSFER;
        lln_init.TriggerMode       = LL_DMA_TRIGM_BLK_TRANSFER;
        lln_init.TriggerPolarity   = LL_DMA_TRIG_POLARITY_MASKED;
        modified_regs++;

        /* CBR1 */
        lln_init.BlkDataLength = sizeof(hlptim_instance->CR);
        modified_regs++;

        /* CSAR */
        lln_init.SrcAddress = (uint32_t)(void *)&client_ctx->peripherals_cfg.dma_regs_storage.lptim_cr_val_dis;
        modified_regs++;

        /* CDAR */
        lln_init.DestAddress = (uint32_t)&(hlptim_instance->CR);
        modified_regs++;

        /* CLLR */
        modified_regs++;

        /* Buffer space check */
        total_regs += modified_regs;
        if (total_regs > client_ctx->dma_lln_buf_length)
        {
            SID_PAL_LOG_ERROR("Not enough space to fit DMA LLN for disabling LPTIM");
            err = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* DMA channel regs to be updated */
        lln_init.UpdateRegisters = LL_DMA_UPDATE_CTR2 | LL_DMA_UPDATE_CBR1 | LL_DMA_UPDATE_CSAR | LL_DMA_UPDATE_CDAR | LL_DMA_UPDATE_CLLR;

        /* Create node and advance the pointers */
        LL_DMA_CreateLinkNode(&lln_init, ll_node_next);
        _connect_adjacent_ll_nodes((void*)&ll_node_prev, &prev_ll_node_llr_idx, (void*)&ll_node_next, (modified_regs - 1u));
        /* LL node configured ************************/

        /* LLN[2] - Re-enable LPTIM to configure it */
        LL_DMA_NodeStructInit(&lln_init);
        modified_regs = 0u;

        /* CTR1 - no updates */

        /* CTR2 - no updates */

        /* CBR1 */
        lln_init.BlkDataLength = sizeof(hlptim_instance->CR);
        modified_regs++;

        /* CSAR */
        lln_init.SrcAddress = (uint32_t)(void *)&client_ctx->peripherals_cfg.dma_regs_storage.lptim_cr_val_en;
        modified_regs++;

        /* CDAR - no updates */

        /* CLLR */
        modified_regs++;

        /* Buffer space check */
        total_regs += modified_regs;
        if (total_regs > client_ctx->dma_lln_buf_length)
        {
            SID_PAL_LOG_ERROR("Not enough space to fit DMA LLN for re-enabling LPTIM");
            err = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* DMA channel regs to be updated */
        lln_init.UpdateRegisters = LL_DMA_UPDATE_CBR1 | LL_DMA_UPDATE_CSAR | LL_DMA_UPDATE_CLLR;

        /* Create node and advance the pointers */
        LL_DMA_CreateLinkNode(&lln_init, ll_node_next);
        _connect_adjacent_ll_nodes((void*)&ll_node_prev, &prev_ll_node_llr_idx, (void*)&ll_node_next, (modified_regs - 1u));
        /* LL node configured ************************/

        /* LLN[3] - Reconfigure LPTIM CCR value */
        LL_DMA_NodeStructInit(&lln_init);
        modified_regs = 0u;

        /* CTR1 - no updates */

        /* CTR2 - no updates */

        /* CBR1 */
        lln_init.BlkDataLength = sizeof(hlptim_instance->CCR1);
        modified_regs++;

        /* CSAR */
        lln_init.SrcAddress = (uint32_t)(void *)&client_ctx->peripherals_cfg.dma_regs_storage.lptim_ccr1_val_xfer_end;
        modified_regs++;

        /* CDAR */
        lln_init.DestAddress = (uint32_t)&(hlptim_instance->CCR1);
        modified_regs++;

        /* CLLR */
        modified_regs++;

        /* Buffer space check */
        total_regs += modified_regs;
        if (total_regs > client_ctx->dma_lln_buf_length)
        {
            SID_PAL_LOG_ERROR("Not enough space to fit DMA LLN for reconfiguring LPTIM");
            err = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* DMA channel regs to be updated */
        lln_init.UpdateRegisters = LL_DMA_UPDATE_CBR1 | LL_DMA_UPDATE_CSAR | LL_DMA_UPDATE_CDAR | LL_DMA_UPDATE_CLLR;

        /* Create node and advance the pointers */
        LL_DMA_CreateLinkNode(&lln_init, ll_node_next);
        _connect_adjacent_ll_nodes((void*)&ll_node_prev, &prev_ll_node_llr_idx, (void*)&ll_node_next, (modified_regs - 1u));
        /* LL node configured ************************/

        /* LLN[4] - Start LPTIM upon SPI DMA Tx/Rx end */
        LL_DMA_NodeStructInit(&lln_init);
        modified_regs = 0u;

        /* CTR1 - no updates */

        /* CTR2 */
        lln_init.TransferEventMode = LL_DMA_TCEM_LAST_LLITEM_TRANSFER;
        lln_init.TriggerMode       = LL_DMA_TRIGM_BLK_TRANSFER;
        lln_init.TriggerPolarity   = LL_DMA_TRIG_POLARITY_RISING;
        lln_init.TriggerSelection  = bus_ctx->dma_periph_cfg.dma_trigger_spi_dmarx_ch_val;
        modified_regs++;

        /* CBR1 */
        lln_init.BlkDataLength = sizeof(hlptim_instance->CR);
        modified_regs++;

        /* CSAR */
        lln_init.SrcAddress = (uint32_t)(void *)&client_ctx->peripherals_cfg.dma_regs_storage.lptim_cr_val_start;
        modified_regs++;

        /* CDAR */
        lln_init.DestAddress = (uint32_t)&(hlptim_instance->CR);
        modified_regs++;

        /* CLLR */
        modified_regs++;

        /* Buffer space check */
        total_regs += modified_regs;
        if (total_regs > client_ctx->dma_lln_buf_length)
        {
            SID_PAL_LOG_ERROR("Not enough space to fit DMA LLN for disabling LPTIM");
            err = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* DMA channel regs to be updated */
        lln_init.UpdateRegisters = LL_DMA_UPDATE_CTR2 | LL_DMA_UPDATE_CBR1 | LL_DMA_UPDATE_CSAR | LL_DMA_UPDATE_CDAR | LL_DMA_UPDATE_CLLR;

        /* Create node and advance the pointers */
        LL_DMA_CreateLinkNode(&lln_init, ll_node_next);
        _connect_adjacent_ll_nodes((void*)&ll_node_prev, &prev_ll_node_llr_idx, (void*)&ll_node_next, (modified_regs - 1u));
        /* LL node configured ************************/

        /* LLN[5] - NSS deactivation */
        LL_DMA_NodeStructInit(&lln_init);
        modified_regs = 0u;

        /* CTR1 - no updates */

        /* CTR2 */
        SID_PAL_ASSERT(SPI_DMA_LLN_NSS_EOT_TRIGGER_SRC_OFFSET == (modified_regs + total_regs)); /* Ensure buffer offset is defined properly */
        lln_init.TransferEventMode = LL_DMA_TCEM_LAST_LLITEM_TRANSFER;
        lln_init.TriggerMode       = LL_DMA_TRIGM_BLK_TRANSFER;
        lln_init.TriggerPolarity   = LL_DMA_TRIG_POLARITY_MASKED;
        /* lln_init.TriggerSelection is configured dynamically */
        modified_regs++;

        /* CBR1 */
        lln_init.BlkDataLength = sizeof(client_ctx->peripherals_cfg.nss_gpio_port->BSRR);
        modified_regs++;

        /* CSAR */
        lln_init.SrcAddress = (uint32_t)(void *)&client_ctx->peripherals_cfg.nss_gpio_pin;
        modified_regs++;

        /* CDAR */
        SID_PAL_ASSERT(SPI_DMA_LLN_NSS_EOT_DST_ADDR_OFFSET == (modified_regs + total_regs)); /* Ensure buffer offset is defined properly */
        lln_init.DestAddress = (uint32_t)&(client_ctx->peripherals_cfg.nss_gpio_port->BSRR);
        modified_regs++;

        /* CLLR */
        modified_regs++;

        /* Buffer space check */
        total_regs += modified_regs;
        if (total_regs > client_ctx->dma_lln_buf_length)
        {
            SID_PAL_LOG_ERROR("Not enough space to fit DMA LLN for disabling LPTIM");
            err = SID_ERROR_BUFFER_OVERFLOW;
            break;
        }

        /* DMA channel regs to be updated */
        lln_init.UpdateRegisters = LL_DMA_UPDATE_CTR2 | LL_DMA_UPDATE_CBR1 | LL_DMA_UPDATE_CSAR | LL_DMA_UPDATE_CDAR | LL_DMA_UPDATE_CLLR;

        /* Create node and advance the pointers */
        LL_DMA_CreateLinkNode(&lln_init, ll_node_next);
        _connect_adjacent_ll_nodes((void*)&ll_node_prev, &prev_ll_node_llr_idx, (void*)&ll_node_next, (modified_regs - 1u));
        /* LL node configured ************************/

        /* IMPORTANT: always check the lines below are applied to the very last DMA linked list node, otherwise the linked list
         * sequence will not be processed properly since DMA won't stop
         */

        /* Indicate that was the last node in the list */
        ll_node_prev->LinkRegisters[prev_ll_node_llr_idx] = 0u;

        /* Done with configuration */
        err = SID_ERROR_NONE;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _compute_peripheral_reg_values(const serial_bus_spi_ctx * const bus_ctx, const struct sid_pal_serial_bus_client * const client, spi_client_context_t * const client_ctx)
{
    dma_regs_storage_t * const regs_storage = &client_ctx->peripherals_cfg.dma_regs_storage;

    /* SPI_CR1 values */
    regs_storage->spi_cr1_reg_val_start     = 0u; /* Actual value is updated dynamically for each transaction */

    /* LPTIM_CR values */
    regs_storage->lptim_cr_val_en           = LPTIM_CR_ENABLE;
    regs_storage->lptim_cr_val_start        = (LPTIM_CR_SNGSTRT | LPTIM_CR_ENABLE);
    regs_storage->lptim_cr_val_dis          = 0u; /* Completely disables LPTIM */

    /* LPTIM_CCRx values */
    /* NSS-to-NSS delay */
    regs_storage->lptim_ccr1_val_nss_to_nss = LPTIM_RECALCULATE_CYCLES_FROM_REFERENCE(client_ctx->peripherals_cfg.nss_lptim_clock_hz,
                                                                                      client->speed_hz,
                                                                                      client_ctx->peripherals_cfg.nss_to_nss_cycles);

    /* NSS-to-SCK delay */
    regs_storage->lptim_ccr2_val_nss_to_sck = LPTIM_RECALCULATE_CYCLES_FROM_REFERENCE(client_ctx->peripherals_cfg.nss_lptim_clock_hz,
                                                                                      client->speed_hz,
                                                                                      client_ctx->peripherals_cfg.nss_to_sck_cycles);
    if (regs_storage->lptim_ccr2_val_nss_to_sck < SPI_LPTIM_NSS_TO_SCK_MIN_DELAY_CYCLES(client_ctx->peripherals_cfg.nss_lptim_clock_hz))
    {
        /* User wants way too short delay, HW cannot keep it. Override with the minimum possible delay */
        SID_PAL_LOG_WARNING("SPI NSS-to-SCK delay is too short. Minimum delay of %u SCK cycles will be used", SPI_LPTIM_NSS_TO_SCK_MIN_DELAY_CYCLES(client->speed_hz));
        client_ctx->peripherals_cfg.nss_to_sck_cycles = SPI_LPTIM_NSS_TO_SCK_MIN_DELAY_CYCLES(client_ctx->peripherals_cfg.nss_lptim_clock_hz);
    }
    /* Calculate the final value with the adjustment */
    regs_storage->lptim_ccr2_val_nss_to_sck = regs_storage->lptim_ccr1_val_nss_to_nss + regs_storage->lptim_ccr2_val_nss_to_sck
                                              - SPI_LPTIM_NSS_TO_SCK_COMPENSATORY_CYCLES(client_ctx->peripherals_cfg.nss_lptim_clock_hz);

    /* SCK-to-NSS delay */
    regs_storage->lptim_ccrx_val_sck_to_nss = LPTIM_RECALCULATE_CYCLES_FROM_REFERENCE(client_ctx->peripherals_cfg.nss_lptim_clock_hz,
                                                                                      client->speed_hz,
                                                                                      client_ctx->peripherals_cfg.sck_to_nss_cycles);
    if (client_ctx->peripherals_cfg.crc_polynomial != 0u)
    {
        /* CRC is used - add an additional SCK-to-NSS delay to allow CRC transfer */
        regs_storage->lptim_ccrx_val_sck_to_nss += LPTIM_RECALCULATE_CYCLES_FROM_REFERENCE(client_ctx->peripherals_cfg.nss_lptim_clock_hz,
                                                                                           client->speed_hz,
                                                                                           SPI_HAL_CRC_SIZE_TO_INT_VAL( client_ctx->peripherals_cfg.crc_length));
    }
    regs_storage->lptim_ccrx_val_sck_to_nss_dma_rx_compensation = LPTIM_RECALCULATE_CYCLES_FROM_REFERENCE(client_ctx->peripherals_cfg.nss_lptim_clock_hz,
                                                                                                          SPI_LPTIM_REFERENCE_CLOCK_HZ,
                                                                                                          SPI_LPTIM_SCK_TO_NSS_DMA_RX_COMPENSATORY_CYCLES_24MHZ_REFERENCE);

    regs_storage->lptim_ccr1_val_xfer_end   = 0u; /* This value is updated dynamically for each transaction */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _dma_lln_buf_alloc(spi_client_context_t * const client_ctx)
{
    client_ctx->dma_lln_buffer = NULL;
    client_ctx->dma_lln_buf_length = 0u;

    sid_pal_enter_critical_region();
    for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(dma_lln_buf); i++)
    {
        if (SPI_DMA_LLN_BUF_FREE_WATERMARK == dma_lln_buf[i][0])
        {
            dma_lln_buf[i][0] = 0u;
            client_ctx->dma_lln_buffer = dma_lln_buf[i];
            client_ctx->dma_lln_buf_length = SID_STM32_UTIL_ARRAY_SIZE(dma_lln_buf[i]);
            break;
        }
    }
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _dma_lln_buf_free(spi_client_context_t * const client_ctx)
{
    sid_pal_enter_critical_region();
    for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(dma_lln_buf); i++)
    {
        if (dma_lln_buf[i] == client_ctx->dma_lln_buffer)
        {
            client_ctx->dma_lln_buffer = NULL;
            client_ctx->dma_lln_buf_length = 0u;
            dma_lln_buf[i][0] = SPI_DMA_LLN_BUF_FREE_WATERMARK;
        }
    }
    sid_pal_exit_critical_region();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline spi_client_context_t * _find_spi_client_ctx(const serial_bus_spi_ctx * const bus_ctx, const struct sid_pal_serial_bus_client * const client)
{
    spi_client_context_t * found_ctx = NULL;

    SID_PAL_ASSERT(bus_ctx != NULL);
    SID_PAL_ASSERT(client != NULL);

    if (LST_is_empty((tListNode *)&bus_ctx->client_contexts) == FALSE)
    {
        /* Iterate over the list o client contexts and try to find a matching one */
        tListNode * iterator;
        LST_get_next_node((tListNode *)&bus_ctx->client_contexts, &iterator);

        while (iterator != &(bus_ctx->client_contexts))
        {
            /* Check if this node matches the client */
            spi_client_context_t * const client_ctx = containerof(iterator, spi_client_context_t, node);
            if (client->client_selector == client_ctx->client_selector)
            {
                /* We've found what we were looking for */
                found_ctx = client_ctx;
                break;
            }
            else
            {
                /* Proceed to the next list node */
                LST_get_next_node(iterator, &iterator);
            }
        }
    }

    return found_ctx;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_error_t _create_spi_client_ctx(const serial_bus_spi_ctx * const bus_ctx, const struct sid_pal_serial_bus_client * const client, spi_client_context_t ** const out_client_ctx)
{
    sid_error_t err = SID_ERROR_GENERIC;
    spi_client_context_t * new_client_ctx = NULL;

    SID_PAL_ASSERT(bus_ctx != NULL);
    SID_PAL_ASSERT(bus_ctx->factory_config != NULL);
    SID_PAL_ASSERT(client != NULL);

    do
    {
        /* Allocate RAM for the context itself */
        new_client_ctx = malloc(sizeof(spi_client_context_t));
        if (NULL == new_client_ctx)
        {
            SID_PAL_LOG_ERROR("Unable to allocate memory for SPI client context");
            err = SID_ERROR_OOM;
            break;
        }

        /* Set critical default values for the newly allocated context */
        new_client_ctx->dma_lln_buffer = NULL;
        new_client_ctx->peripherals_cfg.nss_gpio_port = NULL;

        /* Setup the context based on the client configuration */
        new_client_ctx->client_selector = client->client_selector;
        /*----------------------------------------------------------------------------*/

        /* SPI baud rate settings */
        /* Store the SPI peripheral config to the context */
        uint32_t spi_baud_rate_prescaler;
        uint32_t spi_peripheral_clock = _get_spi_peripheral_clock(bus_ctx->factory_config->hspi->Instance);
        if (0u == spi_peripheral_clock)
        {
            SID_PAL_LOG_ERROR("Unable to determine SPI instance base clock");
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        if (client->speed_hz < spi_peripheral_clock)
        {
            /* Calculate the nearest prescaler that gives the SCK clock not higher than requested in the config */
            spi_baud_rate_prescaler = SID_STM32_UTIL_ROUNDUP_NEXT_POW_2_EXP((spi_peripheral_clock + (client->speed_hz - 1u))/ client->speed_hz);

            /* Warn the user if we did not get a perfect match - SPI will run on a lower frequency than requested */
            if ((spi_peripheral_clock >> spi_baud_rate_prescaler) != client->speed_hz)
            {
                SID_PAL_LOG_WARNING("Unable to set SPI SCK frequency to %u. Using nearest possible value of %u", client->speed_hz, (spi_peripheral_clock >> spi_baud_rate_prescaler));
            }

            /* Translate into SPI register value */
            spi_baud_rate_prescaler = SPI_POW_2_EXP_TO_HAL_PRESCALER_VAL(spi_baud_rate_prescaler);
        }
        else if (client->speed_hz == spi_peripheral_clock)
        {
            /* Requested client speed matches SPI peripheral clock */
            spi_baud_rate_prescaler = SPI_BAUDRATEPRESCALER_BYPASS;
        }
        else /* if (client->speed_hz > spi_peripheral_clock) */
        {
            /* Requested client speed is above the peripheral clock, we can't go that high */
            SID_PAL_LOG_WARNING("Requested SPI speed is above maximum. Clock will be limited to %uHz", spi_peripheral_clock);
            spi_baud_rate_prescaler = SPI_BAUDRATEPRESCALER_BYPASS;
        }
        new_client_ctx->peripherals_cfg.spi_prescaler = spi_baud_rate_prescaler;
        /*----------------------------------------------------------------------------*/

        /* NSS pin init */
        /* Decode port and pin number from the client config */
        new_client_ctx->peripherals_cfg.nss_gpio_port = NUM_TO_GPIO_ST_PORT(client->client_selector);

        /* Proceed with GPIO and LPTIM configuration if NSS is used for this client */
        if (new_client_ctx->peripherals_cfg.nss_gpio_port != NULL)
        {
            GPIO_InitTypeDef gpio_init_struct = {0};

            /* Decode port and pin number from the client config */
            new_client_ctx->peripherals_cfg.nss_gpio_pin = NUM_TO_GPIO_PIN(client->client_selector);

            /* Set NSS pin to inactive state before configuring it as output */
            //TODO: possibly support NSS polarity selection by writing into BRR or BSRR
            new_client_ctx->peripherals_cfg.nss_gpio_port->BSRR = new_client_ctx->peripherals_cfg.nss_gpio_pin;

            /* Set GPIO pin as output */
            gpio_init_struct.Pin = new_client_ctx->peripherals_cfg.nss_gpio_pin;
            gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
            gpio_init_struct.Pull = GPIO_NOPULL;
            gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(new_client_ctx->peripherals_cfg.nss_gpio_port, &gpio_init_struct);
            /*----------------------------------------------------------------------------*/

            /* LPTIM settings */
            SID_PAL_ASSERT(bus_ctx->factory_config->hlptim != NULL);

            /* Calculate the nearest prescaler for LPTIM that gives the SCK clock not higher than requested in the config */
            const uint32_t nss_lptim_base_clock_hz = _get_lptim_peripheral_clock(bus_ctx->factory_config->hlptim->Instance);
            uint32_t lptim_prescaler = SID_STM32_UTIL_ROUNDUP_NEXT_POW_2_EXP((nss_lptim_base_clock_hz + (client->speed_hz - 1u)) / client->speed_hz);

            if (lptim_prescaler >= SPI_LPTIM_OVERSAMPLING_POW_2_EXP)
            {
                /* Run LPTIM faster than SPI if possible */
                lptim_prescaler -= SPI_LPTIM_OVERSAMPLING_POW_2_EXP;
            }

            /* Ensure the calculated prescaler is within the limits */
            SID_PAL_ASSERT(lptim_prescaler < 8u);

            /* Warn the user if we did not get a perfect match - SPI will run on a lower frequency than requested */
            if ((nss_lptim_base_clock_hz >> lptim_prescaler) != (client->speed_hz << SPI_LPTIM_OVERSAMPLING_POW_2_EXP))
            {
                SID_PAL_LOG_WARNING("Unable to set SPI NSS control frequency to %u. Using nearest possible value of %u", (client->speed_hz << SPI_LPTIM_OVERSAMPLING_POW_2_EXP), (nss_lptim_base_clock_hz >> lptim_prescaler));
            }

            new_client_ctx->peripherals_cfg.nss_lptim_clock_hz = (nss_lptim_base_clock_hz >> lptim_prescaler);

            /* Translate into LPTIM register value */
            lptim_prescaler = LPTIM_POW_2_EXP_TO_HAL_PRESCALER_VAL(lptim_prescaler);
            new_client_ctx->peripherals_cfg.nss_lptim_prescaler = lptim_prescaler;
        }
        /*----------------------------------------------------------------------------*/

        if (client->client_selector_extension != NULL)
        {
            const sid_pal_serial_bus_stm32wbaxx_transaction_config_t * const xfer_cfg = (sid_pal_serial_bus_stm32wbaxx_transaction_config_t *)client->client_selector_extension;

            /* SPI CRC config */
            new_client_ctx->peripherals_cfg.crc_polynomial    = xfer_cfg->crc_polynomial;
            new_client_ctx->peripherals_cfg.crc_length        = xfer_cfg->crc_length;
            new_client_ctx->peripherals_cfg.crc_init_pattern  = xfer_cfg->crc_init_pattern;

            /* NSS timings */
            new_client_ctx->peripherals_cfg.nss_to_sck_cycles = xfer_cfg->nss_to_sck_cycles;
            new_client_ctx->peripherals_cfg.sck_to_nss_cycles = xfer_cfg->sck_to_nss_cycles;
            new_client_ctx->peripherals_cfg.nss_to_nss_cycles = xfer_cfg->nss_to_nss_cycles;
        }
        else
        {
            SID_PAL_LOG_DEBUG("No SPI client selector extension supplied - assuming default values");

            /* No CRC by default */
            new_client_ctx->peripherals_cfg.crc_polynomial    = 0u;
            new_client_ctx->peripherals_cfg.crc_length        = 0u;

            /* NSS timings */
            new_client_ctx->peripherals_cfg.nss_to_sck_cycles = 0u;
            new_client_ctx->peripherals_cfg.sck_to_nss_cycles = 0u;
            new_client_ctx->peripherals_cfg.nss_to_nss_cycles = 0u;
        }

        /*----------------------------------------------------------------------------*/

        if (new_client_ctx->peripherals_cfg.nss_gpio_port != NULL)
        {
            /* Calculate the values for the peripheral registers that are not modified during the operation */
            _compute_peripheral_reg_values(bus_ctx, client, new_client_ctx);

            /* Allocate buffer for the DMA linked list nodes in the designated RAM section */
            _dma_lln_buf_alloc(new_client_ctx);
            if (NULL == new_client_ctx->dma_lln_buffer)
            {
                SID_PAL_LOG_ERROR("Unable to allocate DMA LLN memory for SPI client context");
                err = SID_ERROR_OOM;
                break;
            }

            /* Pre-populate DMA linked list for Transfer Control channel */
            err = _build_spi_transaction_dma_ll_skeleton(bus_ctx, new_client_ctx);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Failed to configure DMA LLN. Error %d", (int32_t)err);
                break;
            }
        }

        /* Add newly created context to the list */
        LST_insert_tail((tListNode *)&bus_ctx->client_contexts, &new_client_ctx->node);

        err = SID_ERROR_NONE;
    } while (0);

    if ((err != SID_ERROR_NONE) && (new_client_ctx != NULL))
    {
        /* Clean up if context creation failed */
        _delete_spi_client_ctx(new_client_ctx);
        *out_client_ctx = NULL;
    }
    else
    {
        *out_client_ctx = new_client_ctx;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _delete_spi_client_ctx(spi_client_context_t * const client_ctx)
{
    if (client_ctx != NULL)
    {
        if (client_ctx->peripherals_cfg.nss_gpio_port != NULL)
        {
            /* Set NSS GPIO pin to Hi-Z */
            HAL_GPIO_DeInit(client_ctx->peripherals_cfg.nss_gpio_port, client_ctx->peripherals_cfg.nss_gpio_pin);
        }

        /* Release DMA LLN buffer */
        if (client_ctx->dma_lln_buffer != NULL)
        {
            _dma_lln_buf_free(client_ctx);
        }

        free(client_ctx);
    }
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_error_t _set_spi_frame_mode(SPI_HandleTypeDef * const hspi, const sid_pal_serial_bus_stm32wbaxx_spi_mode_t spi_mode)
{
    sid_error_t err = SID_ERROR_NONE;

    switch (spi_mode)
    {
        case SID_PAL_SERIAL_BUS_STM32WBAxx_SPI_MODE_0:
            MODIFY_REG(hspi->Instance->CFG2, (SPI_CFG2_CPHA | SPI_CFG2_CPOL), (SPI_POLARITY_LOW | SPI_PHASE_1EDGE));
            break;

        case SID_PAL_SERIAL_BUS_STM32WBAxx_SPI_MODE_1:
            MODIFY_REG(hspi->Instance->CFG2, (SPI_CFG2_CPHA | SPI_CFG2_CPOL), (SPI_POLARITY_LOW | SPI_PHASE_2EDGE));
            break;

        case SID_PAL_SERIAL_BUS_STM32WBAxx_SPI_MODE_2:
            MODIFY_REG(hspi->Instance->CFG2, (SPI_CFG2_CPHA | SPI_CFG2_CPOL), (SPI_POLARITY_HIGH | SPI_PHASE_1EDGE));
            break;

        case SID_PAL_SERIAL_BUS_STM32WBAxx_SPI_MODE_3:
            MODIFY_REG(hspi->Instance->CFG2, (SPI_CFG2_CPHA | SPI_CFG2_CPOL), (SPI_POLARITY_HIGH | SPI_PHASE_2EDGE));
            break;

        default:
            err = SID_ERROR_INVALID_ARGS;
            break;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline sid_error_t _handle_spi_xfer_polling(LPTIM_HandleTypeDef * const hlptim, SPI_HandleTypeDef * const hspi, const uint32_t transaction_timeout_us)
{
    sid_error_t err, uptime_err;
    struct sid_timespec start_ts;
    struct sid_timespec elapsed_ts;

    uptime_err = sid_pal_uptime_now(&start_ts);
    SID_PAL_ASSERT(SID_ERROR_NONE == uptime_err);
    __COMPILER_BARRIER();

    DMA_HandleTypeDef * hdma_xfer_ctrl = NULL;
    const uint32_t      dma_error_flag_mask = (DMA_CSR_DTEF | DMA_CSR_ULEF | DMA_CSR_USEF | DMA_CSR_TOF);
    const uint32_t      spi_error_flag_mask = (SPI_SR_UDR | SPI_SR_OVR | SPI_SR_CRCE | SPI_SR_TIFRE | SPI_SR_MODF);

    SID_PAL_ASSERT(hspi != NULL);

    if (hlptim != NULL)
    {
        hdma_xfer_ctrl = hlptim->hdma[LPTIM_DMA_ID_CC1]; /* Extract Transaction Control DMA channel selection from the associated LPTIM configuration */
    }

    /* Capture transfer size at start */
    const uint32_t spi_ctsize_start = (hspi->Instance->SR & SPI_SR_CTSIZE_Msk);

    /* Indicate timeout error by default */
    err = SID_ERROR_TIMEOUT;
    do
    {
              uint32_t dma_ctrl_ch_events = 0u;
        const uint32_t dma_tx_ch_events   = hspi->hdmatx->Instance->CSR;
        const uint32_t dma_rx_ch_events   = hspi->hdmarx->Instance->CSR;
        const uint32_t spi_events         = hspi->Instance->SR;

        /* Monitor Transfer Control DMA channel if it is used */
        if (hdma_xfer_ctrl != NULL)
        {
            dma_ctrl_ch_events = hdma_xfer_ctrl->Instance->CSR;

            if ((dma_ctrl_ch_events & dma_error_flag_mask) != 0u)
            {
                /* Monitor Transaction Control DMA channel */
                SID_PAL_LOG_ERROR("SPI Transaction Control DMA channel error. Flags: 0x%x", dma_ctrl_ch_events);
                if (hdma_xfer_ctrl->XferErrorCallback != NULL)
                {
                    hdma_xfer_ctrl->XferErrorCallback(hdma_xfer_ctrl);
                }
                err = SID_ERROR_IO_ERROR;
                break;
            }
        }

        /* Monitor SPI peripheral errors */
        if ((spi_events & spi_error_flag_mask) != 0u)
        {
            //TODO: call SPI error callback
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Monitor DMA Tx channel */
        if ((dma_tx_ch_events & dma_error_flag_mask) != 0u)
        {
            SID_PAL_LOG_ERROR("SPI Tx DMA channel error. Flags: 0x%08X", dma_tx_ch_events);
            if (hspi->hdmatx->XferErrorCallback != NULL)
            {
                hspi->hdmatx->XferErrorCallback(hspi->hdmatx);
            }
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* Monitor DMA Rx channel */
        if ((dma_rx_ch_events & dma_error_flag_mask) != 0u)
        {
            SID_PAL_LOG_ERROR("SPI Rx DMA channel error. Flags: 0x%08X", dma_rx_ch_events);
            if (hspi->hdmarx->XferErrorCallback != NULL)
            {
                hspi->hdmarx->XferErrorCallback(hspi->hdmarx);
            }
            err = SID_ERROR_IO_ERROR;
            break;
        }

        /* If both Transaction Control DMA channel (when used) and SPI indicate transaction end we can proceed */
        if (((NULL == hdma_xfer_ctrl) || (dma_ctrl_ch_events & DMA_CSR_TCF) != 0u) && ((spi_events & SPI_SR_EOT) != 0u))
        {
            _hal_like_spi_close_transfer(hspi, hlptim);
            err = SID_ERROR_NONE;
            break;
        }
        __COMPILER_BARRIER();

        /* Update elapsed time */
        uptime_err = sid_pal_uptime_now(&elapsed_ts);
        SID_PAL_ASSERT(SID_ERROR_NONE == uptime_err);
        sid_time_sub(&elapsed_ts, &start_ts);
    } while (sid_timespec_to_us(&elapsed_ts) <= transaction_timeout_us);

    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("SPI transfer failed. Error %d", (int32_t)err);
        if ((SID_ERROR_TIMEOUT == err) && (hdma_xfer_ctrl != NULL) && (spi_ctsize_start == (hspi->Instance->SR & SPI_SR_CTSIZE_Msk)))
        {
            /**
             * There's no sign of SPI transfering any bytes - this situation is possible when LPTIM and Xfer Control DMA channels are used to keep NSS-to-NSS
             * and NSS-to-SCK delays. TO achieve that, the Xfer Control DMA channel is configured in a linked list (LL) mode, while LPTIM channels 0 and 1 are
             * used to trigger the Xfer Control DMA channel. However, if the time difference between LTPIM Channel 0 and Channel 1 events is too short, there
             * may be not enough time for the DMA channel to fetch the next LL node and apply its configuration. As a result, the Xfer Control may fail to
             * trigger the SPI transfer since any DMA triggers that arrive before the LL node is fully loaded are discarded. If you see the below warning in
             * the logs, try increasing the NSS-to-SCK delay to give the DMA more time to load the next LL node after the LPTIM Channel 0 (NSS-to-NSS delay)
             * event has fired.
             */
            SID_PAL_LOG_WARNING("SPI failed to start. Check Transfer Control DMA channel has enough time to reload linked list nodes");
        }
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _sid_pal_serial_bus_spi_stm32wbaxx_xfer(const struct sid_pal_serial_bus_iface *iface, const struct sid_pal_serial_bus_client *client,
                                       uint8_t *tx, uint8_t *rx, size_t xfer_size)
{
    sid_error_t err = SID_ERROR_GENERIC;
    HAL_StatusTypeDef hal_err;
    const uint32_t use_irqs = FALSE; //TODO: determine dynamically based on the caller context

    spi_client_context_t * client_ctx = NULL;
    DMA_HandleTypeDef * hdma_xfer_ctrl = NULL;
    LPTIM_HandleTypeDef * hlptim = NULL;
    SPI_HandleTypeDef * hspi = NULL;

    do
    {
        /* Validate inputs */
        if (NULL == iface)
        {
            SID_PAL_LOG_ERROR("SPI xfer failed - iface cannot be null");
            err = SID_ERROR_INVALID_ARGS;
            break;
        }
        if (NULL == client)
        {
            SID_PAL_LOG_ERROR("SPI xfer failed - client cannot be null");
            err = SID_ERROR_INVALID_ARGS;
            break;
        }
        if (0u == client->speed_hz)
        {
            SID_PAL_LOG_ERROR("SPI xfer failed - desired clock speed cannot be 0");
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        serial_bus_spi_ctx * bus_ctx = containerof(iface, serial_bus_spi_ctx, iface);
        hspi = bus_ctx->factory_config->hspi;
        SID_PAL_ASSERT(hspi != NULL);

        sid_pal_enter_critical_region();
        /* Try to load client context */
        client_ctx = _find_spi_client_ctx(bus_ctx, client);

        /* Create client context if it does not exist */
        if (NULL == client_ctx)
        {
            err = _create_spi_client_ctx(bus_ctx, client, &client_ctx);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Unable to create SPI client context. Error %d", (int32_t)err);
                break;
            }
        }

        /* Lock SPI before any further actions will take place */
        hal_err = _hal_like_lock_spi(hspi);
        sid_pal_exit_critical_region();

        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Unable to lock SPI before configuring the transaction. Possible access collision");
            return SID_ERROR_BUSY; /* We have to exit from here */
        }

        //TODO: either ensure client context has not changed or implement dynamic updates of the transaction settings (e.g. clock speed, SPI mode, CRC length, etc.)

        /* Pre-configure the peripherals */
        /* Setup LPTIM for NSS handling at transaction start */
        if (client_ctx->peripherals_cfg.nss_gpio_port != NULL)
        {
            /* Ensure there's an LPTIM assigned to control NSS pin timings */
            hlptim = bus_ctx->factory_config->hlptim;
            SID_PAL_ASSERT(hlptim != NULL);

            /* Extract Transaction Control DMA channel info from LPTIM settings */
            hdma_xfer_ctrl = hlptim->hdma[LPTIM_DMA_ID_CC1];
            SID_PAL_ASSERT(hdma_xfer_ctrl != NULL);

            /* Configure LPTIM prescaler */
            WRITE_REG(hlptim->Instance->CR, 0u); /* Disable LPTIM and clear any counting mode configurations */
            LL_LPTIM_SetPrescaler(hlptim->Instance, client_ctx->peripherals_cfg.nss_lptim_prescaler);
            __COMPILER_BARRIER(); /* Ensure presacler is updated before enabling the timer */
            LL_LPTIM_Enable(hlptim->Instance);

            /* Since LPTIM needs 2 clock cycles to start, proceed with SPI config while it is starting */
        }

        /* Baud rate config */
        SID_PAL_ASSERT(IS_SPI_BAUDRATE_PRESCALER(client_ctx->peripherals_cfg.spi_prescaler));
        MODIFY_REG(hspi->Instance->CFG1, (SPI_CFG1_BPASS | SPI_CFG1_MBR), client_ctx->peripherals_cfg.spi_prescaler);

        /* SPI Mode */
        err = _set_spi_frame_mode(hspi, client->mode);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Invalid SPI mode specified - xfer aborted. Mode: 0x%x", (uint32_t)client->mode);
            return err;
        }

        /* SPI bit order */
        switch (client->bit_order)
        {
            case SID_PAL_SERIAL_BUS_BIT_ORDER_MSB_FIRST:
                MODIFY_REG(hspi->Instance->CFG2, SPI_CFG2_LSBFRST, SPI_FIRSTBIT_MSB);
                break;

            case SID_PAL_SERIAL_BUS_BIT_ORDER_LSB_FIRST:
                MODIFY_REG(hspi->Instance->CFG2, SPI_CFG2_LSBFRST, SPI_FIRSTBIT_LSB);
                break;

            default:
                SID_PAL_LOG_ERROR("Invalid SPI mode specified - xfer aborted. Mode: 0x%x", (uint32_t)client->mode);
                err = SID_ERROR_INVALID_ARGS;
                return err;
        }

        /* CRC config */
        const uint32_t enable_crc = (client_ctx->peripherals_cfg.crc_polynomial != 0u) ? TRUE : FALSE;
        if (enable_crc != FALSE)
        {
            SID_PAL_ASSERT(IS_SPI_CRC_LENGTH(client_ctx->peripherals_cfg.crc_length));

            /* Configure CRC seed */
            if (SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN == client_ctx->peripherals_cfg.crc_init_pattern)
            {
                CLEAR_BIT(hspi->Instance->CR1, (SPI_CR1_TCRCINI | SPI_CR1_RCRCINI));
            }
            else
            {
                SET_BIT(hspi->Instance->CR1, (SPI_CR1_TCRCINI | SPI_CR1_RCRCINI));
            }

            /* Configure CRC polynomial */
            WRITE_REG(hspi->Instance->CRCPOLY, client_ctx->peripherals_cfg.crc_polynomial);

            /* Enable 33/17 bits CRC computation */
            if (((IS_SPI_LIMITED_INSTANCE(hspi->Instance)) && (SPI_CRC_LENGTH_16BIT == client_ctx->peripherals_cfg.crc_length)) ||
            ((IS_SPI_FULL_INSTANCE(hspi->Instance)) && (SPI_CRC_LENGTH_32BIT == client_ctx->peripherals_cfg.crc_length)))
            {
                SET_BIT(hspi->Instance->CR1, SPI_CR1_CRC33_17);
            }
            else
            {
                CLEAR_BIT(hspi->Instance->CR1, SPI_CR1_CRC33_17);
            }

            /* Set CRC length and enable it */
            MODIFY_REG(hspi->Instance->CFG1, (SPI_CFG1_CRCEN | SPI_CFG1_CRCSIZE), (SPI_CRCCALCULATION_ENABLE | client_ctx->peripherals_cfg.crc_length));
        }
        else
        {
            /* Disable CRC computation */
            MODIFY_REG(hspi->Instance->CFG1, (SPI_CFG1_CRCEN | SPI_CFG1_CRCSIZE), (SPI_CRCCALCULATION_DISABLE | SPI_CRC_LENGTH_8BIT /* Reset value */));
        }

        /* Now configure NSS timings since LPTIM will be enabled by this time */
        if (client_ctx->peripherals_cfg.nss_gpio_port != NULL)
        {
            SID_PAL_ASSERT(client_ctx->dma_lln_buffer != NULL);

            /* Configure NSS timings for transaction start */
            /* NSS-to-NSS */
            LL_LPTIM_OC_SetCompareCH1(hlptim->Instance, client_ctx->peripherals_cfg.dma_regs_storage.lptim_ccr1_val_nss_to_nss);

            /* NSS-to-SCK */
            LL_LPTIM_OC_SetCompareCH2(hlptim->Instance, (client_ctx->peripherals_cfg.dma_regs_storage.lptim_ccr2_val_nss_to_sck));

            /* Configure Transaction Control DMA channel to drive the NSS pin and trigger the transaction start */
            //TODO: possibly support NSS polarity selection by writing into BRR or BSRR
            LL_DMA_ConfigAddresses(LL_DMA_GET_INSTANCE(hdma_xfer_ctrl->Instance), LL_DMA_GET_CHANNEL(hdma_xfer_ctrl->Instance),
                                   (uint32_t)&client_ctx->peripherals_cfg.nss_gpio_pin, (uint32_t)&client_ctx->peripherals_cfg.nss_gpio_port->BRR);
            LL_DMA_SetBlkDataLength(LL_DMA_GET_INSTANCE(hdma_xfer_ctrl->Instance), LL_DMA_GET_CHANNEL(hdma_xfer_ctrl->Instance), sizeof(client_ctx->peripherals_cfg.nss_gpio_port->BRR));

            /* Configure the gap between adjacent transactions */
            if (client_ctx->peripherals_cfg.nss_to_nss_cycles > 0u)
            {
                /* Configure the channel to be triggered by LPTIM compare event */
                LL_DMA_SetTriggerPolarity(LL_DMA_GET_INSTANCE(hdma_xfer_ctrl->Instance), LL_DMA_GET_CHANNEL(hdma_xfer_ctrl->Instance), LL_DMA_TRIG_POLARITY_RISING);
                LL_DMA_SetHWTrigger(LL_DMA_GET_INSTANCE(hdma_xfer_ctrl->Instance), LL_DMA_GET_CHANNEL(hdma_xfer_ctrl->Instance), bus_ctx->dma_periph_cfg.dma_trigger_lptimx_ch1_val);
            }
            else
            {
                /* Configure the channel to act immediately without any additional triggers */
                LL_DMA_SetTriggerPolarity(LL_DMA_GET_INSTANCE(hdma_xfer_ctrl->Instance), LL_DMA_GET_CHANNEL(hdma_xfer_ctrl->Instance), LL_DMA_TRIG_POLARITY_MASKED);
            }

            /* Configure the delay between NSS activation and the very first SCK pulse */
            if (client_ctx->peripherals_cfg.nss_to_sck_cycles > 0u)
            {
                /* Modify LLN[0] trigger to LPTIM CH2 */
                MODIFY_REG(client_ctx->dma_lln_buffer[SPI_DMA_LLN_WRITE_SPI_CSTART_CTR2_OFFSET], DMA_CTR2_TRIGSEL, (bus_ctx->dma_periph_cfg.dma_trigger_lptimx_ch2_val << DMA_CTR2_TRIGSEL_Pos));
                MODIFY_REG(client_ctx->dma_lln_buffer[SPI_DMA_LLN_WRITE_SPI_CSTART_CTR2_OFFSET], DMA_CTR2_TRIGPOL, LL_DMA_TRIG_POLARITY_RISING);
            }
            else
            {
                /* Deactivate LLN[0] trigger - SPI will be started immediatly after driving the NSS pin */
                MODIFY_REG(client_ctx->dma_lln_buffer[SPI_DMA_LLN_WRITE_SPI_CSTART_CTR2_OFFSET], DMA_CTR2_TRIGPOL, LL_DMA_TRIG_POLARITY_MASKED);
            }

            /* Configure DMA LLN update */
            const uint32_t cllr_regs_to_modify = client_ctx->dma_lln_buffer[SPI_DMA_LLN_INITIAL_CLLR_VALS_OFFSET];
            LL_DMA_ConfigLinkUpdate(LL_DMA_GET_INSTANCE(hdma_xfer_ctrl->Instance), LL_DMA_GET_CHANNEL(hdma_xfer_ctrl->Instance),
                                    cllr_regs_to_modify, (uint32_t)&client_ctx->dma_lln_buffer[SPI_DMA_LLN_INITIAL_CLLR_VALS_OFFSET + 1u /* LL nodes follow right after SPI_DMA_LLN_INITIAL_CLLR_VALS_OFFSET */]);
        }

        /* Configure DMA for transaction end handling */
        /* Use Rx TC event is a starting point for countdown towards SPI end of transaction */
        if ((FALSE == enable_crc) && (0u == client_ctx->peripherals_cfg.sck_to_nss_cycles))
        {
            /* No need to wait before deselcting the client's NSS - trigger Transaction Control DMA channel immediately upon Rx DMA channel TC event */
            MODIFY_REG(client_ctx->dma_lln_buffer[SPI_DMA_LLN_NSS_EOT_TRIGGER_SRC_OFFSET], DMA_CTR2_TRIGPOL, LL_DMA_TRIG_POLARITY_MASKED);
        }
        else
        {
            /* We will have to wait a bit after DMA TC event, so setup LPTIM trigger for the Transaction Control DMA channel */
            MODIFY_REG(client_ctx->dma_lln_buffer[SPI_DMA_LLN_NSS_EOT_TRIGGER_SRC_OFFSET], DMA_CTR2_TRIGSEL, (bus_ctx->dma_periph_cfg.dma_trigger_lptimx_ch1_val << DMA_CTR2_TRIGSEL_Pos));
            MODIFY_REG(client_ctx->dma_lln_buffer[SPI_DMA_LLN_NSS_EOT_TRIGGER_SRC_OFFSET], DMA_CTR2_TRIGPOL, LL_DMA_TRIG_POLARITY_RISING);

            /* Calculate the delay for the LPTIM */
            client_ctx->peripherals_cfg.dma_regs_storage.lptim_ccr1_val_xfer_end = client_ctx->peripherals_cfg.dma_regs_storage.lptim_ccrx_val_sck_to_nss;
            if (client_ctx->peripherals_cfg.dma_regs_storage.lptim_ccr1_val_xfer_end > client_ctx->peripherals_cfg.dma_regs_storage.lptim_ccrx_val_sck_to_nss_dma_rx_compensation)
            {
                /* Compensate for the DMA processing time */
                client_ctx->peripherals_cfg.dma_regs_storage.lptim_ccr1_val_xfer_end -= client_ctx->peripherals_cfg.dma_regs_storage.lptim_ccrx_val_sck_to_nss_dma_rx_compensation;
            }
            else
            {
                /* Calculated delay is shorted than DMA processing time - leave 1 clock delay since this is an absolute minimum value we can use */
                client_ctx->peripherals_cfg.dma_regs_storage.lptim_ccr1_val_xfer_end = 1u;
            }
        }

        /* Arm SPI */
        hal_err = _hal_like_spi_prepare_transmit_receive_dma(hspi, tx, rx, xfer_size, use_irqs, enable_crc, client_ctx->peripherals_cfg.crc_length);
        if (hal_err != HAL_OK)
        {
            if (HAL_BUSY == hal_err)
            {
                SID_PAL_LOG_ERROR("Unable to configure SPI peripheral for xfer. SPI peripheral is busy. SPI status: 0x%x", hspi->State);
                err = SID_ERROR_BUSY;
            }
            else
            {
                SID_PAL_LOG_ERROR("Unable to configure SPI peripheral for xfer. HAL error 0x%x", hal_err);
                err = SID_ERROR_IO_ERROR;
            }
            break;
        }

        /* Start the transaction */
        if (client_ctx->peripherals_cfg.nss_gpio_port != NULL)
        {
            /* Disable all DMA interrupts */
            MODIFY_REG(hdma_xfer_ctrl->Instance->CCR, DMA_CCR_EVENT_SRC_MASK, 0u);

            /* Clear all interrupt flags */
            MODIFY_REG(hdma_xfer_ctrl->Instance->CFCR, DMA_CFCR_ALL_EVENT_FLAGS, DMA_CFCR_ALL_EVENT_FLAGS);

            /* Configure callbacks */
            hdma_xfer_ctrl->XferHalfCpltCallback = NULL;
            hdma_xfer_ctrl->XferCpltCallback     = NULL;
            hdma_xfer_ctrl->XferAbortCallback    = NULL;
            hdma_xfer_ctrl->XferErrorCallback    = _hal_like_spi_handle_dma_error;

            /* Configure interrupts to be used */
            if (use_irqs != FALSE)
            {
                /* Enable Transfer Complete and error interrupts since the others are irrelevant */
                MODIFY_REG(hdma_xfer_ctrl->Instance->CCR, DMA_CCR_EVENT_SRC_MASK, (DMA_CCR_TCIE | DMA_CCR_DTEIE | DMA_CCR_ULEIE | DMA_CCR_USEIE | DMA_CCR_TOIE));
            }
            __COMPILER_BARRIER();

            /* Prepare the reg value to be written into SPI->CR1 to start the transaction */
            client_ctx->peripherals_cfg.dma_regs_storage.spi_cr1_reg_val_start = (READ_REG(hspi->Instance->CR1) | SPI_CR1_CSTART);

            /* Now start the Transaction Control DMA channel */
            LL_DMA_EnableChannel(LL_DMA_GET_INSTANCE(hdma_xfer_ctrl->Instance), LL_DMA_GET_CHANNEL(hdma_xfer_ctrl->Instance));
            __COMPILER_BARRIER(); /* Ensure DMA is started before the LPTIM */

            /* Go - LPTIM and DMA will drive NSS pin and initiate SPI transaction respecting the selected timings */
            LL_LPTIM_StartCounter(hlptim->Instance, LL_LPTIM_OPERATING_MODE_ONESHOT);
        }
        else
        {
            /* Start SPI peripheral directly since no NSS handling is required */
            SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);
        }

        /* Unlock the process */
        __HAL_UNLOCK(hspi);

        err = SID_ERROR_NONE;
    } while (0);

    if (err != SID_ERROR_NONE)
    {
        SID_PAL_LOG_ERROR("SPI prep error %d", err);
    }
    else
    {
        /* Compute expected transaction timeout */
        const uint32_t spi_clock_khz           = client->speed_hz / 1000u;
        const uint32_t full_transfer_size_bits = (client_ctx->peripherals_cfg.crc_polynomial != 0u) ? ((xfer_size << 3u) + SPI_HAL_CRC_SIZE_TO_INT_VAL(client_ctx->peripherals_cfg.crc_length)) : (xfer_size << 3u);
        const uint32_t nss_handling_ticks      = client_ctx->peripherals_cfg.nss_to_sck_cycles + client_ctx->peripherals_cfg.sck_to_nss_cycles + client_ctx->peripherals_cfg.nss_to_nss_cycles;
        const uint32_t nss_handling_time_us    = (nss_handling_ticks * 1000u + (spi_clock_khz >> 1u)) / spi_clock_khz;      /* Use math ceiling here */
        const uint32_t spi_transfer_time_us    = (full_transfer_size_bits * 1000u + (spi_clock_khz >> 1u)) / spi_clock_khz; /* Use math ceiling here */
              uint32_t transaction_timeout_us  = ((nss_handling_time_us + spi_transfer_time_us) << 1u); /* Set timeout to 2x the expected transfer time */
        if (transaction_timeout_us < SPI_TRANSACTION_MINIMUM_TIMEOUT_US)
        {
            /* Override the timeout if it is too short */
            transaction_timeout_us = SPI_TRANSACTION_MINIMUM_TIMEOUT_US;
        }

        /* Wait for transfer end */
        if (FALSE == use_irqs)
        {
            /* Use polling mode (blocking transfer) */
            err = _handle_spi_xfer_polling(hlptim, hspi, transaction_timeout_us);
        }
        else
        {
            /* We are in a task context, it's ok to wait for a Xfer completion event */
            err = SID_ERROR_NOSUPPORT;
            SID_PAL_LOG_ERROR("SPI transactions using IRQs are not supported");
        }
    }

    /* Ensure deterministic state if Xfer has failed at any point */
    if (err != SID_ERROR_NONE)
    {
        _hal_like_spi_close_transfer(hspi, hlptim);

        /* Ensure NSS is deactivated */
        if ((client_ctx != NULL) && (client_ctx->peripherals_cfg.nss_gpio_port != NULL))
        {
            //TODO: possibly support NSS polarity selection by writing into BRR or BSRR
            client_ctx->peripherals_cfg.nss_gpio_port->BSRR = client_ctx->peripherals_cfg.nss_gpio_pin;
        }

        /* Ensure SPI handle is released */
        __HAL_UNLOCK(hspi);
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static sid_error_t _sid_pal_serial_bus_spi_stm32wbaxx_xfer_hd(const struct sid_pal_serial_bus_iface *iface, const struct sid_pal_serial_bus_client *client,
                                                                                        uint8_t *tx, uint8_t *rx, size_t tx_size, size_t rx_size)
{
    SID_PAL_LOG_ERROR("SPI half-duplex mode is not supported");
    return SID_ERROR_NOSUPPORT;
}

/*----------------------------------------------------------------------------*/

static sid_error_t _sid_pal_serial_bus_spi_stm32wbaxx_destroy(const struct sid_pal_serial_bus_iface * iface)
{
    sid_error_t err = SID_ERROR_GENERIC;

    SID_PAL_LOG_DEBUG("sid_pal_serial_bus_destroy()...");

    sid_pal_enter_critical_region();

    do
    {
        if (NULL == iface)
        {
            err = SID_ERROR_INVALID_ARGS;
            break;
        }

        serial_bus_spi_ctx * bus_ctx = containerof(iface, serial_bus_spi_ctx, iface);

        if (bus_ctx == &bus)
        {
            HAL_StatusTypeDef hal_err = HAL_OK;

            /* Stop any active operations */
            __HAL_LPTIM_DISABLE(bus_ctx->factory_config->hlptim);
            __COMPILER_BARRIER();

            /* Delete client contexts - this will also set all NSS pins to deselected state */
            if (LST_is_empty((tListNode *)&bus_ctx->client_contexts) == FALSE)
            {
                /* Iterate over the list o client contexts and try to find a matching one */
                tListNode * iterator;
                LST_get_next_node((tListNode *)&bus_ctx->client_contexts, &iterator);

                while (iterator != &(bus_ctx->client_contexts))
                {
                    spi_client_context_t * const client_ctx = containerof(iterator, spi_client_context_t, node);

                    /* Advance iterator to the next list node */
                    LST_get_next_node(iterator, &iterator);

                    /* Destroy current node */
                    _delete_spi_client_ctx(client_ctx);
                }
            }

            /* Ensure SPI is not running any longer */
            __HAL_SPI_DISABLE(bus_ctx->factory_config->hspi);
            __COMPILER_BARRIER();

            /* Bring down LPTIM */
            if (bus_ctx->factory_config->hlptim->State != HAL_LPTIM_STATE_RESET)
            {
                hal_err = HAL_LPTIM_DeInit(bus_ctx->factory_config->hlptim);
                if (hal_err != HAL_OK)
                {
                    /* Report error but continue deinitialization to stop as many peripherals as we can */
                    SID_PAL_LOG_ERROR("Unable to deinitialize LPTIM while deinitializing SPI bus. HAL error 0x%x", hal_err);
                }
            }

            /* Bring down SPI */
            if (bus_ctx->factory_config->hspi->State != HAL_SPI_STATE_RESET)
            {
                hal_err |= HAL_SPI_DeInit(bus_ctx->factory_config->hspi);
                if (hal_err != HAL_OK)
                {
                    /* Report error but continue deinitialization to stop as many peripherals as we can */
                    SID_PAL_LOG_ERROR("Unable to deinitialize SPI peripheral. HAL error 0x%x", hal_err);
                }
            }

            /* Kill the interface as the last step */
            SID_STM32_UTIL_fast_memset(&bus_ctx->iface, 0u, sizeof(bus_ctx->iface));
            bus_ctx->factory_config = NULL;

            /* After all peripherals are processed report an error if any peripheral failed to deinitialize */
            if (hal_err != HAL_OK)
            {
                err = SID_ERROR_IO_ERROR;
                break;
            }
        }
        else
        {
            SID_PAL_LOG_WARNING("Unable to destroy SPI bus - unknown bus referenced in the request");
            err = SID_ERROR_NOT_FOUND;
            break;
        }

        /* Everything is ok if we've got to this point */
        err = SID_ERROR_NONE;
    } while (0);

    sid_pal_exit_critical_region();

    return err;
}

/* Global function definitions -----------------------------------------------*/

sid_error_t sid_pal_serial_bus_stm32wbaxx_create(const struct sid_pal_serial_bus_iface **iface, const void *const context)
{
    sid_error_t err = SID_ERROR_GENERIC;

    SID_PAL_LOG_DEBUG("sid_pal_serial_bus_create()...");

    do
    {
        /* Validate inputs */
        if ((NULL == iface) || (NULL == context))
        {
            SID_PAL_LOG_ERROR("Failed to create SPI bus, invalid arguments");
            break;
        }

        //TODO: either support multiple requests or report SID_ERROR_ALREADY_INITIALIZED

        if (FALSE == dma_lln_buf_initialized)
        {
            for (uint32_t i = 0u; i < SID_STM32_UTIL_ARRAY_SIZE(dma_lln_buf); i++)
            {
                dma_lln_buf[i][0] = SPI_DMA_LLN_BUF_FREE_WATERMARK;
            }

            dma_lln_buf_initialized = TRUE;
        }

        /* Store factory config */
        const sid_pal_serial_bus_stm32wbaxx_factory_config_t * const factory_cfg = (const sid_pal_serial_bus_stm32wbaxx_factory_config_t *)context;
        bus.factory_config = factory_cfg;

        /* Initialize LPTIM that will be used for NSS pin timings */
        err = _nss_lptim_init(&bus);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to create SPI bus. LPTIM peripheral initialization err %d", (int32_t)err);
            break;
        }

        /* Initialize SPI peripheral */
        err = _spi_peripheral_init(&bus);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to create SPI bus. SPI peripheral initialization err %d", (int32_t)err);
            break;
        }

        /* Initialize the list of client contexts */
        LST_init_head(&bus.client_contexts);

        /* Set the control interface */
        bus.iface = bus_ops;

        /* Prepare outputs */
        *iface = &bus.iface;

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* If initialization failed for any reason ensure we do not set iface with an invalid interface */
    if (err != SID_ERROR_NONE)
    {
        *iface = NULL;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_serial_bus_ext_ifc_activate_nss(const struct sid_pal_serial_bus_iface * const iface, const struct sid_pal_serial_bus_client * const client)
{
    sid_error_t err = SID_ERROR_GENERIC;
    HAL_StatusTypeDef hal_err;

    spi_client_context_t * client_ctx = NULL;
    SPI_HandleTypeDef * hspi = NULL;

    /* Validate inputs */
    if (NULL == iface)
    {
        SID_PAL_LOG_ERROR("SPI Activate NSS failed - iface cannot be null");
        return SID_ERROR_INVALID_ARGS;
    }
    if (NULL == client)
    {
        SID_PAL_LOG_ERROR("SPI Activate NSS failed - client cannot be null");
        return SID_ERROR_INVALID_ARGS;
    }

    do
    {
        serial_bus_spi_ctx * bus_ctx = containerof(iface, serial_bus_spi_ctx, iface);
        hspi = bus_ctx->factory_config->hspi;
        SID_PAL_ASSERT(hspi != NULL);

        sid_pal_enter_critical_region();
        /* Try to load client context */
        client_ctx = _find_spi_client_ctx(bus_ctx, client);

        /* Create client context if it does not exist */
        if (NULL == client_ctx)
        {
            err = _create_spi_client_ctx(bus_ctx, client, &client_ctx);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Unable to create SPI client context. Error %d", (int32_t)err);
                return err; /* We have to exit from here */
            }
        }

        /* Lock SPI before any further actions will take place */
        hal_err = _hal_like_lock_spi(hspi);
        sid_pal_exit_critical_region();

        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Unable to lock SPI before driving the NSS line. Possible access collision");
            return SID_ERROR_BUSY; /* We have to exit from here */
        }

        /* Ensure the client has NSS pin assigned */
        if (NULL == client_ctx->peripherals_cfg.nss_gpio_port)
        {
            err = SID_ERROR_NOSUPPORT;
            break;
        }

        /* Configure SPI peripheral to properly drive SCK, MOSI, and MISO pins for the selected client */
        err = _set_spi_frame_mode(hspi, client->mode);
        if (err != SID_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Invalid SPI mode specified - xfer aborted. Mode: 0x%x", (uint32_t)client->mode);
            break;
        }
        SPI_2LINES(hspi);

        /* Enable I/O state retention to ensure SCK, MOSI, MISO will be valid before NSS is activated */
        MODIFY_REG(hspi->Instance->CFG2, SPI_CFG2_AFCNTR, SPI_MASTER_KEEP_IO_STATE_ENABLE);
        __COMPILER_BARRIER();

        /* Finally drive the desired NSS line */
        //TODO: possibly support NSS polarity selection by writing into BRR or BSRR
        client_ctx->peripherals_cfg.nss_gpio_port->BRR = client_ctx->peripherals_cfg.nss_gpio_pin;

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Unlock the SPI regardless of the outcome */
    __HAL_UNLOCK(hspi);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t sid_pal_serial_bus_ext_ifc_deactivate_nss(const struct sid_pal_serial_bus_iface * const iface, const struct sid_pal_serial_bus_client * const client)
{
    sid_error_t err = SID_ERROR_GENERIC;
    HAL_StatusTypeDef hal_err;

    spi_client_context_t * client_ctx = NULL;
    SPI_HandleTypeDef * hspi = NULL;

    /* Validate inputs */
    if (NULL == iface)
    {
        SID_PAL_LOG_ERROR("SPI Deactivate NSS failed - iface cannot be null");
        return SID_ERROR_INVALID_ARGS;
    }
    if (NULL == client)
    {
        SID_PAL_LOG_ERROR("SPI Deactivate NSS failed - client cannot be null");
        return SID_ERROR_INVALID_ARGS;
    }

    do
    {
        serial_bus_spi_ctx * bus_ctx = containerof(iface, serial_bus_spi_ctx, iface);
        hspi = bus_ctx->factory_config->hspi;
        SID_PAL_ASSERT(hspi != NULL);

        sid_pal_enter_critical_region();
        /* Try to load client context */
        client_ctx = _find_spi_client_ctx(bus_ctx, client);

        /* Create client context if it does not exist */
        if (NULL == client_ctx)
        {
            err = _create_spi_client_ctx(bus_ctx, client, &client_ctx);
            if (err != SID_ERROR_NONE)
            {
                SID_PAL_LOG_ERROR("Unable to create SPI client context. Error %d", (int32_t)err);
                return err; /* We have to exit from here */
            }
        }

        /* Lock SPI before any further actions will take place */
        hal_err = _hal_like_lock_spi(hspi);
        sid_pal_exit_critical_region();

        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Unable to lock SPI before releasing the NSS line. Possible access collision");
            return SID_ERROR_BUSY; /* We have to exit from here */
        }

        /* Ensure the client has NSS pin assigned */
        if (NULL == client_ctx->peripherals_cfg.nss_gpio_port)
        {
            err = SID_ERROR_NOSUPPORT;
            break;
        }

        /* Release the desired NSS line */
        //TODO: possibly support NSS polarity selection by writing into BRR or BSRR
        client_ctx->peripherals_cfg.nss_gpio_port->BSRR = client_ctx->peripherals_cfg.nss_gpio_pin;

        /* Disable I/O state retention since it's not needed any longer */
        MODIFY_REG(hspi->Instance->CFG2, SPI_CFG2_AFCNTR, SPI_MASTER_KEEP_IO_STATE_DISABLE);

        /* Done */
        err = SID_ERROR_NONE;
    } while (0);

    /* Unlock the SPI regardless of the outcome */
    __HAL_UNLOCK(hspi);

    return err;
}
