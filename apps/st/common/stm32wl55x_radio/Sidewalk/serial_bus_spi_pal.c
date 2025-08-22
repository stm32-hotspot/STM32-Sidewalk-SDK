/**
  ******************************************************************************
  * @file    serial_bus_spi_pal.c
  * @brief   Handling of the SPI bus for inter-MCU communication
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

#include <cmsis_os2.h>

/* Platform definitions */
#include <stm32wlxx.h>

/* SPI protocol definitions */
#include "comm_def.h" /* Used only to keep SPI frame size consistent with the counterpart */

/* STM32 Utilities */
#include "sid_pal_log_like.h"
#include <sid_stm32_common_utils.h>

/* App-specific headers */
#include "main.h"
#include "serial_bus_spi_pal.h"

/* Private defines -----------------------------------------------------------*/

#define SPI_FRAME_DATA_SIZE                (STM32WLxx_RADIO_COMM_SPI_DATA_FRAME_SIZE)
static_assert((SPI_FRAME_DATA_SIZE) % sizeof(uint32_t) == 0u); /* SPI frame's data part shall be multiple of 4 to speed up memcpy */

#if (USE_SPI_CRC != 0U)
#  define SPI_FRAME_CRC_SIZE               (2u)
#else
#  define SPI_FRAME_CRC_SIZE               (0u)
#endif
#define SPI_FRAME_SIZE                     ((SPI_FRAME_DATA_SIZE) + (SPI_FRAME_CRC_SIZE))
#define SPI_RING_BUF_FRAMES                (24u)

#define NSS_DMA_INDEX_NSS_ACTIVATED        (0u)
#define NSS_DMA_INDEX_NSS_DEACTIVATED      (1u)

#define SPI_HAL_DEFAULT_WAIT_TIMEOUT_TICKS (10u)
#define SPI_HAL_COMM_START_TIMEOUT_TICKS   (100u)
#define SPI_PERIPHERAL_RESET_DELAY_US      (10u)

#define SPI_RINGBUF_OVERRUN_DIAG_WATERMARK ((uint32_t)(0xDEADBEEFu))

#ifndef SIDEWALK_RADIO_SPI_INSTANCE_BASE
#  error "SIDEWALK_RADIO_SPI_INSTANCE_BASE is not defined. Please specify the SPI peripheral instance to use for inter-MCU communication in STM32WLxx Sidewalk Radio app"
#endif

/* Private macro -------------------------------------------------------------*/

#define SPI_RINGBUF_FRAME_PARRETN_SET(__start_addr__, __pattern__)  do\
                                                                    {\
                                                                        register uint32_t * write_ptr = (void *)((__start_addr__) + SPI_FRAME_DATA_SIZE);\
                                                                        do\
                                                                        {\
                                                                            write_ptr--;\
                                                                            *write_ptr = (__pattern__);\
                                                                        } while (write_ptr != (uint32_t *)(void *)(__start_addr__));\
                                                                    } while (0)
#define SPI_RINGBUF_ADVANCE_POSITION(__position_addr__, __advance_size__, __buf_start_addr__, __buf_end_addr__) \
                                                                    do\
                                                                    {\
                                                                        __position_addr__ = (__position_addr__) + (__advance_size__);\
                                                                        while (__position_addr__ >= (__buf_end_addr__))\
                                                                        {\
                                                                            (__position_addr__) = (__buf_start_addr__) + (__position_addr__ - (__buf_end_addr__));\
                                                                        }\
                                                                    } while (0)
#define SPI_RINGBUF_REWIND_POSITION(__position_addr__, __rewind_size__, __buf_start_addr__, __buf_end_addr__) \
                                                                    do\
                                                                    {\
                                                                        __position_addr__ = (__position_addr__) - (__rewind_size__);\
                                                                        while (__position_addr__ < (__buf_start_addr__))\
                                                                        {\
                                                                            (__position_addr__) = (__buf_end_addr__) - ((__buf_start_addr__) - __position_addr__);\
                                                                        }\
                                                                    } while (0)

#if (SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE == GPIO_PIN_RESET)
#  define SIDEWALK_RADIO_SPI_NSS_ENABLE_TRIGGER()                   do\
                                                                    {\
                                                                        register const uint32_t mcu_rev = LL_DBGMCU_GetRevisionID();\
                                                                        if (mcu_rev > STM32WLxx_MCU_REV_Z)\
                                                                        {\
                                                                            /* Normally activate just rising edge detection */\
                                                                            SET_BIT(EXTI->RTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                        }\
                                                                        else\
                                                                        {\
                                                                            /* Activate both edges as part of workaround for errata 2.2.1 */\
                                                                            SET_BIT(EXTI->RTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                            SET_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                        }\
                                                                    } while (0)
#  define SIDEWALK_RADIO_SPI_NSS_DISABLE_TRIGGER()                  do\
                                                                    {\
                                                                        register const uint32_t mcu_rev = LL_DBGMCU_GetRevisionID();\
                                                                        if (mcu_rev > STM32WLxx_MCU_REV_Z)\
                                                                        {\
                                                                            /* Normally deactivate just rising edge detection */\
                                                                            CLEAR_BIT(EXTI->RTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                        }\
                                                                        else\
                                                                        {\
                                                                            /* Deactivate both edges as part of workaround for errata 2.2.1 */\
                                                                            CLEAR_BIT(EXTI->RTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                            CLEAR_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                        }\
                                                                        /* Clear IRQ flag */\
                                                                        __HAL_GPIO_EXTI_CLEAR_IT(SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                    } while (0)
#else
#  define SIDEWALK_RADIO_SPI_NSS_ENABLE_TRIGGER()                   do\
                                                                    {\
                                                                        register const uint32_t mcu_rev = LL_DBGMCU_GetRevisionID();\
                                                                        if (mcu_rev > STM32WLxx_MCU_REV_Z)\
                                                                        {\
                                                                            /* Normally activate just falling edge detection */\
                                                                            SET_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                        }\
                                                                        else\
                                                                        {\
                                                                            /* Activate both edges as part of workaround for errata 2.2.1 */\
                                                                            SET_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                            SET_BIT(EXTI->RTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                        }\
                                                                    } while (0)
#  define SIDEWALK_RADIO_SPI_NSS_DISABLE_TRIGGER()                  do\
                                                                    {\
                                                                        register const uint32_t mcu_rev = LL_DBGMCU_GetRevisionID();\
                                                                        if (mcu_rev > STM32WLxx_MCU_REV_Z)\
                                                                        {\
                                                                            /* Normally deactivate just falling edge detection */\
                                                                            CLEAR_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                        }\
                                                                        else\
                                                                        {\
                                                                            /* Deactivate both edges as part of workaround for errata 2.2.1 */\
                                                                            CLEAR_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                            CLEAR_BIT(EXTI->RTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                        }\
                                                                        /* Clear IRQ flag */\
                                                                        __HAL_GPIO_EXTI_CLEAR_IT(SIDEWALK_RADIO_SPI_NSS_Pin);\
                                                                    } while (0)
#endif

/* Private variables ---------------------------------------------------------*/

/* SPI DMA buffers. IMPORTANT: These buffers shall be aligned to 4 bytes boundary for better performance */
SID_STM32_ALIGN_4BYTES(static uint8_t rx_ringbuf[SPI_FRAME_SIZE      * SPI_RING_BUF_FRAMES]); /* Rx buffer needs space for data AND CRC since DMA will extract the CRC from the SPI Rx FIFO */
SID_STM32_ALIGN_4BYTES(static uint8_t tx_ringbuf[SPI_FRAME_DATA_SIZE * SPI_RING_BUF_FRAMES]); /* Tx buffer needs space for data only, CRC is added automatically by the SPI peripheral if requested */

/* SPI buffer pointers */
static uint32_t tx_ringbuf_ingest_addr;
static uint32_t rx_ringbuf_extract_addr;
static uint32_t tx_ringbuf_end_addr; /* Pointer to the first byte AFTER the tx_ringbuf - used to speed up DMA Tx channel reloading */
static uint32_t rx_ringbuf_end_addr; /* Pointer to the first byte AFTER the rx_ringbuf - used to speed up processing */

/* SPI NSS handling DMA buffers */
SID_STM32_ALIGN_4BYTES(static uint16_t nss_dma_spi_cr2_values[2]);
SID_STM32_ALIGN_4BYTES(static uint8_t  nss_dma_sck_gpio_moder[2]);
static uint32_t dma_dummy_tx_pending;

static serial_bus_spi_pal_rx_data_available_cb_t spi_rx_user_callback    = NULL;
static serial_bus_spi_pal_error_cb_t             spi_error_user_callback = NULL;

static uint32_t spi_pal_initialized = FALSE;

/* External variables --------------------------------------------------------*/

extern SPI_HandleTypeDef SIDEWALK_RADIO_SPI;
extern DMA_HandleTypeDef SIDEWALK_RADIO_SPI_DMA_TX;
extern DMA_HandleTypeDef SIDEWALK_RADIO_SPI_DMA_RX;
extern DMA_HandleTypeDef SIDEWALK_RADIO_SPI_DMA_NSS_DETECT;
extern DMA_HandleTypeDef SIDEWALK_RADIO_SPI_DMA_SCK_CTRL;

/* Private function prototypes -----------------------------------------------*/

static inline HAL_StatusTypeDef _hal_like_spi_hard_reset(void);
static inline void              _reset_xfer_control_dma_channel_states(void);
static inline HAL_StatusTypeDef _hal_like_spi_start_communication(const uint32_t timeout_ticks);
static inline HAL_StatusTypeDef _hal_like_spi_wait_fifo_state_until_timeout(const uint32_t fifo, const uint32_t state, const uint32_t timeout_ticks);
static inline HAL_StatusTypeDef _hal_like_spi_wait_flag_state_until_timeout(const uint32_t flag, const FlagStatus state, const uint32_t timeout_ticks);
static inline HAL_StatusTypeDef _hal_like_spi_end_txrx_transaction(void);
static        HAL_StatusTypeDef _hal_like_spi_abort(void);
static        HAL_StatusTypeDef _hal_like_spi_instance_init(void);
static inline HAL_StatusTypeDef _hal_like_spi_dma_channel_start(DMA_HandleTypeDef * const hdma, const uint32_t src_address, const uint32_t dst_address,
                                                                const uint32_t transactions_count);
static        void              _hal_like_spi_dma_transmit_cplt(DMA_HandleTypeDef * const hdma);
static        void              _hal_like_spi_dma_error_callback(DMA_HandleTypeDef * const hdma);
static        void              _hal_like_spi_error_callback(void);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline HAL_StatusTypeDef _hal_like_spi_hard_reset(void)
{
    HAL_StatusTypeDef err = HAL_OK;

    /* Completely de-initialize SPI and related DMA channels */
    (void)HAL_SPI_DeInit(&SIDEWALK_RADIO_SPI);

    /* Hard-reset the SPI peripheral */
    switch ((uint32_t)(void*)SIDEWALK_RADIO_SPI.Instance)
    {
#if defined (SPI1)
        case (uint32_t)SPI1:
            __HAL_RCC_SPI1_FORCE_RESET();
            sid_pal_delay_us(SPI_PERIPHERAL_RESET_DELAY_US);
             __HAL_RCC_SPI1_RELEASE_RESET();
            break;
#endif /* SPI1 */

#if defined (SPI2)
        case (uint32_t)SPI2:
            __HAL_RCC_SPI2_FORCE_RESET();
            sid_pal_delay_us(SPI_PERIPHERAL_RESET_DELAY_US);
            __HAL_RCC_SPI2_RELEASE_RESET();
            break;
#endif /* SPI1 */

        default:
            SID_PAL_LOG_ERROR("Cannot hard-reset SPI. Unknown SPI instance (base address 0x%x)", (uint32_t)(void*)SIDEWALK_RADIO_SPI.Instance);
            err = HAL_ERROR;
            break;
    }

    if (err != HAL_ERROR)
    {
        /* Re-initialize the SPI peripheral and related DMA channels */
        __HAL_SPI_RESET_HANDLE_STATE(&SIDEWALK_RADIO_SPI);
        err = HAL_SPI_Init(&SIDEWALK_RADIO_SPI);
    }

    /* Indicate SPI driver is not initialized from now on */
    spi_pal_initialized = FALSE;

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _reset_xfer_control_dma_channel_states(void)
{
    /* Store channel states */
    register const uint32_t dma_nss_detect_ccr_copy = SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.Instance->CCR;
    register const uint32_t dma_sck_ctrl_ccr_copy = SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.Instance->CCR;

    /* Disable channels before re-configuring */
    __HAL_DMA_DISABLE(&SIDEWALK_RADIO_SPI_DMA_NSS_DETECT);
    __HAL_DMA_DISABLE(&SIDEWALK_RADIO_SPI_DMA_SCK_CTRL);

    /* Restore the state of the SCK GPIO pin's MODER register and SPI's CR2 DMA requests config */
    const uint32_t sck_gpio_moder_offset = SID_STM32_UTIL_POSITION_VAL(SIDEWALK_RADIO_SPI_SCK_Pin) * 2u; /* 2 bits per pin config in MODER register */
    SIDEWALK_RADIO_SPI_SCK_GPIO_Port->MODER |= (MODE_ANALOG << sck_gpio_moder_offset); /* Set SCK to Hi-Z for now */
    CLEAR_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN));

    /* Restore source address for the DMA channels */
    SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.Instance->CMAR = (uint32_t)&nss_dma_spi_cr2_values[NSS_DMA_INDEX_NSS_ACTIVATED];
    SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.Instance->CNDTR = SID_STM32_UTIL_ARRAY_SIZE(nss_dma_spi_cr2_values);

    SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.Instance->CMAR = (uint32_t)&nss_dma_sck_gpio_moder[NSS_DMA_INDEX_NSS_ACTIVATED];
    SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.Instance->CNDTR = SID_STM32_UTIL_ARRAY_SIZE(nss_dma_sck_gpio_moder);

    /* Restore channel states - this will re-enable them if they were enabled on entry to this function */
    SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.Instance->CCR = dma_nss_detect_ccr_copy;
    SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.Instance->CCR = dma_sck_ctrl_ccr_copy;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline HAL_StatusTypeDef _hal_like_spi_start_communication(const uint32_t timeout_ticks)
{
    register const uint32_t tick_start = HAL_GetTick();
    __COMPILER_BARRIER();

    HAL_StatusTypeDef err = HAL_ERROR;
    register uint32_t count = timeout_ticks * ((SystemCoreClock * 32U) >> 20U); /* Calculate timeout based on a software loop to avoid blocking issue if SysTick is disabled */

    /* Process locked */
    __HAL_LOCK(&SIDEWALK_RADIO_SPI);

    if (SIDEWALK_RADIO_SPI.State != HAL_SPI_STATE_READY)
    {
        err = HAL_BUSY;
        goto exit;
    }

    /* Ensure NSS is deselected before enabling the DMAMUX trigger */
    if (HAL_GPIO_ReadPin(SIDEWALK_RADIO_SPI_NSS_GPIO_Port, SIDEWALK_RADIO_SPI_NSS_Pin) == SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE)
    {
        /* Wait for NSS to go high, otherwise NSS-controlled channel will not operate correctly */
        SID_PAL_LOG_WARNING("SPI NSS is activated before SPI is started. Waiting for NSS deactivation...");
        while (HAL_GPIO_ReadPin(SIDEWALK_RADIO_SPI_NSS_GPIO_Port, SIDEWALK_RADIO_SPI_NSS_Pin) == SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE)
        {
            if (SID_STM32_UTIL_IS_IRQ_MODE() == FALSE)
            {
                /* We can use scheduler delays to allow other part of the firmware to do something useful while we are waiting */
                osDelay(1u);
            }
            else
            {
                /* We are in an IRQ context or the interrupts are disabled globally - scheduler delays are not possible */
                __NOP();
            }

            if (timeout_ticks != HAL_MAX_DELAY)
            {
                if (((HAL_GetTick() - tick_start) >= timeout_ticks) || (0u == count))
                {
                    SID_PAL_LOG_ERROR("SPI NSS is activated before SPI is started. Timeout on wait");
                    err = HAL_TIMEOUT;
                    goto exit;
                }

                /* Count iterations to protect from the SysTick being disabled or not incremented */
                if (count != 0u)
                {
                    count--;
                }
            }
        }
    }

    /* Enable DMAMUX trigger that handles the NSS pin transitions */
    err = HAL_DMAEx_EnableMuxRequestGenerator(&SIDEWALK_RADIO_SPI_DMA_NSS_DETECT);
    if (err != HAL_OK)
    {
        /* Update SPI error code */
        SID_PAL_LOG_ERROR("Failed to enable DMAMUX request for SPI DMA Request Control channel. HAL error 0x%x", (uint32_t)err);
        SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_DMA);
        err = HAL_ERROR;
        goto exit;
    }

    /* Enable DMAMUX trigger that handles the NSS pin transitions */
    err = HAL_DMAEx_EnableMuxRequestGenerator(&SIDEWALK_RADIO_SPI_DMA_SCK_CTRL);
    if (err != HAL_OK)
    {
        /* Update SPI error code */
        SID_PAL_LOG_ERROR("Failed to enable DMAMUX request for SPI SCK Pin Control channel. HAL error 0x%x", (uint32_t)err);
        SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_DMA);
        err = HAL_ERROR;
        goto exit;
    }

    /* Enable SPI peripheral if all other parts were configured properly */
    __HAL_SPI_ENABLE(&SIDEWALK_RADIO_SPI);

    /* Check that NSS remains deactivated  */
    if (HAL_GPIO_ReadPin(SIDEWALK_RADIO_SPI_NSS_GPIO_Port, SIDEWALK_RADIO_SPI_NSS_Pin) == SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE)
    {
        SID_PAL_LOG_ERROR("SPI NSS got activated while SPI is starting. SPI was not started to avoid undefined behavior");
        err = HAL_ERROR;
        goto exit;
    }

    /* Set HAL state indication */
    SIDEWALK_RADIO_SPI.State = HAL_SPI_STATE_BUSY_TX_RX;
    err = HAL_OK;

exit:
    if (err != HAL_OK)
    {
        /* Ensure DMAMUX generators remain disabled if something went wrong */
        (void)HAL_DMAEx_DisableMuxRequestGenerator(&SIDEWALK_RADIO_SPI_DMA_NSS_DETECT);
        (void)HAL_DMAEx_DisableMuxRequestGenerator(&SIDEWALK_RADIO_SPI_DMA_SCK_CTRL);

        /* Ensure control channels are in predefined state */
        _reset_xfer_control_dma_channel_states();
    }

    /* Process Unlocked */
    __HAL_UNLOCK(&SIDEWALK_RADIO_SPI);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline HAL_StatusTypeDef _hal_like_spi_wait_fifo_state_until_timeout(const uint32_t fifo, const uint32_t state, const uint32_t timeout_ticks)
{
    register const uint32_t tick_start = HAL_GetTick();
    __COMPILER_BARRIER();

    HAL_StatusTypeDef err = HAL_OK;
    volatile uint8_t * const ptmpreg8 = (volatile uint8_t *)&SIDEWALK_RADIO_SPI.Instance->DR;
    uint8_t  tmpreg8;
    register uint32_t count = timeout_ticks * ((SystemCoreClock * 35U) >> 20U); /* Calculate timeout based on a software loop to avoid blocking issue if SysTick is disabled */

    while ((SIDEWALK_RADIO_SPI.Instance->SR & fifo) != state)
    {
        /* If Rx FIFO shall be cleared... */
        if ((SPI_SR_FRLVL == fifo) && (SPI_FRLVL_EMPTY == state))
        {
            /* Flush Data Register by a blank read */
            tmpreg8 = *ptmpreg8;

            /* Avoid the compiler warning */
            UNUSED(tmpreg8);
        }

        if (timeout_ticks != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) >= timeout_ticks) || (0u == count))
            {
                /* Disable the SPI and reset the CRC: the CRC value should be cleared
                 * on both master and slave sides in order to resynchronize the master
                 * and slave for their respective CRC calculation
                 */

                /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
                __HAL_SPI_DISABLE_IT(&SIDEWALK_RADIO_SPI, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

                /* Disable SPI peripheral */
                __HAL_SPI_DISABLE(&SIDEWALK_RADIO_SPI);

                /* Reset CRC Calculation */
                if (SPI_CRCCALCULATION_ENABLE == SIDEWALK_RADIO_SPI.Init.CRCCalculation)
                {
                    SPI_RESET_CRC(&SIDEWALK_RADIO_SPI);
                }

                SIDEWALK_RADIO_SPI.State = HAL_SPI_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(&SIDEWALK_RADIO_SPI);

                err = HAL_TIMEOUT;
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

SID_STM32_SPEED_OPTIMIZED static inline HAL_StatusTypeDef _hal_like_spi_wait_flag_state_until_timeout(const uint32_t flag, const FlagStatus state, const uint32_t timeout_ticks)
{
    register const uint32_t tick_start = HAL_GetTick();
    __COMPILER_BARRIER();

    HAL_StatusTypeDef err = HAL_OK;
    register uint32_t count = timeout_ticks * ((SystemCoreClock * 32U) >> 20U); /* Calculate timeout based on a software loop to avoid blocking issue if SysTick is disabled */

    while ((__HAL_SPI_GET_FLAG(&SIDEWALK_RADIO_SPI, flag) ? SET : RESET) != state)
    {
        if (timeout_ticks != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) >= timeout_ticks) || (0u == count))
            {
                /* Disable the SPI and reset the CRC: the CRC value should be cleared
                 * on both master and slave sides in order to resynchronize the master
                 * and slave for their respective CRC calculation
                 */

                /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
                __HAL_SPI_DISABLE_IT(&SIDEWALK_RADIO_SPI, (SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

                /* Disable SPI peripheral */
                __HAL_SPI_DISABLE(&SIDEWALK_RADIO_SPI);

                /* Reset CRC Calculation */
                if (SPI_CRCCALCULATION_ENABLE == SIDEWALK_RADIO_SPI.Init.CRCCalculation)
                {
                    SPI_RESET_CRC(&SIDEWALK_RADIO_SPI);
                }

                SIDEWALK_RADIO_SPI.State = HAL_SPI_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(&SIDEWALK_RADIO_SPI);

                err = HAL_TIMEOUT;
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

SID_STM32_SPEED_OPTIMIZED static inline HAL_StatusTypeDef _hal_like_spi_end_txrx_transaction(void)
{
    HAL_StatusTypeDef err = HAL_ERROR;

    do
    {
        /* Control if the TX fifo is empty */
        err = _hal_like_spi_wait_fifo_state_until_timeout(SPI_FLAG_FTLVL, SPI_FTLVL_EMPTY, SPI_HAL_DEFAULT_WAIT_TIMEOUT_TICKS);
        if (err != HAL_OK)
        {
            SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_FLAG);
            break;
        }

        /* Control the BSY flag */
        err = _hal_like_spi_wait_flag_state_until_timeout(SPI_FLAG_BSY, RESET, SPI_HAL_DEFAULT_WAIT_TIMEOUT_TICKS);
        if (err != HAL_OK)
        {
            SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_FLAG);
            break;
        }

        /* Control if the RX fifo is empty */
        err = _hal_like_spi_wait_fifo_state_until_timeout(SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, SPI_HAL_DEFAULT_WAIT_TIMEOUT_TICKS);
        if (err != HAL_OK)
        {
            SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_FLAG);
            break;
        }

        /* Everything is ok */
        err = HAL_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static HAL_StatusTypeDef _hal_like_spi_abort(void)
{
    HAL_StatusTypeDef err = HAL_ERROR;

    /* Disable DMAMUX generators - don't care about error codes here since any error means the respective generator is not functional anyway */
    (void)HAL_DMAEx_DisableMuxRequestGenerator(&SIDEWALK_RADIO_SPI_DMA_NSS_DETECT);
    (void)HAL_DMAEx_DisableMuxRequestGenerator(&SIDEWALK_RADIO_SPI_DMA_SCK_CTRL);

    /* Clear ERRIE interrupt to avoid error interrupts generation during Abort procedure */
    CLEAR_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, SPI_CR2_ERRIE);

    /* Disable the SPI DMA Tx request if enabled */
    if (HAL_IS_BIT_SET(SIDEWALK_RADIO_SPI.Instance->CR2, SPI_CR2_TXDMAEN))
    {
        /* Disable Tx DMA Request */
        CLEAR_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, (SPI_CR2_TXDMAEN));

        /* Abort the SPI DMA Tx Stream/Channel : use blocking DMA Abort API (no callback) */
        SIDEWALK_RADIO_SPI.hdmatx->XferAbortCallback = NULL;

        /* Abort DMA Tx Handle linked to SPI Peripheral */
        err = HAL_DMA_Abort(SIDEWALK_RADIO_SPI.hdmatx);
        if (err != HAL_OK)
        {
            SIDEWALK_RADIO_SPI.ErrorCode = HAL_SPI_ERROR_ABORT;
        }

        err = _hal_like_spi_end_txrx_transaction();
        if (err != HAL_OK)
        {
            SIDEWALK_RADIO_SPI.ErrorCode = HAL_SPI_ERROR_ABORT;
        }

        /* Disable SPI Peripheral */
        __HAL_SPI_DISABLE(&SIDEWALK_RADIO_SPI);

        /* Ensure the Rx FIFO is empty after disabling the SPI */
        err = _hal_like_spi_wait_fifo_state_until_timeout(SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, SPI_HAL_DEFAULT_WAIT_TIMEOUT_TICKS);
        if (err != HAL_OK)
        {
            SIDEWALK_RADIO_SPI.ErrorCode = HAL_SPI_ERROR_ABORT;
        }
    }

    /* Disable the SPI DMA Rx request if enabled */
    if (HAL_IS_BIT_SET(SIDEWALK_RADIO_SPI.Instance->CR2, SPI_CR2_RXDMAEN))
    {
        /* Disable Rx DMA Request */
        CLEAR_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, (SPI_CR2_RXDMAEN));

        /* Abort the SPI DMA Rx Stream/Channel : use blocking DMA Abort API (no callback) */
        SIDEWALK_RADIO_SPI.hdmarx->XferAbortCallback = NULL;

        /* Abort DMA Rx Handle linked to SPI Peripheral */
        err = HAL_DMA_Abort(SIDEWALK_RADIO_SPI.hdmarx);
        if (err != HAL_OK)
        {
            SIDEWALK_RADIO_SPI.ErrorCode = HAL_SPI_ERROR_ABORT;
        }

        /* Disable peripheral */
        __HAL_SPI_DISABLE(&SIDEWALK_RADIO_SPI);

        /* Control the BSY flag */
        err = _hal_like_spi_wait_flag_state_until_timeout(SPI_FLAG_BSY, RESET, SPI_HAL_DEFAULT_WAIT_TIMEOUT_TICKS);
        if (err != HAL_OK)
        {
            SIDEWALK_RADIO_SPI.ErrorCode = HAL_SPI_ERROR_ABORT;
        }

        /* Ensure the Rx FIFO is empty after disabling the SPI */
        err = _hal_like_spi_wait_fifo_state_until_timeout(SPI_FLAG_FRLVL, SPI_FRLVL_EMPTY, SPI_HAL_DEFAULT_WAIT_TIMEOUT_TICKS);
        if (err != HAL_OK)
        {
            SIDEWALK_RADIO_SPI.ErrorCode = HAL_SPI_ERROR_ABORT;
        }
    }

    /* Reset Tx and Rx transfer counters */
    SIDEWALK_RADIO_SPI.RxXferCount = 0U;
    SIDEWALK_RADIO_SPI.TxXferCount = 0U;

    /* Check error during Abort procedure */
    if (HAL_SPI_ERROR_ABORT == SIDEWALK_RADIO_SPI.ErrorCode)
    {
        /* return HAL_Error in case of error during Abort procedure */
        err = HAL_ERROR;
    }
    else
    {
        /* Reset errorCode */
        SIDEWALK_RADIO_SPI.ErrorCode = HAL_SPI_ERROR_NONE;
        err = HAL_OK;
    }

    /* Clear the Error flags in the SR register */
    __HAL_SPI_CLEAR_OVRFLAG(&SIDEWALK_RADIO_SPI);
    __HAL_SPI_CLEAR_FREFLAG(&SIDEWALK_RADIO_SPI);

    /* Indicate we are in error state and require re-initialization to resume operations */
    SIDEWALK_RADIO_SPI.State = HAL_SPI_STATE_ERROR;

    return err;
}

/*----------------------------------------------------------------------------*/

static HAL_StatusTypeDef _hal_like_spi_instance_init(void)
{
    HAL_StatusTypeDef err = HAL_ERROR;

    /*--------------------- EXTI Mode Configuration for NSS pin ------------*/
    /* Enable EXTI in SYSCFG */
    uint32_t position = SID_STM32_UTIL_POSITION_VAL(SIDEWALK_RADIO_SPI_NSS_Pin);
    uint32_t temp = SYSCFG->EXTICR[position >> 2u];
    temp &= ~(0x07uL << (4U * (position & 0x03U)));
    temp |= (GPIO_GET_INDEX(SIDEWALK_RADIO_SPI_NSS_GPIO_Port) << (4U * (position & 0x03U)));
    SYSCFG->EXTICR[position >> 2u] = temp;

    /* Clear any residual events */
    CLEAR_BIT(EXTI->RTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);
    CLEAR_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_NSS_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(SIDEWALK_RADIO_SPI_NSS_Pin);

    /* Enable NSS deactivation event for the pin. For Rev Z both activation and deactivation shall be enabled as part of workaround for errata 2.2.1 */
    SIDEWALK_RADIO_SPI_NSS_ENABLE_TRIGGER();

    /* Trigger EXTI interrupt, but don't generate event */
#ifdef CORE_CM0PLUS
    SET_BIT(EXTI->C2IMR1, SIDEWALK_RADIO_SPI_NSS_Pin);
    CLEAR_BIT(EXTI->C2EMR1, SIDEWALK_RADIO_SPI_NSS_Pin);
#else
    SET_BIT(EXTI->IMR1, SIDEWALK_RADIO_SPI_NSS_Pin);
    CLEAR_BIT(EXTI->EMR1, SIDEWALK_RADIO_SPI_NSS_Pin);
#endif /* CORE_CM0PLUS */

    /* Setup SPI buffer pointers */
    rx_ringbuf_extract_addr = (uint32_t)&rx_ringbuf[0];
    rx_ringbuf_end_addr = (uint32_t)&rx_ringbuf[0] + sizeof(rx_ringbuf);
    tx_ringbuf_ingest_addr = (uint32_t)&tx_ringbuf[0];
    tx_ringbuf_end_addr = (uint32_t)&tx_ringbuf[0] + sizeof(tx_ringbuf);

    /* Configure Rx FIFO overrun diagnostic - since we know the first extract position we don't actually need to
     * prepopulate the entire Rx ring buffer with watermarks. It's enough to write down a single watermark to the
     * location that preceeds the extractions position. 
    */
    uint32_t overrun_check_addr = rx_ringbuf_extract_addr;
    SPI_RINGBUF_REWIND_POSITION(overrun_check_addr, sizeof(SPI_RINGBUF_OVERRUN_DIAG_WATERMARK), (uint32_t)&rx_ringbuf[0], rx_ringbuf_end_addr);
    uint32_t * const overrun_watermark_ptr = (uint32_t *)(void *)overrun_check_addr;
    *overrun_watermark_ptr = SPI_RINGBUF_OVERRUN_DIAG_WATERMARK;

    /* Check rx & tx dma handles */
    assert_param(IS_SPI_DMA_HANDLE(SIDEWALK_RADIO_SPI.hdmarx));
    assert_param(IS_SPI_DMA_HANDLE(SIDEWALK_RADIO_SPI.hdmatx));

    /* Check Direction parameter */
    assert_param(IS_SPI_DIRECTION_2LINES(SIDEWALK_RADIO_SPI.Init.Direction));

    /* Check SPI data frame settings */
    assert_param(SPI_DATASIZE_8BIT == SIDEWALK_RADIO_SPI.Init.DataSize); /* Only 8 bit data size is accepted, otherwise the bytes will be swapped due to Little Endian nature of the MCU */
    assert_param(SPI_POLARITY_LOW == SIDEWALK_RADIO_SPI.Init.CLKPolarity); /* Only low SCK polarity is supported due to hardware limitations of the implemented NSS control mechanism */

    /* NSS DMA channels params check */
    assert_param(sizeof(uint16_t)        == sizeof(nss_dma_spi_cr2_values[0]));
    assert_param(DMA_PDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.Init.PeriphDataAlignment);
    assert_param(DMA_MDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.Init.MemDataAlignment);
    assert_param(DMA_CIRCULAR            == SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.Init.Mode);

    assert_param(sizeof(uint8_t)         == sizeof(nss_dma_sck_gpio_moder[0]));
    assert_param(DMA_PDATAALIGN_BYTE     == SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.Init.PeriphDataAlignment);
    assert_param(DMA_MDATAALIGN_BYTE     == SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.Init.MemDataAlignment);
    assert_param(DMA_CIRCULAR            == SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.Init.Mode);

    /* Check Tx DMA setting */
    assert_param((DMA_MDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI.hdmatx->Init.MemDataAlignment) || (DMA_MDATAALIGN_BYTE == SIDEWALK_RADIO_SPI.hdmatx->Init.MemDataAlignment));
    if (DMA_MDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI.hdmatx->Init.MemDataAlignment)
    {
        assert_param(DMA_PDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI.hdmatx->Init.PeriphDataAlignment);
    }
    else
    {
        assert_param(DMA_PDATAALIGN_BYTE == SIDEWALK_RADIO_SPI.hdmatx->Init.PeriphDataAlignment);
    }

    assert_param(DMA_NORMAL == SIDEWALK_RADIO_SPI.hdmatx->Init.Mode);

    /* Check Rx DMA setting */
    assert_param((DMA_MDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI.hdmarx->Init.MemDataAlignment) || (DMA_MDATAALIGN_BYTE == SIDEWALK_RADIO_SPI.hdmarx->Init.MemDataAlignment));
    if (DMA_MDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI.hdmarx->Init.MemDataAlignment)
    {
        assert_param(DMA_PDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI.hdmarx->Init.PeriphDataAlignment);
    }
    else
    {
        assert_param(DMA_PDATAALIGN_BYTE == SIDEWALK_RADIO_SPI.hdmarx->Init.PeriphDataAlignment);
    }
    assert_param(DMA_CIRCULAR == SIDEWALK_RADIO_SPI.hdmarx->Init.Mode);

    /* Process locked */
    __HAL_LOCK(&SIDEWALK_RADIO_SPI);

    if (SIDEWALK_RADIO_SPI.State != HAL_SPI_STATE_READY)
    {
        err = HAL_BUSY;
        goto exit;
    }

    /* Abort any ongoing transaction and bring the SPI peripheral to the Disabled state before any further configuration will take place */
    err = _hal_like_spi_abort();
    if (err != HAL_OK)
    {
        /* Failed to abort the SPI, try hard-reset instead */
        SID_PAL_LOG_WARNING("Failed to perform SPI abort. HAL error 0x%x. Will try hard reset instead", (uint32_t)err);

        err = _hal_like_spi_hard_reset();
        if (err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to perform SPI hard reset. HAL error 0x%x. Further initialization is not possible", (uint32_t)err);
            goto exit;
        }
    }
    else
    {
        /* Restore Ready state after successful abort */
        SIDEWALK_RADIO_SPI.State = HAL_SPI_STATE_READY;
    }

    /* Set the transaction information */
    SIDEWALK_RADIO_SPI.ErrorCode   = HAL_SPI_ERROR_NONE;
    SIDEWALK_RADIO_SPI.pTxBuffPtr  = (uint8_t *)tx_ringbuf;
    SIDEWALK_RADIO_SPI.TxXferSize  = SPI_FRAME_DATA_SIZE;
    SIDEWALK_RADIO_SPI.TxXferCount = SIDEWALK_RADIO_SPI.TxXferSize; /* Initialize with bytes count, Tx channel always loops within one frame */
    SIDEWALK_RADIO_SPI.pRxBuffPtr  = (uint8_t *)rx_ringbuf;
    SIDEWALK_RADIO_SPI.RxXferSize  = sizeof(rx_ringbuf);
    SIDEWALK_RADIO_SPI.RxXferCount = SIDEWALK_RADIO_SPI.RxXferSize; /* Initialize with bytes count, actual xfer number will be adjusted later based on DMA transaction width */

    /* Init field not used in handle to zero */
    SIDEWALK_RADIO_SPI.RxISR       = NULL;
    SIDEWALK_RADIO_SPI.TxISR       = NULL;

    /* Reset CRC Calculation */
    if (SPI_CRCCALCULATION_ENABLE == SIDEWALK_RADIO_SPI.Init.CRCCalculation)
    {
        assert_param(USE_SPI_CRC != 0u); /* Ensure STM32_HAL is properly configured for SPI CRC usage */
        SPI_RESET_CRC(&SIDEWALK_RADIO_SPI);
    }

    /* Reset the threshold bit */
    CLEAR_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, SPI_CR2_LDMATX | SPI_CR2_LDMARX);

    /* We are not going to use odd number of transfers anyway */
    CLEAR_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, SPI_CR2_LDMATX);
    CLEAR_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, SPI_CR2_LDMARX);

    /* The packing mode management is enabled by the DMA settings according the spi data size */
    if (DMA_MDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI.hdmatx->Init.MemDataAlignment)
    {
        /* Ensure Rx & Tx buffers have the aligned size */
        assert_param(SID_STM32_UTIL_ARRAY_SIZE(rx_ringbuf) % sizeof(uint16_t) == 0u);
        assert_param(SID_STM32_UTIL_ARRAY_SIZE(tx_ringbuf) % sizeof(uint16_t) == 0u);

        /* Adjust the number of xfers for the DMA Rx channel */
        SIDEWALK_RADIO_SPI.TxXferCount = SIDEWALK_RADIO_SPI.TxXferCount >> 1; /* Each transaction will be 2 bytes, so we need twice less xfers */
    }

    if (DMA_MDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI.hdmarx->Init.MemDataAlignment)
    {
        /* Set fiforxthreshold according the DMA transaction length: 16bit */
        CLEAR_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, SPI_RXFIFO_THRESHOLD);

        /* Ensure Rx & Tx buffers have the aligned size */
        assert_param(SID_STM32_UTIL_ARRAY_SIZE(rx_ringbuf) % sizeof(uint16_t) == 0u);
        assert_param(SID_STM32_UTIL_ARRAY_SIZE(tx_ringbuf) % sizeof(uint16_t) == 0u);

        /* Adjust the number of xfers for the DMA Rx channel */
        SIDEWALK_RADIO_SPI.RxXferCount = SIDEWALK_RADIO_SPI.RxXferCount >> 1; /* Each transaction will be 2 bytes, so we need twice less xfers */
    }
    else
    {
        /* Set RX Fifo threshold according the DMA transaction length: 8bit */
        SET_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, SPI_RXFIFO_THRESHOLD);
    }

    /* Set the DMA error callback */
    SIDEWALK_RADIO_SPI.hdmarx->XferErrorCallback = _hal_like_spi_dma_error_callback;

    /* Clear the unsued DMA callbacks */
    SIDEWALK_RADIO_SPI.hdmarx->XferHalfCpltCallback = NULL;
    SIDEWALK_RADIO_SPI.hdmarx->XferCpltCallback     = NULL;
    SIDEWALK_RADIO_SPI.hdmarx->XferAbortCallback    = NULL;

    /* Enable the Rx DMA Stream/Channel */
    err = _hal_like_spi_dma_channel_start(SIDEWALK_RADIO_SPI.hdmarx, (uint32_t)&SIDEWALK_RADIO_SPI.Instance->DR,
                                          (uint32_t)SIDEWALK_RADIO_SPI.pRxBuffPtr, SIDEWALK_RADIO_SPI.RxXferCount);
    if (err != HAL_OK)
    {
        /* Update SPI error code */
        SID_PAL_LOG_ERROR("Failed to start SPI DMA Rx channel. HAL error 0x%x", (uint32_t)err);
        SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_DMA);
        err = HAL_ERROR;

        SIDEWALK_RADIO_SPI.State = HAL_SPI_STATE_READY;
        goto exit;
    }

    /* Set the DMA error callback */
    SIDEWALK_RADIO_SPI.hdmatx->XferErrorCallback    = _hal_like_spi_dma_error_callback;

    /* Set the SPI Tx/Rx DMA Half transfer complete callback */
    SIDEWALK_RADIO_SPI.hdmatx->XferCpltCallback     = _hal_like_spi_dma_transmit_cplt;

    /* Clear the unsued DMA callbacks */
    SIDEWALK_RADIO_SPI.hdmatx->XferHalfCpltCallback = NULL;
    SIDEWALK_RADIO_SPI.hdmatx->XferAbortCallback    = NULL;

    /* Prepare the first slot for a dummy Tx transaction */
    dma_dummy_tx_pending = TRUE;

    /* Enable the Tx DMA Stream/Channel */
    err = _hal_like_spi_dma_channel_start(SIDEWALK_RADIO_SPI.hdmatx, (uint32_t)SIDEWALK_RADIO_SPI.pTxBuffPtr, (uint32_t)&SIDEWALK_RADIO_SPI.Instance->DR,
                                          SIDEWALK_RADIO_SPI.TxXferCount);
    if (err != HAL_OK)
    {
        /* Update SPI error code */
        SID_PAL_LOG_ERROR("Failed to start SPI DMA Tx channel. HAL error 0x%x", (uint32_t)err);
        SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_DMA);
        err = HAL_ERROR;

        SIDEWALK_RADIO_SPI.State = HAL_SPI_STATE_READY;
        goto exit;
    }

    /* Configure SPI interrupts */
    __HAL_SPI_DISABLE_IT(&SIDEWALK_RADIO_SPI, (SPI_IT_TXE | SPI_IT_RXNE));
    __HAL_SPI_ENABLE_IT(&SIDEWALK_RADIO_SPI, SPI_IT_ERR);

    /* Clear any residual interrupt flags */
    __HAL_SPI_CLEAR_FREFLAG(&SIDEWALK_RADIO_SPI);
    __HAL_SPI_CLEAR_MODFFLAG(&SIDEWALK_RADIO_SPI);
    __HAL_SPI_CLEAR_OVRFLAG(&SIDEWALK_RADIO_SPI);

    /* Use Software-controled NSS and set it to permanently selected state to allow automatic CRC counters reset at the end of the frame */
    SET_BIT(SIDEWALK_RADIO_SPI.Instance->CR1, SPI_CR1_SSM);
    CLEAR_BIT(SIDEWALK_RADIO_SPI.Instance->CR1, SPI_CR1_SSI);

    /* Configure DMA channel that controls SPI operation based on physical NSS pin transitions */
    nss_dma_spi_cr2_values[NSS_DMA_INDEX_NSS_ACTIVATED] = nss_dma_spi_cr2_values[NSS_DMA_INDEX_NSS_DEACTIVATED] = SIDEWALK_RADIO_SPI.Instance->CR2; /* Current state of SPI CR2 reg with SPI_CR2_TXDMAEN and SPI_CR2_RXDMAEN cleared - to be applied upon physical NSS goes high */
    nss_dma_spi_cr2_values[NSS_DMA_INDEX_NSS_ACTIVATED] = nss_dma_spi_cr2_values[NSS_DMA_INDEX_NSS_DEACTIVATED] | (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);  /* CR2 with SPI_CR2_RXDMAEN and SPI_CR2_TXDMAEN set - to be applied upon physical NSS goes low */

    /* Configure DMA writes to the SCK GPIO pin's MODER register to simulate SPI enabling/disabling by the NSS line. Use 8bit write operations
     * for MODER to reduce the amount of affected adjacent pins - only 4 pins instead of 16 on the same port will be affected because pin mode
     * configuration occupies two bits per pin.
     */
    const uint32_t sck_gpio_moder_position                 = SID_STM32_UTIL_POSITION_VAL(SIDEWALK_RADIO_SPI_SCK_Pin) * 2u; /* 2 bits per pin config in MODER register */
    const uint32_t sck_gpio_moder_byte_offset              = sck_gpio_moder_position    >> 3; /* byte addition to the MODER address since DMA will perform single byte writes */
    const uint32_t sck_gpio_moder_byte_shift               = sck_gpio_moder_byte_offset << 3; /* bit shift value to get the byte of interest from the MODER reg */
    nss_dma_sck_gpio_moder[NSS_DMA_INDEX_NSS_ACTIVATED]    = nss_dma_sck_gpio_moder[NSS_DMA_INDEX_NSS_DEACTIVATED] =  (uint8_t)(SIDEWALK_RADIO_SPI_SCK_GPIO_Port->MODER >> sck_gpio_moder_byte_shift);
    nss_dma_sck_gpio_moder[NSS_DMA_INDEX_NSS_ACTIVATED]    = nss_dma_sck_gpio_moder[NSS_DMA_INDEX_NSS_DEACTIVATED] &= (uint8_t)(~(GPIO_MODE << (sck_gpio_moder_position - sck_gpio_moder_byte_shift))); /* Clear MODE config */
    nss_dma_sck_gpio_moder[NSS_DMA_INDEX_NSS_ACTIVATED]   |= (uint8_t)(MODE_AF     << (sck_gpio_moder_position - sck_gpio_moder_byte_shift)); /* Configure for NSS select - Connect SCK to the SPI peripheral */
    nss_dma_sck_gpio_moder[NSS_DMA_INDEX_NSS_DEACTIVATED] |= (uint8_t)(MODE_ANALOG << (sck_gpio_moder_position - sck_gpio_moder_byte_shift)); /* Configure for NSS de-select - set SCK to Hi-Z - internal logic will be forced to zero */
    SIDEWALK_RADIO_SPI_SCK_GPIO_Port->MODER               |= (MODE_ANALOG << sck_gpio_moder_position); /* Set SCK to Hi-Z for now */

    /* Set the DMA error callbacks */
    SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.XferErrorCallback    = _hal_like_spi_dma_error_callback;
    SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.XferErrorCallback      = _hal_like_spi_dma_error_callback;

    /* No other callbacks and DMA interrupts needed as this channel operates autonomously in circular mode */
    SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.XferHalfCpltCallback = NULL;
    SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.XferCpltCallback     = NULL;
    SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.XferAbortCallback    = NULL;

    SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.XferHalfCpltCallback   = NULL;
    SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.XferCpltCallback       = NULL;
    SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.XferAbortCallback      = NULL;

    /* Enable NSS-controled DMA channel */
    err = _hal_like_spi_dma_channel_start(&SIDEWALK_RADIO_SPI_DMA_NSS_DETECT,
                                          (uint32_t)&nss_dma_spi_cr2_values[NSS_DMA_INDEX_NSS_ACTIVATED],
                                          (uint32_t)&SIDEWALK_RADIO_SPI.Instance->CR2,
                                          SID_STM32_UTIL_ARRAY_SIZE(nss_dma_spi_cr2_values));
    if (err != HAL_OK)
    {
        /* Update SPI error code */
        SID_PAL_LOG_ERROR("Failed to start SPI DMA Requests Control DMA channel. HAL error 0x%x", (uint32_t)err);
        SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_DMA);
        err = HAL_ERROR;

        SIDEWALK_RADIO_SPI.State = HAL_SPI_STATE_READY;
        goto exit;
    }

    /* Enable NSS-controled DMA channel */
    err = _hal_like_spi_dma_channel_start(&SIDEWALK_RADIO_SPI_DMA_SCK_CTRL,
                                          (uint32_t)&nss_dma_sck_gpio_moder[NSS_DMA_INDEX_NSS_ACTIVATED],
                                          (uint32_t)&SIDEWALK_RADIO_SPI_SCK_GPIO_Port->MODER + sck_gpio_moder_byte_offset,
                                          SID_STM32_UTIL_ARRAY_SIZE(nss_dma_sck_gpio_moder));
    if (err != HAL_OK)
    {
        /* Update SPI error code */
        SID_PAL_LOG_ERROR("Failed to start SPI SCK Pin Control DMA channel. HAL error 0x%x", (uint32_t)err);
        SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_DMA);
        err = HAL_ERROR;

        SIDEWALK_RADIO_SPI.State = HAL_SPI_STATE_READY;
        goto exit;
    }

    /* If we got here everything is ok, indicate init is done */
    spi_pal_initialized = TRUE;

exit:
    if (err != HAL_OK)
    {
        /* Disable EXTI and DMA if configuration error happened */
        (void)HAL_DMAEx_DisableMuxRequestGenerator(&SIDEWALK_RADIO_SPI_DMA_NSS_DETECT);
        (void)HAL_DMAEx_DisableMuxRequestGenerator(&SIDEWALK_RADIO_SPI_DMA_SCK_CTRL);
        CLEAR_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN));

        (void)HAL_DMA_Abort(&SIDEWALK_RADIO_SPI_DMA_NSS_DETECT);
        (void)HAL_DMA_Abort(&SIDEWALK_RADIO_SPI_DMA_SCK_CTRL);
        (void)HAL_DMA_Abort(SIDEWALK_RADIO_SPI.hdmatx);
        (void)HAL_DMA_Abort(SIDEWALK_RADIO_SPI.hdmarx);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(&SIDEWALK_RADIO_SPI);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline HAL_StatusTypeDef _hal_like_spi_dma_channel_start(DMA_HandleTypeDef * const hdma, const uint32_t src_address, const uint32_t dst_address,
                                                                                          const uint32_t transactions_count)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* Check the parameters */
    assert_param(IS_DMA_BUFFER_SIZE(transactions_count));

    /* Process locked */
    __HAL_LOCK(hdma);

    if (hdma->State == HAL_DMA_STATE_READY)
    {
        /* Change DMA peripheral state */
        hdma->State = HAL_DMA_STATE_BUSY;
        hdma->ErrorCode = HAL_DMA_ERROR_NONE;

        /* Disable the peripheral */
        __HAL_DMA_DISABLE(hdma);

        /* Configure the source, destination address and the data length & clear flags*/
        /* Clear the DMAMUX synchro overrun flag */
        hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;

        if (hdma->DMAmuxRequestGen != NULL)
        {
            /* Clear the DMAMUX request generator overrun flag */
            hdma->DMAmuxRequestGenStatus->RGCFR = hdma->DMAmuxRequestGenStatusMask;
        }

        /* Clear all flags */
        hdma->DmaBaseAddress->IFCR = (DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1CU));

        /* Configure DMA Channel data length */
        hdma->Instance->CNDTR = transactions_count;

        /* Memory to Peripheral */
        if ((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
        {
            /* Configure DMA Channel destination address */
            hdma->Instance->CPAR = dst_address;

            /* Configure DMA Channel source address */
            hdma->Instance->CMAR = src_address;
        }
        /* Peripheral to Memory */
        else
        {
            /* Configure DMA Channel source address */
            hdma->Instance->CPAR = src_address;

            /* Configure DMA Channel destination address */
            hdma->Instance->CMAR = dst_address;
        }

        /* Disable all interrupts */
        __HAL_DMA_DISABLE_IT(hdma, (DMA_IT_TC | DMA_IT_HT | DMA_IT_TE));

        /* Enable the Transfer Complete interrupt if requested */
        if (NULL != hdma->XferCpltCallback)
        {
            __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TC);
        }

        /* Enable the Half-Transfer Complete interrupt if requested */
        if (NULL != hdma->XferHalfCpltCallback)
        {
            __HAL_DMA_ENABLE_IT(hdma, DMA_IT_HT | DMA_IT_TE);
        }

        /* Enable the transfer Error interrupt if requested */
        if (NULL != hdma->XferHalfCpltCallback)
        {
            /* Enable the Half transfer complete interrupt as well */
            __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TE);
        }

        /* Check if DMAMUX Synchronization is enabled*/
        if ((hdma->DMAmuxChannel->CCR & DMAMUX_CxCR_SE) != 0U)
        {
            /* Enable DMAMUX sync overrun IT*/
            hdma->DMAmuxChannel->CCR |= DMAMUX_CxCR_SOIE;
        }

        if (hdma->DMAmuxRequestGen != NULL)
        {
            /* if using DMAMUX request generator, enable the DMAMUX request generator overrun IT*/
            /* enable the request gen overrun IT*/
            hdma->DMAmuxRequestGen->RGCR |= DMAMUX_RGxCR_OIE;
        }

        /* Enable the Peripheral */
        __HAL_DMA_ENABLE(hdma);
    }
    else
    {
        /* Change the error code */
        hdma->ErrorCode = HAL_DMA_ERROR_BUSY;

        /* Process Unlocked */
        __HAL_UNLOCK(hdma);

        /* Return error status */
        status = HAL_ERROR;
    }

    return status;
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  STM32 HAL style DMA TC event callback to handle SPI's end of transaction. This callback
  *         simulates circular mode for the DMA channel and supports the continuous SPI Tx/Rx operation.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains  the configuration information
  *              for the specified DMA module.
  * @retval None
 */
SID_STM32_SPEED_OPTIMIZED static void _hal_like_spi_dma_transmit_cplt(DMA_HandleTypeDef * const hdma)
{
    /* Disable all IRQs since it's essential to complete this callback ASAP and that tx_ringbuf_ingest_addr is not modified during the operation */
    UTILS_ENTER_CRITICAL_SECTION();

    /* Prevent any further DMA Tx requests */
    CLEAR_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, SPI_CR2_TXDMAEN);
    __HAL_DMA_DISABLE(hdma);
    __COMPILER_BARRIER(); /* Ensure DMA channels is disabled an DMA requests are stopped before any further actions */

    /* Move to the next frame placeholder in the ring buffer */
    const register uint32_t prev_dma_src_addr = hdma->Instance->CMAR;
    register uint32_t next_dma_src_addr = prev_dma_src_addr;

    /* Calculate the address of the next frame */
    next_dma_src_addr += SPI_FRAME_DATA_SIZE;
    if (next_dma_src_addr >= tx_ringbuf_end_addr)
    {
        /* Full circle done, rewind to the beginning */
        next_dma_src_addr = (uint32_t)&tx_ringbuf[0]; /* Don't use SIDEWALK_RADIO_SPI.pTxBuffPtr here since it will take more CPU instructions to load */
    }

    /* Act depending on the type of transaction that have just ended */
    if (FALSE == dma_dummy_tx_pending)
    {
        /* Handle the completion of a normal transaction */
        if (next_dma_src_addr == tx_ringbuf_ingest_addr)
        {
            /* Currently there's no more data to transmit, enter the dummy transaction mode */
            dma_dummy_tx_pending = TRUE;

            /* Clear the slot that will be used for dummy Tx to avoid sending out duplicated responses
             *
             * IMPORTANT: not using memset() here to avoid branching instructions and maximize the processing speed.
             *            It is essential to ensure this handler completes ASAP
             */
            SPI_RINGBUF_FRAME_PARRETN_SET(next_dma_src_addr, 0x00000000u);
        }

        /* Update CMAR to point to the next transaction start address */
        hdma->Instance->CMAR = next_dma_src_addr;
    }
    else
    {
        /* Handle the completion of a dummy transaction */

        /* If tx_ringbuf_ingest_addr has moved it means we have new data to send out, dummy mode is not needed anymore */
        if (prev_dma_src_addr != tx_ringbuf_ingest_addr)
        {
            dma_dummy_tx_pending = FALSE;
        }

        /* Don't touch CMAR reg as the payload transfer will commence from the same SPI frame slot as dummy transaction */
        /* hdma->Instance->CMAR = prev_dma_src_addr; - keep old address, don't actually uncomment this line to save some runtime. CMAR is already set to prev_dma_src_addr */
    }

    /* Restore transactions count register */
    hdma->Instance->CNDTR = SIDEWALK_RADIO_SPI.TxXferCount;

    /* Re-enable needed DMA interrupts since they are turned off by the generic HAL handler */
    __HAL_DMA_ENABLE_IT(hdma, DMA_IT_TE | DMA_IT_TC);
    __COMPILER_BARRIER(); /* Ensure all adjustments are applied before re-enablid the DMA channel */

    /* Re-enable DMA Tx channel after the configuration update */
    __HAL_DMA_ENABLE(hdma);
    /* Restore virtual state for compativbility with the HAL layer */
    hdma->State = HAL_DMA_STATE_BUSY;

    UTILS_EXIT_CRITICAL_SECTION();
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  STM32 HAL style DMA SPI communication error callback.
  * @param  hdma pointer to a DMA_HandleTypeDef structure that contains the configuration information for the specified DMA module.
  * @retval None
  */
SID_STM32_SPEED_OPTIMIZED static void _hal_like_spi_dma_error_callback(DMA_HandleTypeDef * const hdma)
{
    (void)hdma;

    /* Indicate DMA error */
    SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_DMA);
    SIDEWALK_RADIO_SPI.State = HAL_SPI_STATE_READY;

    /* Call the unified SPI/DMA error handler */
    _hal_like_spi_error_callback();
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  STM32 HAL style SPI communication error callback.
  *         SPI handle is not required since it's guaranteed by the custom SPI IRQ handler implementation in this file
  *         that the only trigger source is SIDEWALK_RADIO_SPI
  * @retval None
  */
SID_STM32_SPEED_OPTIMIZED static void _hal_like_spi_error_callback(void)
{
    serial_bus_spi_pal_bus_error_details_t error_info;

    /* Collect SPI state info before processing the error */
    if (spi_error_user_callback != NULL)
    {
        error_info.error_code       = SIDEWALK_RADIO_SPI.ErrorCode;
        error_info.spi_sr           = SIDEWALK_RADIO_SPI.Instance->SR;
        error_info.dma_tx_ccr       = SIDEWALK_RADIO_SPI_DMA_TX.Instance->CCR;
        error_info.dma_rx_ccr       = SIDEWALK_RADIO_SPI_DMA_RX.Instance->CCR;
        error_info.dma_nss_dtct_ccr = SIDEWALK_RADIO_SPI_DMA_NSS_DETECT.Instance->CCR;
        error_info.dma_sck_ctrl_ccr = SIDEWALK_RADIO_SPI_DMA_SCK_CTRL.Instance->CCR;
    }
    __COMPILER_BARRIER();

    /* Stop any further SPI & DMA operations */
    (void)_hal_like_spi_abort();

    /* Invoke the user callback if it is defined */
    if (spi_error_user_callback != NULL)
    {
        /* Add any new errors that may have occured during SPI Abort */
        error_info.error_code |= SIDEWALK_RADIO_SPI.ErrorCode;
        spi_error_user_callback(&error_info);
    }
}

/* Global function definitions -----------------------------------------------*/

serial_bus_spi_pal_err_t sid_pal_serial_bus_spi_init(serial_bus_spi_pal_rx_data_available_cb_t rx_data_available_callback,
                                                     serial_bus_spi_pal_error_cb_t error_callback)
{
    serial_bus_spi_pal_err_t err = SERIAL_BUS_SPI_PAL_OK;
    HAL_StatusTypeDef hal_err;

    do
    {
        /* Ensure SPI is not initialized already */
        if (spi_pal_initialized != FALSE)
        {
            err = SERIAL_BUS_SPI_PAL_ALREADY_INITIALIZED;
            break;
        }

        /* Set user callbacks */
        spi_rx_user_callback = rx_data_available_callback;
        spi_error_user_callback = error_callback;

        /* Now init the hardware (SPI, DMA, GPIO) */
        hal_err = _hal_like_spi_instance_init();
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Host comm SPI initialization failed. HAL error 0x%x", (uint32_t)hal_err);
            err = SERIAL_BUS_SPI_PAL_HARDWARE_ERROR;
            break;
        }
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED serial_bus_spi_pal_err_t serial_bus_spi_xfer_start(void)
{
    serial_bus_spi_pal_err_t err = SERIAL_BUS_SPI_PAL_ERROR;
    HAL_StatusTypeDef hal_err;

    do
    {
        /* Check that prior initializations were completed successfully */
        if (FALSE == spi_pal_initialized)
        {
            err = SERIAL_BUS_SPI_PAL_NOT_INITIALIZED;
            break;
        }

        /* Check that SPI is enabled */
        if (READ_BIT(SIDEWALK_RADIO_SPI.Instance->CR1, SPI_CR1_SPE) != 0u)
        {
            err = SERIAL_BUS_SPI_PAL_INVALID_STATE;
            break;
        }

        /* Enable SPI peripheral if all other parts were configured properly */
        hal_err = _hal_like_spi_start_communication(SPI_HAL_COMM_START_TIMEOUT_TICKS);
        if (hal_err != HAL_OK)
        {
            if (HAL_TIMEOUT == hal_err)
            {
                /* This is recoverable */
                err = SERIAL_BUS_SPI_PAL_TRY_LATER;
            }
            else
            {
                /* SPI and DMA state may be inconsistent, full-scale reset&restart is required */
                err = SERIAL_BUS_SPI_PAL_HARDWARE_ERROR;
            }
            break;
        }

        err = SERIAL_BUS_SPI_PAL_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED serial_bus_spi_pal_err_t serial_bus_spi_full_reinit(void)
{
    serial_bus_spi_pal_err_t err = SERIAL_BUS_SPI_PAL_ERROR;
    HAL_StatusTypeDef hal_err;

    do
    {
        /* Abort any ongoing SPI transactions */
        (void)_hal_like_spi_abort();

        /* Perform a hard reset of the SPI peripheral */
        hal_err = _hal_like_spi_hard_reset();
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Failed to perform SPI hard reset. HAL error 0x%x. Further initialization is not possible", (uint32_t)hal_err);
            err = SERIAL_BUS_SPI_PAL_HARDWARE_ERROR;
            break;
        }

        hal_err = _hal_like_spi_instance_init();
        if (hal_err != HAL_OK)
        {
            SID_PAL_LOG_ERROR("Host comm SPI reinitialization failed. HAL error 0x%x", (uint32_t)hal_err);
            err = SERIAL_BUS_SPI_PAL_HARDWARE_ERROR;
            break;
        }

        err =SERIAL_BUS_SPI_PAL_OK;
    } while (0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED serial_bus_spi_pal_err_t serial_bus_spi_clear_fifos(const uint32_t timeout_ms)
{
    register const uint32_t tick_start = HAL_GetTick();
    __COMPILER_BARRIER();

    register const uint32_t timeout_ticks = timeout_ms != osWaitForever ? (timeout_ms / (uint32_t)HAL_GetTickFreq()) : HAL_MAX_DELAY;
    register uint32_t count = timeout_ticks * ((SystemCoreClock * 26U) >> 20U); /* Calculate timeout based on a software loop to avoid blocking issue if SysTick is disabled */

    /* Disable IRQ upon NSS deactivation to prevent software reaction on the frame that may be transmitted right now */
    SIDEWALK_RADIO_SPI_NSS_DISABLE_TRIGGER();

    /* Wait for any ongoing transactions to complete */
    while (READ_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN)) != 0u)
    {
        if (timeout_ticks != HAL_MAX_DELAY)
        {
            if (((HAL_GetTick() - tick_start) >= timeout_ticks) || (0u == count))
            {
                return SERIAL_BUS_SPI_PAL_TIMEOUT;
            }

            /* Count iterations to protect from the SysTick being disabled or not incremented */
            if (count != 0u)
            {
                count--;
            }
        }
    }

    /* Disable interrupts and place the data into the buffer */
    UTILS_ENTER_CRITICAL_SECTION();
    do
    {
        const register uint32_t current_tx_pos = SIDEWALK_RADIO_SPI.hdmatx->Instance->CMAR; /* Capture the start address of the current DMA Tx operation */
        const register uint32_t current_rx_cndtr = SIDEWALK_RADIO_SPI.hdmarx->Instance->CNDTR; /* Capture the CNDTR register of the DMA Rx operation. Wecannot use CMAR register since it's not updated in circular mode */
        register uint32_t current_rx_ingest_addr;

        /* Ensure there's no systematic SW failure */
        assert_param((current_tx_pos >= (uint32_t)&tx_ringbuf[0]) && (current_tx_pos <= tx_ringbuf_end_addr));

        if (DMA_MDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI.hdmarx->Init.MemDataAlignment)
        {
            /* Calculate the nearest CNDTR for the fully completed SPI frames */
            const register uint32_t dma_transactions_per_spi_frame = SPI_FRAME_SIZE >> 1;
            const register uint32_t frame_aligned_cndtr = (current_rx_cndtr % dma_transactions_per_spi_frame) != 0u ?
            current_rx_cndtr - (dma_transactions_per_spi_frame - (current_rx_cndtr % dma_transactions_per_spi_frame)) : current_rx_cndtr;

            /* Calculate the write start pointer in the Rx buffer */
            current_rx_ingest_addr = rx_ringbuf_end_addr - (sizeof(uint16_t) * frame_aligned_cndtr);
        }
        else /* DMA_MDATAALIGN_BYTE */
        {
            /* The only other option for STM32WLxx is 8 bit for MemDataAlignment */
            /* Calculate the nearest CNDTR for the fully completed SPI frames */
            const register uint32_t dma_transactions_per_spi_frame = SPI_FRAME_SIZE;
            const register uint32_t frame_aligned_cndtr = (current_rx_cndtr % dma_transactions_per_spi_frame) != 0u ?
            current_rx_cndtr - (dma_transactions_per_spi_frame - (current_rx_cndtr % dma_transactions_per_spi_frame)) : current_rx_cndtr;

            /* Calculate the write start pointer in the Rx buffer */
            current_rx_ingest_addr = rx_ringbuf_end_addr - (sizeof(uint8_t) * frame_aligned_cndtr);
        }

        /* Handle a corner case when CNDTR was captured during DMA channel reload (CNDTR == 0)  */
        if (current_rx_ingest_addr >= rx_ringbuf_end_addr)
        {
            current_rx_ingest_addr = (uint32_t)&rx_ringbuf[0];
        }

        /* There's no active transaction, we can safely  */
        rx_ringbuf_extract_addr = current_rx_ingest_addr;
        tx_ringbuf_ingest_addr = current_tx_pos;

        /* Clear the slot that will be used for dummy Tx to avoid sending out duplicated responses
         *
         * IMPORTANT: not using memset() here to avoid branching instructions and maximize the processing speed.
         *            It is essential to ensure this handler completes ASAP
         */
        SPI_RINGBUF_FRAME_PARRETN_SET(current_tx_pos, 0x00000000u);
        dma_dummy_tx_pending = TRUE;

        /* Put the FIFO Override watermark */
        register uint32_t overrun_check_addr = current_rx_ingest_addr;
        SPI_RINGBUF_REWIND_POSITION(overrun_check_addr, sizeof(SPI_RINGBUF_OVERRUN_DIAG_WATERMARK), (uint32_t)&rx_ringbuf[0], rx_ringbuf_end_addr);
        uint32_t * overrun_watermark_ptr     = (uint32_t *)(void *)overrun_check_addr;
        *overrun_watermark_ptr               = SPI_RINGBUF_OVERRUN_DIAG_WATERMARK;
    } while (0);

    /* Re-enable NSS deactivation IRQ that notifies the app about freshly arrived data */
    SIDEWALK_RADIO_SPI_NSS_ENABLE_TRIGGER();

    UTILS_EXIT_CRITICAL_SECTION();

    return SERIAL_BUS_SPI_PAL_OK;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t serial_bus_spi_get_tx_fifo_free_level(void)
{
    register uint32_t fifo_free_space;

    /* Ensure there's no systematic SW failure */
    assert_param((tx_ringbuf_ingest_addr >= (uint32_t)&tx_ringbuf[0]) && (tx_ringbuf_ingest_addr <= tx_ringbuf_end_addr));

    /* Disable interrupts and place the data into the buffer */
    UTILS_ENTER_CRITICAL_SECTION();
    const register uint32_t current_tx_pos = SIDEWALK_RADIO_SPI.hdmatx->Instance->CMAR; /* Capture the start address of the current DMA Tx operation */
    const register uint32_t spi_dma_tx_en = READ_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, SPI_CR2_TXDMAEN); /* Capture if DMA Tx is currently scheduled */
    const register uint32_t tx_ringbuf_ingest_addr_copy = tx_ringbuf_ingest_addr;
    UTILS_EXIT_CRITICAL_SECTION();

    /* Ensure there's no systematic SW failure */
    assert_param((current_tx_pos >= (uint32_t)&tx_ringbuf[0]) && (current_tx_pos <= tx_ringbuf_end_addr));

    if (current_tx_pos == tx_ringbuf_ingest_addr_copy)
    {
        if ((spi_dma_tx_en != 0u) && (FALSE == dma_dummy_tx_pending))
        {
            /* Special case: ingest pointer is equal to the write out pointer, but since DMA Tx is active the FIFO is actually full,
             * unless we are actually doing a dummy tx now - in such case the buffer is actually totally free and DMA is sending out
             * dummy data to keep the SPI peripheral's internal state machine rolling
             */
            fifo_free_space = 0u;
        }
        else
        {
            /* FIFO is absolutely free */
            fifo_free_space = sizeof(tx_ringbuf);
        }
    }
    else if (current_tx_pos < tx_ringbuf_ingest_addr_copy)
    {
        /* Ingest pointer is ahead of the write out pointer, it's easier and faster to calculate the occupied space first */
        const register uint32_t occupied_space = (tx_ringbuf_ingest_addr_copy - current_tx_pos);
        fifo_free_space = sizeof(tx_ringbuf) - occupied_space;
    }
    else
    {
        /* Ingest pointer is behind the write out pointer, it's easier and faster to calculate the free space directly */
        fifo_free_space = current_tx_pos - tx_ringbuf_ingest_addr_copy;
    }

    /* Calculate the amouint of free slots in Tx FIFO. Keep in mind CRC is calculated by the hardware during transmission and is not placed into the Tx buffer */
    const uint32_t fifo_free_level = (fifo_free_space / SPI_FRAME_DATA_SIZE);
    return fifo_free_level;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t serial_bus_spi_get_rx_fifo_level(void)
{
    register uint32_t current_rx_ingest_addr;
    register uint32_t fifo_level;

    UTILS_ENTER_CRITICAL_SECTION();
    const register uint32_t current_rx_cndtr = SIDEWALK_RADIO_SPI.hdmarx->Instance->CNDTR; /* Capture the CNDTR register of the DMA Rx operation. Wecannot use CMAR register since it's not updated in circular mode */
    const register uint32_t rx_ringbuf_extract_addr_copy = rx_ringbuf_extract_addr;
    UTILS_EXIT_CRITICAL_SECTION();

    /* Perform FIFO overrun check */
    register uint32_t overrun_check_addr = rx_ringbuf_extract_addr_copy;
    SPI_RINGBUF_REWIND_POSITION(overrun_check_addr, sizeof(SPI_RINGBUF_OVERRUN_DIAG_WATERMARK), (uint32_t)&rx_ringbuf[0], rx_ringbuf_end_addr);
    const uint32_t * const overrun_watermark_ptr = (uint32_t *)(void *)overrun_check_addr;
    if (*overrun_watermark_ptr != SPI_RINGBUF_OVERRUN_DIAG_WATERMARK)
    {
        /* FIFO overrun detected */
        return SERIAL_BUS_SPI_PAL_FIFO_LEVEL_OVERRUN;
    }

    if (DMA_MDATAALIGN_HALFWORD == SIDEWALK_RADIO_SPI.hdmarx->Init.MemDataAlignment)
    {
        /* Calculate the nearest CNDTR for the fully completed SPI frames */
        const register uint32_t dma_transactions_per_spi_frame = SPI_FRAME_SIZE >> 1;
        const register uint32_t frame_aligned_cndtr = (current_rx_cndtr % dma_transactions_per_spi_frame) != 0u ?
        current_rx_cndtr - (dma_transactions_per_spi_frame - (current_rx_cndtr % dma_transactions_per_spi_frame)) : current_rx_cndtr;

        /* Calculate the write start pointer in the Rx buffer */
        current_rx_ingest_addr = rx_ringbuf_end_addr - (sizeof(uint16_t) * frame_aligned_cndtr);
    }
    else /* DMA_MDATAALIGN_BYTE */
    {
        /* The only other option for STM32WLxx is 8 bit for MemDataAlignment */
        /* Calculate the nearest CNDTR for the fully completed SPI frames */
        const register uint32_t dma_transactions_per_spi_frame = SPI_FRAME_SIZE;
        const register uint32_t frame_aligned_cndtr = (current_rx_cndtr % dma_transactions_per_spi_frame) != 0u ?
        current_rx_cndtr - (dma_transactions_per_spi_frame - (current_rx_cndtr % dma_transactions_per_spi_frame)) : current_rx_cndtr;

        /* Calculate the write start pointer in the Rx buffer */
        current_rx_ingest_addr = rx_ringbuf_end_addr - (sizeof(uint8_t) * frame_aligned_cndtr);
    }

    /* Handle a corner case when CNDTR was captured during DMA channel reload (CNDTR == 0)  */
    if (current_rx_ingest_addr >= rx_ringbuf_end_addr)
    {
        current_rx_ingest_addr = (uint32_t)&rx_ringbuf[0];
    }

    /* Now we can calculate the occupied space */
    if (rx_ringbuf_extract_addr_copy == current_rx_ingest_addr)
    {
        /* FIFO is empty */
        fifo_level = 0u;
    }
    else if (rx_ringbuf_extract_addr_copy < current_rx_ingest_addr)
    {
        /* Ingest pointer is ahead of the read out pointer, it's easier and faster to calculate the occupied space directly */
        fifo_level = current_rx_ingest_addr - rx_ringbuf_extract_addr_copy;
    }
    else
    {
        /* Ingest pointer is behind the read out pointer, it's easier and faster to calculate the free space first */
        const register uint32_t free_space = (rx_ringbuf_extract_addr_copy - current_rx_ingest_addr);
        fifo_level = sizeof(rx_ringbuf) - free_space;
    }

    /* Finally convert raw size in bytes into FIFO frames count */
    fifo_level = fifo_level / SPI_FRAME_SIZE;

    return fifo_level;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED serial_bus_spi_pal_err_t serial_bus_spi_pop_from_rx_fifo(uint8_t * const buf, const uint32_t buf_size, uint32_t * const bytes_copied)
{
    serial_bus_spi_pal_err_t err = SERIAL_BUS_SPI_PAL_ERROR;

    /* Inputs check */
    if ((NULL == buf) || (NULL == bytes_copied) || (buf_size < SPI_FRAME_DATA_SIZE))
    {
        return SERIAL_BUS_SPI_PAL_INVALID_ARGS;
    }

    /* Calculate how much complete SPI frames can be fit into the supplied buffer */
    const register uint32_t max_fit_frames = buf_size / SPI_FRAME_DATA_SIZE;

    /* Initialize the output parameter */
    *bytes_copied = 0u;

    /* Disable interrupts while touching the buffer */
    UTILS_ENTER_CRITICAL_SECTION();

    /* Check how many frames we have and how many of them we can copy */
    const register uint32_t available_frames = serial_bus_spi_get_rx_fifo_level();
    if (SERIAL_BUS_SPI_PAL_FIFO_LEVEL_OVERRUN == available_frames)
    {
        /* Rx buffer overrun happened so we don't know how much data is valid in there - nothing can be copied */
        UTILS_EXIT_CRITICAL_SECTION();
        return SERIAL_BUS_SPI_PAL_FIFO_OVERRUN;
    }
    const register uint32_t frames_to_copy = max_fit_frames >= available_frames ? available_frames : max_fit_frames;

    /* Store a local copy for further processing */
    register uint32_t copy_start_addr = rx_ringbuf_extract_addr;

    /* Advance the extract address for the next call - add entire copy size */
    SPI_RINGBUF_ADVANCE_POSITION(rx_ringbuf_extract_addr, (frames_to_copy * SPI_FRAME_SIZE), (uint32_t)&rx_ringbuf[0], rx_ringbuf_end_addr);

    /* Re-enable the interrupts since we are done with the critical stuff */
    UTILS_EXIT_CRITICAL_SECTION();

    /* Calculate the address of the FIFO Override watermark */
    register uint32_t overrun_check_addr = copy_start_addr;
    SPI_RINGBUF_REWIND_POSITION(overrun_check_addr, sizeof(SPI_RINGBUF_OVERRUN_DIAG_WATERMARK), (uint32_t)&rx_ringbuf[0], rx_ringbuf_end_addr);

    /* Copy data - it's safe to do it outside of the critical section, the pointers are moved already, so nobody will be able to read them out in parallel */
    uint8_t * buf_write_ptr = buf;
    err = SERIAL_BUS_SPI_PAL_OK;
    for (uint32_t i = 0u; i < frames_to_copy; i++)
    {
        /* Copy the payload only, without CRC bytes */
        SID_STM32_UTIL_fast_memcpy(buf_write_ptr, (void *)copy_start_addr, SPI_FRAME_DATA_SIZE);
        __COMPILER_BARRIER();

        /* FIFO override check shall take place here for each individual frame since if we are popping out multiple frames,
         * we actually don't care about an overwrite of the already-copied ones.
         *
         * Also, it's important to check for override AFTER the copying because DMA may overwrite the frame while we are copying
         */
        uint32_t * overrun_watermark_ptr = (uint32_t *)(void *)overrun_check_addr;
        if (*overrun_watermark_ptr != SPI_RINGBUF_OVERRUN_DIAG_WATERMARK)
        {
            err = SERIAL_BUS_SPI_PAL_FIFO_OVERRUN;
            break;
        }
        __COMPILER_BARRIER();

        /* Adjust pointers */
        buf_write_ptr += SPI_FRAME_DATA_SIZE; /* Advance the pointer by the payload size */
        SPI_RINGBUF_ADVANCE_POSITION(copy_start_addr, SPI_FRAME_SIZE, (uint32_t)&rx_ringbuf[0], rx_ringbuf_end_addr); /* Advance the source pointer by full SPI frame size, including CRC bytes */
        SPI_RINGBUF_ADVANCE_POSITION(overrun_check_addr, SPI_FRAME_SIZE, (uint32_t)&rx_ringbuf[0], rx_ringbuf_end_addr); /* Advance the overrun diag pointer by full SPI frame size, including CRC bytes */

        /* Note the amount of extracted data */
        *bytes_copied += SPI_FRAME_DATA_SIZE;

        /* Put FIFO overrun diagnostic watermark at the location of the just-extracted frame */
        overrun_watermark_ptr = (uint32_t *)(void *)overrun_check_addr;
        *overrun_watermark_ptr = SPI_RINGBUF_OVERRUN_DIAG_WATERMARK;
    }

    /* Everything is ok if we got here */
    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED serial_bus_spi_pal_err_t serial_bus_spi_enqueue_tx(const uint8_t * const data, const uint32_t length)
{
    serial_bus_spi_pal_err_t ret = SERIAL_BUS_SPI_PAL_OK;
    const register uint32_t padding_length = (length % SPI_FRAME_DATA_SIZE) > 0u ? (SPI_FRAME_DATA_SIZE - (length % SPI_FRAME_DATA_SIZE)) : 0u;
    const register uint32_t aligned_length = length + padding_length;

    /* Validate inputs first */
    if ((NULL == data) || (0u == length) || (aligned_length > sizeof(tx_ringbuf)))
    {
        return SERIAL_BUS_SPI_PAL_INVALID_ARGS;
    }

    /* Ensure there's no systematic SW failure */
    assert_param((tx_ringbuf_ingest_addr >= (uint32_t)&tx_ringbuf[0]) && (tx_ringbuf_ingest_addr <= tx_ringbuf_end_addr));

    /* Disable interrupts and place the data into the buffer */
    UTILS_ENTER_CRITICAL_SECTION();
    do
    {
        register uint32_t fifo_free_space;
        const register uint32_t current_tx_pos = SIDEWALK_RADIO_SPI.hdmatx->Instance->CMAR; /* Capture the start address of the current DMA Tx operation */
        const register uint32_t spi_dma_tx_en = READ_BIT(SIDEWALK_RADIO_SPI.Instance->CR2, SPI_CR2_TXDMAEN); /* Capture if DMA Tx is currently scheduled */

        /* Ensure there's no systematic SW failure */
        assert_param((current_tx_pos >= (uint32_t)&tx_ringbuf[0]) && (current_tx_pos <= tx_ringbuf_end_addr));

        if (current_tx_pos == tx_ringbuf_ingest_addr)
        {
            if ((spi_dma_tx_en != 0u) && (FALSE == dma_dummy_tx_pending))
            {
                /* Special case: ingest pointer is equal to the write out pointer, but since DMA Tx is active the FIFO is actually full,
                 * unless we are actually doing a dummy tx now - in such case the buffer is actually totally free and DMA is sending out
                 * dummy data to keep the SPI peripheral's internal state machine rolling
                 */
                fifo_free_space = 0u;
            }
            else
            {
                /* FIFO is absolutely free */
                fifo_free_space = sizeof(tx_ringbuf);
            }
        }
        else if (current_tx_pos < tx_ringbuf_ingest_addr)
        {
            /* Ingest pointer is ahead of the write out pointer, it's easier and faster to calculate the occupied space first */
            const register uint32_t occupied_space = (tx_ringbuf_ingest_addr - current_tx_pos);
            fifo_free_space = sizeof(tx_ringbuf) - occupied_space;
        }
        else
        {
            /* Ingest pointer is behind the write out pointer, it's easier and faster to calculate the free space directly */
            fifo_free_space = current_tx_pos - tx_ringbuf_ingest_addr;
        }

        /* Now check if we have enough space in Tx FIFO */
        if (aligned_length > fifo_free_space)
        {
            ret = SERIAL_BUS_SPI_PAL_NO_FREE_SPACE;
            break;
        }

        register uint32_t direct_copy_length;
        register uint32_t wrapped_copy_length;
        if ((tx_ringbuf_ingest_addr + length) <= tx_ringbuf_end_addr)
        {
            /* All the pending data fits linearly into the reminder of the Tx ring buffer */
            direct_copy_length = length;
            wrapped_copy_length = 0u;
        }
        else
        {
            direct_copy_length = tx_ringbuf_end_addr - tx_ringbuf_ingest_addr;
            wrapped_copy_length = length - direct_copy_length;
        }

        if (direct_copy_length > 0u)
        {
            /* Fill up the space till the end of the buffer region in RAM */
            SID_STM32_UTIL_fast_memcpy((void *)tx_ringbuf_ingest_addr, data, direct_copy_length);

            /* Advance the ingestion pointer and wrap it if necessary */
            tx_ringbuf_ingest_addr += direct_copy_length;
            if (tx_ringbuf_ingest_addr == tx_ringbuf_end_addr)
            {
                tx_ringbuf_ingest_addr = (uint32_t)&tx_ringbuf[0];
            }
            else if (tx_ringbuf_ingest_addr > tx_ringbuf_end_addr)
            {
                /* This should never happen unless there's a systematic failure in the code above */
                ret = SERIAL_BUS_SPI_PAL_ALIGNMENT_ERROR;
                break;
            }
            else
            {
                /* No need to do anything */
            }
        }

        /* Copy the wrapped part (if any) to the start of the buffer */
        if (wrapped_copy_length > 0u)
        {
            SID_STM32_UTIL_fast_memcpy((void *)tx_ringbuf_ingest_addr, (void *)((uint32_t)&data[0] + direct_copy_length), wrapped_copy_length);
            tx_ringbuf_ingest_addr += wrapped_copy_length;
        }

        /* Add padding to complete the frame */
        register uint32_t direct_padding_length;
        register uint32_t wrapped_padding_length;
        if ((tx_ringbuf_ingest_addr + padding_length) <= tx_ringbuf_end_addr)
        {
            /* All the pending data fits linearly into the reminder of the Tx ring buffer */
            direct_padding_length = padding_length;
            wrapped_padding_length = 0u;
        }
        else
        {
            direct_padding_length = tx_ringbuf_end_addr - tx_ringbuf_ingest_addr;
            wrapped_padding_length = padding_length - direct_padding_length;
        }

        /* Fill up the space till the end of the buffer region in RAM */
        if (direct_padding_length > 0u)
        {
            SID_STM32_UTIL_fast_memset((void *)tx_ringbuf_ingest_addr, 0x00u, direct_padding_length);
        }

        /* Advance the ingestion pointer and wrap it if necessary */
        tx_ringbuf_ingest_addr += direct_padding_length;
        if (tx_ringbuf_ingest_addr == tx_ringbuf_end_addr)
        {
            tx_ringbuf_ingest_addr = (uint32_t)&tx_ringbuf[0];
        }
        else if (tx_ringbuf_ingest_addr > tx_ringbuf_end_addr)
        {
            /* This should never happen unless there's a systematic failure in the code above */
            ret = SERIAL_BUS_SPI_PAL_ALIGNMENT_ERROR;
            break;
        }
        else
        {
            /* No need to do anything */
        }

        /* Fill the wrapped part (if any) */
        if (wrapped_padding_length > 0u)
        {
            SID_STM32_UTIL_fast_memset((void *)tx_ringbuf_ingest_addr, 0x00u, wrapped_padding_length);
            tx_ringbuf_ingest_addr += wrapped_padding_length;
        }

        /* Ensure all the stuff is done before we proceed with arming the DMA */
        __COMPILER_BARRIER();

        /* If no active SPI communication was running on entry we should cancel dma_dummy_tx_pending flag, otherwise keep it as is,
         * and DMA Tx Completed interrupt will take care of it
         */
        if (spi_dma_tx_en == 0u)
        {
            dma_dummy_tx_pending = FALSE;
        }
    } while (0);
    UTILS_EXIT_CRITICAL_SECTION();

    return ret;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void * serial_bus_spi_get_current_enqueue_ptr(void)
{
    void * ptr;

    UTILS_ENTER_CRITICAL_SECTION();
    ptr = (void *)tx_ringbuf_ingest_addr;
    UTILS_EXIT_CRITICAL_SECTION();

    return ptr;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void serial_bus_spi_on_frame_received(void)
{
    if (spi_rx_user_callback != NULL)
    {
        spi_rx_user_callback();
    }
}

/*----------------------------------------------------------------------------*/

/**
 * @brief Customized SPI IRQ callback to speed up SPI IRQ processing
 *        Since this module uses DMA for SPI xfers only the error IRQs are generated
 */
#if   (defined (SPI1)) && ((SPI1_BASE) == (SIDEWALK_RADIO_SPI_INSTANCE_BASE))
SID_STM32_SPEED_OPTIMIZED void SPI1_IRQHandler(void)
#elif (defined (SPI2)) && ((SPI2_BASE) == (SIDEWALK_RADIO_SPI_INSTANCE_BASE))
SID_STM32_SPEED_OPTIMIZED void SPI2_IRQHandler(void)
#else
#  error "Unknown SPI instance specified in SIDEWALK_RADIO_SPI_INSTANCE"
#endif
{
    register const uint32_t spi_cr2_copy = SIDEWALK_RADIO_SPI.Instance->CR2;
    register const uint32_t spi_sr_copy  = SIDEWALK_RADIO_SPI.Instance->SR;

    /* SPI in Error Treatment --------------------------------------------------*/
    if ((spi_cr2_copy & SPI_IT_ERR) != 0u)
    {
        /* SPI Overrun error interrupt occurred ----------------------------------*/
        if ((spi_sr_copy & SPI_FLAG_OVR) != 0u)
        {
            SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_OVR);
            __HAL_SPI_CLEAR_OVRFLAG(&SIDEWALK_RADIO_SPI);
        }

        /* SPI Mode Fault error interrupt occurred -------------------------------*/
        if ((spi_sr_copy & SPI_FLAG_MODF) != 0u)
        {
            SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_MODF);
            __HAL_SPI_CLEAR_MODFFLAG(&SIDEWALK_RADIO_SPI);
        }

        /* SPI Frame error interrupt occurred ------------------------------------*/
        if ((spi_sr_copy & SPI_FLAG_FRE) != 0u)
        {
            SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_FRE);
            __HAL_SPI_CLEAR_FREFLAG(&SIDEWALK_RADIO_SPI);
        }

#if (USE_SPI_CRC != 0U)
        /* SPI CRC error interrupt occurred --------------------------------------*/
        if ((spi_sr_copy & SPI_FLAG_CRCERR) != 0u)
        {
            SET_BIT(SIDEWALK_RADIO_SPI.ErrorCode, HAL_SPI_ERROR_CRC);
            __HAL_SPI_CLEAR_CRCERRFLAG(&SIDEWALK_RADIO_SPI);
        }
#endif /* USE_SPI_CRC */

        /* Trigger SPI Abort if any critical error was detected */
        if (SIDEWALK_RADIO_SPI.ErrorCode != HAL_SPI_ERROR_NONE)
        {
            _hal_like_spi_error_callback();
        }
    }
}
