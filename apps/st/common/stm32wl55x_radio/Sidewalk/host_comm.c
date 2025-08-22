/**
  ******************************************************************************
  * @file    host_comm.c
  * @brief   Sidewalk STM32WLxx Radio App's host MCU communication module
  * 
  * This module handles communication with the host MCU as well as handles the
  * SubGHz radio
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

/* App-specific */
#include "main.h"
#include "sys_app.h"
#include "utilities_conf.h"

/* Sidewalk Radio related */
#include "comm_def.h"
#include "comm_opcodes.h"
#include "host_comm.h"
#include "radio_driver.h"
#include "serial_bus_spi_pal.h"
#include "sid_radio.h"

/* Platform headers */
#include <cmsis_os2.h>
#include <stm32wlxx.h>

/* STM32 Utils */
#include "sid_pal_log_like.h"
#include <sid_stm32_common_utils.h>
#include <stm32_timer.h>

/* LPM */
#include "sys_conf.h"
#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
#  include "utilities_def.h"
#  include <stm32_lpm.h>
#endif /* LOW_POWER_DISABLE */


/* Private typedef -----------------------------------------------------------*/

/**
 * @brief States of the Host Comm Rx (HC_RX) State Machine
 */
typedef enum {
    HC_RX_STATE_IDLE         = 0, /*!< Idle state, ready to receive new frames */
    HC_RX_STATE_LONG_DATA_RX = 1, /*!< Log Data Transfer mode is active, awaiting one or more LDTC frames */
    HC_RX_STATE_MSG_RECEIVED = 2, /*!< A complete app-level protocol frame received (either if the app-level frame fits into a single SPI frame or Long Data Transfer mode has completed) */
    HC_RX_STATE_ERROR        = 3, /*!< Error occurred while processing SPI Rx input. This is a no-return state and no further processing is possible until the Host Comm Rx state machine is reset */
} host_comm_rx_state_t;

/**
 * @brief Error codes reported by the Host Comm Rx state machine
 */
typedef enum {
    HC_RX_ERROR_NONE                     =  0,  /*!< No errors occurred during the processing */
    HC_RX_ERROR_NO_DATA_AVAILABLE        =  1,  /*!< Not an error, just indication that SPI Rx FIFO contains no data */
    HC_RX_ERROR_LONG_DATA_PENDING        =  2,  /*!< Not an error, just indication that more SPI frames shall be receive to complete the app protocol frame */
    HC_RX_ERROR_PAST_ERRORS_ACTIVE       =  3,  /*!< There were errors during previous processing iteration and these errors are still active (not processed) */
    HC_RX_ERROR_GENERIC                  =  4,  /*!< Generic error, typically means systematic SW failure */
    HC_RX_ERROR_INVALID_STATE            =  5,  /*!< Host Comm Rx state machine is in invalid (unknown) state */
    HC_RX_ERROR_INSUFFICIENT_STORE_SPACE =  6,  /*!< Rx buffer is too small to accommodate the upcoming app protocol frame */
    HC_RX_ERROR_UNEXPECTED_POP_SIZE      =  7,  /*!< SPI driver popped different amount of bytes from its Rx FIFO than requested */
    HC_RX_ERROR_FIFO_OVERRUN             =  8,  /*!< SPI driver reported Rx FIFO overrun, data in the FIFO is corrupted */
    HC_RX_ERROR_SPI_DRIVER               =  9,  /*!< SPI driver reported some hardware-related error */
    HC_RX_ERROR_STRAY_LDTC               =  10, /*!< Suddenly received a Long Data Transfer Continuation (LDTC) SPI frame when no Long Data Transfer was active */
    HC_RX_ERROR_NON_LDTC_DATA_IN_LDTC    =  11, /*!< Suddenly received a non-LDTC SPI frame when Long Data Transfer mode was active */
} host_comm_rx_error_t;

/* Private defines -----------------------------------------------------------*/

/* Host Comm task activation flags */
#define COMM_PROC_NO_ACTIVE_FLAGS                 ((uint32_t)(0u))
#define COMM_PROC_DATA_AVAILABLE_FLAG             ((uint32_t)(1u << 0))
#define COMM_PROC_SPI_ERROR_FLAG                  ((uint32_t)(1u << 1))
#define COMM_PROC_RADIO_INTERRUPT_FLAG            ((uint32_t)(1u << 2))
#define COMM_PROC_HANDSHAKE_EXTERNAL_REQUEST_FLAG ((uint32_t)(1u << 3))
#define COMM_PROC_HANDSHAKE_INTERNAL_REQUEST_FLAG ((uint32_t)(1u << 4))

/* Buffer sizes */
#define SPI_RX_BUF_SIZE                           (STM32WLxx_RADIO_COMM_MTU_SIZE)                 /*!< Buffer to accommodate reassembled Rx data */
#define SPI_RX_PROCESSING_BUF_SIZE                (8u * sizeof(stm32wlxx_radio_comm_spi_frame_t)) /*!< Temporary working buffer to process split SPI frames */
#define SPI_TX_PROCESSING_BUF_SIZE                (8u * sizeof(stm32wlxx_radio_comm_spi_frame_t)) /*!< Temporary working buffer to prepare split SPI frames for Tx */

/* Timeouts and retry limits */
#define HOST_COMM_SPI_XFER_START_ATTEMPTS         (5u)
#define HOST_COMM_SPI_XFER_START_COOLDOWN_MS      (10u)

#define HOST_COMM_SPI_FIFO_CLEAN_TIMEOUT_MS       (10u)

#define HOST_COMM_IRQ_PIN_STATE_PROBE_COUNT       (5u) /*!< IRQ line will be inspected this amount of times before Handshake IRQ will be reported */
#define HOST_COMM_IRQ_PIN_STATE_PROBE_PERIOD_US   (2u) /*!< Delay between consecutive IRQ line state samples */
#define HOST_COMM_IRQ_PIN_STATE_PROBE_DEBOUNCE    (3u) /*!< Pin must remain in the desired state for this number of consecutive reads */
#if HOST_COMM_IRQ_PIN_STATE_PROBE_DEBOUNCE > HOST_COMM_IRQ_PIN_STATE_PROBE_COUNT
#  error "HOST_COMM_IRQ_PIN_STATE_PROBE_DEBOUNCE cannot exceed HOST_COMM_IRQ_PIN_STATE_PROBE_COUNT since debounce counter physically cannot exceed the number of probe iterations"
#endif

#ifndef STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
#  error "STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER is not defined"
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/* Private macro -------------------------------------------------------------*/

#ifndef HOST_COMM_EXTRA_LOGGING
/* Set HOST_COMM_RADIO_EXTRA_LOGGING to 1 to enable extended logs */
#  define HOST_COMM_EXTRA_LOGGING         (0)
#endif /* HOST_COMM_EXTRA_LOGGING */

#ifndef HOST_COMM_HC_RX_SM_DEBUG
/* Set HOST_COMM_HC_RX_SM_DEBUG to 1 to enable the debug of Host Comm Rx State Machine */
#  define HOST_COMM_HC_RX_SM_DEBUG        (0)
#endif /* HOST_COMM_HC_RX_SM_DEBUG */

#ifndef HOST_COMM_SPI_TX_DEBUG
/* Set HOST_COMM_HC_RX_SM_DEBUG to 1 to enable the debug of Host Comm SPI Tx handling */
#  define HOST_COMM_SPI_TX_DEBUG          (0)
#endif /* HOST_COMM_SPI_TX_DEBUG */

#if defined(HOST_COMM_EXTRA_LOGGING) && HOST_COMM_EXTRA_LOGGING
#  define HOST_COMM_LOG_ERROR(...)        SID_PAL_LOG_ERROR(__VA_ARGS__)
#  define HOST_COMM_LOG_WARNING(...)      SID_PAL_LOG_WARNING(__VA_ARGS__)
#  define HOST_COMM_LOG_INFO(...)         SID_PAL_LOG_INFO(__VA_ARGS__)
#  define HOST_COMM_LOG_DEBUG(...)        SID_PAL_LOG_DEBUG(__VA_ARGS__)
#  define HOST_COMM_LOG_TRACE(...)        SID_PAL_LOG_TRACE(__VA_ARGS__)
#else
#  define HOST_COMM_LOG_ERROR(...)        ((void)0u)
#  define HOST_COMM_LOG_WARNING(...)      ((void)0u)
#  define HOST_COMM_LOG_INFO(...)         ((void)0u)
#  define HOST_COMM_LOG_DEBUG(...)        ((void)0u)
#  define HOST_COMM_LOG_TRACE(...)        ((void)0u)
#endif

#define HC_RX_STATE_MACHINE_RESET()       do\
                                          {\
                                              hc_rx_state = HC_RX_STATE_IDLE;\
                                              hc_rx_expected_data_size = 0u;\
                                              hc_rx_received_data_size = 0u;\
                                          } while (0)

#define IS_GPIO_CLK_ENABLED(__GPIOx__)    (GPIOA == (__GPIOx__) ? __HAL_RCC_GPIOA_IS_CLK_ENABLED() : \
                                           GPIOB == (__GPIOx__) ? __HAL_RCC_GPIOB_IS_CLK_ENABLED() : \
                                           GPIOC == (__GPIOx__) ? __HAL_RCC_GPIOC_IS_CLK_ENABLED() : \
                                           GPIOH == (__GPIOx__) ? __HAL_RCC_GPIOH_IS_CLK_ENABLED() : \
                                           0u)

#define HC_HANDSHAKE_PRE_ENTRY_ACTION()   do\
                                          {\
                                              HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn); /* Ensure SubGHz will not interfere with the Handshake */\
                                              SID_HOST_COMM_IRQ_DISABLE_TRIGGER();    /* Disable EXTI on IRQ falling edge since we are going to drive the line, otherwise GPIO Handshake will be triggered */\
                                              __COMPILER_BARRIER();                   /* Ensure EXTI trigger is deactivated before the next step */\
                                          } while (0)

/* External variables --------------------------------------------------------*/

extern SPI_HandleTypeDef SIDEWALK_RADIO_SPI;
extern UTIL_TIMER_Object_t radio_timeout_mon;

/* Private variables ---------------------------------------------------------*/

/* Host comm SPI buffers */
static uint8_t              host_comm_rx_buf[SPI_RX_BUF_SIZE];                 /*!< Rx buffer that receives the final Rx data (e.g. re-assembled long data or a full single frame for normal transfers) */
static uint8_t              spi_rx_processing_buf[SPI_RX_PROCESSING_BUF_SIZE]; /*!< Rx processing working buffer - used to temporarily store received frames (partial data), e.g. for re-assembling long data */
static uint8_t              spi_tx_processing_buf[SPI_TX_PROCESSING_BUF_SIZE]; /*!< Tx processing buffer - used as temporary storage when preparing Tx data (e.g. to wrap the data into Long Data Transfer frames) */
static host_comm_rx_state_t hc_rx_state;                                       /*!< Host Comm Rx State Machine state */
static uint32_t             hc_rx_expected_data_size;                          /*!< Expected amount of bytes for current transaction (either for a single-frame or Long Data Transfer sequence) */
static uint32_t             hc_rx_received_data_size;                          /*!< Actually received amount of data for the current transaction */

/* SPI error handling */
static serial_bus_spi_pal_bus_error_details_t host_comm_spi_last_error_info;   /*!< Storage for the last known SPI driver error - used to quickly store the error info in interrupt nd later process it within the task context */

static osThreadId_t         host_comm_task_handle = NULL;
const osThreadAttr_t        host_comm_task_attributes = {
                                .name       = "SID HostComm",
                                .priority   = (osPriority_t) osPriorityISR,
                                .stack_size = (512u + ((12u * UTIL_ADV_TRACE_TMP_BUF_SIZE) / 10u)), /* Always reserve the task stack space for the logs at 120% of the maximum log line size */
                            };

static uint32_t             subghz_irq_timestamp_s;
static uint32_t             subghz_irq_timestamp_us;

/* Private function prototypes -----------------------------------------------*/

static host_comm_rx_error_t         _host_comm_spi_rx_process(void); /*!< Host Comm Rx State Machine implementation */
static inline host_comm_rx_error_t  _hc_rx_process_idle_state(uint32_t * const processing_finished);
static inline host_comm_rx_error_t  _hc_rx_process_long_data_rx_state(uint32_t * const processing_finished);

static inline sid_host_comm_error_t _host_comm_gpio_init(void);
static        uint32_t              _enter_handshake_state(const uint32_t full_spi_reset);
static inline void                  _internal_handshake_request(void);
static        void                  _host_comm_task(void * argument);
static        void                  _on_host_comm_spi_error(const serial_bus_spi_pal_bus_error_details_t * const error_info);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static host_comm_rx_error_t _host_comm_spi_rx_process(void)
{
    host_comm_rx_error_t err = HC_RX_ERROR_GENERIC;
    uint32_t processing_finished = FALSE;
    uint32_t fifo_level;

    /* Reset the SM state if the previous frame was received successfully */
    if (HC_RX_STATE_MSG_RECEIVED == hc_rx_state)
    {
        hc_rx_state = HC_RX_STATE_IDLE;
    }

    /* Extract SPI frames from the FIFO until either we receive a complete Radio App protocol frame or FIFO is empty or an error is encountered */
    do
    {
        /* First of all check if we have anything to process */
        fifo_level = serial_bus_spi_get_rx_fifo_level();
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
        HOST_COMM_LOG_DEBUG("HC RX SM on entry - state: %u, err: %u, fifo lvl: %u", (uint32_t)hc_rx_state, (uint32_t)err, fifo_level);
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
        if (0u == fifo_level)
        {
            err = HC_RX_ERROR_NO_DATA_AVAILABLE;
            processing_finished = TRUE;
            break;
        }

        /* State Machine */
        switch (hc_rx_state)
        {
            case HC_RX_STATE_IDLE:
                err = _hc_rx_process_idle_state(&processing_finished);
                break;

            case HC_RX_STATE_LONG_DATA_RX:
                err = _hc_rx_process_long_data_rx_state(&processing_finished);
                break;

            case HC_RX_STATE_MSG_RECEIVED:
                /* Normally we should not get here since this state is for indication purposes only, state machine is not expected to be run */
                SID_PAL_LOG_ERROR("Running host comm Rx SM in state HC_RX_STATE_MSG_RECEIVED");
                err = HC_RX_ERROR_INVALID_STATE;
                processing_finished = TRUE;
                break;

            case HC_RX_STATE_ERROR:
                /* Do nothing since we cannot proceed until the error(s) are processed */
                err = HC_RX_ERROR_PAST_ERRORS_ACTIVE;
                processing_finished = TRUE;
                break;

            default:
                /* Normally we should not get here ever */
                SID_PAL_LOG_ERROR("Host comm Rx SM is in unknown state: %u", (uint32_t)hc_rx_state);
                err = HC_RX_ERROR_INVALID_STATE;
                processing_finished = TRUE;
                break;
        }

#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
        fifo_level = serial_bus_spi_get_rx_fifo_level();
        HOST_COMM_LOG_DEBUG("HC RX SM on exit - state: %u, err: %u, fifo lvl: %u", (uint32_t)hc_rx_state, (uint32_t)err, fifo_level);
#else
        if (processing_finished != TRUE)
        {
            /* Refresh the FIFO level if we may proceed since some new SPI data may has arrived while we were processing the already-available SPI frames */
            fifo_level = serial_bus_spi_get_rx_fifo_level();
        }
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
    } while ((FALSE == processing_finished) && ((fifo_level > 0u) && (fifo_level != SERIAL_BUS_SPI_PAL_FIFO_LEVEL_OVERRUN)));

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline host_comm_rx_error_t _hc_rx_process_idle_state(uint32_t * const processing_finished)
{
    host_comm_rx_error_t err;
    serial_bus_spi_pal_err_t spi_pal_err;
    uint32_t bytes_popped;

    assert_param(processing_finished != NULL);

    /* Extract a single frame */
    spi_pal_err = serial_bus_spi_pop_from_rx_fifo(spi_rx_processing_buf, sizeof(stm32wlxx_radio_comm_spi_frame_t), &bytes_popped);
    if (SERIAL_BUS_SPI_PAL_OK == spi_pal_err)
    {
        if (sizeof(stm32wlxx_radio_comm_spi_frame_t) == bytes_popped)
        {
            /* Everything is ok, we can proceed */
            const stm32wlxx_radio_comm_spi_frame_t * const packet = (stm32wlxx_radio_comm_spi_frame_t *)(void *)spi_rx_processing_buf;

            /* If this frame indicates a start of a long data transaction then arrange it and proceed with reassembling the long data */
            if (STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_START == packet->opcode)
            {
                /* Ensure that what is pending can actually fit into our buffer */
                if (packet->payload.ldts.full_data_size > sizeof(host_comm_rx_buf))
                {
                    /* Too much of a data, can't store it */
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
                    HOST_COMM_LOG_ERROR("LDTS rejected. Expected data size exceeds the local Rx buf size (%u vs %u)", packet->payload.ldts.full_data_size, sizeof(host_comm_rx_buf));
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
                    hc_rx_state = HC_RX_STATE_ERROR;
                    err = HC_RX_ERROR_INSUFFICIENT_STORE_SPACE;
                    *processing_finished = TRUE;
                }
                else
                {
                    /* That's fine, let's store the partial data we have and proceed with reassembling the entire long data */
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
                    HOST_COMM_LOG_DEBUG("Valid LDTS arrived, expected data size: %u",  packet->payload.ldts.full_data_size);
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
                    SID_STM32_UTIL_fast_memcpy(host_comm_rx_buf, packet->payload.ldts.partial_data, sizeof(packet->payload.ldts.partial_data));
                    hc_rx_expected_data_size = packet->payload.ldts.full_data_size;
                    hc_rx_received_data_size = sizeof(packet->payload.ldts.partial_data);
                    hc_rx_state = HC_RX_STATE_LONG_DATA_RX; /* Transition to the LDTC state */
                    err = HC_RX_ERROR_LONG_DATA_PENDING; /* Indicate that we are expecting more bytes to come */
                }
            }
            else if (STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_CONT == packet->opcode)
            {
                /* Hmm, we were not expecting to receive a stray long data continuation, something is not ok */
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
                HOST_COMM_LOG_ERROR("Received LDTC frame while no active LDT running");
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
                hc_rx_state = HC_RX_STATE_ERROR;
                err = HC_RX_ERROR_STRAY_LDTC;
                *processing_finished = TRUE;
            }
            else
            {
                /* This is not a Long Data Transfer frame, but a regular one - extract the payload and we are done */
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
                HOST_COMM_LOG_DEBUG("Successfully received a single-frame host comm protocol message");
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
                SID_STM32_UTIL_fast_memcpy(host_comm_rx_buf, (void *)packet, sizeof(*packet));
                hc_rx_expected_data_size = sizeof(*packet);
                hc_rx_received_data_size = sizeof(*packet);
                hc_rx_state = HC_RX_STATE_MSG_RECEIVED;
                err = HC_RX_ERROR_NONE;
                *processing_finished = TRUE;
            }
        }
        else
        {
            /* Something is seriously wrong as FIFO should always return a fixed-size frame */
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
            HOST_COMM_LOG_ERROR("Host comm Rx FIFO popped unexpected amount of data: %u instead of %u bytes", bytes_popped, sizeof(stm32wlxx_radio_comm_spi_frame_t));
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
            hc_rx_state = HC_RX_STATE_ERROR;
            err = HC_RX_ERROR_UNEXPECTED_POP_SIZE;
            *processing_finished = TRUE;
        }
    }
    else if (SERIAL_BUS_SPI_PAL_FIFO_OVERRUN == spi_pal_err)
    {
        /* FIFO overrun was detected, data may be corrupted */
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
        HOST_COMM_LOG_ERROR("Host comm Rx FIFO overrun detected");
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
        hc_rx_state = HC_RX_STATE_ERROR;
        err = HC_RX_ERROR_FIFO_OVERRUN;
        *processing_finished = TRUE;
    }
    else
    {
        /* Some other SPI driver error */
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
        HOST_COMM_LOG_ERROR("Failed to pop a frame from host comm Rx FIFO. SPI driver error %u", (uint32_t)spi_pal_err);
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
        hc_rx_state = HC_RX_STATE_ERROR;
        err = HC_RX_ERROR_SPI_DRIVER;
        *processing_finished = TRUE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline host_comm_rx_error_t _hc_rx_process_long_data_rx_state(uint32_t * const processing_finished)
{
    host_comm_rx_error_t err;
    serial_bus_spi_pal_err_t spi_pal_err;
    uint32_t bytes_popped;

    assert_param(processing_finished != NULL);

    /* See how many frames we can pop from the Rx FIFO at one */
    uint32_t fifo_level = serial_bus_spi_get_rx_fifo_level();
    const uint32_t remaining_bytes = hc_rx_expected_data_size - hc_rx_received_data_size;
    const uint32_t remaining_spi_frames = (remaining_bytes
                                           + (SID_STM32_UTIL_STRUCT_MEMBER_SIZE(stm32wlxx_rcp_ldtc_t, partial_data) - 1u)
                                          ) / SID_STM32_UTIL_STRUCT_MEMBER_SIZE(stm32wlxx_rcp_ldtc_t, partial_data);

    uint32_t frames_to_pop = fifo_level > remaining_spi_frames ? remaining_spi_frames : fifo_level;
    if (frames_to_pop > (sizeof(spi_rx_processing_buf) / sizeof(stm32wlxx_radio_comm_spi_frame_t)))
    {
        /* We have more frames in the FIFO than can fit into a working buffer. Pop only the amount we can process */
        frames_to_pop = (sizeof(spi_rx_processing_buf) / sizeof(stm32wlxx_radio_comm_spi_frame_t));
    }
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
        HOST_COMM_LOG_DEBUG("LDTC stats: expected bytes: %u, remaining bytes: %u, remaining frames: %u, fifo lvl: %u, frames to pop: %u", hc_rx_expected_data_size, remaining_bytes, remaining_spi_frames, fifo_level, frames_to_pop);
#endif /* HOST_COMM_HC_RX_SM_DEBUG */

    /* Extract multiple frames at once (if available) */
    spi_pal_err = serial_bus_spi_pop_from_rx_fifo(spi_rx_processing_buf, (frames_to_pop * sizeof(stm32wlxx_radio_comm_spi_frame_t)), &bytes_popped);
    if (SERIAL_BUS_SPI_PAL_OK == spi_pal_err)
    {
        if ((frames_to_pop * sizeof(stm32wlxx_radio_comm_spi_frame_t)) == bytes_popped)
        {
            /* If we got the expected amount of frames, process them one by one */
            for (uint32_t i = 0u; i < frames_to_pop; i++)
            {
                const stm32wlxx_radio_comm_spi_frame_t * const packet = (stm32wlxx_radio_comm_spi_frame_t *)(void *)&spi_rx_processing_buf[i * sizeof(stm32wlxx_radio_comm_spi_frame_t)];

                if (STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_CONT == packet->opcode)
                {
                    /* Ensure we are going to fit into the final Rx buffer - padding (if any) may overflow it and shall not be copied */
                    const uint32_t copy_length = sizeof(packet->payload.ldtc.partial_data) <= (hc_rx_expected_data_size - hc_rx_received_data_size) ?
                                                    sizeof(packet->payload.ldtc.partial_data) : (hc_rx_expected_data_size - hc_rx_received_data_size);

                    /* Store the partial data we have and proceed with reassembling the entire long data */
                    SID_STM32_UTIL_fast_memcpy(&host_comm_rx_buf[hc_rx_received_data_size], packet->payload.ldtc.partial_data, copy_length);
                    hc_rx_received_data_size += copy_length;
                    err = HC_RX_ERROR_NONE;
                }
                else
                {
                    /* Unexpectedly received something that is not a valid LDTC packet */
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
                    HOST_COMM_LOG_ERROR("Received non-LDTC SPI packet when in LDTC mode. Received OpCode: 0x%x", packet->opcode);
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
                    hc_rx_state = HC_RX_STATE_ERROR;
                    err = HC_RX_ERROR_NON_LDTC_DATA_IN_LDTC;
                    *processing_finished = TRUE;
                    break; /* from the for() loop */
                }
            }

            /* Check if we received all the data */
            if (HC_RX_ERROR_NONE == err)
            {
                if (hc_rx_received_data_size >= hc_rx_expected_data_size)
                {
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
                    HOST_COMM_LOG_DEBUG("Long Data Transfer finished successfully, received %u bytes (with padding)", hc_rx_received_data_size);
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
                    hc_rx_received_data_size = hc_rx_expected_data_size; /* Exclude padding */
                    hc_rx_state = HC_RX_STATE_MSG_RECEIVED;
                    *processing_finished = TRUE;
                }
                else
                {
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
                    HOST_COMM_LOG_DEBUG("Valid LDTC arrived, now have %u of %u bytes",  hc_rx_received_data_size, hc_rx_expected_data_size);
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
                    err = HC_RX_ERROR_LONG_DATA_PENDING; /* Indicate that we are expecting more bytes to come and reside in the current state */
                }
            }
        }
        else
        {
            /* Something is seriously wrong as FIFO should always return a fixed-size frame */
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
            HOST_COMM_LOG_ERROR("Host comm Rx FIFO popped unexpected amount of data: %u instead of %u bytes", bytes_popped, (frames_to_pop * sizeof(stm32wlxx_radio_comm_spi_frame_t)));
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
            hc_rx_state = HC_RX_STATE_ERROR;
            err = HC_RX_ERROR_UNEXPECTED_POP_SIZE;
            *processing_finished = TRUE;
        }
    }
    else if (SERIAL_BUS_SPI_PAL_FIFO_OVERRUN == spi_pal_err)
    {
        /* FIFO overrun was detected, data may be corrupted */
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
        HOST_COMM_LOG_ERROR("Host comm Rx FIFO overrun detected");
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
        hc_rx_state = HC_RX_STATE_ERROR;
        err = HC_RX_ERROR_FIFO_OVERRUN;
        *processing_finished = TRUE;
    }
    else
    {
        /* Some other SPI driver error */
#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
        HOST_COMM_LOG_ERROR("Failed to pop frames from host comm Rx FIFO. SPI driver error %u", (uint32_t)spi_pal_err);
#endif /* HOST_COMM_HC_RX_SM_DEBUG */
        hc_rx_state = HC_RX_STATE_ERROR;
        err = HC_RX_ERROR_SPI_DRIVER;
        *processing_finished = TRUE;
    }

    return err;
}

/*----------------------------------------------------------------------------*/

static inline sid_host_comm_error_t _host_comm_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*
     * Sidewalk Host Comm IRQ line and Sidewalk Host Comm SPI NSS line cannot share the same pin number because both pins make use of EXTI.
     * The MCU shall be able to differentiate between the two EXTI IRQ sources
     */
    assert_param(SIDEWALK_RADIO_SPI_IRQ_Pin != SIDEWALK_RADIO_SPI_NSS_Pin);
    assert_param(IS_GPIO_CLK_ENABLED(SIDEWALK_RADIO_SPI_IRQ_GPIO_Port));

#if SID_HOST_COMM_PROCESSING_DELAY_PROFILING
    /* Configure GPIO pins that are used for Sidewalk delays profiling */
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin   = SID_PDP_RADIO_IRQ_ACTIVITY_Pin;

    SID_PDP_RADIO_IRQ_ACTIVITY_GPIO_Port->BRR = SID_PDP_RADIO_IRQ_ACTIVITY_Pin;

    HAL_GPIO_Init(SID_PDP_RADIO_IRQ_ACTIVITY_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = SID_PDP_HOST_COMM_ACTIVITY_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    SID_PDP_HOST_COMM_ACTIVITY_GPIO_Port->BRR = SID_PDP_HOST_COMM_ACTIVITY_Pin;

    HAL_GPIO_Init(SID_PDP_HOST_COMM_ACTIVITY_GPIO_Port, &GPIO_InitStruct);

    /* RF Busy pin */
    __HAL_RCC_GPIOA_CLK_ENABLE() ;
    GPIO_InitStruct.Pin       = (GPIO_PIN_12);
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_RF_BUSY;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif /* SID_HOST_COMM_PROCESSING_DELAY_PROFILING */

    /*  Configure IO in output open drain mode to drive IRQ pin */
    HAL_NVIC_DisableIRQ(SIDEWALK_RADIO_SPI_IRQ_EXTI_IRQn);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin   = SIDEWALK_RADIO_SPI_IRQ_Pin;

    /* Set predefined pin state before configuring it as output */
    SID_HOST_COMM_IRQ_CLEAR_EVENT();

    /* Apply GPIO config */
    HAL_GPIO_Init(SIDEWALK_RADIO_SPI_IRQ_GPIO_Port, &GPIO_InitStruct);

    /* Preconfigure EXTI line for the host comm IRQ line to be able to detect GPIO Handshake requests */
    /* Enable EXTI in SYSCFG */
    uint32_t position              =  SID_STM32_UTIL_POSITION_VAL(SIDEWALK_RADIO_SPI_IRQ_Pin);
    uint32_t temp                  =  SYSCFG->EXTICR[position >> 2];
    temp                           &= ~(0x07u << (4u * (position & 0x03u)));
    temp                           |= (GPIO_GET_INDEX(SIDEWALK_RADIO_SPI_IRQ_GPIO_Port) << (4u * (position & 0x03u)));
    SYSCFG->EXTICR[position >> 2u] =  temp;

    /* Ensure all trigger sources are disabled */
    CLEAR_BIT(EXTI->RTSR1, SIDEWALK_RADIO_SPI_IRQ_Pin);
    CLEAR_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_IRQ_Pin);

    /* Configure EXTI to trigger interrupt, but not to generate an event */
#ifdef CORE_CM0PLUS
    SET_BIT(EXTI->C2IMR1, SIDEWALK_RADIO_SPI_IRQ_Pin);
    CLEAR_BIT(EXTI->C2EMR1, SIDEWALK_RADIO_SPI_IRQ_Pin);
#else
    SET_BIT(EXTI->IMR1, SIDEWALK_RADIO_SPI_IRQ_Pin);
    CLEAR_BIT(EXTI->EMR1, SIDEWALK_RADIO_SPI_IRQ_Pin);
#endif /* CORE_CM0PLUS */

    return SID_HOST_COMM_ERROR_NONE;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static uint32_t _enter_handshake_state(const uint32_t full_spi_reset)
{
    uint32_t success = FALSE;

    /* SubGHz reset and initialization -------------------------------------------*/
    sid_radio_error_t radio_err = sid_radio_reset_hardware();
    if (radio_err != SID_RADIO_ERROR_NONE)
    {
        success = FALSE;
        goto exit;
    }
    /*----------------------------------------------------------------------------*/

    /* SPI handling - try just Tx/Rx buffers cleanup first -----------------------*/
    serial_bus_spi_pal_err_t spi_err = SERIAL_BUS_SPI_PAL_OK;
    if ((FALSE == full_spi_reset) && (HAL_SPI_STATE_BUSY_TX_RX == SIDEWALK_RADIO_SPI.State))
    {
        /* SPI is in a valid state and full SPI reset was not requested - we may just clear the Tx/Rx buffers and that's it */
        spi_err = serial_bus_spi_clear_fifos(HOST_COMM_SPI_FIFO_CLEAN_TIMEOUT_MS);

        if (spi_err != SERIAL_BUS_SPI_PAL_OK)
        {
            /* Something went wrong */
            SID_PAL_LOG_WARNING("Failed to clear host comm SPI Tx/Rx buffer. Error %u. Will try full SPI reinit", (uint32_t)spi_err);
        }
        else
        {
            SID_PAL_LOG_DEBUG("Cleared host comm SPI Tx/Rx buffers");
        }
    }

    /* If SPI encountered an error or buffers cleanup failed try a more powerful method */
    if ((full_spi_reset != FALSE) || (HAL_SPI_STATE_ERROR == SIDEWALK_RADIO_SPI.State) || (spi_err != SERIAL_BUS_SPI_PAL_OK))
    {
        spi_err = serial_bus_spi_full_reinit();
        if (spi_err != SERIAL_BUS_SPI_PAL_OK)
        {
            /* Not too much we can do about it, abandon Handshake entry attempt */
            SID_PAL_LOG_ERROR("Failed to reset host comm SPI. Error %u. Handshake state entry failed", (uint32_t)spi_err);
            success = FALSE;
            goto exit;
        }
        else
        {
            SID_PAL_LOG_DEBUG("Host comm SPI reinitialized");
        }
    }

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
    /* Remove LPM restriction since SPI FIFO is now empty and there's nothing to process */
    UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_CMD_PROCESS), UTIL_LPM_ENABLE);
#endif /* LOW_POWER_DISABLE */

    /* If SPI is ok, but transfers are not started yet */
    if (HAL_SPI_STATE_READY == SIDEWALK_RADIO_SPI.State)
    {
        /* Start the autonomous SPI communication. Recoverable errors are possible so several attempts can be made */
        uint32_t remaining_attempts = HOST_COMM_SPI_XFER_START_ATTEMPTS;
        do
        {
            spi_err = serial_bus_spi_xfer_start();
            if (SERIAL_BUS_SPI_PAL_TRY_LATER == spi_err)
            {
                /* Recoverable error (probably the NSS line is active and blocks the start) */
                SID_PAL_LOG_WARNING("Can't start host comm SPI bus communication. A retry will be attempted");
                sid_pal_scheduler_delay_ms(HOST_COMM_SPI_XFER_START_COOLDOWN_MS);
            }
            else
            {
                /* Either everything is ok or unrecoverable error happened - jump out in both cases */
                break;
            }

            remaining_attempts--;
        } while (remaining_attempts != 0u);

        /* Inspect the results of SPI communication startup */
        if (SERIAL_BUS_SPI_PAL_OK != spi_err)
        {
            SID_PAL_LOG_ERROR("Can't start host comm SPI bus communication. Unrecoverable error %u", (uint32_t)spi_err);
            success = FALSE;
            goto exit;
        }
        else
        {
            SID_PAL_LOG_DEBUG("Host comm SPI xfers started");
        }
    }
    /*----------------------------------------------------------------------------*/

    /* Host Comm protocol Rx state machine reset ---------------------------------*/
    HC_RX_STATE_MACHINE_RESET();
    /*----------------------------------------------------------------------------*/

    /* Enqueue IRQ Status frame with the Handshake Event bit set -----------------*/
    sid_radio_error_t sid_radio_err = sid_radio_set_handshake_response();
    if (sid_radio_err != SID_RADIO_ERROR_NONE)
    {
        /* Something went wrong */
        SID_PAL_LOG_ERROR("Failed to set Handshake Response SPI payload. SID Radio error %u", (uint32_t)sid_radio_err);
        success = FALSE;
        goto exit;
    }
    else
    {
        SID_PAL_LOG_DEBUG("Set Handshake Response SPI payload");
    }
    /*----------------------------------------------------------------------------*/

    /* Everything is ok if we've got here */
    success = TRUE;

exit:
    return success;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void _internal_handshake_request(void)
{
    /* Go into the Handshake state to re-synchronize the communication */
    HC_HANDSHAKE_PRE_ENTRY_ACTION();
#ifdef DEBUG
    const uint32_t notify_flags =
#else
    /* Discard the return value for release builds to facilitate more compiler optimizations and speed up the processing.
     * osThreadFlagsSet return error only for the systematic failures so not having this check in release is absolutely fine
     */
    (void)
#endif /* DEBUG */
    osThreadFlagsSet(host_comm_task_handle, COMM_PROC_HANDSHAKE_INTERNAL_REQUEST_FLAG);
#ifdef DEBUG
    if ((notify_flags & osFlagsError) != 0u)
    {
        /* This is a systematic failure, stop SW execution */
        SID_PAL_LOG_ERROR("Unable to notify handler task to enter GPIO Handshake request. Error 0x%x", notify_flags);
        Error_Handler();
    }
#endif /* DEBUG */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _host_comm_task(void * argument)
{
    (void)argument;

    /* Configure Handshake IRQ once the task is started and ready to accept Handshake requests */
    __HAL_GPIO_EXTI_CLEAR_IT(SIDEWALK_RADIO_SPI_IRQ_Pin);
    HAL_NVIC_SetPriority(SIDEWALK_RADIO_SPI_IRQ_EXTI_IRQn, SIDEWALK_RADIO_SPI_IRQ_EXTI_IRQn_PRIORITY, 0); /* Set the highest possible IRQ priority that still allows to use RTOS API */
    HAL_NVIC_EnableIRQ(SIDEWALK_RADIO_SPI_IRQ_EXTI_IRQn);

    SID_PAL_LOG_INFO("STM32WLxx Radio App started");

    while (1)
    {
#if CFG_LED_SUPPORTED
        /* Indicate there's nothing to do and we are waiting for the incoming data */
        BSP_LED_Off(LED_RED);
#endif /* CFG_LED_SUPPORTED */

        uint32_t event_flags = osThreadFlagsWait(COMM_PROC_HANDSHAKE_INTERNAL_REQUEST_FLAG
                                               | COMM_PROC_HANDSHAKE_EXTERNAL_REQUEST_FLAG
                                               | COMM_PROC_SPI_ERROR_FLAG
                                               | COMM_PROC_DATA_AVAILABLE_FLAG
                                               | COMM_PROC_RADIO_INTERRUPT_FLAG,
                                                 osFlagsWaitAny, osWaitForever);
#if CFG_LED_SUPPORTED
        /* Indicate the processing has started */
        BSP_LED_On(LED_RED);
#endif /* CFG_LED_SUPPORTED */

        /* Check if the flags were received at all -----------------------------------*/
        if ((event_flags & osFlagsError) != 0u)
        {
            /* This is a systematic failure, stop SW execution */
            SID_PAL_LOG_ERROR("Unable to retrieve Host Comm event flags. Error 0x%x", event_flags);
            Error_Handler();
        }

        /* SPI error reaction - highest priority (0) ---------------------------------*/
        if ((event_flags & COMM_PROC_SPI_ERROR_FLAG) != 0u)
        {
            if ((HAL_SPI_ERROR_CRC == host_comm_spi_last_error_info.error_code))
            {
                /* If that's only the CRC error provide short and clear message without excessive details */
                SID_PAL_LOG_ERROR("Host comm SPI CRC error occurred");
            }
            else
            {
                /* Provide full scale SPI error message */
                SID_PAL_LOG_ERROR("Host comm SPI error occurred: 0x%x", host_comm_spi_last_error_info.error_code);
                SID_PAL_LOG_DEBUG("SPI SR: 0x%08x, DMA Tx CCR: 0x%08x, DMA Rx CCR: 0x%08x, DMA NSS Detect CCR: 0x%08x, DMA SCK Control CCR: 0x%08x",
                                  host_comm_spi_last_error_info.spi_sr, host_comm_spi_last_error_info.dma_tx_ccr, host_comm_spi_last_error_info.dma_rx_ccr,
                                  host_comm_spi_last_error_info.dma_nss_dtct_ccr, host_comm_spi_last_error_info.dma_sck_ctrl_ccr);
            }

            /* Fall through to the Handshake procedure since logical state synchronization is now lost due to SPI error */
            event_flags |= COMM_PROC_HANDSHAKE_INTERNAL_REQUEST_FLAG;
            HC_HANDSHAKE_PRE_ENTRY_ACTION();
        }
        /*----------------------------------------------------------------------------*/

        /* Handshake with the external MCU - priority 1 ----------------------------- */
        if ((event_flags & (COMM_PROC_HANDSHAKE_INTERNAL_REQUEST_FLAG | COMM_PROC_HANDSHAKE_EXTERNAL_REQUEST_FLAG)) != 0u)
        {
            /* Ensure we are not keeping the IRQ line low from some past SubGHz events */
            SID_HOST_COMM_IRQ_CLEAR_EVENT();

            /* Print out a log message */
            if ((event_flags & COMM_PROC_HANDSHAKE_INTERNAL_REQUEST_FLAG) != 0u)
            {
                SID_PAL_LOG_INFO("Entering GPIO Handshake state by an internal request");
            }
            else
            {
                SID_PAL_LOG_WARNING("Got external request for GPIO Handshake");
            }

            /* Enter the Handshake state. For external handshake request perform a full SPI re-init to avoid partial frame captures */
            if (_enter_handshake_state((event_flags & COMM_PROC_HANDSHAKE_EXTERNAL_REQUEST_FLAG) == 0u ? FALSE : TRUE) != FALSE)
            {
                /* Now actively drive the IRQ line to indicate readiness */
                SID_HOST_COMM_IRQ_INDICATE_EVENT();
                SID_PAL_LOG_DEBUG("Entered GPIO Handshake state");
            }
            else
            {
                SID_PAL_LOG_ERROR("Failed to enter the Handshake state");
                /* Reactivate IRQ trigger source and leave it for good */
                SID_HOST_COMM_IRQ_ENABLE_TRIGGER();
            }

            /* Since processing of other flags is irrelevant after resetting everything just proceed with the next task iteration */
            continue;
        }
        /*----------------------------------------------------------------------------*/

        /* SubGHz Interrupts Handling - priority 2 ---------------------------------- */
        if ((event_flags & COMM_PROC_RADIO_INTERRUPT_FLAG) != 0u)
        {
            sid_radio_error_t sid_radio_err;

            /* Call SubGHz IRQ handler to retrieve the IRQ status from SubGHz peripheral */
            sid_radio_err = sid_radio_handle_subghz_irq(subghz_irq_timestamp_s, subghz_irq_timestamp_us);

            if (sid_radio_err != SID_RADIO_ERROR_NONE)
            {
                /* Logs provided by sid_radio_handle_subghz_irq() */
                _internal_handshake_request();
            }
            else
            {
                HOST_COMM_LOG_DEBUG("SubGHz IRQ processed");
            }
        }
        /*----------------------------------------------------------------------------*/

        /* Process Data from the Host MCU - priority 3 ------------------------------ */
        if ((event_flags & COMM_PROC_DATA_AVAILABLE_FLAG) != 0u)
        {
            uint32_t fifo_level = 0u;

#if defined(HOST_COMM_HC_RX_SM_DEBUG) && HOST_COMM_HC_RX_SM_DEBUG
            HOST_COMM_LOG_DEBUG("Host Comm task SPI Rx FIFO level on entry: %u", fifo_level);
#endif /* HOST_COMM_HC_RX_SM_DEBUG */

            do
            {
                /* Run the generic SPI Rx FIFO processing */
                const host_comm_rx_error_t hc_rx_err = _host_comm_spi_rx_process();

                switch (hc_rx_err)
                {
                    case HC_RX_ERROR_LONG_DATA_PENDING:
                        if (hc_rx_received_data_size <= SID_STM32_UTIL_STRUCT_MEMBER_SIZE(stm32wlxx_radio_comm_spi_frame_t, payload.raw))
                        {
                            /* Too few bytes received, keep on waiting */
                            break;
                        }
                        else
                        {
                            /* We can start processing even if partial data is available, just the first frame is enough - fall through to HC_RX_ERROR_NONE */
                            SID_STM32_JUSTIFY_FALLTHROUGH();
                        }
                    case HC_RX_ERROR_NONE:
#if SID_HOST_COMM_PROCESSING_DELAY_PROFILING
                        SID_PDP_HOST_COMM_ACTIVITY_GPIO_Port->BSRR = SID_PDP_HOST_COMM_ACTIVITY_Pin;
#endif /* SID_HOST_COMM_PROCESSING_DELAY_PROFILING */
                        {
                            /* We've got a valid protocol message and can process it */
                            sid_radio_error_t proc_err;
                            proc_err = sid_radio_process_app_frame(host_comm_rx_buf, hc_rx_received_data_size, hc_rx_expected_data_size);

                            if (proc_err != SID_RADIO_ERROR_NONE)
                            {
                                if (proc_err != SID_RADIO_ERROR_NOT_SUPPORTED)
                                {
                                    /* Go into the Handshake state to re-synchronize the communication */
                                    SID_PAL_LOG_ERROR("Failed to process Sidewalk Radio app command. Error %u", (uint32_t)proc_err);
                                    _internal_handshake_request();
                                }
                                else
                                {
                                    /* That was a valid command that is not supported by the current firmware and/or hardware - just ignore it and proceed */
                                }
                            }
                        }
#if SID_HOST_COMM_PROCESSING_DELAY_PROFILING
                        SID_PDP_HOST_COMM_ACTIVITY_GPIO_Port->BRR = SID_PDP_HOST_COMM_ACTIVITY_Pin;
#endif /* SID_HOST_COMM_PROCESSING_DELAY_PROFILING */
                        break;

                    case HC_RX_ERROR_NO_DATA_AVAILABLE:
                        /* These are not the errors, but they mean we don't have a valid protocol message yet - do nothing, just wait */
                        break;


                    default:
                        /* All the other errors are considered critical and require communication reset */
                        SID_PAL_LOG_ERROR("Host comm SPI Rx processing fatal error %u. Inter-MCU comm reset is required", (uint32_t)hc_rx_err);
                        _internal_handshake_request();
                        break;
                }

                /* Check if anything new happened while we were processing the incoming data */
                const uint32_t task_flags_now = osThreadFlagsGet();
                if ((task_flags_now != COMM_PROC_NO_ACTIVE_FLAGS) && (task_flags_now != event_flags))
                {
                    /* There's something additional, let's quit and collect the new flags to avoid event priority inversion */
                    HOST_COMM_LOG_DEBUG("Stopping host comm Rx processing - new task notification arrived. Old: 0x%08x, new: 0x%08x", event_flags, task_flags_now);
                    break;
                }
                else
                {
                    /* Update the FIFO level since new data may have arrived */
                    fifo_level = serial_bus_spi_get_rx_fifo_level();
                }
            } while (fifo_level > 0u); /* No SERIAL_BUS_SPI_PAL_FIFO_LEVEL_OVERRUN check here since we need error reaction */

#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
            /* FIFO will be empty at this point, remove LPM restriction */
            UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_CMD_PROCESS), UTIL_LPM_ENABLE);
#endif /* LOW_POWER_DISABLE */
        }
        /*----------------------------------------------------------------------------*/
    }

    SID_PAL_LOG_WARNING("Host Comm task is terminated");

    /* If it ever gets here terminate the task nicely */
    osThreadExit();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _on_host_comm_spi_error(const serial_bus_spi_pal_bus_error_details_t * const error_info)
{
    /* Store error info */
    host_comm_spi_last_error_info = *error_info; /* use direct assignment to avoid function calls and minimize the execution time */

    /* Notify the HostComm task about SPI error */
#ifdef DEBUG
    register const uint32_t err =
#else
    /* Discard the return value for release builds to facilitate more compiler optimizations and speed up the processing.
     * osThreadFlagsSet return error only for the systematic failures so not having this check in release is absolutely fine
     */
    (void)
#endif /* DEBUG */
    osThreadFlagsSet(host_comm_task_handle, COMM_PROC_SPI_ERROR_FLAG);
#ifdef DEBUG
    if ((err & osFlagsError) != 0u)
    {
        SID_PAL_LOG_ERROR("Unable to notify handler task about SPI error. Error 0x%x", err);
        Error_Handler();
    }
#endif /* DEBUG */
}

/* Global function definitions -----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_host_comm_notify_handshake_requested(void)
{
    register uint32_t probe_cnt    = 0u;
    register uint32_t debounce_cnt = 0u;

    /* Since the IRQ line is driven by open-drain output and a weak pull up of the host MCU is used, it is prone to EMI and
     * other noise sources. That's why some filtering is needed before acting on the IRQ 
     */

    /* There's no need to disable either SubGHZ Radio or EXTI IRQs here:
     * 1. If SubGHz IRQ fires while this handler is running and SubGHz IRQ has lower priority it won't affect filtering process
     * 2. If SubGHz IRQ fires and it has higher priority it will preempt current IRQ handler, but the only action is setting an
     *    event flag for the Host Comm task.
     * 3. EXTI for this pin will not fire again until we clear the flag in EXTI here even if the actual pin is toggling
     */
    do
    {
        /* Wait a bit for pin state to settle */
        sid_pal_delay_us(HOST_COMM_IRQ_PIN_STATE_PROBE_PERIOD_US);

        /* Check pin status */
        if ((SIDEWALK_RADIO_SPI_IRQ_GPIO_Port->IDR & SIDEWALK_RADIO_SPI_IRQ_Pin) == 0u)
        {
            /* IRQ line is driven low */
            debounce_cnt++;
        }
        else
        {
            /* IRQ line is now high, IRQ may have been triggered by a surge */
            debounce_cnt = 0u;
        }

        probe_cnt++;
    } while (probe_cnt < HOST_COMM_IRQ_PIN_STATE_PROBE_COUNT);

    if (debounce_cnt < HOST_COMM_IRQ_PIN_STATE_PROBE_DEBOUNCE)
    {
        /* Seems the IRQ was accidentally triggered by a glitch - ignore it */
        HOST_COMM_LOG_DEBUG("IRQ line filtered");
        __HAL_GPIO_EXTI_CLEAR_IT(SIDEWALK_RADIO_SPI_IRQ_Pin);
    }
    else
    {
        /* Deactivate IRQ trigger source and clear IRQ flag */
        HC_HANDSHAKE_PRE_ENTRY_ACTION();

        /* Notify the task that GPIO Handshake was requested */
#ifdef DEBUG
        register const uint32_t err =
#else
        /* Discard the return value for release builds to facilitate more compiler optimizations and speed up the processing.
         * osThreadFlagsSet return error only for the systematic failures so not having this check in release is absolutely fine
         */
        (void)
#endif /* DEBUG */
        osThreadFlagsSet(host_comm_task_handle, COMM_PROC_HANDSHAKE_EXTERNAL_REQUEST_FLAG);
#ifdef DEBUG
        if ((err & osFlagsError) != 0u)
        {
            SID_PAL_LOG_ERROR("Unable to notify handler task about external GPIO Handshake request. Error 0x%x", err);
            Error_Handler();
        }
#endif /* DEBUG */
    }
}

/*----------------------------------------------------------------------------*/

sid_host_comm_error_t sid_host_comm_init()
{
    sid_host_comm_error_t    err = SID_HOST_COMM_ERROR_GENERIC;
    serial_bus_spi_pal_err_t spi_pal_err;
    sid_radio_error_t        radio_err;

    do
    {
        /* Initialize IRQ line GPIO pin */
        err = _host_comm_gpio_init();
        if (err != SID_HOST_COMM_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Unable to initialize Sidewalk Radio-related GPIO pins, error %u", (uint32_t)err);
            break;
        }

        /* Initialize host communication SPI bus */
        spi_pal_err = sid_pal_serial_bus_spi_init(sid_host_comm_on_spi_frame_received, _on_host_comm_spi_error);
        if (SERIAL_BUS_SPI_PAL_OK != spi_pal_err)
        {
            SID_PAL_LOG_ERROR("Unable to initialize Sidewalk Radio SPI driver. Error %u", (uint32_t)spi_pal_err);
            err = SID_HOST_COMM_ERROR_HARDWARE;
            break;
        }

        /* Initialize SubGHz driver */
        radio_err = sid_radio_app_init();
        if (radio_err != SID_RADIO_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to perform SubGHz driver initialization. Error %u", (uint32_t)radio_err);
            err = SID_HOST_COMM_ERROR_HARDWARE;
            break;
        }

        /* Create host comm processing task if needed */
        if(NULL == host_comm_task_handle)
        {
            host_comm_task_handle = osThreadNew(_host_comm_task, NULL, &host_comm_task_attributes);
            if (NULL == host_comm_task_handle)
            {
                SID_PAL_LOG_ERROR("Unable to create Sidewalk Radio communication processing task");
                err = SID_HOST_COMM_ERROR_RESOURCE_ALLOC;
                break;
            }
        }

        /* Hardware configuration done, proceed to the Handshake State */
        _internal_handshake_request();

        err = SID_HOST_COMM_ERROR_NONE;
    } while(0);

    return err;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_host_comm_on_spi_frame_received(void)
{
#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
    /* Prohibit the system to enter Stop mode until incoming data is processed */
    UTIL_LPM_SetStopMode((1u << CFG_LPM_HOST_COMM_CMD_PROCESS), UTIL_LPM_DISABLE);
    __COMPILER_BARRIER();
#endif /* LOW_POWER_DISABLE */

    /* Notify the HostComm task about fresh data in the host comm SPI Rx buffer */
#ifdef DEBUG
    register const uint32_t err =
#else
    /* Discard the return value for release builds to facilitate more compiler optimizations and speed up the processing.
     * osThreadFlagsSet return error only for the systematic failures so not having this check in release is absolutely fine
     */
    (void)
#endif /* DEBUG */
    osThreadFlagsSet(host_comm_task_handle, COMM_PROC_DATA_AVAILABLE_FLAG);
#ifdef DEBUG
    if ((err & osFlagsError) != 0u)
    {
        SID_PAL_LOG_ERROR("Unable to notify handler task about newly received host comm SPI data. Error 0x%x", err);
        Error_Handler();
    }
#endif /* DEBUG */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_host_comm_error_t sid_host_comm_enqueue_tx(const uint8_t * const data, const uint32_t data_size)
{
    sid_host_comm_error_t err = SID_HOST_COMM_ERROR_GENERIC;
    serial_bus_spi_pal_err_t spi_pal_err;

    do
    {
        /* Validate the inputs */
        if ((NULL == data) || (0u == data_size))
        {
            SID_PAL_LOG_WARNING("Requested to add null data to host comm SPI Tx queue - ignored");
            err = SID_HOST_COMM_INVALID_ARGS;
            break;
        }

        if (data_size > STM32WLxx_RADIO_COMM_MTU_SIZE)
        {
            SID_PAL_LOG_ERROR("Unable to send %u bytes via host comm SPI. MTU size limit is %u", data_size, STM32WLxx_RADIO_COMM_MTU_SIZE);
            err = SID_HOST_COMM_ERROR_EXCEEDS_MTU;
            break;
        }

        /* Get the number of the available slots in the Tx FIFO */
        const uint32_t fifo_free_slots = serial_bus_spi_get_tx_fifo_free_level();
#if defined(HOST_COMM_SPI_TX_DEBUG) && HOST_COMM_SPI_TX_DEBUG
        HOST_COMM_LOG_DEBUG("Host comm SPI Tx FIFO free level before enqueue: %u", fifo_free_slots);
#endif /* HOST_COMM_SPI_TX_DEBUG */

        /* Determine the type of transfer - normal mode or Long Data Transfer mode */
        if (data_size <= sizeof(stm32wlxx_radio_comm_spi_frame_t))
        {
            /* Data fits into a single SPI frame - do a straight forward data transfer */
            if (0u == fifo_free_slots)
            {
                SID_PAL_LOG_ERROR("Failed to enqueue single SPI Tx frame. Tx FIFO is full");
                err = SID_HOST_COMM_ERROR_TX_FIFO_FULL;
                break;
            }
            else
            {
                spi_pal_err =  serial_bus_spi_enqueue_tx(data, data_size);
                if (spi_pal_err !=  SERIAL_BUS_SPI_PAL_OK)
                {
                    SID_PAL_LOG_ERROR("Failed to enqueue single SPI Tx frame. PAL error %u", (uint32_t)err);
                    err = SID_HOST_COMM_ERROR_HARDWARE;
                    break;
                }
                else
                {
#if defined(HOST_COMM_SPI_TX_DEBUG) && HOST_COMM_SPI_TX_DEBUG
                    HOST_COMM_LOG_DEBUG("Successfully enqueued single SPI Tx frame (payload size: %u)", data_size);
#endif /* HOST_COMM_SPI_TX_DEBUG */
                    err = SID_HOST_COMM_ERROR_NONE;
                    break;
                }
            }
        }
        else
        {
            /* We have more data than  a single frame, Long Data Transfer mode shall be used */
            const uint32_t total_frames_to_enqueue = 1u /* LDTS frame is always present */
                                                     + (  (data_size
                                                           - SID_STM32_UTIL_STRUCT_MEMBER_SIZE(stm32wlxx_rcp_ldts_t, partial_data) /* Subtract bytes that will go into LDTS frame */
                                                           + (SID_STM32_UTIL_STRUCT_MEMBER_SIZE(stm32wlxx_rcp_ldtc_t, partial_data) - 1u) /* Add this for rounding up */
                                                          )
                                                          / SID_STM32_UTIL_STRUCT_MEMBER_SIZE(stm32wlxx_rcp_ldtc_t, partial_data) /* Calculate the amount of LDTC frames */
                                                       );

            if (total_frames_to_enqueue > fifo_free_slots)
            {
                SID_PAL_LOG_ERROR("Failed to enqueue %u SPI Tx LDT frames. Tx FIFO has only %u free slots", total_frames_to_enqueue, fifo_free_slots);
                err = SID_HOST_COMM_ERROR_TX_FIFO_FULL;
                break;
            }
            else
            {
                /* Arrange src and dst pointers */
                const uint8_t * src_ptr = data;
                stm32wlxx_radio_comm_spi_frame_t * out_packet = (stm32wlxx_radio_comm_spi_frame_t *)(void *)spi_tx_processing_buf;
                register uint32_t bytes_remaining = data_size;
                register uint32_t processing_buf_filled_data_size;

                /*Build LDTS frame first */
                out_packet->opcode = STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_START;
                out_packet->payload.ldts.full_data_size = data_size;
                SID_STM32_UTIL_fast_memcpy(out_packet->payload.ldts.partial_data, src_ptr, sizeof(out_packet->payload.ldts.partial_data));

                /* Advance the pointers */
                src_ptr += sizeof(out_packet->payload.ldts.partial_data);
                out_packet++;

                /* Update the counters */
                bytes_remaining -=  sizeof(out_packet->payload.ldts.partial_data);
                processing_buf_filled_data_size = sizeof(*out_packet);

                while (bytes_remaining > 0u)
                {
                    /* Put as many frames into the processing buffer as we can */
                    while ((processing_buf_filled_data_size < sizeof(spi_tx_processing_buf)) && (bytes_remaining > 0))
                    {
                        /*Build LDTC frame */
                        out_packet->opcode = STM32WLxx_RADIO_COMM_OPCODE_LONG_DATA_CONT;

                        /* Source buffer may not have enough bytes to fill the entire payload space - copy only as many bytes as the source buffer can provide,
                         * padding will be arranged by the SPI driver automatically
                         */
                        const uint32_t bytes_to_copy = bytes_remaining > sizeof(out_packet->payload.ldtc.partial_data) ? sizeof(out_packet->payload.ldtc.partial_data) : bytes_remaining;
                        SID_STM32_UTIL_fast_memcpy(out_packet->payload.ldtc.partial_data, src_ptr, bytes_to_copy);

                        /* Advance the pointers */
                        src_ptr += bytes_to_copy;
                        out_packet++;

                        /* Update the counters */
                        bytes_remaining -=  bytes_to_copy;
                        processing_buf_filled_data_size += (sizeof(*out_packet)
                                                            - sizeof(out_packet->payload)
                                                            + bytes_to_copy);
                    }

                    /* Now enqueue what we've prepared into the SPI Tx FIFO at once */
                    spi_pal_err =  serial_bus_spi_enqueue_tx(spi_tx_processing_buf, processing_buf_filled_data_size);
                    if (spi_pal_err !=  SERIAL_BUS_SPI_PAL_OK)
                    {
                        SID_PAL_LOG_ERROR("Failed to enqueue SPI Tx LDT frames. PAL error %u", (uint32_t)err);
                        err = SID_HOST_COMM_ERROR_HARDWARE;
                        break;
                    }
                    else
                    {
                        err = SID_HOST_COMM_ERROR_NONE;
                        /* No break - continue with the next iteration if needed */

                        /* Reset the prepared data counter and processing buffer pointer */
                        processing_buf_filled_data_size = 0u;
                        out_packet = (stm32wlxx_radio_comm_spi_frame_t *)(void *)spi_tx_processing_buf;
                    }
                }

#if defined(HOST_COMM_SPI_TX_DEBUG) && HOST_COMM_SPI_TX_DEBUG
                if (SID_HOST_COMM_ERROR_NONE == err)
                {
                    HOST_COMM_LOG_DEBUG("Successfully enqueued all SPI Tx LDT frames (payload size: %u)", data_size);
                }
#endif /* HOST_COMM_SPI_TX_DEBUG */
            }
        }
    } while (0);

#if defined(HOST_COMM_SPI_TX_DEBUG) && HOST_COMM_SPI_TX_DEBUG
    HOST_COMM_LOG_DEBUG("Host comm SPI Tx FIFO free level after enqueue: %u", serial_bus_spi_get_tx_fifo_free_level());
#endif /* HOST_COMM_SPI_TX_DEBUG */

    return err;
}

/*----------------------------------------------------------------------------*/

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
__WEAK sid_host_comm_error_t sid_host_comm_udt_user_init(void)
{
    SID_PAL_LOG_WARNING("No UDT init function is defined. UDT will be inoperable");
    return SID_HOST_COMM_ERROR_NONE;
}
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/*----------------------------------------------------------------------------*/

/**
  * @brief This function handles SUBGHZ Radio Interrupt.
  */
SID_STM32_SPEED_OPTIMIZED void SUBGHZ_Radio_IRQHandler(void)
{
    /* Capture the IRQ timestamp */
    const register uint32_t primask_bit = __get_PRIMASK();
    __disable_irq(); /* Disable all interrupts */
    __COMPILER_BARRIER();

    /* Capture radio event timestamp */
    uint32_t timestamp_s;
    uint32_t timestamp_us;
    timestamp_s = TIMER_IF_GetTimeUs(&timestamp_us);
    __COMPILER_BARRIER();

    /*vvv WORKAROUND FOR STMC-495 vvvvvvvvvv*/
#ifdef DEBUG
    if (LL_EXTI_IsActiveFlag_32_63(LL_EXTI_LINE_45) != 0u)
    {
        /* If the IRQ was triggered by the BUSY line (EXTI line 45), something is wrong with the workaround mechanism. Normally IRQ should be disabled whenever edge detection is enabled for EXTI line 45 */
        assert_param(0);
    }
#endif /* DEBUG */
    /*^^^ END OF WORKAROUND FOR STMC-495 ^^^*/

    /* Immediately stop software timer for Tx/Rx/CS/CAD timeout - we don't care about the exact IRQ here*/
    (void)UTIL_TIMER_Stop(&radio_timeout_mon);
    __COMPILER_BARRIER();

#if SID_HOST_COMM_PROCESSING_DELAY_PROFILING
    /* Indicate the actual radio IRQ event for profiling */
    SID_PDP_RADIO_IRQ_ACTIVITY_GPIO_Port->BSRR = SID_PDP_RADIO_IRQ_ACTIVITY_Pin;
#endif /* SID_HOST_COMM_PROCESSING_DELAY_PROFILING */

    /* Restore global interrupt settings */
    __set_PRIMASK(primask_bit);

    /* Get current radio IRQ priority */
    uint32_t radio_irq_prio, radio_irq_sub_prio;
    HAL_NVIC_GetPriority(SUBGHZ_Radio_IRQn, NVIC_GetPriorityGrouping(), &radio_irq_prio, &radio_irq_sub_prio);
    (void)radio_irq_sub_prio;

    /* Check if ISR runs in the elevated priority mode */
    if (radio_irq_prio != RADIO_IRQ_PRIO_LOW)
    {
        /* Store radio event timestamp */
        subghz_irq_timestamp_s  = timestamp_s;
        subghz_irq_timestamp_us = timestamp_us;

        /* Lower the radio IRQ priority so that RTOS API can be safely used */
        HAL_NVIC_SetPriority(SUBGHZ_Radio_IRQn, RADIO_IRQ_PRIO_LOW, 0u);

        /* Return from ISR to allow the NVIC to re-evaluate the IRQ priorities and call this handler again when it safe to access RTOS API */
        return;
    }

    /* Disable SubGHz IRQ until the task processes the current one. Otherwise it will result in an infinite IRQ handler loop */
    HAL_NVIC_DisableIRQ(SUBGHZ_Radio_IRQn);
    __COMPILER_BARRIER();

#ifdef DEBUG
    const uint32_t notify_flags =
#else
    /* Discard the return value for release builds to facilitate more compiler optimizations and speed up the processing.
     * osThreadFlagsSet return error only for the systematic failures so not having this check in release is absolutely fine
     */
    (void)
#endif /* DEBUG */
    osThreadFlagsSet(host_comm_task_handle, COMM_PROC_RADIO_INTERRUPT_FLAG);
#ifdef DEBUG
    if ((notify_flags & osFlagsError) != 0u)
    {
        /* This is a systematic failure, stop SW execution */
        SID_PAL_LOG_ERROR("Unable to notify handler task to handle SubGHz IRQ. Error 0x%x", notify_flags);
        Error_Handler();
    }
#endif /* DEBUG */
}
