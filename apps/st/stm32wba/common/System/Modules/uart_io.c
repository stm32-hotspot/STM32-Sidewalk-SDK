/**
  ******************************************************************************
  * @file           : uart_io.c
  * @brief          : Implementation of the standard I/O functions using UART
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <sid_pal_assert_ifc.h>
#include <sid_pal_log_ifc.h>
#include <sid_stm32_common_utils.h>

#include <cmsis_os.h>

#include "main.h"
#include "app_conf.h"
#include "stm32_adv_trace.h"
#include "stm32wbaxx_hal.h"

/* Private defines -----------------------------------------------------------*/

#ifndef UART_IO_TX_TIMEOUT_MS
#  define UART_IO_TX_TIMEOUT_MS      (5u)
#endif /* UART_IO_TX_TIMEOUT_MS */

#ifndef UART_IO_RX_TIMEOUT_MS
#  define UART_IO_RX_TIMEOUT_MS      (5u)
#endif /* UART_IO_RX_TIMEOUT_MS */

#ifndef UART_IO_RX_POLL_PERIOD_MS
#  define UART_IO_RX_POLL_PERIOD_MS  (2u)
#endif /* UART_IO_RX_POLL_PERIOD_MS */

#ifndef UART_IO_RX_BUFER_SIZE
#  define UART_IO_RX_BUFER_SIZE      (512u)
#endif /* UART_IO_RX_BUFER_SIZE */

#ifndef UART_IO_RX_ENABLED
#  define UART_IO_RX_ENABLED         (1)
#endif /* UART_IO_RX_ENABLED */

#ifndef UART_IO_USE_UTIL_ADV_TRACE
#  define UART_IO_USE_UTIL_ADV_TRACE (1)
#endif

/* Private macro -------------------------------------------------------------*/

#define UART_IO_MS_TO_OS_TICKS(_MS_) (((_MS_) * osKernelGetTickFreq()) / 1000u) /*!< Convert milliseconds to RTOS ticks */

/* Private variables ---------------------------------------------------------*/

#if UART_IO_RX_ENABLED
static char     uart_rx_buf[UART_IO_RX_BUFER_SIZE];
static char *   rx_buf_read_ptr = &uart_rx_buf[0];
static uint32_t rx_buf_write_wrap_flag;
#endif /* UART_IO_RX_ENABLED */

/* Private function prototypes -----------------------------------------------*/

#if UART_IO_RX_ENABLED
static        void     uart_io_rx_buffer_wrap(UART_HandleTypeDef *huart);
#  if (UART_IO_RX_TIMEOUT_MS > 0)
static inline uint32_t uart_io_rx_buffer_has_data(void);
#  endif /* (UART_IO_RX_TIMEOUT_MS > 0) */
#endif /* UART_IO_RX_ENABLED */

/* Private function definitions ----------------------------------------------*/

#if UART_IO_RX_ENABLED
SID_STM32_SPEED_OPTIMIZED static void uart_io_rx_buffer_wrap(UART_HandleTypeDef *huart)
{
    (void)huart;

    UTILS_ENTER_CRITICAL_SECTION();
    rx_buf_write_wrap_flag = TRUE;
    UTILS_EXIT_CRITICAL_SECTION();
}
#endif /* UART_IO_RX_ENABLED */

/*----------------------------------------------------------------------------*/

#if (UART_IO_RX_TIMEOUT_MS > 0)
SID_STM32_SPEED_OPTIMIZED static inline uint32_t uart_io_rx_buffer_has_data(void)
{
    UTILS_ENTER_CRITICAL_SECTION();
    register uint32_t has_data = FALSE;
    register uint32_t dma_write_ptr = LOG_UART_HANDLER.hdmarx->Instance->CDAR;
    __COMPILER_BARRIER();
    if ((dma_write_ptr > (uint32_t)(void *)rx_buf_read_ptr) || (rx_buf_write_wrap_flag != FALSE))
    {
        has_data = TRUE;
    }
    UTILS_EXIT_CRITICAL_SECTION();

    return has_data;
}
#endif /* (UART_IO_RX_TIMEOUT_MS > 0) */

/*----------------------------------------------------------------------------*/

void uart_io_init(void)
{
#if UART_IO_RX_ENABLED
    HAL_StatusTypeDef status;

    if (NULL == LOG_UART_HANDLER.hdmarx)
    {
        /* This module uses DMA for UART Rx, we can't proceed without an assigned DMA channel */
        SID_PAL_LOG_ERROR("No DMA channel assigned for UART Rx");
        Error_Handler();
    }

    /* Configure DMA ring buffer for continuous Rx */
    LOG_UART_HANDLER.RxCpltCallback = uart_io_rx_buffer_wrap;
    rx_buf_write_wrap_flag = FALSE;

    status = HAL_UART_Receive_DMA(&LOG_UART_HANDLER, (uint8_t *)uart_rx_buf, sizeof(uart_rx_buf));
    if (status != HAL_OK)
    {
        SID_PAL_LOG_ERROR("Failed to start UART Rx. HAL error 0x%02X", status);
        Error_Handler();
    }
#endif /* UART_IO_RX_ENABLED */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int puts(const char *s)
{
    int ret = EOF;
    const uint16_t length = (uint16_t)(strlen(s) * sizeof(char));

#if UART_IO_USE_UTIL_ADV_TRACE
    UTIL_ADV_TRACE_Status_t status;

    status = UTIL_ADV_TRACE_Send((const uint8_t*)s, length);
    if (UTIL_ADV_TRACE_OK == status)
    {
        ret = 0;
    }
#else
    HAL_StatusTypeDef status;

    status = HAL_UART_Transmit(&LOG_UART_HANDLER, (const uint8_t*)s, length, UART_IO_TX_TIMEOUT_MS);
    if (HAL_OK == status)
    {
        ret = 0;
    }
#endif /* UART_IO_USE_UTIL_ADV_TRACE */

    return ret;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int putchar(int x)
{
    int ret = EOF;
    const char character = (char) x;

#if UART_IO_USE_UTIL_ADV_TRACE
    UTIL_ADV_TRACE_Status_t status;

    status = UTIL_ADV_TRACE_Send((const uint8_t*)&character, sizeof(character));
    if (UTIL_ADV_TRACE_OK == status)
    {
        ret = 0;
    }
#else
    HAL_StatusTypeDef status;

    status = HAL_UART_Transmit(&LOG_UART_HANDLER, (const uint8_t*)&character, sizeof(character), UART_IO_TX_TIMEOUT_MS);
    if (HAL_OK == status)
    {
        ret = (int) x;
    }
#endif /* UART_IO_USE_UTIL_ADV_TRACE */

    return ret;
}

/*----------------------------------------------------------------------------*/

#if UART_IO_RX_ENABLED
SID_STM32_SPEED_OPTIMIZED int getchar(void)
{
    int ret = EOF;

#if (UART_IO_RX_TIMEOUT_MS > 0)
    if (SID_STM32_UTIL_IS_IRQ() == FALSE)
    {
        const uint32_t poll_period_ticks = UART_IO_MS_TO_OS_TICKS(UART_IO_RX_POLL_PERIOD_MS);
        const uint32_t timeout_ticks     = UART_IO_MS_TO_OS_TICKS(UART_IO_RX_TIMEOUT_MS);
        uint32_t accumulated_ticks       = 0u;

        /**
         * If this method was called from a task context we can wait for the char to arrive. Unfortunately we can't use semaphores here because
         * interrupt is generated only on DMA buffer wrap event, not on every successful byte reception. So we are limited to polling here
         */
        while ((uart_io_rx_buffer_has_data() == FALSE) && (accumulated_ticks < timeout_ticks))
        {
            osDelay(poll_period_ticks);
            accumulated_ticks += poll_period_ticks;
        }
    }
#endif /* (UART_IO_RX_TIMEOUT_MS > 0) */

    UTILS_ENTER_CRITICAL_SECTION();

    register uint32_t dma_write_ptr = LOG_UART_HANDLER.hdmarx->Instance->CDAR;
    __COMPILER_BARRIER();

    do
    {
        /* Check if DMA IRQ is pending */
        if (__HAL_DMA_GET_FLAG(LOG_UART_HANDLER.hdmarx , DMA_FLAG_TC) != 0U)
        {
            /* Let the Transfer Complete IRQ to be processed before proceeding */
            break;
        }

        /* Check for buffer overflow */
        if ((dma_write_ptr > (uint32_t)(void *)rx_buf_read_ptr) && (rx_buf_write_wrap_flag != FALSE))
        {
            /* Buffer overflow, DMA wrapped around and writes on top of read space */
            SID_PAL_LOG_ERROR("UART CLI Rx buffer overrun");
            SID_PAL_ASSERT(0);
        }

        /* Check if we have a new char(s) in the ring buffer */
        if ((dma_write_ptr > (uint32_t)(void *)rx_buf_read_ptr) || (rx_buf_write_wrap_flag != FALSE))
        {
            /* Get char from buffer */
            ret = (int)(*rx_buf_read_ptr);

            /* Advance the read pointer */
            rx_buf_read_ptr++;
            if (rx_buf_read_ptr >= (uart_rx_buf + SID_STM32_UTIL_ARRAY_SIZE(uart_rx_buf)))
            {
                /* Wrap the read pointer */
                rx_buf_read_ptr = &uart_rx_buf[0];

                /* Clear the buffer write wrap flag on read pointer wrap */
                rx_buf_write_wrap_flag = FALSE;
            }
        }
    } while (0);

    UTILS_EXIT_CRITICAL_SECTION();

    return ret;
}
#endif /* UART_IO_RX_ENABLED */

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED int _write_r(void *reent, int fd, char *s, size_t n)
{
    (void) reent;
    (void) fd;
    int ret = EOF;

#if UART_IO_USE_UTIL_ADV_TRACE
    UTIL_ADV_TRACE_Status_t status;

    status = UTIL_ADV_TRACE_Send((const uint8_t*)s, n);
    if (UTIL_ADV_TRACE_OK == status)
    {
        ret = (int)n;
    }
#else
    HAL_StatusTypeDef status;

    status = HAL_UART_Transmit(&LOG_UART_HANDLER, (const uint8_t*)s, n, UART_IO_TX_TIMEOUT_MS);
    if (HAL_OK == status)
    {
        ret = (int)n;
    }
#endif /* UART_IO_USE_UTIL_ADV_TRACE */

    return ret;
}
