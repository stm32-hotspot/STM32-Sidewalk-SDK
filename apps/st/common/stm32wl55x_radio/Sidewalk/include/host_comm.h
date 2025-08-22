/**
  ******************************************************************************
  * @file    host_comm.h
  * @brief   Sidewalk STM32WLxx Radio App's host MCU communication module
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

#ifndef __SID_HOST_COMM_H_
#define __SID_HOST_COMM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "stm32wlxx.h"
#include "sys_app.h"
#include "sys_conf.h"

#if defined(NUCLEO_WL55_BOARD)
#  include <stm32wlxx_nucleo.h>
#endif /* NUCLEO_WL55_BOARD */

/* Exported types ------------------------------------------------------------*/

typedef enum {
    SID_HOST_COMM_ERROR_NONE           = 0, /*!< No errors */
    SID_HOST_COMM_ERROR_GENERIC        = 1, /*!< Generic error of unknown nature */
    SID_HOST_COMM_INVALID_ARGS         = 2, /*!< Invalid argument ssupplied by the caller function */
    SID_HOST_COMM_ERROR_EXCEEDS_MTU    = 3, /*!< The amount of data requested to be sent exceeds SPI protocol MTU */
    SID_HOST_COMM_ERROR_TX_FIFO_FULL   = 4, /*!< No free space left in SPI driver's Tx FIFO, cannot enqueue the data to be sent */
    SID_HOST_COMM_ERROR_HARDWARE       = 5, /*!< Underlying hardware/driver reported an error */
    SID_HOST_COMM_ERROR_RESOURCE_ALLOC = 6, /*!< Failed to create or allocate a resource (e.g. Task, Semaphore, etc.) */
    SID_HOST_COMM_ERROR_INVALID_PARAMS = 7, /*!< A method called by the Host Comm module failed because Host Comm provided invalid parameters when calling the function */
    SID_HOST_COMM_ERROR_NOT_SUPPORTED  = 8, /*!< Invoked method is not supported */
    SID_HOST_COMM_ERROR_INVALID_STATE  = 9, /*!< Invalid logical state */
} sid_host_comm_error_t;

typedef void (*sid_host_comm_on_incoming_user_data)(const uint8_t * const data, const uint32_t data_len);

typedef struct {
    uint8_t * data_ptr;  /*!< Pointer to the data buffer to be sent. The pointer shall remain valid until the outbound message is processed */
    uint32_t  data_len;  /*!< Length of the data to be sent to the host MCU side */
    uint32_t  auto_free; /*!< Set to zero if data_ptr points to a static buffer. Set to non-zero value to automatically call free() after the message is sent */
} sid_host_comm_out_udt_msg_desc_t;

/* Exported constants --------------------------------------------------------*/

#define SID_HOST_COMM_IRQ_PIN_SETTLE_TIME_US (5u) /*!< Since the IRQ line is open-drain with weak pull-up on the host side, some delay is required after setting pin high to allow it to charge the line and reach high logic level */

#ifndef SID_HOST_COMM_PROCESSING_DELAY_PROFILING
#  define SID_HOST_COMM_PROCESSING_DELAY_PROFILING 0 /*!< Set this to non-zero value to enable additional GPIO indications for Sidewalk processing delays profiling */
#endif

#ifndef STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
#  define STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER 0 /*!< Setting this macro to non-zero value enables User Data Transfer (UDT) extension in the radio driver */
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

#if SID_HOST_COMM_PROCESSING_DELAY_PROFILING
#  define SID_PDP_RADIO_IRQ_ACTIVITY_GPIO_Port (GPIOB)
#  define SID_PDP_RADIO_IRQ_ACTIVITY_Pin       (GPIO_PIN_5)
#  define SID_PDP_HOST_COMM_ACTIVITY_GPIO_Port (GPIOB)
#  define SID_PDP_HOST_COMM_ACTIVITY_Pin       (GPIO_PIN_13)
#endif /* SID_HOST_COMM_PROCESSING_DELAY_PROFILING */

/* Exported macro ------------------------------------------------------------*/

#define SID_HOST_COMM_IRQ_ENABLE_TRIGGER()   do\
                                             {\
                                                 __HAL_GPIO_EXTI_CLEAR_IT(SIDEWALK_RADIO_SPI_IRQ_Pin);\
                                                 SET_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_IRQ_Pin);\
                                             } while (0)
#define SID_HOST_COMM_IRQ_DISABLE_TRIGGER()  do\
                                             {\
                                                 CLEAR_BIT(EXTI->FTSR1, SIDEWALK_RADIO_SPI_IRQ_Pin);\
                                                 __HAL_GPIO_EXTI_CLEAR_IT(SIDEWALK_RADIO_SPI_IRQ_Pin);\
                                             } while (0)

/* Exported functions --------------------------------------------------------*/

__STATIC_FORCEINLINE void SID_HOST_COMM_IRQ_INDICATE_EVENT(void)
{
    SIDEWALK_RADIO_SPI_IRQ_GPIO_Port->BRR = SIDEWALK_RADIO_SPI_IRQ_Pin;
#if defined (LOW_POWER_DISABLE) && (LOW_POWER_DISABLE == 0)
    /* Prohibit the system to enter LPM until IRQ is acknowledged */
    extern void sid_radio_tmp_lpm_prohibit(void);
    sid_radio_tmp_lpm_prohibit();
#endif /* LOW_POWER_DISABLE */
    __COMPILER_BARRIER();

#if CFG_LED_SUPPORTED
    BSP_LED_On(LED_BLUE);
#endif /* CFG_LED_SUPPORTED */
}

/*----------------------------------------------------------------------------*/

__STATIC_FORCEINLINE void SID_HOST_COMM_IRQ_CLEAR_EVENT(void)
{
    SIDEWALK_RADIO_SPI_IRQ_GPIO_Port->BSRR = SIDEWALK_RADIO_SPI_IRQ_Pin;
    __COMPILER_BARRIER();
    sid_pal_delay_us(SID_HOST_COMM_IRQ_PIN_SETTLE_TIME_US);

#if CFG_LED_SUPPORTED
    BSP_LED_Off(LED_BLUE);
#endif /* CFG_LED_SUPPORTED */
}

/*----------------------------------------------------------------------------*/

/* FIXME: add description to functions */
void sid_host_comm_notify_handshake_requested(void);
sid_host_comm_error_t sid_host_comm_init(void);
void sid_host_comm_on_spi_frame_received(void);
sid_host_comm_error_t sid_host_comm_enqueue_tx(const uint8_t * const data, const uint32_t data_size);
void host_comm_notify_subghz_irq_end(void);

#if STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER
/**
 * @brief User-defined function called during the initialization phase. Can be used to setup UDT as per the user app needs
 *
 * @returns Non-zero value in case of error
 */
sid_host_comm_error_t sid_host_comm_udt_user_init(void);
#endif /* STM32WLXX_EXT_IFC_ENABLE_USER_DATA_TRANSFER */

/**
 * @brief Set the user callback to invoke when UDT data arrives from the host MCU side
 *
 * @param [in] callback Pointer to the user-defined callback function. Can be set to NULL.
 * @returns             Non-zero value in case of error
 */
sid_host_comm_error_t sid_host_comm_set_user_data_received_cb(sid_host_comm_on_incoming_user_data callback);

/**
 * @brief Enqueues user data for sending to the host MCU side
 *
 * @note Supplied data must remain valid until sent by the radio. The driver does not make any copy of the supplied data buffer
 * 
 * @param [in] data      Pointer to the data buffer
 * @param [in] data_len  Length of the data buffer
 * @param [in] auto_free If set to non-zero value a free() call will be made to automatically free the data buffer after send out. This option is useful for dynamically allocated buffers
 * @returns              Non-zero value in case of error
 */
sid_host_comm_error_t sid_host_comm_send_user_data(const uint8_t * const data, const uint32_t data_len, const uint32_t auto_free);

#ifdef __cplusplus
}
#endif

#endif /* __SID_HOST_COMM_H_ */
