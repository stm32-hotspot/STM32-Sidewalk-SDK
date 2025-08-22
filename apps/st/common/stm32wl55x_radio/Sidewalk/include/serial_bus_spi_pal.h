/**
  ******************************************************************************
  * @file    serial_bus_spi_pal.h
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

#ifndef SERIAL_BUS_SPI_PAL_H
#define SERIAL_BUS_SPI_PAL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/

#include "stm32wlxx_hal.h"

/* Exported constants --------------------------------------------------------*/

#define SERIAL_BUS_SPI_PAL_FIFO_LEVEL_OVERRUN             (0xFFFFFFFFu)

#define SERIAL_BUS_SPI_PAL_NSS_ACTIVATED_GPIO_PIN_STATE   (GPIO_PIN_RESET)
#define SERIAL_BUS_SPI_PAL_NSS_DEACTIVATED_GPIO_PIN_STATE (GPIO_PIN_SET)

/* Exported types ------------------------------------------------------------*/

typedef enum 
{
    SERIAL_BUS_SPI_PAL_OK                  =  0, /*!< Everything is ok */
    SERIAL_BUS_SPI_PAL_ERROR               =  1, /*!< Generic error, typically means systematic software failure */
    SERIAL_BUS_SPI_PAL_INVALID_ARGS        =  2, /*!< Invalid arguments supplied by a function caller (e.g. null pointer, zero-length buffer, etc.) */ 
    SERIAL_BUS_SPI_PAL_TIMEOUT             =  3, /*!< Timeout on waiting for the requested operation to complete */
    SERIAL_BUS_SPI_PAL_ALIGNMENT_ERROR     =  4, /*!< RAM alignment error or SPI frame size and data size mismatch */
    SERIAL_BUS_SPI_PAL_NO_FREE_SPACE       =  5, /*!< No free space in the SPI Tx buffer, cannot add new data to it */
    SERIAL_BUS_SPI_PAL_FIFO_OVERRUN        =  6, /*!< Overrun detected in the SPI Rx buffer, received data may be corrupted */
    SERIAL_BUS_SPI_PAL_NOT_INITIALIZED     =  7, /*!< Requesting SPI operations when SPI is not initialized yet */
    SERIAL_BUS_SPI_PAL_ALREADY_INITIALIZED =  8, /*!< Requesting SPI initialization when SPI is initialized already */
    SERIAL_BUS_SPI_PAL_INVALID_STATE       =  9, /*!< Invalid logical state of the SPI driver and/or of the used peripherals */
    SERIAL_BUS_SPI_PAL_HARDWARE_ERROR      = 10, /*!< Some hardware fault detected, e.g. writing to the peripheral registers has failed or the peripheral(s) rejected an operation */
    SERIAL_BUS_SPI_PAL_TRY_LATER           = 11, /*!< The requested operation cannot be completed now, but this is a recoverable error and it is safe to retry the operation a bit later */
} serial_bus_spi_pal_err_t; 

typedef struct 
{
    uint32_t error_code;
    uint32_t spi_sr;
    uint32_t dma_tx_ccr; 
    uint32_t dma_rx_ccr;
    uint32_t dma_nss_dtct_ccr;
    uint32_t dma_sck_ctrl_ccr;
} serial_bus_spi_pal_bus_error_details_t;


/**
 * @brief User callback to be triggered whenever there's some new data available in SPI's Rx FIFO ring buffer
 */
typedef void (*serial_bus_spi_pal_rx_data_available_cb_t)(void);

/**
 * @brief User callback to be triggered whenever some kind of SPI (or related DMA channels) error occurred
 */
typedef void (*serial_bus_spi_pal_error_cb_t) (const serial_bus_spi_pal_bus_error_details_t * const);

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Initializes the Serial Peripheral Interface (SPI) driver.
 *
 * This function sets up the SPI driver by configuring the SPI peripheral, initializing
 * the internal buffer, and associating the provided callback functions.
 *
 * @param[in] rx_data_available_callback Callback function to be invoked upon successful SPI reception.
 * @param[in] error_callback             Callback function to handle SPI transfer errors.
 *
 * @return #serial_bus_spi_pal_err_t indicating the initialization status.
 */
serial_bus_spi_pal_err_t sid_pal_serial_bus_spi_init(serial_bus_spi_pal_rx_data_available_cb_t rx_data_available_callback,
                                                     serial_bus_spi_pal_error_cb_t error_callback);

/**
 * @brief Initiates a Serial Peripheral Interface (SPI) data transfer.
 *
 * This function starts an SPI data transfer. It takes no parameters as SPI interface
 * uses fixed-size frames and Tx & Rx FIFO buffers are arranged internally.
 * Use @ref serial_bus_spi_enqueue_tx to send out data and @ref serial_bus_spi_pop_from_rx_fifo
 * to extract already received data
 *
 * @return @ref serial_bus_spi_pal_err_t indicating the transfer status.
 */
serial_bus_spi_pal_err_t serial_bus_spi_xfer_start(void);

/**
 * @brief Performs full reinitialization of the SPI, including resetting the SPI peripheral
 * and reconfiguring it and the related DMA channels.
 * 
 * @note The user must explicitly call @ref serial_bus_spi_xfer_start after performing reinitialization
 */
serial_bus_spi_pal_err_t serial_bus_spi_full_reinit(void);

/**
 * @brief Clears both Tx and Rx ring buffers by discarding any data in them.
 * 
 *  If an active SPI transaction is ongoing this method will wait for the transaction to end before clearing the buffers
 *
 * @param[in] timeout_ms Timeout in milliseconds to wait for any ongoing transaction to end. Can be set to @ref osWaitForever to disable the timeout
 */
serial_bus_spi_pal_err_t serial_bus_spi_clear_fifos(const uint32_t timeout_ms);

/**
 * @brief Gets the number of the free SPI data frame slots in the SPI Tx FIFO
 * @return The number of the SPI data frames that can be put into the SPI Tx FIFO immediately
 */
uint32_t serial_bus_spi_get_tx_fifo_free_level(void);

/**
 * @brief Gets the number of the available SPI frames in the SPI Rx FIFO
 * @return The number of the available SPI frames in SPI FIFO or UINT32_MAX if FIFO overrun was detected
 */
uint32_t serial_bus_spi_get_rx_fifo_level(void);

/**
 * @brief Reads and pops a specified number of bytes from the Serial Peripheral Interface (SPI) receive buffer.
 *
 * This function reads a specified number of bytes and copies them into the provided buffer.
 * It then frees the corresponding space in RX buffer. If the requested number of bytes is not available, it reads and
 * frees the available bytes.
 *
 * @param[out] buff       Pointer to the destination buffer for storing the received data.
 * @param[in]  read_bytes Number of bytes to read from the receive buffer.
 *
 * @return The actual number of bytes copied from the receive buffer and freed.
 */
serial_bus_spi_pal_err_t serial_bus_spi_pop_from_rx_fifo(uint8_t * const buf, const uint32_t buf_size, uint32_t * const bytes_copied);

/**
 * @brief Adds data to the SPI Tx FIFO.
 * 
 * Supplied data is automatically divided into separate SPI frames. If data length is not a multiple of the
 * SPI frame size padding is used to complete the last SPI frame. While the padding bytes can be ignored they
 * are still covered by CRC checks.
 * 
 * @param data   Pointer to the buffer to be sent. Cannot be NULL
 * @param length Length of the data to be sent. Must be greater than zero
 * @return @ref serial_bus_spi_pal_err_t indicating the operation result 
*/
serial_bus_spi_pal_err_t serial_bus_spi_enqueue_tx(const uint8_t * const data, const uint32_t length);

/**
 * @brief Gets the direct pointer to the current Tx FIFO ingest location
 * 
 * @warning DANGER ZONE. EXERCISE EXTREME CAUTION WHEN USING THIS API.
 *          The only safe way to use it is to make sure that no active SPI transfers are currently running
 *
 * @note This API is intended to be used to modify the contents of the SPI data after it was enqueued
 * into the FIFO (e.g. setting a timestamp in the Tx frame after enqueue and right before the actual Tx start)
 */
void * serial_bus_spi_get_current_enqueue_ptr(void);

/**
 * @brief A callback triggered after a full SPI frame was successfully received
 */
void serial_bus_spi_on_frame_received(void);

void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_BUS_SPI_PAL_H */
