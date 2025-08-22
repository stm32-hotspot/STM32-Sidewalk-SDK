/**
  ******************************************************************************
  * @file    sid_pal_serial_bus_spi_config.h
  * @brief   Platform-specific SPI configuration declarations for Sidewalk
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef SID_PAL_SERIAL_BUS_SPI_CONFIG_H
#define SID_PAL_SERIAL_BUS_SPI_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <sid_pal_serial_bus_ifc.h>
#include <stm32wbaxx.h>

/* Exported constants --------------------------------------------------------*/

#define SID_PAL_SERIAL_BUS_STM32WBAxx_CRC_NOT_USED (0u)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Convenience typedef for the Sidewalk SDK's struct sid_pal_serial_bus_factory
 * See @ref sid_pal_serial_bus_factory for more details
 */
typedef struct sid_pal_serial_bus_factory sid_pal_serial_bus_factory_t;

/**
 * @brief Convenience typedef for the Sidewalk SDK's struct sid_pal_serial_bus_client
 * See @ref sid_pal_serial_bus_client for more details
 */
typedef struct sid_pal_serial_bus_client sid_pal_serial_bus_client_t;

typedef enum {
  SID_PAL_SERIAL_BUS_STM32WBAxx_SPI_MODE_0 = 0,
  SID_PAL_SERIAL_BUS_STM32WBAxx_SPI_MODE_1 = 1,
  SID_PAL_SERIAL_BUS_STM32WBAxx_SPI_MODE_2 = 2,
  SID_PAL_SERIAL_BUS_STM32WBAxx_SPI_MODE_3 = 3,
} sid_pal_serial_bus_stm32wbaxx_spi_mode_t;

/**
 * @brief Configuration of the SPI factory. This configuration is applied on the
 * first bus creation and remains unchanged till the bus is destroyed
 */
typedef struct {
    /**
     * @brief Pointer to the SPI HAL peripheral to use
     */
    SPI_HandleTypeDef * const hspi;
    /**
     *  @brief Pointer to the generated MX_SPIx_Init() function
     */
    void (* const mx_spi_init)(void);

    /**
     * @brief Pointer to the LPTIM HAL peripheral to use
     */
    LPTIM_HandleTypeDef * const hlptim;
    /**
     * @brief Pointer to the generated MX_LPTIMx_Init() function
     */
    void (* const mx_lptim_init)(void);
} sid_pal_serial_bus_stm32wbaxx_factory_config_t;

typedef struct {
    /**
     * @brief CRC polynomial specification
     */
    uint32_t crc_polynomial;
    /**
     * @brief Specifies the CRC Length used for the CRC calculation.
     * This parameter can be a value of @ref SPI_CRC_length
     */
    uint32_t crc_length;
    /**
     * @brief Specifies the CRC initialization Pattern used for the CRC calculation.
     * This parameter can be a value of @ref SPI_CRC_Calculation_Initialization_Pattern
     */
    uint32_t crc_init_pattern;

    /**
     * @brief Delay in SPI bus clock cycles between activating the NSS line and the first SCK pulse
     */
    uint32_t nss_to_sck_cycles;
    /**
     * @brief Delay in SPI bus clock cycles between the last SCK pulse and deactivating the NSS line
     */
    uint32_t sck_to_nss_cycles;
    /**
     * @brief Delay in SPI bus clock cycles between two consecutive transactions
     */
    uint32_t nss_to_nss_cycles;
    /**
     * NSS: ‾‾|___________________________________________________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_______________________________
     * SCK: ______________________|‾‾|__|‾‾|______________________________________________________________|‾‾|__|‾‾|__
     *        ^ nss_to_sck_cycles ^           ^ sck_to_nss_cycles ^ nss_to_nss_cycles ^ nss_to_sck_cycles ^  
     */
} sid_pal_serial_bus_stm32wbaxx_transaction_config_t;

/* Exported functions prototypes ---------------------------------------------*/

sid_error_t sid_pal_serial_bus_stm32wbaxx_create(const struct sid_pal_serial_bus_iface **iface, const void *const context);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SID_PAL_SERIAL_BUS_SPI_CONFIG_H */
