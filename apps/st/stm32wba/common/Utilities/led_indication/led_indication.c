/**
  ******************************************************************************
  * @file           : led_indication.c
  * @brief          : STM32WBA5x-specific Sidewalk states indication using LEDs
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

#include "led_indication.h"

#include <cmsis_os2.h>
#include <sid_stm32_common_utils.h>
#if defined(NUCLEO_WBA52_BOARD) || defined(NUCLEO_WBA55_BOARD) || defined(NUCLEO_WBA65_BOARD)
#  include <stm32wbaxx_nucleo.h>
#endif

/* Private define ------------------------------------------------------------*/

#if !defined(NUCLEO_WBA52_BOARD) && !defined(NUCLEO_WBA55_BOARD) && !defined(NUCLEO_WBA65_BOARD)
#  error "This module supports only NUCLEO-WBA52, NUCLEO-WBA55, and NUCLEO-WBA65 boards. Please refine the implementation for your board or turn off the LED indication."
#endif

#define LED_INDICATION_TASK_STACK_SIZE       (96u*4u)
#define LED_INDICATION_TASK_PRIO             ((osPriority_t) osPriorityLow)
#define LED_INDICATION_MSG_QUEUE_LEN         (4u)

#define LED_INDICATION_KEEP_FOREVER          (0xFFFFu)

/* Private types -------------------------------------------------------------*/

typedef struct {
    uint16_t    on_time;           /*!< Amount of OS ticks to keep the LED on. Can be LED_INDICATION_KEEP_FOREVER to keep the LED on until the next pattern arrives. Both on_time and of_time cannot be LED_INDICATION_KEEP_FOREVER simultaneously */
    uint16_t    off_time;          /*!< Amount of OS ticks to keep the LED off. Can be LED_INDICATION_KEEP_FOREVER to keep the LED on until the next pattern arrives. */
    uint16_t    separation_before; /*!< Amount of OS ticks to keep all the LED off before the patter starts - can be used for visual separation of the patterns */
    uint16_t    separation_after;  /*!< Amount of OS ticks to keep all the LED off after the patter ends - can be used for visual separation of the patterns */
    uint16_t    repetitions;       /*!< Number of pattern repetitions (blinks). A repetition is started from On cycle followed by the Off cycle. Setting this to zero will result in infinite repetitions (until the next pattern is requested) */
    Led_TypeDef led;               /*!< Target LED to be operated by current pattern */
} led_indication_pattern_t;

/* Private constants ---------------------------------------------------------*/

static const osMessageQueueAttr_t LED_Indication_Queue_Attr = {
    .name = "LED_Ind_Queue",
};

static const osThreadAttr_t LED_Indication_Thread_Attr = {
  .name       = "LED_Ind_Thread",
  .priority   = LED_INDICATION_TASK_PRIO,
  .stack_size = LED_INDICATION_TASK_STACK_SIZE,
};

static const led_indication_pattern_t leds_off_ind = {
    .on_time           = 0u,
    .off_time          = LED_INDICATION_KEEP_FOREVER,
    .separation_before = 0u,
    .separation_after  = 0u,
    .repetitions       = 0u,
    .led               = LED_GREEN,
};

static const led_indication_pattern_t idle_state_ind = {
    .on_time           = 1000u,
    .off_time          = 1000u,
    .separation_before = 0u,
    .separation_after  = 0u,
    .repetitions       = 0u,
    .led               = LED_GREEN,
};

static const led_indication_pattern_t bonding_state_ind = {
    .on_time           = 200u,
    .off_time          = 200u,
    .separation_before = 0u,
    .separation_after  = 0u,
    .repetitions       = 0u,
    .led               = LED_GREEN,
};

static const led_indication_pattern_t conn_established_ind = {
    .on_time           = 75u,
    .off_time          = 75u,
    .separation_before = 0u,
    .separation_after  = 0u,
    .repetitions       = 5u,
    .led               = LED_GREEN,
};

static const led_indication_pattern_t connected_ind = {
    .on_time           = LED_INDICATION_KEEP_FOREVER,
    .off_time          = 0u,
    .separation_before = 0u,
    .separation_after  = 0u,
    .repetitions       = 0u,
    .led               = LED_GREEN,
};

static const led_indication_pattern_t msg_tx_enqueued_ind = {
    .on_time           = 0u,
    .off_time          = 125u,
    .separation_before = 0u,
    .separation_after  = 0u,
    .repetitions       = 1u,
    .led               = LED_GREEN,
};

static const led_indication_pattern_t msg_tx_done_ind = {
    .on_time           = 125u,
    .off_time          = 125u,
    .separation_before = 125u,
    .separation_after  = 0u,
    .repetitions       = 2u,
    .led               = LED_GREEN,
};

static const led_indication_pattern_t msg_tx_error_ind = {
    .on_time           = 125u,
    .off_time          = 125u,
    .separation_before = 125u,
    .separation_after  = 0u,
    .repetitions       = 3u,
    .led               = LED_RED,
};

static const led_indication_pattern_t msg_rx_done_ind = {
    .on_time           = 125u,
    .off_time          = 125u,
    .separation_before = 125u,
    .separation_after  = 0u,
    .repetitions       = 1u,
    .led               = LED_GREEN,
};

static const led_indication_pattern_t sidewalk_error_ind = {
    .on_time           = 250u,
    .off_time          = 100u,
    .separation_before = 0u,
    .separation_after  = 0u,
    .repetitions       = 0u,
    .led               = LED_RED,
};

/* Private variables ---------------------------------------------------------*/

static osMessageQueueId_t indication_queue;
static osThreadId_t       indication_thread;

/* Private function prototypes -----------------------------------------------*/

static void _led_indication_task(void * context);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _led_indication_task(void * context)
{
    osStatus_t               os_err;
    const led_indication_pattern_t * current_pattern;
    const led_indication_pattern_t * last_known_steady_pattern = &leds_off_ind;

    (void)context;

    BSP_LED_Off(LED_GREEN);
    BSP_LED_Off(LED_RED);

    do
    {
        os_err = osMessageQueueGet(indication_queue, &current_pattern, NULL, osWaitForever);
        if ((os_err & osFlagsError) != 0u)
        {
            /* Error while getting queue message - skip and continue */
            continue;
        }

process_pattern:
        if ((LED_INDICATION_KEEP_FOREVER == current_pattern->on_time) && (LED_INDICATION_KEEP_FOREVER == current_pattern->off_time))
        {
            /* This is invalid pattern, only one state can last for indefinite amount of time - skip */
            continue;
        }

        /* Check if the new pattern is considered steady - this state will be kept till the new steady pattern arrives */
        if ((0u == current_pattern->repetitions) || (LED_INDICATION_KEEP_FOREVER == current_pattern->on_time) || (LED_INDICATION_KEEP_FOREVER == current_pattern->off_time))
        {
            last_known_steady_pattern = current_pattern;
        }

        /* Turn off the LEDs */
        BSP_LED_Off(LED_GREEN);
        BSP_LED_Off(LED_RED);

        /* Blackout time before the pattern start */
        if (current_pattern->separation_before > 0u)
        {
            osDelay(current_pattern->separation_before);
        }

        uint32_t repetitions_done = 0u;
        do
        {
            /* On cycle */
            if (current_pattern->on_time != LED_INDICATION_KEEP_FOREVER)
            {
                if (current_pattern->on_time > 0u)
                {
                    BSP_LED_On(current_pattern->led);
                    osDelay(current_pattern->on_time);
                }
            }
            else
            {
                BSP_LED_On(current_pattern->led);
                /* Wait indefinitely until new pattern is requested */
                os_err = osMessageQueueGet(indication_queue, &current_pattern, NULL, osWaitForever);
                if ((os_err & osFlagsError) == 0u)
                {
                    /* New pattern successfully extracted, proceed with the update */
                    BSP_LED_Off(LED_GREEN);
                    BSP_LED_Off(LED_RED);

                    /* Blackout time after the pattern end */
                    if (current_pattern->separation_after > 0u)
                    {
                        osDelay(current_pattern->separation_after);
                    }
                }
                goto process_pattern;
            }

            /* Off cycle */
            if (current_pattern->off_time != LED_INDICATION_KEEP_FOREVER)
            {
                if (current_pattern->off_time > 0u)
                {
                    BSP_LED_Off(current_pattern->led);
                    osDelay(current_pattern->off_time);
                }
            }
            else
            {
                BSP_LED_Off(current_pattern->led);
                /* Wait indefinitely until new pattern is requested */
                os_err = osMessageQueueGet(indication_queue, &current_pattern, NULL, osWaitForever);
                if ((os_err & osFlagsError) == 0u)
                {
                    /* New pattern successfully extracted, proceed with the update */
                    BSP_LED_Off(LED_GREEN);
                    BSP_LED_Off(LED_RED);

                    /* Blackout time after the pattern end */
                    if (current_pattern->separation_after > 0u)
                    {
                        osDelay(current_pattern->separation_after);
                    }
                }
                goto process_pattern;
            }

            if (0u == current_pattern->repetitions)
            {
                /* This pattern shall last as long as there's no new indication requests */
                if (osMessageQueueGetCount(indication_queue) > 0u)
                {
                    break;
                }
            }
            else
            {
                /* Increment repetitions counter */
                repetitions_done++;
            }
        } while ((0u == current_pattern->repetitions) || (repetitions_done < current_pattern->repetitions));

        BSP_LED_Off(LED_GREEN);
        BSP_LED_Off(LED_RED);

        /* Blackout time after the pattern end */
        if (current_pattern->separation_after > 0u)
        {
            osDelay(current_pattern->separation_after);
        }

        /* If there's no new pattern pending switch back to the last known steady pattern */
        if (osMessageQueueGetCount(indication_queue) == 0u)
        {
            current_pattern = last_known_steady_pattern;
            goto process_pattern;
        }
    } while (TRUE);

    /* Terminate properly if the code ever gets here */
    osThreadExit();
}

/* Global function definitions -----------------------------------------------*/

sid_error_t led_indication_init(void)
{
    sid_error_t status = SID_ERROR_GENERIC;

    do
    {
        indication_queue = osMessageQueueNew(LED_INDICATION_MSG_QUEUE_LEN, sizeof(led_indication_pattern_t), &LED_Indication_Queue_Attr);
        if (NULL == indication_queue)
        {
            status = SID_ERROR_OOM;
            break;
        }

        indication_thread = osThreadNew(_led_indication_task, NULL, &LED_Indication_Thread_Attr);
        if (NULL == indication_queue)
        {
            osMessageQueueDelete(indication_queue);
            indication_queue = NULL;
            status = SID_ERROR_OOM;
            break;
        }

        status = led_indication_set(LED_INDICATE_IDLE);
    } while (0);

    return status;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED sid_error_t led_indication_set(led_indication_t indicate)
{
    sid_error_t status = SID_ERROR_GENERIC;
    osStatus_t  os_err;
    const led_indication_pattern_t * pattern_ptr;

    switch (indicate)
    {
        case LED_INDICATE_IDLE:
            pattern_ptr = &idle_state_ind;
            os_err = osMessageQueuePut(indication_queue, &pattern_ptr, 0u, 0u);
            status = osOK == os_err ? SID_ERROR_NONE : SID_ERROR_OUT_OF_RESOURCES;
            break;

        case LED_INDICATE_BONDING:
            pattern_ptr = &bonding_state_ind;
            os_err = osMessageQueuePut(indication_queue, &pattern_ptr, 0u, 0u);
            status = osOK == os_err ? SID_ERROR_NONE : SID_ERROR_OUT_OF_RESOURCES;
            break;

        case LED_INDICATE_CONNECTED:
            pattern_ptr = &conn_established_ind;
            os_err = osMessageQueuePut(indication_queue, &pattern_ptr, 0u, 0u);
            if (osOK == os_err)
            {
                pattern_ptr = &connected_ind;
                os_err = osMessageQueuePut(indication_queue, &pattern_ptr, 0u, 0u);
            }
            status = osOK == os_err ? SID_ERROR_NONE : SID_ERROR_OUT_OF_RESOURCES;
            break;

        case LED_INDICATE_SEND_ENQUEUED:
            pattern_ptr = &msg_tx_enqueued_ind;
            os_err = osMessageQueuePut(indication_queue, &pattern_ptr, 0u, 0u);
            status = osOK == os_err ? SID_ERROR_NONE : SID_ERROR_OUT_OF_RESOURCES;
            break;

        case LED_INDICATE_SENT_OK:
            pattern_ptr = &msg_tx_done_ind;
            os_err = osMessageQueuePut(indication_queue, &pattern_ptr, 0u, 0u);
            status = osOK == os_err ? SID_ERROR_NONE : SID_ERROR_OUT_OF_RESOURCES;
            break;

        case LED_INDICATE_SEND_ERROR:
            pattern_ptr = &msg_tx_error_ind;
            os_err = osMessageQueuePut(indication_queue, &pattern_ptr, 0u, 0u);
            status = osOK == os_err ? SID_ERROR_NONE : SID_ERROR_OUT_OF_RESOURCES;
            break;

        case LED_INDICATE_RCV_OK:
            pattern_ptr = &msg_rx_done_ind;
            os_err = osMessageQueuePut(indication_queue, &pattern_ptr, 0u, 0u);
            status = osOK == os_err ? SID_ERROR_NONE : SID_ERROR_OUT_OF_RESOURCES;
            break;

        case LED_INDICATE_ERROR:
            pattern_ptr = &sidewalk_error_ind;
            os_err = osMessageQueuePut(indication_queue, &pattern_ptr, 0u, 0u);
            status = osOK == os_err ? SID_ERROR_NONE : SID_ERROR_OUT_OF_RESOURCES;
            break;

        case LED_INDICATE_OFF:
            pattern_ptr = &leds_off_ind;
            os_err = osMessageQueuePut(indication_queue, &pattern_ptr, 0u, 0u);
            status = osOK == os_err ? SID_ERROR_NONE : SID_ERROR_OUT_OF_RESOURCES;

        default:
            status = SID_ERROR_INVALID_ARGS;
            break;
    }

    return status;
}
