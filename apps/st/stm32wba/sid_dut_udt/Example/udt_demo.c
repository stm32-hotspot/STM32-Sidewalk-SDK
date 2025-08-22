/**
  ******************************************************************************
  * @file    udt_demo.c
  * @brief   Demo of the UDT feature of STM32WLxx Radio App
  * 
  * This module showcases the User Data Transfer (UDT) feature. It allows
  * transferring of the user data over the SPI bus concurrently with Sidewalk
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

/* Includes ------------------------------------------------------------------*/

#include "host_comm.h"
#include "sid_pal_log_like.h"

#include <cmsis_os2.h>
#include <sid_stm32_common_utils.h>
#include <FreeRTOS.h>

/* Private defines -----------------------------------------------------------*/

#define UDT_DEMO_INBOUND_MSG_QUEUE_LEN  (5u)
#define UDT_DEMO_INBOUND_MSG_LEN_MAX    (160u)

/* Private typedef -----------------------------------------------------------*/

typedef struct {
    uint8_t * data_ptr;  /*!< Pointer to the data buffer to be sent. The pointer shall remain valid until the outbound message is processed */
    uint32_t  data_len;  /*!< Length of the data to be sent to the host MCU side */
} udt_demo_inbound_msg_desc_t;

/* Private variables ---------------------------------------------------------*/

static osMessageQueueId_t inbound_msg_queue = NULL;
static osThreadId_t       udt_demo_task = NULL;

/* Private constants ---------------------------------------------------------*/

static const osThreadAttr_t udt_demo_processing_threadattributes = {
    .name       = "UDT_PROC",
    .priority   = ((osPriority_t) osPriorityNormal),
    .stack_size = (1024u),
};

/* Private function prototypes -----------------------------------------------*/

static void _on_incoming_user_data_cb(const uint8_t * const data, const uint32_t data_len);
static void _udt_demo_processing_thread(void * context);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static void _on_incoming_user_data_cb(const uint8_t * const data, const uint32_t data_len)
{
    osStatus_t os_err;

    /* This callback is triggered from the radio driver directly. Don't do any lengthy processing here and do not use delays, semaphores, etc. */

    if ((data != NULL) && (data_len > 0u))
    {
        /* Store the received data into a static buffer - original data is allocated on stack and may get corrupted/overwritten by the moment UART printout is processed */
        uint32_t len_to_copy = data_len > UDT_DEMO_INBOUND_MSG_LEN_MAX ? UDT_DEMO_INBOUND_MSG_LEN_MAX : data_len;

        uint8_t * data_buf = pvPortMalloc(len_to_copy);
        if (NULL == data_buf)
        {
            SID_PAL_LOG_ERROR("UDT Demo - out of memory");
            return;
        }

        SID_STM32_UTIL_fast_memcpy(data_buf, data, len_to_copy);

        /* Put received data into the processing queue of this demo */
        udt_demo_inbound_msg_desc_t msg = {
            .data_ptr = data_buf,
            .data_len = len_to_copy,
        };
        os_err = osMessageQueuePut(inbound_msg_queue, &msg, 0u, 0u);
        if (os_err != osOK)
        {
            /* Failed to put the message in the queue, probably it is full */
            SID_PAL_LOG_ERROR("UDT Demo - unable to enqueue inbound data");
        }
    }
}

/*----------------------------------------------------------------------------*/

static void _udt_demo_processing_thread(void * context)
{
    (void)context;

    do
    {
        osStatus_t os_err;
        assert_param(inbound_msg_queue != NULL);

        /* Wait for any inbound data from the WBA side */
        udt_demo_inbound_msg_desc_t inbound_msg_desc;
        os_err = osMessageQueueGet(inbound_msg_queue, &inbound_msg_desc, NULL, osWaitForever);

        if (osOK == os_err)
        {
            if ((inbound_msg_desc.data_ptr != NULL) && (inbound_msg_desc.data_len > 0u))
            {
                /* Non-empty data received from the WBA side */
                sid_host_comm_error_t hc_err;

                /* Ensure the demo string is always terminated properly */
                inbound_msg_desc.data_ptr[inbound_msg_desc.data_len - 1] = '\0';

                /* Print out received data */
                SID_PAL_LOG_INFO("%s", (char *)inbound_msg_desc.data_ptr);

                /* Modify "ping" to "pong" */
                inbound_msg_desc.data_ptr[5] = 'o';

                /* Send out UDT reply */
                hc_err = sid_host_comm_send_user_data(inbound_msg_desc.data_ptr, inbound_msg_desc.data_len, TRUE);
                if (hc_err != SID_HOST_COMM_ERROR_NONE)
                {
                    SID_PAL_LOG_ERROR("Failed to send UDT reply. Error %u", (uint32_t)hc_err);
                    vPortFree(inbound_msg_desc.data_ptr);
                }
                else
                {
                    /* Keep the buffer, it will be de-allocated automatically upon send out */
                }
            }
        }
    } while (TRUE);

    /* Terminate the thread properly if the code ever gets here */
    SID_PAL_LOG_WARNING("UDT Demo thread terminated");
    osThreadExit();
}

/* Global function definitions -----------------------------------------------*/

sid_host_comm_error_t sid_host_comm_udt_user_init(void)
{
    sid_host_comm_error_t err = SID_HOST_COMM_ERROR_GENERIC;

    do
    {
        /* Create inbound message queue */
        if (NULL == inbound_msg_queue)
        {
            inbound_msg_queue = osMessageQueueNew(UDT_DEMO_INBOUND_MSG_QUEUE_LEN, sizeof(udt_demo_inbound_msg_desc_t), NULL);
            if (NULL == inbound_msg_queue)
            {
                SID_PAL_LOG_ERROR("Can't create UDT Demo inbound message queue. No memory");
                err = SID_HOST_COMM_ERROR_RESOURCE_ALLOC;
                break;
            }
        }

        /* Create the task that will handle the inbound messages */
        if (NULL == udt_demo_task)
        {
            udt_demo_task = osThreadNew(_udt_demo_processing_thread, NULL, &udt_demo_processing_threadattributes);
            if (NULL == udt_demo_task)
            {
                SID_PAL_LOG_ERROR("Can't create UDT demo thread. No memory");
                err = SID_HOST_COMM_ERROR_RESOURCE_ALLOC;
                break;
            }
        }

        /* Set a callback for incoming user data */
        err = sid_host_comm_set_user_data_received_cb(_on_incoming_user_data_cb);
        if (err != SID_HOST_COMM_ERROR_NONE)
        {
            SID_PAL_LOG_ERROR("Failed to register UDT callback");
            break;
        }

        SID_PAL_LOG_INFO("UDT Demo initialized");
        err = SID_HOST_COMM_ERROR_NONE;
    } while (0u);

    return err;
}
