/**
  ******************************************************************************
  * @file           : log.c
  * @brief          : Implementation of the Sidewalk's sid_pal_log module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023-2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

/* libc */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Sidewalk API */
#include <sid_log_control.h>
#include <sid_pal_assert_ifc.h>
#include <sid_pal_delay_ifc.h>
#include <sid_pal_log_ifc.h>

/* App-specific config */
#include <app_conf.h>

/* STM32 utils */
#include <sid_stm32_common_utils.h>

#ifndef SID_PAL_LOG_USE_STM32_SYSTIME
#  define SID_PAL_LOG_USE_STM32_SYSTIME (1u)
#endif

#if SID_PAL_LOG_USE_STM32_SYSTIME
    #include <stm32_systime.h>
#endif

/* Private typedef -----------------------------------------------------------*/

typedef struct severity_param {
    char *name;
    char *color;
} severity_param_t;

/* Private defines -----------------------------------------------------------*/

#ifndef SID_PAL_LOG_MSG_LENGTH_MAX
#  error "Maximum length of the log message is not defined. Please set SID_PAL_LOG_MSG_LENGTH_MAX compile definition"
#endif


#define REGULAR_COLOR_BLACK             "\e[0;30m"
#define REGULAR_COLOR_RED               "\e[0;31m"
#define REGULAR_COLOR_GREEN             "\e[0;32m"
#define REGULAR_COLOR_YELLOW            "\e[0;33m"
#define REGULAR_COLOR_BLUE              "\e[0;34m"
#define REGULAR_COLOR_MAGENTA           "\e[0;35m"
#define REGULAR_COLOR_CYAN              "\e[0;36m"
#define REGULAR_COLOR_WHITE             "\e[0;37m"

#define BOLD_COLOR_BLACK                "\e[1;30m"
#define BOLD_COLOR_RED                  "\e[1;31m"
#define BOLD_COLOR_GREEN                "\e[1;32m"
#define BOLD_COLOR_YELLOW               "\e[1;33m"
#define BOLD_COLOR_BLUE                 "\e[1;34m"
#define BOLD_COLOR_MAGENTA              "\e[1;35m"
#define BOLD_COLOR_CYAN                 "\e[1;36m"
#define BOLD_COLOR_WHITE                "\e[1;37m"

#define UNDERLINE_COLOR_BLACK           "\e[4;30m"
#define UNDERLINE_COLOR_RED             "\e[4;31m"
#define UNDERLINE_COLOR_GREEN           "\e[4;32m"
#define UNDERLINE_COLOR_YELLOW          "\e[4;33m"
#define UNDERLINE_COLOR_BLUE            "\e[4;34m"
#define UNDERLINE_COLOR_MAGENTA         "\e[4;35m"
#define UNDERLINE_COLOR_CYAN            "\e[4;36m"
#define UNDERLINE_COLOR_WHITE           "\e[4;37m"

#define COLOR_RESET                     "\e[0m"


#define LOG_ERROR_COLOR                 BOLD_COLOR_RED
#define LOG_WARNING_COLOR               BOLD_COLOR_YELLOW
#define LOG_INFO_COLOR                  BOLD_COLOR_GREEN
#define LOG_DEBUG_COLOR                 BOLD_COLOR_BLUE
#define LOG_UNDEFINED_COLOR             BOLD_COLOR_WHITE


#define LOG_SEVERITY_NAME_ERROR         "ERROR"
#define LOG_SEVERITY_NAME_WARNING       "WARNING"
#define LOG_SEVERITY_NAME_INFO          "INFO"
#define LOG_SEVERITY_NAME_DEBUG         "DEBUG"
#define LOG_SEVERITY_NAME_UNDEFINED     "UNDEFINED"


#define LOG_LINE_ENDING                 "\r\n"


#define STATIC_STR_LEN(x)               (sizeof((x)) - 1u)


#define LOG_BUFFER_ADDITIONS_SIZE       (STATIC_STR_LEN(REGULAR_COLOR_BLACK) + STATIC_STR_LEN(COLOR_RESET) + STATIC_STR_LEN(LOG_LINE_ENDING))
#define LOG_FLUSH_SLEEP_PERIOD_MS       (1u)

#if (!CFG_LOG_SUPPORTED && SID_PAL_LOG_ENABLED)
/* This is critical - Sidewalk will try to print out the logs while the app does not support logging */
#  error "Configuration mismatch: app config disables logging while Sidewalk keeps them enabled. Either enable logs on app level or disable logging in Sidewalk config"
#elif (CFG_LOG_SUPPORTED && !SID_PAL_LOG_ENABLED)
/* This is not critical, but may be misleading - the app enables logging, but Sidewalk stack will remain silent */
#  warning "Configuration mismatch: app config enables logging while Sidewalk keeps them disabled. If this is the desired config feel free to keep it"
#endif

/* Private variables ---------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

static int print_severity(sid_pal_log_severity_t severity, char * buf, const size_t size);
static int snprintf_like(char * buf, const size_t size, const char *fmt, ...);

/* Private function definitions ----------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline int print_severity(sid_pal_log_severity_t severity, char * buf, const size_t size)
{
    severity_param_t severity_param = {.color = BOLD_COLOR_WHITE, .name = LOG_SEVERITY_NAME_UNDEFINED};
    int print_len;

    switch (severity) 
    {
        case SID_PAL_LOG_SEVERITY_ERROR:
            severity_param.color = LOG_ERROR_COLOR;
            severity_param.name = LOG_SEVERITY_NAME_ERROR;
            break;

        case SID_PAL_LOG_SEVERITY_WARNING:
            severity_param.color = LOG_WARNING_COLOR;
            severity_param.name = LOG_SEVERITY_NAME_WARNING;
            break;

        case SID_PAL_LOG_SEVERITY_INFO:
            severity_param.color = LOG_INFO_COLOR;
            severity_param.name = LOG_SEVERITY_NAME_INFO;
            break;

        case SID_PAL_LOG_SEVERITY_DEBUG:
            severity_param.color = LOG_DEBUG_COLOR;
            severity_param.name = LOG_SEVERITY_NAME_DEBUG;
            break;

        default:
            severity_param.color = LOG_UNDEFINED_COLOR;
            severity_param.name = LOG_SEVERITY_NAME_UNDEFINED;
            break;
    }

    print_len = snprintf_like(buf, size, "%s[%s]%s: ", severity_param.color, severity_param.name, COLOR_RESET);
    return print_len;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED  static inline int snprintf_like(char * buf, const size_t size, const char *fmt, ...)
{
    va_list args;
    int print_len;

    va_start(args, fmt);
    print_len = UTIL_ADV_TRACE_VSNPRINTF(buf, size, fmt, args);
    va_end(args);

    return print_len;
}

/* Global function definitions -----------------------------------------------*/

void sid_pal_log_flush(void)
{
    if (SID_STM32_UTIL_IS_IRQ())
    {
        /* For interrupt context just poll the status repeatedly */
        while (UTIL_ADV_TRACE_IsBufferEmpty() == 0u)
        {
            __NOP();
        }
    }
    else
    {
        while (UTIL_ADV_TRACE_IsBufferEmpty() == 0u)
        {
#ifdef SID_PAL_ENABLE_SCHEDULER_DELAY
            sid_pal_scheduler_delay_ms(LOG_FLUSH_SLEEP_PERIOD_MS);
#else
#  warning "Sidewalk API has scheduler delays disabled, falling back to the blocking call to sid_pal_delay_us(). Set SID_PAL_ENABLE_SCHEDULER_DELAY to enable scheduler delays"
            sid_pal_delay_us(LOG_FLUSH_SLEEP_PERIOD_MS * 1000u);
#endif
        }
    }
}

/*----------------------------------------------------------------------------*/

char const *sid_pal_log_push_str(char *string)
{
    return (char const *)string;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_pal_log(sid_pal_log_severity_t severity, uint32_t num_args, const char *fmt, ...)
{
#if SID_PAL_LOG_ENABLED
    /* Check if the logs are enabled globally */
    struct sid_log_control_severity sid_log_settings;
    sid_log_control_get_severity(&sid_log_settings);
    if (false == sid_log_settings.enable)
    {
        /* Logs are disabled globally, skip the output */
        return;
    }

    va_list args;
    char msg_buf[SID_PAL_LOG_MSG_LENGTH_MAX + LOG_BUFFER_ADDITIONS_SIZE + 1u /* for '\0' */];
    char * msg_pos = &msg_buf[0];
    size_t remaining_space = (sizeof(msg_buf) - STATIC_STR_LEN(LOG_LINE_ENDING));
    int chars_written;

    (void)num_args;

    /* Make an empty string */
    msg_buf[0] = '\0';

    /* Print timestamp */
#if SID_PAL_LOG_USE_STM32_SYSTIME
    SysTime_t sys_time = SysTimeGetMcuTime();
    const uint32_t timestamp = SysTimeToMs(sys_time);
#else
    const uint32_t timestamp = (uint32_t)((uint64_t)osKernelGetTickCount() * 1000u / osKernelGetTickFreq());
#endif
    chars_written = snprintf_like(msg_pos, remaining_space, "[%u]", timestamp);
    remaining_space -= (size_t)chars_written;
    msg_pos += chars_written;

    /* Print severity */
    chars_written = print_severity(severity, msg_pos, remaining_space);
    if (chars_written >= 0)
    {
        if ((size_t)chars_written < remaining_space)
        {
            /* Normal operation - all characters were put into the buffer */
            remaining_space -= (size_t)chars_written;
            msg_pos += chars_written;
        }
        else
    	{
            /* Some characters did not fit, indicate the buffer is full */
            msg_pos += (remaining_space - 1u);
            remaining_space = 1u; /* For '\0' */
        }
    }
    else
    {
        /* Negative length means an invalid format */
        SID_PAL_ASSERT(0); /* This is a systematic failure of this module, stop execution */
    }

    /* Print the message itself */
    va_start(args, fmt);
    chars_written = UTIL_ADV_TRACE_VSNPRINTF(msg_pos, remaining_space, fmt, args);
    va_end(args);
    if (chars_written >= 0)
    {
        if ((size_t)chars_written < remaining_space)
        {
            /* Normal operation - all characters were put into the buffer */
            remaining_space -= (size_t)chars_written;
            msg_pos += chars_written;
        }
        else
    	{
            /* Some characters did not fit, indicate the buffer is full */
            msg_pos += (remaining_space - 1u);
            remaining_space = 1u; /* For '\0' */
        }
    }
    else
    {
        /* Negative length means an invalid format */
        SID_PAL_LOG_ERROR("Failed to log message, encoding error for format \"%s\"", fmt);  /* This is an error in user input, show error and proceed */
    }

    /* Print line ending */
    remaining_space += STATIC_STR_LEN(LOG_LINE_ENDING); /* restore the space reserved for LOG_LINE_ENDING */
    chars_written = snprintf_like(msg_pos, remaining_space, "%s", LOG_LINE_ENDING);
    if (chars_written >= 0)
    {
        if ((size_t)chars_written < remaining_space)
        {
            /* Normal operation - all characters were put into the buffer */
            remaining_space -= (size_t)chars_written;
            msg_pos += chars_written;
        }
        else
    	{
            /* Some characters did not fit, indicate the buffer is full */
            msg_pos += (remaining_space - 1u);
            remaining_space = 1u; /* For '\0' */
        }
    }
    else
    {
        /* Negative length means an invalid format */
        SID_PAL_ASSERT(0); /* This is a systematic failure of this module, stop execution */
    }

    /* Now print the entire message */
    UTIL_ADV_TRACE_FSend("%s", msg_buf); /* This is faster than just UTIL_ADV_TRACE_FSend(msg_buf) because the underlying vsnprintf() won't have to parse the entire msg_buf but rather it will just copy it into output buffer as is */
#else
    (void)severity;
    (void)num_args;
    (void)fmt;
#endif
}

/*----------------------------------------------------------------------------*/

bool sid_pal_log_get_log_buffer(struct sid_pal_log_buffer *const log_buffer)
{
    (void)log_buffer;
    SID_PAL_LOG_WARNING("%s - not implemented (optional).", __func__);

    return false;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void sid_pal_hexdump(sid_pal_log_severity_t severity, const void *address, int length)
{
#if SID_PAL_LOG_ENABLED
    struct sid_log_control_severity sid_log_settings;
    sid_log_control_get_severity(&sid_log_settings);
    if (false == sid_log_settings.enable)
    {
        /* Logs are disabled globally, skip the output */
        return;
    }

    if (severity <= sid_log_settings.level)
    {
        const char digit[16] = "0123456789ABCDEF";
        const uint8_t * const data = (uint8_t *)address;
        size_t remaining_len = (size_t)length;

        while (remaining_len > 0)
        {
            const size_t print_len = remaining_len > SID_PAL_HEXDUMP_MAX ? SID_PAL_HEXDUMP_MAX : remaining_len;

            char hex_buf[(SID_PAL_HEXDUMP_MAX * 3) /* 2 characters per byte + space */ + 1 /* for '\0' */];
            size_t print_pos = 0u;

            /* Make an empty string */
            hex_buf[0] = '\0';

            /* Print out hex string byte by byte */
            for (size_t i = 0u; i < print_len; i++)
            {
                hex_buf[print_pos] = digit[(data[((size_t)length - remaining_len) + i] >> 4) & 0x0Fu];
                print_pos++;
                hex_buf[print_pos] = digit[data[((size_t)length - remaining_len) + i] & 0x0Fu];
                print_pos++;
                hex_buf[print_pos] = ' ';
                print_pos++;
                hex_buf[print_pos] = '\0';
            }

            if (print_pos != 0u)
            {
                SID_PAL_LOG(severity, "%s", SID_PAL_LOG_PUSH_STR(hex_buf));
            }

            remaining_len -= print_len;
        }
    }
#else
    (void)severity;
    (void)address;
    (void)length;
#endif
}
