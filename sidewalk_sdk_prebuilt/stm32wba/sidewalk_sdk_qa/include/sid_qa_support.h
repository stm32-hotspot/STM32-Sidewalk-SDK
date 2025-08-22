/*
 * Copyright 2023 Amazon.com, Inc. or its affiliates. All rights reserved.
 *
 * AMAZON PROPRIETARY/CONFIDENTIAL
 *
 * You may not use this file except in compliance with the terms and
 * conditions set forth in the accompanying LICENSE.txt file.
 *
 * THESE MATERIALS ARE PROVIDED ON AN "AS IS" BASIS. AMAZON SPECIFICALLY
 * DISCLAIMS, WITH RESPECT TO THESE MATERIALS, ALL WARRANTIES, EXPRESS,
 * IMPLIED, OR STATUTORY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
 */

#ifndef SID_QA_SUPPORT_H
#define SID_QA_SUPPORT_H

#include <sid_pal_log_ifc.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef SID_ENABLE_QA
#define SID_ENABLE_QA 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if SID_ENABLE_QA
#define SID_QA_LOG_ERROR(...)           \
    if (qa_debug_log.is_enabled)        \
    {                                   \
        SID_PAL_LOG_ERROR(__VA_ARGS__); \
    }
#define SID_QA_LOG_WARNING(...)                                                            \
    if (qa_debug_log.is_enabled && qa_debug_log.log_level >= SID_PAL_LOG_SEVERITY_WARNING) \
    {                                                                                      \
        SID_PAL_LOG_WARNING(__VA_ARGS__);                                                  \
    }
#define SID_QA_LOG_INFO(...)                                                            \
    if (qa_debug_log.is_enabled && qa_debug_log.log_level >= SID_PAL_LOG_SEVERITY_INFO) \
    {                                                                                   \
        SID_PAL_LOG_INFO(__VA_ARGS__);                                                  \
    }
#if SID_BUILD_DEBUG
#define SID_QA_LOG_DEBUG(...)                                                            \
    if (qa_debug_log.is_enabled && qa_debug_log.log_level >= SID_PAL_LOG_SEVERITY_DEBUG) \
    {                                                                                    \
        SID_PAL_LOG_INFO(__VA_ARGS__);                                                   \
    }
#else
#define SID_QA_LOG_DEBUG(...)
#endif
#define SID_QA_LOG_HEXDUMP_DEBUG(...)                                                    \
    if (qa_debug_log.is_enabled && qa_debug_log.log_level >= SID_PAL_LOG_SEVERITY_DEBUG) \
    {                                                                                    \
        SID_PAL_HEXDUMP(SID_PAL_LOG_SEVERITY_INFO, __VA_ARGS__);                         \
    }

struct sid_qa_debug_log {
    bool is_enabled: 1;
    uint8_t log_level: 3;
};
extern volatile struct sid_qa_debug_log qa_debug_log;

void sid_qa_debug_log_init(void);
void sid_qa_debug_log_set(bool is_enabled, uint8_t log_level);

#else
#define SID_QA_LOG_ERROR(...)
#define SID_QA_LOG_WARNING(...)
#define SID_QA_LOG_INFO(...)
#define SID_QA_LOG_DEBUG(...)
#define SID_QA_LOG_HEXDUMP_DEBUG(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* SID_QA_SUPPORT_H */
