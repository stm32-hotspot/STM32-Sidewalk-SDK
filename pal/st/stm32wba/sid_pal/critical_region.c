/**
  ******************************************************************************
  * @file    crtical_region.c
  * @brief   Critical region implementation for Sidewalk SDK
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

#include <cmsis_compiler.h>

#include <sid_pal_assert_ifc.h>
#include <sid_pal_critical_region_ifc.h>
#include <sid_stm32_common_utils.h>

#include <stdatomic.h>

#include <stm32wbaxx.h>

/* Private defines -----------------------------------------------------------*/

#ifndef SID_PAL_CRITICAL_REGION_NVIC_NMI_SIMULATION_THRESHOLD_PRIORITY
#  define SID_PAL_CRITICAL_REGION_NVIC_NMI_SIMULATION_THRESHOLD_PRIORITY (1u) /*!< Any NVIC interrupts with this or higher priority will be considered as NMI in the context of Sidewalk's critical region */
#endif /* SID_PAL_CRITICAL_REGION_NVIC_NMI_SIMULATION_THRESHOLD_PRIORITY */

#ifndef SID_PAL_CRITICAL_REGION_RECURSION_LIMIT
#  define SID_PAL_CRITICAL_REGION_RECURSION_LIMIT                        (8u) /*!< The maximum depth of the critical section. Use some reasonable value for your application */
#endif /* SID_PAL_CRITICAL_REGION_RECURSION_LIMIT */

#define SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN                        (defined(BLE))

/* Private variables ---------------------------------------------------------*/

static atomic_int count = ATOMIC_VAR_INIT(0);
#if SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN
static uint32_t basepri_on_entry;
#else
static uint32_t primask_on_entry;
#endif /* SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN */

/* Global function definitions -----------------------------------------------*/

/**
  * @brief  Enter critical region
  */
SID_STM32_SPEED_OPTIMIZED void sid_pal_enter_critical_region()
{
    /* 1. Immediately mask all NVIC interrupts */
    register uint32_t primask_reg = __get_PRIMASK();
    __disable_irq();
    __DSB();
    __ISB();
    __COMPILER_BARRIER();

    /* 2. Increment re-entrancy counter */
    const int32_t prev_val = atomic_fetch_add(&count, 1);
    SID_PAL_ASSERT(prev_val <= SID_PAL_CRITICAL_REGION_RECURSION_LIMIT); /* Check for systematic failures */

    /* 3. Store PRIMASK or BASEPRI on entry to the root critical section */
    if (0u == prev_val)
    {
#if SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN
        /**
         * NOTE: The 2.4GHz radio on STM32WBAxx is driven by the shared CPU core that runs user applications. Given the radio has some
         *       mission-critical and time-sensitive operations while Sidewalk stack (and potentially user app in some cases) have a
         *       tendency to spend significant amount of time in critical sections, we cannot block all the interrupts, otherwise this
         *       will result in poor 2.4GHz radio performance (missed or misaligned radio events). To avoid that we can leave the NVIC
         *       interrupts with priorities of 0 and 1 still active, effectively simulating NMI for these two priorities and assuming
         *       these two priorities will be used for mission-critical operations like 2.4GHz LL handling and respective IRQ handlers
         *       won't do anything that requires a true critical section with all NVIC interrupts disabled. That's the closest we can
         *       to a critical section implementation that does not affect the radio peripheral. It's responsibility of the end user
         *       of this Sidewalk API to ensure that all the interrupt priorities they use in their app have a priority value in 2-15
         *       range.
         * 
         * NOTE: Unfortunately, it's not possible to simulate NMI on STM32WBAxx for NVIC priority 0 only. The reason is that LSB bits
         *       of the BASEPRI CPU core register are not implemented on this platform and are always read as '0'. As a result. This
         *       means an attempt to configure BASEPRI for NVIC priority 0 will end up in writing 0x00 to the BASEPRI register, and
         *       zero value in BASEPRI completely disables priority level boost. That's why the highest usable NVIC priority is 1 and
         *       NMI simulation covers NVIC priorities 0 and 1
         */

         /* Simulate NMI for NVIC priorities 0 and 1 to keep 2.4GHz radio up & running even if some parts of the app spend to much time in a critical section */
         const register uint32_t basepri_reg         = __get_BASEPRI(); /* Store the original BASEPRI value */
         const register uint32_t subgroup_bits       = NVIC_GetPriorityGrouping() + 1u; /* Number of bits allocated for subgroup. By default this is set to 4 bits (SCB->AIRCR.PRIGROUP = 3), matching the non-implemented 4 LSB in NVIC, but the code may set PRIGROUP to other value in runtime */
         const register uint32_t nmi_sim_basepri_val = (SID_PAL_CRITICAL_REGION_NVIC_NMI_SIMULATION_THRESHOLD_PRIORITY << subgroup_bits) & 0xFFu; /* Compute new BASEPRI value, ensure the reserved bits remain clear */
         SID_PAL_ASSERT(nmi_sim_basepri_val != 0x00u); /* Check that BASEPRI is non-zero, otherwise all NVIC interrupts will remain enabled */
         __set_BASEPRI_MAX(nmi_sim_basepri_val);

        basepri_on_entry = basepri_reg; /* Store the initial BASEPRI value to restore when critical section is terminated */
#else
        primask_on_entry = primask_reg;
#endif /* SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN */
    }

#if SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN
    /* 4. If NMI simulation is used we must restore BASEPRI state after BASEPRI setup is finished */
    __set_PRIMASK(primask_reg);
    __DSB();
    __ISB();
#endif /* SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN */
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  Exit critical region
  */
SID_STM32_SPEED_OPTIMIZED void sid_pal_exit_critical_region() 
{
#if SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN
    /* 1. Immediately mask all NVIC interrupts when NMI simulation is used */
    register uint32_t primask_reg = __get_PRIMASK();
    __disable_irq();
    __DSB();
    __ISB();
    __COMPILER_BARRIER();
#endif /* SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN */

    /* 2. Decrement re-entrancy counter */
    const int32_t prev_val = atomic_fetch_sub(&count, 1);
    SID_PAL_ASSERT(prev_val > 0);

    if (prev_val == 1)
    {
        /* 3. Restore PRIMASK or BASEPRI when leaving the root critical section */
#if SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN
        __set_BASEPRI(basepri_on_entry); /* Clear BASEPRI, stop NMI simulation */
#else
        __set_PRIMASK(primask_on_entry); /* Restore PRIMASK bit to re-enable NVIC IRQs */
#endif /* SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN */
    }

#if SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN
    /* 4. If NMI simulation is used we must restore BASEPRI state after BASEPRI setup is finished */
    __set_PRIMASK(primask_reg);
    __DSB();
    __ISB();
#endif /* SID_PAL_CRITICAL_REGION_NMI_SIMULATION_EN */
}
