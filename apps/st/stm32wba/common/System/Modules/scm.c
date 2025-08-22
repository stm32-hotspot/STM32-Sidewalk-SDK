/**
  ******************************************************************************
  * @file    scm.c
  * @author  MCD Application Team
  * @brief   Functions for the System Clock Manager.
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
#include "app_conf.h"
#include "scm.h"
#include "RTDebug.h"
#include "utilities_common.h"
#include "timer_if.h" /* Use interface methods directly and not the UTIL_TIMER driver to save on function pointer resolvong */
#include <sid_stm32_common_utils.h>

#if (CFG_SCM_SUPPORTED)

SID_STM32_SPEED_OPTIMIZED __WEAK void SCM_HSI_CLK_ON(void)
{
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() == 0u);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED __WEAK void SCM_HSI_CLK_OFF(void)
{
  LL_RCC_HSI_Disable();
}

/*----------------------------------------------------------------------------*/

/* SCM HSE BEGIN */
SID_STM32_SPEED_OPTIMIZED __WEAK void SCM_HSI_SwithSystemClock_Entry(void)
{

}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED __WEAK void SCM_HSI_SwithSystemClock_Exit(void)
{

}
/* SCM HSE END */
/* Private typedef -----------------------------------------------------------*/
#define PLL_INPUTRANGE0_FREQMAX         8000000u  /* 8 MHz is maximum frequency for VCO input range 0 */

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* SCM HSE BEGIN */
static uint32_t SW_HSERDY = 0u;
static uint32_t HSE_STABILIZATION_DELAY_TICKS = 0u;
/* SCM HSE END */

RAMCFG_HandleTypeDef sram1_ns =
{
  RAMCFG_SRAM1,           /* Instance */
  HAL_RAMCFG_STATE_READY, /* RAMCFG State */
  0U,                     /* RAMCFG Error Code */
};

RAMCFG_HandleTypeDef sram2_ns =
{
  RAMCFG_SRAM2,           /* Instance */
  HAL_RAMCFG_STATE_READY, /* RAMCFG State */
  0U,                     /* RAMCFG Error Code */
};

static scm_system_clock_t scm_system_clock_config;
static scm_clockconfig_t scm_system_clock_requests[(scm_user_id_t)TOTAL_CLIENT_NUM] = {NO_CLOCK_CONFIG};
static scm_radio_state_t RadioState;

/* Private function prototypes -----------------------------------------------*/

static inline scm_clockconfig_t scm_getmaxfreq(void);
static inline void scm_systemclockconfig(void);
static inline void SwitchHse32toPll(void);
static inline void ConfigHwPll(const scm_pll_config_t * const p_hw_config);
static inline void SwitchHsePre(const scm_hse_hsepre_t hse_pre);
static inline void SwitchHse16toHse32(void);
static inline void SwitchHse32toHse16(void);
static inline void SwitchPlltoHse32(void);
/* SCM HSE BEGIN */
static inline void scm_hserdy_postprocess(void);
static inline void scm_pllrdy_postprocess(void);
/* SCM HSE END */

/* Private functions ---------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline scm_clockconfig_t scm_getmaxfreq(void)
{
  scm_clockconfig_t max = NO_CLOCK_CONFIG;

  for (uint32_t idx = 0u; idx < sizeof(scm_system_clock_requests) ; idx++)
  {
    if(scm_system_clock_requests[idx] > max)
    {
      max = scm_system_clock_requests[idx];
    }
  }

  return max;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void scm_systemclockconfig(void)
{
  SYSTEM_DEBUG_SIGNAL_SET(SCM_SYSTEM_CLOCK_CONFIG);

  switch (scm_system_clock_config.targeted_clock_freq)
  {
    case HSE_16MHZ:

      if(LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL1R)
      {
        /* currently running on PLL */
        SwitchPlltoHse32();
      }

      SwitchHse32toHse16();

      /* Ensure time base clock coherency */
      SystemCoreClockUpdate();

      break;

    case HSE_32MHZ:

      if (LL_RCC_HSE_IsEnabledPrescaler())
      {
        /* currently running on HSE16 */
        SwitchHse16toHse32();

        /* Ensure time base clock coherency */
        SystemCoreClockUpdate();
      }
      else if(LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_PLL1R)
      {
        /* currently running on PLL */
        SwitchPlltoHse32();

        /* Ensure time base clock coherency */
        SystemCoreClockUpdate();
      }
      else
      {
        /**
          * The system is already running on HSE32
          * The only case is when the PLL has been requested and
          * aborted before the system switched to PLL
          */

        /* Disable PLL */
        LL_RCC_PLL1_Disable();
      }

      break;

    case SYS_PLL:

      if (LL_RCC_HSE_IsEnabledPrescaler())
      {
        /* currently running on HSE16 */
        SwitchHse16toHse32();
      }

      SwitchHse32toPll();

      break;

    default:
      break;
  }

  SYSTEM_DEBUG_SIGNAL_RESET(SCM_SYSTEM_CLOCK_CONFIG);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void SwitchHsePre(const scm_hse_hsepre_t hse_pre)
{
  /* Start HSI */
  SCM_HSI_CLK_ON();

  /* SCM HSE BEGIN */
  /* Entry hook for HSI switch */
  SCM_HSI_SwithSystemClock_Entry();
  /* SCM HSE END */

  /* Set HSI as SYSCLK */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

  /* Wait for clock switch to complete */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI);

  /* Ensure time base clock coherency */
  SystemCoreClockUpdate();

  /* Enable HSEON */
  /* SCM HSE BEGIN */
  LL_RCC_HSE_Enable();
  SCM_HSE_WaitUntilReady();

  /* Exit hook for HSI switch */
  SCM_HSI_SwithSystemClock_Exit();
  /* SCM HSE END */

  /* Set/Clear HSEPRE */
  if(hse_pre == HSEPRE_DISABLE)
  {
    LL_RCC_HSE_DisablePrescaler();
  }
  else
  {
    LL_RCC_HSE_EnablePrescaler();
  }

  /* Set HSE as SYSCLK */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

  /* Wait for clock switch to complete */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE);

  /* Ensure time base clock coherency */
  SystemCoreClockUpdate();

  /* Disable HSI */
  SCM_HSI_CLK_OFF();

#if defined(STM32WBA5x) && defined(STM32WBAXX_SI_CUT1_0)
  /* STM32WBA5 Cut1.0 only: if the radio is not active is set to OFF by the hardware. */
  if(isRadioActive() == SCM_RADIO_NOT_ACTIVE)
  {
    /* SCM HSE BEGIN */
    SCM_HSE_Clear_SW_HSERDY();
    /* SCM HSE END */
  }
#endif /* STM32WBA5x && STM32WBAXX_SI_CUT1_0 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void SwitchHse16toHse32(void)
{
  /**
    * Switch from HSE_16MHz to HSE_32MHz
    * 1. Voltage Range1
    * 2. Disable prescaler ==> HSE16 to HSE32
    * 3. Change RAM/FLASH waitstates (no limitation in Rang1)
    * 4. AHB5 Div 1
    */

  /* first switch to VOS1 */
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() == 0u);

  /* Switch to 32Mhz */
  SwitchHsePre(HSEPRE_DISABLE);

  /* Configure flash and SRAMs */
  scm_setwaitstates(HSE32);

  /* Need to set HDIV5 */
  LL_RCC_SetAHB5Divider(LL_RCC_AHB5_DIVIDER_1); /* divided by 1 */
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void SwitchHse32toHse16(void)
{
  /**
    * Switch from HSE_16MHz to HSE_32MHz
    * 1. AHB5 Div 2
    * 2. Change RAM/FLASH waitstates
    * 3. Disable prescaler ==> HSE16 to HSE32
    * 4. Voltage Range2
    */

  /* Divide HDIV5 by 2 */
  LL_RCC_SetAHB5Divider(LL_RCC_AHB5_DIVIDER_2);

  /* Configure flash and SRAMs before switching to VOS2 */
  scm_setwaitstates(HSE16);

  /* Switch to HSE 16 */
  SwitchHsePre(HSEPRE_ENABLE);

  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void SwitchPlltoHse32(void)
{
  /**
    * Switch from PLL to HSE_32MHz
    * 1. Switch system clock source to HSE
    * 2. Turn OFF PLL
    * 3. Change RAM/FLASH waitstates (no limitation in Rang1)
    */

  /* Switch to HSE */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

  /* Wait for clock switch to complete */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE);

  /* Ensure time base clock coherency */
  SystemCoreClockUpdate();

  /* Disable PLL */
  LL_RCC_PLL1_Disable();

  /* Configure flash and SRAMs */
  scm_setwaitstates(HSE32);
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void SwitchHse32toPll(void)
{
  /* Enable PLL1 output for SYSCLK (PLL1R) */
  LL_RCC_PLL1_EnableDomain_PLL1R();

  /* Configure and start the PLL */
  LL_RCC_PLL1_SetMainSource(LL_RCC_PLL1SOURCE_HSE);

  /**
   * @attention Do not use interrupts here because running on intermediate clock (e.g., HSI or HSE) will result in
   *            wrong peripheral clocks (e.g., wrong U(S)ART baud rate, wrong TIM counting speed, etc.). Due to this
   *            limitation, running on an intermediate clock has little to no practical value, but may result in major
   *            glitches and introduce significant risks of undefined behavior due to that. For the reliability and
   *            robustness purposes clock configuration should be fully completed before the RTOS or user code is
   *            allowed to run
   */

  /* Enable PLL1 */
  __HAL_RCC_PLL1_ENABLE();

  /* Wait for PLL to be ready */
  while (LL_RCC_PLL1_IsReady() == 0u);

  /* Setup system clock once PLL is ready */
  scm_pllrdy_postprocess();
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED static inline void ConfigHwPll(const scm_pll_config_t * const p_hw_config)
{
  uint32_t freq_vco_in = 0;

  /* Apply user PLL mode */
  if(p_hw_config->pll_mode == PLL_FRACTIONAL_MODE)
  {
    scm_pll_fractional_update(p_hw_config->PLLFractional);
  }
  else
  {
    /* Integer configuration will be used for PLL mode */
    LL_RCC_PLL1FRACN_Disable();
  }

  /* Apply correct frequency range for VCO_IN */
  /* Note as PLL clock source is always HSE 32MHz, only PLL1M value impact VCO_IN */

  freq_vco_in = HSE_VALUE/p_hw_config->PLLM;
  if (freq_vco_in > PLL_INPUTRANGE0_FREQMAX)
  {
    freq_vco_in = RCC_PLL_VCOINPUT_RANGE1;
  }
  else
  {
    freq_vco_in = RCC_PLL_VCOINPUT_RANGE0;
  }
  __HAL_RCC_PLL1_VCOINPUTRANGE_CONFIG(freq_vco_in);

  __HAL_RCC_PLL1_CONFIG(RCC_PLLSOURCE_HSE, /* PLL clock source is always HSE 32MHz */
                        p_hw_config->PLLM,
                        p_hw_config->PLLN,
                        p_hw_config->PLLP,
                        p_hw_config->PLLQ,
                        p_hw_config->PLLR
                        );

  LL_RCC_SetAHB5Prescaler(p_hw_config->AHB5_PLL1_CLKDivider);

  /* PLL is now initialized */
  scm_system_clock_config.pll.are_pll_params_initialized = 1u;
}

/*----------------------------------------------------------------------------*/

/* SCM HSE BEGIN */
/**
  * @brief  SCM HSERDY interrupt handler.
  *         Switch system clock on HSE.
  * @param  None
  * @retval None
  */
SID_STM32_SPEED_OPTIMIZED static inline void scm_hserdy_postprocess(void)
{
  SYSTEM_DEBUG_SIGNAL_SET(SCM_HSERDY_ISR);

  if(LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
    /* Wait for VOS switch to complete */
    while (LL_PWR_IsActiveFlag_VOS() == 0u);

    if(scm_system_clock_config.targeted_clock_freq == HSE_16MHZ)
    {
      /* Configure the wait states for HSE16 operation */
      scm_setwaitstates(HSE16);

      /**
        * The current system configuration is:
        * Range2, HDIV5 set, Wait States compliant to HSE16
        */
      LL_RCC_HSE_EnablePrescaler();
      /* Switch to HSE */
      LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

      /* Wait for clock switch to complete */
      while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE);

      /* Ensure time base clock coherency */
      SystemCoreClockUpdate();
    }
    else /* Target clock is either HSE32 or PLL */
    {
      /**
        * The current system configuration is:
        * Range1
        */

      LL_RCC_HSE_DisablePrescaler();

      /* Switch to HSE */
      LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

      /* Wait for clock switch to complete */
      while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE);

      /* Ensure time base clock coherency */
      SystemCoreClockUpdate();

      /* Configure the wait states for HSE32 operation */
      scm_setwaitstates(HSE32); /* There is no limitation when in Range1 */

      /* Set HDIV 5 */
      LL_RCC_SetAHB5Divider(LL_RCC_AHB5_DIVIDER_1); /* divided by 1 */

      if(scm_system_clock_config.targeted_clock_freq == SYS_PLL)
      {
        /* The system clock target is based on PLL */

        /* Configure and start PLL */
        SwitchHse32toPll();
      }
    }

    /* As system switched to HSE, disable HSI */
    SCM_HSI_CLK_OFF();
  }

  SYSTEM_DEBUG_SIGNAL_RESET(SCM_HSERDY_ISR);
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  SCM PLLRDY interrupt handler.
  *         Switch system clock on PLL.
  * @param  None
  * @retval None
  */
SID_STM32_SPEED_OPTIMIZED static inline void scm_pllrdy_postprocess(void)
{
  if(scm_system_clock_config.targeted_clock_freq == SYS_PLL)
  {
    /* Set PLL compatible waitstates */
    scm_setwaitstates(PLL);

    /* Switch to PLL */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1R);

    /* Wait for clock switch to complete */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1R);

    /* Ensure time base clock coherency */
    SystemCoreClockUpdate();

    scm_pllready();
  }
  else
  {
    /**
      * The PLL was enabled but is not used anymore as system clock
      * The only case is when a request has been made and cancelled before
      * the system had time to switch the system clock on PLL
      */
    /* Disable PLL */
    LL_RCC_PLL1_Disable();
  }
}
/* SCM HSE END */

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  System Clock Manager init code
  * @param  None
  * @retval None
  */
SID_STM32_SPEED_OPTIMIZED void scm_init()
{
  /* init scm_system_clock_config with LP config
   * scm_system_clock_config SHALL BE UPDATED BY READING HW CONFIG FROM HAL APIs
   * SHALL BE CALLED AFTER SystemClock_Config()
   **/

  /* Default PLL configuration => no configuration */
  memset(&(scm_system_clock_config.pll), 0, sizeof(scm_pll_config_t));

  /* Reading FLASH and SRAMs waitstates from registers */
  scm_system_clock_config.flash_ws_cfg = __HAL_FLASH_GET_LATENCY();
  scm_system_clock_config.sram_ws_cfg = HAL_RAMCFG_GetWaitState(&sram1_ns);

  /* Link Layer is not active at this stage */
  RadioState = SCM_RADIO_NOT_ACTIVE;

  /* Enable RAMCFG clock */
  __HAL_RCC_RAMCFG_CLK_ENABLE();

  /* SCM HSE BEGIN */
  /* Init SW HSE Flag */
  SCM_HSE_Set_SW_HSERDY();

  /* Compute delay for HSE stabilization */
  HSE_STABILIZATION_DELAY_TICKS = TIMER_IF_Convert_us2Tick(CFG_LPM_HSE_STABILIZATION_DELAY_US);
  /* SCM HSE END */

  /* Reading system core clock configuration from registers */
  switch(LL_RCC_GetSysClkSource())
  {
    case LL_RCC_SYS_CLKSOURCE_STATUS_HSI:
      /* HSI system clock configuration is not supported on SCM module as radio activity is not possible.
       * Switch to HSE_16MHz required.
       */

      /* Target system clock frequency is now HSE_16MHZ */
      scm_system_clock_config.targeted_clock_freq = HSE_16MHZ;

      /* Enable prescaler */
       LL_RCC_HSE_EnablePrescaler();

      /* Set HDIV 5 */
      LL_RCC_SetAHB5Divider(LL_RCC_AHB5_DIVIDER_2); /* divided by 2 */

      scm_setup();

      /* Set VOS to range 2 */
      LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);

      break;

    case LL_RCC_SYS_CLKSOURCE_STATUS_HSE:

      /* Get AHB5 divider for HSE frequency */
      if (LL_RCC_HSE_IsEnabledPrescaler())
      {
        /* System core clock is HSE_16MHz */
        scm_system_clock_config.targeted_clock_freq = HSE_16MHZ;
      }
      else
      {
        /* System core clock is HSE_32MHz */
        scm_system_clock_config.targeted_clock_freq = HSE_32MHZ;
      }

      break;

    case LL_RCC_SYS_CLKSOURCE_STATUS_PLL1R:
        scm_system_clock_config.targeted_clock_freq = SYS_PLL;

        /* Initial PLL configuration */
        scm_system_clock_config.pll.PLLM = LL_RCC_PLL1_GetDivider();
        scm_system_clock_config.pll.PLLN = LL_RCC_PLL1_GetN();
        scm_system_clock_config.pll.PLLP = LL_RCC_PLL1_GetP();
        scm_system_clock_config.pll.PLLQ = LL_RCC_PLL1_GetQ();
        scm_system_clock_config.pll.PLLR = LL_RCC_PLL1_GetR();
        scm_system_clock_config.pll.PLLFractional = LL_RCC_PLL1_GetFRACN();
        scm_system_clock_config.pll.AHB5_PLL1_CLKDivider = LL_RCC_GetAHB5Prescaler();
        if(scm_system_clock_config.pll.PLLFractional == PLL_FRACTIONAL_MODE)
        {
          scm_system_clock_config.pll.pll_mode = PLL_FRACTIONAL_MODE;
        }
        else
        {
          scm_system_clock_config.pll.pll_mode = PLL_INTEGER_MODE;
        }

      break;
  }

  scm_system_clock_requests[SCM_USER_APP]= scm_system_clock_config.targeted_clock_freq;
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  Setup the system clock source in usable configuration for Connectivity use cases.
  *         Called at startup or out of low power modes.
  * @param  None
  * @retval None
  */
SID_STM32_SPEED_OPTIMIZED void scm_setup(void)
{
  SYSTEM_DEBUG_SIGNAL_SET(SCM_SETUP);

  /* System clock is now on HSI 16Mhz, as it exits from LPM mode */

  /**
   * @attention Do not use interrupts here because running on intermediate clock (e.g., HSI or HSE) will result in
   *            wrong peripheral clocks (e.g., wrong U(S)ART baud rate, wrong TIM counting speed, etc.). Due to this
   *            limitation, running on an intermediate clock has little to no practical value, but may result in major
   *            glitches and introduce significant risks of undefined behavior due to that. For the reliability and
   *            robustness purposes clock configuration should be fully completed before the RTOS or user code is
   *            allowed to run
   */

  if(scm_system_clock_config.targeted_clock_freq == NO_CLOCK_CONFIG)
  {
    /* Keep running on HSI */
    return;
  }

  /* Check if HSE was running through out the LPM phase due to 2.4GHz radio request */
  if ((SCM_HSE_Get_SW_HSERDY() != 0u) && (RadioState == SCM_RADIO_ACTIVE))
  {
    /**
      * The current system configuration is:
      * Range1, HDIV5 cleared, HSEPRE cleared
      */

    /* Set HSEON bit in RCC_CR to indicate that HSE is used by the CPU core as well, not only by the 2.4GHz radio */
    LL_RCC_HSE_Enable();

    /* HSE was running and it is stable, proceed with the system clock configuration */
    scm_hserdy_postprocess();
  }
  else
  {
    /* Check if the system need to increase VOS range (clock frequency higher than HSE 16Mhz)*/
    if(scm_system_clock_config.targeted_clock_freq != HSE_16MHZ)
    {
      /* Initiate VOS switch to range 1 - do this here to run voltage switch concurrently with HSE start and saving some time */
      LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    }

    /* Start HSE */
    LL_RCC_HSE_Enable();

    /* SCM HSE BEGIN */
    /* Wait for RCC to indicate HSE is up and running */
    SCM_HSE_WaitUntilReady();
    /* SCM HSE END */

    /* Now HSE is stable, proceed with system clock configuration */
    scm_hserdy_postprocess();
  }

  SYSTEM_DEBUG_SIGNAL_RESET(SCM_SETUP);
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  Configure the PLL mode and parameters before PLL selection as system clock.
  * @param  p_pll_config PLL configuration to apply
  * @retval None
  * @note   scm_pll_setconfig to be called before PLL activation (PLL set as system core clock)
  */
SID_STM32_SPEED_OPTIMIZED void scm_pll_setconfig(const scm_pll_config_t *p_pll_config)
{
  /* Initial PLL configuration */
  scm_system_clock_config.pll.PLLM = p_pll_config->PLLM;
  scm_system_clock_config.pll.PLLN = p_pll_config->PLLN;
  scm_system_clock_config.pll.PLLP = p_pll_config->PLLP;
  scm_system_clock_config.pll.PLLQ = p_pll_config->PLLQ;
  scm_system_clock_config.pll.PLLR = p_pll_config->PLLR;
  scm_system_clock_config.pll.PLLFractional = p_pll_config->PLLFractional;
  scm_system_clock_config.pll.pll_mode = p_pll_config->pll_mode;
  scm_system_clock_config.pll.AHB5_PLL1_CLKDivider = p_pll_config->AHB5_PLL1_CLKDivider;

  ConfigHwPll(&scm_system_clock_config.pll);
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  Configure the PLL for switching fractional parameters on the fly.
  * @param  pll_frac Up to date fractional configuration.
  * @retval None
  * @note   A PLL update is requested only when the system clock is
  *         running on the PLL with a different configuration that the
  *         one required
  */
SID_STM32_SPEED_OPTIMIZED void scm_pll_fractional_update(uint32_t pll_frac)
{
  /* PLL1FRACEN set to 0 */
  LL_RCC_PLL1FRACN_Disable();

  /* Update PLL1FRACR register */
  LL_RCC_PLL1_SetFRACN(pll_frac);

  /* PLL1FRACEN set to 1 */
  LL_RCC_PLL1FRACN_Enable();

  /* Ensure time base clock coherency */
  SystemCoreClockUpdate();
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  Set the system clock to the requested frequency.
  * @param  user_id This parameter can be one of the following:
  *         @arg SCM_USER_APP
  *         @arg SCM_USER_LL_FW
  * @param  sysclockconfig This parameter can be one of the following:
  *         @arg HSE_16MHZ
  *         @arg HSE_32MHZ
  *         @arg SYS_PLL
  * @retval None
  */
SID_STM32_SPEED_OPTIMIZED void scm_setsystemclock(scm_user_id_t user_id, scm_clockconfig_t sysclockconfig)
{
  scm_clockconfig_t max_freq_requested;

  /**
   * @attention Do not use interrupts here because running on intermediate clock (e.g., HSI or HSE) will result in
   *            wrong peripheral clocks (e.g., wrong U(S)ART baud rate, wrong TIM counting speed, etc.). Due to this
   *            limitation, running on an intermediate clock has little to no practical value, but may result in major
   *            glitches and introduce significant risks of undefined behavior due to that. For the reliability and
   *            robustness purposes clock configuration should be fully completed before the RTOS or user code is
   *            allowed to run
   */

  UTILS_ENTER_LIMITED_CRITICAL_SECTION(RCC_INTR_PRIO<<4);

  /* Register the request by updating the requested frequency for this user */
  scm_system_clock_requests[user_id] = sysclockconfig;

  /* Get the higher frequency required by the clients */
  max_freq_requested = scm_getmaxfreq();

  /* Check if we need to apply another clock frequency */
  if(scm_system_clock_config.targeted_clock_freq != max_freq_requested)
  {
    scm_system_clock_config.targeted_clock_freq = max_freq_requested;

    /* Check the current system clock source (HSI or HSE) */
    if(LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
    {
      /* HSI is still the system clock */

      if(scm_system_clock_config.targeted_clock_freq == HSE_16MHZ)
      {
        /* The system clock target is HSE 16Mhz */

        /* Clear VOS (Range 2) */
        LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
      }
      else
      {
        /* The system clock target is higher than HSE 16Mhz */

        /* Set VOS (Range 1) */
        LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

        if(RadioState != SCM_RADIO_NOT_ACTIVE)
        {
          /* Wait until VOS has changed */
          while (LL_PWR_IsActiveFlag_VOS() == 0u);

          /* Wait until HSE is ready */
          /* SCM HSE BEGIN */
          SCM_HSE_WaitUntilReady();
          /* SCM HSE END */

          LL_RCC_HSE_DisablePrescaler();

          /* Switch to HSE */
          LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);

          /* Wait for clock switch to complete */
          while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE);

          /* Ensure time base clock coherency */
          SystemCoreClockUpdate();

          scm_setwaitstates(HSE32); /* There is no limitation when in Range1 */

          LL_RCC_SetAHB5Divider(LL_RCC_AHB5_DIVIDER_1);

          SCM_HSI_CLK_OFF();

          /* Check if PLL is requested */
          if(scm_system_clock_config.targeted_clock_freq == SYS_PLL)
          {
            /* Configure system clock to use PLL */
            SwitchHse32toPll();
          }
        }
      }

      /* System clock is going to be configured in RCC HSERDY interrupt */
    }
    else
    {
      /* HSE is already the system clock source */
      /* Configure the system clock */
      scm_systemclockconfig();
    }
  }

  UTILS_EXIT_LIMITED_CRITICAL_SECTION();
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  Called each time the PLL is ready
  * @param  None
  * @retval None
  * @note   This function is defined as weak in SCM module.
  *         Can be overridden by user.
  */
SID_STM32_SPEED_OPTIMIZED __WEAK void scm_pllready(void)
{
  /* To be override by user */
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  Configure the Flash and SRAMs wait cycle (when required for system clock source change)
  * @param  ws_lp_config: This parameter can be one of the following:
  *         @arg LP
  *         @arg RUN
  *         @arg HSE16
  *         @arg HSE32
  *         @arg PLL
  * @retval None
  */
SID_STM32_SPEED_OPTIMIZED void scm_setwaitstates(const scm_ws_lp_t ws_lp_config)
{
  /* Configure flash and SRAMs */
  switch (ws_lp_config) {
  case LP:
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_3);
    while(__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_3);
    HAL_RAMCFG_ConfigWaitState(&sram1_ns, RAMCFG_WAITSTATE_1);
    HAL_RAMCFG_ConfigWaitState(&sram2_ns, RAMCFG_WAITSTATE_1);
    break;

  case RUN:
    __HAL_FLASH_SET_LATENCY(scm_system_clock_config.flash_ws_cfg);
    while(__HAL_FLASH_GET_LATENCY() != scm_system_clock_config.flash_ws_cfg);
    HAL_RAMCFG_ConfigWaitState(&sram1_ns, scm_system_clock_config.sram_ws_cfg);
    HAL_RAMCFG_ConfigWaitState(&sram2_ns, scm_system_clock_config.sram_ws_cfg);
    break;

  case HSE16:
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);
    while(__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_1);
    HAL_RAMCFG_ConfigWaitState(&sram1_ns, RAMCFG_WAITSTATE_1);
    HAL_RAMCFG_ConfigWaitState(&sram2_ns, RAMCFG_WAITSTATE_1);

    scm_system_clock_config.flash_ws_cfg = FLASH_LATENCY_1;
    scm_system_clock_config.sram_ws_cfg = RAMCFG_WAITSTATE_1;

    break;

  case HSE32:
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
    while(__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_0);
    HAL_RAMCFG_ConfigWaitState(&sram1_ns, RAMCFG_WAITSTATE_0);
    HAL_RAMCFG_ConfigWaitState(&sram2_ns, RAMCFG_WAITSTATE_0);

    scm_system_clock_config.flash_ws_cfg = FLASH_LATENCY_0;
    scm_system_clock_config.sram_ws_cfg = RAMCFG_WAITSTATE_0;

    break;

  case PLL:
    /* RAM latencies are alreadey set to 0WS */
    /* Set Flash LATENCY according to PLL configuration */
    /* BELOW CONFIGURATION IS WORST CASE, SHALL BE OPTIMIZED */
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_3);
    while(__HAL_FLASH_GET_LATENCY() != FLASH_LATENCY_3);
    scm_system_clock_config.flash_ws_cfg = FLASH_LATENCY_3;
    break;

  default:
    break;
  }
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  Notify the state of the Radio
  * @param  radio_state: This parameter can be one of the following:
  *         @arg SCM_RADIO_ACTIVE
  *         @arg SCM_RADIO_NOT_ACTIVE
  * @retval None
  */
SID_STM32_SPEED_OPTIMIZED void scm_notifyradiostate(const scm_radio_state_t radio_state)
{
  if(radio_state != SCM_RADIO_NOT_ACTIVE)
  {
    RadioState = SCM_RADIO_ACTIVE; /* shall be set before calling scm_setsystemclock() */
    scm_setsystemclock(SCM_USER_LL_FW, HSE_32MHZ); /* shall be set before calling scm_setsystemclock() */
  }
  else
  {
    RadioState = SCM_RADIO_NOT_ACTIVE;
    scm_setsystemclock(SCM_USER_LL_FW, HSE_16MHZ);
  }
}

/*----------------------------------------------------------------------------*/

/**
  * @brief  Restore system clock configuration when moving out of standby.
  * @param  None
  * @retval None
  */
SID_STM32_SPEED_OPTIMIZED void scm_standbyexit(void)
{
  if(scm_system_clock_config.pll.are_pll_params_initialized == 1u)
  {
    /* Restore PLL even if not yet used in case it has been setup upfront at initialization */
    ConfigHwPll(&scm_system_clock_config.pll);
  }

  scm_setup();
}

/*----------------------------------------------------------------------------*/

/* SCM HSE BEGIN */
SID_STM32_SPEED_OPTIMIZED scm_radio_state_t isRadioActive(void)
{
  return RadioState;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED uint32_t SCM_HSE_Get_SW_HSERDY(void)
{
  /**
   * @note A critical sectio is not required here because the operation is effectively atomic
   */
  return SW_HSERDY;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void SCM_HSE_Set_SW_HSERDY(void)
{
  /**
   * @note A critical sectio is not required here because the operation is effectively atomic
   */
  SW_HSERDY = 1u;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void SCM_HSE_Clear_SW_HSERDY(void)
{
  /**
   * @note A critical sectio is not required here because the operation is effectively atomic
   */
  SW_HSERDY = 0u;
}

/*----------------------------------------------------------------------------*/

SID_STM32_SPEED_OPTIMIZED void SCM_HSE_WaitUntilReady(void)
{
  register uint32_t hse_start_timestamp;
  register uint32_t now_timestamp;

  UTILS_ENTER_CRITICAL_SECTION();

  if (SCM_HSE_Get_SW_HSERDY() == 0u)
  {
    /* Wait for RCC to indicate HSE is up and running */
    do
    {
      hse_start_timestamp = TIMER_IF_GetTimerValue();
    } while (LL_RCC_HSE_IsReady() == 0u);

    /* Give HSE some time to stabilize since it has just started */
    do
    {
      now_timestamp = TIMER_IF_GetTimerValue();
    } while ((now_timestamp - hse_start_timestamp) < HSE_STABILIZATION_DELAY_TICKS);

    /* Set the SW HSERDY flag */
    SCM_HSE_Set_SW_HSERDY();
  }

  UTILS_EXIT_CRITICAL_SECTION();
}
#endif /* CFG_SCM_SUPPORTED */

/*----------------------------------------------------------------------------*/

__WEAK void scm_pllrdy_isr(void){/* Intentionally empty */}
__WEAK void scm_hserdy_isr(void){/* Intentionally empty */}
