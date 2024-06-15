/**
  ******************************************************************************
  * @file    system_stm32g4xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include <string.h>

#include "stm32h5xx.h"
#include "drivers/system.h"
#include "platform.h"
#include "drivers/persistent.h"

#if !defined  (HSE_VALUE)
  #define HSE_VALUE    (25000000UL) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (CSI_VALUE)
  #define CSI_VALUE    (4000000UL)  /*!< Value of the Internal oscillator in Hz*/
#endif /* CSI_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    (64000000UL) /*!< Value of the Internal oscillator in Hz */
#endif /* HSI_VALUE */

/* The SystemCoreClock variable is updated in three ways:
  1) by calling CMSIS function SystemCoreClockUpdate()
  2) by calling HAL API function HAL_RCC_GetHCLKFreq()
  3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
     Note: If you use this function to configure the system clock; then there
           is no need to call the 2 first functions listed above, since SystemCoreClock
           variable is updated automatically.
*/
uint32_t SystemCoreClock = HSI_VALUE;

const uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
const uint8_t APBPrescTable[8] =  {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

void SystemClock_Config(void); // Forward

void SystemInit(void)
{
  systemProcessResetReason();

  initialiseMemorySections();

  /* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 20U)|(3UL << 22U));  /* set CP10 and CP11 Full Access */
#endif


  /* Configure the Vector Table location add offset address ------------------*/

    extern uint8_t isr_vector_table_flash_base;
    extern uint8_t isr_vector_table_base;
    extern uint8_t isr_vector_table_end;

    if (&isr_vector_table_base != &isr_vector_table_flash_base) {
        memcpy(&isr_vector_table_base, &isr_vector_table_flash_base, (size_t) (&isr_vector_table_end - &isr_vector_table_base));
    }

    /* Reset the RCC clock configuration to the default reset state ------------*/
    /* Set HSION bit */
    RCC->CR = RCC_CR_HSION;

    /* Reset CFGR register */
    RCC->CFGR1 = 0U;
    RCC->CFGR2 = 0U;

    /* Reset HSEON, HSECSSON, HSEBYP, HSEEXT, HSIDIV, HSIKERON, CSION, CSIKERON, HSI48 and PLLxON bits */
#if defined(RCC_CR_PLL3ON)
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSECSSON | RCC_CR_HSEBYP | RCC_CR_HSEEXT | RCC_CR_HSIDIV | RCC_CR_HSIKERON | \
                 RCC_CR_CSION | RCC_CR_CSIKERON |RCC_CR_HSI48ON | RCC_CR_PLL1ON | RCC_CR_PLL2ON | RCC_CR_PLL3ON);
#else
    RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSECSSON | RCC_CR_HSEBYP | RCC_CR_HSEEXT | RCC_CR_HSIDIV | RCC_CR_HSIKERON | \
                 RCC_CR_CSION | RCC_CR_CSIKERON |RCC_CR_HSI48ON | RCC_CR_PLL1ON | RCC_CR_PLL2ON);
#endif

    /* Reset PLLxCFGR register */
    RCC->PLL1CFGR = 0U;
    RCC->PLL2CFGR = 0U;
  #if defined(RCC_CR_PLL3ON)
    RCC->PLL3CFGR = 0U;
  #endif /* RCC_CR_PLL3ON */

    /* Reset PLL1DIVR register */
    RCC->PLL1DIVR = 0x01010280U;
    /* Reset PLL1FRACR register */
    RCC->PLL1FRACR = 0x00000000U;
    /* Reset PLL2DIVR register */
    RCC->PLL2DIVR = 0x01010280U;
    /* Reset PLL2FRACR register */
    RCC->PLL2FRACR = 0x00000000U;
  #if defined(RCC_CR_PLL3ON)
    /* Reset PLL3DIVR register */
    RCC->PLL3DIVR = 0x01010280U;
    /* Reset PLL3FRACR register */
    RCC->PLL3FRACR = 0x00000000U;
  #endif /* RCC_CR_PLL3ON */

    /* Reset HSEBYP bit */
    RCC->CR &= ~(RCC_CR_HSEBYP);

    /* Disable all interrupts */
    RCC->CIER = 0U;

    SCB->VTOR = (uint32_t)&isr_vector_table_base;

#ifdef USE_HAL_DRIVER
    HAL_Init();
#endif

    SystemClock_Config();
    SystemCoreClockUpdate();
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @note   - The system frequency computed by this function is not the real
  *           frequency in the chip. It is calculated based on the predefined
  *           constant and the selected clock source:
  *
  *           - If SYSCLK source is CSI, SystemCoreClock will contain the CSI_VALUE(*)
  *
  *           - If SYSCLK source is HSI, SystemCoreClock will contain the HSI_VALUE(**)
  *
  *           - If SYSCLK source is HSE, SystemCoreClock will contain the HSE_VALUE(***)
  *
  *           - If SYSCLK source is PLL, SystemCoreClock will contain the HSE_VALUE(***)
  *             or HSI_VALUE(**) or CSI_VALUE(*) multiplied/divided by the PLL factors.
  *
  *         (*) CSI_VALUE is a constant defined in stm32h5xx_hal.h file (default value
  *             4 MHz) but the real value may vary depending on the variations
  *             in voltage and temperature.
  *
  *         (**) HSI_VALUE is a constant defined in stm32h5xx_hal.h file (default value
  *              64 MHz) but the real value may vary depending on the variations
  *              in voltage and temperature.
  *
  *         (***) HSE_VALUE is a constant defined in stm32h5xx_hal.h file (default value
  *              25 MHz), user has to ensure that HSE_VALUE is same as the real
  *              frequency of the crystal used. Otherwise, this function may
  *              have wrong result.

  *
  *         - The result of this function could be not correct when using fractional
  *           value for HSE crystal.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t pllp, pllsource, pllm, pllfracen, hsivalue, tmp;
  float_t fracn1, pllvco;

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (RCC->CFGR1 & RCC_CFGR1_SWS)
  {
  case 0x00UL:  /* HSI used as system clock source */
    SystemCoreClock = (uint32_t) (HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV)>> 3));
    break;

  case 0x08UL:  /* CSI used as system clock  source */
    SystemCoreClock = CSI_VALUE;
    break;

  case 0x10UL:  /* HSE used as system clock  source */
    SystemCoreClock = HSE_VALUE;
    break;

  case 0x18UL:  /* PLL1 used as system clock source */
    /* PLL_VCO = (HSE_VALUE or HSI_VALUE or CSI_VALUE/ PLLM) * PLLN
    SYSCLK = PLL_VCO / PLLR
    */
    pllsource = (RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1SRC);
    pllm = ((RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1M)>> RCC_PLL1CFGR_PLL1M_Pos);
    pllfracen = ((RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1FRACEN)>>RCC_PLL1CFGR_PLL1FRACEN_Pos);
    fracn1 = (float_t)(uint32_t)(pllfracen* ((RCC->PLL1FRACR & RCC_PLL1FRACR_PLL1FRACN)>> RCC_PLL1FRACR_PLL1FRACN_Pos));

    switch (pllsource)
    {
    case 0x01UL:  /* HSI used as PLL clock source */
      hsivalue = (HSI_VALUE >> ((RCC->CR & RCC_CR_HSIDIV)>> 3)) ;
      pllvco = ((float_t)hsivalue / (float_t)pllm) * ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_PLL1N) + \
                (fracn1/(float_t)0x2000) +(float_t)1 );
      break;

    case 0x02UL:  /* CSI used as PLL clock source */
      pllvco = ((float_t)CSI_VALUE / (float_t)pllm) * ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_PLL1N) + \
                (fracn1/(float_t)0x2000) +(float_t)1 );
      break;

    case 0x03UL:  /* HSE used as PLL clock source */
      pllvco = ((float_t)HSE_VALUE / (float_t)pllm) * ((float_t)(uint32_t)(RCC->PLL1DIVR & RCC_PLL1DIVR_PLL1N) + \
                (fracn1/(float_t)0x2000) +(float_t)1 );
      break;

    default:  /* No clock sent to PLL*/
      pllvco = (float_t) 0U;
      break;
    }

    pllp = (((RCC->PLL1DIVR & RCC_PLL1DIVR_PLL1P) >>RCC_PLL1DIVR_PLL1P_Pos) + 1U ) ;
    SystemCoreClock =  (uint32_t)(float_t)(pllvco/(float_t)pllp);

    break;

  default:
    SystemCoreClock = HSI_VALUE;
    break;
  }
  /* Compute HCLK clock frequency --------------------------------------------*/
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR2 & RCC_CFGR2_HPRE) >> RCC_CFGR2_HPRE_Pos)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
}

// SystemSYSCLKSource
//   0: HSI
//   1: HSE
//   (2: PLLP)
//   3: PLLR

int SystemSYSCLKSource(void)
{
    uint32_t rawSrc = RCC->CFGR1 & RCC_CFGR1_SW;
    int src = 0;

    switch (rawSrc) {
    case 0: // can't happen, fall through
        FALLTHROUGH;
    case 1:
        src = 0; // HSI 
        break;

    case 2:
        src = 1; // HSE
        break;

    case 3:
        src = 3; // PLL-R
    }

    return src;
}

// SystemPLLSource
//   0: HSI
//   1: HSE

int SystemPLLSource(void)
{
    return ((RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1SRC) & 2) >> 1; // LSB determines HSI(0) or HSE(1)
}

void Error_Handler(void)
{
    while (1) {
    }
}

/*
 * G4 is capable of fine granularity overclocking thanks to a separate 48MHz source,
 * but we keep the overclocking capability to match that of F405 (clocks at 168, 192, 216, 240).
 *
 * However, due to restrictions on MCO source, designs that wishes to generate 27MHz on MCO
 * must use a 27MHz HSE source. The 27MHz HSE source will produce slightly different series of clocks
 * due to restriction on PLL multipler range.
 *
 * If mhz == 8, 16 or 24 then scale it down to 4 and start with PLLN=42 for base 168MHz,
 * with PLLN increment of 6 (4 * 6 = 24MHz a part)
 *
 * If mhz == 27 then scale it down to 9 with PLL=19 for base 171MHz with PLLN increment of 3 (9 * 3 = 27MHz a part)
 *
 * We don't prepare a separate frequency selection for 27MHz series in CLI, so what is set with "cpu_overclock" 
 * will result in slightly higher clock when examined with "status" command.
 */

// Target frequencies for cpu_overclock (Levels 0 through 3)

uint16_t sysclkSeries8[] =  { 168, 192, 216, 240 };
uint16_t sysclkSeries27[] = { 171, 198, 225, 252 };
#define OVERCLOCK_LEVELS ARRAYLEN(sysclkSeries8)

// Generic fine granularity PLL parameter calculation

static bool systemComputePLLParameters(uint8_t src, uint16_t target, int *sysclk, int *pllm, int *plln, int *pllr)
{
    int vcoTarget = target * 2;
    int vcoBase;
    int vcoDiff;
    int multDiff;
    int vcoFreq;

    *pllr = 2;

    if (src == 8 || src == 16 || src == 24) {
        *pllm = src / 8;
        vcoBase = 168 * 2;
        vcoDiff = vcoTarget - vcoBase;
        multDiff = vcoDiff / 16 * 2;
        *plln = 42 + multDiff;
        vcoFreq = 8 * *plln;
    } else if (src == 27) {
        *pllm = 3;
        vcoBase = 171 * 2;
        vcoDiff = vcoTarget - vcoBase;
        multDiff = vcoDiff / 18 * 2;
        *plln = 38 + multDiff;
        vcoFreq = 9 * *plln;
    } else {
        return false;
    }

    // VCO seems to top out at 590MHz or so. Give it some margin.

    if (vcoFreq >= 560) {
        return false;
    }
    *sysclk = vcoFreq / 2;

    return true;
}

static int pll_m;
static int pll_n;
static int pll_r;
static uint32_t pllSrc;
static int sysclkMhz;

static bool systemClock_PLLConfig(int overclockLevel)
{
    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);
    int pllInput;
    int targetMhz;

    if (hse_value == 0) {
        pllInput = 16; // HSI
        pllSrc = RCC_PLLSOURCE_HSI;
        targetMhz = 168;
    } else {
        pllInput = hse_value / 1000000;
        pllSrc = RCC_PLLSOURCE_HSE;
        if (pllInput == 8 || pllInput == 16 || pllInput == 24) {
            targetMhz = sysclkSeries8[overclockLevel];
        } else if (pllInput == 27) {
            targetMhz = sysclkSeries8[overclockLevel];
        } else {
            return false;
        }
    }

    return systemComputePLLParameters(pllInput, targetMhz, &sysclkMhz, &pll_m, &pll_n, &pll_r);
}

void systemClockSetHSEValue(uint32_t frequency)
{
    uint32_t freqMhz = frequency / 1000000;

    // Only supported HSE crystal/resonator is 27MHz or integer multiples of 8MHz

    if (freqMhz != 27 && (freqMhz / 8) * 8 != freqMhz) {
        return;
    } 

    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);

    if (hse_value != frequency) {
        persistentObjectWrite(PERSISTENT_OBJECT_HSE_VALUE, frequency);

        if (hse_value != 0) {
            __disable_irq();
            NVIC_SystemReset();
        }
    }
}

void OverclockRebootIfNecessary(unsigned requestedOverclockLevel)
{
    uint32_t currentOverclockLevel = persistentObjectRead(PERSISTENT_OBJECT_OVERCLOCK_LEVEL);

    if (requestedOverclockLevel >= OVERCLOCK_LEVELS ) {
        requestedOverclockLevel = 0;
    }

    // If we are not running at the requested speed or
    // we are running on PLL-HSI even HSE has been set,
    // then remember the requested clock and issue soft reset

    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);

    if ((currentOverclockLevel != requestedOverclockLevel) ||
            (hse_value &&
             SystemSYSCLKSource() == 3 /* PLL-R */ &&
             SystemPLLSource() != 1 /* HSE */)) {

        // Make sure we can configure the requested clock.
        if (!systemClock_PLLConfig(requestedOverclockLevel)) {
            return;
        }
        persistentObjectWrite(PERSISTENT_OBJECT_OVERCLOCK_LEVEL, requestedOverclockLevel);
        __disable_irq();
        NVIC_SystemReset();
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
// Extracted from MX generated main.c 

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_CRSInitTypeDef pInit = {0};

  systemClock_PLLConfig(persistentObjectRead(PERSISTENT_OBJECT_OVERCLOCK_LEVEL));

  // Configure the main internal regulator output voltage 

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1); // TODO figure out why we set this

  // Initializes the CPU, AHB and APB busses clocks 

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;

  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = pllSrc;
  RCC_OscInitStruct.PLL.PLLM = pll_m;
  RCC_OscInitStruct.PLL.PLLN = pll_n;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  // Initializes the CPU, AHB and APB busses clocks 

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }

  // Initializes the peripherals clocks 

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_I2C4|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_FDCAN
                              |RCC_PERIPHCLK_LPUART1
                              ;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK3;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK3;
  PeriphClkInit.I2c4ClockSelection = RCC_I2C3CLKSOURCE_PCLK3;
  PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_SYSCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  // Configures CRS 

  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);

  /* Configure Flash prefetch and wait state */
  FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_4WS;

#ifdef HAL_ICACHE_MODULE_ENABLED
  // Enable instruction cache
  HAL_ICACHE_Enable();
#endif

#ifdef HAL_DCACHE_MODULE_ENABLED
  // Enable data cache
  DCACHE_HandleTypeDef hdcache;
  hdcache.Instance = DCACHE1;
  hdcache.Init.ReadBurstType = DCACHE_READ_BURST_WRAP;
  HAL_DCACHE_Init(&hdcache);
#endif
}
