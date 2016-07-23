/**
  ******************************************************************************
  * @file    Templates/Src/main.c
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    29-April-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <stdlib.h>

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void increment_speed(void);

/* Private functions ---------------------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
#define BLUE_LED 8
#define GREEN_LED 9
#define BUTTON 0

#define PRESCALE_VALUE 2399

// .1ms per tick
static uint16_t PWM_SCALING = 24000000 / ((PRESCALE_VALUE + 1) * 1000);
// Following defines are all in ms
#define PWM_PERIOD 20 * PWM_SCALING
#define PWM_MIN_DUTY 1 * PWM_SCALING
#define PWM_MAX_DUTY 2 * PWM_SCALING
#define PWM_HALF_DUTY (PWM_MAX_DUTY + PWM_MIN_DUTY) / 2
#define PWM_TEST_DUTY PWM_MIN_DUTY + 1

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  /* STM32F1xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 24 MHz */
  SystemClock_Config();


  /* Add your application code here */
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  // Configures PC8 to push-pull output, alternate function
  GPIOC->CRH = (GPIOC->CRH & 0xFFFFFFF0) | (GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1);
  // Configures PC9 to push-pull output, alternate function
  GPIOC->CRH = (GPIOC->CRH & 0xFFFFFF0F) | (GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1);

  AFIO->MAPR = AFIO_MAPR_TIM3_REMAP; // Map PC9 to timer3 channel 4

  // Enables TIM3 for pins. 24mhz system clock
  TIM3->PSC = PRESCALE_VALUE;
  TIM3->ARR = PWM_PERIOD;
  TIM3->CCR4 = PWM_MIN_DUTY;
  TIM3->CCR3 = PWM_MAX_DUTY;
  TIM3->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; // PWM Mode 1
  TIM3->CCER = TIM_CCER_CC4E | TIM_CCER_CC3E;   // Enable compare output
  TIM3->CR1 = TIM_CR1_CEN;

  // Configures PA0 to Pull-up/pull-down input
  GPIOA->CRL = (GPIOA->CRL & 0xFFFFFFF0) | (GPIO_CRL_CNF0_1);

  NVIC_EnableIRQ(TIM3_IRQn);
  TIM3->DIER = TIM_DIER_UIE;

  /* Infinite loop */
  while (1) {
  }
}

// Used to debounce the button press.
// Hysteresis like effect 
// Interrupt should trigger once evry 20ms
void TIM3_IRQHandler(void) {
  static const uint8_t counter_max = 3;
  static int8_t button_debounce_counter = -counter_max;
  volatile uint8_t button_state = GPIOA->IDR & GPIO_IDR_IDR0;
  // Button pressed
  if (button_state) {
    if (button_debounce_counter == 0) {
      increment_speed();
    }
    if (++button_debounce_counter > counter_max) {
      button_debounce_counter--;
    }
  } else { // Button not pressed
    if (abs(--button_debounce_counter) > counter_max) {
      button_debounce_counter++;
    }
  }
}

void increment_speed(void) {
  volatile uint16_t current_duty = TIM3->CCR4;
  uint16_t increment = current_duty - PWM_MIN_DUTY;
  increment = (increment + 1) % 5;
  TIM3->CCR4 = PWM_MIN_DUTY + increment;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 24000000
  *            HCLK(Hz)                       = 24000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV1                    = 2
  *            PLLMUL                         = 6
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV2;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_0)!= HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
