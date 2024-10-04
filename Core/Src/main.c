/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stm32f411xe.h"
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define GLED_PIN     (12U)
#define GLED_PIN_ODR_MSK     (1U << GLED_PIN)  // Pin 12

#define BLED_PIN  (15U)
#define BLED_PIN_ODR_MSK     (1U << BLED_PIN)  // Pin 15


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

volatile bool toggle_led = false;
volatile bool switch_toggle = true;

void delay(int ms)
{
    while(ms--);
}
void switch_init()
{
	__disable_irq();
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	GPIOA->MODER &= ~(GPIO_MODER_MODER0);

	// Enable SYSCFG clock for EXTI
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	 // Connect PA0 to EXTI0
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    // Enable EXTI0 interrupt
    EXTI->IMR |= EXTI_IMR_MR0;

    // Configure EXTI0 for falling edge trigger (assuming switch connects to ground when pressed)
    EXTI->FTSR |= EXTI_FTSR_TR0;
	  // Enable EXTI0 interrupt in NVIC
	NVIC_EnableIRQ(EXTI0_IRQn);
	__enable_irq();

}

//gpio interrupt handler
void EXTI0_IRQHandler(void)
{
	switch_toggle = !switch_toggle;
	//delay to handle switch debounce
	delay(600);
	EXTI->PR |= EXTI_PR_PR0;

}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        // Clear update interrupt flag
    	toggle_led = !toggle_led;
        TIM2->SR &= ~TIM_SR_UIF;
        if(switch_toggle)
        {
        	/* toggle green and blue led */
            if(toggle_led)
            {
                GPIOD->ODR &= ~(BLED_PIN_ODR_MSK);
        	    GPIOD->ODR |= GLED_PIN_ODR_MSK;

            }
            else
            {
                GPIOD->ODR &= ~(GLED_PIN_ODR_MSK);
        	    GPIOD->ODR |= BLED_PIN_ODR_MSK;
            }
        }
        else
        {
        	/* keep the leds om*/
    	    GPIOD->ODR |= GLED_PIN_ODR_MSK;
    	    GPIOD->ODR |= BLED_PIN_ODR_MSK;
        }
    }
}

void timer2_Int_set()
{
	/* prescalar and auto reload register */
    TIM2->PSC = 639;
    TIM2->ARR = 26999;
    // timer interrupt enable
    TIM2->DIER = TIM_DIER_UIE;

    NVIC_EnableIRQ(TIM2_IRQn);
    // start timer
    TIM2->CR1 = TIM_CR1_CEN;
}

void timer2_init()
{
    RCC->APB1ENR |=RCC_APB1ENR_TIM2EN;
}
void init_led()
{
	//Send the clock to port D
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    // Configure PD12 and PD15 as outputs
    GPIOD->MODER &= ~(3U << (2 * GLED_PIN));
    GPIOD->MODER |= (1U << (2 * GLED_PIN));

    GPIOD->MODER &= ~(3U << (2 * BLED_PIN));
    GPIOD->MODER |= (1U << (2 * BLED_PIN));

}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    init_led();

    switch_init();

    timer2_init();

    timer2_Int_set();


    while (1)
    {

    }
}

