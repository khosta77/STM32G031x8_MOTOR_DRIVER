#include "stm32g0xx.h"

#include "exti.hpp"

void EXIT_init(const uint8_t pb3_priority, const uint8_t pa4_pa5_priority)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

    GPIOB->MODER &= ~GPIO_MODER_MODE3; // PB3 input
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD3;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD3_1; // pull-down

    GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5); // PA4, PA5 input
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD4_1 | GPIO_PUPDR_PUPD5_1; // pull-down

    EXTI->EXTICR[0] &= ~EXTI_EXTICR1_EXTI3;
    EXTI->EXTICR[0] |= EXTI_EXTICR1_EXTI3_0; // для PBx
    EXTI->EXTICR[1] &= ~(EXTI_EXTICR2_EXTI4 | EXTI_EXTICR2_EXTI5);

    EXTI->RTSR1 |= EXTI_RTSR1_RT3 | EXTI_RTSR1_RT4 | EXTI_RTSR1_RT5; // Rising trigger
    EXTI->FTSR1 |= EXTI_FTSR1_FT3 | EXTI_FTSR1_FT4 | EXTI_FTSR1_FT5; // Falling trigger

    NVIC_SetPriority(EXTI2_3_IRQn, pb3_priority);
    NVIC_EnableIRQ(EXTI2_3_IRQn);

    NVIC_SetPriority(EXTI2_3_IRQn, pb3_priority);
    NVIC_EnableIRQ(EXTI2_3_IRQn);

    EXTI->IMR1 |= EXTI_IMR1_IM3 | EXTI_IMR1_IM4 | EXTI_IMR1_IM5;
}
