#ifndef SYSTEMCORECLOCKSET_HPP_
#define SYSTEMCORECLOCKSET_HPP_

#include "stm32g0xx.h"

/*
 * https://github.com/olikraus/stm32g031/blob/main/common/sys_util.c
 * */
void SystemCoreClockSet64MHz( void )
{
    if ( ( RCC->CFGR & RCC_CFGR_SWS ) != 0 )
    {
        RCC->CR &= (uint32_t) ( ~RCC_CR_HSIDIV );
        RCC->CR |= RCC_CR_HSION;
        while ( ( RCC->CR & RCC_CR_HSIRDY ) == 0 )
            ;

        RCC->CFGR &= (uint32_t) ( ~RCC_CFGR_SW );
        while ( ( RCC->CFGR & RCC_CFGR_SWS ) != 0 )
            ;
    }

    RCC->CR &= (uint32_t) ( ~RCC_CR_PLLON );
    while ( ( RCC->CR & RCC_CR_PLLRDY ) != 0 )
        ;

    FLASH->ACR &= (uint32_t) ( ~FLASH_ACR_LATENCY );
    FLASH->ACR |= FLASH_ACR_LATENCY_1;

    while ( ( FLASH->ACR & FLASH_ACR_LATENCY ) != FLASH_ACR_LATENCY_1 )
        ;

    FLASH->ACR |= FLASH_ACR_PRFTEN;

    FLASH->ACR |= FLASH_ACR_ICEN;

    RCC->PLLCFGR = ( 3 << RCC_PLLCFGR_PLLR_Pos ) | RCC_PLLCFGR_PLLREN |
                   ( 1 << RCC_PLLCFGR_PLLQ_Pos ) | RCC_PLLCFGR_PLLQEN |
                   ( 3 << RCC_PLLCFGR_PLLP_Pos ) | RCC_PLLCFGR_PLLPEN |
                   ( 32 << RCC_PLLCFGR_PLLN_Pos ) |
                   ( 1 << RCC_PLLCFGR_PLLM_Pos ) | RCC_PLLCFGR_PLLSRC_1;

    RCC->CR |= RCC_CR_PLLON;

    while ( ( RCC->CR & RCC_CR_PLLRDY ) == 0 )
        ;

    RCC->CFGR |= (uint32_t) ( RCC_CFGR_SW_1 );
    while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_1 )
        ;

    RCC->CCIPR |= RCC_CCIPR_TIM1SEL;
    SystemCoreClockUpdate();

    // Включение MCO на PA8
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    GPIOA->MODER &= ~GPIO_MODER_MODE8_Msk;
    GPIOA->MODER |= GPIO_MODER_MODE8_1;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT8_Msk;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD8_Msk;
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL8_Msk;
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED8;

    RCC->CFGR &= ~RCC_CFGR_MCOPRE_Msk;
    RCC->CFGR |= RCC_CFGR_MCOPRE_2 | RCC_CFGR_MCOPRE_1;
    RCC->CFGR &= ~RCC_CFGR_MCOSEL_Msk;
    RCC->CFGR |= RCC_CFGR_MCOSEL_0;
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APBENR2 |= RCC_APBENR2_USART1EN;

    // На выходе должно быть 1 MHz
}

#endif // SYSTEMCORECLOCKSET_HPP_
