#include "stm32g0xx.h"

#include "usart.hpp"

USART::USART( const uint32_t baud, const uint8_t priotity )
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN; /* Enable clock for GPIO Port A */
    RCC->APBENR2 |= RCC_APBENR2_USART1EN;
    RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;

    RCC->CCIPR &= ~RCC_CCIPR_USART1SEL; // clear clock selection
    // RCC->CCIPR |= RCC_CCIPR_USART1SEL_0;	// select system clock --> 16 MHz
    // RCC->CCIPR |= RCC_CCIPR_USART1SEL_1; // HSI16 --> 16 MHz

    // RCC->CCIPR &= ~RCC_CCIPR_USART1SEL;  // clear clock selection
    RCC->CCIPR |= RCC_CCIPR_USART1SEL_0; // select system clock --> 16 MHz

    SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_RMP; // remap to A9 (TX)
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA12_RMP; // remap to A10

    GPIOA->MODER &= ~GPIO_MODER_MODE9;
    GPIOA->MODER |= GPIO_MODER_MODE9_1;
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL9;
    GPIOA->AFR[1] |= 1 << GPIO_AFRH_AFSEL9_Pos;

    GPIOA->MODER &= ~GPIO_MODER_MODE10;
    GPIOA->MODER |= GPIO_MODER_MODE10_1;
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL10;
    GPIOA->AFR[1] |= 1 << GPIO_AFRH_AFSEL10_Pos; // AF1: USART pins

    USART1->CR1 &= ~USART_CR1_UE;
    USART1->CR1 &= ~( USART_CR1_M | USART_CR1_OVER8 );
    USART1->BRR = ( SystemCoreClock / baud );
    USART1->CR2 &= ~USART_CR2_STOP;
    // configuration, transmit enable
    // default 8-N-1 configuration, transmit & receive enable
    USART1->CR1 = ( USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE );
    // USART1->CR1 |= USART_CR1_UE;

    NVIC_EnableIRQ( USART1_IRQn );
    NVIC_SetPriority( USART1_IRQn, priotity );
}

void USART::writeb( const uint8_t data )
{
    // while ( ( USART1->ISR & USART_ISR_TXE_TXFNF ) == 0 );
    USART1->TDR = data;
    while ( ( USART1->ISR & USART_ISR_TC ) == 0 );
}

void USART::writebs( const uint8_t *ptr, const uint32_t size )
{
    if ( ptr == nullptr )
        return;

    for ( uint32_t i = 0; i < size; ++i )
    {
        this->writeb( *( ptr + i ) );
    }
}

