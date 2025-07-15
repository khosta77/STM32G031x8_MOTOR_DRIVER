#include "systemcoreclockset.hpp"
#include <stdint.h>

void usart_write_byte( uint8_t data )
{
    /*
     *  Так тут был косяк с FIFO он не включе а флаг был настроен на то что FIFI
     * используется, нужно перенастроить
     * */
    // while ( ( USART1->ISR & USART_ISR_TXE_TXFNF ) == 0 );
    USART1->TDR = data;
    while ( ( USART1->ISR & USART_ISR_TC ) == 0 );
}

void usart_write_string( const char *s )
{
    if ( s == nullptr )
        return;
    while ( *s != '\0' )
    {
        usart_write_byte( *s++ );
    }
}

void usart_init( uint32_t baud )
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

    // USART1->BRR = 138; 	    // 16000000/115200 with 16x oversampling
    // USART1->BRR = 278; // 16000000/57600= 277.7 with 16x oversampling
    // USART1->BRR = 32000000U  // 9600;
    // USART1->CR2 = 0;
    // USART1->CR3 = 0;
    // USART1->PRESC = 0;
    USART1->CR1 |= USART_CR1_UE;

    // NVIC_EnableIRQ( USART1_IRQn );
    // NVIC_SetPriority( USART1_IRQn, 3 );
}

uint8_t MSG_STATUS = 0x00;
uint8_t data = 0x00;

#if 0
extern "C"
{
    void __attribute__( ( interrupt, used ) ) USART1_IRQHandler( void )
    {
        if ( ( USART1->ISR & USART_ISR_RXNE_RXFNE ) )
        {
            uint32_t primask;
            uint16_t end;

            primask = __get_PRIMASK(); /* get the interrupt status */
            __disable_irq(); /* disable IRQs, this will modify PRIMASK */
            data = USART1->RDR;
            MSG_STATUS = 0xFF;
            __set_PRIMASK( primask ); /* restore interrupt state */
            return;
        }
        if ( USART1->ISR &
             ( USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE ) )
        {
            // Clear error flags
            USART1->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_CMCF |
                          USART_ICR_ORECF;
        }
    }
}
#endif
int main( void )
{
    SystemCoreClockSet64MHz();
    usart_init( 115200 );

    uint8_t buffer = 0x00;
    while ( 1 )
    {
        usart_write_byte( buffer );

        for ( uint32_t t = 0; t < 10000; ++t );
        // while ( 1 )
        //{
        if ( USART1->ISR & USART_ISR_RXNE_RXFNE )
        {
            buffer = ++data;
            MSG_STATUS = 0x00;
        }
    }
}
