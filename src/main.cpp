#include "systemcoreclockset.hpp"
#include <cmath>
#include <stdint.h>

void usart_write_byte( const uint8_t data )
{
    /*
     *  Так тут был косяк с FIFO он не включе а флаг был настроен на то что FIFI
     * используется, нужно перенастроить
     * */
    // while ( ( USART1->ISR & USART_ISR_TXE_TXFNF ) == 0 );
    USART1->TDR = data;
    while ( ( USART1->ISR & USART_ISR_TC ) == 0 );
}

void usart_write_bytes( const uint8_t *ptr, const uint32_t size )
{
    if ( ptr == nullptr )
        return;

    for ( uint32_t i = 0; i < size; ++i )
    {
        usart_write_byte( *( ptr + i ) );
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
    USART1->CR1 |= USART_CR1_UE;

    NVIC_EnableIRQ( USART1_IRQn );
    NVIC_SetPriority( USART1_IRQn, 2 );
}

uint8_t MSG_STATUS = 0x00;
uint8_t data = 0x00;

union MotorSettingsFromMessage
{
    uint8_t _frame[12];
    uint32_t _settings[3];
    // 0 - max_speed    uint32_t
    // 1 - acceleration uint32_t
    // 2 - target_pos   uint32_t
} msfm1, msfm2;

uint8_t msfm_i = 0;

#define MSFM_MRK_MODE 0x01  // if 1 - msfm1 else msfm2
#define MSFM_MRK_READY 0x02 // if 1 ready else no ready
uint8_t msfm_mrk = 0b00000001;

extern "C"
{
void __attribute__( ( interrupt, used ) ) USART1_IRQHandler( void )
{

    if ( READ_BIT( USART1->ISR, USART_ISR_RXNE_RXFNE ) == USART_ISR_RXNE_RXFNE )
    {
        uint32_t primask;
        uint16_t end;

        primask = __get_PRIMASK(); /* get the interrupt status */
        __disable_irq();           /* disable IRQs, this will modify PRIMASK */

        if ( READ_BIT( msfm_mrk, MSFM_MRK_MODE ) == MSFM_MRK_MODE )
        {
            msfm1._frame[msfm_i++] = ( USART1->RDR + 1 );
        }
        else
        {
            msfm2._frame[msfm_i++] = ( USART1->RDR + 1 );
        }

        if ( msfm_i == 12 )
        {
            msfm_i = 0;
            if ( READ_BIT( msfm_mrk, MSFM_MRK_MODE ) == MSFM_MRK_MODE )
            {
                CLEAR_BIT( msfm_mrk, MSFM_MRK_MODE );
            }
            else
            {
                SET_BIT( msfm_mrk, MSFM_MRK_MODE );
            }
        }

        SET_BIT( msfm_mrk, MSG_STATUS );
        __set_PRIMASK( primask ); /* restore interrupt state */

        return;
    }
}
} // extern "C"

int main( void )
{
    SystemCoreClockSet64MHz();
    usart_init( 115200 );

    // Инициализация периферии
    GPIOA->MODER &= ~GPIO_MODER_MODE0;
    GPIOA->MODER |= GPIO_MODER_MODE0_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT0;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;

    GPIOA->MODER &= ~GPIO_MODER_MODE1;
    GPIOA->MODER |= GPIO_MODER_MODE1_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD1;

    /*
     * Какая то херня с прерываниями
     * */
    uint8_t buffer[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
    uint8_t *ptr = &buffer[0];
    while ( 1 )
    {
        usart_write_bytes( ptr, 12 );

        for ( uint32_t t = 0; t < 100000; ++t );

        while ( 1 )
        {
            if ( READ_BIT( msfm_mrk, MSG_STATUS ) == MSG_STATUS )
            {
                GPIOA->ODR |= GPIO_ODR_OD0;
                CLEAR_BIT( msfm_mrk, MSG_STATUS );
                if ( READ_BIT( msfm_mrk, MSFM_MRK_MODE ) )
                {
                    ptr = &msfm2._frame[0];
                }
                else
                {
                    ptr = &msfm1._frame[0];
                }
                GPIOA->ODR &= ~GPIO_ODR_OD0;
                break;
            }
        }
    }
}
