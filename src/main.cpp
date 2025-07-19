#include "systemcoreclockset.hpp"
#include <cmath>
#include <stdint.h>

const uint8_t USART1_NVIC_PRIORITY = 4;
const uint8_t PB3_NVIC_PRIORITY = 2;
const uint8_t PA4_PA5_NVIC_PRIORITY = 3;

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
    NVIC_SetPriority( USART1_IRQn, USART1_NVIC_PRIORITY );
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

        // primask = __get_PRIMASK(); // get the interrupt status
        //__disable_irq();           // disable IRQs, this will modify PRIMASK

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
        //__set_PRIMASK( primask ); // restore interrupt state

        return;
    }
}

void __attribute__( ( interrupt, used ) ) EXTI4_15_IRQHandler( void )
{
    // Проверяем флаги прерываний для PA4 и PA5
    if ( EXTI->RPR1 & EXTI_RPR1_RPIF4 )
    { // PA4
        // Ваш код обработки прерывания для PA4
        EXTI->RPR1 = EXTI_RPR1_RPIF4; // Сброс флага
        GPIOA->ODR |= GPIO_ODR_OD0;
        GPIOA->ODR &= ~GPIO_ODR_OD0;
    }

    if ( EXTI->RPR1 & EXTI_RPR1_RPIF5 )
    { // PA5
        // Ваш код обработки прерывания для PA5
        EXTI->RPR1 = EXTI_RPR1_RPIF5; // Сброс флага
        GPIOA->ODR |= GPIO_ODR_OD1;
        GPIOA->ODR &= ~GPIO_ODR_OD1;
    }
}

void EXTI2_3_IRQHandler( void )
{
    // Проверяем флаг прерывания для PB3
    if ( EXTI->RPR1 & EXTI_RPR1_RPIF3 )
    { // PB3
        // Ваш код обработки прерывания для PB3
        EXTI->RPR1 = EXTI_RPR1_RPIF3; // Сброс флага
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

    ////
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    // Настраиваем PB3, PA4, PA5 на вход (пример)
    GPIOB->MODER &= ~GPIO_MODER_MODE3; // PB3 input
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD3;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD3_1; // pull-down

    GPIOA->MODER &= ~( GPIO_MODER_MODE4 | GPIO_MODER_MODE5 ); // PA4, PA5 input
    GPIOA->PUPDR &= ~( GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 );
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD4_1 | GPIO_PUPDR_PUPD5_1;

    // Настраиваем EXTI для PB3 (EXTI3)
    EXTI->EXTICR[0] &= ~EXTI_EXTICR1_EXTI3;
    EXTI->EXTICR[0] |= ( 1 << EXTI_EXTICR1_EXTI3_Pos ); // PB3 для EXTI3

    // Настраиваем EXTI для PA4 (EXTI4) и PA5 (EXTI5)
    EXTI->EXTICR[1] &= ~( EXTI_EXTICR2_EXTI4 | EXTI_EXTICR2_EXTI5 );
    // PA4 и PA5 уже подключены к EXTI4 и EXTI5 (0b0000)

    // Включаем триггер по фронту (настраивайте по необходимости)
    EXTI->RTSR1 |= EXTI_RTSR1_RT3 | EXTI_RTSR1_RT4 | EXTI_RTSR1_RT5; // Rising trigger
    // EXTI->FTSR1 |= EXTI_FTSR1_FT3 | EXTI_FTSR1_FT4 | EXTI_FTSR1_FT5; // Falling trigger

    // Разрешаем прерывания
    EXTI->IMR1 |= EXTI_IMR1_IM3 | EXTI_IMR1_IM4 | EXTI_IMR1_IM5;

    // Настраиваем приоритеты и включаем прерывания в NVIC
    NVIC_SetPriority( EXTI2_3_IRQn, PB3_NVIC_PRIORITY ); // Приоритет для EXTI2_3 (PB3)
    NVIC_EnableIRQ( EXTI2_3_IRQn );

    NVIC_SetPriority( EXTI4_15_IRQn, PA4_PA5_NVIC_PRIORITY ); // Приоритет для EXTI4_15 (PA4, PA5)
    NVIC_EnableIRQ( EXTI4_15_IRQn );
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
                CLEAR_BIT( msfm_mrk, MSG_STATUS );
                if ( READ_BIT( msfm_mrk, MSFM_MRK_MODE ) )
                {
                    ptr = &msfm2._frame[0];
                }
                else
                {
                    ptr = &msfm1._frame[0];
                }
                break;
            }
        }
    }
}
