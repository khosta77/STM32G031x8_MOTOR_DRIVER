#ifndef USART_HPP_
#define USART_HPP_

#include "stm32g0xx.h"

class USART
{
public:
    USART( const uint32_t baud, const uint8_t priotity );
    ~USART() = default;

    void writeb( const uint8_t data );
    void writebs( const uint8_t *ptr, const uint32_t size );
};

#endif // USART_HPP_
