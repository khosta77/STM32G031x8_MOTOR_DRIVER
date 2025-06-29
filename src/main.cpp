#include "stm32g0xx.h"

void PA4_Output_Init(void) {
  // 1. Включить тактирование GPIOA
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  // 2. Настроить PA4 как выход push-pull
  GPIOA->MODER &= ~GPIO_MODER_MODE4_Msk;          // Очистить биты
  GPIOA->MODER |= (0b01 << GPIO_MODER_MODE4_Pos); // Output mode (01)

  // 3. Установить высокий уровень на PA4
  GPIOA->BSRR = GPIO_BSRR_BS4;
}

int main(void) {
  PA4_Output_Init();

  while (1) {
    // Бесконечный цикл
  }
}
