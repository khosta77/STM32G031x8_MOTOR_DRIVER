#include "stm32g0xx.h"

#include "stepper.hpp"

Stepper::Stepper(const uint8_t priority)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    GPIOA->MODER &= ~GPIO_MODER_MODE1; // step
    GPIOA->MODER |= GPIO_MODER_MODE1_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT1;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD1;
    m_step_pin = GPIO_PIN_1;

    GPIOA->MODER &= ~GPIO_MODER_MODE2; // dir
    GPIOA->MODER |= GPIO_MODER_MODE2_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD2;
    m_dir_pin = GPIO_PIN_2;

    RCC->APBENR1 |= RCC_APBENR1_TIM2EN;
    TIM2->PSC = 0;
    TIM2->ARR = 0;
    TIM2->DIER |= TIM_DIER_UIE;

    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, priority);
}

void Stepper::moving(const uint32_t max_speed, const uint32_t acceleration, const uint32_t target_pos)
{
    m_max_speed = max_speed;
    m_acceleration = acceleration;
    m_target_pos = target_pos;

    // Направление
    GPIOA->BSRR = m_target_pos > 0 ? m_dir_pin : (uint32_t)m_dir_pin << 16;

    // Расчёт параметров
    uint32_t total_steps = fabs(m_target_pos);
    m_accel_steps = (m_max_speed * m_max_speed) / (2 * m_acceleration);
    m_accel_steps = m_accel_steps > total_steps / 2 ? total_steps / 2 : m_accel_steps;
    m_cruise_steps = total_steps - 2 * m_accel_steps;

    m_current_speed = sqrtf(2 * m_acceleration);
    m_speed_increment = (m_max_speed - m_current_speed) / m_accel_steps;

    m_step_count = 0;
    m_state = StepperState::STEPPER_ACCEL;

    // Запуск таймера
    uint32_t period = (uint32_t)(SystemCoreClock / (m_current_speed * 2));
    TIM2->ARR = period - 1;
    TIM2->CNT = 1;
}

void Stepper::launch()
{
    TIM2->CR1 |= TIM_CR1_CEN;
}

bool Stepper::isfinish()
{
    return true;
}
