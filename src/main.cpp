#include "stm32g0xx.h"

#include "exti.hpp"
#include "systemcoreclockset.hpp"
#include "usart.hpp"

#include "stepper.hpp"

const uint8_t USART1_NVIC_PRIORITY = 5;
const uint8_t PB3_NVIC_PRIORITY = 2;
const uint8_t PA4_PA5_NVIC_PRIORITY = 3;
const uint8_t TIM2_NVIC_PRIORITY = 4;

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

#define MSFM_MRK_MODE   0x01 // if 1 - msfm1 else msfm2
#define MSFM_MRK_READY  0x02 // if 1 ready else no ready
#define MSFM_MRK_MOVING 0x04 // if 1 moving
#define MSFM_MRK_FINISH 0x08 // if 1 finish
#define MSFM_MRK_EN     0x80 // enable status
uint8_t msfm_mrk = 0b00000001;

Stepper stepper_(TIM2_NVIC_PRIORITY);

extern "C"
{
void __attribute__((interrupt, used)) USART1_IRQHandler(void)
{
    if (READ_BIT(USART1->ISR, USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE)
    {
        if (READ_BIT(msfm_mrk, MSFM_MRK_EN) != MSFM_MRK_EN)
        {
            (void)USART1->RDR;
            return;
        }

        // uint32_t primask = __get_PRIMASK(); // get the interrupt status
        // __disable_irq();           // disable IRQs, this will modify PRIMASK

        if (READ_BIT(msfm_mrk, MSFM_MRK_MODE) == MSFM_MRK_MODE)
        {
            msfm1._frame[msfm_i++] = (USART1->RDR + 1);
        }
        else
        {
            msfm2._frame[msfm_i++] = (USART1->RDR + 1);
        }

        if (msfm_i == 12)
        {
            msfm_i = 0;
            if (READ_BIT(msfm_mrk, MSFM_MRK_MODE) == MSFM_MRK_MODE)
            {
                CLEAR_BIT(msfm_mrk, MSFM_MRK_MODE);
            }
            else
            {
                SET_BIT(msfm_mrk, MSFM_MRK_MODE);
            }
        }

        SET_BIT(msfm_mrk, MSG_STATUS);
        // __set_PRIMASK( primask ); // restore interrupt state

        return;
    }
}

void __attribute__((interrupt, used)) EXTI4_15_IRQHandler(void)
{
    //// EN
    // Включение работы
    if (EXTI->RPR1 & EXTI_RPR1_RPIF4)
    {
        uint32_t primask = __get_PRIMASK(); // get the interrupt status
        __disable_irq();                    // disable IRQs, this will modify PRIMASK

        // TODO: Включение работы
        SET_BIT(msfm_mrk, MSFM_MRK_EN);
        // on exti    | stop-end      | SELECT        |
        EXTI->IMR1 |= (EXTI_IMR1_IM3 | EXTI_IMR1_IM5);
        GPIOA->ODR |= GPIO_ODR_OD3;
        EXTI->RPR1 = EXTI_RPR1_RPIF4;

        __set_PRIMASK(primask); // restore interrupt state
        return;
    }

    // Остановка работы при переводе ENABLE в 0 останавливается работа и перемещение
    if (EXTI->FPR1 & EXTI_FPR1_FPIF4)
    {
        uint32_t primask = __get_PRIMASK(); // get the interrupt status
        __disable_irq();                    // disable IRQs, this will modify PRIMASK

        CLEAR_BIT(msfm_mrk, MSFM_MRK_EN);
        USART1->CR1 &= ~USART_CR1_UE;
        TIM2->CR1 &= ~TIM_CR1_CEN;
        // off         | step         | dir          | en           | STATUSx      |
        GPIOA->ODR &= ~(GPIO_ODR_OD1 | GPIO_ODR_OD2 | GPIO_ODR_OD3 | GPIO_ODR_OD6);
        // off exti    | stop-end      | SELECT        |
        EXTI->IMR1 &= ~(EXTI_IMR1_IM3 | EXTI_IMR1_IM5);

        EXTI->FPR1 = EXTI_FPR1_FPIF4;

        __set_PRIMASK(primask); // restore interrupt state
        return;
    }

    //// SELECT
    if (EXTI->RPR1 & EXTI_RPR1_RPIF5)
    {
        if (READ_BIT(msfm_mrk, MSFM_MRK_EN) != MSFM_MRK_EN)
        {
            EXTI->RPR1 = EXTI_RPR1_RPIF5;
            return;
        }

        // TODO: Выбор и подготовка к приему
        USART1->CR1 |= USART_CR1_UE;
        EXTI->RPR1 = EXTI_RPR1_RPIF5;
        return;
    }

    if (EXTI->FPR1 & EXTI_FPR1_FPIF5)
    {
        if (READ_BIT(msfm_mrk, MSFM_MRK_EN) != MSFM_MRK_EN)
        {
            EXTI->FPR1 = EXTI_FPR1_FPIF5;
            return;
        }

        // TODO: Старт передачи
        USART1->CR1 &= ~USART_CR1_UE;
        SET_BIT(msfm_mrk, MSFM_MRK_MOVING);
        EXTI->FPR1 = EXTI_FPR1_FPIF5;
        return;
    }
}

void __attribute__((interrupt, used)) EXTI2_3_IRQHandler(void)
{
    if (EXTI->RPR1 & EXTI_RPR1_RPIF3)
    {
        if (READ_BIT(msfm_mrk, MSFM_MRK_EN) != MSFM_MRK_EN)
        {
            EXTI->RPR1 = EXTI_RPR1_RPIF3;
            return;
        }

        // TODO: Остановить передачу и потому что уперлись в концевик
        EXTI->RPR1 = EXTI_RPR1_RPIF3;
        return;
    }

    if (EXTI->FPR1 & EXTI_FPR1_FPIF3)
    {
        if (READ_BIT(msfm_mrk, MSFM_MRK_EN) != MSFM_MRK_EN)
        {
            EXTI->FPR1 = EXTI_FPR1_FPIF3;
            return;
        }

        // TODO: Остановить передачу и потому что уперлись в концевик
        EXTI->FPR1 = EXTI_FPR1_FPIF3;
        return;
    }
}

void __attribute__((interrupt, used)) TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;
        if (READ_BIT(msfm_mrk, MSFM_MRK_EN) != MSFM_MRK_EN)
        {
            TIM2->CR1 &= ~TIM_CR1_CEN;
            return;
        }

        GPIOA->BSRR = stepper_.m_step_pin;
        (void)GPIOA->BSRR;
        GPIOA->BSRR = (uint32_t)stepper_.m_step_pin << 16;

        switch (stepper_.m_state)
        {
            case StepperState::STEPPER_ACCEL:
                stepper_.m_current_speed += stepper_.m_speed_increment;
                if (++stepper_.m_step_count >= stepper_.m_accel_steps)
                {
                    stepper_.m_step_count = 0;
                    stepper_.m_state = StepperState::STEPPER_CRUISE;
                }
                break;

            case StepperState::STEPPER_CRUISE:
                if (++stepper_.m_step_count >= stepper_.m_cruise_steps)
                {
                    stepper_.m_step_count = 0;
                    stepper_.m_state = StepperState::STEPPER_DECEL;
                }
                break;

            case StepperState::STEPPER_DECEL:
                stepper_.m_current_speed -= stepper_.m_speed_increment;
                if (++stepper_.m_step_count >= stepper_.m_accel_steps)
                {
                    stepper_.m_state = StepperState::STEPPER_IDLE;
                    GPIOA->BSRR = (uint32_t)stepper_.m_dir_pin << 16;
                    TIM2->CR1 &= ~TIM_CR1_CEN;
                    GPIOA->ODR &= ~GPIO_ODR_OD6; // off STATUSx
                }
                break;

            default:
                break;
        }

        if (stepper_.m_state != StepperState::STEPPER_IDLE)
        {
            TIM2->CR1 &= ~TIM_CR1_CEN;
            uint32_t period = (uint32_t)(SystemCoreClock / (stepper_.m_current_speed * 2));
            TIM2->ARR = period - 1;
            TIM2->CNT = 0;
            TIM2->CR1 |= TIM_CR1_CEN;
        }
    }
}
}

int main(void)
{
    SystemCoreClockSet64MHz();
    USART usart_(115200, USART1_NVIC_PRIORITY);
    EXIT_init(PB3_NVIC_PRIORITY, PA4_PA5_NVIC_PRIORITY);

    GPIOA->MODER &= ~GPIO_MODER_MODE0;
    GPIOA->MODER |= GPIO_MODER_MODE0_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT0;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0;

    // STATUS
    GPIOA->MODER &= ~GPIO_MODER_MODE6;
    GPIOA->MODER |= GPIO_MODER_MODE6_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT6;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD6;

    // EN_motor
    GPIOA->MODER &= ~GPIO_MODER_MODE3;
    GPIOA->MODER |= GPIO_MODER_MODE3_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD3;

    stepper_.moving(20000, 20000, 5000);

    uint32_t *ptr = nullptr;
    while (1)
    {
        if (READ_BIT(msfm_mrk, MSG_STATUS) == MSG_STATUS)
        {
            CLEAR_BIT(msfm_mrk, MSG_STATUS);
            ptr = ((READ_BIT(msfm_mrk, MSFM_MRK_MODE)) ? &msfm2._settings[0] : &msfm1._settings[0]);
            stepper_.moving(*(ptr + 0), *(ptr + 1), *(ptr + 2));
            GPIOA->ODR |= GPIO_ODR_OD6;
            while (READ_BIT(msfm_mrk, MSFM_MRK_MOVING) != MSFM_MRK_MOVING)
                ;
            CLEAR_BIT(msfm_mrk, MSFM_MRK_MOVING);
            stepper_.launch();
        }
    }
}
