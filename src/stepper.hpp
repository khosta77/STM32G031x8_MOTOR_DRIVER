#ifndef STEPPER_HPP_
#define STEPPER_HPP_

#include <cmath>
#include <cstdint>

enum class StepperState
{
    STEPPER_IDLE,
    STEPPER_ACCEL,
    STEPPER_CRUISE,
    STEPPER_DECEL
};

struct Stepper
{
    // Конфигурация
    uint32_t m_max_speed;    // steps/sec
    uint32_t m_acceleration; // steps/sec²
    int32_t m_target_pos;
    uint16_t m_step_pin;
    uint16_t m_dir_pin;

    // Состояние
    StepperState m_state;
    int32_t m_step_count;
    uint32_t m_accel_steps;
    uint32_t m_cruise_steps;
    uint32_t m_step_delay;
    float m_current_speed;
    float m_speed_increment;

    Stepper( const uint8_t priority );
    ~Stepper() = default;

    Stepper( const Stepper & ) = delete;
    Stepper( Stepper && ) = delete;
    Stepper &operator=( const Stepper & ) = delete;
    Stepper &operator=( Stepper && ) = delete;

    void moving( const uint32_t, const uint32_t, const uint32_t );
    void launch();
    bool isfinish();
};

#define GPIO_PIN_0 ( (uint16_t) 0x0001 )   /* Pin 0 selected    */
#define GPIO_PIN_1 ( (uint16_t) 0x0002 )   /* Pin 1 selected    */
#define GPIO_PIN_2 ( (uint16_t) 0x0004 )   /* Pin 2 selected    */
#define GPIO_PIN_3 ( (uint16_t) 0x0008 )   /* Pin 3 selected    */
#define GPIO_PIN_4 ( (uint16_t) 0x0010 )   /* Pin 4 selected    */
#define GPIO_PIN_5 ( (uint16_t) 0x0020 )   /* Pin 5 selected    */
#define GPIO_PIN_6 ( (uint16_t) 0x0040 )   /* Pin 6 selected    */
#define GPIO_PIN_7 ( (uint16_t) 0x0080 )   /* Pin 7 selected    */
#define GPIO_PIN_8 ( (uint16_t) 0x0100 )   /* Pin 8 selected    */
#define GPIO_PIN_9 ( (uint16_t) 0x0200 )   /* Pin 9 selected    */
#define GPIO_PIN_10 ( (uint16_t) 0x0400 )  /* Pin 10 selected   */
#define GPIO_PIN_11 ( (uint16_t) 0x0800 )  /* Pin 11 selected   */
#define GPIO_PIN_12 ( (uint16_t) 0x1000 )  /* Pin 12 selected   */
#define GPIO_PIN_13 ( (uint16_t) 0x2000 )  /* Pin 13 selected   */
#define GPIO_PIN_14 ( (uint16_t) 0x4000 )  /* Pin 14 selected   */
#define GPIO_PIN_15 ( (uint16_t) 0x8000 )  /* Pin 15 selected   */
#define GPIO_PIN_All ( (uint16_t) 0xFFFF ) /* All pins selected */

#endif // STEPPER_HPP_
