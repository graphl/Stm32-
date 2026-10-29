#include "board.h"
#include "stm32f1xx.h"
#include <stddef.h>

#define MOTOR_PWM_HZ       20000UL
#define LED_STRIP_PWM_HZ  800000UL
#define USART1_BAUD        115200UL
#define DEBOUNCE_SAMPLES        3U
#define UART_ECHO_BUFFER_SIZE  128U
#define UART_ECHO_BUFFER_MASK  (UART_ECHO_BUFFER_SIZE - 1U)

typedef struct
{
    bool stable;
    uint8_t count;
} DebounceState;

static DebounceState dialog_button;
static DebounceState function_button;
static DebounceState touch_input;
static volatile uint8_t uart_echo_buffer[UART_ECHO_BUFFER_SIZE];
static volatile uint16_t uart_echo_head;
static volatile uint16_t uart_echo_tail;
static volatile bool uart_echo_enabled;

volatile uint32_t g_uart_echo_count;
volatile uint32_t g_uart_echo_dropped;

static void GPIO_Config(GPIO_TypeDef *gpio, uint8_t pin, uint32_t mode);
static void MotorPWM_Init(void);
static void LEDStripTimer_Init(void);
static void ADC1_Init(void);
static void USART1_Init(void);
static bool Debounce_Update(DebounceState *state, bool sample, bool *changed);
static uint16_t Motor_DutyFromSpeed(int16_t speed);

void Board_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN |
                    RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN |
                    RCC_APB2ENR_IOPCEN |
                    RCC_APB2ENR_ADC1EN |
                    RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;

    /* PB15/nSLEEP is made safe before any motor PWM pin becomes an output. */
    GPIO_Config(GPIOB, 15U, 0x2U); /* Push-pull output, 2 MHz. */
    GPIOB->BRR = GPIO_BRR_BR15;

    GPIO_Config(GPIOC, 13U, 0x2U);
    Board_StatusLedSet(false);

    /* PA0/PA1: ADC1_IN0/ADC1_IN1, analog input. */
    GPIO_Config(GPIOA, 0U, 0x0U);
    GPIO_Config(GPIOA, 1U, 0x0U);

    /* PB12/PB13: active-low buttons with internal pull-ups. */
    GPIO_Config(GPIOB, 12U, 0x8U);
    GPIO_Config(GPIOB, 13U, 0x8U);
    GPIOB->BSRR = GPIO_BSRR_BS12 | GPIO_BSRR_BS13;

    /* PB14: TTP223 push-pull output, active high, both-edge EXTI14. */
    GPIO_Config(GPIOB, 14U, 0x4U); /* Floating input. */
    AFIO->EXTICR[3] = (AFIO->EXTICR[3] & ~(0xFUL << 8U)) | (0x1UL << 8U);
    EXTI->PR = EXTI_PR_PR14;
    EXTI->RTSR |= EXTI_RTSR_TR14;
    EXTI->FTSR |= EXTI_FTSR_TR14;
    EXTI->IMR |= EXTI_IMR_MR14;
    NVIC_SetPriority(EXTI15_10_IRQn, 6U);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    MotorPWM_Init();
    LEDStripTimer_Init();
    ADC1_Init();
    USART1_Init();

    dialog_button.stable = ((GPIOB->IDR & GPIO_IDR_IDR12) == 0U);
    function_button.stable = ((GPIOB->IDR & GPIO_IDR_IDR13) == 0U);
    touch_input.stable = ((GPIOB->IDR & GPIO_IDR_IDR14) != 0U);

    /* All four compare registers are zero before the bridge is enabled. */
    GPIOB->BSRR = GPIO_BSRR_BS15;
}

static void MotorPWM_Init(void)
{
    const uint32_t period = (SystemCoreClock / MOTOR_PWM_HZ) - 1UL;

    TIM3->PSC = 0U;
    TIM3->ARR = period;
    TIM3->CCR1 = 0U;
    TIM3->CCR2 = 0U;
    TIM3->CCR3 = 0U;
    TIM3->CCR4 = 0U;
    TIM3->CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM3->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 |
                  TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->EGR = TIM_EGR_UG;

    /* PA6/PA7/PB0/PB1: TIM3 CH1..CH4, AF push-pull, 50 MHz. */
    GPIO_Config(GPIOA, 6U, 0xBU);
    GPIO_Config(GPIOA, 7U, 0xBU);
    GPIO_Config(GPIOB, 0U, 0xBU);
    GPIO_Config(GPIOB, 1U, 0xBU);

    TIM3->CR1 |= TIM_CR1_CEN;
}

static void LEDStripTimer_Init(void)
{
    const uint32_t period = (SystemCoreClock / LED_STRIP_PWM_HZ) - 1UL;

    /* PB8: TIM4_CH3. CCR3=0 keeps DIN low until a DMA waveform is supplied. */
    GPIO_Config(GPIOB, 8U, 0xBU);
    TIM4->PSC = 0U;
    TIM4->ARR = period;
    TIM4->CCR3 = 0U;
    TIM4->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
    TIM4->CCER = TIM_CCER_CC3E;
    TIM4->CR1 = TIM_CR1_ARPE;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->CR1 |= TIM_CR1_CEN;
}

static void ADC1_Init(void)
{
    volatile uint32_t delay;

    /* ADC clock is PCLK2/6 = 12 MHz at the normal 72 MHz system clock. */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;
    ADC1->SMPR2 = ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_0 |
                  ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_0; /* 55.5 cycles. */
    ADC1->CR2 = ADC_CR2_ADON;
    for (delay = 0U; delay < 1000U; ++delay)
    {
        __NOP();
    }
    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while ((ADC1->CR2 & ADC_CR2_RSTCAL) != 0U)
    {
    }
    ADC1->CR2 |= ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0U)
    {
    }
    ADC1->CR2 |= ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL;
}

static void USART1_Init(void)
{
    GPIO_Config(GPIOA, 9U, 0xBU);  /* USART1 TX, AF push-pull. */
    GPIO_Config(GPIOA, 10U, 0x4U); /* USART1 RX, floating input. */

    USART1->BRR = (SystemCoreClock + (USART1_BAUD / 2UL)) / USART1_BAUD;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    NVIC_SetPriority(USART1_IRQn, 6U);
    Board_USART1SetEcho(true);
}

void Board_MotorSet(int16_t left_speed, int16_t right_speed)
{
    const uint16_t left = Motor_DutyFromSpeed(left_speed);
    const uint16_t right = Motor_DutyFromSpeed(right_speed);

    if (left_speed > 0)
    {
        TIM3->CCR1 = left;
        TIM3->CCR2 = 0U;
    }
    else if (left_speed < 0)
    {
        TIM3->CCR1 = 0U;
        TIM3->CCR2 = left;
    }
    else
    {
        TIM3->CCR1 = 0U;
        TIM3->CCR2 = 0U;
    }

    if (right_speed > 0)
    {
        TIM3->CCR3 = right;
        TIM3->CCR4 = 0U;
    }
    else if (right_speed < 0)
    {
        TIM3->CCR3 = 0U;
        TIM3->CCR4 = right;
    }
    else
    {
        TIM3->CCR3 = 0U;
        TIM3->CCR4 = 0U;
    }
}

void Board_MotorCoast(void)
{
    TIM3->CCR1 = 0U;
    TIM3->CCR2 = 0U;
    TIM3->CCR3 = 0U;
    TIM3->CCR4 = 0U;
}

void Board_MotorBrake(void)
{
    const uint32_t high = TIM3->ARR + 1UL;
    TIM3->CCR1 = high;
    TIM3->CCR2 = high;
    TIM3->CCR3 = high;
    TIM3->CCR4 = high;
}

void Board_MotorSleep(bool sleep)
{
    if (sleep)
    {
        Board_MotorCoast();
        GPIOB->BRR = GPIO_BRR_BR15;
    }
    else
    {
        GPIOB->BSRR = GPIO_BSRR_BS15;
    }
}

static uint16_t Motor_DutyFromSpeed(int16_t speed)
{
    int32_t magnitude = speed;

    if (magnitude < 0)
    {
        magnitude = -magnitude;
    }
    if (magnitude > 1000)
    {
        magnitude = 1000;
    }
    return (uint16_t)(((TIM3->ARR + 1UL) * (uint32_t)magnitude) / 1000UL);
}

uint16_t Board_ADCRead(uint8_t channel)
{
    if (channel > 1U)
    {
        return 0U;
    }

    ADC1->SQR3 = channel;
    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while ((ADC1->SR & ADC_SR_EOC) == 0U)
    {
    }
    return (uint16_t)ADC1->DR;
}

void Board_USART1WriteByte(uint8_t byte)
{
    while ((USART1->SR & USART_SR_TXE) == 0U)
    {
    }
    USART1->DR = byte;
}

bool Board_USART1ReadByte(uint8_t *byte)
{
    if ((byte == NULL) || ((USART1->SR & USART_SR_RXNE) == 0U))
    {
        return false;
    }
    *byte = (uint8_t)USART1->DR;
    return true;
}

void Board_USART1SetEcho(bool enable)
{
    NVIC_DisableIRQ(USART1_IRQn);
    USART1->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_TXEIE);
    uart_echo_head = 0U;
    uart_echo_tail = 0U;
    uart_echo_enabled = enable;

    if (enable)
    {
        /* Read SR/DR once so a stale RX/error condition cannot trigger first. */
        volatile uint32_t discard = USART1->SR;
        discard = USART1->DR;
        (void)discard;
        NVIC_ClearPendingIRQ(USART1_IRQn);
        USART1->CR1 |= USART_CR1_RXNEIE;
        NVIC_EnableIRQ(USART1_IRQn);
    }
}

void Board_StatusLedToggle(void)
{
    GPIOC->ODR ^= GPIO_ODR_ODR13;
}

void Board_StatusLedSet(bool on)
{
    if (on)
    {
        GPIOC->BRR = GPIO_BRR_BR13; /* Common Blue Pill LED is active low. */
    }
    else
    {
        GPIOC->BSRR = GPIO_BSRR_BS13;
    }
}

uint32_t Board_InputScan10ms(void)
{
    uint32_t events = 0U;
    bool changed;
    bool state;

    state = Debounce_Update(&dialog_button,
                            (GPIOB->IDR & GPIO_IDR_IDR12) == 0U,
                            &changed);
    if (changed)
    {
        events |= state ? BOARD_EVENT_DIALOG_PRESSED : BOARD_EVENT_DIALOG_RELEASED;
    }

    state = Debounce_Update(&function_button,
                            (GPIOB->IDR & GPIO_IDR_IDR13) == 0U,
                            &changed);
    if (changed)
    {
        events |= state ? BOARD_EVENT_FUNCTION_PRESSED : BOARD_EVENT_FUNCTION_RELEASED;
    }

    state = Debounce_Update(&touch_input,
                            (GPIOB->IDR & GPIO_IDR_IDR14) != 0U,
                            &changed);
    if (changed)
    {
        events |= state ? BOARD_EVENT_TOUCH_ACTIVE : BOARD_EVENT_TOUCH_INACTIVE;
    }

    return events;
}

static bool Debounce_Update(DebounceState *state, bool sample, bool *changed)
{
    *changed = false;
    if (sample == state->stable)
    {
        state->count = 0U;
    }
    else
    {
        ++state->count;
        if (state->count >= DEBOUNCE_SAMPLES)
        {
            state->stable = sample;
            state->count = 0U;
            *changed = true;
        }
    }
    return state->stable;
}

static void GPIO_Config(GPIO_TypeDef *gpio, uint8_t pin, uint32_t mode)
{
    volatile uint32_t *config = (pin < 8U) ? &gpio->CRL : &gpio->CRH;
    const uint32_t shift = ((uint32_t)pin & 7UL) * 4UL;
    uint32_t value = *config;

    value &= ~(0xFUL << shift);
    value |= mode << shift;
    *config = value;
}

void EXTI15_10_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR14) != 0U)
    {
        /* Debouncing is intentionally performed by Board_InputScan10ms(). */
        EXTI->PR = EXTI_PR_PR14;
    }
}

void USART1_IRQHandler(void)
{
    const uint32_t status = USART1->SR;

    if ((status & USART_SR_RXNE) != 0U)
    {
        const uint8_t byte = (uint8_t)USART1->DR;

        if (uart_echo_enabled)
        {
            const uint16_t next = (uint16_t)((uart_echo_head + 1U) & UART_ECHO_BUFFER_MASK);
            if (next != uart_echo_tail)
            {
                uart_echo_buffer[uart_echo_head] = byte;
                uart_echo_head = next;
                USART1->CR1 |= USART_CR1_TXEIE;
            }
            else
            {
                ++g_uart_echo_dropped;
            }
        }
    }
    else if ((status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
    {
        /* Reading DR after SR clears STM32F1 USART receive error flags. */
        volatile uint32_t discard = USART1->DR;
        (void)discard;
        ++g_uart_echo_dropped;
    }

    if (((USART1->CR1 & USART_CR1_TXEIE) != 0U) &&
        ((USART1->SR & USART_SR_TXE) != 0U))
    {
        if (uart_echo_tail != uart_echo_head)
        {
            USART1->DR = uart_echo_buffer[uart_echo_tail];
            uart_echo_tail = (uint16_t)((uart_echo_tail + 1U) & UART_ECHO_BUFFER_MASK);
            ++g_uart_echo_count;
        }
        else
        {
            USART1->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}
