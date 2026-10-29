#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"

volatile uint32_t g_scheduler_heartbeat = 0U;
volatile uint32_t g_fast_task_counter = 0U;
volatile uint32_t g_input_event_flags = 0U;

static void SystemClock_Config(void);
static void HeartbeatTask(void *argument);
static void CounterTask(void *argument);
static void Error_Handler(void);

int main(void)
{
    SystemClock_Config();
    Board_Init();

    if (xTaskCreate(HeartbeatTask, "heartbeat", 160U, NULL, 2U, NULL) != pdPASS)
    {
        Error_Handler();
    }

    if (xTaskCreate(CounterTask, "counter", 160U, NULL, 1U, NULL) != pdPASS)
    {
        Error_Handler();
    }

    vTaskStartScheduler();

    /* Reached only if the idle task cannot be allocated. */
    Error_Handler();
}

static void HeartbeatTask(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    (void)argument;

    for (;;)
    {
        Board_StatusLedToggle();
        ++g_scheduler_heartbeat;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500U));
    }
}

static void CounterTask(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    (void)argument;

    for (;;)
    {
        ++g_fast_task_counter;
        g_input_event_flags |= Board_InputScan10ms();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10U));
    }
}

static void SystemClock_Config(void)
{
    uint32_t timeout;

    RCC->CR |= RCC_CR_HSEON;
    timeout = 1000000U;
    while (((RCC->CR & RCC_CR_HSERDY) == 0U) && (timeout > 0U))
    {
        --timeout;
    }

    if ((RCC->CR & RCC_CR_HSERDY) != 0U)
    {
        FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

        /* HCLK=72 MHz, PCLK2=72 MHz, PCLK1=36 MHz, HSE(8 MHz)*9. */
        RCC->CFGR = RCC_CFGR_HPRE_DIV1 |
                    RCC_CFGR_PPRE2_DIV1 |
                    RCC_CFGR_PPRE1_DIV2 |
                    RCC_CFGR_PLLSRC |
                    RCC_CFGR_PLLMULL9;

        RCC->CR |= RCC_CR_PLLON;
        timeout = 1000000U;
        while (((RCC->CR & RCC_CR_PLLRDY) == 0U) && (timeout > 0U))
        {
            --timeout;
        }

        if ((RCC->CR & RCC_CR_PLLRDY) != 0U)
        {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
            {
                /* Wait until the PLL is the system clock. */
            }
        }
    }

    /* If HSE startup failed, the reset-default 8 MHz HSI remains in use. */
    SystemCoreClockUpdate();
}

void vApplicationMallocFailedHook(void)
{
    Error_Handler();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    Error_Handler();
}

void vAssertCalled(const char *file, int line)
{
    (void)file;
    (void)line;
    Error_Handler();
}

static void Error_Handler(void)
{
    __disable_irq();
    for (;;)
    {
        /* Set a breakpoint here when diagnosing startup/allocation failures. */
    }
}
