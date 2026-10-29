# STM32F103C8T6 + FreeRTOS (Keil MDK)

This is a self-contained CMSIS/FreeRTOS example for an STM32F103C8T6 (64 KiB
Flash, 20 KiB RAM). It uses the modern Arm Compiler 6 toolchain in Keil MDK.

## Open and run

1. Open `STM32F103C8_FreeRTOS.uvprojx` in Keil uVision.
2. If prompted, install the `Keil::STM32F1xx_DFP` device pack.
3. Select your ST-Link/J-Link under **Options for Target > Debug**.
4. Build (`F7`), download (`F8`), and run (`F5`).

The active-low status LED on PC13 toggles every 500 ms.
In the debugger, `g_scheduler_heartbeat` increments every 500 ms and
`g_fast_task_counter` increments every 10 ms.

## Board pin assignment

| Pin | Function | Configuration |
| --- | --- | --- |
| PA0 | Battery voltage | ADC1_IN0, analog input; external divider must keep it below 3.3 V |
| PA1 | Current/extension | ADC1_IN1, analog input |
| PA6/PA7 | Left motor AIN1/AIN2 | TIM3_CH1/CH2, 20 kHz PWM |
| PB0/PB1 | Right motor BIN1/BIN2 | TIM3_CH3/CH4, 20 kHz PWM |
| PB8 | 16-pixel strip DIN | TIM4_CH3, 800 kHz PWM base; initially held low |
| PA9/PA10 | ESP32-S3 UART | USART1 TX/RX, 115200 8N1 |
| PB12/PB13 | Dialog/function buttons | Input pull-up, active low, 30 ms software debounce |
| PB14 | TTP223 OUT | Floating EXTI14 input, active high, 30 ms software debounce |
| PB15 | DRV8833 nSLEEP | Push-pull output; held low until all motor PWM outputs are zero |
| PC13 | Status LED | Push-pull output, active low |
| PA13/PA14 | SWDIO/SWCLK | Left untouched for programming and debugging |

PB6/PB7 (I2C1), PB10/PB11 (USART3), PA11/PA12 (USB), PA2/PA3,
PA4/PA5, PB9, PA15/PB3/PB4, and PC14/PC15 remain at their reset state so the
reserved interfaces can be enabled later without a pin conflict.

Board-level APIs are declared in `Core/Inc/board.h`. The motor speed API uses
signed permille values (`-1000..1000`). `Board_InputScan10ms()` produces
debounced press/release/touch event bits; the sample task accumulates them in
`g_input_event_flags` for the future ESP32 message-protocol task.

### Temporary UART echo mode

USART1 currently runs in byte-for-byte echo mode: every byte received on PA10
is returned unchanged on PA9. RXNE/TXE interrupts and a 128-byte ring buffer are
used so continuous 115200-baud traffic does not depend on task polling latency.
Watch `g_uart_echo_count` and `g_uart_echo_dropped` in the debugger. Before a
real ESP32 protocol task starts reading USART1, call
`Board_USART1SetEcho(false)` to disable the temporary interrupt echo mode.

## Scheduler control flow

```text
Reset_Handler
  -> SystemInit
  -> main
  -> xTaskCreate (heartbeat + counter)
  -> vTaskStartScheduler
  -> SVC_Handler (start the first task)

SysTick_Handler (1 kHz)
  -> xTaskIncrementTick
  -> request PendSV when a reschedule is needed
  -> PendSV_Handler
  -> save current context / select next task / restore next context
```

FreeRTOS exception routing is configured in `Core/Inc/FreeRTOSConfig.h`.
Do not add another `SysTick_Handler`, `SVC_Handler`, or `PendSV_Handler` unless
it deliberately forwards to the corresponding FreeRTOS port handler.

## Versions and important settings

- MCU define: `STM32F103xB`
- CMSIS Device F1: `v4.3.3`
- CMSIS Core: `v5.4.0_cm3`
- FreeRTOS Kernel: `V11.3.0` LTS
- FreeRTOS port: `portable/GCC/ARM_CM3` (required for LLVM-based Arm Compiler 6)
- Tick: 1 kHz
- Kernel heap: 10 KiB using `heap_4.c`
- Maximum syscall interrupt priority: 5 (four implemented priority bits)

Any ISR that calls a FreeRTOS `...FromISR()` API must use a numerically equal
or lower urgency than priority 5 (priority values 5 through 15). Priorities 0
through 4 must not call FreeRTOS APIs.

## Hardware note

The clock setup expects an 8 MHz external crystal and runs at 72 MHz. If the
external oscillator does not start, the firmware remains on the internal 8 MHz
HSI clock and FreeRTOS still derives its tick from `SystemCoreClock`.
