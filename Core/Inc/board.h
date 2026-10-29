#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

/* Debounced input event bits returned by Board_InputScan10ms(). */
#define BOARD_EVENT_DIALOG_PRESSED      (1UL << 0)
#define BOARD_EVENT_DIALOG_RELEASED     (1UL << 1)
#define BOARD_EVENT_FUNCTION_PRESSED    (1UL << 2)
#define BOARD_EVENT_FUNCTION_RELEASED   (1UL << 3)
#define BOARD_EVENT_TOUCH_ACTIVE        (1UL << 4)
#define BOARD_EVENT_TOUCH_INACTIVE      (1UL << 5)

void Board_Init(void);

void Board_StatusLedToggle(void);
void Board_StatusLedSet(bool on);

/* Signed speed is in permille: -1000 (full reverse) to +1000 (full forward). */
void Board_MotorSet(int16_t left_speed, int16_t right_speed);
void Board_MotorCoast(void);
void Board_MotorBrake(void);
void Board_MotorSleep(bool sleep);

uint16_t Board_ADCRead(uint8_t channel);

void Board_USART1WriteByte(uint8_t byte);
bool Board_USART1ReadByte(uint8_t *byte);
void Board_USART1SetEcho(bool enable);

/* Debug counters for the temporary USART1 byte-for-byte echo mode. */
extern volatile uint32_t g_uart_echo_count;
extern volatile uint32_t g_uart_echo_dropped;

/* Call once every 10 ms. Events are emitted after three identical samples. */
uint32_t Board_InputScan10ms(void);

#endif /* BOARD_H */
