/*
 * serial.c
 *
 *  Created on: Apr 19, 2022
 *      Author:
 */

#include "main.h"


#include "serial.h"
#include <string.h>
#include "stdin.h"

#define SERIAL_TIMEOUT_MS    (50)


/* Private function declarations */
static void stdin_callback(uint8_t *u8p_buffer, uint16_t u16_size);

/* Private variables */
static stdin_t hstdin;
static uint8_t u8arr_buffer[UART_BUF_SZ];

extern UART_HandleTypeDef 	huart2;

static uint8_t u8arr_uart[UART_BUF_SZ];
static uint8_t u8idx;

/* Public functions definitions */
void serial_init(void)
{
  /* initialize serial */
  stdin_init(&hstdin, &huart2, u8arr_buffer, UART_BUF_SZ);
  stdin_set_callback(&hstdin, stdin_callback);
  stdin_start(&hstdin);
}

void serial_handlerUART(void)
{
  stdin_irq_uart(&hstdin);
}

void serial_handlerDMA(void)
{
  stdin_irq_dma(&hstdin);
}

/* Private function definitions */
static void stdin_callback(uint8_t *u8p_buffer, uint16_t u16_size)
{
	//printf("UART RX Handler\r\n\n");

	memcpy(&u8arr_uart[u8idx], u8p_buffer, u16_size);
	u8idx += u16_size;

	if((u8p_buffer[u16_size - 1] == '\n')&&(u8p_buffer[u16_size - 2]== '\r'))
	{
		uartProcessing (u8arr_uart, u8idx);
		memset(u8arr_uart, 0, UART_BUF_SZ);
		u8idx = 0;
	}


}

