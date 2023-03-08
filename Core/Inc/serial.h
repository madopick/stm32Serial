/*
 * serial.h
 *
 *  Created on: Apr 19, 2022
 *      Author:
 */

#ifndef SRC_SERIAL_SERIAL_H_
#define SRC_SERIAL_SERIAL_H_

#include "main.h"
#include <stdint.h>

/*******************************************************************************
	Macro Definition
  *****************************************************************************/
#define UART_BUF_SZ (512)

/*******************************************************************************
	Extern Function Declaration
  *****************************************************************************/
void serial_init(void);
void serial_handlerUART(void);
void serial_handlerDMA(void);


#endif /* SRC_SERIAL_SERIAL_H_ */
