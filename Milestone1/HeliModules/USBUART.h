#ifndef USBUART_H_
#define USBUART_H_

// *******************************************************
//
// USBUART.h
//
// Support for use of the USB UART peripheral on the Tiva
// C series microcontroller.
//
// P.J. Bones UCECE
// Last modified:   9.4.2019
//
// *******************************************************

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/uart.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"

//*****************************************************************************
// Constants
//*****************************************************************************
//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE               9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

//********************************************************
// initUSB_UART - 8 bits, 1 stop bit, no parity
//********************************************************
void
initUSB_UART (void);

//**********************************************************************
// Transmit a string via UART0
//**********************************************************************
void
UARTSend (char *pucBuffer);

#endif /*USBUART_H_*/
