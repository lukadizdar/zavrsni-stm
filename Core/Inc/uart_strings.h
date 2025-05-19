/*
 * uart_strings.h
 *
 *  Created on: May 18, 2025
 *      Author: luka
 */

#ifndef INC_UART_STRINGS_H_
#define INC_UART_STRINGS_H_

void UART_String_Handling_CH1();
void UART_String_Handling_CH2();

extern volatile uint32_t NewReceivedChars, NewReceivedChars_2;
extern volatile uint8_t cnt, cnt_2;
extern volatile uint8_t last_size, last_size_2;


#endif /* INC_UART_STRINGS_H_ */
