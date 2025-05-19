/*
 * uart_strings.c
 *
 *  Created on: May 18, 2025
 *      Author: luka
 */

#include "main.h"

volatile uint32_t NewReceivedChars, NewReceivedChars_2;
volatile uint8_t cnt = 0, cnt_2 = 0;
volatile uint8_t last_size, last_size_2 = 0;



void UART_String_Handling_CH1() {
	if (cnt != last_size) {
			memset(string_buffer_1, 0, RX_BUFFER_CH1_SIZE);
			if (cnt > last_size) {
				NewReceivedChars = cnt - last_size;
				memset(string_buffer_1,0,RX_BUFFER_CH1_SIZE);
					for (uint8_t i=0; i < NewReceivedChars; i++) {
						string_buffer_1[i] = rx_buffer_ch1[i+last_size];
					}

			}
			else {
				NewReceivedChars = RX_BUFFER_CH1_SIZE - last_size;
			      for (uint8_t i = 0; i < NewReceivedChars; i++)
			      {
			        string_buffer_1[i] = rx_buffer_ch1[last_size + i];
			      }
			      if (cnt > 0) {
			    	  for (uint8_t i = 0; i < cnt; i++)
			    	  {
			    	    string_buffer_1[NewReceivedChars + i] = rx_buffer_ch1[i];
			    	  }
			    	  NewReceivedChars += cnt;
			      }
			}

		}
		last_size = cnt;
}
void UART_String_Handling_CH2() {
	if (cnt_2 != last_size_2) {
			if (cnt_2 > last_size_2) {
				NewReceivedChars_2 = cnt_2 - last_size_2;
				memset(string_buffer_2,0,RX_BUFFER_CH2_SIZE);
					for (uint8_t i=0; i < NewReceivedChars_2; i++) {
						string_buffer_2[i] = rx_buffer_ch2[i+last_size_2];
					}

			}
			else {
				NewReceivedChars_2 = RX_BUFFER_CH2_SIZE - last_size_2;
			      for (uint8_t i = 0; i < NewReceivedChars_2; i++)
			      {
			        string_buffer_2[i] = rx_buffer_ch2[last_size_2 + i];
			      }
			      if (cnt_2 > 0) {
			    	  for (uint8_t i = 0; i < cnt_2; i++)
			    	  {
			    	    string_buffer_2[NewReceivedChars_2 + i] = rx_buffer_ch2[i];
			    	  }
			    	  NewReceivedChars_2 += cnt_2;
			      }
			}

		}
		last_size_2 = cnt_2;
}


