/*
 * AQM0802.h
 *
 *  Created on: Jun 21, 2021
 *      Author: Owner
 */

#ifndef INC_AQM0802_H_
#define INC_AQM0802_H_

#include "main.h"
#include <stdarg.h>

void lcd_cmd(uint8_t);
void lcd_data(uint8_t);
void lcd_init(void);
void lcd_clear(void);
void lcd_locate(int,int);
void lcd_print(const char *);
short lcd_printf(const char *, ...);



#endif /* INC_AQM0802_H_ */
