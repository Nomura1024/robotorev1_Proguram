/*
 * flash.h
 *
 *  Created on: 2021/10/23
 *      Author: Owner
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "main.h"
#include "stdbool.h"
#include "string.h"

bool Flash_clear();
bool Flash_crear2();

uint16_t* Flash_load();


bool Flash_store();




#endif /* INC_FLASH_H_ */
