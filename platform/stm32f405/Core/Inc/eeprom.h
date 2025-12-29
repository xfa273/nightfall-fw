/*
 * eeprom.h
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define EEPROM_START_ADDRESS 0x80E0000 // EEPROM emulation start address

HAL_StatusTypeDef eeprom_enable_write(void);

HAL_StatusTypeDef eeprom_disable_write(void);

HAL_StatusTypeDef eeprom_write_halfword(uint32_t, uint16_t);

HAL_StatusTypeDef eeprom_write_word(uint32_t, uint32_t);

uint16_t eeprom_read_halfword(uint32_t);

uint32_t eeprom_read_word(uint32_t);

#endif /* INC_EEPROM_H_ */
