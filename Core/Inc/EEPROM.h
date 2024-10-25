/*
 * M34E04.h
 *
 *  Created on: Oct 5, 2024
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define EEPROM_ADDR (0b1010000 << 1)
#define EEPROM_SET_PAGE0 (0b0110110 << 1)
#define EEPROM_SET_PAGE1 (0b0110111 << 1)
#define EEPROM_READ_PAGE (0b0110110 << 1)

#endif /* INC_EEPROM_H_ */
