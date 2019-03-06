/* Author: Alex St. Clair
 * Filename: Cutdown_Commission.h
 * Created: 1-9-19
 * 
 * Used to initially flash the Flash-Emulated EEPROM
 * and test hardware when a board is commissioned.
 */

#ifndef CUTDOWN_COMMISSION_H
#define CUTDOWN_COMMISSION_H

bool write_config(void);

void commission_setup(void);

void commission_loop(void);

#endif