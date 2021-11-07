/*
 *  EADOGS164.h
 *
 *  Created on: Sep 2, 2020
 *  Author: NguyenPham
 */

#ifndef EADOGS164_H_
#define EADOGS164_H_

#include "stm32l0xx_hal.h" // Stm hal lib
#include "i2c.h"           // I2c lib
#include <string>

class EADOGS164
{
public:
    void init(bool backlight, bool view_angle, bool cursor_show); //start lcd
    
    void clear(); //clear lcd
    void home();  //move cursor home
    
    void backLight(bool status); //backlight on off
    
    void setCursor(uint8_t col, uint8_t row); 	//set lcd cursor
    void setContrast(uint8_t contrast); 		//set contrast
	void drawString(uint8_t col, uint8_t row, const char *str);  //send string to the lcd
    
private:
    void reset(); //reset lcd
	void send_cmd(uint8_t cmd); //send command
    void send_data(uint8_t data); //send data
};

#endif /* EADOGS164_H_ */
