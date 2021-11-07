/*
 *  EADOGS164.cpp
 *
 *  Created on: Sep 2, 2020
 *  Author: NguyenPham
 */

#include "EADOGS164.h"

#define SLAVE_ADDRESS_LCD 0x78   //lcd i2c address

#define lcd_cols 16     //number of column
#define lcd_rows 4      //number of row

#define _min(a,b) ({ decltype(a) _a = (a); decltype(b) _b = (b); _a < _b? _a : _b; }) //find min value
#define _max(a,b) ({ decltype(a) _a = (a); decltype(b) _b = (b); _a > _b? _a : _b; }) //find max value

//LCD HEX CODE DEFINE
#define CMD_CLEAR_DISPLAY           0X01
#define CMD_RETURN_HOME             0X02

#define CMD_8BIT_NORMAL_RE1_REV0    0x3A  //8 bit data length extension RE=1;REV=0
#define CMD_8BIT_NORMAL_RE0_IS1     0x39  //8 bit data length extension RE=0;IS1
#define CMD_8BIT_NORMAL_RE0_IS0     0X38  //8 bit data length extension RE=0;IS0

#define CMD_4_LINE_DISPLAY          0x09  //4 line display
#define CMD_BOTTOM_VIEW             0X06  //bottom view
#define CMD_TOP_VIEW                0X05  //top view
#define CMD_BIAS_SETTING            0x1E  //BS1=1

#define CMD_INTERNAL_OSC            0x1B  //BS0=1 => Bias=1/6
#define CMD_DIVIDER_ON              0X6C  //divider on

#define CMD_ADDRESS_DDRAM           0x80
#define CMD_ADDRESS_CGRAM           0x40

#define CMD_POWER_AND_SET_CONTRAST      0X5C  //default 0x56 //booster on and set contrast
#define CMD_SET_CONSTRAST               0x70  //default 0X7A //set contrast

#define CMD_DISPLAY_ON_CURSOR_ON_BLINK_ON		0x0F //display on, cursor on, blink
#define CMD_DISPLAY_ON_CURSOR_ON_BLINK_OFF		0x0E //display on, cursor on, no blink
#define CMD_DISPLAY_ON_CURSOR_OFF_BLINK_OFF		0x0C //display on, cursor off, no blink
#define CMD_DISPLAY_OFF_CURSOR_OFF_BLINK_OFF	0x08 //display off, cursor off, no blink

//lcd view angle for cursor calculation
bool lcd_view = true;

//PUBLIC FUNCTION
//initialize lcd
void EADOGS164::init(bool backlight, bool view_angle, bool cursor_show)
{
	//reset lcd
	reset();
	//set back light
	backLight(backlight);
	//function set
    send_cmd(CMD_8BIT_NORMAL_RE1_REV0);
	send_cmd(CMD_4_LINE_DISPLAY);
    
    //bottom view or top view
	lcd_view = view_angle;
    if (view_angle == true){
        send_cmd(CMD_BOTTOM_VIEW);
    }
    else if (view_angle == false){
        send_cmd(CMD_TOP_VIEW);
    }
    //function set
	send_cmd(CMD_BIAS_SETTING);
	send_cmd(CMD_8BIT_NORMAL_RE0_IS1);
	send_cmd(CMD_INTERNAL_OSC);
	send_cmd(CMD_DIVIDER_ON);
	send_cmd(CMD_POWER_AND_SET_CONTRAST);
	send_cmd(CMD_SET_CONSTRAST);
	send_cmd(CMD_8BIT_NORMAL_RE0_IS0);
    
    //display on, set cursor on/off
    if (cursor_show == true){
        send_cmd(CMD_DISPLAY_ON_CURSOR_ON_BLINK_ON);
    }
    else if (cursor_show == false){
        send_cmd(CMD_DISPLAY_ON_CURSOR_OFF_BLINK_OFF);
    }
    //clear lcd
	clear();
	//move cursor home
	home();
}
//clear lcd
void EADOGS164::clear()
{
	send_cmd(CMD_CLEAR_DISPLAY);
}
//move cursor home
void EADOGS164::home()
{
	send_cmd(CMD_RETURN_HOME);
}
//set back light
void EADOGS164::backLight(bool status)
{
    if (status == true)
        HAL_GPIO_WritePin(LCD_LIGHT_GPIO_Port, LCD_LIGHT_Pin, GPIO_PIN_SET);
    else if (status == false)
        HAL_GPIO_WritePin(LCD_LIGHT_GPIO_Port, LCD_LIGHT_Pin, GPIO_PIN_RESET);
}
//move cursor
void EADOGS164::setCursor(uint8_t col, uint8_t row)
{
    //limit row and col
	row = _min(row, lcd_rows - 1);
	col = _min(col, lcd_cols - 1);
	if (lcd_view == true){
		send_cmd(CMD_ADDRESS_DDRAM | (row * 0x20 + col));
	}
	else if (lcd_view == false){
		send_cmd(CMD_ADDRESS_DDRAM | ((row * 0x20 + col) + 0x04));
	}
}
//set contrast
void EADOGS164::setContrast(uint8_t contrast)
{
	//contrast = 0x08 => 0x40
	send_cmd(CMD_8BIT_NORMAL_RE0_IS1);
	send_cmd(0x5C | ((contrast >> 4) & 0x03));
	send_cmd(0x70 | (contrast & 0x0F));
	send_cmd(CMD_8BIT_NORMAL_RE0_IS0);
}
//draw character to lcd
void EADOGS164::drawString(uint8_t col, uint8_t row, const char *str) {
	setCursor(col,row);
	while (*str) send_data(*str++);
}

//PRIVATE FUNCTION
//reset lcd
void EADOGS164::reset()
{
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(50);
}
//send command
void EADOGS164::send_cmd(uint8_t cmd)
{
	uint8_t buf[2];
	buf[0] = 0x00; //command control byte
	buf[1] = cmd;
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, buf, 2, HAL_MAX_DELAY);
//	HAL_I2C_Master_Transmit_DMA(&hi2c2, SLAVE_ADDRESS_LCD, buf, 2);
}
//send data
void EADOGS164::send_data(uint8_t data)
{
	uint8_t buf[2];
	buf[0] = 0x40; //data control byte
	buf[1] = data;
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_LCD, buf, 2, HAL_MAX_DELAY);
//	HAL_I2C_Master_Transmit_DMA(&hi2c2, SLAVE_ADDRESS_LCD, buf, 2);
}

