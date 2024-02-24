
#ifndef __LCD_C__
#define __LCD_C__

#include <p18cxxx.h>
#include <delays.h>
#include <stdio.h>
#include "lcd.h"

//http://home.iae.nl/users/pouweha/lcd/lcd0.shtml

#define TRIS_LCD_RS 	TRISBbits.TRISB7
#define TRIS_LCD_EN 	TRISBbits.TRISB6
#define	TRIS_LCD_DB4	TRISBbits.TRISB5
#define	TRIS_LCD_DB5	TRISBbits.TRISB4
#define	TRIS_LCD_DB6	TRISBbits.TRISB3
#define	TRIS_LCD_DB7	TRISBbits.TRISB2

#define STROBE { LCD_EN = 1; Nop(); LCD_EN = 0; }

void lcd_config(void)
{
	stdout = _H_USER;
	
	TRIS_LCD_EN = 0;
	TRIS_LCD_RS = 0;
	TRIS_LCD_DB4 = 0;
	TRIS_LCD_DB5 = 0;
	TRIS_LCD_DB6 = 0;
	TRIS_LCD_DB7 = 0;

	LCD_RS = 0;	
	delay5ms;
	
	//LCD_DB = 0x3;
	LCD_DB4 = 1;
	LCD_DB5 = 1;
	LCD_DB6 = 0;
	LCD_DB7 = 0;
	STROBE;
	delay40us;
	
	STROBE;
	delay40us;
	
	STROBE;
	delay40us;
	
	//LCD_DB = 0x2;	
	LCD_DB4 = 0;
	LCD_DB5 = 1;
	LCD_DB6 = 0;
	LCD_DB7 = 0;
	STROBE;
	delay40us;

	lcd_write(0x28); // Function set (4bit interface, 2 line display, display off)
	lcd_write(0x0C); // Display on/off control (display on, cursor off, blinking off)
	lcd_write(0x06); // Entry mode set (increase, not shifted) 
	lcd_write(0x01); // Reset
	delay2ms;
	LCD_RS = 1;
}



void lcd_clear(void)
{
	LCD_RS = 0;
	lcd_write(0x01);
	delay2ms;
	LCD_RS = 1;
}

void lcd_write(char c)
{
	LCD_DB4 = (c & 0x10) != 0;
	LCD_DB5 = (c & 0x20) != 0;
	LCD_DB6 = (c & 0x40) != 0;
	LCD_DB7 = (c & 0x80) != 0;
	STROBE;
	Delay10TCYx(48); //40us
	
	LCD_DB4 = (c & 0x01) != 0;
	LCD_DB5 = (c & 0x02) != 0;
	LCD_DB6 = (c & 0x04) != 0;
	LCD_DB7 = (c & 0x08) != 0;
	STROBE;
	Delay10TCYx(48); //40us
}

void lcd_pos(unsigned char row, unsigned char col)
{
	LCD_RS = 0;
	switch (row)
	{
		case 0: lcd_write(0x80 | 0x00 + col); break;
		case 1: lcd_write(0x80 | 0x40 + col); break;
		case 2: lcd_write(0x80 | 0x14 + col); break;
		case 3: lcd_write(0x80 | 0x54 + col); break;
	}		
}	
void lcd_line1()
{
	LCD_RS = 0;
	lcd_write(0x80 | 0x00);
}
void lcd_line2()
{
	LCD_RS = 0;
	lcd_write(0x80 | 0x40);
}
void lcd_line3()
{
	LCD_RS = 0;
	lcd_write(0x80 | 0x14);
}
void lcd_line4()
{
	LCD_RS = 0;
	lcd_write(0x80 | 0x54);
}

void lcd_puts(const char *s)
{
	LCD_RS = 1;
	while(*s) lcd_write(*s++);
}
void lcd_putsDec(const char *s, char dec)
{
	LCD_RS = 1;
	while(*s)
	{
		if (!dec--)
		{
			lcd_write('.');
			continue;
		}	
		lcd_write(*s++);
	} 
}

void lcd_putss(const rom char *s)
{
	LCD_RS = 1;
	while(*s) lcd_write(*s++);
}

void _user_putc(char c) // redirect printf to LCD 
{
	LCD_RS = 1;
	lcd_write(c);	
}	
#endif 
