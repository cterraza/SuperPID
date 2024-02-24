
#ifndef __LCD_H__
#define __LCD_H__

#include <delays.h>

//pins
#define	LCD_RS	LATBbits.LATB7
#define	LCD_EN	LATBbits.LATB6
#define	LCD_DB4	LATBbits.LATB5
#define	LCD_DB5	LATBbits.LATB4
#define	LCD_DB6	LATBbits.LATB3
#define	LCD_DB7	LATBbits.LATB2


// Cycles = (TimeDelay * Fosc) / 4 		//Para 48MHz
#define delay40us Delay10TCYx(20)		//48
#define delay50us Delay10TCYx(25) 		//60
#define delay1ms Delay100TCYx(50)  		//120
#define delay2ms Delay100TCYx(100)		//240
#define delay5ms Delay1KTCYx(25)		//60
#define delay15ms Delay1KTCYx(75)		//180
#define delay100ms Delay10KTCYx(50);	//120
#define delay200ms Delay10KTCYx(100);	//240

extern void lcd_config(void);
extern void lcd_clear(void);
extern void lcd_write(char c);
extern void lcd_pos(unsigned char row, unsigned char col);
extern void lcd_line1(void);
extern void lcd_line2(void);
extern void lcd_line3(void);
extern void lcd_line4(void);
extern void lcd_puts(const char *s);
extern void lcd_putsDec(const char *s, char dec);
extern void lcd_putss(const rom char *s);
extern void _user_putc(char c); // redirect printf to LCD 

#endif 
