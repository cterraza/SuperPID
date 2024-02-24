/******************************************************************************* 
   \file FLEX_LCD_C18.h
   \version: 1.0
   
   \author Suky. 
   \web www.infopic.comlu.com
   \mail inf.pic.suky@live.com.ar
   \date 03/04/10
*/
#include <p18cxxx.h>
#include <delays.h>

#ifndef _FLEX_LCD_H
#define _FLEX_LCD_H


#define FCY 20000000UL
#define _delay_100us() Delay100TCYx(FCY/4000000UL)
#define _delay_5ms() Delay1KTCYx(FCY/800000UL)

/* ** Definicion de tipo de variables **/
typedef unsigned char	int8;
typedef unsigned int	int16;
typedef unsigned long	int32;
/* ** No olvidarse de short long de 24-bits ** */


/* ** Para no utilizar el PIN RW comentar la siguiente definición ** */
//#define USE_RW 


#define DATA_PIN_7 	LATBbits.LATB2
#define DATA_PIN_6 	LATBbits.LATB3
#define DATA_PIN_5 	LATBbits.LATB4
#define DATA_PIN_4 	LATBbits.LATB5
	
#define READ_PIN_7	 PORTBbits.RB2
#define READ_PIN_6	 PORTBbits.RB3
#define READ_PIN_5	 PORTBbits.RB4
#define READ_PIN_4	 PORTBbits.RB5
	
	
#define TRIS_DATA_PIN_7       TRISBbits.TRISB2
#define TRIS_DATA_PIN_6       TRISBbits.TRISB3
#define TRIS_DATA_PIN_5       TRISBbits.TRISB4
#define TRIS_DATA_PIN_4       TRISBbits.TRISB5
	
	
#define E_PIN	 LATBbits.LATB6 
#define RS_PIN	 LATBbits.LATB7 
#ifdef USE_RW
	 #define RW_PIN	 LATBbits.LATB1 
#endif
	
 #define TRIS_E	 TRISBbits.TRISB6 
#define TRIS_RS	 TRISBbits.TRISB7  
#ifdef USE_RW 
	#define TRIS_RW	 TRISBbits.TRISB1 
#endif 


#define lcd_type 2 // 0=5x7, 1=5x10, 2=2 lines
#define lcd_line_two 0x40 // LCD RAM address for the 2nd line

/*;--- Modos de Entrada
; Incrementa Direccion, Display fijo	 0x06
; Decrementa Direccion, Display fijo	 0x04
; Incrementa Direccion, Cursor fijo		 0x07
; Decrementa Direccion, Cursor fijo		 0x05
;--- Corriemiento Cursor
; Cursor a la Izquierda					 0x10
; Cursor a la Derecha					 0x14
; Display a la Izquierda				 0x18
; Display a la Derecha					 0x1C
;--- Fijar Sistema
; Bus 8 bits, 1 línea, 5x7				 0x30
; Bus 8 bits, 1 linea, 5x10				 0x34
; Bus 8 bits, 2 líneas, 5x7				 0x38
; Bus 4 bits, 1 línea, 5x7				 0x20	
 ; Bus 4 bits, 1 linea, 5x10				 0x24
; Bus 4 bits, 2 líneas, 5x7 			0x28*/
#endif