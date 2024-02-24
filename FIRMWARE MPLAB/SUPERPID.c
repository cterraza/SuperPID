#include <p18cxxx.h>
#include <p18f2550.h>
#include <string.h>
#include <delays.h>
#include <stdio.h>
#include "flex_lcd_c18.c"
#include <timers.h> //header file for Timers
#include <pwm.h> //header file for PWM
#include <adc.h> //header file for ADC

#pragma config PLLDIV   = 1 		//Divide by 1 (8 MHz oscillator input)	(0xF9)
#pragma config USBDIV   = 1 		//USB clock source comes from the 96 MHz PLL divided by 2	(0xFF)
#pragma config FOSC     = HS	 	//HS oscillator, PLL disabled, HS used by CPU	(0xFE)
#pragma config FCMEN    = OFF 		//Fail-Safe Clock Monitor disabled	(0xBF)
#pragma config IESO     = OFF 		//Oscillator Switchover mode disabled	(0x7F)
#pragma config PWRT     = ON		//PWRT enabled
#pragma config BOR      = OFF		//Brown-out Reset enabled in hardware only (SBOREN is disabled)
#pragma config BORV 	= 2			//4.33V 
#pragma config VREGEN   = OFF		//USB voltage regulator enabled
#pragma config WDT      = OFF		//Watchdog Timer Disabled
#pragma config WDTPS    = 512		//Watchdog Timer Postscale 1:512
#pragma config MCLRE    = ON		//MCLR pin enabled// RE3 input pin disabled
#pragma config LPT1OSC  = OFF		//Timer1 configured for higher power operation
#pragma config PBADEN   = OFF		//PORTB<4:0> pins are configured as digital I/O on Reset
#pragma config CCP2MX   = ON		//CCP2 input/output is multiplexed with RB3
#pragma config STVREN   = ON		//Stack full/underflow will cause Reset
#pragma config LVP      = OFF		//Single-Supply ICSP disabled
#pragma config XINST    = OFF		//Instruction set extension and Indexed Addressing mode disabled (Legacy mode)
#pragma config DEBUG    = OFF		//Background debugger disabled, RB6 and RB7 configured as general purpose I/O pins

//20MHz
#define delay40us Delay10TCYx(20)		//48
#define delay50us Delay10TCYx(25) 		//60
#define delay1ms Delay100TCYx(50)  		//120
#define delay2ms Delay100TCYx(100)		//240
#define delay5ms Delay1KTCYx(25)		//60
#define delay15ms Delay1KTCYx(75)		//180
#define delay100ms Delay10KTCYx(50);	//120
#define delay200ms Delay10KTCYx(100);	//240
/////////////////////////////////////////////////
//pines
#define PWM	LATCbits.LATC0			//Pin de control de angulo de disparo
#define RUN PORTAbits.RA1			//Pin de entrada para operacion
#define SENSOR PORTBbits.RB1		//Pin de entrada del RPM sensor
//Constantes
#define RES_ADC 1024         	//resolucion del adc
#define LIMINF  10				//limite inferior de velocidad
#define LIMSUP  1020         	//limite superior de velocidad
#define buffer	100				//Buffer para el moving Average
//Variables
float const uSxTick = 1.6;		// Microsegundos por Tick de TMR1 a 20 Mhz
unsigned char flagCX0 = 0;      //flag de cruce por cero
unsigned char flagAC = 1;       //flag de señal AC
unsigned char flagsoft = 1;		//Flag soft-start
unsigned char flanco = 1;
unsigned int referencia = LIMINF;    //velocidad actual del motor
unsigned int sensor = 0;
unsigned int RPM=0,tt=0xFFFF;
unsigned int conteo=0;
unsigned int CCP_1=0;			//Registro intermedio para el CCP1
unsigned int muestra[buffer];	//Sensor de RPM
unsigned long Semiciclo = 0;	//Duracion del semiciclo
//Variables PID
signed int co=0;
signed int e=0,e_pasado=0;
float kp=2.0,in=0,d=0;
float ki=1.0;
float kd=0.0;  
float const Ts = 0.0166;
//Rutinas
void init(void);
void CalcularVelocidad(void);
void PID(void);
void softstart(void);
void testAC(void);
void InterruptHandlerHigh (void);
/////////////////////////////////////////////////

//////////Vector de interrupciones//////////
//----------------------------------------------------------------------------
// High priority interrupt vector

#pragma code InterruptVectorHigh = 0x08
void InterruptVectorHigh (void)
{
_asm
    goto InterruptHandlerHigh 
_endasm
}
/*
#pragma code InterruptVectorLow = 0x18
void InterruptVectorLow (void)
{
_asm 
	goto InterruptHandlerLow
 _endasm
}
#pragma code*/

//----------------------------------------------------------------------------
// High priority interrupt routine

#pragma code
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh (void)
{
if(INTCONbits.INT0IF)
    {                                 								//check for INT0 interrupt
	INTCONbits.INT0IF = 0;            
   	PWM = 0;                  										//empieza semiciclo, apagar triac
   	Semiciclo = ((unsigned int)TMR1H<<8) | ((unsigned int)TMR1L);	//establece la duracion del semiciclo
   	TMR1H = 0;
	TMR1L = 0;                										//restablece timer
   	flagCX0 = 1;              										//flag para leer ADC y calcular velocidad
	goto jump;    
	}

if(INTCON3bits.INT1IF)
    {                                 								//check for INT1 interrupt
	INTCON3bits.INT1IF = 0;
	if(flanco==0)
		{
      	flanco = 1;
      	TMR3H = 0;
		TMR3L = 0;
      	}
   	else
      	{
      	flanco = 0;
      	tt = ((unsigned int)TMR3H<<8) | ((unsigned int)TMR3L);
      	}
	goto jump;
	}

if(PIR1bits.TMR1IF)													//check for TIMER1 interrupt
	{
   	PIR1bits.TMR1IF = 0;
   	flagAC = 1;
	PWM = 0;
	PIE1bits.CCP1IE = 0;
	//INTCON3bits.INT1IE = 0;
	CCPR1L = 0XFF;
	CCPR1H = 0XFF;
	goto jump;
   	}

if(PIR2bits.TMR3IF)													//check for TIMER3 interrupt
   	{
   	PIR2bits.TMR3IF = 0;
	tt = 0xFFFF;
	goto jump;
   	}

if(PIR1bits.CCP1IF)													//check for CCP1 interrupt
   	{
   	PIR1bits.CCP1IF = 0;
   	PWM = 1;                  										//encender triac, estamos en la intensidad deseada
   	}
jump:
Delay1TCY();
}

void main(void)
{
unsigned char visualizar = 0;
unsigned int mostrar;
stdout =_H_USER;
init();
test:
PWM = 0;
referencia = 10;
testAC();	
delay200ms;

for(;;)
{
if(flagAC)
	{
    flagAC = 0;
    PWM = 0;
    //PIE1bits.CCP1IE = 0;
    //INTCONbits.INT0IE = 0;
	lcd_putrs("\f");
    goto test;
    }

if(!RUN)
	{
	PIE1bits.CCP1IE = 1;
	INTCONbits.INT0IE = 1;
	T1CONbits.TMR1ON = 1;
	if(flagCX0)
	    {
		flagCX0 = 0;
		RPM = (int)(30000000/(uSxTick*(float)tt));
		CalcularVelocidad();
		conteo++;
	    if(conteo>=24)
	    	{visualizar=1;conteo=0;}
	    }
	
	if(visualizar)
		{
		visualizar=0;
		RPM = (int)(30000000/(uSxTick*tt));
		referencia=referencia*29.32;
		//lcd_putrs("\f");
		lcd_gotoxy(1,1);printf("REF= %5u     ",referencia);
		lcd_gotoxy(1,2);printf("RPM= %5u     ",RPM);
		}
	}
else
	{
	PWM = 0;
	T1CONbits.TMR1ON = 0;
	PIE1bits.CCP1IE = 0;
	INTCONbits.INT0IE = 0;
	TMR1H = 0;TMR1L = 0;
	CCPR1L = 0XFF;CCPR1H = 0XFF;
	co=d=in=0;
	e_pasado=e=0;
	delay200ms;
	lcd_gotoxy(1,1);lcd_putrs(" SPINDLE VIEW....");
	if(SENSOR)
		{lcd_gotoxy(1,2);lcd_putrs("================");}
	else
		{lcd_gotoxy(1,2);lcd_putrs("                ");}
	}
}
 
		
}//END MAIN	



/////////////////////uP INIT ROUTINE/////////////////
void init(void)
{
TRISA = 0xFF; //LATA = 0x00;
TRISB = 0x03; //LATB = 0x00;
TRISC = 0xF8; //LATC = 0x00;

lcd_init();
lcd_putrs("\f");
lcd_gotoxy(2,1);
lcd_putrs(" SUPER-PID  V1.0");
lcd_gotoxy(2,2);
lcd_putrs("SUPER-PID  V1.0");
delay200ms;delay200ms;delay200ms;delay200ms;delay200ms;
lcd_putrs("\f");

OpenADC( ADC_FOSC_32 & ADC_RIGHT_JUST & ADC_12_TAD, ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, 14 );// ADCON1 = 14;
//OpenTimer0( TIMER_INT_OFF & T0_8BIT & T0_SOURCE_INT & T0_PS_1_32 );
OpenTimer1( TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_1 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF & T1_SOURCE_CCP );
OpenTimer2( TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1 );
OpenTimer3( TIMER_INT_ON & T3_16BIT_RW & T3_SOURCE_INT & T3_PS_1_8 & T3_SYNC_EXT_OFF );
CCP1CON = 0x0A; //Configura modulo CCP1 en modo comparacion con el TIMER1
OpenPWM2(131); //26.4 us overflow, 26.4 us interrupt 38KHZ
SetDCPWM2(40);

INTCON = 0x00;			//Desactiva todas las interrupciones

INTCON2bits.INTEDG0 = 1;//configura INT para flanco de subida
INTCON2bits.INTEDG1 = 1;//configura INT para flanco de subida

RCONbits.IPEN = 0;		/* Disable interrupt priority */

INTCONbits.INT0IE = 0;	//Habilita interrupcion INT0
INTCON3bits.INT1IE = 1; //Habilita interrupcion INT1
INTCON3bits.INT1IP = 1; //Alta prioridad INT1
//INTCON3bits.INT1IP = 1;	//Prioridad alta INT1
//PIE1bits.CCP1IE = 1;	//Habilita interrupcion CCP1
//IPR1bits.CCP1IP = 1;	//Prioridad alta CCP1
PIE1bits.TMR1IE = 1;	//Habilita interrupcion TIMER1
//IPR1bits.TMR1IP = 1;	//Prioridad alta TIMER1
PIE2bits.TMR3IE = 1;	//Habilita interrupcion TIMER3
//IPR2bits.TMR3IP = 1;	//Prioridad alta TIMER3
INTCONbits.PEIE = 1;	//Habilita interrupcion de perifericos
INTCONbits.GIE = 1;		//Habilita interrupciones Globales
//INTCONbits.GIEH = 1;	//Habilita interrupciones alta prioridad
//INTCONbits.GIEL = 1;	//Habilita interrupciones baja prioridad
PWM = 0;               //pwm apagado}
flagAC=0;
CCPR1L = 0XFF;
CCPR1H = 0XFF;
}
/////////////////////DIMMER ROUTINE//////////////
void CalcularVelocidad(void)
{
ConvertADC();
while( BusyADC() );
referencia = ReadADC();

if(referencia < LIMINF)   //comprueba que la velocidad nunca sea inferior al limite
	referencia = LIMINF;
if(referencia > LIMSUP)   //comprueba que la velocidad nunca sea superior al limite
	referencia = LIMSUP;
   
CCP_1 = Semiciclo - ((Semiciclo / RES_ADC) * referencia);
CCPR1L = (unsigned char)CCP_1;
CCPR1H = (unsigned char)(CCP_1>>8);
}
///////////////////PID ROUTINE////////////////
void  PID(void) 
{
unsigned long promedio=0;
unsigned char j;
ConvertADC();
while( BusyADC() );
muestra[buffer-1] = ReadADC();
  
for(j=0;j<=buffer-1;j++)//0-->M
	{
	muestra[j] = muestra[j+1];
	}
   	for(j=0;j<=buffer-2;j++)//0-->M
	{
    promedio = promedio + muestra[j];
    }
referencia = promedio/(buffer-1);

sensor = RPM*0.0341;
//////////////////////////////////////////     
e=(referencia - sensor);
if(e!=0)
	{
    d=(e-e_pasado)/Ts;
    in=e*Ts+in;
    if(in>1020){in=1020;}
    if(in<0){in=0;}
    co=kp*e+kd*d+ki*in;
    e_pasado=e;
    if(co < LIMINF){co = LIMINF;}
    if(co > LIMSUP){co = LIMSUP;}     
    CCP_1 = Semiciclo - ((Semiciclo / RES_ADC) * co);
	CCPR1L = (unsigned char)CCP_1;
	CCPR1H = (unsigned char)(CCP_1>>8);
	}
}
///////////////////TEST AC/////////////////
void testAC(void)
{
   	flagCX0 = 0;
//	PIE1bits.CCP1IE = 0;
   	INTCONbits.INT0IE = 1;
   	conteo=0;
   	do
		{
   		delay1ms;
   		conteo++;
   		if(conteo>=250)
      		{goto not_main;}
   		}while(flagCX0==0);
   
   	flagCX0 = 0;
   	goto normal;

   	not_main:
   	flagCX0 = 0;
   	do
		{
   		lcd_putrs("Main not found");
   		delay200ms;delay200ms;delay200ms;
   		lcd_putrs("\f");
   		delay200ms;delay200ms;delay200ms; 
   		}while(flagCX0==0);

   	normal:
   	do{}while(flagCX0==0);
   	//semiciclo contiene el tiempo en ticks del uP para obtener la frecuencia de la red si semiclo entre 25000 y 62500 o 100Hz a 40Hz con ticks de 0.2us
   	RPM = (int)(25000090/(float)Semiciclo);//Exponencial para calcular la Frecuencia de la red a partir de los ticks del uP
   	lcd_gotoxy(4,1);
   	lcd_putrs("Main found");
   	lcd_gotoxy(6,2);
   	printf("%3u Hz",RPM);
   	delay200ms;delay200ms;delay200ms;delay200ms;delay200ms;
   	lcd_putrs("\f");
	PIE1bits.CCP1IE = 1;
}