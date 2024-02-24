#include <16F877a.h>
#device *=16
#device adc=10


#FUSES NOWDT                    //No Watch Dog Timer
#FUSES HS                       //High speed Osc (> 4mhz for PCM/PCH) (>10mhz for PCD)
#FUSES NOBROWNOUT               //No brownout reset
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
#FUSES NODEBUG


#use delay(clock=20000000)
#use fast_io(A)
#use fast_io(B)
#use fast_io(C)

#include "flex_lcd.c"

#byte PORTC = 0x07
#bit PWM = PORTC.0
#bit LED = PORTC.1


//Constantes
#define RES_ADC  1024         //resolucion del adc
#define LIMINF   10            //limite inferior de velocidad
#define LIMSUP   1014         //limite superior de velocidad

//Variables
float const uSxTick = 1.6;    // Microsegundos por Tick de TMR1 a 20 Mhz
int1 flagCX0 = FALSE;         //flag de cruce por cero
int16 referencia = LIMINF;    //velocidad actual del motor
int16 sensor = 0;             //Sensor de RPM
long Semiciclo = 0;           //duracion del semiciclo
int1 flanco = false;
int16 RPM=0,tt=0;
int conteo=0;


signed int16 co;
signed int16 e=0,e_pasado=0;
float kp=1.0,in=0,d=0;
float ki=0.5;
float kd=0.0;  
float const Ts = 0.0166;

//Rutinas
void CalcularVelocidad(void);
void  PID(void);

#int_default 
void default_isr(void) 
{

if(interrupt_active(int_EXT))
   {
   clear_interrupt(int_EXT);
   PWM = FALSE;                  //empieza semiciclo, apagar triac
   Semiciclo = get_timer1();     //establece la duracion del semciclo
   set_timer1(0);                //restablece timer
   flagCX0 = TRUE;               //flag para leer ADC y calcular velocidad
   goto end;
   }
   
if(interrupt_active(int_CCP2))
   {
   clear_interrupt(int_CCP2);
   if(!flanco)
      {
      flanco=true;
      set_timer0(0);//reset the timer.
      }
   else
      {
      flanco=false;
      tt=get_timer0();
      }
   goto end;
   }

if(interrupt_active(int_CCP1))
   {
   clear_interrupt(int_CCP1);
   PWM = TRUE;                  //encender triac, estamos en la intensidad deseada
   }
end:;
}

void main() 
   {
   int1 visualizar=false;
   
   SET_TRIS_A( 0xFF );
   SET_TRIS_B( 0xF7 );
   SET_TRIS_C( 0xF0 );
   
   lcd_init();
   lcd_putc("\f");
   lcd_gotoxy(3,1);
   lcd_putc("SUPERPID");
   delay_ms(500);
   lcd_putc("\f");
   printf(lcd_putc,"RPM=");
   
   setup_adc_ports(AN0_AN1_AN3);
   setup_adc(ADC_CLOCK_INTERNAL);
   set_adc_channel(0);
   setup_timer_0(RTCC_INTERNAL);
   setup_timer_1(T1_INTERNAL|T1_DIV_BY_1);
   setup_timer_2(T2_DIV_BY_1,131,1);      //26.4 us overflow, 26.4 us interrupt 38KHZ
   setup_ccp1(CCP_COMPARE_INT);
   setup_ccp2(CCP_PWM);
   set_pwm2_duty(40);//%65=50%
   setup_vref(FALSE);

   enable_interrupts(INT_EXT);
   //enable_interrupts(INT_CCP2);
   enable_interrupts(INT_CCP1);
   enable_interrupts(GLOBAL);
 
   ext_int_edge(0,L_TO_H);    //configura INT para que solo se active a la subida
  // ext_int_edge(2,L_TO_H);    //configura INT para que solo se active a la subida
   
   PWM = FALSE;               //pwm apagado
   
for(;;)
   {
   if(flagCX0 == TRUE)
      {
      conteo++;
      if(conteo==19)
         {visualizar=true;conteo=0;}
      flagCX0 = FALSE;
      CalcularVelocidad();
      
      }
   if(visualizar)
      {
      visualizar=false;
      RPM = (int16)(60000000/(uSxTick*(float)tt));
      lcd_gotoxy(5,1);printf(lcd_putc,"%5ld",referencia);
      lcd_gotoxy(5,2);printf(lcd_putc,"%5ld",sensor);
      }
   }
 
}

void CalcularVelocidad(void)
{
   referencia = read_adc();   //leo adc
   
   if(referencia < LIMINF)   //comprueba que la velocidad nunca sea inferior al limite
      referencia = LIMINF;
   if(referencia > LIMSUP)   //comprueba que la velocidad nunca sea superior al limite
      referencia = LIMSUP;
   
   CCP_1 = Semiciclo - ((Semiciclo / RES_ADC) * referencia);
}

void  PID(void) 
{
   referencia = read_adc();
   
   if(tt<1250){sensor = 1014;}
   else
      {
      if((1250<=tt)&&(tt<=7500)){sensor = (int16)(1214.75 - (float)0.1606*tt);}
      else{sensor=10;}
      }
//////////////////////////////////////////     
   e=(referencia - sensor);
   d=(e-e_pasado)/Ts;
   in=e*Ts+in;
   if(in>5000){in=5000;}
   if(in<-5000){in=-5000;}
   co=kp*e+kd*d+ki*in;
   e_pasado=e;
   if(co < LIMINF){co = LIMINF;}
   if(co > LIMSUP){co = LIMSUP;}
   
   CCP_1 = Semiciclo - ((Semiciclo / RES_ADC) * co);
}

