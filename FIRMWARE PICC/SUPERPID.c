#include <18F2550.h>
#device *=16
#device adc=10


#FUSES NOWDT                    //No Watch Dog Timer
#FUSES WDT128                   //Watch Dog Timer uses 1:128 Postscale
#FUSES PLL1                     //No PLL PreScaler
#FUSES CPUDIV1                  //No System Clock Postscaler
#FUSES NOUSBDIV                 //USB clock source comes from primary oscillator
#FUSES HS                    //High speed Osc (> 4mhz for PCM/PCH) (>10mhz for PCD)
#FUSES NOFCMEN                  //Fail-safe clock monitor disabled
#FUSES NOIESO                   //Internal External Switch Over mode disabled
#FUSES NOBROWNOUT               //No brownout reset
#FUSES NOVREGEN                 //USB voltage regulator disabled
#FUSES NOPBADEN                 //PORTB pins are configured as digital I/O on RESET
#FUSES NOLPT1OSC                //Timer1 configured for higher power operation
#FUSES NOSTVREN                 //Stack full/underflow will not cause reset
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES NODEBUG
#FUSES MCLR


#use delay(clock=20000000)
#use fast_io(A)
#use fast_io(B)
#use fast_io(C)

#include "flex_lcd.c"

#byte LATC = 0xF8B
#bit PWM = LATC.0



//Constantes
#define RES_ADC  1024            //resolucion del adc
#define LIMINF   5               //limite inferior de velocidad
#define LIMSUP   1020            //limite superior de velocidad
#define RUN PIN_A1
const int buffer=100;

//Variables
float const uSxTick = 1.6;    // Microsegundos por Tick de TMR1 a 20 Mhz
int1 flagCX0 = FALSE;         //flag de cruce por cero
int1 flagAC = FALSE;          //flag de señal AC
int1 flagsoft = FALSE;        //Flag soft-start
int16 referencia = LIMINF;    //velocidad actual del motor
int16 sensor = 0;             //Sensor de RPM
long Semiciclo = 0;           //duracion del semiciclo
int1 flanco = false;
int16 RPM=0,tt=0;
int16 conteo=0;
int16 muestra[buffer];


signed int16 co=0;
signed int16 e=0,e_pasado=0;
float kp=5.0,in=0,d=0;
float ki=10.0;
float kd=0.0;  
float const Ts = 0.0166;

//Rutinas
void init(void);
void CalcularVelocidad(void);
void PID(void);
void softstart(void);
void testAC(void);

#int_default 
void default_isr(void) 
{

if(interrupt_active(int_EXT))    //Interrupcion cruces por cero
   {
   clear_interrupt(int_EXT);
   PWM = FALSE;                  //empieza semiciclo, apagar triac
   Semiciclo = get_timer1();     //establece la duracion del semiciclo
   set_timer1(0);                //restablece timer
   flagCX0 = TRUE;               //flag para leer ADC y calcular velocidad
   goto jump;
   }
   
if(interrupt_active(int_EXT1))   //Interrupcion RPM
   {
   clear_interrupt(int_EXT1);
   if(!flanco)
      {
      flanco=true;
      set_timer3(0);//reset the timer.
      }
   else
      {
      flanco=false;
      tt=get_timer3();
      }
   goto jump;
   }

if(interrupt_active(INT_TIMER1)) //Interrupcion para comprobar la presencia de AC
   {
   clear_interrupt(INT_TIMER1);
   flagAC = TRUE;
   goto jump;
   }

if(interrupt_active(INT_TIMER3)) //Interrupcion que vence cuando el sensor de RPM no detecta 
   {
   clear_interrupt(INT_TIMER3);
   tt=0;
   goto jump;
   }

if(interrupt_active(int_CCP1))   //Interrupcion que coloca en algunlo de disparo
   {
   clear_interrupt(int_CCP1);
   PWM = TRUE;                  //encender triac, estamos en la intensidad deseada
   }
jump:
delay_cycles(1);
}

void main() 
   {
   int1 visualizar=false;
   int16 mostrar;
   
   init();
   test:
   testAC();
   
   
for(;;)
   {
   if(flagAC==TRUE)
      {
      flagAC=FALSE;
      PWM = FALSE;
      disable_interrupts(INT_CCP1);
      disable_interrupts(int_EXT);
      CCP_1 = 0;
      goto test;
      }
   if(input(RUN)==0)
      {
      enable_interrupts(INT_CCP1);
      enable_interrupts(int_EXT);
      if(flagCX0 == TRUE)//Si hay cruce por cero del AC
         {
         RPM = (int16)(60000000/(uSxTick*(float)tt));
         conteo++;
         PID();
         //CalcularVelocidad();
         if(conteo>=24)
            {visualizar=true;conteo=0;}
         flagCX0 = FALSE;        
         }
      if(visualizar)
         {
         visualizar=false;
         mostrar=referencia*29.32;
         lcd_gotoxy(1,1);printf(lcd_putc,"RPM=%5lu      ",mostrar);
         lcd_gotoxy(5,2);printf(lcd_putc,"%5lu",RPM);
         lcd_gotoxy(11,2);printf(lcd_putc,"%5ld",co);
         }
      }
   else
      {
      PWM = FALSE;
      disable_interrupts(INT_CCP1);
      disable_interrupts(int_EXT);set_timer1(0);CCP_1 = 0;
      co=d=in=0;
      e_pasado=e=0;
      lcd_gotoxy(1,1);lcd_putc("SPINDLE VIEW....");
      if(input(PIN_B1))
         {lcd_gotoxy(1,2);lcd_putc("================");}
      else
         {lcd_gotoxy(1,2);lcd_putc("                ");}
      lcd_putc("\f");
      }
   }
 
}

void CalcularVelocidad(void)
{
   referencia = read_adc();

   if(referencia < LIMINF)   //comprueba que la velocidad nunca sea inferior al limite
      referencia = LIMINF;
   if(referencia > LIMSUP)   //comprueba que la velocidad nunca sea superior al limite
      referencia = LIMSUP;
   
   CCP_1 = Semiciclo - ((Semiciclo / RES_ADC) * referencia);
}

void  PID(void) 
{
   int32 promedio=0;
   int j;
   muestra[buffer-1] = read_adc();
   
   for(j=0;j<=buffer-1;j++)//0-->M
      {
      muestra[j] = muestra[j+1];
      }
   for(j=0;j<=buffer-2;j++)//0-->M
      {
      promedio = promedio + muestra[j];
      }
   referencia = promedio/(buffer-1);

   //referencia = read_adc();
   sensor = RPM*0.0341;
//////////////////////////////////////////     
   e = (referencia - sensor);
   d = (float)(e-e_pasado)/Ts;
   in = (float)e*Ts+in;
   if(in>1020){in=1020;}
   if(in<0){in=0;}
   co = kp*e+kd*d+ki*in;
   e_pasado=e;
   if(co < LIMINF){co = LIMINF;}
   if(co > LIMSUP){co = LIMSUP;}     
   CCP_1 = Semiciclo - ((Semiciclo / RES_ADC) * co);
}

void softstart(void)
{
conteo=0;
do {
   if(flagCX0 == TRUE)//Si hay cruce por cero del AC
         {
         RPM = (int16)(60000000/(uSxTick*(float)tt));
         conteo++;
         PID();
         }
   }while (conteo<60);
}

void testAC(void)
{
   flagCX0 = FALSE;
   enable_interrupts(int_EXT);
   conteo=0;
   do{
   delay_ms(1);
   conteo++;
   if(conteo>=250)
      {goto not_main;}
   }while(flagCX0==FALSE);
   
   flagCX0 = FALSE;
   goto normal;

   not_main:
   flagCX0 = FALSE;
   do{
   lcd_putc("Main not found");
   delay_ms(750);
   lcd_putc("\f");
   delay_ms(750);
   }while(flagCX0==FALSE);

   normal:
   do{}while(flagCX0==FALSE);
   //semiciclo contiene el tiempo en ticks del uP para obtener la frecuencia de la red si semiclo entre 25000 y 62500 o 100Hz a 40Hz con ticks de 0.2us
   RPM = (int16)(25000090/(float)semiciclo);//Exponencial para calcular la Frecuencia de la red a partir de los ticks del uP
   lcd_gotoxy(4,1);
   lcd_putc("Main found");
   lcd_gotoxy(6,2);
   printf(lcd_putc,"%2.1wHz",RPM);
   delay_ms(1000);
   lcd_putc("\f");
}

void init()
{
   SET_TRIS_A( 0b11111111 );
   SET_TRIS_B( 0b00000011 );
   SET_TRIS_C( 0b11111000 );
   
   OUTPUT_C(0X00);
   
   lcd_init();
   lcd_putc("\f");
   lcd_gotoxy(2,1);
   lcd_putc("SUPER-PID  V1.0");
   lcd_gotoxy(2,2);
   lcd_putc("SUPER-PID  V1.0");
   delay_ms(1000);
   lcd_putc("\f");
   
   setup_adc_ports(AN0);
   setup_adc(ADC_CLOCK_INTERNAL);
   set_adc_channel(0);
   setup_timer_0(T0_INTERNAL|T0_DIV_8);
   setup_timer_1(T1_INTERNAL|T1_DIV_BY_1);
   setup_timer_2(T2_DIV_BY_1,131,1);   //26.4 us overflow, 26.4 us interrupt 38KHZ
   setup_timer_3(T3_INTERNAL|T3_DIV_BY_8);
   setup_ccp1(CCP_COMPARE_INT);
   setup_ccp2(CCP_PWM);
   set_pwm2_duty(40);//%65=50%

   enable_interrupts(INT_EXT);
   enable_interrupts(INT_EXT1);
   //enable_interrupts(INT_CCP1);
   enable_interrupts(INT_TIMER1);
   enable_interrupts(INT_TIMER3);
   enable_interrupts(GLOBAL);
 
   ext_int_edge(0,L_TO_H);    //configura INT para que solo se active a la subida
   ext_int_edge(1,L_TO_H);
   
   PWM = FALSE;               //pwm apagado}
   flagAC=FALSE;
}
