/*
 * code.c
 *
 *  Created on: 26/08/2014
 *      Author: L01073411
 */


#include "ee.h"
#include "ee_irq.h"
#define FCY 40000000UL
#include <libpic30.h>

static unsigned char BufferOut[80];
static unsigned int my_time=0;
static unsigned int analog_value;
static unsigned int kalman_value;


// MIPS40 - Run CPU at maximum speed 40MIPS (25ns), oscillator with PLL at 80Mhz
// MIPS4 - Run CPU at clock speed 4MIPS (250ns), oscillator without PLL at 8Mhz
#define MIPS40

#ifdef MIPS40
	// Primary (XT, HS, EC) Oscillator with PLL
	_FOSCSEL(FNOSC_PRIPLL);
#endif
#ifdef MIPS4
	// Primary (XT, HS, EC) Oscillator without PLL
	_FOSCSEL(FNOSC_PRI);
#endif

// OSC2 Pin Function: OSC2 is Clock Output - Primary Oscillator Mode: XT Crystanl
_FOSC(OSCIOFNC_ON & POSCMD_XT);
// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF);
// Disable Code Protection
_FGS(GCP_OFF);


/* Program the Timer1 peripheral to raise interrupts */
void T1_program(void)
{
	T1CON = 0;		/* Stops the Timer1 and reset control reg	*/
	TMR1  = 0;		/* Clear contents of the timer register	*/
#ifdef MIPS40
	PR1   = 0x9c40;		/* PR1=40000 Load the Period register with the value of 1ms	*/
#endif
#ifdef MIPS4
	PR1   = 0x0fa0;		/* PR1=4000 Load the Period register with the value of 1ms	*/
#endif
	IPC0bits.T1IP = 5;	/* Set Timer1 priority to 1		*/
	IFS0bits.T1IF = 0;	/* Clear the Timer1 interrupt status flag	*/
	IEC0bits.T1IE = 1;	/* Enable Timer1 interrupts		*/
	T1CONbits.TON = 1;	/* Start Timer1 with prescaler settings at 1:1
						* and clock source set to the internal
						* instruction cycle			*/
}

/* Clear the Timer1 interrupt status flag */
void T1_clear(void)
{
	IFS0bits.T1IF = 0;
}

/* This is an ISR Type 2 which is attached to the Timer 1 peripheral IRQ pin
 * The ISR simply calls CounterTick to implement the timing reference
 */
ISR2(_T1Interrupt)
{
	/* clear the interrupt source */
	T1_clear();

	/* count the interrupts, waking up expired alarms */
	CounterTick(myCounter);
}


/* Writes an initial message in the LCD display first row */
void put_LCD_initial_message()
{
	EE_lcd_goto( 0, 0 );

	EE_lcd_putc('R');
	EE_lcd_putc('e');
	EE_lcd_putc('a');
	EE_lcd_putc('l');
	EE_lcd_putc(' ');
	EE_lcd_putc('T');
	EE_lcd_putc('i');
	EE_lcd_putc('m');
	EE_lcd_putc('e');
	EE_lcd_putc(' ');
	EE_lcd_putc('-');
	EE_lcd_putc(' ');
	EE_lcd_putc('L');
	EE_lcd_putc('a');
	EE_lcd_putc('b');
	EE_lcd_putc('7');

}

/******************************************************************************************
 * Función:	Serial_init()						     										  *
 * Descripción:	Configura Serial Port			 		          								  *
 ******************************************************************************************/
void Serial_Init(void)
{
	/* Stop UART port */
	U2MODEbits.UARTEN = 0;

	/* Disable Interrupts */
	IEC1bits.U2RXIE = 0;
	IEC1bits.U2TXIE = 0;

	/* Clear Interrupt flag bits */
	IFS1bits.U2RXIF = 0;
	IFS1bits.U2TXIF = 0;

	/* Set IO pins */
	TRISFbits.TRISF12 = 0;  // CTS Output
	TRISFbits.TRISF13 = 0;  // RTS Output
	TRISFbits.TRISF5 = 0;   // TX Output
	TRISFbits.TRISF4 = 1;   // RX Input

	/* baud rate */
	U2MODEbits.BRGH = 0;
	U2BRG  = 21; // 42 -> 57600 baud rate / 21-> 115200 baud rate

	/* Operation settings and start port */
	U2MODE = 0;
	U2MODEbits.UEN = 0; //2
	U2MODEbits.UARTEN = 1;

	/* TX & RX interrupt modes */
	U2STA = 0;
	U2STAbits.UTXEN=1;
}

/******************************************************************************************
 * Send one byte						     										  *
 ******************************************************************************************/
int Serial_Send(unsigned char data)
{
	while (U2STAbits.UTXBF);
	U2TXREG = data;
	while(!U2STAbits.TRMT);
	return 0;
}

/******************************************************************************************
 * Send a group of bytes						     										  *
 ******************************************************************************************/
void Serial_Send_Frame(unsigned char *ch, unsigned char len)
{
   unsigned char i;

   for (i = 0; i < len; i++) {
	Serial_Send(*(ch++));
   }
}


/******************************************************************************************
 * Función:	ADC1_init()						     										  *
 * Descripción:	Configura ADC1.			 		          								  *
 ******************************************************************************************/
void ADC1_init(void)
{
	TRISBbits.TRISB4 = 1;
	TRISBbits.TRISB5 = 1;

	AD1CON1bits.ADON  = 0;// ADC Operating Mode bit. Turn off the A/D converter

	/*ADC Configuration*/
	AD1PCFGL = 0xFFFF;		//ADC1 Port Configuration Register Low
	AD1PCFGH = 0xFFFF;		//ADC1 Port Configuration Register High

	AD1PCFGLbits.PCFG4=0;   //Potentiometer input RB4/AN4
	AD1PCFGLbits.PCFG5=0;   //Potentiometer input RB4/AN4

	AD1CON2bits.VCFG = 0;    /*Converter Voltage Reference Configuration bits
				(ADRef+=AVdd, ADRef-=AVss)*/
	AD1CON3bits.ADCS = 63;   /* ADC Conversion Clock Select bits
			     	*(Tad = Tcy*(ADCS+1) = (1/40000000)*64 = 1.6us)
				*Tcy=Instruction Cycle Time=40MIPS */
	AD1CON2bits.CHPS = 0;	/* Selects Channels Utilized bits, When AD12B = 1,
				 * CHPS<1:0> is: U-0, Unimplemented, Read as ‘0’ */
	AD1CON1bits.SSRC = 7;/*Sample Clock Source Select bits:
		  	111 = Internal counter ends sampling and starts
			  	conversion (auto-convert) */

	AD1CON3bits.SAMC = 31;	// Auto Sample Time bits. (31*Tad = 49.6us)
	AD1CON1bits.FORM = 0;	// Data Output Format bits. Integer
				/* For 12-bit operation:
				   00 = Integer
				   (DOUT = 0000 dddd dddd dddd)*/

	AD1CON1bits.AD12B = 1;	/* Operation Mode bit:
				   0 = 10 bit
				   1 = 12 bit*/
	AD1CON1bits.ASAM  = 0;	/* ADC Sample Auto-Start bit:
			       1 = Sampling begins immediately after last
			       conversion. SAMP bit is auto-set.
			   0 = Sampling begins when SAMP bit is set*/
	AD1CHS0bits.CH0NA = 0;	// MUXA  -Ve input selection (Vref-) for CH0.

	AD1CON1bits.ADON  = 1;	// ADC Operating Mode bit. Turn on A/D converter


}


/******************************************************************************************
 * TASKS					     										  *
 ******************************************************************************************/
static unsigned int value_ant=2048;
static float Pk_ant = 1.0;
static float R = 1.0;
static float Q = 0.1;


unsigned int kalman_filter(unsigned int raw_value)
{
	unsigned int filtered_value;
	float Pk,K_gain,temp_value;

	K_gain = (Pk_ant + Q)/(Pk_ant + Q + R);
	temp_value = (1.0-K_gain)*value_ant + K_gain*raw_value;
	filtered_value=(unsigned int)temp_value;
	Pk = (Pk_ant + Q)*(1 - K_gain);
	Pk_ant = Pk;
	value_ant = filtered_value;

	return (filtered_value);

}

TASK(Task1)
{
	AD1CHS0 = 5;   			// Channel 5
	AD1CON1bits.SAMP = 1;  	// Start conversion
	while(!IFS0bits.AD1IF);	// Wait till the EOC
	IFS0bits.AD1IF = 0;    	// reset ADC interrupt flag
	analog_value=ADC1BUF0;  	// Display ADC value
	kalman_value=kalman_filter(analog_value);
}

TASK(Task2)
{
	/* Blink leds every 1 second */
	LATAbits.LATA0^=1;
	put_LCD_initial_message();
}

TASK(Task3)
{

}


TASK(Task4)
{

	// Send task status to PC
	BufferOut[0]=0x01;
	BufferOut[1]=(char)(my_time>>8);
	BufferOut[2]=(char)my_time;
	BufferOut[3]=(char)(analog_value>>8);
	BufferOut[4]=(char)analog_value;
	BufferOut[5]=(char)(kalman_value>>8);
	BufferOut[6]=(char)kalman_value;
	BufferOut[7]=0x00;
	Serial_Send_Frame(BufferOut,8);
	my_time++;
}


// Main function
int main(void)
{
#ifdef MIPS40
	/* Clock setup for 40MIPS */
	/* PLL Configuration */
	PLLFBD=38; 				// M=40
	CLKDIVbits.PLLPOST=0; 	// N1=2
	CLKDIVbits.PLLPRE=0; 	// N2=2
	OSCTUN=0; 				// FRC clock use
	RCONbits.SWDTEN=0; 		//watchdog timer disable
	while(OSCCONbits.LOCK!=1); //wait for PLL LOCK
#endif
#ifdef MIPS4
	/* Clock setup for 4MIPS */
	/* No PLL Configuration */
#endif

	/* Program Timer 1 to raise interrupts */
	T1_program();

	/* Init leds */
	TRISAbits.TRISA0=0;
	TRISAbits.TRISA1=0;
	TRISAbits.TRISA2=0;

	Serial_Init();

	/* Init LCD */
	EE_lcd_init();
	EE_lcd_clear();

	/* ADC1 init */
	ADC1_init();

	SetRelAlarm(Alarm1, 1000,  100);
	SetRelAlarm(Alarm2, 1000,  1000);
	SetRelAlarm(Alarm3, 1000,  1000);
	SetRelAlarm(Alarm4, 1050,  100);

	 /* Forever loop: background activities (if any) should go here */
	for (;;);

	return 0;
}
