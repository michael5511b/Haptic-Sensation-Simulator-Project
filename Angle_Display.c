/*
 * Lab2_AngleDisplay.c
 *
 * Created: 3/7/2017 3:57:31 PM
 *  Author: michaelcheng
 */ 


#define F_CPU 16000000L 
#define BAUD 9600   
#define VREF 5

#include <util/setbaud.h>  
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>  
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>
#include <math.h>


#define ADC_PIN	1
#define	LED_PIN	PB0


volatile uint8_t tot_overflow;  



void uart_init (void)
{
	UBRR0H = UBRRH_VALUE; 
	UBRR0L = UBRRL_VALUE;
	
	UCSR0B = (1<<TXEN0)|(1<<RXEN0);				   
	UCSR0C = (0<<USBS0)|(1<<UCSZ00)|(1<<UCSZ01);  
}


void uart_transmit (unsigned char data)
{
	while (!(UCSR0A & (1<<UDRE0)));                
	UDR0 = data;                                  
}



unsigned char uart_receive (void)
{
	while ( !(UCSR0A & (1<<RXC0)) );               
	return UDR0;                                   
}


void printfloat(float number)
{
	uint8_t *p = (uint8_t *)&number; // uses a pointer and an array.
	uart_transmit(p[0]);
	uart_transmit(p[1]);
	uart_transmit(p[2]);
	uart_transmit(p[3]);
}





void InitADC()
{
	ADMUX|=(1<<REFS0);
	ADCSRA|=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}



float ReadADC(uint8_t ADCchannel)
{
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
	//single conversion mode
	ADCSRA |= (1<<ADSC);
	// wait until ADC conversion is complete
	while( ADCSRA & (1<<ADSC) );
	return ADC;
}

void timer0_init(void)
{
// set up timer //WITH PRESCALER
	TCCR0B |= (1<< CS02) | (1 << CS00) ; // it seems to be 1024 
	//TCCR0B |= (1 << CS00);
// initialize counter
	TCNT0=0;
	//ENABLE OWVERFLOW INTERUPTS
	TIMSK0 |= (1<< TOIE0);
	//ENABLE GLOBAL INTERUPTS
	sei();
	 // init overflow counter var
	tot_overflow=0;
}




  

void timer1_init(void)
{
	// set up timer //WITH PRESCALER
	TCCR1B|= (1<<CS10);//(1<< CS02) | (1 << CS00) ; // 
	//TCCR0B |= (1 << CS00);
	// initialize counter
	TCNT1=0;
	 // enable overflow interrupt
	 //TIMSK |= (1 << TOIE1);
	//ENABLE GLOBAL INTERUPTS
	sei();
	// init overflow counter var
	tot_overflow=0;
}


float Vol2D(float volt)
{
	if (volt>=0.4834 && volt<=0.714)	
	{
		return 86.580*volt+118.182;
	}
	else 
	{
		return 89.804*volt-244.12;
	}
	
}
ISR(TIMER0_OVF_vect)
{
	
	tot_overflow++;
}



float currentD;
float readV;

int main(void)
{
    timer0_init();
	timer1_init();
	uart_init(); 
	InitADC(); 
	
	
	DDRC = (1 << PINC0);
	PORTC = (1 << PINC0);
	
	
	
	while(1)
	{
		
        

	   if (TCNT1 >=15999) //1ms
	   {
			readV = (float)VREF/1024 * ReadADC(ADC_PIN);
			currentD = Vol2D(readV);
			TCNT1 = 0;
	   }

       if (TCNT0 >= 155) //10 ms
	   {
		    //printfloat(readV);
			printfloat(currentD);
			TCNT0=0;
	   }	
	}
	return 0;
}