#define F_CPU 16000000L
#define BAUD 9600    // Define baud rate
#define VREF 5// Reference voltage

#include <stdbool.h>
#include <util/setbaud.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>
#include <math.h>


#define ADC_PIN	1 // Defining which analog pin to read, We are reading ADC1, thus 1
#define	LED_PIN	PB0

//Declaring variables
float curr_Deg;
float curr_Vel;
float curr_Vol;
float prev_Vol;
float prev_Deg;
float prev_Vel;
float vel_Out;
float vel_1;
float vel_2;
float fvel_1;
float fvel_2;
float fvel;
//Tested saturation values
float sat_High = 120;
float sat_Low = 100;
float velocity[9];
float calcVal;
int i=0;
float expected_Out;
float curr_err;
float prev_err;
float control_Vol;
float prev_I;
float P;
float I;
float D;
float Kp;
float Ki;
float Kd;
float dutyCycle_float;


volatile uint16_t time_count;  // Uses of interrupt


// Initializing UART
void uart_init (void)
{
	UBRR0H = UBRRH_VALUE; // enabled by setbaud.h
	UBRR0L = UBRRL_VALUE;
	UCSR0B = (1<<TXEN0)|(1<<RXEN0);				   // enable receiver and transmitter
	UCSR0C = (0<<USBS0)|(1<<UCSZ00)|(1<<UCSZ01);   // set frame format: 1 stop bit, 8bit data
}


// Data transmit function
void uart_transmit (unsigned char data) //(unsigned char data)
{
	while (!(UCSR0A & (1<<UDRE0)));                // wait for empty transmit buffer
	UDR0 = data;                                   // load data in the register, which sends it
}


// Data receive function
unsigned char uart_receive (void)
{
	while ( !(UCSR0A & (1<<RXC0)) );               // wait for data to be received
	return UDR0;                                   // get and return 8-bit data from buffer
}


// Float transmit function
void printfloat(float number)
{
	uint8_t *p = (uint8_t *)&number;
	uart_transmit(p[0]);
	uart_transmit(p[1]);
	uart_transmit(p[2]);
	uart_transmit(p[3]);
}


// Initializing ADC
void InitADC()
{
	ADMUX|=(1<<REFS0); // Sets 5v internal source as reference voltage
	ADCSRA|=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // ADEN enables the ADC, ADPS0~2 sets pre-scaler
}


// Function that reads the ADC value from the designated channel
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


// Function that converts the voltage reading into degrees
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


// Function that calculates the angular velocity
float D2Vel(float curr_Deg, float prev_Deg)
{
	return (curr_Deg-prev_Deg)/0.01; //We are sampling every 1ms
}


// Apply filter (Butterworth filter)
float butter(float t, float t_step1, float t_step2,float tf_step1, float tf_step2)
{

	//float b1 = 0.0002414; // wn = 5/( (1/dt)/2)
	//float b1 = 0.0009447; // wn = 10/( (1/dt)/2)
	float b1 = 0.005543; // wn = 25/( (1/dt)/2)
	//float b1 = 0.06746; // wn = 100/( (1/dt)/2)
	//float b2 = 0.0004827; // wn = 5/( (1/dt)/2)
	//float b2 = 0.001889; // wn = 10/( (1/dt)/2)
	float b2 = 0.01109; // wn = 25/( (1/dt)/2)
	//float b2 = 0.1349; // wn = 100/( (1/dt)/2)
	float b3 = b1;
	float a1 = 1;
	//float a2 = -1.956; // wn = 5/( (1/dt)/2)
	//float a2 = -1.911; // wn = 10/( (1/dt)/2)
	float a2 = -1.779; // wn = 25/( (1/dt)/2)
	//float a2 = -1.143; // wn = 100/( (1/dt)/2)
	//float a3 = 0.9565; // wn = 5/( (1/dt)/2)
	//float a3 = 0.915; // wn = 10/( (1/dt)/2)
	float a3 = 0.8008; // wn = 25/( (1/dt)/2)
	//float a3 = 0.4128; // wn = 100/( (1/dt)/2)

	float Out = (b1*t + b2*t_step1 + b3*t_step2 - a2*tf_step1 - a3*tf_step2);

	return Out;
}


// Saturation function (this will fix the issue of discontinuities occurring at breaking point and 180/-180 degrees)
float vel_after_sat(float curr_Deg,float prev_Deg, float sat_Low,float sat_High,float prev_Vel)
{
	if ((prev_Deg <= sat_Low && curr_Deg >= sat_Low) || (prev_Deg >= sat_High && curr_Deg <= sat_High))
	{
		return prev_Vel;
	}
	return D2Vel(curr_Deg,prev_Deg);
}


//Initializing timer0
void timer0(void)
{
	OCR0A = 155; //interrupts happen when the counter hits this value

	TCNT0 = 0;

	TIMSK0 = TIMSK0 | (1<<OCIE0A); 	//enable compare match interrupt

	TCCR0B |= (1<< CS02) | (1 << CS00);

	TCCR0A =(1<<WGM01);  //CTC

	sei();
}




/////////////////////////PWM CODES////////////////////////////

//Initialize duty cycle
uint8_t dutyCycle = 128;

//Initializes timer2 for PWM control
void pwm_init(void)
{
	DDRB |= (1 << DDB0); //Set pin 8 as output
	DDRB |= (1 << DDB3); //Set pin 11 (correspond to motor output B on the Ardumoto) as output

	//Set fast PWM Mode
	TCCR2A |= (1 << WGM21) | (1 << WGM20);

	//Set pre-scaler clock to 256 and starts PWM
	TCCR2B |= (1 << CS21)|(1 << CS22);

	//Set duty cycle, the outputs will be switched on/off if counter hits this value
	OCR2A = dutyCycle;
}


//Switches the power of the motor on
void switch_ON(void)
{
	PORTB |= (1 << DDB0); //Enable 12V into the board
	PORTB |= (1 << DDB3); //Set pin 11 as HIGH

}


//Switches the power of the motor off
void switch_OFF(void)
{
	//Clear all bits
	PORTB &= ~(1 << DDB0);
	DDRB &= ~(1 << DDB5);
	PORTB &= ~(1 << DDB5);
	PORTB &= ~(1 << DDB3);
	TCCR2A &= ~(1 << COM2A0)|(1 << COM2A1);
}


//Makes the rotation of the motor counter-clockwise
void CCW(void)
{
	//CCW
	DDRB |= (1 << DDB5); //Set DIR B as output
	PORTB |= (1 << DDB5); //Activate DIR B
	//Since Activating DIR B inverts both the direction and the LOW/HIGH, we have to use invert mode
	TCCR2A |= (1 << COM2A0)|(1 << COM2A1); //Inverted mode (LOW at bottom, HIGH on Match)
	switch_ON();
}


//Makes the rotation of the motor clockwise
void CW(void)
{
	//CW
	PORTB &= ~(1 << DDB5); //De-activate DIR B
	TCCR2A |= (0 << COM2A0)|(1 << COM2A1); //Non-inverted mode (HIGH at bottom, LOW on Match)
	switch_ON();
}


/////////////////////////Controls////////////////////////////


ISR(TIMER0_COMPA_vect)
{
	//switch_OFF();
	curr_Vol = (float)VREF/1024 * ReadADC(ADC_PIN);
	curr_Deg = Vol2D(curr_Vol);

	//Removing discontinuity
	//if ((prev_Deg <= -160 && curr_Deg >= -160) || (prev_Deg >= 160 && curr_Deg <= 160))
	//{
	//curr_Deg = prev_Deg;
	//}

	//Calculate the current error
	curr_err = expected_Out - curr_Deg;

	//Calculate the three parts of the PID control
	P = Kp * curr_err;
	I = prev_I + (Ki * curr_err * 0.01);
	D = (Kd * (curr_err - prev_err))/0.01;

	//The wall
	if(curr_Deg>45)
	{
		Kp = 0.07;
		Kd = 0.0004;
	}
	else
	{
		Kp = 0;
		Kd = 0;
	}

	//Get the control voltage from summing all three PID values
	control_Vol = P + D;

	//PWM saturation
	if (control_Vol > 9.65)
	{
		control_Vol = 9.65;
	}
	else if(control_Vol < -9.65)
	{
		control_Vol = -9.65;
	}


	//Convert the control voltage into duty cycle, duty cycles can't be negative, so keep the absolute value of duty cycle and switch the direction
	if (control_Vol > 0.3)
	{
		dutyCycle_float =  abs((control_Vol/9.65)*255);
		dutyCycle = (int)dutyCycle_float; //duty cycles can only be integers, so round the duty cycle value

		pwm_init();
		CCW();
	}
	else if (control_Vol < -0.3)
	{
		dutyCycle_float =  abs((control_Vol/9.65)*255);
		dutyCycle = (int)dutyCycle_float; //duty cycles can only be integers, so round the duty cycle value

		pwm_init();
		CW();
	}
	else
	{
		switch_OFF();
	}

	//Serial communication, sends angle data
	printfloat(curr_Deg);

	//After outputting the angle, set the variables as the previous time step
	prev_Deg = curr_Deg;
	prev_err = curr_err;
	prev_I = I;

	//reset counter
	TCNT0 = 0;
}



int main(void)
{
	//Initializing everything

	InitADC(); //Initializing the Analog-Digital Converter
	prev_Vol = (float)VREF/1024*ReadADC(ADC_PIN); //Read the initial voltage before timer starts
	prev_Deg = Vol2D(prev_Vol);
	expected_Out = 0; //Expected Output value

	//Set the K values for PID control
	//Kp = 0.05;

	//Kd = 0.0004;

	//Get the initial error and integral
	prev_err = expected_Out - prev_Deg;
	prev_I = Ki * prev_err * 0.01;

	timer0(); //Initializing timer0
	//pwm_init();

	// Initializing UART
	uart_init();



	//Power up the Hall Effect sensor
	DDRC = (1 << PINC0);
	PORTC = (1 << PINC0);

	while(1)
	{


	}
	return 0;
}
