#include<avr/io.h>
#include<avr/interrupt.h>
#include"util/delay.h"

#define DELAY 3

unsigned char s0=0;// first segment
unsigned char s1=0;//second segment
unsigned char m0=0;//third segment
unsigned char m1=0;//fourth segment
unsigned char hour0=0;//fifth segment
unsigned char hour1=0;//sixth segment

void timer1_init()
{
	TCNT1=0;// start counting from zero
	OCR1A=997;// counting till 1 secong
	TIMSK|=(1<<OCIE1A);// enable the interrupt for timer 1
	TCCR1A=(1<<FOC1A);/* compare mode with 1024*/
	TCCR1B=(1<<WGM12)|(1<<CS12)|(1<<CS10);
}


ISR(TIMER1_COMPA_vect)
{
	s0++;

	if(s0>9)
	{
		s0=0;
		s1++;
	}
	else if(s1>=6)
	{
		m0++;
		s0=0;
		s1=0;
	}
	else if(m0>9)
	{
		m1++;
		m0=0;
		s0=0;
		s1=0;
	}
	else if(m1>=6)
	{
		hour0++;
		m1=0;
		m0=0;
		s0=0;
		s1=0;
	}
	else if(hour0>9)
	{
		hour1++;
		hour0=0;
		m1=0;
		m0=0;
		s0=0;
		s1=0;
	}
	else if(hour1 >12)
	{
		TCNT1=0;
		s0=0;
		s1=0;
		m0=0;
		m1=0;
		hour0=0;
		hour1=0;
	}
}



void Int0()// restart the counter
{
	DDRD  &= (~(1<<PD2));// pD2 input pin
	PORTD|=(1<<PD2);//  pull up resistor
	GICR  |= (1<<INT0);// enable the int0 module
	MCUCR |= (1<<ISC01) &~(1<<ISC00);// falling edge
}

void Int1()// make the watch stop
{
	DDRD  &= (~(1<<PD3));// PD3 input pin
	GICR  |= (1<<INT1);// enable interrupt 1
	MCUCR |= (1<<ISC11)|(1<<ISC10);//rising edge
}


void Int2()// resume
{
	DDRB  &= (~(1<<PB2));// PB2 input pin
	PORTB|=(1<<PB2);// internal pull up
	GICR  |= (1<<INT2);// enable interrupt
	MCUCSR &= ~(1<<ISC2);
}

ISR(INT0_vect)
{
	TCNT1=0;
	s0=0;
	s1=0;
	m0=0;
	m1=0;
	hour0=0;
	hour1=0;
}
ISR(INT1_vect)
{
	if(PIND&(1<<PD3))
	{
		TIFR|=(1<<OCF1A);
		TCCR1A=0;
		TCCR1B=0;
	}
}

ISR(INT2_vect)
{
	if(!(PINB&(1<<PB2)))
	{
		TCCR1A=(1<<FOC1A);
		TCCR1B=(1<<WGM12)|(1<<CS12)|(1<<CS10);
	}
}



int main()
{
	DDRC|=0x0f;// pin0 ,1,2,3 is output pins
	PORTC&=0xf0;
	DDRA|=0x3f;// output pins
	PORTA|=0x00;
	timer1_init();
	Int0();
	Int1();
	Int2();
	SREG|=(1<<7);// enable the i bit
	while(1)
	{
		// multiplexing
		PORTA=(1<<0);
		PORTC=s0;
		_delay_ms(DELAY);
		PORTA&=~(1<<0);

		PORTA|=(1<<1);
		PORTC=s1;
		_delay_ms(DELAY);
		PORTA&=~(1<<1);

		PORTA|=(1<<2);
		PORTC=m0;
		_delay_ms(DELAY);
		PORTA&=~(1<<2);

		PORTA|=(1<<3);
		PORTC=m1;
		_delay_ms(DELAY);
		PORTA&=~(1<<3);

		PORTA|=(1<<4);
		PORTC=hour0;
		_delay_ms(DELAY);
		PORTA&=~(1<<4);

		PORTA|=(1<<5);
		PORTC=hour1;
		_delay_ms(DELAY);
		PORTA&=~(1<<5);

	}
}


