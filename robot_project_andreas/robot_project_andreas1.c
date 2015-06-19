/*
 * robot_project_andreas.c
 *
 * Created: 18.05.2015 22:14:06
 *  Author: Andreas Glatz
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

void init_ddr(void);
void initpwm(void);
void gangschaltung(unsigned char richtung_l, unsigned char richtung_r);

// global variable to count the number of overflows
char toggle = 0;
char tim0 = 0;
unsigned char adc1;
unsigned char adc2;
unsigned char adc3;
unsigned char adc4;
unsigned char adcergebnis1;
unsigned char adcergebnis2;
uint16_t findline;
unsigned char direction=0;
unsigned char command=0;
unsigned char counter=0;
unsigned char regi = 0;
unsigned char tot_overflow=0;
unsigned char ultrasonic_left;
unsigned char ultrasonic_right;
unsigned char last_ultrasonic_left;
unsigned char last_ultrasonic_right;

void UART_Configuration(void);
void initadc(unsigned char links,unsigned char rechts);
void gangschaltung(unsigned char richtung_l, unsigned char richtung_r);
void initpwm(void);
void init_ddr(void);
void inittim0(void);
void inittim2(void);
void initint0(void);
void initint1(void);
void linefollow(void);
void ausgabe(unsigned char temp);






void UART_Configuration(void){
	UBRRH = 0;
	UBRRL = 51; 					//Baudrate 9600, Asynchronous Normal mode
	UCSRC =(1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0); 	//Frameformat 8Bit, 1 Stopbit, no Parity
	UCSRB =(1<<RXEN)| (1<<RXCIE)|(1<<TXEN);			//receive enabled, rx interrupt enabled
}

void initadc(unsigned char links,unsigned char rechts){
	ADCSRA |= (1<<ADEN);	//|(1<<ADPS2)|(1<<ADPS1)AD-Enable|Prescaler CLK/128
	if ((links == 0 && rechts == 0)|(links == 1 && rechts == 1))
	{
		ADMUX = (1<<REFS0)|(1<<ADLAR);		//ADC0 Select AVCC as Vref, left justify data registers and select ADC0 as input channel
	}
	
	if (links == 1 && rechts == 0)
	{
		ADMUX = (1<<REFS0)|(1<<MUX0)|(1<<MUX1)|(1<<ADLAR);		//ADC3 Select AVCC as Vref, left justify data registers and select ADC0 as input channel
	}
	
	if (links == 0 && rechts == 1)
	{
		ADMUX = (1<<REFS0)|(1<<MUX1)|(1<<ADLAR);		//ADC2 Select AVCC as Vref, left justify data registers and select ADC0 as input channel
	}
	ADCSRA |= (1<<ADSC);				// Start Conversion
}

void gangschaltung(unsigned char richtung_l, unsigned char richtung_r){
	OCR1AH = 0x00;
	OCR1AL = richtung_l;
	
	OCR1BH = 0x00;	//Endwert (Top)
	OCR1BL = richtung_r;
	
}

void initpwm(void){
	
	TCCR1A |= (1<<WGM10)|(1<<COM1A1)|(1<<COM1B1);
	TCCR1B |= (1<<CS11); //Prescaler
	
	ICR1H = 0x00;	//Startwert (Bottom)
	ICR1L = 0x00;
	
	OCR1AH = 0x00;	//Endwert (Top)
	OCR1AL = 0x00;
	
	OCR1BH = 0x00;	//Endwert (Top)
	OCR1BL = 0x00;
	
	
}

void init_ddr(void){			//DDR-config
	//OUTPUT
	DDRB |= (1<<PB1)|(1<<PB2);	//PWM
	DDRC |= (1<<PC4);			//LED
	DDRC |= (1<<PC0)|(1<<PC1);	//Ultrasonic trigger
	DDRD |= (1<<PD6)|(1<<PD7);	//motor enable
	DDRD |= (1<<PD4)|(1<<PD5);	//back led
	PORTD |= (1<<PD4)|(1<<PD5);
}

void inittim0(void){
		TIMSK |= (1<<TOIE0);         //Overflow Interrupt aktivieren
		TCCR0 |= (1<<CS01)|(1<<CS00);       // Prescaler auf 1 setzen, Takt = 8MHz
}

void inittim2(void)
{
	// set up timer with prescaler = 256
	TCCR2 |= (1 << CS21);
	
	// initialize counter
	TCNT2 = 0;
	
	// enable overflow interrupt
	TIMSK |= (1 << TOIE2);
	
	// enable global interrupts
	sei();
	
	// initialize overflow counter variable
	tot_overflow = 0;
}
void initint0(void)
{
	GICR |= (1<<INT0);
	MCUCR |= (1<<ISC01);
	sei();
}
void initint1(void)
{
	GICR |= (1<<INT1);
	MCUCR |= (1<<ISC11);
	sei();
}

void linefollow(void)
{
		char x = 25;
		if (adcergebnis1 < x && adcergebnis2 > x)	//at the line right
		{
			findline = 900;
			direction = 1;
			gangschaltung(125,0);
			PORTD |= (1<<PD6)|(1<<PD7);
		}
		if (adcergebnis1 > x && adcergebnis2 < x)	//at the line left
		{
			findline = 900;
			direction = 2;
			gangschaltung(0,125);
			PORTD |= (1<<PD6)|(1<<PD7);
		}
		if (adcergebnis1 < x && adcergebnis2 < x)	//on the line
		{
			findline = 900;
			gangschaltung(0,0);
			PORTD |= (1<<PD6)|(1<<PD7);
		}
		if (adcergebnis1 > x && adcergebnis2 > x) //No line
		{
			findline++;
			
			if (findline<=1600)
			{
				if (direction==2)
				{
					gangschaltung(0,255);
					PORTD |= (1<<PD7);
					PORTD &= ~(1<<PD6);
				}
				else
				{
					gangschaltung(255,0);
					PORTD |= (1<<PD6);
					PORTD &= ~(1<<PD7);
				}
			}
			if (findline>1600)
			{
				gangschaltung(255,100);
				PORTD |= (1<<PD6);
				PORTD &= ~(1<<PD7);
			}
			if (findline>6000)
			{
				gangschaltung(0,0);
				PORTD |= (1<<PD6)|(1<<PD7);
			}
			if (findline>8000)
			{
				findline=0;
			}
			
		}
}


void ausgabe(unsigned char temp){
	while(!(UCSRA & (1<<UDRE))){};
	UDR = temp;
	while(!(UCSRA & (1<<TXC))){};
	UCSRA |= (1<<TXC);
}


int main(void)
{

	sei();           // Globale Interrupts aktivieren
	inittim2();
	initint0();
	initint1();
	init_ddr();
	initpwm();
	inittim0();
	initadc(1,0);
	UART_Configuration();
    char x = 3;
	while(1)
    {
		// Interrupt register----------------------
		if (regi & (1<<0)) // Timer2
		{
			regi &= ~(1<<0);
			if (tot_overflow > 254)
			{
				tot_overflow = 0;
			}
			else
			{
				tot_overflow++;
			}
			
			
		}
		if (regi & (1<<1)) // USART
		{
			regi &= ~(1<<1);			
			command=UDR; //commands in main
			command++;
			ausgabe(command);
			command--;
		}
		
		if (regi & (1<<2)) // INT0
		{
			regi &= ~(1<<2);
			last_ultrasonic_left=ultrasonic_left;
			ultrasonic_left = tot_overflow;
			ultrasonic_left -= 125;
			ausgabe(ultrasonic_left);
			
		}
		if (regi & (1<<3)) // INT1
		{
			regi &= ~(1<<3);
			last_ultrasonic_right=ultrasonic_right;
			ultrasonic_right = tot_overflow;
			ausgabe(ultrasonic_right);
			ausgabe(adcergebnis1);
			ausgabe(adcergebnis2);
		}
		if (regi & (1<<4)) // Timer0
		{
			regi &= ~(1<<4);
			tim0++;
			if (tim0 > 1)
			{
				PORTC |= (1<<PC4);
				if (tim0 > 2)
				{
					tim0 = 0;
					
					
					initadc(1,0);
					adc2 = ADCH;
					initadc(0,1);
					adc3 = ADCH;
					adcergebnis1 = adc1 - adc2;
					adcergebnis2 = adc3 - adc4;

				}
			}
			else
			{
				PORTC &= ~(1<<PC4);
				if (tim0 > 0)
				{
					initadc(1,0);
					adc1 = ADCH;
					initadc(0,1);
					adc4 = ADCH;
				}
			}
		}
		
		// I-Register end---------------------------------
		// Ultrasonic trigger ----------------------------
		if (tot_overflow == 0)
		{
			GICR |= (1<<INT1);
			GICR &= ~(1<<INT0);
			PORTC |= (1<<PC1);
			PORTC &= ~(1<<PC0);
		}
		if (tot_overflow == 125)
		{
			GICR |= (1<<INT0);
			GICR &= ~(1<<INT1);
			PORTC &= ~(1<<PC1);
			PORTC |= (1<<PC0);
		}
		// Ultrasonic trigger end ------------------------
		if (command == 'b')
		{
			gangschaltung(0,0);
			PORTC &= ~(1<<PC4);
			PORTD |= (1<<PD6)|(1<<PD7);
		}
		if (command == 'c')
		{
			gangschaltung(255,255);
			PORTC &= ~(1<<PC4);
			PORTD |= (1<<PD6)|(1<<PD7);
		}
		if (command == 'f')
		{
			char x = 3;
			if ((adcergebnis1 < x) && (adcergebnis2 > x))	//at the line right
			{
				findline = 900;
				direction = 1;
				gangschaltung(125,0);
				PORTD |= (1<<PD6)|(1<<PD7);
			}
			if (adcergebnis1 > x && adcergebnis2 < x)	//at the line left
			{
				findline = 900;
				direction = 2;
				gangschaltung(0,125);
				PORTD |= (1<<PD6)|(1<<PD7);
			}
			if (adcergebnis1 < x && adcergebnis2 < x)	//on the line
			{
				findline = 900;
				gangschaltung(0,0);
				PORTD |= (1<<PD6)|(1<<PD7);
			}
			if (adcergebnis1 > x && adcergebnis2 > x) //No line
			{
				findline++;
						
				if (findline<=1600)
				{
					if (direction==2)
					{
						gangschaltung(0,255);
						PORTD |= (1<<PD7);
						PORTD &= ~(1<<PD6);
					}
					else
					{
						gangschaltung(255,0);
						PORTD |= (1<<PD6);
						PORTD &= ~(1<<PD7);
					}
				}
				if (findline>1600)
				{
					gangschaltung(255,100);
					PORTD |= (1<<PD5);
					PORTD &= ~(1<<PD4);
					PORTD |= (1<<PD6);
					PORTD &= ~(1<<PD7);
				}
				if (findline>6000)
				{
					gangschaltung(0,0);
					PORTD |= (1<<PD5);
					PORTD |= (1<<PD4);
					PORTD |= (1<<PD6)|(1<<PD7);
				}
				if (findline>8000)
				{
					findline=0;
				}
						
			}
		}
		if (command == 'd')
		{
			if (ultrasonic_right < ultrasonic_left)
			{
				PORTD |= (1<<PD6);
				PORTD &= ~(1<<PD7);
				gangschaltung(125,125);
			}
			if (ultrasonic_left < ultrasonic_right)
			{
				PORTD |= (1<<PD7);
				PORTD &= ~(1<<PD6);
				gangschaltung(125,125);
			}	
			if (ultrasonic_left == ultrasonic_right)
			{
				PORTD &= ~(1<<PD6);
				PORTD &= ~(1<<PD7);
				gangschaltung(255,255);
			}
			if (ultrasonic_left < 3 && ultrasonic_right < 3)
			{
				PORTD &= ~(1<<PD6);
				PORTD &= ~(1<<PD7);
				gangschaltung(255,255);
			}
		}
		
    }	
}

// TIMER0 overflow interrupt service routine
// called whenever TCNT0 overflows
ISR(TIMER2_OVF_vect)
{
	regi |= (1<<0);
}

ISR(USART_RXC_vect)
{
	regi |= (1<<1);
}

ISR(INT0_vect)
{
	regi |= (1<<2);
}

ISR(INT1_vect)
{
	regi |= (1<<3);
}

ISR(TIMER0_OVF_vect)
{	
	regi |= (1<<4);
}