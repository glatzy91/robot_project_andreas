/*
 * robot_project.c
 *
 * Created: 18.05.2015 22:14:06
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

void init_ddr(void);
void initpwm(void);
void drive(unsigned char direction_l, unsigned char direction_r);

// global variable to count the number of overflows
char tim0 = 0;					//Counter that incements if tim0 has an overflow -> system clock
unsigned char tot_overflow=0;	//Counter that increments if tim2 has an overflow -> ultrasonic timings
unsigned char counter=0;		//Counter then increments if tim0 has 4 overflows -> time for the backleds
unsigned char adc1;				//Fototransistor Left with LED on
unsigned char adc2;				//Fototransistor Left with LED off
unsigned char adc3;				//Fototransistor right with LED on
unsigned char adc4;				//Fototransistor right with LED off
unsigned char foto_right;		//difference between adc3 and adc4	
unsigned char foto_left;		//difference between adc1 and adc2
unsigned char command=0;		//Command from the bluetooth
unsigned char regi = 0;			//Interrupt register
unsigned char ultrasonic_left;	//Time/distance between trigger and echo signal left ultrasonic
unsigned char ultrasonic_right;	//Time/distance between trigger and echo signal right ultrasonic
unsigned char last_ultrasonic=2;//memory variable for last position
signed char diff;				//control deviation
int y=0,y2=0,mleft=0,mright=0;//control parameter

void UART_Configuration(void);
void initadc(unsigned char left,unsigned char right);
void drive(unsigned char direction_l, unsigned char direction_r);
void initpwm(void);
void init_ddr(void);
void inittim0(void);
void inittim2(void);
void initint0(void);
void initint1(void);
void linefollow(void);
void UART_output(unsigned char temp);
void backward(void);
void forward(void);
void turn(void);
void killenemy(void);






void UART_Configuration(void){
	UBRRH = 0;
	UBRRL = 51; 					//Baudrate 9600, Asynchronous Normal mode
	UCSRC =(1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0); 	//Frameformat 8Bit, 1 Stopbit, no Parity
	UCSRB =(1<<RXEN)| (1<<RXCIE)|(1<<TXEN);			//receive enabled, rx interrupt enabled
}
void initadc(unsigned char left,unsigned char right){
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1);//AD-Enable|Prescaler CLK/128

	if (left == 1 && right == 0)
	{
		ADMUX = (1<<REFS0)|(1<<MUX0)|(1<<MUX1)|(1<<ADLAR);		//ADC3 Select AVCC as Vref, left justify data registers and select ADC0 as input channel
	}
	
	if (left == 0 && right == 1)
	{
		ADMUX = (1<<REFS0)|(1<<MUX1)|(1<<ADLAR);		//ADC2 Select AVCC as Vref, left justify data registers and select ADC0 as input channel
	}
	ADCSRA |= (1<<ADSC);				// Start Conversion
	while(ADCSRA & (1<<ADSC)){}
}
void drive(unsigned char direction_l, unsigned char direction_r){
	OCR1AH = 0x00;
	OCR1AL = direction_l;
	
	OCR1BH = 0x00;	//Endvalue (Top)
	OCR1BL = direction_r;
	
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
void UART_output(unsigned char temp){
	while(!(UCSRA & (1<<UDRE))){};
	UDR = temp;
	while(!(UCSRA & (1<<TXC))){};
	UCSRA |= (1<<TXC);
}
void killenemy(void)
{
		
		if (ultrasonic_right > ultrasonic_left)
		{
			PORTD &= ~(1<<PD7);
			PORTD &= ~(1<<PD6);
			drive(255,200);
			last_ultrasonic=1;//left
		}
		if (ultrasonic_left > ultrasonic_right)
		{
			PORTD &= ~(1<<PD6);
			PORTD &= ~(1<<PD7);
			drive(200,255);
			last_ultrasonic=0;//right
		}
		
		if (ultrasonic_left == ultrasonic_right)
		{
			PORTD &= ~(1<<PD6);
			PORTD &= ~(1<<PD7);
			drive(255,255);
			last_ultrasonic=2;//center
		}
	
	/*else
	{
		if (last_ultrasonic == 2)
		{
			PORTD &= ~(1<<PD7);
			PORTD &= ~(1<<PD6);
			drive(200,200);
		}
		else if (last_ultrasonic == 1)
		{
			PORTD |= (1<<PD6);
			PORTD &= ~(1<<PD7);
			drive(0,255);
			
		}
		else if (last_ultrasonic == 0)
		{
			PORTD |= (1<<PD7);
			PORTD &= ~(1<<PD6);
			drive(255,0);
		}	
	}*/
}

void backward(){
	drive(0,0);
	PORTD |= (1<<PD6)|(1<<PD7);
}

void forward(){
	drive(255,255);
	PORTD &= ~(1<<PD6);
	PORTD &= ~(1<<PD7);
}

void turn(){
	drive(255,0);		//turn
	PORTD |= (1<<PD6);
	PORTD &= ~(1<<PD7);
}

void linefollow(void) 
{
		PORTD|=(1<<PD5);
		PORTD&=~(1<<PD4);
		unsigned char kp=35;
		PORTD&=~(1<<PD6);
		PORTD&=~(1<<PD7);
		y=diff*kp;
		y2=y/2;
		mright=210;
		mleft=210;
		
		if(y>0){
			mright+=y2;
			if (mright>255)
			{
				mright=255;
			}
			mleft-=y2;
			if (mleft<0)
			{
				mleft=0;
			}
		}
		
		if(y<0){
			mleft-=y2;
			if (mleft>255)
			{
				mleft=255;
			}
			mright+=y2;
			if (mright<0)
			{
				mright=0;
			}
		}
		drive(mleft,mright);
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
		}
		
		if (regi & (1<<2)) // INT0
		{
			regi &= ~(1<<2);
			ultrasonic_left = tot_overflow;
			ultrasonic_left -= 125;
			/*UART_output(0);
			UART_output(mleft);
			UART_output(1);
			UART_output(mright);
			//UART_output(2);
			//UART_output(diff);
			UART_output(3);
			UART_output(y2);*/
			UART_output(ultrasonic_left);
			UART_output(ultrasonic_right);
			
			
		}
		if (regi & (1<<3)) // INT1
		{
			regi &= ~(1<<3);
			ultrasonic_right = tot_overflow;
			//UART_output(1);
			//UART_output(ultrasonic_left);
			//UART_output(2);
			//UART_output(ultrasonic_right);
			//UART_output(adc1);
			//UART_output(adc2);
			//UART_output(0);
			//UART_output(foto_right);
			//UART_output(adc3);
			//UART_output(adc4);
			//UART_output(foto_right);
			//UART_output(1);
			//UART_output(foto_left);
		}
		if (regi & (1<<4)) // Timer0
		{
			regi &= ~(1<<4);
			
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
			backward();
		}
		if (command == 'f')
		{
			forward();
		}
		if (command == 't')
		{
			turn();
		}
		if (command == 'S')
		{
			cli();
			diff=0;
			mright=0;
			mleft=0;
			drive(255,255);
			PORTD |= (1<<PD6)|(1<<PD7);	
		}
		if (command == 'E')
		{	
			
			diff=0;
			mright=0;
			mleft=0;
			drive(255,255);
			PORTD |= (1<<PD6)|(1<<PD7);
			if(counter>=150)
			{
				PORTD^=(1<<PD4);
				PORTD^=(1<<PD5);
				counter=0;
			}
			
		}
		if (command == 'F')
		{
			linefollow();
		}
		if (command == 'B')
		{
			
			PORTD|=(1<<PD4);
			PORTD|=(1<<PD5);
			
			if (foto_left > 6 && foto_right > 6)
			{	
				killenemy();
				
			}
			else
			{
				counter = 0;
				while(counter < 50)
				{
					drive(0,0);		//Drive backward
					PORTD |= (1<<PD6)|(1<<PD7);
				}
				while(counter < 100)
				{
					drive(255,0);		//turn
					PORTD |= (1<<PD6);
					PORTD &= ~(1<<PD7);
				}
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
	tim0++;
	if (tim0 > 1)
	{
		PORTC |= (1<<PC4);
		if (tim0 > 2)
		{
			tim0 = 0;
			
			
			initadc(1,0);
			adc1 = ADCH;
			initadc(0,1);
			adc3 = ADCH;
			foto_right = adc1 - adc2;
			foto_left = ((adc3 - adc4)/2)-3;
			diff = foto_left-foto_right;
			counter++;
		}
	}
	else
	{
		PORTC &= ~(1<<PC4);
		if (tim0 > 0)
		{
			initadc(1,0);
			adc2 = ADCH;
			initadc(0,1);
			adc4 = ADCH;
		}
	}
}
