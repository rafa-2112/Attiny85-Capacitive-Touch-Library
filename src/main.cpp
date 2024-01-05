/*******************************************************************
Author: Rafael Reyes
Date: 01/05/2024
Contact: catto.electronics@gmail.com

Description: Turns any ADC input channel pin into a capacitive  
touch sensor.

References: Attiny85 Datasheet
*******************************************************************/

#include <avr/io.h>

#define F_CPU 8000000UL  // 8 MHz

#include <avr/interrupt.h>
#include <util/delay.h>

uint16_t dischargeCounter = 0; // Time until capacitor discharge
uint16_t min_time = 0;

// STATUS flags 
// Bit         7 6 5 4 3 2       1
// Function                 calibration
//uint8_t  flag = 0b00000001; 

void Init();
void InitADC();
void TIMER0_Init();
void Init_interrupts();

uint16_t readADC();

int main()
{
  Init();
  InitADC();
  TIMER0_Init();
    
  _delay_ms(100);
  while(1)
  {
    PORTB |= (1 << PB2); // SET pin 2 HIGH
    DDRB  |= (1 << PB2); // SET pin 2 to OUTPUT
    
    _delay_us(1); // WAIT to charge capacitor
    
    DDRB  &= ~(1 << PB2); // CLEAR pin 2 to INPUT
    PORTB &= ~(1 << PB2); // CLEAR pin 2 to Tri-state(HI-Z)
    
    dischargeCounter = 0; // CLEAR discharge counter
    while(readADC() > 163)  // WAIT until input reaches 0.8V
      dischargeCounter++;
    
    if(min_time == 0)
      min_time = dischargeCounter;
    
    if(dischargeCounter > (min_time+5))
    {
      PORTB |= (1 << PB1);  // SET pin 1 HIGH
      _delay_ms(1000);
      PORTB &= ~(1 << PB1); //CLEAR pin 1 LOW
    }
  }

  return 0;
}

void Init()
{
  DDRB |= (1 << PB1);  // SET pin 1 to OUTPUT
  DDRB &= ~(1 << PB2); // CLEAR pin 2 to INPUT
}

void InitADC()
{
  ADCSRA |= (1 << ADEN);  // ENABLE ADC
  ADCSRA |= (1 << ADPS2)|(1 << ADPS1); // Prescale of 64 50kHz<(8MHz/64)<250kHz
  ADMUX  |= (1 << MUX0); // READ INPUT from pin 2
}

uint16_t readADC()
{
  uint16_t ADC_result;
  
  ADCSRA |= (1 << ADSC); // START conversion
  
  while(ADCSRA & (1 << ADSC)) // WAIT until conversion is complete
    continue;
  
  ADC_result = ADC;
  return ADC_result;
}

void TIMER0_Init()
{
  // Timer/Counter in NORMAL MODE at 31.25KHz
  TCCR0A |= (1 << COM0A1)|(1 << COM0A0); // Normal port operation | pg.79 in datasheet
  TCCR0A |= (1 << WGM01)|(1 << WGM00); // Normal operation default values
  TCCR0B |= (1 << CS00); // No prescaling F0 = 8MHz/(1*256) = 31.25KHz 
                         // 1ms = 32 * (1/31.25KHz)
}

void Init_interrupts()
{
  sei(); // ENABLE interrupts

  TIMSK |= (1 << TOIE0); // ENABLE Timer/Counter0 overflow interrupt 
}

ISR(TIMER0_OVF_vect)
{
  
}