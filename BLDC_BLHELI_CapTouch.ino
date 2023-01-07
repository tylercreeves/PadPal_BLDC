#include <avr/io.h>
#include <util/delay.h>

/*Author: Tyler Reeves
 *Purpose: A finite state machine to control the motor of a sleeping pad inflator.
 *This program works by transitioning between states (each state performs a unique function) based on user selected transition conditions.
 *
*/


//Inflator user paramiters
const double OnDelay = 1.5;           //OnDelay is the time in seconds the inflator will wait to start the fan after being pluged in
const double OnTime = 172;            //OnTime is the time the inflator will stay on in seconds before powering itself off
const double Max_DutyCycle = 90;      //98 = 3.1 amps / 90 = 2.6 amps
const double Phone_DutyCycle = 69;      //Phone_DutyCycle
const double Start_DutyCycle = 60;

//peramiters governing finite state machine behavior
const int motorPin = 0;           //Which microcontroller pin sends out the PWM signal?
const int TouchPin = 2;          //Pin for capacitive touch
const double TickDelay = 50;      //Time between state transitions in milliseconds
const uint8_t capOffset = 1;        //Capacitive offset governs the capacitive button sensitivity. Higher the number means less sensitive

//Finite State Machine variables
int Speed;                        //Speed is used to ramp up and down fan speed during state ticks. If Speed == 0 no fan, if Speed==255 Max fan.
int i;                            //used to keep track of # of ticks through a given state of the finite state machine
uint8_t ref0;
uint8_t level;

//========================Capacitive Touch Function============================//

uint8_t adc_touch()
{
    uint8_t samples = 4;
    uint8_t level = 0;

    do {
        // ADC1/PB2
        ADMUX = (1 << ADLAR) | (1 << MUX0);

        DDRB |= (1 << TouchPin);               // discharge
        // enable ADC & discharge S/H cap
        ADCSRA |= (1 << ADEN);
        _delay_us(1);                       // S/H RC time
        ADCSRA = 0;                         // ADC off
        PORTB |= (1 << TouchPin);              // charge touchpad

        DDRB &= ~(1 << TouchPin);              // input mode
        PORTB &= ~(1 << TouchPin);             // pullup off

        // enable ADC with /16 prescaler, equalize S/H cap charge
        ADCSRA = (1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0)
            | (1 << ADEN);
        _delay_us(1);                       // S/H RC time

        ADCSRA |= (1 << ADSC);
        while ( bit_is_set(ADCSRA, ADSC) );
        level += ADCH;
        ADCSRA = 0;                         // ADC off
    } while (--samples);
    return level;
}
//============================================================================//


enum States{Wait, PhoneRampUp, PhoneSustain, MaxRampUp, MaxSustain, Off} State = Wait; //Declare the states of the finite state machine and set the initial starting state.

void setup()
{
  pinMode (motorPin, OUTPUT);
  Speed=Start_DutyCycle;
  i=0;
}

void Tick(){
  switch(State){ // State transitions
    case Wait:
    if(i<(OnDelay/(TickDelay/1000))){ //Check i to see if power has been on for less than onDelay time
        State = Wait;
      }
      else {
        i=0;
        State = PhoneRampUp;
        ref0 = adc_touch();
      }
    break;
      
    case PhoneRampUp:
      if(Speed<(255*(Phone_DutyCycle/100))){ //replace 20 with (255*(DutyCycle/100))
        State = PhoneRampUp;
      }
      else if(Speed>=(255*(Phone_DutyCycle/100))){
        State = PhoneSustain;
        ref0 = adc_touch();
      }
    break;
    
    case PhoneSustain:
      level = adc_touch();
      if((i<(OnTime/(TickDelay/1000)))&&(level < (uint8_t)(ref0 + capOffset))){ //Check i to see if fan has ran for its max time. Sustain power if not.
        State = PhoneSustain;
      }
      else if(i>=(OnTime/(TickDelay/1000))){ //Check i to see if fan has run for its max time. Power off by changing speed to 0 if true.
        Speed = 0;
        State = Off;
      }
      else if(level > (uint8_t)(ref0 + capOffset)){
        State = MaxRampUp;
      }
    break;

    case MaxRampUp:
      if(Speed<(255*(Max_DutyCycle/100))){
        State = MaxRampUp;
      }
      else if(Speed>=(255*(Max_DutyCycle/100))){
        State = MaxSustain;
        ref0 = adc_touch();
      }
    break;

    case MaxSustain:
      level = adc_touch();
      if((i<(OnTime/(TickDelay/1000)))&&(level < (uint8_t)(ref0 + capOffset))){ //Check i to see if fan has ran for its max time. Sustain power if not.
        State = MaxSustain;
      }
      else if(i>=(OnTime/(TickDelay/1000))){ //Check i to see if fan has run for its max time. Power off by changing speed to 0 if true.
        Speed = 0;
        State = Off;
      }
      else if(level > (uint8_t)(ref0 + capOffset)){
        State = Wait;
      }
    break;

    case Off:
      State = Off;
    break;
  }

  switch(State){ // State Actions
    case Wait:
      i++;
      analogWrite(motorPin, Speed);
    break;
      
    case PhoneRampUp:
      Speed++;
      analogWrite(motorPin, Speed);
    break;

    case PhoneSustain:
      analogWrite(motorPin, Speed);
      i++;
    break;

    case MaxRampUp:
      Speed++;
      analogWrite(motorPin, Speed);
    break;
    
    case MaxSustain:
      analogWrite(motorPin, Speed);
      i++;
    break;

    case Off:
    analogWrite(motorPin, Speed);
    break;
  }
}




void loop()
{
  Tick();
  delay(TickDelay);
}
