// This code is to control the speed of the BLDC motor in CW direction.

volatile double Ki =0.5 ,Kp = 5,dv = 0, dvmax = 40;
volatile double error = 0,Ismax = 99, Ismin = 0,Pterm = 0,Iterm = 0,Istate = 0;
volatile uint32_t d = 0;
volatile uint8_t Sp_count = 0,Sp_count2 = 0;
volatile uint8_t s = 0, count = 0, ZZ = 0,dd = 0;

void setup() 
{
   // enabling the interrupts.
   sei();
  // configuring 
   
  // starting the USART for Debugging purposes.
   Serial.begin(57600);
   
  // starting the timer1 for PI speed loop.
   TCNT1 = 0x0000;
   TCCR1A = 0;
   TIMSK1 |= (1<<TOIE3);
   TCCR1B |= (1<<CS31)|(1<<CS30);
   
  // starting the timer3 for Speed profile and making the actual speed variable Zero. 
   TCNT3 = 0x0000;
   TCCR3A = 0;
   TIMSK3 |= (1<<TOIE3);
   TCCR3B |= (1<<CS31)|(1<<CS30); 
   
    // starting the timer1 for PI current loop.
   TCNT4 = 0x0000;
   TCCR4A = 0;
   TIMSK4 |= (1<<TOIE3);
   TCCR4B |= (1<<CS31)|(1<<CS30);  
  /* setting PORTC as input port */
  DDRC =0x00;
  /* turning on the internal pull-up resistors of the MCU */
  PORTC = 0xFF;
  
  
  /* pin 11 and 12 of Arduino board are set as output for T4 and T1 switch driving*/
  DDRB |= (1<<PINB5)|(1<<PINB6);
  /* pin 8 and 9 of Arduino board are set as output for T6 and T3 switch driving*/
  DDRH |= (1<<PINH5)|(1<<PINH6);
  /* pin 6 and 5 of Arduino board are set as output for T5 and T2 switch driving*/
  DDRH |= (1<<PINH3);
  DDRE |= (1<<PINE3);
  
  /* Setting PWM output pin OC0B which is pin4 in the board as output*/
  DDRG |= (1<<PING5);
//  PORTG |= (1<<PING5);    /* since this example is for open-loop*/
  
  // starting the timer0
  TCNT0 = 0x00;
  TCCR0A |= (1<<WGM00)|(1<<COM0B1);
  TCCR0B |= (1<<WGM02)|(1<<CS01)|(1<<CS00);
  OCR0A = 100;
  OCR0B = 40;
    
  /* considering the 8-bit integer variable for monitoring the inputs */
  //uint8_t ZZ = 0;
}

void loop() 
{
  ZZ = PINC;
    switch (ZZ)
    {
      case 0b11100111:
      /* turning on T6 and T5 for 001 output */
      PORTH = 0b00101000;
      PORTE = 0x00;
      PORTB = 0x00;
      if (d < 2)
      {
        Sp_count++;
      }
      d++;
      break;
      case 0b11101011:
      /* turning on T4 and T3 for 010 output */
      PORTB = 0b00100000;
      PORTH = 0b01000000;
      PORTE = 0x00;
      d = 1;
      break;
      case 0b11101111:
      /* turning on T4 and T5 for 011 output */
      PORTB = 0b00100000;
      PORTH = 0b00001000;
      PORTE = 0x00;
      d = 1;
      break;
      case 0b11110011:
      /* turning on T1 and T2 for 100 output */
      PORTB = 0b01000000;
      PORTH = 0x00;
      PORTE = 0b00001000;
      d = 1;
      break;
      case 0b11110111:
      /* turning on T1 and T6 for 101 output */
      PORTB = 0b01000000;
      PORTH = 0b00100000;
      PORTE = 0x00;
      d = 1;
      break;
      case 0b11111011:
      /* turning on T2 and T3 for 110 output */
      PORTB = 0x00;
      PORTH = 0b01000000;
      PORTE = 0b00001000;
      d = 1;
      break;
      
      default:
      PORTB = 0x00;
      PORTH = 0x00;
      PORTE = 0x00;
      d = 1;
      break;
    }
}

ISR(TIMER3_OVF_vect)
{
  cli();
    TCNT3 = 40000;
  if (count > 100)
  {
    count = 0;
  }
  if ( count < 10)
  {
    ++s;
  }
  if (count > 90)
  {
    --s;
  }
    count++;
    // Decoupling of the speed count variable to avoid the dependency on Algorithm.
    Sp_count2 = Sp_count;
    // The speed count will be set zero in the timer3.
    Sp_count = 0;
   sei();
}

ISR(TIMER1_OVF_vect)
{
  cli();
  TCNT1 = 60000;
  // here PI Speed loop is coded.
  error = (double) (s - Sp_count2);
  Pterm = Kp*error;
  // i_block of the controller
  Istate = Istate + error;
// antiwind up method.
          if(Istate > Ismax)
          {
             Istate = Ismax;
          }
          else if(Istate < Ismin)
          {
             Istate = Ismin;
          }
   Iterm = Istate*Ki;
   dv = Pterm + Iterm;
    if (dv < 0)
    {
      dv = 0;
    }
   // Serial.println(dv);
   // dvmax is calculated for zero speed.
    dvmax = 100;
    // normalization of the dv
    dv = dv/dvmax;
    // setting the duty ratio for the timer.
    dd = (uint8_t) 100*dv;
    OCR0B = dd;
    Serial.println(Sp_count2);
  sei();
}

ISR(TIMER4_OVF_vect)
{
  cli();
  
  sei();
}

// This code is working :-)
