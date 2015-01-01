// based largely on Atmel's AVR136: Low-Jitter Multi-Channel Software PWM Application Note:
// http://www.atmel.com/dyn/resources/prod_documents/doc8020.pdf

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define CHMAX 3 // maximum number of PWM channels
#define PWMDEFAULT 0x00 // default PWM value at start up for all channels

#define RED_CLEAR (pinlevelB &= ~(1 << RED)) // map RED to PB0
#define GREEN_CLEAR (pinlevelB &= ~(1 << GREEN)) // map GREEN to PB1
#define BLUE_CLEAR (pinlevelB &= ~(1 << BLUE)) // map BLUE to PB2

//! Set bits corresponding to pin usage above
#define PORTB_MASK  (1 << PB0)|(1 << PB1)|(1 << PB2)

#define set(x) |= (1<<x) 
#define clr(x) &=~(1<<x) 
#define inv(x) ^=(1<<x)

#define RED PB0
#define GREEN PB1
#define BLUE PB2
#define LED_PORT PORTB
#define LED_DDR DDRB

void delay_ms(uint16_t ms);
void init();

unsigned char compare[CHMAX];
volatile unsigned char compbuff[CHMAX];

int r_val = 0x00;
int g_val = 0x00;
int b_val = 0x00;
float dim = 1;


int rnds[] = {
	254,254,254,
	254,0,0,
	0,254,0,
	0,0,254,
	254,254,0,
	254,0,254,
	0,254,254};
int rnd_i = -1;

int rand = 89;
int get_rand()
{
	return rand = (rand*109+89)%251;//251;
	//rnd_i++;
	//if (rnd_i == 12)
	//	rnd_i = 0;
	//return rnds[rnd_i];
}

int steps = 100;

int timerCycles = 22; // 3.25 s per cycle.

int main()
{
	init();
	
	int i_rnd_last = -1;
	int i_rnd=0;
	
	for (int iTimerCycle = 0;iTimerCycle < timerCycles;iTimerCycle++)
	{
		do
		{
			i_rnd = get_rand()%7;
		}while (i_rnd == i_rnd_last);
		i_rnd_last = i_rnd;
		
		int r_target = rnds[i_rnd*3];
		int g_target = rnds[i_rnd*3+1];
		int b_target = rnds[i_rnd*3+2];
		
		for (int iCycle = steps; iCycle > 0; iCycle--)
		{
			
			r_val += (int)(((double)(r_target - r_val)) / iCycle);
			g_val += (int)(((double)(g_target - g_val)) / iCycle);
			b_val += (int)(((double)(b_target - b_val)) / iCycle);
			
	    compbuff[0] = r_val;
  	  compbuff[1] = g_val;
    	compbuff[2] = b_val;
    
			delay_ms(100);
		}
		
		delay_ms(10000);
	}
	
	compbuff[0] = 0;
	compbuff[1] = 0;
	compbuff[2] = 0;
	
	delay_ms(100);
	
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
}

int main1() {
  init();
  int r_factor = 7;
  int g_factor = 17;
  int b_factor = 29;
  
  int r_dir = r_factor;
  int g_dir = g_factor;
  int b_dir = b_factor;
  
  for(;;) {
  	
    if (r_val > 254 - r_factor) {
      r_dir = -r_factor;
    }
    if (r_val < 1 + r_factor) {
      r_dir = r_factor;
    }

    if (g_val > 254 - g_factor) {
      g_dir = -g_factor;
    }
    if (g_val < 1 + g_factor) {
      g_dir = g_factor;
    }

    if (b_val > 254 - b_factor) {
      b_dir = -b_factor;
    }
    if (b_val < 1 + b_factor) {
      b_dir = b_factor;
    }
    
    r_val += r_dir;
    g_val += g_dir;
    b_val += b_dir;

    compbuff[0] = r_val;
    compbuff[1] = g_val;
    compbuff[2] = b_val;

    delay_ms(500);
  }
}


void delay_ms(uint16_t ms) {
  while (ms) {
    _delay_ms(1);
    ms--;
  }
}

void init(void) {
  // set the direction of the ports
  LED_DDR set(RED);
  LED_DDR set(GREEN);
  LED_DDR set(BLUE);
  
  unsigned char i, pwm;

  CLKPR = (1 << CLKPCE);        // enable clock prescaler update
  CLKPR = 0;                    // set clock to maximum (= crystal)

  pwm = PWMDEFAULT;

  // initialise all channels
  for(i=0 ; i<CHMAX ; i++) {
    compare[i] = pwm;           // set default PWM values
    compbuff[i] = pwm;          // set default PWM values
  }

  TIFR = (1 << TOV0);           // clear interrupt flag
  TIMSK = (1 << TOIE0);         // enable overflow interrupt
  TCCR0B = (1 << CS00);         // start timer, no prescale

  sei();
}


ISR (TIM0_OVF_vect) {
  static unsigned char pinlevelB=PORTB_MASK;
  static unsigned char softcount=0xFF;

  PORTB = pinlevelB;            // update outputs
  
  if(++softcount == 0){         // increment modulo 256 counter and update
                                // the compare values only when counter = 0.
    compare[0] = compbuff[0];   // verbose code for speed
    compare[1] = compbuff[1];
    compare[2] = compbuff[2];

    pinlevelB = PORTB_MASK;     // set all port pins high
  }
  // clear port pin on compare match (executed on next interrupt)
  if(compare[0] == softcount) RED_CLEAR;
  if(compare[1] == softcount) GREEN_CLEAR;
  if(compare[2] == softcount) BLUE_CLEAR;
}

