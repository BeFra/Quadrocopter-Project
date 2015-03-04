/*
 * time.c
 *
 * Created: 03.03.2015 21:10:24
 *  Author: befrank
 */ 
#include <avr/interrupt.h>

#define F_CPU 16000000

#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )

/* the prescaler is set so that timer0 ticks every 64 clock cycles, and the
/ the overflow handler is called every 256 ticks.*/
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

/* the whole number of milliseconds per timer0 overflow */
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

/* the fractional number of milliseconds per timer0 overflow. we shift right
/ by three to fit these numbers into a byte. (for the clock speeds we care
/ about - 8 and 16 MHz - this doesn't lose precision.) */
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)





volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

void init_timer() {
	sei();
	/* timer normal mode */
	TCCR0A = ( 0 << WGM00) | ( 0 << WGM01);
	/* set prescaler to 64 */
	TCCR0B = ( 1 << CS00) | ( 1 << CS01);
	/* overflow interrupt enable */
	TIMSK0 = ( 1 << TOIE0);
}

unsigned long get_millis() {
	
	unsigned long millis;
	/* save global interrupt flag */
	uint8_t oldSREG = SREG;

	/* disable interrupts while we read timer0_millis or we might get an
	 inconsistent value (e.g. in the middle of a write to timer0_millis) */
	cli();
	millis = timer0_millis;
	
	/* restore global interrupt flag */
	SREG = oldSREG;

	return millis;
}


ISR(TIMER0_OVF_vect) {
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}