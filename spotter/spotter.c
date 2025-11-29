/**************************************************************************************************
	@file       : main.c
	@author     : Glushanenko Alexander
	@version    : r10
	@date       : 08.09.2025
	@brief      : Spotter v.2 timer firmware
	
	Changelog:
		
		r10:
		- PB7 as input of ZC detector:
		  pull up to VCC - normal default state, does not affect the output pulses;
		  pull down to GND - delays the output pulses until PB7.

**************************************************************************************************/ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

// Constants --------------------------------------------------------------------------------------

#define DELAY_DEFAULT		50
#define UNITS_DEFAULT		1	// 1 - 0.01s; 2 - 0.1s

#define OUT_SIGNAL_DDR		DDRD
#define OUT_SIGNAL_PORT		PORTD
#define OUT_SIGNAL_BIT		0

#define BTN_PLUS_DDR		DDRD
#define BTN_PLUS_PORT		PORTD
#define BTN_PLUS_PIN		PIND
#define BTN_PLUS_BIT		1

#define BTN_MINUS_DDR		DDRD
#define BTN_MINUS_PORT		PORTD
#define BTN_MINUS_PIN		PIND
#define BTN_MINUS_BIT		3

#define BTN_MODE_DDR		DDRD
#define BTN_MODE_PORT		PORTD
#define BTN_MODE_PIN		PIND
#define BTN_MODE_BIT		2

#define BTN_ACTION_DDR		DDRD
#define BTN_ACTION_PORT		PORTD
#define BTN_ACTION_PIN		PIND
#define BTN_ACTION_BIT		4

#define BTN_CYCLE_DELAY		1
#define BTN_1ST_DELAY		250		// uchar
#define BTN_2ST_DELAY		50		// uchar
#define BTN_1ST_STEPS		10		// plus/minus regulation steps before inc/dec upspeed
#define BTN_RELEASE_HOLD	BTN_CYCLE_DELAY * 250

#define LED_POLARITY		0		// 1 - Common cathode; 0 - Common anode
#define LED_COMMON_DDR		DDRD
#define LED_COMMON_PORT		PORTD
#define LED_COMMON_DIG1		6
#define LED_COMMON_DIG2		5
#define LED_DATA_DDR		DDRB
#define LED_DATA_PORT		PORTB

#define ZCROSSDETECT_DDR	DDRB
#define ZCROSSDETECT_PORT	PORTB
#define ZCROSSDETECT_PIN	PINB
#define ZCROSSDETECT_BIT	7

// 244Hz timer for dynamic renew (renew freq = 244/2 = 122 Hz for two digits)
#define LED_TIMER_PRESCALE	(1 << CS00) | (1 << CS02)
#define LED_TIMER_OFFSET	0xE0
//---------------------------------------------------------------------------

const unsigned char digit[] = {0b0111111, 0b0000110, 0b1011011, 0b1001111, 0b1100110, 0b1101101, 
								0b1111101, 0b0000111, 0b1111111, 0b1101111};

// Global variables -------------------------------------------------------------------------------

unsigned char EEMEM saved_value = DELAY_DEFAULT;
unsigned char EEMEM saved_step = UNITS_DEFAULT;
unsigned char volatile section1 = 0;
unsigned char volatile section2 = 0;
unsigned char volatile auto_mode = 1;

// Global functions -------------------------------------------------------------------------------

void saved() {
	// running light after saving
	for(unsigned char offset = 0; offset < 6; offset++){
		section1 = 1 << offset;
		_delay_ms(50);
	}
}

unsigned char ZeroCrossWait(){
	return !((ZCROSSDETECT_PIN >> ZCROSSDETECT_BIT) &1);
}

void inline out_on(){
	cli();											// r10
	while(ZeroCrossWait()) continue;				// r10
	OUT_SIGNAL_PORT |= 1 << OUT_SIGNAL_BIT;
	sei();											// r10
}

void inline out_off(){
	OUT_SIGNAL_PORT &= ~(1 << OUT_SIGNAL_BIT);
}

unsigned char ButtonPressed(unsigned short port, unsigned char bit){
	return !((port >> bit) &1);
}

void ChangeDelay(unsigned char value){
	if(value > 99) value = 99;
	
	if(value != 0){
		section1 = digit[value%10];
		section2 = digit[value/10];
	}
	else{
		section1 = digit[0];
		section2 = digit[0];
	}
}

// Interrupts -------------------------------------------------------------------------------------

// Dynamic renew
unsigned char volatile digselect = 1;
ISR(TIMER0_OVF_vect) {
	cli();
	TCNT0 = LED_TIMER_OFFSET;
	
	if(auto_mode){
		if(LED_POLARITY){
			if(digselect == 1){
				LED_COMMON_PORT |= 1 << LED_COMMON_DIG2;
				LED_COMMON_PORT &= ~(1 << LED_COMMON_DIG1);
				LED_DATA_PORT &= 0b10000000; LED_DATA_PORT |= section1;
			}
			else if(digselect == 2){
				LED_COMMON_PORT |= 1 << LED_COMMON_DIG1;
				LED_COMMON_PORT &= ~(1 << LED_COMMON_DIG2);
				LED_DATA_PORT &= 0b10000000; LED_DATA_PORT |= section2;
			}
		}
		else{
			if(digselect == 1){
				LED_COMMON_PORT &= ~(1 << LED_COMMON_DIG2);
				LED_COMMON_PORT |= 1 << LED_COMMON_DIG1;
				LED_DATA_PORT &= 0b10000000; LED_DATA_PORT |= ~section1;
			}
			else if(digselect == 2){
				LED_COMMON_PORT &= ~(1 << LED_COMMON_DIG1);
				LED_COMMON_PORT |= 1 << LED_COMMON_DIG2;
				LED_DATA_PORT &= 0b10000000; LED_DATA_PORT |= ~section2;
			}
		}
	}
	else{
		if(LED_POLARITY){
			LED_DATA_PORT &= 0b10000000; LED_DATA_PORT |= 0b01000000;
			if(digselect == 1){
				LED_COMMON_PORT |= 1 << LED_COMMON_DIG2;
				LED_COMMON_PORT &= ~(1 << LED_COMMON_DIG1);
			}
			else if(digselect == 2){
				LED_COMMON_PORT |= 1 << LED_COMMON_DIG1;
				LED_COMMON_PORT &= ~(1 << LED_COMMON_DIG2);
			}
		}
		else{
			LED_DATA_PORT &= 0b10000000; LED_DATA_PORT |= ~0b01000000;
			if(digselect == 1){
				LED_COMMON_PORT &= ~(1 << LED_COMMON_DIG2);
				LED_COMMON_PORT |= 1 << LED_COMMON_DIG1;
			}
			else if(digselect == 2){
				LED_COMMON_PORT &= ~(1 << LED_COMMON_DIG1);
				LED_COMMON_PORT |= 1 << LED_COMMON_DIG2;
			}
		}
	}
	
	if(digselect == 1) digselect = 2;
	else digselect = 1;
	
	sei();
}

// MAIN -------------------------------------------------------------------------------------------

int main(void){
	BTN_ACTION_DDR &= ~(1 << BTN_ACTION_BIT);
	BTN_MODE_DDR &= ~(1 << BTN_MODE_BIT);
	BTN_PLUS_DDR &= ~(1 << BTN_PLUS_BIT);
	BTN_MINUS_DDR &= ~(1 << BTN_MINUS_BIT);
	LED_COMMON_DDR |= (1 << LED_COMMON_DIG1) | (1 << LED_COMMON_DIG2);
	LED_DATA_DDR |= 0b01111111;
	OUT_SIGNAL_DDR |= 1 << OUT_SIGNAL_BIT;
	ZCROSSDETECT_DDR &= ~(1 << ZCROSSDETECT_BIT);
	
	// Pull up
	BTN_ACTION_PORT |= 1 << BTN_ACTION_BIT;
	BTN_MODE_PORT |= 1 << BTN_MODE_BIT;
	BTN_PLUS_PORT |= 1 << BTN_PLUS_BIT;
	BTN_MINUS_PORT |= 1 << BTN_MINUS_BIT;
	ZCROSSDETECT_PORT |= 1 << ZCROSSDETECT_BIT;
	
	// Dynamic renew timer
	TIMSK |= 1 << TOIE0 | 1 << TOIE1;
	TIFR |= 1 << TOIE0 | 1 << TOIE1;
	TCCR0B |= LED_TIMER_PRESCALE;
	TCNT0 = LED_TIMER_OFFSET;
	sei();
	
	unsigned char units = eeprom_read_byte(&saved_step);
	if(units > 2 || units < 1) units = UNITS_DEFAULT;
	
	unsigned char enter_setup = 0;
	
	// wait for 'mode' button press to enter setup menu
	unsigned char tmp_data = 0;
	unsigned char run_fire_offset = 0;
	for(unsigned char i = 0; i < 40 && !enter_setup; i++){
		_delay_ms(25);
		section1 = tmp_data;
		section2 = tmp_data;
		tmp_data = 1 << run_fire_offset;
		if(run_fire_offset < 5) run_fire_offset++;
		else run_fire_offset = 0;
		
		if(ButtonPressed(BTN_MODE_PIN, BTN_MODE_BIT))
			enter_setup = 1;
	}
	// -----------------------------------------------
	
	if(enter_setup){
		while(ButtonPressed(BTN_MODE_PIN, BTN_MODE_BIT))
			continue;
		
		_delay_ms(BTN_CYCLE_DELAY * BTN_1ST_DELAY);
		
		while(1){
			
			_delay_ms(BTN_CYCLE_DELAY);
			
			if(ButtonPressed(BTN_PLUS_PIN, BTN_PLUS_BIT) || ButtonPressed(BTN_MINUS_PIN, BTN_MINUS_BIT)){
				if(units == 1) units = 2;
				else if(units == 2) units = 1;
				
				while(ButtonPressed(BTN_PLUS_PIN, BTN_PLUS_BIT) || ButtonPressed(BTN_MINUS_PIN, BTN_MINUS_BIT))
					continue;
			}
			
			if(ButtonPressed(BTN_MODE_PIN, BTN_MODE_BIT)){
				while(ButtonPressed(BTN_MODE_PIN, BTN_MODE_BIT))
					continue;
				if(eeprom_is_ready()) eeprom_write_byte(&saved_step, units);
				eeprom_busy_wait();
				saved();
				break;
			}				

			section2 = 0b1111000; // t - time
			section1 = digit[units];
		}
	}
	else{
		// VERSION INFO
		section2 = digit[1]; section1 = digit[0];
		_delay_ms(1000);
	
		// UNITS INFO
		section2 = 0b1111000; // t - time
		section1 = digit[units];
		_delay_ms(1000);
	
		section2 = 0;
		section1 = 0;
		_delay_ms(500);
	}
	
	unsigned char units_ms = 10;
	if(units == 1) units_ms = 10;
	else if(units == 2) units_ms = 100;
	
	unsigned char spot_delay = eeprom_read_byte(&saved_value);
	if(spot_delay > 99 || spot_delay < 1) spot_delay = DELAY_DEFAULT;
	
	ChangeDelay(spot_delay);
	
	unsigned char btn_1st_delay_cnt = 0;
	unsigned char btn_2st_delay_cnt = 0;
	unsigned char btn_1st_steps_cnt = 0;
	unsigned char btn_2st_delay_mode = 0;
	
    while(1){
		_delay_ms(BTN_CYCLE_DELAY);
		
		if(ButtonPressed(BTN_ACTION_PIN, BTN_ACTION_BIT)){
			out_on();
			if(auto_mode){
				for(unsigned short i = 0; i < spot_delay * units_ms; i++)
					_delay_ms(1);
				
				out_off();
			}
			
			while(ButtonPressed(BTN_ACTION_PIN, BTN_ACTION_BIT))
				continue;
			out_off();
			
			if(eeprom_is_ready()) eeprom_write_byte(&saved_value, spot_delay);
			eeprom_busy_wait();
			
			_delay_ms(BTN_RELEASE_HOLD); // button debounce filter
			while(ButtonPressed(BTN_ACTION_PIN, BTN_ACTION_BIT)) // button release wait
				continue;
			_delay_ms(BTN_CYCLE_DELAY*10); // button debounce filter after release
		}
		
		if((btn_1st_delay_cnt == BTN_1ST_DELAY) || (btn_2st_delay_cnt == BTN_2ST_DELAY && btn_2st_delay_mode)){ // variable inc/dec speed
		
			btn_1st_delay_cnt = 0;
			btn_2st_delay_cnt = 0;
			
			if(ButtonPressed(BTN_MODE_PIN, BTN_MODE_BIT))
				auto_mode = !auto_mode;
			
			if(ButtonPressed(BTN_PLUS_PIN, BTN_PLUS_BIT) && auto_mode){
				if(btn_1st_steps_cnt == BTN_1ST_STEPS) btn_2st_delay_mode = 1;
				else btn_1st_steps_cnt++;
				if(spot_delay + 1 <= 99){
					spot_delay++;
					ChangeDelay(spot_delay);
				}
			}
		
			if(ButtonPressed(BTN_MINUS_PIN, BTN_MINUS_BIT) && auto_mode){
				if(btn_1st_steps_cnt == BTN_1ST_STEPS) btn_2st_delay_mode = 1;
				else btn_1st_steps_cnt++;
				if(spot_delay - 1 >= 1){
					spot_delay--;
					ChangeDelay(spot_delay);
				}
			}
			
			if(!ButtonPressed(BTN_PLUS_PIN, BTN_PLUS_BIT) && !ButtonPressed(BTN_MINUS_PIN, BTN_MINUS_BIT)){
				btn_1st_steps_cnt = 0;
				btn_2st_delay_mode = 0;
				btn_2st_delay_cnt = 0;
			}
		} 
		else{
			if(!btn_2st_delay_mode) btn_1st_delay_cnt++;
			else btn_2st_delay_cnt++;
		}			
    }
}