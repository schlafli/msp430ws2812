/**
 * WS2812 driver for MSP430
 *
 */

#include <msp430.h>				
#include <stdint.h>
#include "graphics.h"

//#define DUPLICATE
#ifndef DUPLICATE
//#define DOUBLE_LED
#endif


#ifdef DUPLICATE
#define RGB_LED_COUNT 72
#elif defined DOUBLE_LED
#define RGB_LED_COUNT 72
#define DUPLICATE_COUNT 1 //duplicate each LED n number of times
#else
#define RGB_LED_COUNT 10
#endif
#define __LED_COUNT (RGB_LED_COUNT * 3)

//1 amp / 20ma = 50 LEDs at max intensity
// 50 * 255 = 12750
//.5A = 6375
//1A = 12750
//2A = 25500
//3A = 38250
//4A = 51000
//5A = 63750
#define MAX_TOTAL_INTENSITY 6375

//@2.5MHz clock, 50us is ~16 bytes
//#define BLANK_BYTES 24
#define BLANK_BYTES 40

volatile uint16_t runningTotal;
volatile uint16_t bufferPos;
volatile uint8_t currentByte;

#define STATE_TX_0 0
#define STATE_TX_1 1
#define STATE_TX_2 2
#define STATE_TX_BLANK 3
#define STATE_TX_DONE 4	//not really needed, but is nice to have

#ifdef DOUBLE_LED
uint8_t duplicateState = 0;
uint8_t duplicateCounter = 0;
#endif

volatile uint8_t state;

//using packed allows us to access an array of LEDs as a single contiguous uint8_t array
typedef struct __attribute__((__packed__)){
	uint8_t g;
	uint8_t r;
	uint8_t b;
} rgb_led;

rgb_led leds[RGB_LED_COUNT];


#pragma vector=USCIAB0TX_VECTOR
__interrupt void spi_tx_complete(void){
	uint8_t tmpData;

	if(state == STATE_TX_0){
		tmpData = 0x92;
		if(currentByte & 0x80){
			tmpData |= 0x40;
		}
		if(currentByte & 0x40){
			tmpData |= 0x08;
		}
		if(currentByte & 0x20){
			tmpData |= 0x01;
		}
//		WAAAAY more overhead/cycles due to register preservation in ISR
//		txData |= ((currentByte & 0x80) >> 1) | ((currentByte & 0x40) >> 3) | ((currentByte & 0x20) >> 5);
		UCA0TXBUF = tmpData;
		state = STATE_TX_1;
	}else if(state == STATE_TX_1){
		tmpData = 0x49;
		if(currentByte & 0x10){
			tmpData |= 0x20;
		}
		if(currentByte & 0x08){
			tmpData |= 0x04;
		}
//		WAAAAY more overhead/cycles due to register preservation in ISR
//		txData |= ((currentByte & 0x10) << 1) | ((currentByte & 0x08) >> 1);
		UCA0TXBUF = tmpData;
		state = STATE_TX_2;
	}else if(state == STATE_TX_2){
		tmpData = 0x24;
		if(currentByte & 0x04){
			tmpData |= 0x80;
		}
		if(currentByte & 0x02){
			tmpData |= 0x10;
		}
		if(currentByte & 0x01){
			tmpData |= 0x02;
		}
//		WAAAAY more overhead/cycles due to register preservation in ISR
//		txData |= ((currentByte & 0x04) << 5 ) | ((currentByte & 0x02) << 3) | ((currentByte & 0x01) << 1);
		UCA0TXBUF = tmpData;
		bufferPos++;

#ifdef DUPLICATE
		if(bufferPos<(__LED_COUNT*2)){
#else
		if(bufferPos<__LED_COUNT){
#endif
			runningTotal += currentByte;
			if(runningTotal<MAX_TOTAL_INTENSITY){
#ifdef DUPLICATE
				if(bufferPos<__LED_COUNT){
					currentByte = ((uint8_t*)(&leds))[bufferPos];
				}else{
					currentByte = ((uint8_t*)(&leds))[bufferPos-__LED_COUNT];
				}
#elif defined DOUBLE_LED
				tmpData = duplicateCounter;
				tmpData++;
				if(tmpData == 3){ //LEDs per RGB led
					if(tmpData == 0){
						tmpData = DUPLICATE_COUNT;
					}else{
						bufferPos -= 3; //led count
						tmpData--;
					}
					tmpData=0;
					currentByte = ((uint8_t*)(&leds))[bufferPos];
				}
				duplicateCounter = tmpData;
#else
				currentByte = ((uint8_t*)(&leds))[bufferPos];
#endif
			}else{
				currentByte = 0;
			}
			state = STATE_TX_0;
		}else{
			state = STATE_TX_BLANK;
			bufferPos = BLANK_BYTES;
		}
	}else if(state == STATE_TX_BLANK){
		UCA0TXBUF = 0;
		bufferPos--;
		if(bufferPos==0){
			state = STATE_TX_DONE;
		}
	}else{
		IE2 &= ~UCA0TXIE;
	}
	__bic_SR_register_on_exit( CPUOFF );
}

void initSPI(){
	//Set data txt pin:
	P1SEL |= BIT2;
	P1SEL2 |= BIT2;

	UCA0CTL1 = UCSWRST;

	//Set master and MSB
	UCA0CTL0 = UCMSB + UCMST + UCSYNC;

	//set clock source to SMCLK
	UCA0CTL1 = UCSSEL_2 | UCSWRST;

	const uint16_t prescale = 8;

	UCA0BR0 = 0xff & prescale;
	UCA0BR1 = 0xff & (prescale>>8);

	//enable SPI
	UCA0CTL1 &= ~(UCSWRST);

	//enable interrupt
	__enable_interrupt();
}

void setLEDData(uint8_t * buffer, uint8_t data){

	//unrolled:
	/*
	 * byte 0 = 1 _ 0 1 _ 0 1 _
	 * byte 1 = 0 1 _ 0 1 _ 0 1
	 * byte 2 = _ 0 1 _ 0 1 _ 0
	 *
	 * set to black:
	 *  0x92
	 *  0x49
	 *  0x24
	 */
	//MSB first
	buffer[0] = 0x92 | ((data & 0x80) >> 1) | ((data & 0x40) >> 3) | ((data & 0x20) >> 5);
	buffer[1] = 0x49 | ((data & 0x10) << 1) | ((data & 0x08) >> 1);
	buffer[2] = 0x24 | ((data & 0x04) << 5 )| ((data & 0x02) << 3) | ((data & 0x01) << 1);

	//Actually, conditional checks + bit set might be more efficient
	//Or just hand written assembly

}

void updateStrip(){

	//busy loop till tx done
	while(UCA0STAT & UCBUSY){
		__low_power_mode_0(); //disable CPU and enable interrupts
	}
	//begin tx
	bufferPos = 0;
	state = STATE_TX_0;
	currentByte = ((uint8_t*)(&leds))[0];
	runningTotal = 0;
#ifdef DOUBLE_LED
	duplicateState = DUPLICATE_COUNT;
	duplicateCounter = 0;
#endif

	UCA0TXBUF = 0;
	IE2 |= UCA0TXIE;
}

void init(){
	WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer

	//set frequency to 16MHz
//	BCSCTL1 = CALBC1_16MHZ;
//	DCOCTL = CALDCO_16MHZ;

	DCOCTL = DCO2 ;// + MOD0 + MOD1 + MOD2 + MOD3 + MOD4;
	BCSCTL1 = XT2OFF + 15;

}


/*
 * We need a 2.5MHz data rate, giving 0.4 us per bit
 *
 * A low is 1 high, 2 low
 *  _
 * | |__|
 *
 * A high is 2 high 1 low
 *  __
 * |  |_|
 *
 *
 * Required dedicated wam:
 *
 * 10 LEDs * RGB * 3 bits/bit packing = 90 bytes
 *
 */



void hsvtorgb(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t h, uint8_t s, uint8_t v)
{
	uint8_t region, fpart, p, q, t;

    if(s == 0) {
        /* color is grayscale */
        *r = v;
        *g = v;
        *b = v;
        return;
    }

    /* make hue 0-5 */
    region = h / 43;
    /* find remainder part, make it from 0-255 */
    fpart = (h - (region * 43)) * 6;

    /* calculate temp vars, doing integer multiplication */

    //TODO: rewrite to use intermediate vars. (should fix issue)
    uint16_t tmp = v * (255 - s);
    //p = (v * (255 - s)) >> 8;
    p = 0xff & (tmp >> 8);

//    q = (v * (255 - ((s * fpart) >> 8))) >> 8;
    tmp = s * fpart;
    tmp = 0xff & (tmp >> 8);
	tmp = (255 - tmp) * v;
	q = 0xff & (tmp >> 8);

//	t = (v * (255 - ((s * (255 - fpart)) >> 8))) >> 8;
	tmp = s * (255 - fpart);
	tmp = 0xff & (tmp>>8);
	tmp = (255 - tmp) * v;
	t = 0xff & (tmp >> 8);

    /* assign temp vars based on color cone region */
    switch(region) {
        case 0:
            *r = v; *g = t; *b = p; break;
        case 1:
            *r = q; *g = v; *b = p; break;
        case 2:
            *r = p; *g = v; *b = t; break;
        case 3:
            *r = p; *g = q; *b = v; break;
        case 4:
            *r = t; *g = p; *b = v; break;
        default:
            *r = v; *g = p; *b = q; break;
    }

    return;
}

void setLEDhsv(rgb_led * led, uint8_t h, uint8_t s, uint8_t v){
	hsvtorgb(&(led->r),&(led->g),&(led->b), h, s, v);
}

void setLEDrgb(rgb_led * led, uint8_t r, uint8_t g, uint8_t b){
	led->r = r;
	led->g = g;
	led->b = b;
}

const uint8_t skipAmmount = 10;
uint16_t blinkXmas(uint16_t position){
	uint8_t startPos = 0;
	if(position & 0x1){
		startPos = skipAmmount/2;
	}

	uint16_t i,j;
	for(i=0;i<startPos;i++){
		setLEDrgb(&leds[i], 0, 0, 0);
	}
	for(i=startPos;i<RGB_LED_COUNT;i+=skipAmmount){
		setLEDhsv(&leds[i], 20, 220, 200);
		for(j=i+1;j<i+skipAmmount && j<RGB_LED_COUNT;j++){
			setLEDrgb(&leds[j], 0, 0, 0);
		}
	}
	return 2000;
}

void chaseColourAll(uint16_t pos, uint8_t clear, uint8_t skipLeds){
	uint8_t startPos = pos % skipLeds;
	uint16_t i,j;
	for(i=0;i<startPos;i++){
		if(clear){
			setLEDrgb(&leds[i], 0, 0, 0);
		}
	}
	for(i=startPos;i<RGB_LED_COUNT;i+=skipLeds){
		setLEDhsv(&leds[i], pos+i+i+i, 255, 100);
		for(j=i+1;j<i+skipLeds && j<RGB_LED_COUNT;j++){
			if(clear){
				setLEDrgb(&leds[j], 0, 0, 0);
			}
		}
	}
}

uint16_t chaseColour(uint16_t pos){
	chaseColourAll(pos, 1, 10);
	return 25;
}

uint16_t chaseColour2(uint16_t pos){
	chaseColourAll(pos, 0, 45);
	return 50;
}

uint16_t rainbow(uint16_t position){
	uint16_t i;
	for(i=0;i<RGB_LED_COUNT;i++){
		setLEDhsv(&(leds[i]), position+i+i+i+i, 255, 70);
	}
	return 2;
}

uint16_t rainbow2(uint16_t position){
	uint16_t i;
	for(i=0;i<RGB_LED_COUNT;i++){
		setLEDhsv(&(leds[i]), position+i, 255, 70);
	}
	return 2;
}

void setLEDFirePalette(uint8_t led, uint8_t value){
	value = value & 0x1f;
	uint8_t red=0;
	uint8_t green=0;
	if(value<7){
		//calc: 100/6 = 16.6 rounded 16 (96/6)
		red = 16*value;
		green = red;
	}else if(value < 13){
		uint8_t val = value-6;
		//red: 60/6 * i + 96 (96->156)
		//green: 30/6 * (6-i) + 66 (96->66)
		red = 100 + 10 * val;
		green = 70 + (6-val) * 5;
	}else if(value < 19){
		uint8_t val = value-12;
		//red: 42/6 * val + 156 (156->198)
		//green: 66/6 * (6-val)
		red = 7 * val + 156;
		green = 11 * (6-val);
	}else if(value < 25){
		uint8_t val = value - 18;
		//red: 156/6 * (6-val) +42 (198 -> 42)
		red = 26 * (6-val) + 42;
	}else{
		uint8_t val = value - 24;
		//red = 42/7 * (6-val);
		red = 6*(7-val);
	}
	setLEDrgb(&(leds[led]), (red>>1)+100, (green>>1), 0);
}

uint16_t hyberg(uint16_t position){
	uint8_t i;
	for(i=0;i<RGB_LED_COUNT;i++){
		if(i&0x01){
			uint8_t green = t_sin(16+position + (i<<1)); //value between 0 and 31
			setLEDrgb(&(leds[i]), 0, (green<<1) + 40, 30);
		}else{
			uint16_t value = t_sin(i * 3 + position);
			value +=  t_sin( i * t_sin(position/3) + position);
			setLEDFirePalette(i, value);
		}
	}
	return 100;
}

uint16_t hyberg2(uint16_t position){
	uint8_t i;
	for(i=0;i<RGB_LED_COUNT;i++){
		if(i&0x01){
			uint8_t green = t_sin(16+position + (i<<1)); //value between 0 and 31
			setLEDrgb(&(leds[i]), 0, (green<<1) + 40, 30);
		}else{
			uint16_t value = t_sin(i * 3 + position);
			value +=  t_sin( i * t_sin(position/3) + position);
			setLEDFirePalette(i, value>>1);
		}
	}
	return 100;
}

uint16_t hyberg3(uint16_t position){
	uint8_t i;
	for(i=0;i<RGB_LED_COUNT;i++){
		if(i&0x01){
			uint8_t green = t_cos(position/3 + (i<<1)); //value between 0 and 31
			setLEDrgb(&(leds[i]), 0, (green<<1) + 40, 30);
		}else{
			uint16_t value = t_sin(i * 3 + position);
			value += t_sin(i + position);
			value +=  t_sin( i * t_sin(position/3) + position);
			value +=  t_sin( i * t_cos(position/7) + position);
			setLEDFirePalette(i, value>>2);
		}
	}
	return 100;
}


uint16_t hyberg_static(uint16_t position){
	uint8_t i;
	for(i=0;i<RGB_LED_COUNT;i++){
		if(i&0x01){
			setLEDhsv(&(leds[i]), 100, 255, 120);
		}else{
			setLEDhsv(&(leds[i]), 255, 255, 255);
		}
	}
	return 1000;
}

uint16_t hyberg_hsv_wave(uint16_t position){
	uint8_t i;
	for(i=0;i<RGB_LED_COUNT;i++){
		if(i&0x01){
			setLEDhsv(&(leds[i]), 128 + position, 255, 160);
		}else{
			setLEDhsv(&(leds[i]), position, 255, 255);
		}
	}
	return 50;
}

uint16_t hyberg_toggle(uint16_t position){
	uint8_t i;
	for(i=0;i<RGB_LED_COUNT;i++){
		if(i&0x01){
			if(position & 0x1){
				setLEDhsv(&(leds[i]), 100, 255, 120);
			}else{
				setLEDhsv(&(leds[i]), 255, 255, 255);
			}
		}else{
			if(position & 0x1){
				setLEDhsv(&(leds[i]), 255, 255, 255);
			}else{
				setLEDhsv(&(leds[i]), 100, 255, 120);
			}
		}
	}
	return 2000;
}


uint16_t flashRGB(uint16_t position){
	uint16_t i;
	uint8_t r, g, b;

	if((position & 0x07)== 0x07){
		hsvtorgb(&r, &g, &b, position, 255, 50);
	}else{
		r = b = g = 0;
	}
	for(i=0;i<RGB_LED_COUNT;i++){
		leds[i].r = r;
		leds[i].g = g;
		leds[i].b = b;
	}
	return 10;
}

uint16_t ascending(uint16_t position){
	uint16_t i;
	for(i=0;i<RGB_LED_COUNT;i++){
		setLEDrgb(&(leds[i]), 0, 0, 0);
	}
	setLEDrgb(&(leds[ position % RGB_LED_COUNT]), 255, 255, 255);
	return 2;
}

uint16_t ascending2(uint16_t position){
	setLEDhsv(&(leds[ position % RGB_LED_COUNT]), position, 255, 100);
	return 1;
}



#define DISPLAY_PATTERN_COUNT 10
uint16_t (*dispFunc[DISPLAY_PATTERN_COUNT])(uint16_t) = {
	hyberg2,
	hyberg3,
	hyberg,
	hyberg_static,
	hyberg_hsv_wave,
	hyberg_toggle,
	rainbow,
	rainbow2,
	ascending,
	ascending2,
	chaseColour,
	chaseColour2,
	flashRGB,
	blinkXmas
};


void delay0_5ms(volatile uint16_t ms){
	do{
		ms--;
		__delay_cycles(8000);

	}
	while(ms != 0);
}

uint8_t buttonState = 0;

int main(void) {
	init();
	initSPI();
	P1DIR |= 0x01;					// Set P1.0 to output direction
	P1REN |= 0x08;
	P1DIR &= ~(0x08);
	P1OUT |= 0x08; //pullup

	//250ms delay to avoid button trigger on reset
	delay0_5ms(500);

	uint16_t offset = 0;
	uint8_t currentFunc = 0;
	for(;;) {
		uint16_t delayAmount;		// volatile to prevent optimization

		P1OUT ^= 0x01;				// Toggle P1.0 using exclusive-OR
		delayAmount = dispFunc[currentFunc](offset);
		updateStrip();
		offset++;
		while(delayAmount){
			__delay_cycles(7000);
			if( (P1IN & 0x08) == 0 ){
				if(buttonState == 0){
					buttonState = 1;
					delay0_5ms(50); //25ms for debounce
					currentFunc = (currentFunc+1) % DISPLAY_PATTERN_COUNT;
					offset=0;
					break;
				}
			}else{
				if(buttonState == 1){
					buttonState = 0;
					delay0_5ms(50); //25ms for debounce
				}
			}
			delayAmount--;
		}
	}
	
	return 0;
}


