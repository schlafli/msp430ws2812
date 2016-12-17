//***************************************************************************************
//  MSP430 Blink the LED Demo - Software Toggle P1.0
//
//  Description; Toggle P1.0 by xor'ing P1.0 inside of a software loop.
//  ACLK = n/a, MCLK = SMCLK = default DCO
//
//                MSP430x5xx
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |             P1.0|-->LED
//
//  J. Stevenson
//  Texas Instruments, Inc
//  July 2011
//  Built with Code Composer Studio v5
//***************************************************************************************

#include <msp430.h>				
#include <stdint.h>

#define RGB_LED_COUNT 3
#define LED_COUNT (RGB_LED_COUNT * 3)
#define DATA_BUFFER_SIZE (LED_COUNT * 3)

//@2.5MHz clock, 50us is ~16 bytes
#define BLANK_BYTES 16
#define BLANK_END (DATA_BUFFER_SIZE + 16)

uint8_t data[DATA_BUFFER_SIZE];
volatile uint16_t bufferPos = 0;

typedef struct{
	uint8_t r;
	uint8_t g;
	uint8_t b;
} rgb_led;

rgb_led leds[RGB_LED_COUNT];

#pragma vector=USCIAB0TX_VECTOR
__interrupt void spi_tx_complete(void){
	if(bufferPos<DATA_BUFFER_SIZE){
		UCA0TXBUF = data[bufferPos];
		bufferPos++;
	}else if(bufferPos<BLANK_END){
		UCA0TXBUF = 0;
		bufferPos++;
	}else{
		IE2 &= ~UCA0TXIE;
		bufferPos = 0;
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

//	uint32_t result = 0;
//	for (bitNumber = 0; bitNumber < 8; bitNumber++) {
//		if (i & (1 << bitNumber)) {
//			result |= 6 << (bitNumber * 3);
//		} else {
//			result |= 4 << (bitNumber * 3);
//		}
//	}
//	buffer[2] = 0xff & (result >> 16);
//	buffer[1] = 0xff & (result >> 8);
//	buffer[0] = 0xff & (result);

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

void beginTx(){
	while(UCA0STAT & UCBUSY){
		__low_power_mode_0(); //disable CPU and enable interrupts
	}
	uint8_t i;
	//	UCB0TXBUF
	for(i=0;i<RGB_LED_COUNT;i++){
		uint16_t pos = i*9;

		//GRB data layout on WS2812B
		setLEDData(&(data[pos]), leds[i].g);
		setLEDData(&(data[pos+3]), leds[i].r);
		setLEDData(&(data[pos+6]), leds[i].b);
	}

	//begin tx
	bufferPos = 1;
	UCA0TXBUF = data[0];
	IE2 |= UCA0TXIE;
}

void init(){
	WDTCTL = WDTPW | WDTHOLD;		// Stop watchdog timer

	//set frequency to 16MHz
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;

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


int main(void) {
	init();
	initSPI();

	leds[0].r = 255;
	leds[0].g = 0;
	leds[0].b = 0;

	leds[1].r = 0;
	leds[1].g = 255;
	leds[1].b = 0;

	leds[2].r = 0;
	leds[2].g = 0;
	leds[2].b = 255;


	P1DIR |= 0x01;					// Set P1.0 to output direction

	for(;;) {
		volatile unsigned int i;	// volatile to prevent optimization

		P1OUT ^= 0x01;				// Toggle P1.0 using exclusive-OR
		beginTx();

		i = 50000;					// SW Delay
		do i--;
		while(i != 0);
	}
	
	return 0;
}
