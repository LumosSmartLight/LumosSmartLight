#include "asf.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */


/**
* LEDs
*/

#define LED3_PIO_ID	   ID_PIOA
#define LED3_PIO        PIOA
#define LED3_PIN		   4
#define LED3_PIN_MASK   (1<<LED3_PIN)

#define NUM_LEDS 24

#define ERASE 0x00000000
#define BLUE 0x00FF0000
#define GREEN 0x000000FF
#define RED 0x0000FF00

volatile uint32_t buffer[24];


void LED_init(int estado){
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIN_MASK, estado, 0, 0 );
};

void send_0(){
	int i = 0;
	for(i=0; i<26; i++)	
	LED3_PIO->PIO_SODR = LED3_PIN_MASK;
	
	for(i=0; i<59; i++) 
	LED3_PIO->PIO_CODR = LED3_PIN_MASK;
	
}

void send_1(){
	int i = 0;
	for(i=0; i<52; i++) 
	LED3_PIO->PIO_SODR = LED3_PIN_MASK;

	for(i=0; i<44; i++) 
	LED3_PIO->PIO_CODR = LED3_PIN_MASK;
	
}

void reveal(){
	LED3_PIO->PIO_CODR = LED3_PIN_MASK;
	delay_us(900);
}


void send_pixel(uint32_t p){
	for (int i = 0; i < NUM_LEDS; i++){
		if(p >> i & 0x00000001){
			send_1();
		}
		else{
			send_0();
		}
	}	
}

void reset(){
	for(int i = 0; i < NUM_LEDS; i++){
		send_pixel(ERASE);
	}
	reveal();
}

void initialize_buffer(){
	for(int i = 0; i < NUM_LEDS; i++){
		buffer[i] = ERASE;
	}
}

void send_buffer(){
	reset();
	for(int i = 0; i < NUM_LEDS; i++){
		send_pixel(buffer[i]);
	}
	reveal();
}

void frenetic(){
	uint32_t buffer_copy[NUM_LEDS];
	for(int i = 0; i < NUM_LEDS; i++){
		buffer_copy[(i+1)%NUM_LEDS] = buffer[i];
	}
	for(int j = 0; j < NUM_LEDS; j++){
		buffer[j] = buffer_copy[j];
	}
}

uint32_t rgb (int r, int g, int b){
	uint32_t result = 0;
	result = b << 16 | r << 8 | g << 0;
	return result;
	
}




/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

	/* Configura Leds */
	LED_init(0);
	LED3_PIO->PIO_CODR = LED3_PIN_MASK;
	initialize_buffer();
	buffer[10] = rgb(0,0,255);
	buffer[12] = rgb(0,255,0);
	buffer[14] = rgb(255,0,0);
	send_buffer();
	//uint32_t top = rgb(255,0,0);
	
	
	while (1) {
		
		

		
		
		//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		/* Entrar em modo sleep */

	}

}
