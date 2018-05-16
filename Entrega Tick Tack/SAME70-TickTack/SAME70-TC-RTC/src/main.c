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

#define ERASE 0x00000000
#define GREEN 0x00110000
#define BLUE 0x00000011
#define RED 0x00001100

#define INTERVAL 500

volatile uint32_t buffer[24];



void LED_init(int estado){
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIN_MASK, estado, 0, 0 );
};

void send_0(){
	for (int i = 0; i<26; i++);
	LED3_PIO->PIO_SODR = LED3_PIN_MASK;
	
	for(int j = 0; j<59; j++);
	LED3_PIO->PIO_CODR = LED3_PIN_MASK;
	
}

void send_1(){
	for (int i = 0; i<52; i++);
	LED3_PIO->PIO_SODR = LED3_PIN_MASK;
	
	for (int i = 0; i<44; i++);
	LED3_PIO->PIO_CODR = LED3_PIN_MASK;
	
}

void reveal(){
	LED3_PIO->PIO_CODR = LED3_PIN_MASK;
	delay_us(900);
}


void sendB(uint32_t p){
	for (int i = 23; i >= 0; i--){
		if(p >> i & 0x00000001){
			//if(i > 8){
				send_1();
			//}
			
		}
		else{
			//if(i > 8){
				send_0();
			//}
		}
		
	}
	
}

void refresh(){
	for(int i = 0; i <= 23; i++){
		sendB(ERASE);
	}
	reveal();
}

void initialize_buffer(){
	for(int i = 23; i > 0; i++){
		buffer[i] = ERASE;
	}
}

void send_buffer(){
	buffer[11] = RED;
	for(int i = 23; i > 0; i++){
		sendB(buffer[i]);
	}
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
	refresh();
	sendB(BLUE);
	reveal();
	while (1) {
		
		//pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		/* Entrar em modo sleep */

	}

}
