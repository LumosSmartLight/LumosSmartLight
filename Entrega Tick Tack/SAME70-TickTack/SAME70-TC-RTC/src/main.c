#include "asf.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
#define YEAR        2018
#define MOUNTH      3
#define DAY         19
#define WEEK        12
#define HOUR        15
#define MINUTE      45
#define SECOND      0

/**
* LEDs
*/
#define LED_PIO_ID	   ID_PIOC
#define LED_PIO        PIOC
#define LED_PIN		   8
#define LED_PIN_MASK   (1<<LED_PIN)

#define LED1_PIO_ID	   ID_PIOA
#define LED1_PIO        PIOA
#define LED1_PIN		   0
#define LED1_PIN_MASK   (1<<LED1_PIN)

#define LED2_PIO_ID	   ID_PIOC
#define LED2_PIO        PIOC
#define LED2_PIN		   30
#define LED2_PIN_MASK   (1<<LED2_PIN)

#define LED3_PIO_ID	   ID_PIOB
#define LED3_PIO        PIOB
#define LED3_PIN		   2
#define LED3_PIN_MASK   (1<<LED3_PIN)

/**
* Boto
*/
#define BUT_PIO_ID			  ID_PIOA
#define BUT_PIO				  PIOA
#define BUT_PIN				  11
#define BUT_PIN_MASK			  (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

#define BUT1_PIO_ID			  ID_PIOD
#define BUT1_PIO				  PIOD
#define BUT1_PIN				  28
#define BUT1_PIN_MASK			  (1 << BUT1_PIN)
#define BUT1_DEBOUNCING_VALUE  79

#define BUT2_PIO_ID			  ID_PIOC
#define BUT2_PIO				  PIOC
#define BUT2_PIN				  31
#define BUT2_PIN_MASK			  (1 << BUT2_PIN)
#define BUT2_DEBOUNCING_VALUE  79

#define BUT3_PIO_ID			  ID_PIOA
#define BUT3_PIO				  PIOA
#define BUT3_PIN				  19
#define BUT3_PIN_MASK			  (1 << BUT3_PIN)
#define BUT3_DEBOUNCING_VALUE  79
/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

volatile uint8_t flag_led0 = 1;
volatile uint8_t flag_alarm = 1;

volatile uint8_t flag_led1 = 1;
volatile uint8_t flag_b1 = 1;

volatile uint8_t flag_led2 = 1;
volatile uint8_t flag_b2 = 1;

volatile uint8_t flag_led3 = 1;
volatile uint8_t flag_b3 = 1;


/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
*  Handle Interrupcao botao 1
*/
static void Button1_Handler(uint32_t id, uint32_t mask){
	flag_b1 = !flag_b1;
}

static void Button2_Handler(uint32_t id, uint32_t mask){
	flag_b2 = !flag_b2;
}

static void Button3_Handler(uint32_t id, uint32_t mask){
	flag_b3 = !flag_b3;
}


void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupo foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */git 
	if(flag_b1){
		if(flag_led1){
			pin_toggle(LED1_PIO, LED1_PIN_MASK);
			
		}
	}
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupo foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	if(flag_b2){
		if(flag_led2){
			pin_toggle(LED2_PIO, LED2_PIN_MASK);
		}
	}
}

void TC2_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupo foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	if(flag_b3){
		if(flag_led3){
			pin_toggle(LED3_PIO, LED3_PIN_MASK);
		}
	}
}

/**
* \brief Interrupt handler for the RTC. Refresh the display.
*/
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		
	}
	
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
			uint32_t hour, second, minute;
			rtc_get_time(RTC, &hour, &minute, &second);
			if (flag_alarm){
				flag_alarm = 0;
				flag_led0 = 0;
				pmc_disable_periph_clk(ID_TC1);
			}
			else{
				flag_alarm = 1;
				flag_led0 = 1;
				pmc_enable_periph_clk(ID_TC1);
			}
			if(minute == 59){
				minute = -1;
				hour+=1;
			}
			else{
				rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
				rtc_set_time_alarm(RTC, 1, hour, 1, minute+1, second, 1);
			}
	}
	
	
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/**
*  Toggle pin controlado pelo PIO (out)
*/
void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

/**
* @Brief Inicializa o pino do BUT
*/
void BUT_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pio_set_input(BUT1_PIO, BUT1_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupo */
	pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);// INTERRUPCAO
	pio_handler_set(BUT1_PIO, BUT1_PIO_ID, BUT1_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);

	/* habilita interrupco do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 1);
	
	//BOTAO 2
	
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_set_input(BUT2_PIO, BUT2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupo */
	pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK);// INTERRUPCAO
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIN_MASK, PIO_IT_FALL_EDGE, Button2_Handler);

	/* habilita interrupco do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 1);
	
	//BOTAO 3
	
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pio_set_input(BUT3_PIO, BUT3_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupo */
	pio_enable_interrupt(BUT3_PIO, BUT3_PIN_MASK);// INTERRUPCAO
	pio_handler_set(BUT3_PIO, BUT3_PIO_ID, BUT3_PIN_MASK, PIO_IT_FALL_EDGE, Button3_Handler);

	/* habilita interrupco do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 1);
};

/**
* @Brief Inicializa o pino do LED
*/
void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
	
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_PIN_MASK, estado, 0, 0 );
	
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_PIN_MASK, estado, 0, 0 );
	
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_PIN_MASK, estado, 0, 0 );
};

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter  meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrupco no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupco no TC canal 0 */
	/* Interrupo no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS); //INTERRUPCAO

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN); //INTERRUPCAO	

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

	/* Configura os botes */
	BUT_init();

	/** Configura RTC */
	RTC_init();

	/* configura alarme do RTC */
	rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
	rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE+1, 1, SECOND);
	
	/** Configura timer TC0, canal 1 */
	TC_init(TC0, ID_TC0, 0, 2);
	TC_init(TC0, ID_TC1, 1, 4);
	TC_init(TC0, ID_TC2, 2, 8);



	while (1) {
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
		/* Entrar em modo sleep */

	}

}
