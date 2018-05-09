# 11 - TICK ! TACK !

Periféricos envolvidos :

 - Power Managment Controller (PMC)
 - Parallel Input/Output Controller (PIO)
 - Real Time Clock (RTC)
 - Timer Counter (TC)

## Descritivo

Piscar os leds a uma frequência pre determinada utilizando a interrupção gerada (TCx_Handler) pelo TC, um alarme é configurado no RTC
para gerar uma interrupção (RTC_Handler) após um minuto de execução do código, interrompendo o piscar do led.


### Visão geral do código

```C

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
#define YEAR        2017
#define MOUNTH      3
#define DAY         27
#define WEEK        13
#define HOUR        9
#define MINUTE      5
#define SECOND      0

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		    8
#define LED_PIN_MASK    (1<<LED_PIN)

/**
 * Botão
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
volatile uint8_t flag_led0 = 1;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
void TC1_init(void);
void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
 *  Handle Interrupcao botao 1
 */
static void Button1_Handler(uint32_t id, uint32_t mask){...};

/**
 *  Interrupt handler for TC1 interrupt. 
 */
void TC1_Handler(void){...};

/**
 * \brief Interrupt handler for the RTC. Refresh the display.
 */
void RTC_Handler(void){...};

/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/** 
 *  Toggle pin controlado pelo PIO (out)
 */
void pin_toggle(Pio *pio, uint32_t mask){...};

/**
 * @Brief Inicializa o pino do BUT
 */
void BUT_init(void){...};
  
/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){...};

/**
 * Configura TimerCounter (TC0) para gerar uma interrupcao no canal 0-(ID_TC1) 
 * a cada 250 ms (4Hz)
 */
void TC1_init(void){...};
   
/**
 * Configura o RTC para funcionar com interrupcao de alarme
 */
void RTC_init(){...};
  
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

  /* Configura os botões */
  BUT_init();    
    
  /** Configura timer 0 */
  TC1_init();
  
  /** Configura RTC */
  RTC_init();

  /* configura alarme do RTC */    
  rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
  rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE+1, 1, SECOND);
           
  while (1) {
     	
  }
}
```


### Timer Counter - TC

O TC é um periférico de contagem de pulsos, esses pulsos podem ser provenientes do mundo externo (abrir e fechar de portas, encoder de um motor, pulsos de um PWM) ou interno ao próprio microcontrolador, utilizando o clock como sinal de entrada.
Esse contador é formado por 16 bits, limitando o valor máximo que pode contar em : 2^16 -1 = 65535.

![Timer Counter](https://github.com/Insper/Computacao-Embarcada/blob/master/Codigos%20Aula/11-TC-RTC/doc/cnt.png){ width=50% }

No modo de contagem de clocks, o TC operar como um cronometro utilizando o clock do microcontrolador como sua base de tempo, ou seja, a cada pulso de clock o valor do contador é incrementado de um. 

Se usarmos o clock base do sistema (300 Mhz) na entrada do TC, ou seja, 300 milhões de pulsos em um segundo, poderíamos contar até
no máximo 218us (x = 65535/300M). Para possibilitar uma maior flexibilidade ao TC, podemos optar por divir o clock de entrada (300Mhz)
em alguns prescales : 1/1, 1/8, 1/32, 1/128. Possibilitando que o TC conte por períodos mais longos.

A cada vez que o contador é reiniciado (quando atinge o seu valor máximo) uma interrupção (TCx_Handler) é gerada:

![Timer Counter](https://github.com/Insper/Computacao-Embarcada/blob/master/Codigos%20Aula/11-TC-RTC/doc/cntIRQ.png){ width=50% }

Cada canal possui um registrador chamado de RC que possibilita o reset do contador em um valor diferente que o máximo. Esse registrador
é utilizado para gerar a frequência desejada.

![Timer Counter](https://github.com/Insper/Computacao-Embarcada/blob/master/Codigos%20Aula/11-TC-RTC/doc//cntRC.png){ width=50% }

Cada TC possui três canais (canal 0, canal 1, canal 2), podendo cada canal operar de maneira independente, porém com a mesma
configuração de prescale, conforme ilustração a seguir :

![Timer Counter](https://github.com/Insper/Computacao-Embarcada/blob/master/Codigos%20Aula/11-TC-RTC/doc/tc.png){ width=50% }

A função a seguir configura o TC0, canal 1 para operar com uma frequência de 4Hz :

```C
/**
 * Configura TimerCounter (TC0) para gerar uma interrupcao no canal 0-(ID_TC1) 
 * a cada 250 ms (4Hz)
 */
void TC1_init(void){ 
  uint32_t ul_div;
  uint32_t ul_tcclks;
  uint32_t ul_sysclk = sysclk_get_cpu_hz();
  
  uint32_t channel = 1;
  
  /* Configura o PMC */
  pmc_enable_periph_clk(ID_TC1);    
  
  /** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
  tc_find_mck_divisor(4, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
  tc_init(TC0, channel, ul_tcclks | TC_CMR_CPCTRG);
  tc_write_rc(TC0, channel, (ul_sysclk / ul_div) / 4);
  
  /* Configura e ativa interrupçcão no TC canal 0 */
  NVIC_EnableIRQ((IRQn_Type) ID_TC1);
  tc_enable_interrupt(TC0, channel, TC_IER_CPCS);
  
  /* Inicializa o canal 0 do TC */
  tc_start(TC0, channel);
}
```

#### Interrupção

Sempre que houver um reset no contador do TC, a interrupção referente ao canal é chamada, nessa interrupção fazemos a mudança no
status do led.

```C
/**
 *  Interrupt handler for TC1 interrupt. 
 */
  void TC1_Handler(void){
  volatile uint32_t ul_dummy;
  
  /****************************************************************
  * Devemos indicar ao TC que a interrupção foi satisfeita.
  ******************************************************************/
  ul_dummy = tc_get_status(TC0, 1);
  
  /* Avoid compiler warning */
  UNUSED(ul_dummy);
  
  /** Muda o estado do LED */
  if(flag_led0)
      pin_toggle(LED_PIO, LED_PIN_MASK);
}
```






### Real Time Clock - RTC

O RTC é um periférico do uC que serve para contar tempo com resolução de segundos, ele possui toda a lógica interna de um relógio, contando anos, dias, meses, horas, minutos e segundos. 

O RTC é configurado na função RTC_init() :

```C
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
    rtc_enable_interrupt(RTC,  RTC_IER_ALREN);   
}
```
 
#### Alarme

O alarme é uma maneira de configurarmos o RTC para gerar uma interrupção em uma determinada data, no nosso caso, para um minuto
apos a inicialização do micrcontrolador :

```C
  main(){
  ...
  ...
  
  /* configura alarme do RTC */    
  rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
  rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE+1, 1, SECOND);
  
  }
```

#### Interrupção

Sempre que a situação do alarme for satisfeita, o RTC_Handler será chamado. Na interrupção configura a variável global flag_led0 para 0.

```C
/**
 * \brief Interrupt handler for the RTC. 
 */
void RTC_Handler(void)
{
  /* Qual parte do RTC gerou interrupção ? */
  uint32_t ul_status = rtc_get_status(RTC);
  
  /* seta led para parar de piscar */
  flag_led0 = 0;
  
  /* Informa que interrupção foi tratada */
  rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
}
```


