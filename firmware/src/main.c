/************************************************************************
* 5 semestre - Eng. da Computao - Insper
*
* 2021 - Exemplo com HC05 com RTOS

Conectar :
*            MCU  |  SAME70-XPLD |
*           ----------------------
*            SDA  |   EXT2-11    |
*            SCL  |   EXT2-12    |
*            GND  |   EXT2-19    |
*            VCC  |   EXT2-20    |
*
*/

#include <asf.h>
#include "conf_board.h"
#include <string.h>
#include "mcu6050.h"
#include "fusion/fusion.h"
#include <math.h>





/************************************************************************/
/* defines                                                              */
/************************************************************************/

// LEDs
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// Configurações do Led OLED e da placa

#define LED1_PIO			PIOA
#define LED2_PIO			PIOC
#define LED3_PIO			PIOB

// Configurações do LED de feedback

#define FEEDBACK PIOD
#define FEEDBACK_ID ID_PIOD
#define FEEDBACK_IDX 21
#define FEEDBACK_IDX_MASK (1 << FEEDBACK_IDX)


#define LED1_PIO_ID			ID_PIOA
#define LED2_PIO_ID			ID_PIOC
#define LED3_PIO_ID			ID_PIOB


#define LED1_PIO_IDX		0
#define LED2_PIO_IDX		30
#define LED3_PIO_IDX		2


#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)


// Botão
#define BUT_PIO      PIOA
#define BUT_PIO_ID   ID_PIOA
#define BUT_IDX      11
#define BUT_IDX_MASK (1 << BUT_IDX)

#define BUT1_PIO			PIOA
#define BUT1_PIO_ID			ID_PIOA
#define BUT1_PIO_IDX		2
#define BUT1_PIO_IDX_MASK  (1 << BUT1_PIO_IDX)

#define LED_FEEDBACK_PIO PIOA
#define LED_FEEDBACK_PIO_ID ID_PIOA
#define LED_FEEDBACK_PIO_IDX 13
#define LED_FEEDBACK_PIO_IDX_MASK (1 << LED_FEEDBACK_PIO_IDX)

#define BUTTON_FEEDBACK_PIO PIOC
#define BUTTON_FEEDBACK_PIO_ID ID_PIOC
#define BUTTON_FEEDBACK_PIO_IDX 19
#define BUTTON_FEEDBACK_PIO_IDX_MASK (1 << BUTTON_FEEDBACK_PIO_IDX)

#define TRIGGER_PIO PIOA
#define TRIGGER_PIO_ID ID_PIOA
#define TRIGGER_PIO_IDX 6
#define TRIGGER_PIO_IDX_MASK (1 << TRIGGER_PIO_IDX)

#define AFEC_POT AFEC0
#define AFEC_POT_ID ID_AFEC0
#define AFEC_POT_CHANNEL 0 // Canal do pino PD 30

#define AFEC1_POT AFEC1
#define AFEC1_POT_ID ID_AFEC1
#define AFEC1_POT_CHANNEL 1 // Canal do pino PC 13

// IMU

#define TWIHS_MCU6050_ID    ID_TWIHS0
#define TWIHS_MCU6050       TWIHS0

#define TWIHS_MCU6050_ID2 ID_TWIHS2
#define TWIHS_MCU60502 TWIHS2


#define EOF 'X'
#define HEADER 'K'

// usart (bluetooth ou serial)
// Descomente para enviar dados
// pela serial debug

//#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif


//flags

//volatile char  rtt_read_timer_value;

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_BLUETOOTH_STACK_SIZE            (4096/sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_ADC_STACK_SIZE (1024*10 / sizeof(portSTACK_TYPE))
#define TASK_ADC_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_PROC_STACK_SIZE (1024*10 / sizeof(portSTACK_TYPE))
#define TASK_PROC_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_IMU_STACK_SIZE (1024*10 / sizeof(portSTACK_TYPE))
#define TASK_IMU_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_IMU2_STACK_SIZE (1024*10 / sizeof(portSTACK_TYPE))
#define TASK_IMU2_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);


SemaphoreHandle_t xSemaphoreBut;
QueueHandle_t xQueueADC;
QueueHandle_t xQueuePROC;
QueueHandle_t xQueueACC;
QueueHandle_t xQueueGYR;

typedef struct {
	char id;
	uint16_t value;
} adcData;

typedef struct {
	char x1;
	char x2;
	char y1;
	char y2;
} Analog;

typedef struct {
	char yaw_1;
	char yaw_2;
	char yaw_3;
	char yaw_4;
} gyrData ;

float rangePerDigit ; // 2G
//const float rangePerDigit = 9.80665f ; // 2G

volatile uint8_t flag_led0 = 1;



int16_t  accX_raw2, accY_raw2, accZ_raw2;
float accX2, accY2, accZ2;
int acc2;
volatile uint8_t  accXHigh2, accYHigh2, accZHigh2;
volatile uint8_t  accXLow2,  accYLow2,  accZLow2;

int16_t   gyrX_raw2, gyrY_raw2, gyrZ_raw2;
float gyrX2, gyrY2, gyrZ2, gyr2;
volatile uint8_t  gyrXHigh2, gyrYHigh2, gyrZHigh2;
volatile uint8_t  gyrXLow2,  gyrYLow2,  gyrZLow2;


static void USART1_init(void);
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback);
static void configure_console(void);

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* RTOS application HOOK                                                */
/************************************************************************/

/* Called if stack overflow during execution */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	* identify which task has overflowed its stack.
	*/
	for (;;) {
	}
}

/* This function is called by FreeRTOS idle task */
extern void vApplicationIdleHook(void) {
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}

/* This function is called by FreeRTOS each tick */
extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel,
afec_callback_t callback) {
	/*************************************
	* Ativa e configura AFEC
	*************************************/
	/* Ativa AFEC - 0 */
	afec_enable(afec);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(afec, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(afec, AFEC_TRIG_SW);

	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	down to 0.
	*/
	afec_channel_set_analog_offset(afec, afec_channel, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

	/* configura IRQ */
	afec_set_callback(afec, afec_channel, callback, 1);
	NVIC_SetPriority(afec_id, 4);
	NVIC_EnableIRQ(afec_id);
}



void mcu6050_i2c_bus_init(uint32_t twihs_id, Twihs *twihs)
{
	twihs_options_t bno055_option;
	pmc_enable_periph_clk(twihs_id);

	/* Configure the options of TWI driver */
	bno055_option.master_clk = sysclk_get_cpu_hz();
	bno055_option.speed      = 40000;
	twihs_master_init(twihs, &bno055_option);
}



int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt, Twihs *twihs)
{
	int32_t ierror = 0x00;

	twihs_packet_t p_packet;
	p_packet.chip         = dev_addr;
	p_packet.addr[0]      = reg_addr;
	p_packet.addr_length  = 1;
	p_packet.buffer       = reg_data;
	p_packet.length       = cnt;
	
	ierror = twihs_master_write(twihs, &p_packet);

	return (int8_t)ierror;
}


int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt, Twihs *twihs)
{
	int32_t ierror = 0x00;
	
	twihs_packet_t p_packet;
	p_packet.chip         = dev_addr;
	p_packet.addr[0]      = reg_addr;
	p_packet.addr_length  = 1;
	p_packet.buffer       = reg_data;
	p_packet.length       = cnt;
	
	// TODO: Algum problema no SPI faz com que devemos ler duas vezes o registrador para
	//       conseguirmos pegar o valor correto.
	ierror = twihs_master_read(twihs, &p_packet);
	ierror = twihs_master_read(twihs, &p_packet);

	return (int8_t)ierror;
}

void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq) {
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	pmc_enable_periph_clk(ID_TC);

	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	NVIC_SetPriority((IRQn_Type)ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type)ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void TC1_Handler(void) {
	volatile uint32_t ul_dummy;

	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
	afec_start_software_conversion(AFEC_POT);
	
	afec_channel_enable(AFEC1_POT, AFEC1_POT_CHANNEL);
	afec_start_software_conversion(AFEC1_POT);
}

static void AFEC_pot_Callback(void) {
	adcData adc;
	adc.id = 'x';
	adc.value = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
	
}

static void AFEC1_pot_Callback(void){
	adcData adc;
	adc.id = 'y';
	adc.value = afec_channel_get_value(AFEC1_POT, AFEC1_POT_CHANNEL);
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueADC, &adc, &xHigherPriorityTaskWoken);
}


void trigger_led() {
	
	pio_clear(LED_PIO,LED_IDX_MASK);
	vTaskDelay( 50 / portTICK_PERIOD_MS);
	pio_set(LED_PIO,LED_IDX_MASK);
	
	
}

void led_feedback() {
	pio_set(LED_FEEDBACK_PIO,LED_FEEDBACK_PIO_IDX_MASK);
}


void but_callback(void) {
	
}

void but1_callback(void) {

}

void button_feedback_callback(void) {
	printf("BUTAO FEEDBAKC");
}
/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void io_init(void) {

	// Ativa PIOs
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUT1_PIO_ID);
	pmc_enable_periph_clk(LED_FEEDBACK_PIO_ID);
	pmc_enable_periph_clk(TRIGGER_PIO_ID);
	pmc_enable_periph_clk(FEEDBACK_ID);

	// Configura Pinos
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(LED_FEEDBACK_PIO,PIO_OUTPUT_0,LED_FEEDBACK_PIO_IDX_MASK,PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(FEEDBACK, PIO_OUTPUT_0, FEEDBACK_IDX_MASK, PIO_DEFAULT | PIO_DEBOUNCE);
	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP);
	pio_configure(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(BUTTON_FEEDBACK_PIO, PIO_INPUT, BUTTON_FEEDBACK_PIO_IDX_MASK, PIO_PULLUP);
	pio_configure(TRIGGER_PIO, PIO_INPUT, TRIGGER_PIO_IDX_MASK, PIO_PULLUP);
	
	pio_set(LED_PIO,LED_IDX_MASK);
	
	pio_handler_set(BUT_PIO,
	BUT_PIO_ID,
	BUT_IDX_MASK,
	PIO_IT_EDGE,
	but_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
	pio_get_interrupt_status(BUT_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4
	
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_PIO_IDX_MASK,
	PIO_IT_EDGE,
	but1_callback);

	// Ativa interrupção e limpa primeira IRQ gerada na ativacao
	pio_enable_interrupt(BUT1_PIO, BUT1_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT1_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	
	pio_handler_set(BUTTON_FEEDBACK_PIO,
	BUTTON_FEEDBACK_PIO_ID,
	BUTTON_FEEDBACK_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	button_feedback_callback);
	
	pio_enable_interrupt(BUTTON_FEEDBACK_PIO, BUTTON_FEEDBACK_PIO_IDX_MASK);
	pio_get_interrupt_status(BUTTON_FEEDBACK_PIO);
	
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUTTON_FEEDBACK_PIO_ID);
	NVIC_SetPriority(BUTTON_FEEDBACK_PIO_ID, 4); // Prioridade 4
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
		#endif
		.paritytype = CONF_UART_PARITY,
		#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
		#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	#if defined(__GNUC__)
	setbuf(stdout, NULL);
	#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	* emits one character at a time.
	*/
	#endif
}

static void configure_console_imu(void)
{

	/* Configura USART1 Pinos */
	sysclk_enable_peripheral_clock(ID_PIOB);
	sysclk_enable_peripheral_clock(ID_PIOA);
	pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);  // RX
	pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
	MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
	
	const usart_serial_options_t uart_serial_options = {
		.baudrate   = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits   = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

uint32_t usart_puts(uint8_t *pstring) {
	uint32_t i ;

	while(*(pstring + i))
	if(uart_is_tx_empty(USART_COM))
	usart_serial_putchar(USART_COM, *(pstring+i++));
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, uint timeout_ms) {
	uint timecounter = timeout_ms;
	uint32_t rx;
	uint32_t counter = 0;

	while( (timecounter > 0) && (counter < bufferlen - 1)) {
		if(usart_read(usart, &rx) == 0) {
			buffer[counter++] = rx;
		}
		else{
			timecounter--;
			vTaskDelay(1);
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen,
char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void config_usart0(void) {
	sysclk_enable_peripheral_clock(ID_USART0);
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);

	// RX - PB0  TX - PB1
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_init(void) {
	char buffer_rx[128];
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+NAMEJanSan", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	usart_send_command(USART_COM, buffer_rx, 1000, "AT+PIN0000", 100);
	vTaskDelay( 500 / portTICK_PERIOD_MS);
	
}

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/





static void task_imu(void) {
	printf("Task IMU started \n");
	
	int16_t  accX_raw, accY_raw, accZ_raw;
	float accX, accY, accZ;
	int acc;
	uint8_t  accXHigh, accYHigh, accZHigh;
	uint8_t  accXLow,  accYLow,  accZLow;

	int16_t   gyrX_raw, gyrY_raw, gyrZ_raw;
	float gyrX, gyrY, gyrZ, gyr;
	uint8_t  gyrXHigh, gyrYHigh, gyrZHigh;
	uint8_t  gyrXLow,  gyrYLow,  gyrZLow;
	
	uint8_t bufferRX[100];
	uint8_t bufferTX[100];
	
	uint8_t rtn;
	uint8_t rtn_gyro;
	
	char load;
	
	configure_console_imu();
	/************************************************************************/
	/* MPU                                                                  */
	/************************************************************************/
	delay_init( sysclk_get_cpu_hz());
	
	/* Inicializa Função de fusão */
	FusionAhrs ahrs;
	FusionAhrsInitialise(&ahrs);
	
	/* Inicializa i2c */
	printf("Inicializando bus i2c \n");
	//mcu6050_i2c_bus_init(TWIHS_MCU6050_ID, TWIHS_MCU6050);
	
	// Verifica MPU
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, bufferRX, 1, TWIHS_MCU6050);
	if(rtn != TWIHS_SUCCESS){
		printf("[ERRO] [i2c] [read] \n");
	}
	
	// Por algum motivo a primeira leitura é errada.
	if(bufferRX[0] != 0x68){
		printf("[ERRO] [mcu] [Wrong device] [0x%2X] \n", bufferRX[0]);
	}
	
	// Set Clock source
	bufferTX[0] = MPU6050_CLOCK_PLL_XGYRO;
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, bufferTX, 1, TWIHS_MCU6050);
	if(rtn != TWIHS_SUCCESS)
	printf("[ERRO] [i2c] [write] \n");
	
	RTT_init(1000,0,0);
	int tick_old = 0;
	
	
	
	while (1) {
		
		bufferTX[0] = 0x00; // 2G
		rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, bufferTX, 1, TWIHS_MCU6050);
		
		// Le valor do acc X High e Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &accXHigh, 1, TWIHS_MCU6050);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, &accXLow,  1, TWIHS_MCU6050);
		
		// Le valor do acc y High e  Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, &accYHigh, 1, TWIHS_MCU6050);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &accYLow,  1, TWIHS_MCU6050);
		
		// Le valor do acc z HIGH e Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, &accZHigh, 1, TWIHS_MCU6050);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &accZLow,  1, TWIHS_MCU6050);
		
		
		// Configura range gyroscopio para operar com 250 °/s
		bufferTX[0] = 0x00; // 250 °/s
		mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, bufferTX, 1, TWIHS_MCU6050);
		
		
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, &gyrXHigh, 1, TWIHS_MCU6050);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_L, &gyrXLow,  1, TWIHS_MCU6050);
		
		// Le valor do acc y High e  Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_H, &gyrYHigh, 1, TWIHS_MCU6050);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &gyrYLow,  1, TWIHS_MCU6050);
		
		// Le valor do acc z HIGH e Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &gyrZHigh, 1, TWIHS_MCU6050);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &gyrZLow,  1, TWIHS_MCU6050);
		
		// Dados são do tipo complemento de dois
		accX_raw = (accXHigh << 8) | (accXLow << 0);
		accY_raw = (accYHigh << 8) | (accYLow << 0);
		accZ_raw = (accZHigh << 8) | (accZLow << 0);
		accX = (float)accX_raw/16384;
		accY = (float)accY_raw/16384;
		accZ = (float)accZ_raw/16384;
		
		
		
		// Dados são do tipo complemento de dois
		gyrX_raw = (gyrXHigh << 8) | (gyrXLow << 0);
		gyrY_raw = (gyrYHigh << 8) | (gyrYLow << 0);
		gyrZ_raw = (gyrZHigh << 8) | (gyrZLow << 0);
		gyrX = (float)gyrX_raw/131;
		gyrY = (float)gyrY_raw/131;
		gyrZ = (float)gyrZ_raw/131;
		
		
		// replace this with actual gyroscope data in degrees/s
		const FusionVector gyroscope = {gyrX, gyrY, gyrZ};
		// replace this with actual accelerometer data in g
		const FusionVector accelerometer = {accX, accY , accZ};
		
		// calcula runtime do código acima para definir delta t do programa
		int tick = rtt_read_timer_value(RTT);
		float delta_time = (tick - tick_old)/1000.0;
		tick_old = tick;

		// aplica o algoritmo
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta_time);
		
		acc = sqrt(accelerometer.axis.x*accelerometer.axis.x + accelerometer.axis.y*accelerometer.axis.y + accelerometer.axis.z*accelerometer.axis.z);

		// dados em pitch roll e yaw
		const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		//
		//  		printf("Acceleration \n");
		//		printf("x/y/z : %f / %f / %f. mod:%d \n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z,acc);
		//  		printf("\n");
		
		//printf("IMU [1]\n");
		//printf("%f: Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n\n\n", delta_time, euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
		
		if (Abs(acc)> 1) {
			load = 1;
			//printf("entrou na IMU\n\n");
			} else {
			load = 0;
		}
		
		
		xQueueSend(xQueueACC, &load,(TickType_t) 0);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		
		
	}
}

static void task_imu2(void) {
	printf("Task IMU started \n");
	
	int16_t  accX_raw, accY_raw, accZ_raw;
	float accX, accY, accZ;
	int acc;
	uint8_t  accXHigh, accYHigh, accZHigh;
	uint8_t  accXLow,  accYLow,  accZLow;

	int16_t   gyrX_raw, gyrY_raw, gyrZ_raw;
	float gyrX, gyrY, gyrZ, gyr;
	uint8_t  gyrXHigh, gyrYHigh, gyrZHigh;
	uint8_t  gyrXLow,  gyrYLow,  gyrZLow;
	
	uint8_t bufferRX[100];
	uint8_t bufferTX[100];
	
	uint8_t rtn;
	uint8_t rtn_gyro;
	
	int16_t reference_yaw;
	char first_value = 0;
	int16_t yaw;
	gyrData yaws;
	
	
	
	char load;
	
	configure_console_imu();
	/************************************************************************/
	/* MPU                                                                  */
	/************************************************************************/
	delay_init( sysclk_get_cpu_hz());
	
	/* Inicializa Função de fusão */
	FusionAhrs ahrs;
	FusionAhrsInitialise(&ahrs);
	
	/* Inicializa i2c */
	printf("Inicializando bus i2c [IMU 2] \n");
	//mcu6050_i2c_bus_init(TWIHS_MCU6050_ID2, TWIHS_MCU60502);
	
	/** Enable TWIHS port. */
	pmc_enable_periph_clk(ID_PIOD);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 28);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 27);
	
	// Verifica MPU
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_WHO_AM_I, bufferRX, 1, TWIHS_MCU60502);
	if(rtn != TWIHS_SUCCESS){
		printf("[ERRO] [i2c] [read] [IMU 2] \n");
	}
	
	// Por algum motivo a primeira leitura é errada.
	if(bufferRX[0] != 0x68){
		printf("[ERRO] [mcu] [Wrong device] [0x%2X] [IMU 2] \n", bufferRX[0]);
	}
	
	// Set Clock source
	bufferTX[0] = MPU6050_CLOCK_PLL_XGYRO;
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_PWR_MGMT_1, bufferTX, 1, TWIHS_MCU60502);
	if(rtn != TWIHS_SUCCESS)
	printf("[ERRO] [i2c] [write] [IMU 2] \n");
	
	int tick_old = 0;
	
	
	
	while (1) {
		
		bufferTX[0] = 0x00; // 2G
		rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_ACCEL_CONFIG, bufferTX, 1, TWIHS_MCU60502);
		
		// Le valor do acc X High e Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_ACCEL_XOUT_H, &accXHigh, 1, TWIHS_MCU60502);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_ACCEL_XOUT_L, &accXLow,  1, TWIHS_MCU60502);
		
		// Le valor do acc y High e  Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_ACCEL_YOUT_H, &accYHigh, 1, TWIHS_MCU60502);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_ACCEL_ZOUT_L, &accYLow,  1, TWIHS_MCU60502);
		
		// Le valor do acc z HIGH e Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_ACCEL_ZOUT_H, &accZHigh, 1, TWIHS_MCU60502);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_ACCEL_ZOUT_L, &accZLow,  1, TWIHS_MCU60502);
		
		
		// Configura range gyroscopio para operar com 250 °/s
		bufferTX[0] = 0x00; // 250 °/s
		mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_GYRO_CONFIG, bufferTX, 1, TWIHS_MCU60502);
		
		
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_GYRO_XOUT_H, &gyrXHigh, 1, TWIHS_MCU60502);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_GYRO_XOUT_L, &gyrXLow,  1, TWIHS_MCU60502);
		
		// Le valor do acc y High e  Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_GYRO_YOUT_H, &gyrYHigh, 1, TWIHS_MCU60502);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_GYRO_ZOUT_L, &gyrYLow,  1, TWIHS_MCU60502);
		
		// Le valor do acc z HIGH e Low
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_GYRO_ZOUT_H, &gyrZHigh, 1, TWIHS_MCU60502);
		rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS2, MPU6050_RA_GYRO_ZOUT_L, &gyrZLow,  1, TWIHS_MCU60502);
		
		// Dados são do tipo complemento de dois
		accX_raw = (accXHigh << 8) | (accXLow << 0);
		accY_raw = (accYHigh << 8) | (accYLow << 0);
		accZ_raw = (accZHigh << 8) | (accZLow << 0);
		accX = (float)accX_raw/16384;
		accY = (float)accY_raw/16384;
		accZ = (float)accZ_raw/16384;
		
		
		
		// Dados são do tipo complemento de dois
		gyrX_raw = (gyrXHigh << 8) | (gyrXLow << 0);
		gyrY_raw = (gyrYHigh << 8) | (gyrYLow << 0);
		gyrZ_raw = (gyrZHigh << 8) | (gyrZLow << 0);
		gyrX = (float)gyrX_raw/131;
		gyrY = (float)gyrY_raw/131;
		gyrZ = (float)gyrZ_raw/131;
		
		
		
		// replace this with actual gyroscope data in degrees/s
		const FusionVector gyroscope = {gyrX, gyrY, gyrZ};
		// replace this with actual accelerometer data in g
		const FusionVector accelerometer = {accX, accY , accZ};
		
		// calcula runtime do código acima para definir delta t do programa
		int tick = rtt_read_timer_value(RTT);
		float delta_time = (tick - tick_old)/1000.0;
		tick_old = tick;

		// aplica o algoritmo
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta_time);

		// dados em pitch roll e yaw
		const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		//
		//  		printf("Acceleration \n");
		//		printf("x/y/z : %f / %f / %f. mod:%d \n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z,acc);
		//  		printf("\n");
		
		printf("IMU [2] \n");
		//printf("%f: Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n\n\n", delta_time, euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
		
		if (first_value == 6) {
			reference_yaw = euler.angle.yaw;
			yaw = euler.angle.yaw;
			first_value++;
		} else {
			yaw = euler.angle.yaw - reference_yaw;
		}
		
		
		
		printf("%d\n\n",yaw);
		yaws.yaw_1 = yaw;
		yaws.yaw_2 = yaw >> 4;
		yaws.yaw_3 = yaw >> 8;
		yaws.yaw_4 = yaw >> 12;
		
		int16_t new_yaw = (yaws.yaw_4 << 12) | (yaws.yaw_3 << 8) | (yaws.yaw_2 << 4) | (yaws.yaw_1);
		printf("new yaw: %d", new_yaw);
		
		
		
		
		xQueueSend(xQueueGYR, &yaws,(TickType_t) 0);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}


void task_bluetooth(void) {
	printf("Task Bluetooth started \n");
	
	printf("Inicializando HC05 \n");
	config_usart0();
	hc05_init();

	// configura LEDs e Botões
	io_init();

	char buttonA = '0';
	char buttonB = '0';
	char analogx1 = '0';
	char analogx2 = '0';
	char analogy1 = '0';
	char analogy2 = '0';
	char buttonTrigger = '0';
	char buttonMenu = '0';
	char buttonLoad = '0';
	char send = 0;
	uint32_t b = 0;
	// Task não deve retornar.
	
	config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL, AFEC_pot_Callback);
	config_AFEC_pot(AFEC1_POT, AFEC1_POT_ID, AFEC1_POT_CHANNEL, AFEC1_pot_Callback);
	TC_init(TC0, ID_TC1, 1, 10);
	tc_start(TC0, 1);

	adcData adc;
	uint16_t x = 0;
	uint16_t y = 0;
	char c = 0;
	char handshake = 0;
	gyrData yaws;
	
	while(1) {
		
		
		Analog analog;
		char loading;
		
		/* verifica se chegou algum dado na queue, e espera por 0 ticks */
		
		
		if (xQueueReceive(xQueueADC, &adc, (TickType_t) 0)) {
			
			if(adc.id == 'x') {
				analogx2 = adc.value >> 8;
				analogx1 = adc.value;
				} else {
				analogy2 = adc.value >> 8;
				analogy1 = adc.value;
			}
			send = 1;
		}
		
		if (xQueueReceive(xQueueACC,&loading, (TickType_t) 0)) {
			send = 1;
			if (loading) {
				buttonLoad = '1';
				} else {
				buttonLoad = '0';
			}
			} else {
			buttonLoad = '0';
		}
		
		if (xQueueReceive(xQueueGYR, &yaws, (TickType_t) 0)) {
			
			send = 1;
		}
			// atualiza valor do botão
			if(pio_get(BUT_PIO, PIO_INPUT, BUT_IDX_MASK) == 0) {
				buttonA = '1';
				trigger_led();
				} else {
				buttonA = '0';
			}
			
			if(pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIO_IDX_MASK) == 0) {
				buttonB = '1';
				} else {
				buttonB = '0';
			}
			
			if(pio_get(TRIGGER_PIO,PIO_INPUT, TRIGGER_PIO_IDX_MASK) == 0) {
				buttonTrigger = '1';
				pio_set(FEEDBACK, FEEDBACK_IDX_MASK);      // Coloca 1 no pino LED
				delay_ms(10);                       // Delay por software de 200 ms
				pio_clear(FEEDBACK, FEEDBACK_IDX_MASK);    // Coloca 0 no pino do LED
				delay_ms(10);                       // Delay por software de 200 ms
				
				} else {
				buttonTrigger = '0';
			}
			
			if (pio_get(BUTTON_FEEDBACK_PIO, PIO_INPUT, BUTTON_FEEDBACK_PIO_IDX_MASK) == 0) {
				buttonMenu = '1';
				printf("Tenta enviar");
				} else {
				buttonMenu = '0';
			}
			
			if(send && handshake) {
				// envia inicio de pacote
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, HEADER);

				// envia status botãoA
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, buttonA);
				
				// envia status botãoB
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, buttonB);
				
				// envia status botãoC/Trigger
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, buttonTrigger);
				
				// envia status botãoMenu
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, buttonMenu);
				
				// envia status botãoLoad
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, buttonLoad);
				
				// envia status analogx1
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, analogx1);
				
				// envia status analogx2
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, analogx2);
				
				// envia status analogy1
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, analogy1);
				
				// envia status analogy2
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, analogy2);
				
				// envia status yaw_1
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, yaws.yaw_1);
				
				// envia status yaw_2
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, yaws.yaw_2);
				
				// envia status yaw_3
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, yaws.yaw_3);
				
				// envia status yaw_4
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, yaws.yaw_4);
				
				// envia fim de pacote
				while(!usart_is_tx_ready(USART_COM)) {
					vTaskDelay(1 / portTICK_PERIOD_MS);
				}
				usart_write(USART_COM, EOF);
				
				
				send = 0;
				} else if (!handshake) {
				
				while(!usart_is_rx_ready(USART_COM)) {
					
					while(!usart_is_tx_ready(USART_COM)) {
						vTaskDelay(1 / portTICK_PERIOD_MS);
					}
					
					usart_write(USART_COM, HEADER);
					vTaskDelay(1 / portTICK_PERIOD_MS);
					
				}
				
				b = usart_read(USART_COM,&c);
				
				if (c == 'i') {
					handshake = 1;
					led_feedback();
					printf("%c",c);
				}
				
				
			}
			
		}
	}

	/************************************************************************/
	/* main                                                                 */
	/************************************************************************/

	int main(void) {
		/* Initialize the SAM system */
		
		/* buffer para recebimento de dados */
		

		
		sysclk_init();
		board_init();

		configure_console();
		mcu6050_i2c_bus_init(TWIHS_MCU6050_ID, TWIHS_MCU6050);
		mcu6050_i2c_bus_init(TWIHS_MCU6050_ID2, TWIHS_MCU60502);

		
		//CRIA A FILA
		xQueueADC = xQueueCreate(100, sizeof(adcData));
		if (xQueueADC == NULL)
		printf("falha em criar a queue xQueueADC \n");
		
		xQueuePROC = xQueueCreate(100, sizeof(adcData));
		if (xQueuePROC == NULL)
		printf("falha em criar a queue xQueuePROC \n");
		
		xQueueACC = xQueueCreate(100, sizeof(char));
		if (xQueueACC == NULL)
		printf("falha em criar a queue xQueueACC \n");
		
		xQueueGYR = xQueueCreate(100, sizeof(gyrData));
		if (xQueueGYR == NULL)
		printf("falha em criar a queue xQueueGYR \n");
		
		
		
		
		
		
		
		
		if (xTaskCreate(task_imu, "IMU", TASK_IMU_STACK_SIZE, NULL,
		TASK_IMU_STACK_PRIORITY, NULL) != pdPASS) {
			printf("Failed to create test IMU task\r\n");
		}
		
		/* Create task to make led blink */
		xTaskCreate(task_bluetooth, "BLT", TASK_BLUETOOTH_STACK_SIZE, NULL,	TASK_BLUETOOTH_STACK_PRIORITY, NULL);
		
		xTaskCreate(task_imu2, "IMU2", TASK_IMU2_STACK_SIZE, NULL,	TASK_IMU2_STACK_PRIORITY, NULL);

		/* Start the scheduler. */
		vTaskStartScheduler();

		while(1){}

		/* Will only get here if there was insufficient memory to create the idle task. */
		return 0;
		
		}
