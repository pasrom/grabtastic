/*
* This file includes all header files
* And also all programed functions
*/
#include <string.h>
#ifndef CONFIG_FOCUS_FHV_H
#define CONFIG_FOCUS_FHV_H
#define richtung false
#define respond_I "i"
#define respond_H "h"
#define respond_D "d"
#define respond_P "p"
#define respond_ok "ok"
#define respondBussy "bussy"
#define respondUnkown "UNKOWN"
#define respondStartetUp "Started up"
#define respondOK "ok"
#define trennzeichen '/'
//! [definition_DRV8711]
/** definitions commands */
/* setting for the moter, speed and that shit*/
#define trapezSteigung 1.5
/* this are the mm values to drive for target D */
#define targetDinmm 67
//* this are the mm values to drive for target P */
#define targetPinmm 35
#define freifahrenInmm 8
#define forErrorInmm (targetDinmm - 35)
/* this are the mm values to drive after D*/
#define repositioningAfterDownInmm (targetDinmm - 2)
/* it needs 400 spin for one completaly turn at full step */
#define oneSpin 400
#define targetH 1100
#define targetP (int) (targetH + targetPinmm / trapezSteigung * oneSpin)
#define targetD (int) (targetH + targetDinmm / trapezSteigung * oneSpin)
#define targetFreifahren (int) (targetH + freifahrenInmm / trapezSteigung * oneSpin)
#define targetRepositioningAfterDown (int) (targetH + repositioningAfterDownInmm / trapezSteigung * oneSpin)
#define targetForError  (int) (targetH + forErrorInmm / trapezSteigung * oneSpin)
#define treshholdAccelerateUp 9500
#define treshholdAccelerateDown 7600
#define accelerationRate 1
#define differenceTreshhold (treshholdAccelerateUp - treshholdAccelerateDown) / accelerationRate
#define trehsholdNotMovingError 50
//! [definition_DRV8711]
//! [definition_StrainGauge]
/* things for the strain gauge*/
#define maxN 20000.0
#define scalingFactorForce 2.15
#define offsetIfDRVon  0.0
//! [definition_StrainGauge]
/********** Fuction Prototypes *************/
void delete_buffer(uint8_t *buffer);
void writeToDRV(volatile uint8_t *wr_config, volatile uint8_t *rd_config, uint16_t length);
void deleteBuffer(char *arrayBuffer, uint16_t lenght);
void addToWriteBuffer(char *arrayInput, const char *arrayCopy);
double calculateMedian(volatile uint16_t *valueBuffer, uint16_t length);
volatile bool targetDirection (int iActualPos, int iTargetPos);
void executeOrder(void);
void reposIfInLimitSwitch(void);
void finishTheOrder(void);
void calculatePosZaxis(void);
void startRepositioningWhenDown(void);
void errorHandling(void);
void sendRestartTelegram(void);
void sendTheTelegram(void);
void deleteRespondBuffer(void);
void sendBusy(void);
void sendInvalidTelegram(void);
void sendOK(void);
void telegramPreparation(void);
void insertValuesIntoStruct(void);
void initGrabtastic(void);
/* protoype port configuration*/
void configure_port_pins(void);
/* prototype Integer to Int */
void reverse(char s[]);
void itoaEigen(int n, char s[]);
/* protoype DRV8711 write and read default values*/
void drv8711enable (bool reference, bool direction);
void DRV8711disable(void);
/* Timter Counter */
void configure_tc(void);
void configure_tc_callbacks(void);
void tc_callback_to_change_duty_cycle(struct tc_module *const module_inst);
/* USART RS232 */
void usart_read_callback(const struct usart_module *const usart_module);
void usart_write_callback(const struct usart_module *const usart_module);
void configure_usart(void);
void configure_usart_callbacks(void);
/* SPI */
void configure_spi_master_callbacks(void);
void configure_spi_master(void);
/* Interrupt Limit Switch */
void configure_extint_channel(void);
void configure_extint_callbacks(void);
void extint_detection_callback(void);
/* Interrupt Emergency Stop */
void configure_extint_channel_emergency(void);
void extint_detection_callback_emergency(void);
/* ADC */
void configure_adc(void);
void configure_adc_callbacks(void);
void adc_complete_callback(struct adc_module *const module);

/* Timter Counter */
struct adc_config config_adc;
struct tc_module tc_instance;
/* USART RS232 */
struct usart_module usart_instance;
#define MAX_RX_BUFFER_LENGTH 1
volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];
volatile uint8_t	rx_string[MAX_RX_BUFFER_LENGTH] = {0};
/* SPI */
#define BUF_LENGTH 2
#define SLAVE_SELECT_PIN EXT2_PIN_SPI_SS_0
static uint8_t wr_buffer[BUF_LENGTH] = {0x01, 0x02};
static uint8_t rd_buffer[BUF_LENGTH];
struct spi_module spi_master_instance;
struct spi_slave_inst slave;
volatile bool transrev_complete_spi_master = false;
/* ADC */
#define ADC_SAMPLES 128
volatile uint16_t adc_result_buffer[ADC_SAMPLES];
struct adc_module adc_instance;
volatile bool adc_read_done = false;

/* Program Status */
volatile uint8_t globalOrder = 0;		/*!< received Order to execute */
volatile uint8_t globalSendOK = 0;		/*!< send respond after an order received */
volatile uint8_t globalDrive = 0;		/*!< if movement is executing Drive is 1, if its finished it's 2 */
volatile bool globalSendStatus = false; /*!< after drive gets 2, the status will be send. To get the Status ready */
volatile bool globalActualDirection = false;	/*!< the actual direction of the motor. down=false, up=true */
volatile bool globalInvalidTelegram = false;	/*!< respond to a unknown command */
volatile bool globalBusy = false;				/*!< respond if getting an new order, but there is already executing an order, expects it is a reset command */
volatile bool globalEmergencyPressed = false;	/*!< if an emergency stop is detected, this is true */
volatile bool globalLimitSwitchPressed = true;	/*!< if limit switch is pressed the value is true */
volatile bool globalOrderWasDown = false;		/*!< the order before was down */
static uint8_t globalMovingFreeAxis = 0;		/*!< the axis has to move down first, to do the referencing */

char globalRespondTelegram[20];	/*!< is a global buffer for the respond telegram in ASCII */
static bool globalSendRespond = false; /*!< is true, if the respond has to be send */
char globalPositionAxisForTelegram [20] = {0}; /*!< is a global buffer for the position of the Z axis in ASCII */
char globalErrorForTelegram [20] = {0}; /*!< is a global buffer for the error of the Z axis in ASCII */
static bool globalSendRestartTelegram = true; /*!< if the uC has turned on, we want to know it, therefore send something */
double globalResult = 0.0;
/* Measurement */
volatile int globalStepPositionAxis = targetH;
volatile static uint8_t wr_drv8711Defaults[] = {0xF, 0x0, 0x11, 0x64, 0x20, 0xC8, 0x30, 0x64, 0x40, 0x32, 0x50, 0x40, 0x65, 0xA4, 0x70, 0x0};  /*!< uint8 array for send the config of the DRV8711 */
volatile static uint8_t wr_drv8711Read[] = {0x80,  0x00, 0x90, 0x00, 0xA0,  0x00, 0xB0,  0x00, 0xC0,  0x00, 0xD0,  0x00, 0xE0,  0x00, 0x70, 0x00};  /*!< uint8 array for receive the config of the DRV8711 */
volatile static uint8_t rd_drv8711Defaults[sizeof(wr_drv8711Defaults)]={0};  /*!< uint8 array for get the status of the DRV8711 */
volatile static uint8_t rd_drv8711Read[sizeof(wr_drv8711Read)]={0};			/*!< uint8 array for receive the status of the DRV8711 */

/* struct of order */
#define lengt 6
struct index {
	int targetPosition;
	bool direction;
	bool repos;
	int order;
	char receive;
	char respond;
};
struct index positionOfEachOrder[lengt]; /* array number */

/**
* \brief insert values into array of struct
*
* this is an array of struct indes, and carry all necessary values for the order, etc.
*
*/
void insertValuesIntoStruct(void)
{
	uint8_t j = 1;
	positionOfEachOrder[j].targetPosition=1;
	positionOfEachOrder[j].direction=richtung;
	positionOfEachOrder[j].order = 1;
	positionOfEachOrder[j].repos = 0;
	strcpy(&positionOfEachOrder[j].receive, "I");
	strcpy(&positionOfEachOrder[j].respond, respond_I);
	
	positionOfEachOrder[++j].targetPosition=targetD;
	positionOfEachOrder[j].direction=!richtung;
	positionOfEachOrder[j].order = 2;
	positionOfEachOrder[j].repos = 1;
	strcpy(&positionOfEachOrder[j].receive, "D");
	strcpy(&positionOfEachOrder[j].respond, respond_D);
	
	positionOfEachOrder[++j].targetPosition=targetH;
	positionOfEachOrder[j].direction=richtung;
	positionOfEachOrder[j].order = 3;
	positionOfEachOrder[j].repos = 0;
	strcpy(&positionOfEachOrder[j].receive, "H");
	strcpy(&positionOfEachOrder[j].respond, respond_H);
	
	positionOfEachOrder[++j].targetPosition=targetP;
	positionOfEachOrder[j].direction=richtung;
	positionOfEachOrder[j].order = 4;
	positionOfEachOrder[j].repos = 0;
	strcpy(&positionOfEachOrder[j].receive, "P");
	strcpy(&positionOfEachOrder[j].respond, respond_P);
	
	positionOfEachOrder[++j].targetPosition=targetRepositioningAfterDown; /* for repos*/
	positionOfEachOrder[j].direction=richtung;
	positionOfEachOrder[j].order = 5;
	positionOfEachOrder[j].repos = 0;
	strcpy(&positionOfEachOrder[j].receive, "");
	strcpy(&positionOfEachOrder[j].respond, respond_D);
	
}


Function umLagern
Integer j, k, m, n

For k = 0 To 1
For j = 0 To 5
P1 = XY(j * 40.0, k * 40, 35.0, 435.0) /1
Go P1
Print "Local 1: ", P1 @0
'bewegung zum local 1 (k und j)
aufnehmen
P2 = XY(m * 40.0, n * 40, 35.0, 190.0) /2
Go P2
Print "Local 2: ", P2 @0
'bewegung zum local 2 (m und n)
ablegen
m = m + 1
If m > 3 Then
n = n + 1
m = 0
EndIf
Next j
Next k
Off 9
Off 8
Fend

int j, k, m, n;
do {
    do{
        calculationAndMoving;
        m = m + 1;
        if (m > 3) {
            n ++;
            m = 0;
        }
    }while (j = 0; 5; j++);
}while (k = 0; 1; k++);

/**
 * \brief calculateOrder
*
* searches from the received command the belonging order struct
*
*/
void calculateOrder(char receivedOrder){
	int localFound;
	volatile int i = 0;
	for (i = 1; i < lengt ; i++) {
		if (positionOfEachOrder[i].receive == receivedOrder) { //matched
			/*init command immer ausfuehren, auch wenn schon ein order da ist*/
			localFound = strncmp(&positionOfEachOrder[i].receive, "I", sizeof(positionOfEachOrder[i].receive));
			if(!localFound == 0 && positionOfEachOrder[0].order != 0){
				globalBusy = true;
				break;
			}
			else /*sonst ausfuehren*/
			{
				//globalOrder = positionOfEachOrder[i].order;
				/* save the order in array number 0*/
				positionOfEachOrder[0].targetPosition = positionOfEachOrder[i].targetPosition;
				positionOfEachOrder[0].direction = positionOfEachOrder[i].direction;
				positionOfEachOrder[0].order = positionOfEachOrder[i].order;
				positionOfEachOrder[0].repos = positionOfEachOrder[i].repos;
				strcpy(&positionOfEachOrder[0].receive, &positionOfEachOrder[i].receive);
				strcpy(&positionOfEachOrder[0].respond, &positionOfEachOrder[i].respond);
				break;
			}
		}
	}
	if(i > lengt)
	{
		globalInvalidTelegram = true; //UNKOWN
	}

}

/**
* \brief callback frequency
*
* changes the frequency and manages the positioning
*
*/
void tc_callback_to_change_duty_cycle(struct tc_module *const module_inst)
{

	static uint16_t i = treshholdAccelerateUp;
	static bool accelerateUp = true;
	static bool accelerateDown = false;
	static int distanceToTarget = 0;
	static bool newOrder = true;
	volatile bool stop = false;
	if (!newOrder)
	{
		i = treshholdAccelerateUp; // reset if new order arrived
		accelerateUp = true;
		accelerateDown = false;
		newOrder = true;
	}
	
	if (( i < treshholdAccelerateDown ) && accelerateUp)
	{
		accelerateUp = false;

	}
	if (( i > treshholdAccelerateUp ) && accelerateDown)
	{
		accelerateUp = true;
		accelerateDown = false;
	}
	
	if (accelerateDown && (positionOfEachOrder[0].order != 1))
	{
		i = i + accelerationRate;
	}
	if(accelerateUp && (positionOfEachOrder[0].order != 1))
	{
		i = i - accelerationRate;
	}
	distanceToTarget = globalStepPositionAxis - positionOfEachOrder[0].targetPosition;
	distanceToTarget = abs (distanceToTarget);
	
	if(differenceTreshhold > distanceToTarget && (positionOfEachOrder[0].order != 1))
	{
		accelerateDown = true;
		accelerateUp = false;
	}
	tc_set_compare_value(module_inst, TC_COMPARE_CAPTURE_CHANNEL_0, i);
	/* count the virtual measurement of distance */
	if (globalActualDirection)
	{
		globalStepPositionAxis ++; /* down */
	}
	else
	{
		globalStepPositionAxis --;  /* up */
	}
	
	if((distanceToTarget <= 0 && positionOfEachOrder[0].order != 1))
	{
		stop = true;
	}

	if ((globalResult > 15000.0) | (globalResult < -15000.0))
	{
		globalEmergencyPressed = true;
	}

	if(stop | globalEmergencyPressed) // stop the PWM when Z Axis is down or up
	{
		tc_disable(&tc_instance); // pwm aus
		port_pin_set_output_level(EXT2_PIN_5,false); //sleep on
		globalDrive = 2; //-> send respond
		i = treshholdAccelerateUp;
		accelerateUp = true;
		accelerateDown = false;
		newOrder = false;
		stop = false;
		if (positionOfEachOrder[0].order == 2 && !globalEmergencyPressed) //if order was down
		{
			globalOrderWasDown = true;
		}
	}
}

/**
* \brief configure the TC
*
* configure the TC as frequency match mode
*
*/
void configure_tc(void)
{
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);
	config_tc.clock_source = GCLK_GENERATOR_0;
	config_tc.counter_size = TC_COUNTER_SIZE_16BIT;
	config_tc.counter_16_bit.compare_capture_channel[TC_COMPARE_CAPTURE_CHANNEL_0]  = treshholdAccelerateUp; //650
	config_tc.wave_generation = TC_WAVE_GENERATION_MATCH_FREQ_MODE;
	config_tc.pwm_channel[0].enabled = true;
	config_tc.pwm_channel[0].pin_out = PWM_OUT_PIN;
	config_tc.pwm_channel[0].pin_mux = PWM_OUT_MUX;
	tc_init(&tc_instance, PWM_MODULE, &config_tc);
	//tc_enable(&tc_instance);
}

/**
* \brief configure callbacks
*
* the TC callbacks are attached to the interrupt controller
*
*/
void configure_tc_callbacks(void)
{
	tc_register_callback(&tc_instance,tc_callback_to_change_duty_cycle,TC_CALLBACK_CC_CHANNEL0);
	tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
}
/**
* \brief RS232 read callbacks
*
* is called when the PLC sends a command to the uC
* it translates the order into a value stored in globalOrder
*/
void usart_read_callback(const struct usart_module *const usart_module)
{
	//	usart_write_buffer_job(&usart_instance, (uint8_t *) rx_buffer, MAX_RX_BUFFER_LENGTH);
	rx_string [0] = rx_buffer [0];	
	calculateOrder(rx_buffer [0]);
}
/**
* \brief RS232 write callbacks
*
* is called when, the uC is done with sending to the PLC
*/
void usart_write_callback(const struct usart_module *const usart_module)
{
	globalSendStatus = false;
}
/**
* \brief configure RS232
*
* RS232 is configured as standard
*/
void configure_usart(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	//config_usart.baudrate    = 9600;
	config_usart.baudrate    = 115200;
	config_usart.mux_setting = EXT1_UART_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EXT1_UART_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EXT1_UART_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EXT1_UART_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EXT1_UART_SERCOM_PINMUX_PAD3;
	
	while (usart_init(&usart_instance,EXT1_UART_MODULE, &config_usart) != STATUS_OK) {
	}
	usart_enable(&usart_instance);
}

/**
* \brief configure RS232 callbacks
*
* the RS232 callbacks are attached to the interrupt controller
*/
void configure_usart_callbacks(void)
{
	usart_register_callback(&usart_instance, (void*) usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance, (void*) usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);

	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}
/**
* \brief SPI callbacks
*
* is called, when the SPI finished the transmission
*/
static void callback_spi_master( struct spi_module *const module)
{
	transrev_complete_spi_master = true;
}
/**
* \brief configure SPI callbacks
*
* the SPI callbacks are attached to the interrupt controller
*/
void configure_spi_master_callbacks(void)
{
	spi_register_callback(&spi_master_instance, callback_spi_master,SPI_CALLBACK_BUFFER_TRANSCEIVED);
	spi_enable_callback(&spi_master_instance, SPI_CALLBACK_BUFFER_TRANSCEIVED);
}
/**
* \brief configure SPI
*
* SPI is configured as standard, but with on EXT2
*/
void configure_spi_master(void)
{
	struct spi_config config_spi_master;
	struct spi_slave_inst_config slave_dev_config;
	spi_slave_inst_get_config_defaults(&slave_dev_config);
	slave_dev_config.ss_pin = SLAVE_SELECT_PIN;
	spi_attach_slave(&slave, &slave_dev_config);
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mux_setting = EXT2_SPI_SERCOM_MUX_SETTING;
	config_spi_master.pinmux_pad0 = EXT2_SPI_SERCOM_PINMUX_PAD0;
	config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	config_spi_master.pinmux_pad2 = EXT2_SPI_SERCOM_PINMUX_PAD2;
	config_spi_master.pinmux_pad3 = EXT2_SPI_SERCOM_PINMUX_PAD3;
	config_spi_master.mode_specific.master.baudrate = 100000;
	spi_init(&spi_master_instance, EXT2_SPI_MODULE, &config_spi_master);
	spi_enable(&spi_master_instance);
}
/**
* \brief ADC callbacks
*
* is called, when the ADC finished the conversion
*/
void adc_complete_callback(struct adc_module *const module)
{
	adc_read_done = true;
}
/**
* \brief configure ADC
*
* SPI is configured on EXT1
* GCLK_GENERATOR_2
* ADC_CLOCK_PRESCALER_DIV4
* ADC_REFERENCE_AREFB
* ADC_RESOLUTION_10BIT
*/
void configure_adc(void)
{
	//struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);

	config_adc.clock_source	=  GCLK_GENERATOR_0;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV256;
	//	config_adc.reference       = ADC_REFERENCE_INTVCC1;
	config_adc.reference       = ADC_REFERENCE_AREFB;
	// PIN setzen bzw auswaehlen!!
	config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN8;
	config_adc.resolution      = ADC_RESOLUTION_10BIT;
	adc_init(&adc_instance, ADC, &config_adc);
	adc_disable_pin_scan_mode(&adc_instance);
	adc_enable(&adc_instance);
}
/**
* \brief configure ADC callbacks
*
* the ADC callbacks are attached to the interrupt controller
*/
void configure_adc_callbacks(void)
{
	adc_register_callback(&adc_instance,adc_complete_callback, ADC_CALLBACK_READ_BUFFER);
	adc_enable_callback(&adc_instance, ADC_CALLBACK_READ_BUFFER);
}
/**
* \brief configure Ports
*
* the used I/O's are configured here
*/
void configure_port_pins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.input_pull = PORT_PIN_PULL_DOWN;
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(EXT2_PIN_5, &config_port_pin);
	port_pin_set_config(EXT2_PIN_6, &config_port_pin);
	port_pin_set_config(EXT2_PIN_9, &config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_INPUT;
	//port_pin_set_config(EXT1_IRQ_INPUT, &config_port_pin);
}

/**
* \brief enables DRV8711
*
* Writes all necessary registert to DRV7811
*
* \param[in]  reference		bool if to enable the frequency
* \param[in]  direction		bool in which direction should DRV8711 turn
*/
void drv8711enable (bool reference, bool direction){

	port_pin_set_output_level(EXT2_PIN_5,false); //sleep on
	delay_ms(10);
	port_pin_set_output_level(EXT2_PIN_5,true); //sleep off
	delay_ms(10);
	
	port_pin_set_output_level(EXT2_PIN_6,true); //reset on
	delay_ms(10);
	port_pin_set_output_level(EXT2_PIN_6,false); //reset off
	delay_ms(10);
	
	globalActualDirection = direction;
	port_pin_set_output_level(EXT2_PIN_9, direction); // nach oben fahren vorbereiten
	delay_ms(40);
	
	writeToDRV(wr_drv8711Defaults, rd_drv8711Defaults, sizeof(wr_drv8711Defaults)); //-> write the DRV8711 Default
	
	wr_buffer [0] = wr_drv8711Defaults[0];
	wr_buffer [1] = wr_drv8711Defaults[1] | (uint8_t) 0b00000001; //enable drv8711
	writeToDRV(wr_buffer, rd_drv8711Read, sizeof(wr_buffer)); //-> write the enable command to DRV8711

	if (reference)
	{
		tc_enable(&tc_instance);
	}
	delay_ms(10);
}

/**
* \brief disables the DRV8711
*/
void DRV8711disable()
{
	wr_buffer [0] = wr_drv8711Defaults[0];
	wr_buffer [1] = wr_drv8711Defaults[1] | (uint8_t) 0b00000000; //disable drv8711
	writeToDRV( wr_buffer, rd_drv8711Read, sizeof(wr_buffer)); //-> write the enable command to DRV8711
	port_pin_set_output_level(EXT2_PIN_5,false); //sleep on
}

/**
* \brief limit switch callbacks
*
* the EXT3_PIN_IRQ is configured as emergency stop
*/
void configure_extint_channel(void)
{
	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin           = EXT3_PIN_IRQ;
	config_extint_chan.gpio_pin_mux       = EXT3_IRQ_MUX;
	config_extint_chan.filter_input_signal = true;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_DOWN;
	config_extint_chan.detection_criteria = EXTINT_DETECT_FALLING;
	extint_chan_set_config(EXT3_IRQ_INPUT, &config_extint_chan);
}

/**
* \brief configure Emergency Stop callbacks
*
* the EXT1_PIN_IRQ is configured as emergency stop
*/
void configure_extint_channel_emergency(void)
{
	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin           = EXT1_PIN_IRQ;
	config_extint_chan.gpio_pin_mux       = EXT1_IRQ_MUX;
	config_extint_chan.filter_input_signal = true;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_DOWN;
	config_extint_chan.detection_criteria = EXTINT_DETECT_FALLING;
	extint_chan_set_config(EXT1_IRQ_INPUT, &config_extint_chan);
}
/**
* \brief configure extint callbacks
*
* the Emergency Stop and  limit switch callbacks are attached to the interrupt controller
*/
void configure_extint_callbacks(void)
{
	extint_register_callback(extint_detection_callback,EXT3_IRQ_INPUT,EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(EXT3_IRQ_INPUT,EXTINT_CALLBACK_TYPE_DETECT);
	
	extint_register_callback(extint_detection_callback_emergency,EXT1_IRQ_INPUT,EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(EXT1_IRQ_INPUT,EXTINT_CALLBACK_TYPE_DETECT);
}
/**
* \brief   limit switch callbacks
*
* here the motor is stoped
*/
void extint_detection_callback(void)
{
	port_pin_toggle_output_level(LED_0_PIN);
	//globalLimitSwitchPressed = true; /*!globalLimitSwitchPressed;*/
	if ((positionOfEachOrder[0].order == 1) | (positionOfEachOrder[0].order == 3)) // if motor drive up and hit the limit switch, we calibrate new.
	{
		globalStepPositionAxis = targetH;
	}
	if(((positionOfEachOrder[0].order == 1) | (positionOfEachOrder[0].order == 3))){
		tc_disable(&tc_instance); // pwm aus
		globalDrive = 2;
		port_pin_set_output_level(EXT2_PIN_5,false); //sleep on
	}

}
/**
* \brief emergency stop callbacks
*
* stop all moving
*/
void extint_detection_callback_emergency(void)
{
	port_pin_toggle_output_level(LED_0_PIN);
	globalEmergencyPressed = true;
	globalDrive = 2;
	tc_disable(&tc_instance);
	port_pin_set_output_level(EXT2_PIN_5,false); //sleep on
}

/**
* \brief   delete buffer
*
* \param[in] buffer   Pointer to data buffer to delete
*/
void delete_buffer(uint8_t *buffer)
{
	for(uint8_t i=0; i<sizeof(buffer); ++i)
	{
		buffer[i] = 0;
	}
}

/**
* \brief write all necessary register over SPI
*
* Writes the register to DRV8711,	independently of the length
*
* \param[in] wr_config   Pointer to data buffer to send
* \param[out] rd_config  Pointer to data buffer to receive
* \param[in]  length   Data buffer length
*
*/
void writeToDRV(volatile uint8_t *wr_config, volatile uint8_t *rd_config, uint16_t length){
	for (uint16_t j = 0; j < length; j = j +2){
		wr_buffer [0] = wr_config[j];
		wr_buffer [1] = wr_config[j+1];
		spi_select_slave(&spi_master_instance, &slave, false);			// register schreiben
		spi_transceive_buffer_job(&spi_master_instance, wr_buffer, rd_buffer,sizeof(wr_buffer));
		while (!transrev_complete_spi_master) {
		}
		transrev_complete_spi_master = false;
		spi_select_slave(&spi_master_instance, &slave, true);
		rd_config[j] = rd_buffer[0];
		rd_config[j+1] = rd_buffer[1];
		//delay_ms(1);
	}
}
/**
* \brief add values and text to the write buffer
*
* \param[in] arrayInput   Pointer to write buffer
* \param[out] arrayCopy  Pointer to data to copy
*
*/
void addToWriteBuffer(char *arrayInput, const char *arrayCopy)
{
	uint8_t returnValueInput = strcspn(arrayInput, "\0");
	uint8_t returnValueCopy = strcspn(arrayCopy, "\0");
	memmove(arrayInput + returnValueInput, arrayCopy, returnValueCopy);
}
/**
* \brief deletes a buffer
*
* \param[in] arrayBuffer   Pointer to the array
* \param[in]  length   Data buffer length
*
*/
void deleteBuffer(char *arrayBuffer, uint16_t length){
	for (uint16_t j = 0; j < length; j++)
	{
		arrayBuffer[j]='\0';
	}
}

/**
* \brief calcualete median
*
* \param[in out] valueBuffer   Pointer to buffer to store the result
* \param[in]  length			Data buffer length
*
*/
double calculateMedian(volatile uint16_t *valueBuffer, uint16_t length)
{
	double  maxDigit = pow (2.0, 10.0) - 1.0; // 2^10
	uint16_t i = 0;
	double result = 0.0;
	for (i = 0; i < length / 2; i ++)
	{
		result = result + valueBuffer[i];
	}
	result = result / ((double) i);
	result = maxN / maxDigit * (double) result;
	result = result - maxN / 2.0;
	return (result);
}
/**
* \brief calculates the direction to drive
*
* calculates the direction depending the actual position
*
* \param[in] iActualPos		actual postion
* \param[in]  iTargetPos		target position
* \param[out] targetDirection bool if the direction is up or down
*
*/
volatile bool targetDirection (int iActualPos, int iTargetPos)
{
	bool temp = false;
	if ((iActualPos > iTargetPos) & richtung)
	{
		temp = true;
	}
	else
	{
		temp = false;
	}
	
	if ((iActualPos > iTargetPos) & !richtung)
	{
		temp = false;
	}
	else
	{
		temp = true;
	}
	return(temp);
}
/**
* \brief executes an order
*
* if the RS232 receives an order, this function will start the command
*
*/
double abgleich = 0.0;
void executeOrder(void){
	if ((globalDrive == 0) & (positionOfEachOrder[0].order !=0)) /* if we did not start a command */
	{
		globalDrive = 1; //->motor is moving
		if (abgleich==0)
		{
			abgleich = globalResult;
		}
		
		if (positionOfEachOrder[0].order != 1) /* all other commands */
		{
			globalActualDirection = targetDirection(globalStepPositionAxis, positionOfEachOrder[0].targetPosition);
		}
		else
		{
			globalActualDirection = !port_pin_get_input_level(EXT3_PIN_IRQ); /* if init command */
			if (globalActualDirection)
			{
				globalMovingFreeAxis = 1;
			}
			globalEmergencyPressed = false;
		}
		if(positionOfEachOrder[0].repos)
		{
			globalOrderWasDown = false;
		}
		drv8711enable(true, globalActualDirection);
		delay_ms(1);
	}
}
void reposIfInLimitSwitch(void){
	if (globalDrive == 1 && positionOfEachOrder[0].order == 1 && globalStepPositionAxis > targetFreifahren && globalMovingFreeAxis  == 1)
	{
		globalActualDirection = richtung;
		globalLimitSwitchPressed = false;
		drv8711enable(true, richtung); //freifahren
		globalMovingFreeAxis = 2;
	}
}
/**
* \brief finish the order
*
* finish the order prepares the sending of the respond telegram and stops the movement
*
*/
void finishTheOrder(void){
	if ((globalDrive == 2)&!globalOrderWasDown){
		if (positionOfEachOrder[0].order != 1) /* if there was the Init command */
		{
			globalMovingFreeAxis = 0;
		}
		if(positionOfEachOrder[0].repos)
		{
			globalOrderWasDown = false;
		}
		strcpy(globalRespondTelegram, &positionOfEachOrder[0].respond);
		/* ready for new orders */
		globalDrive = 0;
		positionOfEachOrder[0].order = 0;
		globalSendRespond = true;
		port_pin_set_output_level(EXT2_PIN_5,false); //sleep on
		DRV8711disable();
		delay_ms(10);
	}
}
/**
* \brief send telegram
*
* if the ADC is finished with the conversion, a new telegram is sent
* with the before prepared position, error and status massage
*
*/
void sendTheTelegram(void)
{
	if (adc_read_done & !globalSendStatus){
		uint16_t returnValue = 0;
		
		static char writeBufferToPLC [100] = {0};
		char actualForceValue [18] = {0};
		
		errorHandling(); /*status and error code*/
		
		telegramPreparation(); /* prepare the different telegrams */
		
		calculatePosZaxis(); /* calculate the z axis postion in mm*/
		
		deleteBuffer(actualForceValue, sizeof(actualForceValue));
		
		globalResult =  calculateMedian(adc_result_buffer, sizeof(adc_result_buffer));
		
		if (globalDrive == 1)
		{
			globalResult = globalResult + offsetIfDRVon;
		}
		globalResult = globalResult * scalingFactorForce;

		itoaEigen((int)globalResult, actualForceValue);
		
		/*do the positiong rigth in the Buffer*/
		memmove( actualForceValue + 1,  actualForceValue, sizeof(actualForceValue) - 1);
		actualForceValue[0] = 35; //35 for #
		/*delete buffer*/
		deleteBuffer(writeBufferToPLC, sizeof(writeBufferToPLC));
		/*write the respond in the buffer*/
		addToWriteBuffer(writeBufferToPLC, globalRespondTelegram);
		/*add delimiter in the buffer*/
		addToWriteBuffer(writeBufferToPLC, "/");
		/*write the force value in the buffer*/
		addToWriteBuffer(writeBufferToPLC, actualForceValue);
		/*add delimiter in the buffer*/
		addToWriteBuffer(writeBufferToPLC, "/"); //add delimiter
		/*add the positon from the z axis in the buffer*/
		addToWriteBuffer(writeBufferToPLC,  globalPositionAxisForTelegram );
		/*add delimiter in the buffer*/
		addToWriteBuffer(writeBufferToPLC, "/");
		/*add the error from the drv8711 in the buffer*/
		addToWriteBuffer(writeBufferToPLC, globalErrorForTelegram);
		/* line break and so shit...*/
		addToWriteBuffer(writeBufferToPLC, "\r\n");
		returnValue = strcspn(writeBufferToPLC, "\0");
		globalSendStatus = true;
		usart_write_buffer_job(&usart_instance, (uint8_t *) writeBufferToPLC, returnValue);
		adc_read_done = false;
		globalSendRespond = false;
	}
	if (!adc_read_done)
	{
		adc_read_buffer_job(&adc_instance, (uint16_t *) adc_result_buffer, ADC_SAMPLES);
	}
}
/**
* \brief error handling
*
* reading the status from the DRV8711 and prepare the globalErrorForTelegram buffer with the error status in ASCII
*
*/
void errorHandling(void){

	writeToDRV(wr_drv8711Read, rd_drv8711Read, sizeof(wr_drv8711Read));

	if (((rd_drv8711Read[3] == 255) | (rd_drv8711Read[15] != 0))){  /* if a failure happened, the PLC wants to know it*/
		itoaEigen(rd_drv8711Read[15], globalErrorForTelegram); /* the drv8711 resets itself sometimes, then the torque register is 255, so we get it*/
		addToWriteBuffer(globalErrorForTelegram, "E");
	}
	else
	{
		addToWriteBuffer(globalErrorForTelegram, "0E");
	}
	if(!port_pin_get_input_level(EXT1_PIN_IRQ))
	{
		DRV8711disable();
		deleteBuffer(globalErrorForTelegram, sizeof(globalErrorForTelegram));
		addToWriteBuffer(globalErrorForTelegram, "1E");
		globalMovingFreeAxis = 0;
		globalOrderWasDown = false;
	}
//	deleteBuffer(globalErrorForTelegram, sizeof(globalErrorForTelegram));
//	itoaEigen((int)(abgleich - globalResult), globalErrorForTelegram); /* the drv8711 resets itself sometimes, then the torque register is 255, so we get it*/
//	volatile int temp = abs(abgleich - globalResult);
//	if( (temp < trehsholdNotMovingError) & (globalDrive == 1) & (globalStepPositionAxis < targetForError))
//	{
//		deleteBuffer(globalErrorForTelegram, sizeof(globalErrorForTelegram));
//		addToWriteBuffer(globalErrorForTelegram, "2E");
//	}
	
}
/**
* \brief calculate position into ASCII
*
* reading the position from the motor and prepare the globalPositionAxisForTelegram buffer with the position in ASCII status
*
*/
void calculatePosZaxis(void)
{
	int posZ = (double) globalStepPositionAxis / (double) oneSpin * (double) trapezSteigung;
	itoaEigen(posZ , globalPositionAxisForTelegram);
}
/**
* \brief repositioning when down
*
* if the snapper is down, it is necessary to repos, because otherwise
*the disk will scratch on the repository and at the next cycle the
* possibility is high, that there is a crash
*
*/
void startRepositioningWhenDown(void)
{
	if ((globalDrive == 2)& globalOrderWasDown) //lets start repositioning
	{
		positionOfEachOrder[0].order = 5;
		positionOfEachOrder[0].targetPosition = positionOfEachOrder[5].targetPosition=targetRepositioningAfterDown; /* for repos*/
		positionOfEachOrder[0].order = positionOfEachOrder[5].order = 5;
		strcpy(&positionOfEachOrder[0].receive, &positionOfEachOrder[5].receive);
		strcpy(&positionOfEachOrder[0].respond, &positionOfEachOrder[5].respond);
		globalDrive = 0;
	}
}
/**
* \brief send restart telegram
*
* prepare the globalRespondTelegram buffer with the restart message in ASCII
*
*/
void sendRestartTelegram(void){
	
	if (globalSendRestartTelegram & !globalSendRespond)
	{
		memmove(globalRespondTelegram,  respondStartetUp, sizeof(respondStartetUp));
		globalSendRestartTelegram = false;
		globalSendRespond = true;
	}
}
/**
* \brief delete the respond buffer
*
* deletes the respond buffer afterwards a respond is send
*
*/
void deleteRespondBuffer(void)
{
	if (!globalSendRespond)
	{
		deleteBuffer(globalRespondTelegram, sizeof(globalRespondTelegram));
		addToWriteBuffer(globalRespondTelegram, "00");
	}
}
/**
* \brief send busy telegram
*
* prepare the globalRespondTelegram buffer with the busy message in ASCII
*
*/
void sendBusy(void)
{
	
	if (globalBusy & !globalSendRespond) {// bussy
		memmove(globalRespondTelegram,  respondBussy, sizeof(respondBussy));
		globalSendRespond = true;
		globalBusy = false;
	}
}
/**
* \brief send invalid telegram
*
* prepare the globalRespondTelegram buffer with the invalid message in ASCII
*
*/
void sendInvalidTelegram(void)
{
	if (globalInvalidTelegram & !globalSendRespond)
	{
		memmove(globalRespondTelegram,  respondUnkown, sizeof(respondUnkown));
		globalSendRespond = true;
		globalInvalidTelegram = false;
	}
}
/**
* \brief send OK telegram
*
* prepare the globalRespondTelegram buffer with the OK message in ASCII
*
*/
void sendOK(void)
{
	if ((globalSendOK == 1) & !globalSendRespond){
		memmove(globalRespondTelegram,  respondOK, sizeof(respondOK));
		globalSendRespond = true;
		globalSendOK = 0;
	}
}
/**
* \brief prepares all the telegrams
*
* prepare the globalRespondTelegram buffer with the necessary message in ASCII
*
*/
void telegramPreparation(void)
{
	deleteRespondBuffer();
	sendBusy();
	sendInvalidTelegram();
	sendOK();
	sendRestartTelegram();
}
void initGrabtastic(void)
{
	insertValuesIntoStruct();
	system_init();
	delay_init();
	
	configure_extint_channel();
	configure_extint_channel_emergency();
	configure_extint_callbacks();
	
	configure_adc();
	configure_adc_callbacks();
	
	configure_usart();
	configure_usart_callbacks();
	
	configure_tc();
	configure_tc_callbacks();
	
	system_interrupt_enable_global();
	configure_spi_master();
	configure_spi_master_callbacks();
	configure_port_pins();
}
#endif
