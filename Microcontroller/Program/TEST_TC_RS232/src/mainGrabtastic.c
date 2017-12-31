#include <conf_quick_start_callback.h>
#include <asf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "FHV\config_focus_fhv.h"

/**
 * \brief main programm
 *
 * this program is driving an DRV8711
 * It gets the order from a Beckhoff PLC over RS232
 *
 */
int main(void)
{
	//insertValuesIntoStruct();
	//system_init();
	//delay_init();
	//
	//configure_extint_channel();
	//configure_extint_channel_emergency();
	//configure_extint_callbacks();
	//
	//configure_adc();
	//configure_adc_callbacks();
	//
	//configure_usart();
	//configure_usart_callbacks();
	//
	//configure_tc();
	//configure_tc_callbacks();
	//
	//system_interrupt_enable_global();
	//configure_spi_master();
	//configure_spi_master_callbacks();
	//configure_port_pins();
	 initGrabtastic();

	while (true) {
		
		usart_read_buffer_job(&usart_instance, (uint8_t *) rx_buffer, sizeof(rx_buffer)); /* lets read the RS232 */

		executeOrder(); /* execute the order we get from PLC*/
		
		reposIfInLimitSwitch(); /* if we should init and the limit switch is pressed, repos and drive every time from the same direction to reference*/
		
		finishTheOrder(); /* finish the order on prepare the send status*/

		startRepositioningWhenDown(); /*if z axis reaches down position, it is necesary to move 2mm up, because it scratches on the repository*/
		
		sendTheTelegram(); /* send the telegram */

	}
}
