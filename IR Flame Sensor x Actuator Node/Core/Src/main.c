/**
 * @file	main.c
 * @brief	EEE192 IR Sensor Node, AAA Actuator Node
 *
 * @author Jenard Ayan Ascano <jascano@up.edu.ph>,
 * credits goes to the following:
 * @Alberto de Villa <abdevilla@up.edu.ph>, initially by
 * 		Philip Martinez <philip.martinez@eee.upd.edu.ph>
 * 		- for usart fifo setup and initialization
 * @date	17 Apr 2024
 * @copyright
 *
 * Copyright (C) 2022-2023. This source code was created as part of the authors'
 * official duties with the Electrical and Electronics Engineering Institute,
 * University of the Philippines <https://eee.upd.edu.ph>
 *
 * Disclaimer: This source code was created as a partial fulfillment of the
 * requirements for EEE192. The credits for codes responsible for USART
 * handling goes to the rightful owner(s) as mentioned above (Obtained via
 * EEE158).
 *
 *
 */

/*
 * System configuration/build:
 * 	- Clock source == HSI (~16 MHz)
 * 		- No AHB & APB1/2 prescaling
 *	- Inputs:
 * 		- Active-LO NO-wired pushbutton @ PC13
 * 		- USART Input @ PA3 (USART2_RX)
 * 	- Outputs:
 * 		- Active-HI LED @ PA5
 *		- USART Output @ PA2 (USART2_TX)
 *
 */

#include "usart.h"
#include <stdint.h>	// C standard header; contains uint32_t, for example
#include <stdbool.h>// C99 bool
#include <stm32f4xx.h>	// Header for the specific device family

#include <stdio.h>	// Needed for snprintf()
#include <string.h>	// Needed for memorsemp()
#include <stdlib.h> // added for atoi()

////////////////////////////////////////////////////////////////////////////

/*
 * IRQ data shared between the handlers and main()
 *
 * As asynchronous access is possible, all members are declared volatile.
 */
volatile struct
{
	/*
	 * If set, the button has been pressed.
	 *
	 * This should be cleared in main().
	 */
	unsigned int pressed;

	/*
	 * Number of system ticks elapsed
	 */
	unsigned int nr_tick;

} irq_data;

/*
 * Handler for the interrupt
 *
 * This function name is special -- this name is used by the startup code (*.s)
 * to indicate the handler for this interrupt vector.
 */


// Handler for the system tick
void SysTick_Handler(void)
{
	irq_data.nr_tick += 1;
	SysTick->VAL = 0;
}

void GPIO_init(void) {
	//enable GPIOA clock
	RCC->AHB1ENR |= (1 << 0);

	//enable GPIOC clock
	RCC->AHB1ENR |= (1 << 2);

	//configure PA6 to analog mode
	GPIOA->MODER |= (1 << 13);
	GPIOA->MODER |= (1 << 12);

	//configure PA7 to analog mode
	GPIOA->MODER |= (1 << 15);
	GPIOA->MODER |= (1 << 14);

	//configure GPIO pins for actuator node
	GPIOC->MODER &= ~(1 << 5); // buzzer level 3
	GPIOC->MODER |= (1 << 4);

	GPIOC->MODER &= ~(1 << 7); // red light level 3
	GPIOC->MODER |= (1 << 6);

	GPIOC->MODER &= ~(1 << 21); // green light level 1
	GPIOC->MODER |= (1 << 20);

	GPIOC->MODER &= ~(1 << 1); // yellow light level 2
	GPIOC->MODER |= (1 << 0);

	GPIOC->MODER &= ~(1 << 23); // fan level 2
	GPIOC->MODER |= (1 << 22);

	GPIOC->ODR |= (1 << 11);
	GPIOC->ODR &= ~(1 << 2);
	GPIOC->ODR &= ~(1 << 3);
	GPIOC->ODR &= ~(1 << 10);
	GPIOC->ODR &= ~(1 << 0);

}

void ADC_init(void) {
	//enable adc clock
	RCC->APB2ENR |= (1 << 8);

	//prescaler = 2
	ADC->CCR &= ~(1 << 16);
	ADC->CCR &= ~(1 << 17);

	//configure ADC resolution
	ADC1->CR1 &= ~(1 << 25);
	ADC1->CR1 &= ~(1 << 24);

	//Configure to Scan mode
	ADC1->CR1 |= (1 << 8);

	//Enable Interrupt for EOC
	ADC1->CR1 |= (1 << 5);

	//configure sampling time
	ADC1->SMPR2 &= ~(1 << 5);
	ADC1->SMPR2 &= ~(1 << 4);
	ADC1->SMPR2 |= (1 << 3);

	//end of conversion selection
	ADC1->CR2 &= ~(1 << 10);

	//configure data alignment
	ADC1->CR2 &= ~(1 << 11);

	//total number of conversions in the channel conversion sequence (two conversions)
	ADC1->SQR1 &= ~(1 << 23);
	ADC1->SQR1 &= ~(1 << 22);
	ADC1->SQR1 &= ~(1 << 21);
	ADC1->SQR1 &= ~(0 << 20);

	//cont conversion mode
	ADC1->CR2 |= (1 << 1);
}

void delay_ms(int delay) {
	int i;
	for (; delay > 0; delay--) {
		for (i = 0; i < 2657; i++)
			;
	}
}

void ADC_enable(void) {
	ADC1->CR2 |= (1 << 0); //enable the adc
	delay_ms(1); //required to ensure adc stable
}

void ADC_startconv(int channel) {
	ADC1->SQR3 = 0;
	ADC1->SQR3 |= (channel<< 0); //conversion in regular sequence

	ADC1->SR = 0; //clears the status register

	ADC1->CR2 |= (1 << 30); //starts the conversion
}

void ADC_waitconv(void) {
	//wait for the end of conversion
	while (!((ADC1->SR) & (1 << 1))) {
		;
	}
}

int ADC_GetVal(void) {
	return ADC1->DR; //read the value contained at the data register
}
////////////////////////////////////////////////////////////////////////////

// Function to initialize the system; called only once on device reset
static void do_sys_config(void)
{ // this function is copied from EEE158 materials.
	//credits to the rightful owner(s)
	// See the top of this file for assumptions used in this initialization.


	////////////////////////////////////////////////////////////////////

	RCC->AHB1ENR |= (1 << 0);	// Enable GPIOA

	GPIOA->MODER &= ~(0b11 << 10);	// Set PA5 as input...
	GPIOA->MODER |=  (0b10 << 10);	// ... then set it as alternate function.
	GPIOA->OTYPER &= ~(1 << 5);	// PA5 = push-pull output
	GPIOA->OSPEEDR &= ~(0b11 << 10);	// Fast mode (needed for PWM)
	GPIOA->OSPEEDR |=  (0b10 << 10);

	/*
	 * For PA5 -- where the LED on the Nucleo board is -- only TIM2_CH1
	 * is PWM-capable, per the *device* datasheet. This corresponds to
	 * AF01.
	 */
	GPIOA->AFR[0] &= ~(0x00F00000);	// TIM2_CH1 on PA5 is AF01
	GPIOA->AFR[0] |=  (0x00100000);

	////////////////////////////////////////////////////////////////////

	/*
	 * In this setup, TIM2 is used for brightness control (see reason
	 * above), while TIM1 is used to do input-capture on PA9 via
	 * Channel 2.
	 *
	 * The internal clock for TIM2 is the same as the APB1 clock;
	 * in turn, this clock is assumed the same as the system (AHB) clock
	 * without prescaling, which results to the same frequency as HSI
	 * (~16 MHz).
	 *
	 * TIM1, on the other hand, uses the APB2 clock; in turn, no prescaling
	 * is done here, which also yields the same value as HSI (~16 MHz).
	 */
	RCC->APB1ENR	|= (1 << 0);	// Enable TIM2
	RCC->APB2ENR	|= (1 << 0);	// Enable TIM1

	/*
	 * Classic PWM corresponds to the following:
	 * 	- Edge-aligned	(CMS  = CR1[6:5] = 0b00)
	 * 	- Upcounting	(DIR  = CR1[4:4] = 0)
	 *	- Repetitive	(OPM  = CR1[3:3] = 0)
	 * 	- PWM Mode #1	(CCxS[1:0] = 0b00, OCMx = 0b110)
	 * 		- These are in CCMRy; y=1 for CH1 & CH2, and y=2 for
	 * 		  CH3 & CH4.
	 */
	TIM2->CR1 &= ~(0b1111 << 0);
	TIM2->CR1 &= ~(1 << 0);		// Make sure the timer is off
	TIM2->CR1 |=  (1 << 7);		// Preload ARR (required for PWM)
	TIM2->CCMR1 = 0x0068;		// Channel 1 (TIM2_CH1)

	/*
	 * Per the Nyquist sampling theorem (from EEE 147), to appear
	 * continuous the PWM frequency must be more than twice the sampling
	 * frequency. The human eye (the sampler) can usually go up to 24Hz;
	 * some, up to 40-60 Hz. To cover all bases, let's use 500 Hz.
	 *
	 * In PWM mode, the effective frequency changes to
	 *
	 * (f_clk) / (ARR*(PSC+1))
	 *
	 * since each period must be able to span all values on the interval
	 * [0, ARR). For obvious reasons, ARR must be at least equal to one.
	 */
	TIM2->ARR	= 100;		// Integer percentages; interval = [0,100]
	TIM2->PSC	= (320 - 1);	// (16MHz) / (ARR*(TIM2_PSC + 1)) = 500 Hz

	// Let main() set the duty cycle. We initialize at zero.
	TIM2->CCR1	= 0;

	/*
	 * The LED is active-HI; thus, its polarity bit must be cleared. Also,
	 * the OCEN bit must be enabled to actually output the PWM signal onto
	 * the port pin.
	 */
	TIM2->CCER	= 0x0001;

	////////////////////////////////////////////////////////////////////

	// Pushbutton configuration
	RCC->AHB1ENR |= (1 << 2);	// Enable GPIOC

	GPIOC->MODER &= ~(0b11 << 26);	// Set PC13 as input...
	GPIOC->PUPDR &= ~(0b11 << 26);	// ... without any pull-up/pull-down (provided externally).

	////////////////////////////////////////////////////////////////////

	/*
	 * Enable the system-configuration controller. If disabled, interrupt
	 * settings cannot be configured.
	 */
	RCC->APB2ENR |= (1 << 14);

	/*
	 * SYSCFG_EXTICR is a set of 4 registers, each controlling 4 external-
	 * interrupt lines. Within each register, 4 bits are used to select
	 * the source connected to each line:
	 * 	0b0000 = Port A
	 * 	0b0001 = Port B
	 * 	...
	 * 	0b0111 = Port H
	 *
	 * For the first EXTICR register:
	 *	Bits 0-3   correspond to Line 0;
	 * 	Bits 4-7   correspond to Line 1;
	 *	Bits 8-11  correspond to Line 2; and
	 * 	Bits 12-15 correspond to Line 3.
	 *
	 * The 2nd EXTICR is for Lines 4-7; and so on. Also, the line numbers
	 * map 1-to-1 to the corresponding bit in each port. Thus, for example,
	 * a setting of EXTICR2[11:8] == 0b0011 causes PD6 to be tied to
	 * Line 6.
	 *
	 * For this system, PC13 would end up on Line 13; thus, the
	 * corresponding setting is EXTICR4[7:4] == 0b0010. Before we set it,
	 * mask the corresponding interrupt line.
	 */
	EXTI->IMR &= ~(1 << 13);		// Mask the interrupt
	SYSCFG->EXTICR[3] &= (0b1111 << 4);	// Select PC13 for Line 13
	SYSCFG->EXTICR[3] |= (0b0010 << 4);

	/*
	 * Per the hardware configuration, pressing the button causes a
	 * falling-edge event to be triggered, and a rising-edge on release.
	 * Since we are only concerned with presses, just don't trigger on
	 * releases.
	 */
	EXTI->RTSR &= ~(1 << 13);
	EXTI->FTSR |=  (1 << 13);

	/*
	 * Nothing more from the SCC, disable it to prevent accidental
	 * remapping of the interrupt lines.
	 */
	RCC->APB2ENR &= ~(1 << 14);

	////////////////////////////////////////////////////////////////////

	/*
	 *
	 * Per the STM32F411xC/E family datasheet, the handler for EXTI_15_10
	 * is at 40.
	 *
	 * Per the STM32F4 architecture datasheet, the NVIC IP registers
	 * each contain 4 interrupt indices; 0-3 for IPR[0], 4-7 for IPR[1],
	 * and so on. Each index has 8 bits denoting the priority in the top
	 * 4 bits; lower number = higher priority.
	 *
	 * Position 40 in the NVIC table would be at IPR[10][7:0]; or,
	 * alternatively, just IP[40].
	 */
	NVIC->IP[40] = (1 << 4);
	NVIC->IP[6]  = (0b1111 << 4);	// SysTick; make it least-priority

	/*
	 * Per the STM32F4 architecture datasheet, the NVIC ISER/ICER registers
	 * each contain 32 interrupt indices; 0-31 for I{S/C}ER[0], 32-63 for
	 * I{S/C}ER[1], and so on -- one bit per line.
	 *
	 * ISER is written '1' to enable a particular interrupt, while ICER is
	 * written '1' to disable the same interrupt. Writing '0' has no
	 * effect.
	 *
	 * Position 25 in the NVIC table would be at I{S/C}ER[0][25:25]; while
	 * position 27 would be at I{S/C}ER[0][27:27].
	 */
	NVIC->ISER[0] = (1 << 6);	// Note: Writing '0' is a no-op
	NVIC->ISER[1] = (1 << 8);	// Note: Writing '0' is a no-op
	EXTI->IMR |= (1 << 13);		// Unmask the interrupt on Line 13
	TIM2->EGR |= (1 << 0);		// Trigger an update on TIM2
	TIM2->CR1 |= (1 << 0);		// Activate both timers
	irq_data.pressed = 0;

	/*
	 * Enable tick counting; the idea is to allow main() to perform
	 * periodic tasks.
	 */
	SysTick->LOAD = (20000-1);	// Target is 100 Hz with 2MHz clock
	SysTick->VAL  = 0;
	SysTick->CTRL &= ~(1 << 2);	// Clock base = 16MHz / 8 = 2MHz
	SysTick->CTRL &= ~(1 << 16);
	SysTick->CTRL |= (0b11 << 0);	// Enable the tick

	// Do the initialization of USART last.
	usart1_init();
	GPIO_init();
	ADC_init();

}


int find_str(char *find){
	/*
	 * Function to process the esp string responses upon esp initialization
	 * Used to handle simple responses upon setup
	 * inspired from the EEE158 usart rx handling
	 */
	struct usart_rx_event usart_evt1;
	unsigned int is_found = 0;
	char		rxb1_data[128];
	unsigned int	rxb1_idx  = 0;
	unsigned int	rxb1_size = 0;
	bool p;
	while (is_found == 0){
		do { // copied from the template
			if (!usart1_rx_get_event(&usart_evt1))
				// Nothing to do here
				break;
			else if (!usart_evt1.valid)
				break;

			if (usart_evt1.is_idle) {
				rxb1_size = rxb1_idx;
				break;
			} else if (!usart_evt1.has_data) {
				break;
			}

					// Store the data
			if (rxb1_idx >= sizeof(rxb1_data)) {
				rxb1_size = rxb1_idx;
				break;
			}
			rxb1_data[rxb1_idx++] = usart_evt1.c;
			break;
		} while (0);
		//} TODO: string handling received from esp-01

		if (rxb1_size > 0){
			for(int i = 0; i < 127; i++)	// Clear unused chars
			{
				if(i > rxb1_size)
				{
					rxb1_data[i] = '0';
				}
			}
			rxb1_size = rxb1_idx = 0;

			p = strstr(rxb1_data, find);
		}
		if (p){
			is_found = 1;
			memset(rxb1_size, '\0', sizeof(rxb1_size));
			return is_found;
		}
	}
}

void ESP_init(char *SSID, char *PASSWD){
	/*
	 * used to setup the esp with AT commands as seen below.
	 * process as follows:
	 * resets the wifi module
	 * test if it is ready to receive AT commands
	 * disconnect from the previous access point to ensure reset
	 * set to station mode and connect to the target access point
	 *
	 */
	char data[80];
	static const char str_a[] = "AT+RST\r\n";
	static const char str_b[] = "AT\r\n";
	static const char str_c[] = "AT+CWMODE=1\r\n";
	static const char str_d[] = "AT+CIPMUX=0\r\n";
	static const char str_e[] = "AT+CWQAP\r\n";

	usart1_tx_send(str_a, strlen(str_a));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);


	usart1_tx_send(str_b, strlen(str_b));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	while(!find_str("ready\r\n"));

	usart1_tx_send(str_e, strlen(str_e));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	while(!find_str("OK\r\n"));


	usart1_tx_send(str_c, strlen(str_c));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	while(!find_str("OK\r\n"));

	sprintf (data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	usart1_tx_send(data, strlen(data));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	while(!find_str("WIFI GOT IP"));

	usart1_tx_send(str_d, strlen(str_d));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	while(!find_str("OK\r\n"));

}
/////////////////////////////////////////////////////////////////////////////
int find_str2(void){
	/*
	 * Used for ESP_receive function. Modified version of find_str
	 * that returns the last field value obtained after HTTP GET request
	 */
	struct usart_rx_event usart_evt2;
	unsigned int is_found = 0;
	char		rxb2_data[200];
	unsigned int	rxb2_idx  = 0;
	unsigned int	rxb2_size = 0;
	while (1){
		do { // copied from the template
			if (!usart1_rx_get_event(&usart_evt2)){
				// Nothing to do here
				char* fieldValueStr = strstr(rxb2_data, "+IPD");
				if (fieldValueStr != NULL) {
					fieldValueStr = strstr(fieldValueStr, ":");
					if (fieldValueStr != NULL){
						fieldValueStr += 1;
						int fieldValue = atoi(fieldValueStr);
						return fieldValue;
					}
				}
				break;
			}
			else if (!usart_evt2.valid)
				break;

			if (usart_evt2.is_idle) {
				rxb2_size = rxb2_idx;
				break;

			} else if (!usart_evt2.has_data) {
				break;
			}

					// Store the data
			if (rxb2_idx >= sizeof(rxb2_data)) {
				rxb2_size = rxb2_idx;
				break;
			}
			rxb2_data[rxb2_idx++] = usart_evt2.c;
			break;
		} while (0);
		//} TODO: string handling received from esp-01
		if (rxb2_size > 0){
			for(int i = 0; i < 199; i++)	// Clear unused chars
			{
				if(i > rxb2_size)
				{
					rxb2_data[i] = '\0';
				}
			}
			rxb2_size = rxb2_idx = 0;
			char* fieldValueStr = strstr(rxb2_data, "+IPD");
			if (fieldValueStr != NULL) {
				fieldValueStr = strstr(fieldValueStr, ":");
				if (fieldValueStr != NULL){
					fieldValueStr += 1;
					int fieldValue = atoi(fieldValueStr);
					return fieldValue;
				}
			}



		}
	}
}

void ESP_Send_Multi(char *APIkey, int numberoffileds, uint16_t value[]){
	/*
	 * uses fire and forget mechanism to avoid infinite loop when data loss
	 * is encountered.
	 * sends the sensor values to thingspeak channel fields via HTTP GET request
	 */
	char local_buf[500] = {0};
	char local_buf2[30] = {0};
	char field_buf[200] = {0};

	static const char str_g[] = "AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n";

	sprintf (local_buf, "GET /update?api_key=%s", APIkey);
	for (int i=0; i<numberoffileds; i++)
	{
		sprintf(field_buf, "&field%d=%u",i+1, value[i]);
		strcat (local_buf, field_buf);
	}

	strcat(local_buf, "\r\n");
	int len = strlen (local_buf);

	usart1_tx_send(str_g, strlen(str_g));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	usart1_tx_send(local_buf2, strlen(local_buf2));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	usart1_tx_send(local_buf, strlen(local_buf));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);


	memset(local_buf, '\0', sizeof(local_buf));
	memset(local_buf2, '\0', sizeof(local_buf2));
}

int ESP_Receive1(void){
	/*
	 *  for actuator node. may encounter faults and data loss
	 */
	char local_buf2[30] = {0};
	char field_buf[200] = {0};

	static const char str_h[] = "AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n";
	static const char str_i[] = "GET /channels/2532565/fields/1/last.txt?api_key=A2P6YCYLDQ6NYKLL\r\n";

	int len = strlen (str_i);

	usart1_tx_send(str_h, strlen(str_h));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	usart1_tx_send(local_buf2, strlen(local_buf2));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	usart1_tx_send(str_i, strlen(str_i));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);

	int AlertLevel = find_str2();

	memset(local_buf2, '\0', sizeof(local_buf2));
	return AlertLevel;
}

int ESP_Receive2(void){
	char local_buf2[30] = {0};
	char field_buf[200] = {0};

	static const char str_h[] = "AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n";
	static const char str_i[] = "GET /channels/2532565/fields/2/last.txt?api_key=A2P6YCYLDQ6NYKLL\r\n";
// "GET /channels/2571239/fields/1/last.txt?api_key=WNOQN9BTIAVZD9B9\r\n";
	int len = strlen (str_i);

	usart1_tx_send(str_h, strlen(str_h));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	usart1_tx_send(local_buf2, strlen(local_buf2));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	usart1_tx_send(str_i, strlen(str_i));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);

	int AlertLevel = find_str2();

	memset(local_buf2, '\0', sizeof(local_buf2));
	return AlertLevel;
}

int ESP_Receive3(void){
	char local_buf2[30] = {0};
	char field_buf[200] = {0};

	static const char str_h[] = "AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n";
	static const char str_i[] = "GET /channels/2571239/fields/1/last.txt?api_key=WNOQN9BTIAVZD9B9\r\n";


	int len = strlen (str_i);

	usart1_tx_send(str_h, strlen(str_h));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	usart1_tx_send(local_buf2, strlen(local_buf2));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	usart1_tx_send(str_i, strlen(str_i));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);

	int AlertLevel = find_str2();

	memset(local_buf2, '\0', sizeof(local_buf2));
	return AlertLevel;
}

int ESP_Receive4(void){
	char local_buf2[30] = {0};
	char field_buf[200] = {0};

	static const char str_h[] = "AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n";
	static const char str_i[] = "GET /channels/2571239/fields/2/last.txt?api_key=WNOQN9BTIAVZD9B9\r\n";


	int len = strlen (str_i);

	usart1_tx_send(str_h, strlen(str_h));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	usart1_tx_send(local_buf2, strlen(local_buf2));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	usart1_tx_send(str_i, strlen(str_i));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);

	int AlertLevel = find_str2();

	memset(local_buf2, '\0', sizeof(local_buf2));
	return AlertLevel;
}

int ESP_Receive5(void){
	char local_buf2[30] = {0};
	char field_buf[200] = {0};

	static const char str_h[] = "AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n";
	static const char str_i[] = "GET /channels/2522847/fields/1/last.txt?api_key=T8KS7ALNMSLB40PT\r\n";


	int len = strlen (str_i);

	usart1_tx_send(str_h, strlen(str_h));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	usart1_tx_send(local_buf2, strlen(local_buf2));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	usart1_tx_send(str_i, strlen(str_i));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);

	int AlertLevel = find_str2();

	memset(local_buf2, '\0', sizeof(local_buf2));
	return AlertLevel;
}

int ESP_Receive6(void){
	char local_buf2[30] = {0};
	char field_buf[200] = {0};

	static const char str_h[] = "AT+CIPSTART=\"TCP\",\"184.106.153.149\",80\r\n";
	static const char str_i[] = "GET /channels/2522847/fields/2/last.txt?api_key=T8KS7ALNMSLB40PT\r\n";


	int len = strlen (str_i);

	usart1_tx_send(str_h, strlen(str_h));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	sprintf (local_buf2, "AT+CIPSEND=%d\r\n", len);
	usart1_tx_send(local_buf2, strlen(local_buf2));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);
	usart1_tx_send(str_i, strlen(str_i));
	while (usart1_tx_is_busy());
	irq_data.nr_tick = 0;
	while(irq_data.nr_tick < 100);

	int AlertLevel = find_str2();

	memset(local_buf2, '\0', sizeof(local_buf2));
	return AlertLevel;
}


void ESP_Send(void){
	uint16_t ir_1 = 0x0000;
	uint16_t ir_2 = 0x0000;
	uint16_t Value_Buf [2];
	ADC_enable();

					//start conversion for channel 1
	ADC_startconv(6);

			//wait for conversion to finish
	ADC_waitconv();

					//store converted data to a variable
	ir_1 = ADC_GetVal();

			/////////////////////////////////////////////////////////////
			//start conversion for channel 4
	ADC_startconv(7);

					//wait for conversion to finish
	ADC_waitconv();

					//store converted data to a variable
	ir_2 = ADC_GetVal();
	Value_Buf[0] = ir_1;
	Value_Buf[1] = ir_2;

	ESP_Send_Multi("0EVL9R43BFDYXH93", 2, Value_Buf);

}

int AAA(int* temp, int* humid, int* ir1,
		int* ir2, int* smoke, int* co, int* lvl2cnt){
	int exceededThresholdCount = 0;
	int level2count = lvl2cnt;
	//temp node check
	if (temp > 32|| humid < 70){
		exceededThresholdCount ++;
	}
	//ir node check
	if (ir1 < 2800 || ir2 < 2800){ //test
		exceededThresholdCount ++;
	}
	// mq check
	if (smoke > 200|| co > 30){
		exceededThresholdCount ++;
	}
	if (exceededThresholdCount == 1){ // level 1
		GPIOC->ODR &= ~(1 << 0);
		GPIOC->ODR |= (1 << 11);
		GPIOC->ODR &= ~(1 << 2);
		GPIOC->ODR &= ~(1 << 3);
		GPIOC->ODR |= (1 << 10);
	}
	else if (exceededThresholdCount >= 2){ // first level 2
		GPIOC->ODR &= ~(1 << 2);
		GPIOC->ODR &= ~(1 << 3);
		GPIOC->ODR &= ~(1 << 10);
		GPIOC->ODR |= (1 << 0);
		GPIOC->ODR &= ~(1 << 11);
		level2count ++;
		if (level2count >= 2){ // level 3
			GPIOC->ODR &= ~(1 << 10);
			GPIOC->ODR &= ~(1 << 0);
			GPIOC->ODR |= (1 << 11);
			GPIOC->ODR |= (1 << 2);
			GPIOC->ODR |= (1 << 3);
		}

	}
	else {
		level2count = 0; // normal
		GPIOC->ODR &= ~(1 << 0);
		GPIOC->ODR |= (1 << 11);
		GPIOC->ODR &= ~(1 << 2);
		GPIOC->ODR &= ~(1 << 3);
		GPIOC->ODR &= ~(1 << 10);
	}
	return level2count;
}



// The heart of the program
int main(void)
{

	uint16_t temp = 0;
	uint16_t humid = 0;
	uint16_t ir1 = 0;
	uint16_t ir2 = 0;
	uint16_t smoke = 0;
	uint16_t co = 0;
	int level2count = 0;
	unsigned int nr_tick = 0;
	// TODO: Declare any additional variable(s) after this line


	// TODO:

	// Configure the system
	do_sys_config();
	ESP_init("EEE192-429", "EEE192_Room429");
	// Send out the initial message
	/*
	 * Microcontroller main()'s are expected to never return; hence, the
	 * infinite loop.
	 */

	int loop = 0;
	for (;;) { // crucial

		/////////////////////////////////////////////////////////////

		ESP_Send();

		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		temp = ESP_Receive1();
		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		ESP_Send();
		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		humid = ESP_Receive2();
		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		ESP_Send();
		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		ir1 = ESP_Receive3();
		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		ESP_Send();
		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		ir2 = ESP_Receive4();
		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		ESP_Send();
		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		smoke = ESP_Receive5();
		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		ESP_Send();
		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		co = ESP_Receive6();
		irq_data.nr_tick = 0;
		while(irq_data.nr_tick < 200);

		level2count = AAA(temp, humid, ir1, ir2, smoke, co, level2count);

		loop ++;
		/////////////////////////////////////////////////////////////

		/////////////////////////////////////////////////////////////

		/*
		 * Handle periodic ticks here
		 *
		 * Based on the configuration for SysTick in do_sys_config(),
		 * ticks occur at a nominal rate of 100 Hz. irq_data.nr_tick
		 * can be greater than one, in case some events were missed.
		 */
		if ((nr_tick = irq_data.nr_tick) > 0) {
			irq_data.nr_tick = 0;
			// implemented the LED routine outside this template
			// I'm not quite sure how to use this if the encoding is not periodic in a sense
			/*
			 * TODO: Any tasks periodically done nominally every
			 *       10 ms (= 1/100Hz) should be added here.
			 *
			 *       IF any ticks have been missed, nr_tick > 1.
			 */

		}
	}

	// This line is supposed to never be reached.
	return 1;
}

