//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
//
// Copyright (c) 2012-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "TM4C123GH6PM.h"

#include "../include/gpio.h"
#include "../include/uart.h"

#define PA0 (1 << 0)
#define PA1 (1 << 1)
/*
typedef struct rxQ {
	char Q[8];
	bool received;
} rxQ;

typedef struct txQ {
	char Q[8];
	short head, size;
} txQ;
txQ txq;
rxQ rxq;
*/
//*****************************************************************************
// ADD CODE: The mutex that protects concurrent access of UART from multiple tasks.
// Any additional mutex required
//*****************************************************************************
xSemaphoreHandle UART_Rx_lock = 0;
xSemaphoreHandle UART_Tx_lock = 0;
xSemaphoreHandle numActive_lock = 0;
SemaphoreHandle_t Bsem[3] = {0};
//*****************************************************************************
// ADD CODE: Global variables
//*****************************************************************************
uint32_t numWorkers;
uint32_t numActive;
uint32_t linesLeft = 4;
char sentence[4][15];
TaskHandle_t tasks[10];
QueueHandle_t txq, rxq;

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
// This hook is called by FreeRTOS when an stack overflow error is detected.
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
// Configure PA0 and PA1 to be UART pins
//*****************************************************************************
extern void uart0_config_gpio(void)
{
   gpio_enable_port(GPIOA_BASE);
   gpio_config_digital_enable( GPIOA_BASE, PA0 | PA1);
   gpio_config_alternate_function( GPIOA_BASE, PA0 | PA1);
   gpio_config_port_control( GPIOA_BASE, GPIO_PCTL_PA0_U0RX | GPIO_PCTL_PA1_U0TX);
}

extern void wtimer0init(void) {
	SYSCTL->RCGCWTIMER |= SYSCTL_RCGCWTIMER_R0; // set 0th bit
	SYSCTL->RCGCGPIO |= SYSCTL_RCGCGPIO_R2; // GPIO port C
  while( (SYSCTL->PRUART & SYSCTL_PRUART_R0) == 0); //idle wait until GPIO C is up
	GPIOC->PCTL |= GPIO_PCTL_PC4_WT0CCP0;
	GPIOC->PCTL &= (~GPIO_PCTL_PC4_M | GPIO_PCTL_PC4_WT0CCP0);
	// Disable timer
	WTIMER0->CTL &= ~(0x01);
	WTIMER0->CFG = 0x00;
	WTIMER0->TAMR = (1 << 1) | (1 << 4); // count up, periodic
	WTIMER0->TAPR = 0xFFFF; //prescalar
	WTIMER0->TAILR = 0xFFFFFFFF; // what to count up to
	WTIMER0->CTL |= 0x01;
}

extern uint32_t wtimer0Val(void) {
	return WTIMER0->TAV;
}

static void
lab1 (void * pvParameters) {
    // Refer lab document
	  char tmp;
	  long i = 0;
	  const char * slp_cmd = "SLEEP ";
	  bool slp = true;
	  uint32_t slp_time;
	  char *buffer = pvPortMalloc(sizeof(char)*128);
	  char *runstats;
	  char cmd[10];
		while(xSemaphoreTake(numActive_lock, 1000) == pdFALSE);
		numActive++;
		xSemaphoreGive(numActive_lock);
	  while(xSemaphoreTake(UART_Rx_lock, 1000) == pdFALSE);
	  while(uartRxPoll(UART0_BASE, false) != '{');
	  while(tmp != '}' && i < 10){
			tmp = uartRxPoll(UART0_BASE, false);
			if(tmp != 0 && tmp != '}') {
				cmd[i] = tmp;
				++i;
			}
		}
		cmd[i] = '\0';
	  xSemaphoreGive(UART_Rx_lock);
	  // Check if what you read in is a sleep command
		for(i = 0; i < 6; ++i){
			if(slp_cmd[i] != cmd[i])
				slp = false;
		}
	  if(slp){
			slp_time = atoi(&cmd[6]);
			for(i = 0; i < slp_time*200000; ++i) {};
		}
		
		sprintf(buffer, "TIVA >>> Done worker ID: %03d Command: %s\n\r", (int) pvParameters, cmd);
	  while(xSemaphoreTake(UART_Tx_lock, 1000) == pdFALSE);
	  uartTxPoll(UART0_BASE, buffer);
	  xSemaphoreGive(UART_Tx_lock);
		while(xSemaphoreTake(numActive_lock, 1000) == pdFALSE);
		numActive--;
		xSemaphoreGive(numActive_lock);
		if(numActive == 0){
			runstats = pvPortMalloc(sizeof(char)*1024);
			vTaskGetRunTimeStats(runstats);
	    uartTxPoll(UART0_BASE, runstats);
			vPortFree(runstats);
		}
		//vTaskDelete(NULL);
		while(xSemaphoreTake(UART_Tx_lock, 1000) == pdFALSE);
		uartTxPoll(UART0_BASE, "TIVA >>> EOT\n\r");
		xSemaphoreGive(UART_Tx_lock);
		while(1);
}

static void
lab2ex1 (void * pvParameters) {
	  // Refer lab document
	  char *buffer = pvPortMalloc(sizeof(char)*128);
	  char *runstats;
	  long i;
	  // Check if what you read in is a sleep command
	  vTaskDelay(2000);
	  //for(i = 0; i < 400000; ++i) {};
	  if((int) pvParameters > 0)
			while(xSemaphoreTake(Bsem[(int) pvParameters - 1], 1000) == pdFALSE);
		sprintf(buffer, "TIVA >>> %s\n\r", sentence[(int) pvParameters]);
		while(xSemaphoreTake(UART_Tx_lock, 1000) == pdFALSE);
	  uartTxPoll(UART0_BASE, buffer);
	  xSemaphoreGive(UART_Tx_lock);
		if((int) pvParameters < 3)
			xSemaphoreGive(Bsem[(int) pvParameters]);
		vPortFree(buffer);
		while(xSemaphoreTake(numActive_lock, 1000) == pdFALSE);
		linesLeft--;
		xSemaphoreGive(numActive_lock);
		if(linesLeft == 0){
			runstats = pvPortMalloc(sizeof(char)*1024);
			vTaskGetRunTimeStats(runstats);
	    uartTxPoll(UART0_BASE, runstats);
			vPortFree(runstats);
		}
		vTaskDelete(NULL);
		//while(1);
}

static void
lab2ex2 (void * pvParameters) {
	char *buffer = pvPortMalloc(sizeof(char)*128);
	long i;
	//for(i = 0; i < 400000; ++i) {};
	vTaskDelay(10);
	vTaskDelete(NULL);
}

static void
lab2gen (void * pvParameters) {
	char *buffer = pvPortMalloc(sizeof(char)*128);
	long i;
	int j = 0;
	while(1) {
		//for(i = 0; i < 400000; ++i) {};
		vTaskDelay(1000);
		sprintf(buffer, "task%03d", j);
		if(xTaskCreate(lab2ex2, buffer, 128, (void *) j, 1, NULL) != pdPASS){
			sprintf(buffer, "TIVA >>> LAST: %d\n\r", j);
		} else {
			sprintf(buffer, "TIVA >>> PASS: %d\n\r", j);
		}
	  xSemaphoreTake(UART_Tx_lock, 1000);
	  uartTxPoll(UART0_BASE, buffer);
  	xSemaphoreGive(UART_Tx_lock);
		++j;
	}
}
/*
static void
lab2ex5UARTRX (void * pvParameters) {
	char buffer[7];
	short i;
	rxq.received = false;
	while(1) {
		while(uartRxPoll(UART0_BASE, true) != '{');
		for(i = 0; i < 7; ++i){
			buffer[i] = uartRxPoll(UART0_BASE, true);
		}
		sprintf(rxq.Q, "%s", buffer);
		xSemaphoreTake(UART_Rx_lock, 1000);
		rxq.received = true;
		xSemaphoreGive(UART_Rx_lock);
	}
}

static void
lab2ex5UARTTX (void * pvParameters) {
	short i;
	txq.head = 0;
	txq.size = 0;
	while(1) {
		while(xSemaphoreTake(UART_Tx_lock, 1000) == pdFALSE);
		if(txq.size > 0)
			uartTxPollChar(UART0_BASE, txq.Q[txq.head]);
		txq.head = (txq.head + 1) % 7;
		--txq.size;
		xSemaphoreGive(UART_Tx_lock);
	}
}
*/
static void
lab2ex5worker (void * pvParameters) {
	short slp_time;
	short i = 0;
	char *buffer = pvPortMalloc(sizeof(char)*128);
	char *runstats;
	while(1) {
		while(rxq.received == false) {};
		while(xSemaphoreTake(UART_Rx_lock, 1000) == pdFALSE);
	  while(xSemaphoreTake(numActive_lock, 1000) == pdFALSE);
		numActive++;
		xSemaphoreGive(numActive_lock);
		slp_time = atoi(&rxq.Q[6]);
		rxq.received = false;
		xSemaphoreGive(UART_Rx_lock);
		vTaskDelay(slp_time*100);
		
		sprintf(buffer, "TIVA >> Done Worker ID: %d Cmd: SLEEP %d\n\r", (int) pvParameters, slp_time);
		while(buffer[i] != '\0') {
			while(xSemaphoreTake(UART_Tx_lock, 1000) == pdFALSE);
			if(txq.size < 7){
				txq.Q[(txq.head + txq.size) % 7] = buffer[i];
				txq.size++;
				++i;
			}
			xSemaphoreGive(UART_Tx_lock);
		}
		
		while(xSemaphoreTake(numActive_lock, 1000) == pdFALSE);
		numActive--;
		xSemaphoreGive(numActive_lock);
		vPortFree(buffer);
		
		if(numActive == 0){
			runstats = pvPortMalloc(sizeof(char)*1024);
			vTaskGetRunTimeStats(runstats);
			i = 0;
			while(runstats[i] != '\0') {
				while(xSemaphoreTake(UART_Tx_lock, 1000) == pdFALSE);
				if(txq.size < 7){
					txq.Q[(txq.head + txq.size) % 7] = buffer[i];
					txq.size++;
					++i;
				}
				xSemaphoreGive(UART_Tx_lock);
			}
			
			sprintf(runstats, "TIVA >>> EOT\n\r");
			while(runstats[i] != '\0') {
				while(xSemaphoreTake(UART_Tx_lock, 1000) == pdFALSE);
				if(txq.size < 7){
					txq.Q[(txq.head + txq.size) % 7] = buffer[i];
					txq.size++;
					++i;
				}
				xSemaphoreGive(UART_Tx_lock);
			}
		}
	}
}

int 
main(void)
{
	uint32_t input[3];
	uint32_t i;
	char buffer[1024];
	
	sprintf(sentence[0], "The quick");
	sprintf(sentence[1], "brown fox");
	sprintf(sentence[2], "jumps over");
	sprintf(sentence[3], "the lazy dog");
	
	// Initialize UART
	uart0_config_gpio();
	uart_init_115K(UART0_BASE);
	UART_Rx_lock = xSemaphoreCreateMutex();
	UART_Tx_lock = xSemaphoreCreateMutex();
	numActive_lock = xSemaphoreCreateMutex();
	for(i = 0; i < 3; ++i) {
	  vSemaphoreCreateBinary(Bsem[i]);
    xSemaphoreTake(Bsem[i], 0);
	}
	numActive = 0;
  // Display initialization
	while(uartRxPoll(UART0_BASE, true) != 0x39){};
  uartTxPoll(UART0_BASE, "TIVA >>> Maggie White\n\r");

	// Get # of worker threads
	input[2] = uartRxPoll(UART0_BASE, true);
	input[1] = uartRxPoll(UART0_BASE, true);
	input[0] = uartRxPoll(UART0_BASE, true);
	xSemaphoreGive(UART_Rx_lock);
	numWorkers = atoi((char *) input);

	// Create worker threads
	for(i = 0; i < numWorkers; i++) {
		snprintf(buffer, sizeof(buffer), "task%03d", i);
		xTaskCreate(lab2ex1, buffer, 128, (void *) i, 4-i, tasks[i]);
	}
	sprintf(buffer, "TIVA >>> Running with %03d tasks\n\r", numWorkers);
	uartTxPoll(UART0_BASE, buffer);
	//xTaskCreate(lab2gen, "gentask", 128, 0, 1, NULL);
	vTaskStartScheduler();
  while(1)
  {
     // Infinite Loop
  }
}
