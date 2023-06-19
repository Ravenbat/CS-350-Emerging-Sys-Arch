/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */



#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
/* Timer header files */
#include <ti/drivers/Timer.h>
#include <unistd.h>

/* UART2 driver files */
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/dpl/SemaphoreP.h>
#include <ti/drivers/power/PowerCC32XX.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/dma/UDMACC32XX.h>
#include <ti/drivers/Power.h>

/* GPIO driver files */
#include <ti/devices/cc32xx/inc/hw_types.h>
#include <ti/devices/cc32xx/driverlib/gpio.h>
#include <ti/devices/cc32xx/driverlib/pin.h>

/* I2C driver files */
#include <ti/drivers/I2C.h>
#include <ti/drivers/Power.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
size_t bytesWritten;

#define DISPLAY(x)UART2_write(uart, &output, x, &bytesWritten);


typedef struct task {          //Task scheduler
  unsigned long period; // Rate at which the task should tick
  unsigned long elapsedTime; // Time since task's previous tick
  int (*TickFct)(int); // Function to call for task's tick
} task;

task tasks[3];

const unsigned char tasksNum = 3;
const unsigned long tasksCheckButton = 200;
const unsigned long periodCheckTemp = 500;
const unsigned long periodUpdateLEDs = 1000;
int16_t temperature = 0;

enum TH_States { TH_TSStart, TL_CheckTemp, TL_ButtonPress, TL_UpdateLED };


// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}
sensors[3] = {
              { 0x48, 0x0000, "11X" },
              { 0x49, 0x0000, "116" },
              { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// General Global Variables
int16_t setPoint = 31;       //set point
int button_pressed = 0;




// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
Timer_Handle timer0;
I2C_Handle i2c;
UART2_Handle uart;


volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void initUART(void){
    UART2_Params uartParams;
    // Init the driver
    //UART2_init();
    // Configure the driver
    UART2_Params_init(&uartParams);
    /*uartParams.writeDataMode = UART_DATA_BINARY;         //UART2 doesn't have these variables? UART_DATA_BINARY
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART2*/
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

void initTimer(void){
    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}




// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - " ));


    // Init the driver
    I2C_init();


    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;


    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL){
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
                while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */


    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;


    found = 0;
    for (i=0; i<3; ++i){
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
    if (I2C_transfer(i2c, &i2cTransaction)){
        DISPLAY(snprintf(output, 64, "Found\n\r"))
                found = true;
        break;
    }
    DISPLAY(snprintf(output, 64, "No\n\r"))
    }
    if(found){
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address:%x\n\r", sensors[i].id, i2cTransaction.targetAddress))
    }

    else{
        DISPLAY(snprintf(output, 64, "Temperature sensor not found,contact professor\n\r"))
    }
}

int16_t readTemp(void){
    int j;
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        Extract degrees C from the received data;
         see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */

        if (rxBuffer[0] & 0x80){
        temperature |= 0xF000;
        }

    }
    else{
    DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))

    DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;

}

void updateSetPoint(void){
    switch (button_pressed){
    case 1:
        setPoint++;
        break;
    case -1:
        setPoint--;
        break;
    }
    button_pressed = 0;
}

void updateLED(void){
    int16_t temp = readTemp();       //getting temperature from readTemp function

    if(temp >= setPoint){
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    }

    else {
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    }



}


/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)               //Tried to get both buttons to do the switch but one is not working for some reason.
{
    //Change button pressed to 1
        button_pressed = -1;
        printf("SetPoint decreased by 1 degree.");    // This function is intended to set global variable to -1 to let updateSetPoint know specific button
}                                                    //pressed to increase or decrease it.

void gpioButtonFxn0(uint_least8_t index)
{
     //Change button pressed to 1
    button_pressed = 1;
    printf("SetPoint increased by 1 degree.");
}

void timer1Callback(Timer_Handle myHandle, int_fast16_t status)            //Timer 1 belongs to detecting temperature every 500ms, being called every 500ms
{                                                                         //
    temperature = readTemp();

    // Restart the timer
    Timer_start(myHandle);
}

void timer2Callback(Timer_Handle myHandle, int_fast16_t status)            //Timer 2 belongs to detecting button every 200ms, being called every 200ms
{                                                                         //
                                                                          //
    updateSetPoint();

    // Restart the timer
    Timer_start(myHandle);
}

void timer3Callback(Timer_Handle myHandle, int_fast16_t status)            //Timer 3 works to update the LED and report to the server every 1 second/1000ms
{                                                                         //
    //Updating the LED
    updateLED();

    // Restart the timer
    Timer_start(myHandle);
}




/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING | GPIO_CFG_IN_INT_HIGH);*/


    /* Turn off user LEDs */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);


    initUART();
    initI2C();
    initTimer();

    //Thermometer and Task Scheduler attempt #45

    unsigned char i=0;
    tasks[i].period = 500;
    tasks[i].elapsedTime = tasks[i].period;

    ++i;

    tasks[i].period = 200;
    tasks[i].elapsedTime = tasks[i].period;

    ++i;

    tasks[i].period = 1000;
    tasks[i].elapsedTime = tasks[i].period;

    int heat = 0;
    int seconds = 0;


    while(1){
        temperature = readTemp();
        seconds++;
        DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setPoint, heat, seconds))


    while(!TimerFlag){}
    TimerFlag = 0;
    ++timer0;

    }

    return (NULL);
}
