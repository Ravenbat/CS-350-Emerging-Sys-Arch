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

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"


uint32_t blinkCount = 0;         //every 500ms count
int button_pressed = 0;         //to detect if button is pressed at the end of word breaks
enum BL_States {BL_Sos, BL_Ok} BL_State;  //states of SOS and OK

//Using functions for each letters in morse code to have less code in main and in state machine switches.
void blinkS(){
    if(blinkCount == 1){
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);                 //Green LED light won't blink on SOS state during S letter part even though I am telling it to blink
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);                  //to blink with toggle GPIO_toggle(CONFIG_GPIO_LED_1); option. Logic defying LED light....
    }
    else if (blinkCount < 6 || blinkCount >=23 && blinkCount <=27) {
        // Blink LED for "S" in Morse code
        GPIO_toggle(CONFIG_GPIO_LED_0);
        GPIO_toggle(CONFIG_GPIO_LED_1);
    } else
        // Delay between letters
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    }




void blinkO(){
    if (blinkCount == 12 || blinkCount == 16 || blinkCount >= 20) {
            // Blink LED for "O" in Morse code
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
        } else {
            // Delay between letters
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        }
    }

void blinkOk(){
    if (blinkCount == 4 || blinkCount == 8 || blinkCount == 19 || blinkCount == 21) {
            // Blink LED for "O" in Morse code
            GPIO_toggle(CONFIG_GPIO_LED_0);
            GPIO_toggle(CONFIG_GPIO_LED_1);
        }
    else if (blinkCount >= 12 && blinkCount <=15){
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);

    }
        else {
            // Delay between letters
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);
        }
    }

void brkBetweenWords(){
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_OFF);
    switch (BL_State){
    case BL_Sos:
        if(blinkCount == 34 && button_pressed == 1){
            BL_State = BL_Ok;
            button_pressed = 0;
        }
        break;
    case BL_Ok:
        if(blinkCount == 30 && button_pressed == 1){
            BL_State = BL_Sos;
            button_pressed = 0;
        }
        break;
        button_pressed = 0;
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
        button_pressed = 1;
        printf("Button pressed!");
}

void gpioButtonFxn0(uint_least8_t index)
{
     //Change button pressed to 1
    button_pressed = 1;
    printf("Button pressed!");
}

void timerCallback(Timer_Handle myHandle, int_fast16_t status)            //since there is only one set timer of 500ms I relied on universal variable and keeping
{                                                                         //track of each 500ms passes, I get the morse code going. It is not great to look at but I couldn't figure out other way.
                                                                          //timerCallBack will be called every 500ms and blinkcount will increase until SOS or OK is finished.
    switch (BL_State) {
        case BL_Sos:
            if (blinkCount <= 8) {
                blinkS();
            } else if (blinkCount <=22){
                blinkO();
            }
            else if(blinkCount >= 23 && blinkCount <=27){
                blinkS();
            }
            else if(blinkCount < 35){
                brkBetweenWords();
            }
            else {
                blinkCount = 0;
            }
            break;

        case BL_Ok:
            if (blinkCount <= 23) {
                blinkOk();
            }
            else if (blinkCount < 31){
                brkBetweenWords();
            }
            else {
                blinkCount = 0;
            }
            break;
    }
    blinkCount++;

    // Restart the timer
    Timer_start(myHandle);
}


void initTimer(void)                  //it should have been made clear that this function calls Callback function every 500000ms. I wasn't aware of it and took long time to figure it out.
{
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;


    timer0 = Timer_open(CONFIG_TIMER_0, &params);


    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {
            printf("Timer initilization failed.");
        }
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {
            printf("Timer start failed.");}
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING | GPIO_CFG_IN_INT_HIGH);

    /* Turn on user LEDs */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
    GPIO_write(CONFIG_GPIO_LED_1, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    //State Machine attempt
    initTimer();
    BL_State = BL_Sos;



    return (NULL);
}
