

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "wait.h"
#include "gpio.h"
#include "spi1.h"
#include "uart0.h"
#include "UartCmds.h"
#include "adc0.h"

#define LUT_SIZE 2048


// -----GLOBAL VARIABLES----------

uint16_t lutA[LUT_SIZE];
uint16_t lutB[LUT_SIZE];

uint16_t lutA_ptr = 0;
uint16_t lutB_ptr = 0;

uint32_t phase = 0x00000000;
uint32_t step = 0x00000000;
uint16_t currVal;
float Fstep;

//For DAC B frequency
uint32_t phaseB = 0x00000000;
uint32_t stepB = 0x00000000;
uint16_t currValB;
float FstepB;


//for cycles command.
char *cyclesC;
uint8_t cycles;
uint8_t cycleBit = 0;


uint16_t countA = 0;
uint16_t countB = 0;

uint16_t currValLast;

uint8_t vChan;
char* vTemp;


uint8_t chan = 0;

//--------------------------------

#define AIN2_MASK   2
#define AIN1_MASK   4
#define LDAC PORTE,3

#define Fs 80000 //Fs frequency




typedef enum _CHANNEL
{
    A = 0x3000, B = 0xB000
}CHANNEL;


initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    //Initialize the SPI1
    initSpi1(USE_SSI_FSS);
    setSpi1BaudRate(20e6, 40e6); //Set at 4 MHz
    setSpi1Mode(0, 0);

    // Enable clocks
    enablePort(PORTD);
    enablePort(PORTE); //Port E for the LDAC
    //enablePort(PORTB); //For AIN10, analog input.


    //Enable the LDAC as a digital output.
    selectPinPushPullOutput(LDAC);
    enablePinPullup(LDAC);

    // Configure AIN3 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN1_MASK | AIN2_MASK;                 // select alternative functions for AN3 (PE0)
    GPIO_PORTE_DEN_R &= ~AIN1_MASK | AIN2_MASK;                  // turn off digital operation on pin PE0
    GPIO_PORTE_AMSEL_R |= AIN1_MASK | AIN2_MASK;               // turn on analog operation on pin PE0
}



initTimer()
{
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);


    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 500;                             // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    //TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A) in NVIC
}




void setPhase(float freq, uint8_t chan)
{

    float check = 0.5;
    uint32_t ptr = 0x00008000;

    if (chan == 1)
    {
        //Calculating the Step uint32 Value.

           Fstep = freq/((float)Fs/(float)LUT_SIZE);

           //Masking the top 16 int bits of the phase uint32_t phase
           step = ((uint32_t)Fstep << 16);

           Fstep = Fstep - floor(Fstep);

           while(Fstep != 0.0 && ptr > 0)
           {
               if (Fstep >= check)
               {
                   step = step | ptr;
                   Fstep = Fstep - check;
               }

               ptr = ptr >> 1;
               check = check/2;

           }
    }

    else if (chan == 2)
    {
        //Calculating the Step uint32 Value.

        FstepB = freq/((float)Fs/(float)LUT_SIZE);

        //Masking the top 16 int bits of the phase uint32_t phase
        stepB = ((uint32_t)FstepB << 16);

        FstepB = FstepB - floor(FstepB);

        while(FstepB != 0.0 && ptr > 0)
        {
            if (FstepB >= check)
            {
                stepB = stepB | ptr;
                FstepB = FstepB - check;
            }

            ptr = ptr >> 1;
            check = check/2;

        }
    }


}


void turnOnTimer(void)
{
    TIMER1_CTL_R |= TIMER_CTL_TAEN;  // turn-on timer

}


void turnOffTimer(void)
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;// turn-off timer

}



// -5V - 5V -> 0V - 2.048V (Inverse of Op-Amp) calibrate
float calcDACVoltage(CHANNEL channel, float volts)
{
   float tempVolt = 0.0;

   switch (channel) {

       case A:

           tempVolt = -0.1893*volts + 1.0218; //Most accurate
           break;


       case B:

           tempVolt = -0.1892*volts + 1.0208; //Most accurate
           break;

       default:
           break;

       }


   return tempVolt;
}

uint16_t calcDACData(CHANNEL channel, float volts) // 0V - 2.048V -> 0 - 4095
{

   // return channel | (uint16_t)((volts/2.048) * 4095.0);
    float volts2;

    volts2 = calcDACVoltage(channel, volts);


    uint16_t R;

    switch (channel) {

    case A:
        R = (uint16_t)2005.2*volts2 + 3.9834;
        break;


    case B:
        R = (uint16_t)2007.2*volts2 + 4.5327;
        break;

    default:
        break;

    }

    return channel | R;

}

void writeDACData(uint16_t data)
{
    writeSpi1Data(data);
}

void latchDAC()
{
    setPinValue(LDAC,0);
    setPinValue(LDAC,1);
}


float getVoltage(uint8_t channel)
{
    uint16_t temporary;

    float fValue = 0.0;

    if (channel == 1)
    {
        setAdc0Ss3Mux(2);
        temporary = readAdc0Ss3();
        fValue = ((float)temporary) * 0.0008;

    }

    if (channel == 2)
    {
        setAdc0Ss3Mux(1);
        temporary = readAdc0Ss3();
        fValue = ((float)temporary) * 0.0008;

    }


    return fValue;
}


//-----------------------------------


void timer1Isr() //This function is called every time an interrupt fires.
{

    if (cycleBit == 1)
    {
        if (cycles == countA || cycles == countB)
        {
            turnOffTimer();

        }
    }


    phase = phase + step;
    currVal = (uint16_t)((phase & 0xFFFF0000) >> 16) % LUT_SIZE;

    phaseB = phaseB + stepB;
    currValB = (uint16_t)((phaseB & 0xFFFF0000) >> 16) % LUT_SIZE;



    if (currVal == 0 || currValB == 0)
    {
        countA++;
        countB++;
    }




    writeDACData(lutA[lutA_ptr]);
    lutA_ptr = currVal;

    writeDACData(lutB[lutB_ptr]);
    lutB_ptr = currValB;

    latchDAC();



    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}



//----------------------------------------

int main(void)
{

    initHw();
    initUart0();
    initTimer();

    initAdc0Ss3();

    // Use AIN3 input with N=4 hardware sampling
    setAdc0Ss3Mux(3);
    setAdc0Ss3Log2AverageCount(2);


    USER_DATA data;

    char *out;
    char *volt;
    char *freq;
    char *offset;
    char *shift;

    uint8_t temp = 0;

    uint16_t i;
    uint16_t j;

    float voltF;
    float fTemp;
    float off;
    float frequency;
    float Fshift;

    float VoltageIn = 0;
    char* str[100];


    while(true) {


        putsUart0("\nEnter Command:");
        getsUart0(&data);
        parseFields(&data);


        if (isCommand(&data, "cycles", 2))
        {
            cyclesC = getFieldString(&data, 1);
            cycles = atoi(cyclesC);
            cycleBit = 1;
            continue;
        }

        if (isCommand(&data, "reset", 1))
        {
            initHw();
            initUart0();
            initTimer();

            putsUart0("\nSystem Reset...\n");
            continue;

        }

        if (isCommand(&data, "stop", 1))
        {
            turnOffTimer();
            continue;
        }
        else if (isCommand(&data, "run", 1))
        {
            turnOnTimer();
            continue;
        }

        if (isCommand(&data, "voltage", 2))
        {
            vTemp = getFieldString(&data, 1);
            vChan = atoi(vTemp);
            VoltageIn = getVoltage(vChan);
            sprintf(str, "Voltage:   %4f\n", VoltageIn);
            putsUart0(str);

        }

        if (isCommand(&data, "level", 2))
        {
           //Code for level here

        }

        if (isCommand(&data, "gain", 3))
        {
            //Code for gain here.
        }





        //dc OUT, VOLTAGE
        if (isCommand(&data, "dc", 3))
        {
            out = getFieldString(&data, 1);
            volt = getFieldString(&data, 2);


            voltF = atof(volt);
            temp = atoi(out);


            if (temp == 1)
            {
                for(i = 0; i < LUT_SIZE; i++)
                {
                    chan = 1;

                    fTemp = (voltF);

                    lutA[i] = calcDACData(A, fTemp);
                }

            }

            else if (temp == 2)
            {
                for(i = 0; i < LUT_SIZE; i++)
                {

                    chan = 2;

                    fTemp = (voltF);

                    lutB[i] = calcDACData(B, fTemp);


                }

            }

            lutA_ptr = 0;
            lutB_ptr = 0;



        }
        //sine chan freq ampl offset
        else if (isCommand(&data, "sine", 6))
        {
            //Enter sine function in here
            out = getFieldString(&data, 1);
            freq = getFieldString(&data, 2);
            volt = getFieldString(&data, 3);
            offset = getFieldString(&data, 4);
            shift = getFieldString(&data, 5);


            temp = atoi(out);
            frequency = atof(freq);
            voltF = atof(volt);
            off = atof(offset);
            Fshift = atof(shift);


            if (temp == 1) {

               chan = 1;

               for(i = 0; i < LUT_SIZE; i++)
               {

                   fTemp = off + (voltF) * sin(((float)i/(float)LUT_SIZE) * 2.0 * M_PI + Fshift);

                   lutA[i] = calcDACData(A, fTemp);

               }

               setPhase(frequency, 1);
            }

           if (temp == 2) {

               chan = 2;

               for(i = 0; i < LUT_SIZE; i++)
               {

                   fTemp = off + (voltF) * sin(((float)i/(float)LUT_SIZE) * 2.0 * M_PI + Fshift);

                   lutB[i] = calcDACData(B, fTemp);

               }

               setPhase(frequency, 2);
           }


           lutA_ptr = 0;
           lutB_ptr = 0;

           //set the uint32_t phase var.
           //setPhase(frequency);

        }

        //square chan freq ampl offset
        else if (isCommand(&data, "square", 5))
        {
            //Enter square function in here
            out = getFieldString(&data, 1);
            freq = getFieldString(&data, 2);
            volt = getFieldString(&data, 3);
            offset = getFieldString(&data, 4);

            temp = atoi(out);
            frequency = atof(freq);
            voltF = atof(volt);
            off = atof(offset);

            //Output 1 (Dac A)
            if (temp == 1) {

                chan = 1; //global var for ISR channel check .

                //First half of the Square Wave
                for(i = 0; i < (LUT_SIZE/2); i++)
                {
                    fTemp = off + voltF;

                    lutA[i] = calcDACData(A, fTemp);

                }
                //Second Half of the square wave.
                for(i = (LUT_SIZE/2); i < LUT_SIZE; i++)
                {
                    fTemp = off - voltF;

                    lutA[i] = calcDACData(A, fTemp);

                }

                setPhase(frequency, 1);

            }
           //Output 2 (Dac B)
           if (temp == 2)
           {

               chan = 2; //global var for ISR channel check .

               //First half of the Square Wave
               for(i = 0; i < (LUT_SIZE/2); i++)
               {
                   fTemp = off + voltF;

                   lutB[i] = calcDACData(B, fTemp);



               }

               //Second Half of the square wave.
               for(i = (LUT_SIZE/2); i < LUT_SIZE; i++)
               {
                   fTemp = off - voltF;

                   lutB[i] = calcDACData(B, fTemp);



               }

               setPhase(frequency, 2);


           }


           lutA_ptr = 0;
           lutB_ptr = 0;

           //set the uint32_t phase var.
           //setPhase(frequency);


        }

        //Sawtooth chan freq ampl offset
        else if (isCommand(&data, "sawtooth", 5))
        {
            //Enter square function in here
            out = getFieldString(&data, 1);
            freq = getFieldString(&data, 2);
            volt = getFieldString(&data, 3);
            offset = getFieldString(&data, 4);

            temp = atoi(out);
            frequency = atof(freq);
            voltF = atof(volt);
            off = atof(offset);

            //Output 1 (Dac A)
            if (temp == 1) {

                chan = 1; //global var for ISR channel check .

                for(i = 0; i < (LUT_SIZE); i++)
                {
                    fTemp = off + (voltF * ((float)i/(float)LUT_SIZE));

                    lutA[i] = calcDACData(A, fTemp);

                }

                setPhase(frequency, 1);


            }
           //Output 2 (Dac B)
           if (temp == 2) {

               chan = 2; //global var for ISR channel check .

               for(i = 0; i < (LUT_SIZE); i++)
               {
                   fTemp = off + (voltF * ((float)i/(float)LUT_SIZE));

                   lutB[i] = calcDACData(B, fTemp);

               }

               setPhase(frequency, 2);

           }

           lutA_ptr = 0;
           lutB_ptr = 0;

           //set the uint32_t phase var.
          // setPhase(frequency);

        }

        //triangle chan freq ampl offset
        else if (isCommand(&data, "triangle", 5))
        {
            //Enter square function in here
            out = getFieldString(&data, 1);
            freq = getFieldString(&data, 2);
            volt = getFieldString(&data, 3);
            offset = getFieldString(&data, 4);

            temp = atoi(out);
            frequency = atof(freq);
            voltF = atof(volt);
            off = atof(offset);

            //Output 1 (Dac A)
            if (temp == 1) {

                chan = 1; //global var for ISR channel check .

                for(i = 0; i < (LUT_SIZE/2); i++)
                {
                    fTemp = off + (voltF * ((float)i/(float)(LUT_SIZE/2)));

                    lutA[i] = calcDACData(A, fTemp);

                }

                for (i = (LUT_SIZE/2), j = (LUT_SIZE/2); i >= 0, j < LUT_SIZE; i-- ,j++)
                {
                    fTemp = off + (voltF * ((float)i/(float)(LUT_SIZE/2)));
                    lutA[j] = calcDACData(A, fTemp);
                }

                setPhase(frequency, 1);


            }
           //Output 2 (Dac B)
           if (temp == 2) {

               chan = 2; //global var for ISR channel check .

               for(i = 0; i < (LUT_SIZE/2); i++)
               {
                   fTemp = off + (voltF * ((float)i/(float)(LUT_SIZE/2)));

                   lutB[i] = calcDACData(B, fTemp);

               }

               for (i = (LUT_SIZE/2), j = (LUT_SIZE/2); i >= 0, j < LUT_SIZE; i-- ,j++)
               {
                   fTemp = off + (voltF * ((float)i/(float)(LUT_SIZE/2)));

                   lutB[j] = calcDACData(B, fTemp);
               }

               setPhase(frequency, 2);

           }

           lutA_ptr = 0;
           lutB_ptr = 0;

           //set the uint32_t phase var.
          // setPhase(frequency);

        }


    }
}

