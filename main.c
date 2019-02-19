/*
 * File:   main.c
 * Author: samueldaoust
 * 
 * inspiré de l'exemple concernant la communication I2C
 * de Microchip pour le PIC12F1822
 * Created on February 16, 2019, 4:31 PM
 */

#include <xc.h>
#include "const.h"
#include "i2c.h"


// set Config bits
#pragma config FOSC=INTOSC
//TODO set config bit !!!



//Definitions
//#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions
#define DEVICE_CONTROL_CODE  0b1010
#define INPUT_PIN 1
#define OUTPUT_PIN 0
#define SAMPLES_PER_SENSOR 50
#define SENSOR_ADDRESS_OFFSET 48 //0x30
#define NUMBER_OF_SENSOR 4

unsigned int dataSensor[SAMPLES_PER_SENSOR];

//---------------------------------------------------------------------
//configure_PIC:
//configuration du microcontroleur
//---------------------------------------------------------------------
void configure_PIC()
{
    OSCCONbits.SPLLEN=0;    //PLL off
    OSCCONbits.IRCF=0x0F;   //OSC frequency = 16MHz
    OSCCONbits.SCS=0x02;    //internal oscillator block
   
    // PORT A Assignments
    TRISAbits.TRISA0 = INPUT_PIN;	// RA0 = SDA
    TRISAbits.TRISA1 = INPUT_PIN ;	// RA1 = SCLK
    TRISAbits.TRISA2 = OUTPUT_PIN ;	// RA2 = LED
    TRISAbits.TRISA3 = INPUT_PIN;	// RA3 = Vpp / bouton
    TRISAbits.TRISA4 = OUTPUT_PIN;	// RA4 = TX UART
    TRISAbits.TRISA5 = OUTPUT_PIN;	// RA5 =  RX UART
    
    ANSELA=0x1F;		// digital I/O
}

//---------------------------------------------------------------------
//configure_Sensor:
//configuration d'un capteur SI7210 via i2c
//---------------------------------------------------------------------
void configure_Sensor(unsigned char address)
{
    i2c_Start();
    i2c_Address(address, I2C_WRITE);
    i2c_Write(SI72XX_ARAUTOINC);
    i2c_Write(ARAUTOINC__ARAUTOINC_MASK);
    
    i2c_Write(SI72XX_DSPSIGSEL);
    i2c_Write(DSPSIGSEL__MAG_VAL_SEL);
    
    i2c_Write(SI72XX_CTRL4);  //améliorer les lignes et commenter
    i2c_Write(4);
    
    i2c_Write(SI72XX_SLTIME);
    i2c_Write(4);
    
    i2c_Write(SI72XX_CTRL3);
    i2c_Write(4);
    
    i2c_Write(SI72XX_POWER_CTRL);
    i2c_Write(0);
    
    i2c_Stop();
}


//---------------------------------------------------------------------
//read_Sensor:
//configuration d'un capteur SI7210 via i2c
//---------------------------------------------------------------------
void read_Sensor(unsigned char address, unsigned char dataRead[])
{
    i2c_Start();
    i2c_Address(address, I2C_WRITE);
    i2c_Write(SI72XX_DSPSIGM);
    i2c_Restart();
    i2c_Address(address, I2C_READ);
    dataRead[0] = i2c_Read(1);
    dataRead[1] = i2c_Read(0);
}

//---------------------------------------------------------------------
//get_Standard_Deviation:
//calcul de l'écart type 
//---------------------------------------------------------------------
unsigned int get_Standard_Deviation(unsigned char actualCurrentPerSensor[])
{
    unsigned int standardDeviation = 0;
    float average = 0;
    unsigned int sum = 0;
    
    //calcul de la moyenne
    for(unsigned int sampleIndex = 0; sampleIndex<SAMPLES_PER_SENSOR;sampleIndex++)
    {
        average += actualCurrentPerSensor[sampleIndex];
    }
    average = average / SAMPLES_PER_SENSOR;
           
    //calcul de l'écart-type
    for(unsigned int sampleIndex = 0; sampleIndex<SAMPLES_PER_SENSOR;sampleIndex++)
    {
        sum+= (actualCurrentPerSensor[sampleIndex] - average) * 
                                         (actualCurrentPerSensor[sampleIndex] - average);
    }
    standardDeviation = sum / SAMPLES_PER_SENSOR;
    
    return standardDeviation;
}


//---------------------------------------------------------------------
//calibration_Data:
//---------------------------------------------------------------------
void calibration_Data()
{
    
    

}




//---------------------------------------------------------------------
//                          Main Function
//---------------------------------------------------------------------
void main(void) 
{
    unsigned char bufferData[2];

    unsigned int actualCurrentPerSensor[NUMBER_OF_SENSOR];
    
    unsigned int signal = 0;
    // unsigned int signal = 0;
    
    unsigned int dataCount=  0;
    configure_PIC();
    i2c_Init();
    
    while(1)
    {
        for (int sensorIndex= 0; sensorIndex<4; sensorIndex++)
        {   
            read_Sensor(sensorIndex, bufferData);
            //TODO treatment:
            //signal = (bufferData[0] & 0x7F) << 8 | bufferData[1]
            
            switch(sensorIndex)
            {
                case 0:
                    dataSensor[dataCount] = signal;
                /*    
                case 1:
                    dataSensor2[dataCount] = signal;
                case 2:
                    dataSensor3[dataCount] = signal;
                case 3: 
                    dataSensor4[dataCount] = signal; 
                 */     
            }
            dataCount = dataCount + 1 % SAMPLES_PER_SENSOR; //circular buffer
        }
        
    }
    
    return;
}
