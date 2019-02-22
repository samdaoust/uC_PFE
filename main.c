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


//DEFINITIONS
//#define _XTAL_FREQ  16000000        // this is used by the __delay_ms(xx) and __delay_us(xx) functions
#define DEVICE_CONTROL_CODE  0b1010
#define INPUT_PIN 1
#define OUTPUT_PIN 0
#define SAMPLES_PER_SENSOR 90
#define SENSOR_1_ADDRESS 48 //0x30
#define SENSOR_2_ADDRESS 49 //0x31
#define SENSOR_3_ADDRESS 50 //0x32
#define SENSOR_4_ADDRESS 51 //0x33

#define SENSOR_1_ID 0
#define SENSOR_2_ID 1
#define SENSOR_3_ID 2
#define SENSOR_4_ID 3

#define HIGH_CURRENT_READING 60 //environ égal à 15 A

#define NUMBER_OF_SENSOR 4

//VARIABLES GLOBALES
unsigned int dataSensor1[SAMPLES_PER_SENSOR];
unsigned int dataSensor2[SAMPLES_PER_SENSOR];
unsigned int dataSensor3[SAMPLES_PER_SENSOR];
unsigned int dataSensor4[SAMPLES_PER_SENSOR];
unsigned int currentSensorValues[NUMBER_OF_SENSOR];
unsigned char lowCurrentReading = 0;
unsigned char dummyButtonPRESSED = 0;

//---------------------------------------------------------------------
//configure_PIC:
//configuration du microcontroleur
//---------------------------------------------------------------------
void configure_PIC()
{
    OSCCONbits.SPLLEN=0;    //PLL off
    OSCCONbits.IRCF=0x0F;   //OSC frequency = 16MHz
    OSCCONbits.SCS=0x02;    //internal oscillator block
   
    //TODO RECONFIGURER AVEC NOUVEAU PIC ET PINOUT
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
unsigned int get_Standard_Deviation(unsigned char dataSensor[])
{
    unsigned int standardDeviation = 0;
    float average = 0;
    unsigned int sum = 0;
    
    //calcul de la moyenne
    for(unsigned int sampleIndex = 0; sampleIndex<SAMPLES_PER_SENSOR;sampleIndex++)
    {
        average += dataSensor[sampleIndex];
    }
    average = average / SAMPLES_PER_SENSOR;
           
    //calcul de l'écart-type
    for(unsigned int sampleIndex = 0; sampleIndex<SAMPLES_PER_SENSOR;sampleIndex++)
    {
        sum+= (dataSensor[sampleIndex] - average) * 
                                         (dataSensor[sampleIndex] - average);
    }
    //TODO standardDeviation = sqrt(sum / SAMPLES_PER_SENSOR);
    standardDeviation = (sum / SAMPLES_PER_SENSOR);
    
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
    unsigned int signalMag = 0;
    unsigned int dataCount = 0;
    
    //CONFIGURATION uC
    configure_PIC();
    i2c_Init();
    
    //CONFIGURATION DES CAPTEURS
    configure_Sensor(SENSOR_1_ADDRESS);
    configure_Sensor(SENSOR_2_ADDRESS);
    configure_Sensor(SENSOR_3_ADDRESS);
    configure_Sensor(SENSOR_4_ADDRESS);
    
    //BOUCLE DE LECTURE DES CAPTEURS
    while(1)
    {
        for (unsigned char sensorIndex= 0; sensorIndex<NUMBER_OF_SENSOR; sensorIndex++)
        {   
            read_Sensor(sensorIndex, bufferData);
            signalMag = (bufferData[0] & 0x7F) << 8 | bufferData[1];
            
            switch(sensorIndex)
            {
                case SENSOR_1_ID:
                    dataSensor1[dataCount] = signalMag;
                case SENSOR_2_ID:
                    dataSensor2[dataCount] = signalMag;
                case SENSOR_3_ID:
                    dataSensor3[dataCount] = signalMag;
                case SENSOR_4_ID: 
                    dataSensor4[dataCount] = signalMag; 
                 
            }
            dataCount = dataCount + 1; //% SAMPLES_PER_SENSOR; //circular buffer
        }
        
        if (dataCount == SAMPLES_PER_SENSOR)
        {
            //update écart type
            currentSensorValues[SENSOR_1_ID] = get_Standard_Deviation(dataSensor1);
            currentSensorValues[SENSOR_2_ID] = get_Standard_Deviation(dataSensor2);
            currentSensorValues[SENSOR_3_ID] = get_Standard_Deviation(dataSensor3);
            currentSensorValues[SENSOR_4_ID] = get_Standard_Deviation(dataSensor4);
            
            //lancer calcul regression??
                    
            dataCount = 0;  
        }
        else if (dataCount == SAMPLES_PER_SENSOR && !dummyButtonPRESSED)
    }
    
    return;
}
