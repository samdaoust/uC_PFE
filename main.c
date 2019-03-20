/*
 * File:   main.c
 * Author: samueldaoust
 * 
 * inspiré de l'exemple concernant la communication I2C
 * de Microchip pour le PIC12F1822
 * Created on February 16, 2019, 4:31 PM
 */


// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include <xc.h>
#include "const.h"
#include "i2c.h"

//DEFINITIONS
#define _XTAL_FREQ  16000000
#define DEVICE_CONTROL_CODE  0b1010
#define INPUT_PIN 1
#define OUTPUT_PIN 0
#define SAMPLES_PER_SENSOR 300
#define CTRL4_MASK 0b10001000 //value for digital filtering
#define SLEEP_TIME 0 //should not be changed
#define TAMPER_VALUE 2
#define POWER_CONTROL_VALUE 0
#define PIC_ADDRESS 0x41 //must be different on every PIC

#define NUMBER_OF_SENSOR 1
#define SENSOR_STARTING_ADDRESS 48 //0X30
#define SENSOR_1_ADDRESS 48 //0x30
#define SENSOR_2_ADDRESS 49 //0x31
#define SENSOR_3_ADDRESS 50 //0x32
#define SENSOR_4_ADDRESS 51 //0x33
#define SENSOR_1_ID 0
#define SENSOR_2_ID 1
#define SENSOR_3_ID 2
#define SENSOR_4_ID 3

#define UART_BAUD_RATE 9600

#define HIGH_CURRENT_READING 60 //environ égal à 15 A



//VARIABLES GLOBALES
unsigned int dataSensor1[SAMPLES_PER_SENSOR];
//unsigned int dataSensor2[SAMPLES_PER_SENSOR];
//unsigned int dataSensor3[SAMPLES_PER_SENSOR];
//unsigned int dataSensor4[SAMPLES_PER_SENSOR];
unsigned int currentSensorValues[NUMBER_OF_SENSOR];
unsigned char lowCurrentReading = 0;
unsigned char dummyButtonPRESSED = 0;
unsigned char RXFramingErr = 0;
unsigned char RXOverRunErr = 0;

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
   
    TRISAbits.TRISA0 = INPUT_PIN;	// RA0 = ICSPDAT(used only for programming)
    TRISAbits.TRISA1 = INPUT_PIN ;	// RA1 = ICSPCLK (used only for programming)
    TRISAbits.TRISA2 = INPUT_PIN;	// RA2 = BUTTON
    TRISAbits.TRISA3 = INPUT_PIN;	// RA3 = VPP(used only for programming)
    TRISAbits.TRISA4 = INPUT_PIN;	// RA4 = NOT USED
    TRISAbits.TRISA5 = INPUT_PIN;	// RA5 = NOT USED
    
    TRISCbits.TRISC0 = INPUT_PIN;	// RC0 = SCL
    TRISCbits.TRISC1 = INPUT_PIN ;	// RC1 = SDA
    TRISCbits.TRISC2 = OUTPUT_PIN ;	// RC2 = LED
    TRISCbits.TRISC3 = INPUT_PIN;	// RC3 = NOT USED (BUTTON)
    TRISCbits.TRISC4 = OUTPUT_PIN;	// RC4 = TX UART
    TRISCbits.TRISC5 = INPUT_PIN;	// RC5 = RX UART
    
    WPUA0 = 0;
    WPUA1 = 0;
    
    ANSELA=0x00;		// digital I/O
    ANSELC=0x00;
    
    //configuration interrupt
    INTCONbits.INTF = 0;        //reset the external interrupt flag
    PIE1bits.TXIE = 0; // Enable USART Transmitter interrupt
    PIE1bits.RCIE = 1;
    OPTION_REGbits.INTEDG = 1;  //interrupt on the rising edge
    INTCONbits.PEIE = 1; // Enable peripheral interrupts
    INTCONbits.GIE = 1; // Enable global interrupts
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
    
    i2c_Restart();
    i2c_Address(address, I2C_WRITE);
    i2c_Write(SI72XX_ARAUTOINC);
    i2c_Write(ARAUTOINC__ARAUTOINC_MASK);
    
    i2c_Restart();
    i2c_Address(address, I2C_WRITE);
    i2c_Write(SI72XX_DSPSIGSEL);  //améliorer les lignes et commenter
    i2c_Write(0);
    
    i2c_Restart();
    i2c_Address(address, I2C_WRITE);
    i2c_Write(SI72XX_CTRL4);  //améliorer les lignes et commenter
    i2c_Write(CTRL4_MASK);
    
    i2c_Restart();
    i2c_Address(address, I2C_WRITE);
    i2c_Write(SI72XX_SLTIME);
    i2c_Write(SLEEP_TIME);
    
    i2c_Restart();
    i2c_Address(address, I2C_WRITE);
    i2c_Write(SI72XX_CTRL3);
    i2c_Write(TAMPER_VALUE);
    
    i2c_Restart();
    i2c_Address(address, I2C_WRITE);
    i2c_Write(SI72XX_POWER_CTRL);
    i2c_Write(POWER_CONTROL_VALUE);
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
    i2c_Stop();
}

//---------------------------------------------------------------------
//get_Standard_Deviation:
//calcul de l'écart type 
//---------------------------------------------------------------------
long double get_Variance(unsigned int *dataSensor)
{
    double variance = 0;
    double average = 0;
    double sum = 0;
    
    //calcul de la moyenne
    for(unsigned int sampleIndex = 0; sampleIndex<SAMPLES_PER_SENSOR;sampleIndex++)
    {
        average += (double)dataSensor[sampleIndex];
    }
    average = average / SAMPLES_PER_SENSOR;
           
    //calcul de l'écart-type
    for(unsigned int sampleIndex = 0; sampleIndex<SAMPLES_PER_SENSOR;sampleIndex++)
    {
        sum+= ((double)dataSensor[sampleIndex] - average) * ((double)dataSensor[sampleIndex] - average);
    }
    variance = (sum / (SAMPLES_PER_SENSOR - 1));
    
    return variance;
}

//---------------------------------------------------------------------
//init_UART:
//configuration du port UART pour envois des données vers FTDI
// inspiré de: https://circuitdigest.com/microcontroller-projects/uart-communication-using-pic16f877a
//---------------------------------------------------------------------
void init_UART(void)
{
    //baud rate and set BRGH for fast baud_rate
    SPBRG = ((_XTAL_FREQ/16)/UART_BAUD_RATE) - 1;
    BRGH  = 1;  // for high baud_rate
    
    SYNC  = 0;    // Asynchronous
    SPEN  = 1;    // Enable serial port pins
    
    TXEN  = 1;    // enable transmission
    CREN  = 1;    // enable reception
    
    TX9   = 0;    // 8-bit reception selected
    RX9   = 0;    // 8-bit reception mode selected
}

//---------------------------------------------------------------------
//send__byte_UART:
// fonction qui permet d'envoyer un byte sur le port UART
// inspiré de: https://circuitdigest.com/microcontroller-projects/uart-communication-using-pic16f877a
//---------------------------------------------------------------------
void send_byte_UART(unsigned char byte)
{
    while(!TXIF);  // hold the program till TX buffer is free
    TXREG = byte;  //Load the transmitter buffer with the received value
}

//---------------------------------------------------------------------
//get_byte_UART:
// fonction qui permet d'envoyer un byte sur le port UART
// inspiré de: https://circuitdigest.com/microcontroller-projects/uart-communication-using-pic16f877a
//---------------------------------------------------------------------
unsigned char get_byte_UART(void)
{    
    while(!RCIF);  // hold the program till RX buffer is free
    
    if(FERR)
        {
            RXFramingErr = 1;
            SPEN = 0;
            SPEN = 1;

        }
    if(OERR)
        {
            RXOverRunErr = 1;
            CREN = 0;
            CREN = 1;
        }
    
    return RCREG; //receive the value and send it to main function
}
//---------------------------------------------------------------------
//isr:
//      Gestion des interruptions
//---------------------------------------------------------------------
void interrupt ISR(void)
{
    unsigned char rxData = 0;
    if(RCIF) // handle RX pin interrupts
    { 
        rxData = get_byte_UART();

        if (PIC_ADDRESS == rxData)
        {
            for (unsigned char sensorIndex= 0; sensorIndex<NUMBER_OF_SENSOR; sensorIndex++)
            {
                send_byte_UART((char)PIC_ADDRESS);
                send_byte_UART((char)(currentSensorValues[sensorIndex] >> 8)); //LSBs first
                send_byte_UART((char)(currentSensorValues[sensorIndex] & 0x00FF)); //MSBs
                //send_byte_UART(10);
                //send_byte_UART(1);
            }
            //signalisation
            RC2 = 0;
            __delay_ms(15);
            RC2 = 1;
        }
    } 
    INTF = 0; //reset int. flag
}
//---------------------------------------------------------------------
//                          Main Function
//---------------------------------------------------------------------
void main(void) 
{
    unsigned char bufferData[2];
    unsigned int signalMag = 0;
    unsigned int dataCount = 0;
    unsigned int test = 0;
    
    //CONFIGURATION uC
    configure_PIC();
    
    RC2 = 1;
    __delay_ms(1000);
    RC2 = 0;
    __delay_ms(300);
    RC2 = 1;
    __delay_ms(1000);
    RC2 = 0;
    
    i2c_Init();
    init_UART();
    
    //CONFIGURATION DES CAPTEURS
    configure_Sensor(SENSOR_1_ADDRESS);
    //configure_Sensor(SENSOR_2_ADDRESS);
    //configure_Sensor(SENSOR_3_ADDRESS);
    //configure_Sensor(SENSOR_4_ADDRESS);
    
    RC2 = 1; // CONFIGURATION TERMINÉE, LED ALLUMÉE
   
    /*
    i2c_Start();
    i2c_Address(0x30, I2C_WRITE);
    i2c_Write(SI72XX_CTRL4);
    i2c_Restart();
    i2c_Address(0x30, I2C_READ);
    test = i2c_Read(0);
    i2c_Stop();
    */
    
    
    //BOUCLE DE LECTURE DES CAPTEURS
    while(1)
    {
        for (unsigned char sensorIndex= 0; sensorIndex<NUMBER_OF_SENSOR; sensorIndex++)
        {   
            read_Sensor((0x30), bufferData); 
            signalMag = (bufferData[0] & 0x7F) << 8 | bufferData[1];
            
            switch(sensorIndex)
            {
                case SENSOR_1_ID:
                    dataSensor1[dataCount] = signalMag;
                case SENSOR_2_ID:
                    dataSensor1[dataCount] = signalMag; //changer le nom apres test
                case SENSOR_3_ID:
                    dataSensor1[dataCount] = signalMag;
                case SENSOR_4_ID: 
                    dataSensor1[dataCount] = signalMag;   
            }
        }
        dataCount = dataCount + 1 % SAMPLES_PER_SENSOR; //circular buffer
        
        if (dataCount == SAMPLES_PER_SENSOR)
        {
            //update écart type
            currentSensorValues[SENSOR_1_ID] = (int)get_Variance(dataSensor1);
            //currentSensorValues[SENSOR_2_ID] = get_Standard_Deviation(&dataSensor2);
            //currentSensorValues[SENSOR_3_ID] = get_Standard_Deviation(&dataSensor3);
            //currentSensorValues[SENSOR_4_ID] = get_Standard_Deviation(&dataSensor4);           
            dataCount = 0;  
        }
        else if (dataCount == SAMPLES_PER_SENSOR && !dummyButtonPRESSED)
        {
        }
    }
    
    return;
}