/*
 * Hi-Tech C I2C library for 12F1822
 * Master mode routines for I2C MSSP port to read and write to slave device 
 * Copyright (C)2011 HobbyTronics.co.uk 2011
 * Freely distributable.
 * http://www.hobbytronics.co.uk/hi-tech-c-i2c-master
*/

//************
// voir : https://www.teachmemicro.com/pic16-i2c/ pour tutoriel
//************

#pragma once

#define I2C_WRITE 0
#define I2C_READ 1

// Initialise MSSP port. (12F1822 - other devices may differ)
void i2c_Init(void){

   // Initialise I2C MSSP
   // Master 100KHz
   //TRISA1=1;                    // set SCL and SDA pins as inputs
   //TRISA2=1;

   SSPCON1 = 0b00101000; 	// I2C enabled, Master mode
   SSPCON2 = 0x00;
   // I2C Master mode, clock = FOSC/(4 * (SSPADD + 1)) 
   SSPADD = 39;    		// 100Khz @ 16Mhz Fosc

   SSPSTAT = 0b11000000; 	// Slew rate disabled

}

// i2c_Wait - wait for I2C transfer to finish
void i2c_Wait(void){
    while ( ( SSP1CON2 & 0x1F ) || ( SSPSTAT & 0x04 ) );
}

// i2c_Start - Start I2C communication
void i2c_Start(void)
{
    i2c_Wait();
    SEN=1;
}

// i2c_Restart - Re-Start I2C communication
void i2c_Restart(void){
    i2c_Wait();
    RSEN=1;
}

// i2c_Stop - Stop I2C communication
void i2c_Stop(void)
{
    i2c_Wait();
    PEN=1;
}

// i2c_Write - Sends one byte of data
void i2c_Write(unsigned char data)
{
    i2c_Wait();
    SSPBUF = data;
}

// i2c_Address - Sends Slave Address and Read/Write mode
// mode is either I2C_WRITE or I2C_READ
void i2c_Address(unsigned char address, unsigned char mode)
{
    unsigned char l_address;

    l_address=address<<1;
    l_address+=mode;
    i2c_Wait();
    SSPBUF = l_address;
}

// i2c_Read - Reads a byte from Slave device
unsigned char i2c_Read(unsigned char ack)
{
    // Read data from slave
    // ack should be 1 if there is going to be more data read
    // ack should be 0 if this is the last byte of data read
    unsigned char i2cReadData;

    i2c_Wait();
    RCEN=1;
    i2c_Wait();
    i2cReadData = SSPBUF;
    i2c_Wait();
    if ( ack ) ACKDT=0;	        // Ack
    else       ACKDT=1;	        // NAck
    ACKEN=1;                    // send acknowledge sequence

    return( i2cReadData );
}
