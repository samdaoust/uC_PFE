/* 
 * File:   const.h
 * Author: samueldaoust
 *
 * Created on February 16, 2019, 5:17 PM
 */

#ifndef CONST_H
#define	CONST_H

//---------------------------------------------------------------------
//Constantes reliées au SI7210
//---------------------------------------------------------------------

 
#define ARAUTOINC__ARAUTOINC_MASK       0x01
#define OTP_CTRL__OPT_BUSY_MASK         0x01
#define OTP_CTRL__OPT_READ_EN_MASK      0x02
#define POWER_CTRL__SLEEP_MASK          0x01
#define POWER_CTRL__STOP_MASK           0x02
#define POWER_CTRL__ONEBURST_MASK       0x04
#define POWER_CTRL__USESTORE_MASK       0x08
#define POWER_CTRL__MEAS_MASK           0x80
#define DSPSIGSEL__MAG_VAL_SEL          0
#define DSPSIGSEL__TEMP_VAL_SEL         1
 
#define SI72XX_OTP_TEMP_OFFSET  0x1D
#define SI72XX_OTP_TEMP_GAIN    0x1E
#define SI72XX_HREVID           0xC0
#define SI72XX_DSPSIGM          0xC1
#define SI72XX_DSPSIGL          0xC2
#define SI72XX_DSPSIGSEL        0xC3
#define SI72XX_POWER_CTRL       0xC4
#define SI72XX_ARAUTOINC        0xC5
#define SI72XX_CTRL1            0xC6
#define SI72XX_CTRL2            0xC7
#define SI72XX_SLTIME           0xC8
#define SI72XX_CTRL3            0xC9
#define SI72XX_A0               0xCA
#define SI72XX_A1               0xCB
#define SI72XX_A2               0xCC
#define SI72XX_CTRL4            0xCD
#define SI72XX_A3               0xCE
#define SI72XX_A4               0xCF
#define SI72XX_A5               0xD0
#define SI72XX_OTP_ADDR         0xE1
#define SI72XX_OTP_DATA         0xE2
#define SI72XX_OTP_CTRL         0xE3
#define SI72XX_TM_FG            0xE4
 
#define SI7210_BASE_ADDR_7BIT   0x30

//TODO enum des adresses des capteurs

//---------------------------------------------------------------------
//Constantes reliées au uC
//---------------------------------------------------------------------


#endif	/* CONST_H */

