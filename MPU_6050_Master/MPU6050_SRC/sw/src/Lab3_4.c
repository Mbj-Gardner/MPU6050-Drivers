/**
 * @file Lab3_4.c
 * @author your name (you@domain.com), Jonathan Valvano, Matthew Yu (matthewjkyu@gmail.com)
 *    <TA NAME and LAB SECTION # HERE>
 * @brief
 *    A default main file for running lab3 and 4.
 *    Feel free to edit this to match your specifications.
 *
 *    For these two labs, the student must implement an alarm clock with various 
 *    functions (lab 3) and then integrate it with a remote server, Blynk (lab 4). 
 *    This assignment is open ended, so students must plan the features of their 
 *    alarm clock (besides some base required features), design drivers for peripherals 
 *    used by the clock (ST7735 drawing routines, switch debounce drivers, and so forth), 
 *    and integrate it all together to have a functioning device. Good luck!
 * 
 * @version 0.2
 * @date 2022-09-12 <REPLACE WITH DATE OF LAST REVISION>
 *
 * @copyright Copyright (c) 2022
 * @note 
 *    We suggest following the pinouts provided by the 
 *    EE445L_Baseline_Schematic_Guide.pdf, found in the resources folder.
 *    Warning. Initial code for the RGB driver creates bright flashing lights. 
 *    Please remove this code and do not run if you have epilepsy.
 */

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2021

 Copyright 2022 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

/** File includes. */
#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* Register definitions. */
#include "./inc/tm4c123gh6pm.h"
/* Clocking. */
#include "./inc/PLL.h"
/* Clock delay and interrupt control. */
#include "./inc/CortexM.h"
/* Initialization of all the pins. */
#include "./inc/Unified_Port_Init.h"
/* Talking to PC via UART. */
#include "./inc/UART.h"
/* Talking to Blynk via the ESP8266. */
#include "./inc/Blynk.h"
/* ST7735 display. */
#include "./inc/ST7735.h"
/* Add whatever else you need here! */
#include "./lib/RGB/RGB.h"
#include <math.h>
//#include "TM4C123GH6PM.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define RAD_2_DEG           57.29578
#define XG_OFFS_TC          0x00
#define YG_OFFS_TC          0x01
#define ZG_OFFS_TC          0x02
#define X_FINE_GAIN         0x03
#define Y_FINE_GAIN         0x04
#define Z_FINE_GAIN         0x05
#define XA_OFFS_H           0x06 
#define XA_OFFS_L_TC        0x07
#define YA_OFFS_H           0x08 
#define YA_OFFS_L_TC        0x09
#define ZA_OFFS_H           0x0A 
#define ZA_OFFS_L_TC        0x0B
#define XG_OFFS_USRH        0x13
#define XG_OFFS_USRL        0x14
#define YG_OFFS_USRH        0x15
#define YG_OFFS_USRL        0x16
#define ZG_OFFS_USRH        0x17
#define ZG_OFFS_USRL        0x18
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define FF_THR              0x1D
#define FF_DUR              0x1E
#define MOT_THR             0x1F
#define MOT_DUR             0x20
#define ZRMOT_THR           0x21
#define ZRMOT_DUR           0x22
#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define DMP_INT_STATUS      0x39
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define MOT_DETECT_STATUS   0x61
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define MOT_DETECT_CTRL     0x69
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define BANK_SEL            0x6D
#define MEM_START_ADDR      0x6E
#define MEM_R_W             0x6F
#define DMP_CFG_1           0x70
#define DMP_CFG_2           0x71
#define FIFO_COUNTH         0x72
#define FIFO_COUNTL         0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75
void I2C3_Init(void);
char I2C3_Wr(int slaveAddr, char memAddr, char data);
char I2C3_Rd(int slaveAddr, char memAddr, int byteCount, uint8_t* data);
void Delay(unsigned long counter);
void uart5_init(void);
void UART5_Transmitter(unsigned char data);
void calibirate_MPU6050(int num_iterations);
void printstring(char *str);
void MPU6050_Init(void);
char setOffsets(int slaveAddr, char memAddr, int8_t data);
void read_MPU6050(void);
char msg[20];
int16_t accX, accY, accZ, GyroX, GyroY, GyroZ, Temper;
uint8_t sensordata[14];
float AX, AY, AZ, t, GX, GY, GZ;
static int16_t offset_gyx = 0, offset_gyy = 0, offset_gyz = 0;
float angleAccX; float angleAccY;
float offset_angleAccX = 0, offset_angleAccY = 0;
/* TODO: enable this for lab 4. */
#define LAB_4 false

/* TODO: We suggest using the ./inc/ADCSWTrigger.h and the ./inc/TimerXA.h headers. */

/** MMAP Pin definitions. */
#define PF0   (*((volatile uint32_t *)0x40025004)) // Left Button
#define PF1   (*((volatile uint32_t *)0x40025008)) // RED LED
#define PF2   (*((volatile uint32_t *)0x40025010)) // BLUE LED
#define PF3   (*((volatile uint32_t *)0x40025020)) // GREEN LED
#define PF4   (*((volatile uint32_t *)0x40025040)) // Right Button

/** Function declarations. */
/**
 * @brief DelayWait10ms delays the current process by n*10ms. Approximate.
 * 
 * @param n Number of times to delay 10ms.
 * @note Based on a 80MHz clock.
 */
void DelayWait10ms(uint32_t n);

/**
 * @brief Blocks the current process until PF4 (Left Button <=> SW1) is pressed.
 */
void Pause(void);

// float sgZ = accZ<0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
//  double angleAccX =   atan2(accY, sgZsqrt(accZaccZ + accXaccX)) RAD_2_DEG; // [-180,+180] deg
//  double angleAccY = - atan2(accX,     sqrt(accZaccZ + accYaccY)) * RAD_2_DEG; // [- 90,+ 90] deg


/** Entry point. */
int main(void) {
    DisableInterrupts();

    /* Interrupts currently being used:
        Timer0A, pri7 - RGB flashing
        Timer2A, pri4 - ESP8266 sampling
        UART0, pri7 - PC communication
        UART5 (lab4), pri2 - ESP8266 communication
    */

    /* PLL Init. */
    PLL_Init(Bus16MHz);

    /* Allow us to talk to the PC via PuTTy! Check device manager to see which
     COM serial port we are on. The baud rate is 115200 chars/s. */
    //UART_Init();
	
    /* Start up display. */

    /* Initialize all ports. */
	I2C3_Init();
    Unified_Port_Init();
    ST7735_InitR(INITR_REDTAB);
    /* Start RGB flashing. WARNING! BRIGHT FLASHING COLORS. DO NOT RUN IF YOU HAVE EPILEPSY. */
    //RGBInit();
    /* Allows any enabled timers to begin running. */


    /* Print starting message to the PC and the ST7735. */
    ST7735_FillScreen(ST7735_BLACK);
   ST7735_SetCursor(0, 0);
   ST7735_OutString(        "ECE445L Lab 3 & 4.\n"
        "Press SW1 to start.\n");
////    UART_OutString(
////        "ECE445L Lab 3 & 4.\r\n"
////        "Press SW1 to start.\r\n");
//    Pause();
		
    /* Stop RGB and turn off any on LEDs. */
    //RGBStop();
    //PF1 = 0;
    //PF2 = 0;
    //PF3 = 0;
		
//    /* Reset screen. */
//    ST7735_FillScreen(ST7735_BLACK);
//    ST7735_SetCursor(0, 0);
//    ST7735_OutString("Starting...\n");
//    UART_OutString("Starting...\r\n");

    /* Setup ESP8266 to talk to Blynk server. See blynk.h for what each field does. */
    // TODO: enable this for lab 4
    #if LAB_4
          #define USE_TIMER_INTERRUPT true
          blynk_init("EE-IOT-Platform-03", "g!TyA>hR2JTy", "1234567890", USE_TIMER_INTERRUPT);
          #undef USE_TIMER_INTERRUPT
    #endif
//	int16_t accX, accY, accZ, GyroX, GyroY, GyroZ, Temper;
//uint8_t sensordata[14];
//	float AX, AY, AZ, t, GX, GY, GZ;
	I2C3_Init();
	Delay(100);
	MPU6050_Init();
	calibirate_MPU6050(5000);
	EnableInterrupts();
	//Delay(1000);
  //uart5_init();
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_SetCursor(0, 0);
	while(1)
	{	 
		 read_MPU6050();
			// account for offsets
				GyroX -= offset_gyx;
				GyroY -= offset_gyy;
				GyroZ -= offset_gyz;
			 // Convert The Readings
			AX = (float)accX/16384.0;
			AY = (float)accY/16384.0;
			AZ = (float)accZ/16384.0;
			GX = (float)GyroX/131.0 ;
			GY = (float)GyroY/131.0 ;
			GZ = (float)GyroZ/131.0;
			float sgZ = accZ<0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
		float angleAccX =   atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG; // [-180,+180] deg
		float angleAccY = - atan2(accX,     sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG; // [- 90,+ 90] deg
		 angleAccY -= offset_angleAccY;
		angleAccX -= offset_angleAccX;
     printf("Gx = %.2f \n",GX);
    // printstring(msg);
		 printf("Gy = %.2f \n",GY);
    // printstring(msg);
		 printf("Gz  = %.2f \n",GZ);
    // printstring(msg);
		 printf("Ax  = %.2f \n",AX);
    // printstring(msg);
		 printf("Ay  = %.2f \n",AY);
     //printstring(msg);
		 printf("Az  = %.2f \r\n",AZ);
     //printstring(msg);
		 printf("angleAccX  = %.2f \n",angleAccX);
     //printstring(msg);
		 printf("angleAccY  = %.2f \r\n",angleAccY);
		
     Delay(50);
		 ST7735_FillScreen(ST7735_BLACK);
		ST7735_SetCursor(0, 0);
	}
    return 1;
}

/** Function Implementations. */
void DelayWait10ms(uint32_t n) {
    uint32_t volatile time;
    while (n){
        time = 727240 * 2 / 91;  // 10msec
        while (time){
            --time;
        }
        --n;
    }
}

void Pause(void) {
    while (PF4 == 0x00) {
        DelayWait10ms(10);
    }
    while (PF4 == 0x10) {
        DelayWait10ms(10);
    }
}

void MPU6050_Init(void)
{
 I2C3_Wr(0x68,SMPLRT_DIV, 0x07);
 I2C3_Wr(0x68,PWR_MGMT_1,  0x01);
 I2C3_Wr(0x68,CONFIG, 0x00);
 I2C3_Wr(0x68,ACCEL_CONFIG,0x00); 
 I2C3_Wr(0x68,GYRO_CONFIG,0x00);
 I2C3_Wr(0x68,INT_ENABLE, 0x01);
// setOffsets(0x68,XA_OFFS_H, (((-4738)) & 0xFF00) >> 8);
// setOffsets(0x68,XA_OFFS_L_TC, (((-4738)) & 0x00FF));
// setOffsets(0x68,YA_OFFS_H, (((-682)) & 0xFF00) >> 8);
// setOffsets(0x68,YA_OFFS_L_TC, (((-682)) & 0x00FF));
// setOffsets(0x68,ZA_OFFS_H, (((1184)) & 0xFF00) >> 8);
// setOffsets(0x68,ZA_OFFS_L_TC, (((1184)) & 0x00FF));
 setOffsets(0x68,XG_OFFS_USRH, 0x0);
 setOffsets(0x68,XG_OFFS_USRL, (82));
 setOffsets(0x68,YG_OFFS_USRH, 0x0);
 setOffsets(0x68,YG_OFFS_USRL, (((32)) & 0x00FF));
 setOffsets(0x68,ZG_OFFS_USRH, (((-20)) & 0xFF00) >> 8);
 setOffsets(0x68,ZG_OFFS_USRL, (((-20)) & 0x00FF));
}

//void uart5_init(void)
//{
//	
//	  SYSCTL->RCGCUART |= 0x20;  /* enable clock to UART5 */
//    SYSCTL->RCGCGPIO |= 0x10;  /* enable clock to PORTE for PE4/Rx and RE5/Tx */
//    Delay(1);
//    /* UART0 initialization */
//    UART5->CTL = 0;         /* UART5 module disbable */
//    UART5->IBRD = 104;      /* for 9600 baud rate, integer = 104 */
//    UART5->FBRD = 11;       /* for 9600 baud rate, fractional = 11*/
//    UART5->CC = 0;          /*select system clock*/
//    UART5->LCRH = 0x60;     /* data lenght 8-bit, not parity bit, no FIFO */
//    UART5->CTL = 0x301;     /* Enable UART5 module, Rx and Tx */

//    /* UART5 TX5 and RX5 use PE4 and PE5. Configure them digital and enable alternate function */
//    GPIOE->DEN = 0x30;      /* set PE4 and PE5 as digital */
//    GPIOE->AFSEL = 0x30;    /* Use PE4,PE5 alternate function */
//    GPIOE->AMSEL = 0;    /* Turn off analg function*/
//    GPIOE->PCTL = 0x00110000;     /* configure PE4 and PE5 for UART */
//	
//}
void I2C3_Init(void)
{
 SYSCTL_RCGCI2C_R |= 0x0008;           // activate I2C3
  SYSCTL_RCGCGPIO_R |= 0x0008;          // activate port D
  while((SYSCTL_PRGPIO_R&0x0008) == 0){};// ready?
  GPIO_PORTD_AFSEL_R |= 0x03;           // 3) enable alt funct on PD1,0
  GPIO_PORTD_ODR_R |= 0x02;             // 4) enable open drain on PD1 only
  GPIO_PORTD_DR8R_R |= 0x03;            //  high current on PD1,0
  GPIO_PORTD_DEN_R |= 0x03;             // 5) enable digital I/O on PD1,0
                                        // 6) configure PD1,0 as I2C
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFFFFF00)+0x00000033;
  GPIO_PORTD_AMSEL_R &= ~0x03;          // 7) disable analog functionality on PD1,0
  I2C3_MCR_R = I2C_MCR_MFE;             // 9) master function enable, no glitch
  I2C3_MCR2_R = I2C_MCR2_GFPW_BYPASS;   // bypass glitch
/* Configure I2C 3 clock frequency
(1 + TIME_PERIOD ) = SYS_CLK /(2*
( SCL_LP + SCL_HP ) * I2C_CLK_Freq )
TIME_PERIOD = 16 ,000 ,000/(2(6+4) *100000) - 1 = 7 */
I2C3_MTPR_R = 0x07 ;
}

/* Wait until I2C master is not busy and return error code */
/* If there is no error, return 0 */
static int I2C_wait_till_done(void)
{
    while(I2C3_MCS_R   & 1);   /* wait until I2C master is not busy */
    return I2C3_MCS_R   & 0xE; /* return I2C error code */
}

/* Write one byte only */
/* byte write: S-(saddr+w)-ACK-maddr-ACK-data-ACK-P */
char I2C3_Wr(int slaveAddr, char memAddr, char data)
{
		//I2C3_MCS_R &=  ~I2C_MCS_BUSBSY ; // fix reset bug
    char error;

    /* send slave address and starting address */
	  I2C3_MSA_R   = (slaveAddr << 1)&0xFE; // MSA[7:1] slave addr
		I2C3_MSA_R  &= ~0x01; // MSA[0] is 0 for send
    I2C3_MDR_R   = memAddr;
    I2C3_MCS_R   = (I2C_MCS_START | I2C_MCS_RUN);                      /* S-(saddr+w)-ACK-maddr-ACK */

    error = I2C_wait_till_done();       /* wait until write is complete */
    if (error) return error;

    /* send data */
    I2C3_MDR_R   = data;
    I2C3_MCS_R   = (I2C_MCS_STOP | I2C_MCS_RUN);                      /* -data-ACK-P */
    error = I2C_wait_till_done();       /* wait until write is complete */
      while(I2C3_MCS_R   & 0x40);        /* wait until bus is not busy */
     error = I2C3_MCS_R   & 0xE;
    if (error) return error;

    return 0;       /* no error */
}

char setOffsets(int slaveAddr, char memAddr, int8_t data)
{
		//I2C3_MCS_R &=  ~I2C_MCS_BUSBSY ; // fix reset bug
    char error;

    /* send slave address and starting address */
	  I2C3_MSA_R   = (slaveAddr << 1)&0xFE; // MSA[7:1] slave addr
		I2C3_MSA_R  &= ~0x01; // MSA[0] is 0 for send
    I2C3_MDR_R   = memAddr;
    I2C3_MCS_R   = (I2C_MCS_START | I2C_MCS_RUN);                      /* S-(saddr+w)-ACK-maddr-ACK */

    error = I2C_wait_till_done();       /* wait until write is complete */
    if (error) return error;

    /* send data */
    I2C3_MDR_R   = data;
    I2C3_MCS_R   = (I2C_MCS_STOP | I2C_MCS_RUN);                      /* -data-ACK-P */
    error = I2C_wait_till_done();       /* wait until write is complete */
      while(I2C3_MCS_R   & 0x40);        /* wait until bus is not busy */
     error = I2C3_MCS_R   & 0xE;
    if (error) return error;

    return 0;       /* no error */
}
char I2C3_Rd(int slaveAddr, char memAddr, int byteCount, uint8_t* data)
{
     char error;
    
    if (byteCount <= 0)
        return -1;         /* no read was performed */

    /* send slave address and starting address */
    I2C3_MSA_R   = (slaveAddr << 1)&0xFE; // MSA[7:1] slave addr
		I2C3_MSA_R  &= ~0x01; // MSA[0] is 0 for send
		I2C3_MDR_R   = memAddr;
    I2C3_MCS_R   = (I2C_MCS_START | I2C_MCS_RUN);        /* S-(saddr+w)-ACK-maddr-ACK */
    error = I2C_wait_till_done();
    if (error)
        return error;

    /* to change bus from write to read, send restart with slave addr */
		I2C3_MSA_R   = (slaveAddr << 1)&0xFE; // MSA[7:1] slave addr
		I2C3_MSA_R  |= 0x01; // MSA[0] is 1 for rec   /* restart: -R-(saddr+r)-ACK */

    if (byteCount == 1)             /* if last byte, don't ack */
        I2C3_MCS_R   = (I2C_MCS_STOP | I2C_MCS_RUN | I2C_MCS_START);              /* -data-NACK-P */
    else                            /* else ack */
        I2C3_MCS_R   = (I2C_MCS_START | I2C_MCS_RUN);           /* -data-ACK- */
    error = I2C_wait_till_done();
    if (error) return error;

    *data++ = (I2C3_MDR_R &0xFF) ;            /* store the data received */

    if (--byteCount == 0)           /* if single byte read, done */
    {
        while(I2C3_MCS_R   & 0x40);    /* wait until bus is not busy */
        return 0;       /* no error */
    }
 
    /* read the rest of the bytes */
    while (byteCount > 1)
    {
        I2C3_MCS_R   = 9;              /* -data-ACK- */
        error = I2C_wait_till_done();
        if (error) return error;
        byteCount--;
        *data++ = (I2C3_MDR_R &0xFF) ;        /* store data received */
    }

    I2C3_MCS_R   = 5;                  /* -data-NACK-P */
    error = I2C_wait_till_done();
    *data = (I2C3_MDR_R &0xFF) ;              /* store data received */
    while(I2C3_MCS_R   & 0x40);        /* wait until bus is not busy */
    
    return 0;       /* no error */
}

void read_MPU6050(void){
	I2C3_Rd(0x68,ACCEL_XOUT_H, 2, sensordata);
 accX = ((int16_t) ( (sensordata[0] << 8 ) |sensordata[1] ));
 memset(sensordata, 0, sizeof(sensordata)); 
 I2C3_Rd(0x68,ACCEL_YOUT_H, 2, sensordata);
 accY = ((int16_t) ( (sensordata[0] << 8 ) |sensordata[1] ));
 memset(sensordata, 0, sizeof(sensordata)); 
 I2C3_Rd(0x68,ACCEL_ZOUT_H, 2, sensordata);	
 accZ = ((int16_t) ( (sensordata[0] << 8 ) |sensordata[1] ));
 memset(sensordata, 0, sizeof(sensordata)); 
 I2C3_Rd(0x68,GYRO_XOUT_H, 2, sensordata);
 GyroX = ((int16_t) ( (sensordata[0] << 8 ) |sensordata[1] ));
 memset(sensordata, 0, sizeof(sensordata)); 
 I2C3_Rd(0x68,GYRO_YOUT_H, 2, sensordata);
 GyroY = ((int16_t) ( (sensordata[0] << 8 ) |sensordata[1] ));
 memset(sensordata, 0, sizeof(sensordata)); 
 I2C3_Rd(0x68,GYRO_ZOUT_H, 2, sensordata);
 GyroZ = ((int16_t) ( (sensordata[0] << 8 ) |sensordata[1] ));
 
	
}

void calibirate_MPU6050(int num_iterations) {
	for (int i = 0; i < num_iterations; i++) {
    read_MPU6050();
    offset_gyx += GyroX;
    offset_gyy += GyroY;
    offset_gyz += GyroZ;
		float sgZ = accZ<0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
		offset_angleAccX +=  atan2(accY, sgZ*sqrt(accZ*accZ + accX*accX)) * RAD_2_DEG;
		offset_angleAccY +=  - atan2(accX,     sqrt(accZ*accZ + accY*accY)) * RAD_2_DEG;
  }

  offset_gyx /= num_iterations;
  offset_gyy /= num_iterations;
  offset_gyz /= num_iterations;
	offset_angleAccX /= num_iterations;
	offset_angleAccY /= num_iterations;
}
		
//void UART5_Transmitter(unsigned char data)  
//{
//    while((UART5->FR & (1<<5)) != 0); /* wait until Tx buffer not full */
//    UART5->DR = data;                  /* before giving it another byte */
//}

//void printstring(char *str)
//{
//  while(*str)
//	{
//		UART5_Transmitter(*(str++));
//	}
//}
void Delay(unsigned long counter)
{
	unsigned long i = 0;
	
	for(i=0; i< counter*10000; i++);
}

