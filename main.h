/* Define functions */
#include <msp430f248.h>
#include "mpu6000.h"
#include "dataflash.h"

/* General Defines */
#define         LED             BIT0

/* SPI Pin defines */
#define         nSS             BIT0    /* Enable Pin for Sensor */
#define         SPI_SIMO        BIT1
#define         SPI_SOMI        BIT2
#define         SPI_CLK         BIT3
#define         mSS             BIT4    /* Enable Pin for Memory Chip */

/* SPI commands / vars */
#define         MPU_READ        0x80
#define PAGESIZE 1020 				/* PageSize for current Memory is set to 1024, but 6 * 170 = 1020, so we use this for all counters */

/* UART Pins and vars */
#define RXPIN   BIT7
#define TXPIN   BIT6
#define RS232_ESC       27
#define ASCII0  0x30


/* Variables for SPI */
unsigned char read = 0;
unsigned char PageAddress_H = 0;
unsigned char PageAddress_L = 0;

/* Variables for Main Program and to store Data */
unsigned char ActionMode = 0;       /* ActionModes: 0 = LPM3, 1 = Switch between Blink / No Blink modes, 2 = Switch Blink Frequencies, 3 = SPI reading*/
signed char SensorData1[PAGESIZE]; 	/* Stores data to output to Memory. Size of Buffer and Block on Memory is PAGESIZE bytes by default */
signed char SensorData2[PAGESIZE]; 	/* Data received from Memory */
unsigned int CurrentPage = 0;		/* The current page number we are reading or writing */
unsigned int ctr = 0;		/* Variable used for counters. Being Lazy */

/* Timer for reading sensors */
unsigned int BaseTime = 273; 	   /* Blinking frequency / timer on startup 0x1000 is 1 second */
unsigned char ReadingSensor = 0x00; /* Flag is set if Timer interrupts an SPI operation */

/* Variables to store Timer Counts to evaluate communication times */
unsigned int StartTime = 0;
unsigned int EndTime = 0;
unsigned int Time = 0;

/* Variables for logging to UART */
unsigned long ReadIndex = 0;
char PhoneViewStart[] = "START 2014-21-04   07:30:20";
char PhoneViewEnd[] = "END 2014-21-04   20:05:05";


/* Functions */

/* Sensor */
unsigned char Sensor_TXRX(unsigned char add, unsigned char val);
unsigned char _Sensor_write(unsigned char add, unsigned char val);
unsigned char _Sensor_read(unsigned char add);

/* General */
void JustDance();

/* Memory */
unsigned char MEM_TXRX(unsigned char data);
unsigned char Mem_ReadID();
void FlushMemory(void);
void Mem_WriteToBuffer();
void Mem_ReadFromBuffer();
void Mem_ReadFromMem();
void Mem_BufferToPage();
void Mem_ReadAll();

/* UART Functions */

void ClearScreen();
void UART_SendValue(signed char);
void UART_SendChar(unsigned char);
void UART_SendIndex(unsigned long num);
void UART_SendTime(unsigned long index);
void UART_SendValue2(unsigned char num);