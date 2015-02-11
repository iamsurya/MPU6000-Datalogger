/* Define functions */
#include <msp430f248.h>
#include "mpu6000.h"
#include "dataflash.h"

/* General Defines */
#define         LED             BIT0
#define         BTN             BIT4            /* Button on P1.4 */

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
#define RXPIN   BIT5
#define TXPIN   BIT4
#define RS232_ESC       27
#define ASCII0  0x30

/* Variables for Button Debouncing */
unsigned char debounce;              /* 1 Not waiting for Debounce Timer 0 Button was pressed very soon */


/* Variables for SPI */
unsigned char read = 0;
unsigned char PageAddress_H = 0;
unsigned char PageAddress_L = 0;

/* Variables for Main Program and to store Data */
unsigned long TimeStamp = 0x00;
unsigned char ActionMode = 0;       /* ActionModes: 0 = LPM3, 1 = Switch between Blink / No Blink modes, 2 = Switch Blink Frequencies, 3 = SPI reading*/
signed char SensorData[PAGESIZE]; 	/* Stores data to output to Memory. Size of Buffer and Block on Memory is PAGESIZE bytes by default */
unsigned int CurrentPage = 0;		/* The current page number we are reading or writing */
unsigned int ctr = 0;		/* Variable used for counters. Being Lazy */
unsigned char WritingMode = 0; /* 1 = Writing Sensor Data to Memory, 0 = Not Writing Data */
unsigned char SwitchOn = 0; /* 1 = Writing Sensor Data to Memory, 0 = Not Writing Data */
unsigned char FORCESTOP = 0; 
void ONDANCE();
void OFFDANCE();

/* Timer for reading sensors */
unsigned int BaseTime = 2184; 	   /* Blinking frequency / timer on startup 0x1000 is 1 second */
unsigned char ReadingSensor = 0x00; /* Flag is set if Timer interrupts an SPI operation */

/* Variables to store Timer Counts to evaluate communication times */
unsigned int StartTime = 0;
unsigned int EndTime = 0;
unsigned int Time = 0;

/* Variables for logging to UART */
unsigned long ReadIndex = 0;
char PhoneViewStart[] = "START 2014-21-04   07:30:20";
char PhoneViewEnd[] = "END 2014-21-04   20:05:05";
unsigned char UART_Working = 0;
unsigned char UART_data[6];

/* Variables for UART interrupt */
unsigned char length;

/* Functions */

/* Sensor */
unsigned char Sensor_TXRX(unsigned char add, unsigned char val);
unsigned char _Sensor_write(unsigned char add, unsigned char val);
unsigned char _Sensor_read(unsigned char add);

/* General */
void JustDance();
void ReadPageNumberFromFlash();
void WritePageNumberToFlash();

/* Memory */
unsigned char MEM_TXRX(unsigned char data);
unsigned char Mem_ReadID();
void FlushMemory(void);
void Mem_WriteToBuffer();
void Mem_ReadFromBuffer();
void Mem_ReadFromMem();
void Mem_BufferToPage();

void Mem_ReadAllBinary();

/* UART Functions */

void ClearScreen();
void UART_SendValue(signed char);
void UART_SendChar(unsigned char);
void UART_SendIndex(unsigned long num);
void UART_SendTime(unsigned long index);
void UART_SendValue2(unsigned char num);

void UART_SendValue2(unsigned char num)
{
  unsigned char p = 0;
  p = (unsigned char) num / 10;
  UART_SendChar( p + ASCII0 );
  num = num - (p * 10);
  p = (unsigned char) num;
  UART_SendChar( p + ASCII0 );  
}

/******************************************/
/*              Unwated Fn's              */
//
//
//
//void UART_SendValue(signed char num)
//{
//  unsigned char p = 0;
//  
//  if(num < 0)   /* If Number is negative print negative sign and make num positive */
//  {
//    UART_SendChar('-');
//    num = -1 * num;
//  }
//  else
//  {
//    UART_SendChar(' ');
//    
//  }
//
//  p = (unsigned char) num / 100;
//
//  UART_SendChar( p + ASCII0 );
//  num = num - (p * 100);
//  p = (unsigned char) num / 10;
//  UART_SendChar( p + ASCII0 );
//  num = num - (p * 10);
//  p = (unsigned char) num;
//  UART_SendChar( p + ASCII0 );  
//  
//  
//}
//

//
//void UART_SendIndex(unsigned long num)
//{
//  unsigned char p = 0;
//  unsigned char ZeroLocation = 0;
//  
//  p = (unsigned char) (num / 10000);
// 
//  
//  UART_SendChar( p == 0 ? ' ' : (p + ASCII0) );
//  num = num - (p * 10000);
//  if( p != 0) ZeroLocation = 5;
//  
//  p = (unsigned char) (num / 1000);
//  
//  UART_SendChar((p == 0 && ZeroLocation <5) ? ' ' : (p + ASCII0) );
//  num = num - (p * 1000);  
//  if( p != 0) ZeroLocation = 4;
//  
//  p = (unsigned char) (num / 100);
//  UART_SendChar((p == 0 && ZeroLocation <4 )? ' ' : (p + ASCII0) );
//  num = num - (p * 100);
//  if( p != 0) ZeroLocation = 3;
//  
//  p = (unsigned char) (num / 10);
//  UART_SendChar((p == 0 && ZeroLocation <3 ) ? ' ' : (p + ASCII0 ));
//  num = num - (p * 10);
//  if( p != 0) ZeroLocation = 2;
//  
//  p = (unsigned char) num;
//  UART_SendChar((p == 0 && ZeroLocation <2 ) ? ' ' : (p + ASCII0 ));  
//  
//  
//}
//
//void UART_SendTime(unsigned long index)
//{
//  float time = index * 0.07;
//  unsigned long time_seconds = (unsigned long) floor(time);
//  time = time - time_seconds;
//  time = time * 100;
//  
//  UART_SendIndex(time_seconds);
//  UART_SendChar('.');
//  time_seconds = (unsigned long) time;
//  UART_SendValue2((unsigned char) time_seconds);
//  
//}
//
//void ClearScreen()
//{
//    UART_SendChar(RS232_ESC);
//    UART_SendChar('[');
//    UART_SendChar('2');
//    UART_SendChar('J');
//    
//    UART_SendChar(RS232_ESC);
//    UART_SendChar('[');
//    UART_SendChar('H');
//}
