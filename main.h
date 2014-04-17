/* Define functions */
#include <msp430f248.h>
#include "mpu6000.h"
#include "dataflash.h"

#define         LED             BIT0

#define         nSS             BIT0    /* Enable Pin for Sensor */
#define         SPI_SIMO        BIT1
#define         SPI_SOMI        BIT2
#define         SPI_CLK         BIT3
#define         mSS             BIT4    /* Enable Pin for Memory Chip */

#define         MPU_READ        0x80


unsigned char Sensor_TXRX(unsigned char add, unsigned char val);

unsigned char _Sensor_write(unsigned char add, unsigned char val);
unsigned char _Sensor_read(unsigned char add);


unsigned char _Memory_write(unsigned char add, unsigned char val);
unsigned char _Memory_read(unsigned char add);

void JustDance();

unsigned char MEM_TXRX(unsigned char data);
unsigned char Mem_ReadID();
void FlushMemory(void);
void Mem_WriteToBuffer();
void Mem_ReadFromBuffer();
void Mem_ReadFromMem();
void Mem_BufferToPage();
void Mem_ReadAll();