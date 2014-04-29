#include "main.h"

#define READMEM 1                               /* Set to 1 if you want to read data from memory */

#define TIMETOWRITE 60
#define TIMETOREAD 60

#define PAGESTOWRITE 317 // (TIMETOWRITE * 5.5)
#define PAGESTOREAD 317 //(TIMETOREAD * 5.5)


/*
** Main Function. We setup the clocks
** and SPI, then leave the device into 
** sleep mode to be activated by the Timer Interrupt
*/
void main(void)
{
  
  /* Turn off Watchdog Timer */
  WDTCTL = WDTPW+WDTHOLD;                   

  BCSCTL3 |= XCAP_3;          /* Set the capacitance of the external crystal to 12.4pf. */  
  
   /* Initialize DCO */
  BCSCTL1 = CALBC1_1MHZ;                /* Set DCO to 1MHz */
  DCOCTL =  CALDCO_1MHZ;                /* Set DCO to 1MHz */
  __delay_cycles(200000);	// Delay 0.2 s to let clocks settle
  
  /* Enable and Turn LED ON */
  P1DIR |=LED;                          /* Set LED pin as Output */        
  P1OUT |=LED;                          /* Set LED pin as HIGH */
  
  /* Enable SPI */
  P5SEL |= (BIT1 + BIT2 + BIT3);        /* Peripheral function instead of I/O */
  UCB1CTL0 = UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC; /* SPI Polarity = 1, MSB First, Master, 3 Pin Mode, Synchronous Comm. UCCKPH = 0 is ~SPIPhase*/
  UCB1CTL1 = UCSSEL_2 | UCSWRST;         /* Clock from SMCLK; hold SPI in reset */
  
  UCB1BR1 = 0x0;                          /* Upper byte of divider word */
  UCB1BR0 = 0x1;                         /* Clock = SMCLK / 10 = 100 KHz */
  
  UCB1CTL1 &= ~UCSWRST;                 /* Remove SPI reset to enable it*/
  
  /* Configure Pins for SPI. These commands might not be needed */
  P5DIR |= nSS | mSS | SPI_SOMI | SPI_CLK;    /* Output on the pins */      
  P5OUT |= nSS | mSS | SPI_SOMI | SPI_CLK;    /* Set pins to high */
  __delay_cycles(0x80);
  
  
  /* Read the Memory Device ID. Should be 1F */
  read = Mem_ReadID();
  
  /* This Part of the Program executes only if READMEM is 1 */
  if(READMEM)
  {
    Mem_ReadAllBinary();
  }
  
  /* Disable I2C on the sensor */
  _Sensor_write(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
  
  /* Check the Sensor Device ID */
  read = _Sensor_read(MPUREG_WHOAMI);
  
  /* Trap uC if Sensor doesn't ID correctly */
  if(read != 0x68)  JustDance();                    /* Check WHOAMI Hopefully it is 0x68. Otherwise freak out. */

  /* Sensor might be sleeping, read Register with Sleep Bit, reset the bit and write it back */
  read = _Sensor_read(MPUREG_PWR_MGMT_1);
  _Sensor_write(MPUREG_PWR_MGMT_1, (read & ~BIT_SLEEP));/* Reset the Sleep Bit to wake it up */
  
  /* Erase all data SensorData */
  for(ctr = 0; ctr < PAGESIZE; ctr++)
    SensorData[ctr] = 0;

  
  /* Reset Counter to 0 after previous for loop */
  ctr = 0;
  
  /* 
  ** TimerA for Blinking, Button Debouncing, Long Press wait 
  ** TACTL = Timer A Control
  ** TASSEL_1 => Set Timer source to ACLK (ACLK is from LFXT) | MC_2 = MODE => Continuous
  ** TACLR => Clear Timer | ID_3 Divide Clock by 8 = > 4096 = 0x1000
  ** This does not start the timer (Set mode to not 0, and have not zero value in TACCR0)).
  */
  TACTL = TACLR;
  __delay_cycles(8254); /* Delay by 0.2 seconds DELAY CYCLES USES DCO @ 1MHZ*/
  TACTL = TASSEL_1 | MC_2;
  
    /* Other Housekeepigng */
  __delay_cycles(8234);                 /* Delay 0.2s at 1Mhz to let clock settle */
  __enable_interrupt();
  
  /* Set timer to run */
  TACCR0 = BaseTime;
  TACCTL0 |= CCIE;

  /** Main Program **/
  
  while(1)
  {
    __low_power_mode_3();
    
    if(ActionMode == 1)                 /* Long Buttton Press from old Timer Code */
    {
      ActionMode = 0;  
    }
    
    if(ActionMode == 2)                 /* Short Button Press from old Timer Code */
    {
      ActionMode = 0;
    }    
    
    if(ActionMode == 3)                  /* Timer A Freq at 15hz */
    {
      ReadingSensor = 1;
      
      if(CurrentPage == PAGESTOWRITE && ctr > 1007) /* 2nd to Last data should be -128, Last Should be 0 */
      {
      SensorData[ctr++] = -128;
      SensorData[ctr++] = -128;
      SensorData[ctr++] = -128;
      SensorData[ctr++] = -128;
      SensorData[ctr++] = -128;
      SensorData[ctr++] = -128;
      JustDance();
      }
      else
      {
      /* Read X Acc */
      SensorData[ctr++] = _Sensor_read(MPUREG_ACCEL_XOUT_H); // rand--;//
      /* Read Y Acc */
      SensorData[ctr++] =  _Sensor_read(MPUREG_ACCEL_YOUT_H);
      /* Read Z Acc */
      SensorData[ctr++] =  _Sensor_read(MPUREG_ACCEL_ZOUT_H);
      /* Read X Gyro */
      SensorData[ctr++] =  _Sensor_read(MPUREG_GYRO_XOUT_H);
      /* Read Y Gyro */
      SensorData[ctr++] = _Sensor_read(MPUREG_GYRO_YOUT_H);
      /* Read Z Gyro */
      SensorData[ctr++] = _Sensor_read(MPUREG_GYRO_ZOUT_H);
      }
      

          if(ctr >= PAGESIZE)   /* The Buffer is full */
          {
            __disable_interrupt();	/* Disable interrupts b/c the timer might interrupt otherwise */
            StartTime = TAR;
            Mem_WriteToBuffer();	/* Write the stored SensorData to the Memory chip Buffer */
            Mem_BufferToPage();		/* Write the Buffer data to Page. Page Number is stored in Global CurrentPage */
            CurrentPage++;			/* Increment current page for next write */
            ctr=0;					/* Reset ctr so its starts at 0 for next page */
            EndTime = TAR;
            Time = EndTime - StartTime;
            for(int actr = 0; actr < PAGESIZE; actr++)	/* Erase SensorData. Using new ctr variable b/c ctr is used in loop above */
                SensorData[actr] = 0;
            ReadingSensor = 0; /* Flag set to 0 b/c we're done with communication */
            ActionMode = 0;
            
            __enable_interrupt();	/* Enable Interrupts for the timer to start logging data again */

          }
      
      ReadingSensor = 0; /* Flag set to 0 b/c we're done with communication */
      ActionMode = 0;
    }
    
    
  }
  
}


/**********************/
/* Functions for Main */
/**********************/


/* Interrupt for TimerA Channel 0, runs short periodical*/
#pragma vector = TIMERA0_VECTOR
__interrupt void TA0V_ISR(void)
{
  __disable_interrupt();
  BaseTime ^= 0x01;             /* Flip between 2184 and 2185 to average at 2184.5 */
  TACCR0 += (BaseTime); /* Next Interrupt at given time */
  ActionMode = 3;
  if(ReadingSensor == 1)
  {
   JustDance(); 
  }
  __low_power_mode_off_on_exit();

  __enable_interrupt();
}


void JustDance()
{
  P1OUT &= ~LED;                    /* Turn LED off if the WRONG WHO_AM_I response is received */
    while(1)                          /* Trap device and give error code */
    {
      for(int j = 0; j<0x05; j++) for(int i = 0; i<0xFFFF; i++);        /* Long LED off delay */
      P1OUT ^= LED;
      for(int j = 0; j<0x05; j++) for(int i = 0; i<0xFFFF; i++);        /* Long LED on */
      P1OUT ^= LED;
      for(int i = 0; i<0x8FFF; i++);        /* Short LED off */
      P1OUT ^= LED;
      for(int i = 0; i<0x8FFF; i++);        /* Short LED on */
      P1OUT ^= LED;
      for(int i = 0; i<0x8FFF; i++);        /* Short LED off */
      P1OUT ^= LED;
      for(int i = 0; i<0x8FFF; i++);        /* Short LED on */
      P1OUT ^= LED;
      for(int i = 0; i<0x8FFF; i++);        /* Short LED off */
      P1OUT ^= LED;
      for(int i = 0; i<0x8FFF; i++);        /* Short LED on */
      P1OUT ^= LED;
      for(int j = 0; j<0x05; j++) for(int i = 0; i<0xFFFF; i++);        /* Long LED on delay */
      P1OUT ^= LED;                                                     
    }
  
}


/*********************/
/* Code for MPU 6000 */
/*********************/

unsigned char _Sensor_write(unsigned char add, unsigned char val)
{
  P5OUT |= mSS;                         /* Deselect Select Memory as SPI slave */  
  P5OUT &= ~nSS;                        /* Select Sensor as SPI Slave */
  unsigned char RXCHAR = 0x00;
  
  RXCHAR = Sensor_TXRX(add, val);
  
  P5OUT |= nSS;                         /* Deselect Sensor as SPI slave */        
  return RXCHAR;
}

unsigned char _Sensor_read(unsigned char add)
{
  
  unsigned char RXCHAR = 0x00;
  P5OUT |= mSS;                         /* Deselect Select Memory as SPI slave */  
  P5OUT &= ~nSS;                        /* Select Sensor as SPI Slave */
  
  RXCHAR = Sensor_TXRX(add | MPU_READ, 0x00);
  
  P5OUT |= nSS;                         /* Deselect Sensor as SPI slave */   
  return RXCHAR;
}

unsigned char Sensor_TXRX(unsigned char add, unsigned char val)
{
  unsigned char RXCHAR = 0x00;
          
  while (!(UC1IFG & UCB1TXIFG));          /* Wait for TXBUF to be empty */
  
  UCB1TXBUF = add;                      /* Send Address of Register  */
  while(!(UC1IFG & UCB1TXIFG));           /* Wait for TXBUF to be empty (TXBUF data moves to the shift register) */
  while(!(UC1IFG & UCB1RXIFG));           /* Wait for RXBUF to be full */     
  RXCHAR = UCB1RXBUF;                   /* Read what is RX to clear buffer / flags*/
  
  
  UCB1TXBUF = val;                      /* Write the val */      
  while(!(UC1IFG & UCB1TXIFG));           /* Wait for TXBUF to be empty (TXBUF data moves to the shift register) */
  while(!(UC1IFG & UCB1RXIFG));           /* Wait for RXBUF to be full */  
  RXCHAR = UCB1RXBUF;                     /* Read what is RX to clear buffer / flags*/
        

  
  return RXCHAR;
}


/**********************/
/* Code for Dataflash */
/**********************/
unsigned char MEM_TXRX(unsigned char data)
{
  unsigned char RXCHAR = 0x00;
  while (!(UC1IFG & UCB1TXIFG));          /* Wait for TXBUF to be empty */
  
  UCB1TXBUF = data;                      /* Send Address of Register  */
  while(!(UC1IFG & UCB1TXIFG));           /* Wait for TXBUF to be empty (TXBUF data moves to the shift register) */
  while(!(UC1IFG & UCB1RXIFG));           /* Wait for RXBUF to be full */     
  RXCHAR = UCB1RXBUF;                   /* Read what is RX to clear buffer / flags*/
  return RXCHAR;
  
}

unsigned char Mem_ReadID()
{
  unsigned char RXCHAR = 0x00;
  P5OUT |= nSS;                         /* Deselect Sensor as SPI slave */ 
  P5OUT |= mSS;                         /* Deselect memory as SPI slave */ 
  P5OUT &= ~mSS;                        /* Select Memory as SPI Slave */
  
  MEM_TXRX(0x9F);                       /* Send the Buffer OpCode to the Memory */
  RXCHAR = MEM_TXRX(0x00);                       /* Write 3 bytes of address. We starts at byte 0, so this is always 0 */
  RXCHAR = RXCHAR;
  
  P5OUT |= mSS;                         /* Deselect memory as SPI slave */ 
  return RXCHAR;
}

void Mem_WriteToBuffer()
{
  P5OUT |= nSS;                         /* Deselect Sensor as SPI slave */ 
//  __delay_cycles(100);
  P5OUT |= mSS;                         /* Deselect memory as SPI slave */ 
//  __delay_cycles(1);
  P5OUT &= ~mSS;                        /* Select Memory as SPI Slave */
//  __delay_cycles(100);
  MEM_TXRX(Buf1Write);                  /* Send the Buffer OpCode to the Memory */
  MEM_TXRX(0x00);                       /* Write 3 bytes of address. We starts at byte 0, so this is always 0 */
  MEM_TXRX(0x00);
  MEM_TXRX(0x00);
  
  for(ctr = 0; ctr < PAGESIZE; ctr++)
    MEM_TXRX(SensorData[ctr]);

  
  while(UCB1STAT & UCBUSY);             /* Wait for SPI to complete communication. Shouldn't be needed */
  P5OUT |= mSS;                         /* Deselect memory as SPI slave */ 
  __delay_cycles(100);
}

void Mem_ReadFromBuffer()
{
  P5OUT |= nSS;                         /* Deselect Sensor as SPI slave */ 
  P5OUT |= mSS;                         /* Deselect memory as SPI slave */ 
  P5OUT &= ~mSS;                        /* Select Memory as SPI Slave */
  
  MEM_TXRX(Buf1Read);                  /* Send the Buffer OpCode to the Memory */
  MEM_TXRX(0x00);                      /* Don't Care Bytes */
  MEM_TXRX(0x00);                      /* Upper and lower Byter. We starts at byte 0, so this is always 0 */
  MEM_TXRX(0x00);                      /* Lower Byte */
  
  
  for(ctr = 0; ctr < PAGESIZE; ctr++)
      SensorData[ctr] = MEM_TXRX(0x00);

  while(UCB1STAT & UCBUSY);             /* Wait for SPI to complete communication. Shouldn't be needed */
  P5OUT |= mSS;                         /* Deselect memory as SPI slave */ 
  
}

void Mem_BufferToPage()
{
  
  PageAddress_H = (unsigned char) (((CurrentPage<<2) & 0xFF00)>>8);
  PageAddress_L = (unsigned char) (((CurrentPage<<2) & 0xFF));
  
  P5OUT |= nSS;                         /* Deselect Sensor as SPI slave */ 
  P5OUT |= mSS;                         /* Deselect memory as SPI slave */ 
  P5OUT &= ~mSS;                        /* Select Memory as SPI Slave */
  
  MEM_TXRX(Buf1ToFlashWE);                  /* Send the Buffer OpCode to the Memory */
  MEM_TXRX(PageAddress_H);                      /* Don't Care Bytes */
  MEM_TXRX(PageAddress_L);                      /* Upper and lower Byter. We starts at byte 0, so this is always 0 */
  MEM_TXRX(0x00);                      /* Lower Byte */
  
  P5OUT |= mSS;                         /* Deselect memory as SPI slave */ 
}

void Mem_ReadFromMem()
{
  PageAddress_H = (unsigned char) (((CurrentPage<<2) & 0xFF00)>>8);
  PageAddress_L = (unsigned char) (((CurrentPage<<2) & 0xFF));
  
  P5OUT |= nSS;                         /* Deselect Sensor as SPI slave */ 
  P5OUT |= mSS;                         /* Deselect memory as SPI slave */ 
  P5OUT &= ~mSS;                        /* Select Memory as SPI Slave */
  
  MEM_TXRX(0xD2);                  /* Send the Buffer OpCode to the Memory */
  MEM_TXRX(PageAddress_H);                      /* Don't Care Bytes */
  MEM_TXRX(PageAddress_L);                      /* Upper and lower Byter. We starts at byte 0, so this is always 0 */
  MEM_TXRX(0x00);                      /* Lower Byte and Add*/

  MEM_TXRX(0x00);                       /* Dummy Byte */
  MEM_TXRX(0x00);                       /* Dummy Byte */
  MEM_TXRX(0x00);                       /* Dummy Byte */
  MEM_TXRX(0x00);                       /* Dummy Byte */
  
  for(ctr = 0; ctr < PAGESIZE; ctr++)
      SensorData[ctr] = MEM_TXRX(0x00);
  
  while(UCB1STAT & UCBUSY);             /* Wait for SPI to complete communication. Shouldn't be needed */
  P5OUT |= mSS;                         /* Deselect memory as SPI slave */ 
  
}


/* Old function used when READMEM */
void Mem_ReadAllBinary()
{

    /* Initialize UART */
    P3DIR = TXPIN;
    P3OUT = TXPIN;
    P3SEL = TXPIN | RXPIN;
    
    /* Values from MSP430x24x Demo - USCI_A0, 115200 UART Echo ISR, DCO SMCLK */
    
    UCA1CTL1 |= UCSSEL_2;                     // CLK = ACLK
    UCA1BR0 = 8;                           // 32kHz/9600 = 3.41
    UCA1BR1 = 0x00;
    UCA1MCTL = UCBRS2 + UCBRS0;               // Modulation UCBRSx = 3
    UCA1CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
    /* End UART init */
 
    for(CurrentPage = 0; CurrentPage < PAGESTOREAD ; CurrentPage++)
    {
      for(ctr = 0; ctr < PAGESIZE; ctr++)
          SensorData[ctr] = 0;
      
      Mem_ReadFromMem();
      
      for(ctr = 0; ctr < 170; ctr++)
        for(int ctr2 = 0; ctr2 < 6; ctr2++)
		UART_SendChar(SensorData[(ctr*6)+ctr2]);
    }
    
    while(UCA1STAT & UCBUSY);
    
    UCA1CTL1 |= UCSWRST;
	
    
    JustDance();              // Trap program
}



void UART_SendChar(unsigned char data)
{ 
  while(!(UC1IFG & UCA1TXIFG));
  UCA1TXBUF = data;  
}
