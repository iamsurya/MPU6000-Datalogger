#include "main.h"

#define READMEM 0                               /* Set to 1 if you want to read data from memory */

#define TIMETOWRITE 60
#define TIMETOREAD 60

#define PAGESTOWRITE 8189 
#define PAGESTOREAD 8189 


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
  __delay_cycles(200000);
  debounce = 1;
  
  FCTL2 = FWKEY + FSSEL0 + FN1;         /* MCLK/3 for Flash Timing Generator (0x0040u)  | Flash clock select: 1 - MCLK | Divided by 2. */
  SVSCTL = 0xA0;                       /* SVS to set at 3.05V */
  
  /* Initialize Port 1 For LED and Button */
  P1DIR |= LED;                 /* Set LED as output */
  P1REN |= BTN;                 /* Enable Pullup / Pulldown on BTN pin */
  P1OUT |= BTN;           /* Turn LED OFF Till the BTN is pressed, Pull up BTN pin */
  P1OUT &= ~LED;            
  P1IFG = 0;                    /* Clear any interrupts on P1 */
  P1IE  |= BTN;                 /* Set BTN as intterupt (default P1IES sets low to high )*/
  P1IES |= BTN;                 /* Sensitive to (H->L) */
  
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

//  Mem_ReadAllBinary();
  /* Initialize UART */
  P3DIR = TXPIN;
  P3OUT = TXPIN;
  P3SEL = TXPIN | RXPIN;

  /* Values from MSP430x24x Demo - USCI_A0, 115200 UART Echo ISR, DCO SMCLK */
  
  UCA0CTL1 |= UCSSEL_2;                     // CLK = SMCLK
  UCA0BR0 = 8;                           // 32kHz/9600 = 3.41
  UCA0BR1 = 0x00;
  UCA0MCTL = UCBRS2 + UCBRS0;               // Modulation UCBRSx = 3
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
  /* End UART init */

  /* Disable I2C on the sensor */
  _Sensor_write(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
  
  /* Check the Sensor Device ID */
  read = _Sensor_read(MPUREG_WHOAMI);

  /* Trap uC if Sensor doesn't ID correctly */
  if(read != 0x68) JustDance();                    /* Check WHOAMI Hopefully it is 0x68. Otherwise freak out. */

 
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
  TACTL = TASSEL_1 | MC_2; /* Run Timer on ACLK Continuous mode.*/
  
  /* 
  ** TimerB for Time keeping, tracks the time on the microcontroller 
  ** This does not start the timer (Set mode to not 0, and have not zero value in TACCR0)).
  ** The Timer is started whenever time is synced with the device */
  
  
  TBCTL = TBCLR;
  __delay_cycles(8254); /* Delay by 0.2 seconds DELAY CYCLES USES DCO @ 1MHZ*/
  TBCTL = TBSSEL_1 | MC_2 ; /* Run Timer on ACLK, Continuous mode. */ 

  
    /* Other Housekeepigng */
  __delay_cycles(8234);                 /* Delay 0.2s at 1Mhz to let clock settle */
  __enable_interrupt();

  
  OFFDANCE();

  /** Main Program **/
  
  while(1)
  {
    __low_power_mode_3();
    
    if(ActionMode == 1)                 /* Long Buttton Press from old Timer Code */
    {
      if(((WritingMode == 1) && ((P1IN & BTN) == 0)) || (FORCESTOP == 1))
      {
        /* Disable Sensor Reading */
        /* Set timer to disable */
        FORCESTOP = 0;
        TACCR0 = 0;
        TACCTL0 &= ~CCIE;
        OFFDANCE();
        WritingMode = 0;
        SwitchOn = 0;
        WriteTimeStampToMemory();
        __enable_interrupt();
        __delay_cycles(600);
      }
      else if((WritingMode == 0) && ((P1IN & BTN) == 0))
      {
        CurrentPage = 0;                /* Reset Memory count since device is off and LongPress */
        ResetTimeStampFromFlash();
        /* You don't actually erase the memory, just the number of pages you've recorded and the TimeStampData */
        WritePageNumberToFlash();
        ctr = 0;
        JustDance();
        SwitchOn = 0;
        
      }
      else if (SwitchOn == 1)      /* Device needs to be switched on */
      {
        TACCR0 += (BaseTime);
        TACCTL0 |= CCIE;
        ONDANCE();
        WriteTimeStampToMemory();
        P1OUT &= ~LED;
        WritingMode = 1;
        SwitchOn = 0;
      }
      else if (RecordTimeStamp == 1)
      {
        WriteTimeStampToMemory();
        
        
      }
      ActionMode = 0;  
    }
    
    if(ActionMode == 2)                 /* Short Button Press Action Mode. Understand that the long timer will fire after this, and that means Action Mode 1 will be activated too. */
    {
      if(WritingMode == 0)              /* Device is off and Button was shortpress */
      {
        SwitchOn = 1;                     /* Turn the polling timer on */
        RecordTimeStamp = 0;
      }
      /* Maybe you want to move thie to Action Mode 1, just like the Switching on process is    */
      /*                                                                                        */
      /*                                                                                        */
      /*                                                                                        */
      else if (WritingMode == 1)        /* Device is on and Button was shortpress */
      {
                                       /* Store current Timestamp as marker */
        
        SwitchOn = 0;
        RecordTimeStamp = 1;
      }
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
            P1OUT |= LED;
            ReadPageNumberFromFlash();
            StartTime = TAR;
            Mem_WriteToBuffer();	/* Write the stored SensorData to the Memory chip Buffer */
            Mem_BufferToPage();		/* Write the Buffer data to Page. Page Number is stored in Global CurrentPage */
            CurrentPage++;			/* Increment current page for next write */
            WritePageNumberToFlash();
            ctr=0;					/* Reset ctr so its starts at 0 for next page */
            EndTime = TAR;
            Time = EndTime - StartTime;
            for(int actr = 0; actr < PAGESIZE; actr++)	/* Erase SensorData. Using new ctr variable b/c ctr is used in loop above */
                SensorData[actr] = 0;
            ReadingSensor = 0; /* Flag set to 0 b/c we're done with communication */
            ActionMode = 0;
            P1OUT &= ~LED;
            __enable_interrupt();	/* Enable Interrupts for the timer to start logging data again */

          }
      
      ReadingSensor = 0; /* Flag set to 0 b/c we're done with communication */
      ActionMode = 0;
    }
    
    
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
 // while(UCB1STAT & UCBUSY);             /* Wait for SPI to complete communication. Shouldn't be needed */
  RXCHAR = UCB1RXBUF;                   /* Read what is RX to clear buffer / flags*/
  
  
  UCB1TXBUF = val;                      /* Write the val */      
  while(!(UC1IFG & UCB1TXIFG));           /* Wait for TXBUF to be empty (TXBUF data moves to the shift register) */
  while(!(UC1IFG & UCB1RXIFG));           /* Wait for RXBUF to be full */  
    
  RXCHAR = UCB1RXBUF;                     /* Read what is RX to clear buffer / flags*/
        
  //while(UCB1STAT & UCBUSY);             /* Wait for SPI to complete communication. Shouldn't be needed */
  
  return RXCHAR;
}


/**********************/
/* Code for Dataflash */
/**********************/
unsigned char MEM_TXRX(unsigned char data)
{
  unsigned char RXCHAR = 0x00;
  while (!(UC1IFG & UCB1TXIFG));          /* Wait for TXBUF to be empty */
//  while(UCB1STAT & UCBUSY);             /* Wait for SPI to complete communication. Shouldn't be needed */
  UCB1TXBUF = data;                      /* Send Address of Register  */
  while(!(UC1IFG & UCB1TXIFG));           /* Wait for TXBUF to be empty (TXBUF data moves to the shift register) */
  while(!(UC1IFG & UCB1RXIFG));           /* Wait for RXBUF to be full */ 
//  while(UCB1STAT & UCBUSY);             /* Wait for SPI to complete communication. Shouldn't be needed */
  RXCHAR = UCB1RXBUF;                   /* Read what is RX to clear buffer / flags*/
//  while(UCB1STAT & UCBUSY);
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
  P5OUT |= mSS;                         /* Deselect memory as SPI slave */ 
  P5OUT &= ~mSS;                        /* Select Memory as SPI Slave */

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

void Mem_ReadFromMem(unsigned int PageToRead)
{
  PageAddress_H = (unsigned char) (((PageToRead<<2) & 0xFF00)>>8);
  PageAddress_L = (unsigned char) (((PageToRead<<2) & 0xFF));
  
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


/* Function that Dumps data to Computer over UART */
void Mem_ReadAllBinary()
{
   /* Send the string nc to imply Number Correct */
    UART_SendChar('n');
    UART_SendChar('c');
    
    
    /* Send the actual Number of Pages */
    /* I use ***ReadingSensor*** here because I needed */
    /* an unsigned char and did not want to create a new one */
    ReadingSensor = CurrentPage & 0xFF;
    UART_SendChar(ReadingSensor);
    ReadingSensor = ((CurrentPage & 0xFF00) >> 8 );
    UART_SendChar(ReadingSensor);
    
    UART_SendChar('S');
    UART_SendChar('T');
    UART_SendChar('A');
    UART_SendChar('R');
    UART_SendChar('T');
    UART_SendChar('\n');
    
    for(int i = 0; i < CurrentPage ; i++)
    {
      
      for(ctr = 0; ctr < PAGESIZE; ctr++)
          SensorData[ctr] = 0;
      
      Mem_ReadFromMem(i);
      
      for(ctr = 0; ctr < 170; ctr++)
        for(int ctr2 = 0; ctr2 < 6; ctr2++)
		UART_SendChar(SensorData[(ctr*6)+ctr2]);
    }
    
    UART_SendChar('D');
    UART_SendChar('A');
    UART_SendChar('T');
    UART_SendChar('E');
    UART_SendChar('N');
    UART_SendChar('D');
    
    while(UCA0STAT & UCBUSY);
    

}



void UART_SendChar(unsigned char data)
{ 
  while(!(IFG2 & UCA0TXIFG));
  UCA0TXBUF = data;  
}




/* Interrupt for TimerA Channel 0, runs short periods*/
/* This is the timer function that runs at 15Hz */
/* It calls ActionMode 3, which reads sensors */
#pragma vector = TIMERA0_VECTOR
__interrupt void PollTimer_ISR(void)
{
  __disable_interrupt();
  TACCR0 += (BaseTime); /* Next Interrupt at given time */
  BaseTime ^= 0x01;             /* Flip between 2184 and 2185 to average at 2184.5 */
  
  ActionMode = 3;

  if(ReadingSensor == 1)
   JustDance(); 

  if( (SVSCTL & SVSFG) == SVSFG)
  {
    ActionMode = 1;
    FORCESTOP = 1;
  }
  else
  {
    FORCESTOP = 0;
  }
  __low_power_mode_off_on_exit();
  P1IFG = 0;                    /* Changing P1 can change P1IFG */
  __enable_interrupt();
}

/* This function is called if LED's need to slow be blinked */
/* Called when : the device is reset */
/* The readsensor timer fires while sensors are being read */
/* We are out of memory to store more data */
void JustDance()
{
  P1OUT &= ~LED;                    /* Turn LED off if the WRONG WHO_AM_I response is received */
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
//      P1OUT ^= LED;                                                     
  P1IFG = 0;                    /* Clear any interrupts on P1 */
  P1IE  |= BTN;                 /* Set BTN as intterupt (default P1IES sets low to high )*/
  P1IES |= BTN;                 /* Sensitive to (H->L) */
  P1IFG = 0;                    /* Clear any interrupts on P1 */
}


/* Interrupt for Port 1 Button */
#pragma vector = PORT1_VECTOR
__interrupt void ButtonPress_ISR(void)
{
__disable_interrupt();  
  
  
  P1IE &= ~BTN;                         /* Temporarily disable the interrupt (This is enabled after timer compares) */
  switch(P1IFG)
  {
  case BTN:
          if(debounce == 1)
          {
              debounce = 0;             /* Set debounce to 0 to indicate that timer needs to overflow before button can be read again */        
              TACCR1 = TAR + 0x1800;    /* Timer 1 for half second later */
              TACCTL1 = CCIE;           /* Enable Interrupt */        
              TACCR2 = TAR+0x9000;      /* Timer 2 for long press */
              TACCTL2 = CCIE;           /* Enable Timer 2 interrupt */
          }
          break;
          
  default: break;
  }
  
  P1IFG = 0;                    /* Clear any interrupts on P1 */
  P1IE  |= BTN;                 /* Set BTN as intterupt (default P1IES sets low to high )*/
  P1IES |= BTN;                 /* Sensitive to (H->L) */
  P1IFG = 0;                    /* Clear any interrupts on P1 */
  __enable_interrupt();
}


/* Interrupt for TimerA Channel 1 */
/* This interrupt deals with button presses and events */
#pragma vector = TIMERA1_VECTOR
__interrupt void ButtonPressTimer_ISR(void)
{
  
__disable_interrupt();    
  
switch(TAIV)
  {
  
  case TAIV_TACCR1:                     /* half a second has passed, so process the button command */
          TACCR1 = 0;
          TACCTL1 &= ~CCIE;
          debounce = 1;                 /* half Seconds later, so we can say button has been debounced */
          ActionMode = 2;               /* Action Timer for Short Button Press */ 
          __low_power_mode_off_on_exit();
                 
          break;
          
          
  case TAIV_TACCR2:
          /* 3 Seconds later after button press. */
          /* Turn Channel 1 to off. */
          
          TACCTL2 &= ~CCIE;
          TACCR2 = 0;
          ActionMode = 1;               /* Action = Timer for Long Button Press */

          debounce = 1; /* half Seconds later, so we can say button has been debounced */
          __low_power_mode_off_on_exit();
          
            break;
  default:  break; 
  }
 
  P1IFG = 0;                    /* Clear any interrupts on P1 */
  P1IE  |= BTN;                 /* Set BTN as intterupt (default P1IES sets low to high )*/
  P1IES |= BTN;                 /* Sensitive to (H->L) */
  P1IFG = 0;                    /* Clear any interrupts on P1 */
}


/* Timer that increments time */
/* Executes every 1 second */
#pragma vector = TIMERB0_VECTOR
__interrupt void RealTimeTimer_ISR(void)
{
  __disable_interrupt(); 
  TimeStamp++;  
  TBCCR0 += TimerB1Second;
  
  __enable_interrupt();
}

/* UART Recieve Interrupt */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  
__disable_interrupt();

UART_data[0] = UART_data[1];
UART_data[1] = UART_data[2];
UART_data[2] = UART_data[3];
UART_data[3] = UART_data[4];
UART_data[4] = UART_data[5];
UART_data[5] = UCA0RXBUF;
length++;

if(length > 5)
{
// Command Send Time
if(UART_data[0] == 's')
{
  if( UART_data[1] == 'd' )
  {

    /* Reset the Received Command */
    UART_data[0] = 0x00;
    UART_data[1] = 0x00;
    UART_data[2] = 0x00;
    UART_data[3] = 0x00;
    UART_data[4] = 0x00;
    UART_data[5] = 0x00;
    length = 0;
    ReadPageNumberFromFlash();
    Mem_ReadAllBinary();
  }
}
// TC Command Time Sync
else if(UART_data[0] == 'T')
{  if(UART_data[1] == 'C') /*  Time synC with host computer */
  {
    TimeStamp = (unsigned long)UART_data[2] + ((unsigned long)UART_data[3] << 8) + ((unsigned long)UART_data[4] << 16) + ((unsigned long)UART_data[5] << 24);
    
    UART_SendChar(UART_data[2]);
    UART_SendChar(UART_data[3]);
    UART_SendChar(UART_data[4]);
    UART_SendChar(UART_data[5]);

    /* Reset the page number */
    CurrentPage = 0;                /* Reset Memory count since device is off and LongPress */
    ResetTimeStampFromFlash();
    /* You don't actually erase the memory, just the number of pages you've recorded and the TimeStampData */
    WritePageNumberToFlash();
    
    
    /* Reset the Received Command */
    UART_data[0] = 0x00;
    UART_data[1] = 0x00;
    UART_data[2] = 0x00;
    UART_data[3] = 0x00;
    UART_data[4] = 0x00;
    UART_data[5] = 0x00;
    length = 0;
    
    /* Enable Timer B, which tracks the time and its interrupt*/
    TBCCR0 += TimerB1Second;
    TBCCTL0 |= CCIE;
  }
}
// Command CT Check Time
else if (UART_data[0] == 'C')
{
  if(UART_data[1] == 'T')
  {
    /* Reset the Received Command */
    UART_data[0] = 0x00;
    UART_data[1] = 0x00;
    length = 0;
    
    /* Store data LSB first */
    UART_data[2] = (unsigned char) (TimeStamp & 0xFF);
    UART_data[3] = (unsigned char) ((TimeStamp >> 8) & 0xFF);
    UART_data[4] = (unsigned char) ((TimeStamp >> 16) & 0xFF);
    UART_data[5] = (unsigned char) ((TimeStamp >> 24) & 0xFF);
    
    /* Send data LSB first */
    UART_SendChar(UART_data[2]);
    UART_SendChar(UART_data[3]);
    UART_SendChar(UART_data[4]);
    UART_SendChar(UART_data[5]);
    __no_operation();
  }
  
  
  
}
// Command DsT for Sending Time Stamp Data
else if (UART_data[0] == 'D')
  {
    if(UART_data[1] == 's')
    {
      if(UART_data[2] == 'T')
      {
      /* Reset the Received Command */
      UART_data[0] = 0x00;
      UART_data[1] = 0x00;
      UART_data[2] = 0x00;
      UART_data[3] = 0x00;
      UART_data[4] = 0x00;
      UART_data[5] = 0x00;
      length = 0;
      
        SendTimeStamps();
        __no_operation();
      }
    
    }
    
  }
// Command RSt for Reset
else if (UART_data[0] == 'R')
  {
      if(UART_data[1] == 'S')
      {
        if(UART_data[2] == 't')
        {
          WDTCTL = WDT_MRST_0_064; while(1);  // watchdog reset, resets device. MSP430 FAQ Webpage.
          
        }
        
        
      }
    
  }
}

__enable_interrupt();

}

void SendTimeStamps()
{
  
  /* Do this for 32 values starting at Seg C Ptr */
      for(long * longptr = (long *) SEGCPTR; longptr< (long *) SEGBLAST; longptr++)
      {
      /* Store data LSB first */
      UART_data[2] = (unsigned char)  ((*longptr) & 0xFF);
      UART_data[3] = (unsigned char) (((*longptr) >> 8) & 0xFF);
      UART_data[4] = (unsigned char) (((*longptr) >> 16) & 0xFF);
      UART_data[5] = (unsigned char) (((*longptr) >> 24) & 0xFF);
      
      /* Send data LSB first */
      UART_SendChar(UART_data[2]);
      UART_SendChar(UART_data[3]);
      UART_SendChar(UART_data[4]);
      UART_SendChar(UART_data[5]);
      }
  
}
void ResetTimeStampFromFlash()
{
        /* Erase Segment B */
        CurrentTimeStampPtr = (long *) SEGBPTR;  
        FCTL3 = FWKEY;                            /* Clear Lock bit */
        FCTL1 = FWKEY + ERASE;                    /* Set Erase bit  */
        *CurrentTimeStampPtr = 0;                 /* Dummy write to erase Flash seg D */  
        FCTL1 = FWKEY;                            /* Clear WRT bit */
        FCTL3 = FWKEY + LOCK;                     /* Set LOCK bit */
        
        /* Erase Segment C */
        CurrentTimeStampPtr = (long *) SEGCPTR;  
        FCTL3 = FWKEY;                            /* Clear Lock bit */
        FCTL1 = FWKEY + ERASE;                    /* Set Erase bit  */
        *CurrentTimeStampPtr = 0;                 /* Dummy write to erase Flash seg D */  
        FCTL1 = FWKEY;                            /* Clear WRT bit */
        FCTL3 = FWKEY + LOCK;                     /* Set LOCK bit */
        
        /* The CurrentTimerStampPtr points to SEGCPTR, which is where it should be */
        
}


/** This function writes the current TimeStamp to Flash Memory */
/** Called when user taps the button while device is recording data */
void WriteTimeStampToMemory()
{
        FCTL3 = FWKEY;                            /* Clear Lock bit */
        FCTL1 = FWKEY + WRT;                      /* Set WRT bit for write operation */
        
        *CurrentTimeStampPtr++ = TimeStamp;
        if(CurrentTimeStampPtr > (long *) SEGBLAST)            /* Out of Memory for Time Stamps */
        {
          while(1)  JustDance();                  /* Keep blinking the LED */
        }
        FCTL1 = FWKEY;                            /* Clear WRT bit */
        FCTL3 = FWKEY + LOCK;                     /* Set LOCK bit */
}
                         

/* Read the current Page Number from Flash memory */
void ReadPageNumberFromFlash()
{
  char *Flash_ptr;                          /* Pointer to the flash memory */
  unsigned char LSB,MSB;
  
  Flash_ptr = (char *) SEGDPTR;             /* Initialize Flash segment C ptr */
  MSB = *Flash_ptr++;                       /*  Get Current Page number from the Flash location 0x1040 */
  LSB = *Flash_ptr;
  CurrentPage = (((unsigned int)MSB)<<8)+(unsigned int)LSB;
  
}

/* Write the current Page number to Flash Memory */
void WritePageNumberToFlash()
{
  char *Flash_ptr;                          /* Pointer to the flash memory */
  unsigned char LSB,MSB;
  Flash_ptr = (char *) SEGDPTR;         /* Initialize Flash segment D ptr */
  
  FCTL3 = FWKEY;                            /* Clear Lock bit */
  FCTL1 = FWKEY + ERASE;                    /* Set Erase bit  */
  *Flash_ptr = 0;                           /* Dummy write to erase Flash seg D */
  FCTL1 = FWKEY + WRT;                      /* Set WRT bit for write operation */
  
  MSB = (unsigned char)((CurrentPage & 0xFF00u)>>8);
  LSB = (unsigned char)(CurrentPage & 0xFF);
  *Flash_ptr++ = MSB;
  *Flash_ptr = LSB;
  
  
  
  FCTL1 = FWKEY;                            /* Clear WRT bit */
  FCTL3 = FWKEY + LOCK;                     /* Set LOCK bit */
  
}


void OFFDANCE()
{
        P1OUT &= ~LED;
        for(int j = 0; j<0x05; j++) for(int i = 0; i<0xFFFF; i++);        /* Long LED off delay */
        P1OUT ^= LED;
        for(int j = 0; j<0x05; j++) for(int i = 0; i<0xFFFF; i++);        /* Long LED on */
        P1OUT ^= LED;
        for(int j = 0; j<0x05; j++) for(int i = 0; i<0xFFFF; i++);        /* Long LED off */
        P1OUT ^= LED;
        for(int j = 0; j<0x05; j++) for(int i = 0; i<0xFFFF; i++);        /* Long LED on */
        P1OUT ^= LED;
        for(int j = 0; j<0x05; j++) for(int i = 0; i<0xFFFF; i++);        /* Long LED off */  
}


void ONDANCE()
{
  
        P1OUT &= ~LED;
        for(int i = 0; i<0x8FFF; i++);        /* Long LED off delay */
        P1OUT ^= LED;
        for(int i = 0; i<0x8FFF; i++);        /* Long LED on */
        P1OUT ^= LED;
        for(int i = 0; i<0x8FFF; i++);        /* Long LED off */
        P1OUT ^= LED;
        for(int i = 0; i<0x8FFF; i++);        /* Long LED on */
        P1OUT ^= LED;
        for(int i = 0; i<0x8FFF; i++);        /* Long LED off */  
}
