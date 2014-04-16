#include "main.h"




unsigned char read = 0;
unsigned char command = 0;
signed char SensorData1[1024];
unsigned char SensorData2[1024];
unsigned int BaseTime = 273; /* Blinking frequency / timer on startup 0x1000 is 1 second */
unsigned int StartTime = 0;
unsigned int EndTime = 0;
unsigned int Time = 0;
unsigned char ReadingSensor = 0x00;
unsigned int ctr = 0;
unsigned char ActionMode = 0;        /* ActionModes: 0 = LPM3, 1 = Switch between Blink / No Blink modes, 2 = Switch Blink Frequencies, 3 = SPI reading*/



void main(void)
{
  
  /* Turn off Watchdog Timer */
  WDTCTL = WDTPW+WDTHOLD;                   

   /* Initialize DCO */
  BCSCTL1 = CALBC1_1MHZ;                /* Set DCO to 1MHz */
  DCOCTL =  CALDCO_1MHZ;                /* Set DCO to 1MHz */
  __delay_cycles(200000);	// Delay 0.2 s to let clocks settle
  
  /* Enable and Turn LED ON */
  
  P1DIR |=LED;                          /* Set LED pin as Output */        
  P1OUT |=LED;                          /* Set LED pin as HIGH */
  
  /* Enable SPI */
  
  P3SEL |= (BIT1 + BIT2 + BIT3);        /* Peripheral function instead of I/O */
  UCB0CTL0 = UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC; /* SPI Polarity = 1, MSB First, Master, 3 Pin Mode, Synchronous Comm. UCCKPH = 0 is ~SPIPhase*/
  UCB0CTL1 = UCSSEL_2 | UCSWRST;         /* Clock from SMCLK; hold SPI in reset */
  
  UCB0BR1 = 0;                          /* Upper byte of divider word */
  UCB0BR0 = 0x1;                         /* Clock = SMCLK / 10 = 100 KHz */
  
  UCB0CTL1 &= ~UCSWRST;                 /* Remove SPI reset to enable it*/
  
  /* Configure Pins for SPI. These commands might not be needed */
  P3DIR |= nSS | SPI_SOMI | SPI_CLK;    /* Output on P3DIR |= BIT0 | BIT2 | BIT3;  */      
  P3OUT |= nSS | SPI_SOMI | SPI_CLK;    /* Set pins to high */
  P3OUT |= nSS; 
  __delay_cycles(0x80);
  
  /* Disable I2C on the sensor */
  _Sensor_write(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
  
  /* Check Device ID */
  read = _Sensor_read(MPUREG_WHOAMI);
  read = read;
  
  /* Trap uC if Sensor doesn't ID correctly */
  if(read != 0x68)                      /* Check WHOAMI Hopefully it is 0x68. Otherwise freak out. */
  {
    JustDance();
  }
  
  /* Sensor might be sleeping, read Register with Sleep Bit */
  read = _Sensor_read(MPUREG_PWR_MGMT_1);
  
  /* Wake sensor up*/
  command = read & ~BIT_SLEEP;          /* Reset the Sleep Bit */
  _Sensor_write(MPUREG_PWR_MGMT_1, command);/* Write this command to the sensor to wake it up */
  
  /* Erase all data in the three variables */
  
  for(ctr = 0; ctr < 1024; ctr++)
  {
    SensorData1[ctr] = 0;
    SensorData2[ctr] = 0;
    
  }
  
  /* Reset Counter to 0 */
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
  TACTL = TASSEL_1 | MC_2 | ID_3;
  
    /* Other Housekeepigng */
  __delay_cycles(8234);                 /* Delay 0.2s at 1Mhz to let clock settle */
  __enable_interrupt();
  
  /* Set timer to run */
  TACCR0 = BaseTime;
  TACCTL0 |= CCIE;

  
  
  while(1)
  {
    __low_power_mode_3();
    
    if(ActionMode == 1)                 /* Long Buttton Press */
    {
      
    ActionMode = 0;  
    }
    
    
    if(ActionMode == 2)                 /* Short Button Press */
    {
      
      
      ActionMode = 0;
    }    
    
    if(ActionMode == 3)                  /* Timer A Freq at 15hz */
    {
      ReadingSensor = 1;

      StartTime = TAR;
      /* Read X Acc */
      SensorData1[ctr++] = _Sensor_read(MPUREG_ACCEL_XOUT_H);
      /* Read Y Acc */
      SensorData1[ctr++] = _Sensor_read(MPUREG_ACCEL_YOUT_H);
      /* Read Z Acc */
      SensorData1[ctr++] = _Sensor_read(MPUREG_ACCEL_ZOUT_H);
      /* Read X Gyro */
      SensorData1[ctr++] = _Sensor_read(MPUREG_GYRO_XOUT_H);
      /* Read Y Gyro */
      SensorData1[ctr++] = _Sensor_read(MPUREG_GYRO_YOUT_H);
      /* Read Z Gyro */
      SensorData1[ctr++] = _Sensor_read(MPUREG_GYRO_ZOUT_H);
      
      EndTime = TAR;

      if(ctr >= 1020)   /* The Buffer is full */
      {
        Time = EndTime - StartTime;
        JustDance();
      }
      ReadingSensor = 0;
      ActionMode = 0;
    }
    
    
  }
  /*
  for(unsigned int ctr = 0; ctr <1024; ctr++)
  {
    Xdata[ctr] = _Sensor_read(MPUREG_ACCEL_XOUT_H);
    Ydata[ctr] = _Sensor_read(MPUREG_ACCEL_YOUT_H);
    Zdata[ctr] = _Sensor_read(MPUREG_ACCEL_ZOUT_H);
    __delay_cycles(120);
  }
  read = read; */
  
}


/* Interrupt for TimerA Channel 0, runs short periodical*/
#pragma vector = TIMERA0_VECTOR
__interrupt void TA0V_ISR(void)
{
  __disable_interrupt();
  TACCR0 += (BaseTime); /* Next Interrupt at given time */
  ActionMode = 3;
  if(ReadingSensor == 1)
  {
   JustDance(); 
  }
  __low_power_mode_off_on_exit();

  __enable_interrupt();
}

unsigned char _Sensor_write(unsigned char add, unsigned char val)
{
  P3OUT |= mSS;                         /* Deselect Select Memory as SPI slave */  
  P3OUT &= ~nSS;                        /* Select Sensor as SPI Slave */
  unsigned char RXCHAR = 0x00;
  
  RXCHAR = SPI_TXRX(add, val);
  
  P3OUT |= nSS;                         /* Deselect Sensor as SPI slave */        
  return RXCHAR;
}

unsigned char _Sensor_read(unsigned char add)
{
  unsigned char RXCHAR = 0x00;
  P3OUT |= mSS;                         /* Deselect Select Memory as SPI slave */  
  P3OUT &= ~nSS;                        /* Select Sensor as SPI Slave */
  
  RXCHAR = SPI_TXRX(add | MPU_READ, 0x00);
  
  P3OUT |= nSS;                         /* Deselect Sensor as SPI slave */   
  return RXCHAR;
}

unsigned char _Memory_write(unsigned char add, unsigned char val)
{
  P3OUT |= nSS;                         /* Deselect Sensor as SPI slave */ 
  P3OUT &= ~mSS;                        /* Select Memory as SPI Slave */
  unsigned char RXCHAR = 0x00;
  
  RXCHAR = SPI_TXRX(add, val);
  
  P3OUT |= mSS;                         /* Deselect Select Memory as SPI slave */        
  return RXCHAR;
}

unsigned char _Memory_read(unsigned char add)
{
  P3OUT |= nSS;                         /* Deselect Sensor as SPI slave */ 
  unsigned char RXCHAR = 0x00;
  P3OUT &= ~mSS;                        /* Select Select Memory as SPI Slave */
  
  RXCHAR = SPI_TXRX(add | MPU_READ, 0x00);
  
  P3OUT |= mSS;                         /* Deselect Select Memory as SPI slave */   
  return RXCHAR;
}

unsigned char SPI_TXRX(unsigned char add, unsigned char val)
{
  unsigned char RXCHAR = 0x00;
          
  while (!(IFG2 & UCB0TXIFG));          /* Wait for TXBUF to be empty */
  
  UCB0TXBUF = add;                      /* Send Address of Register  */
  while(!(IFG2 & UCB0TXIFG));           /* Wait for TXBUF to be empty (TXBUF data moves to the shift register) */
  while(!(IFG2 & UCB0RXIFG));           /* Wait for RXBUF to be full */     
  RXCHAR = UCB0RXBUF;                   /* Read what is RX to clear buffer / flags*/
  
  
  UCB0TXBUF = val;                      /* Write the val */      
  while(!(IFG2 & UCB0TXIFG));           /* Wait for TXBUF to be empty (TXBUF data moves to the shift register) */
  while(!(IFG2 & UCB0RXIFG));           /* Wait for RXBUF to be full */  
  RXCHAR = UCB0RXBUF;                     /* Read what is RX to clear buffer / flags*/
        
  __delay_cycles(0x10);
  
  return RXCHAR;
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