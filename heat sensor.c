#include <msp430.h>
void Display_Number(long long n);
void LCDInit ();
void LCD_all_off(void);
int INC_FR0M_COOLTERM=0;
int START_HEAT = 0;
int TEMPERATURE=0;
int TEMP_IN=0;
void main()
{
  WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
  PM5CTL0 &= ~LOCKLPM5;		// Clear locked IO Pins 4		
  P3SEL0 |= BIT5 |BIT4 ;                 	// USCI_A0 UART operation
  P3SEL1 &= ~(BIT4 | BIT5);
  P3SEL1 |=BIT6;
  P3DIR |= BIT4+BIT6;
  
  //P3DIR |=BIT6;
  //P3SEL0 =0x00;
  //P3SEL1 |= BIT6;
  
  P8SEL1 |= BIT4;                         // Configure P8.4 (A7)  for ADC
  P8SEL0 |= BIT4;

    // Disable the GPIO power-on default high-impedance mode to activate previously configured port settings
  PM5CTL0 &= ~LOCKLPM5;


  
  UCA1CTLW0 = UCSWRST;                 	// Put eUSCI in reset
  UCA1CTLW0 |= UCSSEL__SMCLK;    	// CLK = SMCLK
  UCA1BRW = 6;                              		// 115,200 baud
  UCA1MCTLW = 0x22D1;		// UCBRSx value = 0x08 (See UG)
  UCA1CTL1 &= ~UCSWRST;                	// Initialize eUSCI
  
  UCA1IE |= UCRXIE;                     		// Enable USCI_A0 RX interrupt
  while(!(UCA1IFG&UCTXIFG)); 		// wait for transmitter ready
  UCA1TXBUF = 0x00;			// Transmit!

  TA0CTL = TASSEL__ACLK + MC__UP+ TACLR+ ID__1+ TAIE;	
  TA0CCR0 = 32768;

  // Configure ADC12
  ADC12CTL0 = ADC12SHT0_2 | ADC12ON;      // Sampling time, S&H=16, ADC12 on
  ADC12CTL1 = ADC12SHP;                   // Use sampling timer
  ADC12CTL2 |= ADC12RES_2;                // 12-bit conversion results
  ADC12MCTL0 |= ADC12INCH_7;              // A7 ADC input select; Vref=AVCC
  ADC12IER0 |= ADC12IE0 + ADC12VRSEL_4;                  // Enable ADC conv complete interrupt

  REFCTL0|=REFVSEL_0+REFON;
  
  LCDInit ();

  __bis_SR_register(LPM3_bits | GIE);       	// Enter LPM3, interrupts enabled
  __no_operation();                         		// For debugger

}
void LCD_all_off(void)
{
	int i;
	char *ptr = 0;
	ptr += 0x0A20;		// LCD memory starts at 0x0A20
	for (i = 0;i<21;i++)
		*ptr++ = 0x00;
}
void LCDInit ()
{
    PJSEL0 = BIT4 | BIT5;                   // For LFXT

    // Initialize LCD segments 0 - 21; 26 - 43
    LCDCPCTL0 = 0xFFFF;
    LCDCPCTL1 = 0xFC3F;
    LCDCPCTL2 = 0x0FFF;

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Configure LFXT 32kHz crystal
    CSCTL0_H = CSKEY >> 8;                  // Unlock CS registers
    CSCTL4 &= ~LFXTOFF;                     // Enable LFXT
    do
    {
      CSCTL5 &= ~LFXTOFFG;                  // Clear LFXT fault flag
      SFRIFG1 &= ~OFIFG;
    }while (SFRIFG1 & OFIFG);               // Test oscillator fault flag
    CSCTL0_H = 0;                           // Lock CS registers

    // Initialize LCD_C
    // ACLK, Divider = 1, Pre-divider = 16; 4-pin MUX
    LCDCCTL0 = LCDDIV__1 | LCDPRE__16 | LCD4MUX | LCDLP;

    // VLCD generated internally,
    // V2-V4 generated internally, v5 to ground
    // Set VLCD voltage to 2.60v
    // Enable charge pump and select internal reference for it
    LCDCVCTL = VLCD_1 | VLCDREF_0 | LCDCPEN;

    LCDCCPCTL = LCDCPCLKSYNC;               // Clock synchronization enabled

    LCDCMEMCTL = LCDCLRM;                   // Clear LCD memory
      LCDCCTL0 |= LCDON;
 }
void Display_Number(long long n)
{
  int i=0;
  const unsigned char lcd_num[10] = { 0xFC, 0x60, 0xDB, 0xF3, 0x67, 0xB7, 0xBF, 0xE4,  0xFF, 0xF7};
    char *Ptr2Num[6] = {0};
    Ptr2Num[0] +=0xA29;
    Ptr2Num[1] +=0xA25;
    Ptr2Num[2] +=0xA23;
    Ptr2Num[3] +=0xA32;
    Ptr2Num[4] +=0xA2E;
    Ptr2Num[5] +=0xA27;

  LCD_all_off();
     do{
        *Ptr2Num[5-i] = lcd_num[n%10];
         i++;
         n = n/10;  // wastefull!!
    }while ( n );
}
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR()
{
  char TAV;
  int x;
  double duty_cycle=0;
  if  (UCA1IFG & UCRXIFG)
  {
       	UCA1IFG &= ~ UCRXIFG;  
	while(!(UCA1IFG&UCTXIFG));	// wait for transmitter ready
        TAV=UCA1RXBUF;
        UCA1TXBUF = TAV;
        Display_Number(TAV);
        INC_FR0M_COOLTERM=0;
        x=(int)TAV;
        x=x-48;
        switch(START_HEAT)
        {
        case 0: TEMPERATURE=x*10; START_HEAT++; break;
        case 1: TEMPERATURE=TEMPERATURE+x; START_HEAT++;  break;      
        }
        if(START_HEAT==2)
        {
          START_HEAT=0;
          duty_cycle=(100*3.3)/TEMPERATURE;
          (int)duty_cycle;
          TB0CTL = TASSEL__ACLK + MC__CONTINUOUS+ TACLR+ ID__1+ TAIE;
          TB0CCR0=65536;
          TB0CCTL2 = OUTMOD_2;
          TB0CCR2=65536/duty_cycle;

        }
        
  }	
}
#pragma vector=TIMER0_A1_VECTOR
__interrupt void timer()
{
  int TAV=UCA1RXBUF;
  if((TA0IV == 0x0E)&&(INC_FR0M_COOLTERM=0))
  {
  UCA1TXBUF = TAV;
  INC_FR0M_COOLTERM=1;
  }
}
#pragma vector=TIMER0_B1_VECTOR
__interrupt void sss()
{
  if (TBIV)
  {
    while(1)
    {
      __delay_cycles(5000);
      ADC12CTL0 |= ADC12ENC | ADC12SC;    // Start sampling/conversion
      __bis_SR_register(LPM0_bits | GIE); 
    }
      
    
    
  }
  

}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(){

    switch(__even_in_range(ADC12IV, ADC12IV_ADC12RDYIFG))
    {
        case ADC12IV_NONE:        break;    // Vector  0:  No interrupt

        case ADC12IV_ADC12IFG0:             // Vector 12:  ADC12MEM0 Interrupt
          TEMP_IN=ADC12MEM0;


                // Exit from LPM0 and continue executing main
           //     __bic_SR_register_on_exit(LPM0_bits);
            break;
    
       case ADC12IV_ADC12IFG1:   break;        // Vector 14:  ADC12MEM1
        case ADC12IV_ADC12IFG2:   break;    // Vector 16:  ADC12MEM2
        case ADC12IV_ADC12IFG3:   break;    // Vector 18:  ADC12MEM3
        case ADC12IV_ADC12IFG4:   break;    // Vector 20:  ADC12MEM4
        // continue here with IFG5 to IFG31

    default: break;
    }
}

