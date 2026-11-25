#include <header/bsp.h>

//---------------------------------------------------------------------
//  Clock: stop the watchdog, DCO = 1 MHz, flash timing generator
//---------------------------------------------------------------------
void ClockConfig(void) {
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

    if (CALBC1_1MHZ == 0xFF) {          // Calibration constants erased
        while (1);                      // Trap CPU: do not run with an uncalibrated DCO
    }
    DCOCTL  = 0;                        // Lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;              // DCO = 1 MHz
    DCOCTL  = CALDCO_1MHZ;

    FCTL2 = FWKEY | FSSEL0 | FN1;       // Flash clock = MCLK / 3 = ~333 kHz (valid: 257-476 kHz)
}

//---------------------------------------------------------------------
//  GPIO: push buttons, LCD, servo, LDRs, ultrasonic sensor
//---------------------------------------------------------------------
void GPIOconfig(void) {

    // Push buttons: GPIO input, interrupt on falling edge
    PBsArrPortSel    &= ~PBsMask;
    PBsArrPortDir    &= ~PBsMask;
    PBsArrIntEdgeSel |=  PBsMask;
    PBsArrIntPend    &= ~PBsMask;
    PBsArrIntEn      |=  PBsMask;

    // LCD: GPIO outputs, driven low
    LCD_DATA_WRITE   &= ~LCD_DATA_MASK;
    LCD_DATA_DIR     |=  LCD_DATA_MASK;
    LCD_DATA_SEL     &= ~LCD_DATA_MASK;

    LCD_CTL_WRITE    &= ~LCD_CTL_MASK;
    LCD_CTL_DIR      |=  LCD_CTL_MASK;
    LCD_CTL_SEL      &= ~LCD_CTL_MASK;

    LCD_CTL_WRITE_RS &= ~LCD_CTL_RS;
    LCD_CTL_DIR_RS   |=  LCD_CTL_RS;
    LCD_CTL_SEL_RS   &= ~LCD_CTL_RS;

    // Servo: timer PWM output (TA0.1)
    SERVO_DIR        |=  SERVO_MASK;
    SERVO_SEL        |=  SERVO_MASK;

    // LDRs: analog inputs, no pull resistors
    LDR_ARR_REN      &= ~LDR_MASK;
    LDR_ARR          &= ~LDR_MASK;
    LDR_ARR_SEL      &= ~LDR_MASK;
    LDR_ARR_DIR      &= ~LDR_MASK;

    // Ultrasonic TRIG: GPIO output, idle low
    SONAR_ARR        &= ~SONAR_TRIG_PIN;
    SONAR_ARR_SEL    &= ~SONAR_TRIG_PIN;
    SONAR_ARR_DIR    |=  SONAR_TRIG_PIN;

    // Ultrasonic ECHO: timer capture input (TA1.2)
    SONAR_ARR_SEL    |=  SONAR_ECHO_PIN;
    SONAR_ARR_DIR    &= ~SONAR_ECHO_PIN;
}

//---------------------------------------------------------------------
//  UART (USCI_A0): 9600 baud, 8N1, SMCLK = 1 MHz
//---------------------------------------------------------------------
void UARTconfig(void) {
    P1SEL  |= RXD | TXD;                // P1.1 = RXD, P1.2 = TXD
    P1SEL2 |= RXD | TXD;

    UCA0CTL1 |= UCSWRST;                // Hold USCI in reset while configuring
    UCA0CTL1 |= UCSSEL_2;               // Clock source: SMCLK
    UCA0BR0   = 104;                    // 1 MHz / 9600 = 104.17
    UCA0BR1   = 0;
    UCA0MCTL  = UCBRS0;                 // Modulation UCBRSx = 1
    UCA0CTL1 &= ~UCSWRST;               // Release USCI from reset

    IE2 |= UCA0RXIE;                    // Enable RX interrupt
}
