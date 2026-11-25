/*
 * halGPIO.c - Hardware Abstraction Layer
 *
 * Drivers and interrupt service routines for the MSP430G2553.
 *
 * Chapters
 *   1. Globals
 *   2. System: configuration, low-power modes, interrupts
 *   3. Utilities: busy-wait delays
 *   4. LCD (HD44780, 4-bit)
 *   5. ADC10 (LDRs)
 *   6. Servo (Timer A0 PWM)
 *   7. Sonar (Timer A1 capture) and object scan
 *   8. UART TX and command decoding
 *   9. Flash file system (data in SEG4..SEG1, file table in SEG5)
 *  10. Info flash (segments B/C/D) and LDR calibration table
 *  11. Light scans
 *  12. File records in SEG7/SEG8 (legacy)
 *  13. Interrupt service routines
 */

#include <header/halGPIO.h>
#include <header/bsp.h>
#include <header/api.h>
#include <string.h>
#include <stdint.h>

// Forward declarations (defined later in this file)
int save_inst_line(const char name[FILES_WORD_SIZE], int len,
                   enum file_type_enum type, uint16_t ptr, uint16_t end_ptr);
static inline uint8_t delete_segment7n8(uint16_t seg_start);
void enable_interrupts_timer1_a1(void);
void ldr_to_lcd(int l1, int l2);
void on_ldr_line_received(uint8_t *buf);
void file_read_ldr_table(uint8_t *out);

//=====================================================================
//  1. Globals
//=====================================================================

// System
enum SYSmode lpm_mode;

volatile unsigned int URAT_received_data = 0;

// UART RX
volatile uint8_t pc_ready = 0;       // 1 after the PC sent PC_READY_TOKEN (0xAB)
char receive_str[RX_BUF_CAPACITY + 1] = { '\0' };    // payload + NUL
unsigned int receive_char_index = 0;
enum UARTReceiveType receiveType;

// Framed RX state
enum { RXF_HDR=0, RXF_LEN, RXF_DATA, RXF_CHK };
volatile uint8_t  RXF_phase      = RXF_HDR;
volatile uint8_t  RXF_len_need   = 0;    // LENGTH byte (DATA bytes to expect)
volatile uint8_t  RXF_sum        = 0;    // incremental sum8 over [LENGTH][DATA...]
volatile uint8_t  RXF_status     = 0;    // 0=OK, 1=bad checksum, 2=length overflow

// UART TX
char* transmit_str;
uint8_t transmit_char_index = 0;
uint8_t transmit_busy = 0;

// Framed TX state
volatile uint8_t  TXF_mode     = 0;    // 1 = framed mode active
enum { TXF_HDR=0, TXF_LEN, TXF_DATA, TXF_CHK, TXF_DONE };
volatile uint8_t  TXF_phase     = TXF_HDR;

volatile uint16_t TX_total_len  = 0;    // number of DATA bytes in the frame
volatile uint16_t TX_remaining  = 0;    // countdown of DATA bytes left
volatile uint8_t  TX_checksum   = 0;    // Sum-8 over [LENGTH][DATA...]
volatile uint8_t tx_frame_buf[16];
volatile ack_state_t tx_ack_state = ACK_IDLE;
volatile uint8_t tx_current_HDR = 0;

volatile uint8_t tx_token_byte    = 0;
volatile tx_token_sched_t tx_token_sched = TX_TOKEN_IDLE;

// Servo
volatile unsigned int  servo_current_angle = SERVO_INIT_ANGLE;
volatile unsigned int  servo_prev_angle = SERVO_INIT_ANGLE;

volatile unsigned int servo_timer_sec =0;
int servo_timer_min = 0;

// Servo auto power-off
volatile unsigned int  servo_cycles_left = 0;   // 40Hz frames left to keep PWM on
volatile unsigned char servo_is_attached = 0;   // 1 = connected to TA0.1

// LDR (ADC10)
volatile unsigned int ldr_arr[4];   // DTC buffer: [A3, A2, A1, A0]
volatile unsigned int ldr1 = 0;    // latest A0 (P1.0)
volatile unsigned int ldr2 = 0;    // latest A3 (P1.3)
volatile unsigned char LDR_ready = 0; // set to 1 when a new pair is available
volatile uint8_t ldr_manual_smaple = 0; // set by PB0: take a calibration sample
volatile uint8_t ldr_manual_stop = 0;   // set by PB1: stop the calibration
volatile uint8_t file_ldr_read_ok = 0;

// Sonar
volatile unsigned int  sonar_input_counter = 0;
volatile unsigned int  timer1_a2_capture_start = -2;
volatile unsigned int timer1_a2_capture_end = 0;
volatile unsigned int sonar_echo_capture_width_val = -1;
volatile unsigned char sonar_done = 0;      // 1 when the echo capture is complete
volatile unsigned int sonar_time_out = 0;   // 1 if Timer A1 wrapped before the echo
volatile unsigned int sonar_speed_40HZ = 0; // debug: Timer A0 period counter
volatile unsigned int sonar_speed_17HZ = 0;
volatile unsigned char sonar_active = 1;

// Servo settle wait (see wait_for_servo)
volatile char servo_en_wait = 0;
volatile unsigned int scan_40HZ_tick_wait = 0;
volatile unsigned int scan_40HZ_tick_limit = 0;
volatile unsigned int servo_timer_ticks = 0;

// File system
volatile int file_num = 0;               // number of files in files[]
volatile int  file_len  = 0;             // bytes written to the open file
volatile file_desc_t files[MAX_FILES];
volatile uint16_t file_ptr  = SEG7_START;   // current flash write pointer
volatile uint16_t file_ldr_ptr  = SEG_D_START       ;
volatile uint8_t file_open = 0;                  // 0=closed, 1=open
char     file_name[17] = {0};
char     file_type = file_script;

volatile int16_t file_read_ptr = -1;
char     file_reciving = 0;
volatile char file_got_shtroodle = 0;
// File data segments, ordered low->high (SEG4..SEG1)
static const uint16_t FS_SEG_START[NUM_SEGS] = { SEG4_START, SEG3_START, SEG2_START, SEG1_START};

// File table write pointer (SEG5)
volatile uint16_t file_stract_ptr = SEG5_START;


//=====================================================================
//  2. System: configuration, low-power modes, interrupts
//=====================================================================

void sysConfig(void){
    ClockConfig();
    GPIOconfig();
    UARTconfig();
}

// Enter the requested low-power mode
void enterLPM(unsigned char LPM_level) {
    switch(LPM_level){
        case mode0: _BIS_SR(LPM0_bits); break;
        case mode1: _BIS_SR(LPM1_bits); break;
        case mode2: _BIS_SR(LPM2_bits); break;
        case mode3: _BIS_SR(LPM3_bits); break;
        case mode4: _BIS_SR(LPM4_bits); break;
    }
}

// Global interrupt enable / disable
void enable_interrupts() { _BIS_SR(GIE); }
void disable_interrupts() { _BIC_SR(GIE); }

// Push-button interrupt enable / disable
void enable_PB_interrupts() { PBsArrIntEn |= PBsMask; }
void disable_PB_interrupts() { PBsArrIntEn &= ~PBsMask; }

//=====================================================================
//  3. Utilities: busy-wait delays
//=====================================================================

// At least cnt us (each loop iteration takes a few cycles at MCLK = 1 MHz)
void DelayUs(unsigned int cnt) {
    unsigned int i;
    for(i=cnt ; i>0 ; i--)
        asm("Nop");
}
// cnt ms (MCLK = 1 MHz: 1000 cycles = 1 ms)
void DelayMs(unsigned int cnt) {
    unsigned int i;
    for(i=cnt ; i>0 ; i--)
        __delay_cycles(1000);
}

//=====================================================================
//  4. LCD (HD44780, 4-bit)
//=====================================================================

// Send a command byte to the LCD
void lcd_cmd(unsigned char c) {

    LCD_WAIT;                           // busy-flag check or fixed delay (see halGPIO.h)

    if (LCD_MODE == FOURBIT_MODE)
    {
        LCD_DATA_WRITE &= ~OUTPUT_DATA;     // clear the data bits
        LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }
}
// Send a data (character) byte to the LCD
void lcd_data(unsigned char c) {

    LCD_WAIT;                           // busy-flag check or fixed delay (see halGPIO.h)

    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_RS(1);                          // RS = 1: data
    if (LCD_MODE == FOURBIT_MODE)
    {
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }

    LCD_RS(0);
}
// Write a NUL-terminated string to the LCD
void lcd_puts(const char * s) {
    while(*s)
        lcd_data(*s++);
}
// Initialize the LCD (HD44780 power-on sequence, 4-bit mode)
void lcd_init() {

    char init_value;

    if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
    else init_value = 0x3F;

    LCD_RS_DIR(OUTPUT_PIN);
    LCD_EN_DIR(OUTPUT_PIN);
    LCD_RW_DIR(OUTPUT_PIN);
    LCD_DATA_DIR |= OUTPUT_DATA;
    LCD_RS(0);
    LCD_EN(0);
    LCD_RW(0);

    DelayMs(15);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;

    lcd_strobe();
    DelayMs(5);

    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();


    DelayUs(200);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();


    if (LCD_MODE == FOURBIT_MODE){
        LCD_WAIT;                           // busy-flag check or fixed delay (see halGPIO.h)
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
        lcd_strobe();
        lcd_cmd(0x28);                  // Function set: 4-bit, 2 lines, 5x8
    }
    else
        lcd_cmd(0x3C);                  // Function set: 8-bit, 2 lines, 5x10

    lcd_cmd(0xF);                       // Display on, cursor on, blink on
    lcd_cmd(0x1);                       // Clear display
    lcd_cmd(0x6);                       // Entry mode: increment, no shift
    lcd_cmd(0x80);                      // DDRAM address = 0
}
// Pulse EN to latch the data lines
void lcd_strobe() {
    LCD_EN(1);
    asm("nop");
    LCD_EN(0);                          // falling edge latches the data
}

//=====================================================================
//  5. ADC10 (LDRs)
//=====================================================================
// ADC10: software-triggered sequence A3 -> A0, results stored by the DTC
static void adc10_setup_seq_A3_to_A0(void)
{
    // Enable analog on P1.0 (A0) and P1.3 (A3)
    LDR_ARR_EN |= LDR_MASK;
    // SHS_0: software trigger | CONSEQ_1: one sequence | INCH_3: start at A3
    ADC10CTL1 = SHS_0
              | CONSEQ_1
              | INCH_3
              | ADC10DIV_7
              | ADC10SSEL_3;   // ADC clock = SMCLK / 8

    ADC10CTL0 = SREF_0
              | ADC10SHT_3     // 64 x ADC10CLK (better for high-Z sources)
              | MSC            // auto-sample entire sequence
              | ADC10ON
              | ADC10IE;

    ADC10DTC1 = 4;                          // 4 conversions per sequence
    ADC10SA   = (unsigned int)ldr_arr;     // DTC destination
    ADC10CTL0 |= ENC;                       // enable conversions
}
// Configure the ADC and enable the Timer A0 CCR0 interrupt that starts each sequence
void LDR_init(void)
{
    adc10_setup_seq_A3_to_A0();
    TA0CCTL0 |= CCIE;
}
// Start a new ADC sequence (called from the Timer A0 CCR0 ISR, 40 Hz)
static inline void LDR_sample(void)
{
    if (ADC10CTL1 & ADC10BUSY) return;
    // DTC increments the start address after each block; reload it every time.
    ADC10SA = (unsigned int)ldr_arr;

    // Start a new sequence (ADC10SC must toggle 0->1; ENC is already set)
    ADC10CTL0 |= ADC10SC;
}

//=====================================================================
//  6. Servo (Timer A0 PWM)
//=====================================================================

// Timer A0: servo PWM on TA0.1 (CCR0 = period), 10 ms script tick on CCR2
void setup_timer0_for_servo() {
    TA0CTL = MC_0 | TACLR;

    // CCR2: 10 ms script tick
    TA0CCR2 = TICK10_DELTA;
    TA0CCTL2 = CCIE;                               // enable CCR2 interrupt


    TA0CCR0 = PWM_PERIOD - 1 ;                  // PWM period
    TA0CCTL1 = OUTMOD_7;                        // reset/set: high from CCR0 to CCR1
    TA0CCR1  = (SERVO_MIN + SERVO_MAX) / 2 ;
    TA0CTL = TASSEL_2 + MC_1 +  ID_0;           // SMCLK, up mode, /1

    servo_set_angle(SERVO_INIT_ANGLE);          // start at the middle position
}
// Connect the PWM output to the servo pin
static inline void servo_attach(void) {
    // Ensure PWM mode before connecting the pin
    TA0CCTL1 = (TA0CCTL1 & ~OUTMOD_7) | OUTMOD_7;  // reset/set PWM
    SERVO_OUT &= ~SERVO_MASK;                      // start low
    SERVO_DIR |= SERVO_MASK;                       // output
    SERVO_SEL |= SERVO_MASK;                       // connect to TA0.1
    servo_is_attached = 1;
}
// Disconnect the servo: no holding torque, no jitter while idle
static inline void servo_detach(void) {
    // Stop PWM cleanly at frame boundary and hold the line LOW
    TA0CCTL1 = (TA0CCTL1 & ~OUTMOD_7) | OUTMOD_0;  // OUT controlled by OUT bit
    TA0CCTL1 &= ~OUT;                               // force LOW now
    SERVO_SEL &= ~SERVO_MASK;                       // disconnect TA0.1
    SERVO_DIR |= SERVO_MASK;                        // keep as OUTPUT
    SERVO_OUT &= ~SERVO_MASK;                       // hold low
    servo_is_attached = 0;
    servo_cycles_left = 0;
}
// Move the servo to angle [0..180]. The PWM stays on for SERVO_OFF_TICKS periods.
void servo_set_angle(unsigned int angle) {
    if (servo_current_angle == angle)
    {
        __disable_interrupt();
        servo_prev_angle = angle;
        servo_current_angle = angle;
        __enable_interrupt();
        return; // no change
    }

    servo_attach();
    if (angle > 180u) angle = 180u;

    const unsigned int SERVO_RANGE = (SERVO_MAX - SERVO_MIN);

    unsigned long range_angle = (unsigned long)SERVO_RANGE * (unsigned long)angle;
    unsigned int pulse = (unsigned int)(SERVO_MIN +
        (unsigned int)((range_angle + 90u) / 180u)); // +90/180 rounds to the nearest tick


    __disable_interrupt();
    TA0CCR1 = pulse;
    servo_cycles_left = SERVO_OFF_TICKS;   // restart the power-off countdown
    servo_prev_angle = servo_current_angle;
    servo_current_angle = angle;
    __enable_interrupt();
}

// Sleep until the servo has reached the new angle:
// 3 + |delta| / 5 PWM periods (25 ms each)
void wait_for_servo()
{
    int d = servo_current_angle - servo_prev_angle;
    if (d < 0) d = -d;

    scan_40HZ_tick_limit = 3 +((d + 4 )/5);
    servo_en_wait = 1;
    scan_40HZ_tick_wait = 0;

    while (servo_en_wait == 1){
        enterLPM(mode0);
    }
}

//=====================================================================
//  7. Sonar (Timer A1 capture) and object scan
//=====================================================================

// Sonar frame (3 bytes): [DEG][PERIOD lo][PERIOD hi]
void create_data_to_send_sonar(uint8_t STR[3], uint8_t DEG, uint16_t PERIOD)
{
    STR[0] = (char)DEG;                     // angle

    // Echo width, little-endian
    STR[1] = (char)(PERIOD & 0xFF);       // LSB
    STR[2] = (char)((PERIOD >> 8) & 0xFF); // MSB
}

// Pack four 10-bit fields (pl1, pl2, l1, l2) into 5 bytes (40 bits), little-endian.
// Layout (LSB->MSB): [pl1(10) | pl2(10) | l1(10) | l2(10)]
static inline void create_data_ldr_table(uint8_t STR[5],
                                         uint16_t pl1, uint16_t pl2,
                                         uint16_t l1,  uint16_t l2)
{
    // Keep only 10 bits from each input
    uint16_t a = (uint16_t)(pl1 & 0x03FF);
    uint16_t b = (uint16_t)(pl2 & 0x03FF);
    uint16_t c = (uint16_t)(l1  & 0x03FF);
    uint16_t d = (uint16_t)(l2  & 0x03FF);

    // Byte 0: bits 7..0   = a[7:0]
    STR[0] = (uint8_t)(a & 0xFF);

    // Byte 1: bits 1..0   = a[9:8]
    //         bits 7..2   = b[5:0]
    STR[1] = (uint8_t)(((a >> 8) & 0x03) | ((b & 0x3F) << 2));

    // Byte 2: bits 3..0   = b[9:6]
    //         bits 7..4   = c[3:0]
    STR[2] = (uint8_t)(((b >> 6) & 0x0F) | ((c & 0x0F) << 4));

    // Byte 3: bits 5..0   = c[9:4]
    //         bits 7..6   = d[1:0]
    STR[3] = (uint8_t)(((c >> 4) & 0x3F) | ((d & 0x03) << 6));

    // Byte 4: bits 7..0   = d[9:2]
    STR[4] = (uint8_t)((d >> 2) & 0xFF);
}

// Light frame (4 bytes): L1[7:0] | L2[7:0] | DEG[3:0],L2[9:8],L1[9:8] | DEG[7:4]
static inline void create_data_to_send_light(uint8_t STR[4],
                                             uint16_t l1, uint16_t l2, uint8_t DEG)
{
    STR[0] = (uint8_t)(l1 & 0xFF);                    // L1[7:0]
    STR[1] = (uint8_t)(l2 & 0xFF);                    // L2[7:0]
    STR[2] = (uint8_t)(((l1 >> 8) & 0x03)             // bits 0-1: L1[9:8]
                     | (((l2 >> 8) & 0x03) << 2)      // bits 2-3: L2[9:8]
                     | ((DEG & 0x0F) << 4));          // bits 4-7: DEG[3:0]
    STR[3] = (uint8_t)((DEG >> 4) & 0x0F);            // bits 3-0: DEG[7:4]
}

// Take trow_samples readings and return the last one
uint16_t sonar_sample(int trow_samples)
{
    uint16_t val = 0;
    disable_interrupts();
    int i=0;
    for (; i < trow_samples; i++)
        val = get_period();
    return val;
}

// 11 us trigger pulse on TRIG
static inline void sonar_trig_11us(void)
{
    unsigned int start = TA1R;   // 1 tick ~= 1us (SMCLK ~1MHz)
    SONAR_ARR |=  SONAR_TRIG_PIN; // HIGH
    while ((unsigned int)(TA1R - start) < TRIG_TICKS_NUM) {}
    SONAR_ARR &= ~SONAR_TRIG_PIN; // LOW
}

// One sonar measurement: trigger, then capture the echo width on TA1.2.
// Returns the echo width in timer ticks (1 tick = 1 us), or 0 on timeout.
uint16_t get_period(void) {
    sonar_done = 0;
    sonar_time_out = 0;
    sonar_input_counter = 0;
    sonar_speed_17HZ = 0;
    sonar_active = 1;

    TA1CTL = MC_0 | TACLR ;                 // stop and clear Timer A1

    // Start Timer A1
    TA1CTL = TASSEL_2 // SMCLK (1 MHz)
    + MC_2            // MC_2: Continuous mode
    + TACLR;          // TACLR: Reset timer to 0

    enable_interrupts_timer1_a1();

    sonar_trig_11us();

    // Ignore the first 250 us (trigger cross-talk)
    unsigned int t0 = TA1R;
    while ((unsigned int)(TA1R - t0) < 250) { }

    // Capture the echo rising edge on TA1.2
    TA1CCTL2 = CM_1  // Capture on rising edge
    | CCIS_0         // Capture input select: CCI2A (P2.4)
    | SCS            // Synchronous capture
    | CAP            // Capture mode
    | CCIE;          // Enable CCR2 interrupt
    TA1CCTL2 &= ~CCIFG;

    // CCR0 = timeout (Timer A1 wrap, ~65 ms)
    TA1CCR0   = 0xFFFF;
    TA1CCTL0  = CCIE;

    // Sleep until echo or timeout. Other ISRs (ADC, UART, Timer0) also wake
    // the CPU, so check the flags with GIE off and go back to sleep.
    __disable_interrupt();
    while (!sonar_done && !sonar_time_out) {
        __bis_SR_register(LPM0_bits + GIE);
        __disable_interrupt();
    }
    __enable_interrupt();

    if (sonar_time_out) {
        sonar_active = 0;
        return 0;
    }

    sonar_active = 0;
    unsigned int start = timer1_a2_capture_start;
    unsigned int end = timer1_a2_capture_end;
    uint16_t period = (uint16_t)(end - start);  // modulo 2^16: also correct if the timer wrapped

    if (period == 0)
        return 0;

    sonar_echo_capture_width_val = period;
    return period;
}

void enable_interrupts_timer1_a1(){
    TA1CCTL2 |= CCIE;
    TA1CCTL1 |= CCIE;
}
void disable_interrupts_timer1_a1(){
    TA1CCTL2 &= ~CCIE;
    TA1CCTL1 &= ~CCIE;
}


// Object scan: one sonar reading at every angle in [deg_start, deg_end),
// one 3-byte frame per angle
void scan_sweep_l2r(int deg_start,int deg_end)
{
    servo_set_angle(deg_start);
    wait_for_servo();
    uint8_t out [3];
    uint16_t period;
    int i = deg_start;
    for (; i < deg_end ; i++)
    {

        period = go_sonar_trow(i,1);

        create_data_to_send_sonar(out,(uint8_t)i,period);
        dynamic_transmit_len(out,3);

    }
}

// Move to angle and take trow_sample sonar readings
uint16_t go_sonar_trow(int angle, int trow_sample) {
    servo_set_angle(angle);
    disable_interrupts();
    uint16_t val = sonar_sample(trow_sample);
    enable_interrupts();
    return val;
}

//=====================================================================
//  8. UART TX and command decoding
//=====================================================================

// 8-bit sum of n bytes (modulo 256)
static inline uint8_t sum8(const uint8_t *b, uint8_t n)
{
    uint16_t s = 0;
    uint8_t i;
    for ( i = 0; i < n; i++) s += b[i];
    return (uint8_t)s;          // same as s & 0xFF
}

static inline uint8_t uart_tx_ready(void) {
    // TX buffer empty flag
    return (IFG2 & UCA0TXIFG) ? 1u : 0u;
}

static inline void uart_send_token_or_defer(uint8_t tok)
{
    // Send the token now if the TX buffer is free; otherwise after the current frame
    // Never inject the token in the middle of a frame we are sending
    if (!transmit_busy && uart_tx_ready()) {
        UCA0TXBUF   = tok;
        tx_token_sched = TX_TOKEN_IDLE;
    } else {
        tx_token_byte  = tok;
        tx_token_sched = TX_TOKEN_DEFERRED;   // sent by the TX ISR at TXF_DONE
    }
}

// Sleep until the PC has sent PC_READY_TOKEN
static inline void wait_for_pc_ready(void) {
    while (!pc_ready) { enable_interrupts(); enterLPM(mode0); }
}
// Send a binary frame [0xFF][LEN][DATA][CHK] and wait for the PC's ACK.
// The frame is sent again until the PC answers ACK_OK.
void dynamic_transmit_len(const uint8_t *data, uint8_t len)
{
    wait_for_pc_ready();
    tx_current_HDR = UART_BIN_HDR;

    for (;;)
    {
        while (transmit_busy) { enable_interrupts(); enterLPM(mode0); }

        if (len > sizeof(tx_frame_buf)) len = sizeof(tx_frame_buf);
        uint8_t i = 0;
        for (; i < len; i++) { tx_frame_buf[i] = data[i]; }

        TX_total_len  = len;
        TX_remaining  = len;
        TX_checksum   = (uint8_t)(len + sum8((const uint8_t*)data, len));

        // Frame TX state
        TXF_phase     = TXF_HDR;
        TXF_mode      = 1;
        transmit_str        = (char*)tx_frame_buf;
        transmit_char_index = 0;

        // Reset ACK state just before kicking TX
        tx_ack_state = ACK_IDLE;

        // Kick TX ISR
        transmit_busy = 1;
        IE2 |= UCA0TXIE;

        // Sleep until the TX ISR has sent the frame and the RX ISR got the ACK token
        while (tx_ack_state != ACK_OK && tx_ack_state != ACK_FAIL) {
            enable_interrupts();
            enterLPM(mode0);
        }

        if (tx_ack_state == ACK_OK) {
            break;  // success
        }
        // ACK_FAIL: send the frame again
    }
}

// Send a text frame [0xAA][LEN][DATA][CHK] and wait for the PC's ACK.
// The frame is sent again until the PC answers ACK_OK.
void dynamic_transmit(const char* str)
{
    wait_for_pc_ready();
    tx_current_HDR = UART_DATA_HDR;

    for (;;)
    {
        while (transmit_busy) { enable_interrupts(); enterLPM(mode0); }

        // Compute LENGTH and CHECKSUM
        uint16_t L = 0; while (str[L] != '\0') L++; if (L > 255) L = 255;
        TX_total_len  = (uint8_t)L;
        TX_remaining  = (uint8_t)L;
        TX_checksum   = (uint8_t)((uint8_t)L + sum8((const uint8_t*)str, (uint8_t)L));

        // Frame TX state
        TXF_phase     = TXF_HDR;
        TXF_mode      = 1;
        transmit_str        = (char*)str;
        transmit_char_index = 0;

        // Reset ACK state just before kicking TX
        tx_ack_state = ACK_IDLE;

        // Kick TX ISR
        transmit_busy = 1;
        IE2 |= UCA0TXIE;

        // Sleep until the TX ISR has sent the frame and the RX ISR got the ACK token
        while (tx_ack_state != ACK_OK && tx_ack_state != ACK_FAIL) {
            enable_interrupts();
            enterLPM(mode0);
        }

        if (tx_ack_state == ACK_OK) {
            break;  // success
        }
        // ACK_FAIL: send the frame again
    }
}

// Called by the RX ISR for every valid text frame.
// CONTROL mode: the first character selects the FSM state.
// DATA mode: the string stays in receive_str and URAT_received_data is set.
void on_string_received(const char* str)
{
    if (receiveType == CONTROL)
    {

        switch (str[0])
        {
            // Script ISA
            case '0':
                state = state0;
                break;
            case '1':
                state = state1;
                break;
            case '2':
                state = state2;
                break;
            case '3':
                state = state3;
                break;
            case '4':
                state = state4;
                break;
            case '5':
                state = state5;
                break;
            case '6':
                state = state6;
                break;
            case '7':
                state = state7;
                break;
            case '8':
                state = state8;
                break;
            // Debug modes
            case 'a':
                state = state_d_servo;
                break;
            case 'b':
                state = state_d_LDR;
                break;
            case 'c':
                state = state_d_sonar;
                break;
            case 'd':
                state = state_d_scan;
                break;
            // Radar modes
            case 'A':
                state = state_prod_obj_scan;
                break;
            case 'B':
                state = state_prod_telemeter;
                break;
            case 'C':
                state = state_prod_ligth_scan;
                break;
            case 'D':
                state = state_prod_full_scan;
                break;
            case 'L':
                state = state_prod_ldr_table;
                break;
            case 'N':
                state = state_prod_show_ldr_table;
                break;
            // File modes
            case 'e':
                state = state_prod_save_file;
                break;
            case 'r':
                state = state_prod_read_file;
                break;
            case 'g':
                state = state_prod_delete_stract;
                break;
            case 'h':
                state = state_prod_load_stract;
                break;
            case 'i':
                state = state_prod_run_script;
                break;

            default:
                break;
            }
    }
    else
    {   // DATA mode
        URAT_received_data = 1;
    }
}


//=====================================================================
//  9. Flash file system (data in SEG4..SEG1, file table in SEG5)
//=====================================================================

// Erase the 512-byte main-flash segment that contains addr
static inline void flash_erase_segment(uint16_t addr)
{
    unsigned int gie = __get_SR_register() & GIE;  // save GIE
    __disable_interrupt();

    uint16_t seg = addr & ~(FLASH_SEG_SIZE - 1);

    FCTL3 = FWKEY;             // unlock
    FCTL1 = FWKEY | ERASE;     // segment erase mode
    *(volatile uint16_t*)seg = 0;   // dummy write starts the erase (all bytes -> 0xFF)
    while (FCTL3 & BUSY);      // wait
    FCTL1 = FWKEY;             // clear ERASE
    FCTL3 = FWKEY | LOCK;      // lock

    if (gie) __bis_SR_register(GIE);               // restore GIE
}

// Erase the whole file data area (SEG4..SEG1)
void fs_init_clean(void)
{
    uint16_t a = FS_BASE;
    for (; a < FS_END; a += FLASH_SEG_SIZE) {
        flash_erase_segment(a);
    }
}

// Erase one 512-byte segment of the file area, or SEG5 (file table)
static inline void delete_segment512(uint16_t seg_start)
{
    if (seg_start != SEG5_START)
    {
        // Bounds check (no erase on failure)
        if ((uint32_t)seg_start + (uint32_t)FLASH_SEG_SIZE > (uint32_t)FS_END) {
            dynamic_transmit("ERROR: erase overflow");
            return;
        }
    }
    // Save/disable interrupts during flash operation
    unsigned int gie = __get_SR_register() & GIE;
    __disable_interrupt();
    // Unlock flash and enter ERASE mode
    FCTL3 = FWKEY;               // unlock (clear LOCK)
    FCTL1 = FWKEY | ERASE;       // set ERASE
    // Dummy write to any address in the target segment triggers full-segment erase
    *(volatile uint8_t*)seg_start = 0;
    // Wait until controller finishes
    while (FCTL3 & BUSY) { /* wait */ }
    // Exit erase mode and re-lock
    FCTL1 = FWKEY;               // clear ERASE
    FCTL3 = FWKEY | LOCK;        // lock
    // Restore interrupt state
    if (gie) __bis_SR_register(GIE);
}


// 1 if addr is the start of a file-data segment
static inline uint8_t is_seg_start_addr(uint16_t addr) {
    uint8_t i = 0;
    for (; i < NUM_SEGS; i++) {
        if (addr == FS_SEG_START[i]) return 1;
    }
    return 0;
}


// Write one word at *dst and advance *dst by 2.
// Erases the segment first if *dst is at a segment start.
static inline void flash_write_word(uint16_t *dst, uint16_t w)
{
    unsigned int gie = __get_SR_register() & GIE;  // save GIE
    __disable_interrupt();

    // Erase only if the write starts exactly at a segment start
    if (is_seg_start_addr(*dst)) {
        delete_segment512(*dst);
    }

    FCTL3 = FWKEY;           // unlock
    FCTL1 = FWKEY | WRT;     // write mode
    *(volatile uint16_t*)(*dst) = w;  // program WORD
    while (FCTL3 & BUSY) { } // wait
    FCTL1 = FWKEY;           // clear WRT
    FCTL3 = FWKEY | LOCK;    // lock

    *dst += 2;               // advance by 2 bytes (word-aligned)

    if (gie) __bis_SR_register(GIE);   // restore GIE
}

// Write n bytes at *dst word by word (an odd tail is padded with 0xFF) and advance *dst.
// Erases every segment whose start falls inside the written range.
static inline void flash_write_bytes(uint16_t *dst, const uint8_t *src, uint16_t n)
{
    unsigned int gie = __get_SR_register() & GIE;   // save GIE
    __disable_interrupt();

    // Range to write, rounded up to whole words
    uint16_t start_dst = *dst;
    uint16_t end_dst   = (uint16_t)(start_dst + ((n + 1u) & ~1u));  // end exclusive

    // Word writes need an even address
    if ((start_dst & 1u) != 0u) {
        dynamic_transmit("ERROR: flash_write_bytes dst not even-aligned");
    }

    // Erase each segment whose start lies in [start_dst, end_dst)
    uint8_t i;
    for (i = 0; i < NUM_SEGS; i++) {
        uint16_t s = FS_SEG_START[i];
        if (start_dst <= s && s < end_dst) {
            delete_segment512(s);
        }
    }

    FCTL3 = FWKEY;               // unlock
    FCTL1 = FWKEY | WRT;         // write mode

    // Pack bytes into words (little-endian); pad an odd tail with 0xFF
    uint16_t k;
    for ( k = 0; k < n; k += 2) {
        uint16_t lo = src[k];
        uint16_t hi = (k + 1u < n) ? src[k + 1u] : 0xFFu;
        *(volatile uint16_t*)( (uintptr_t)(*dst) ) = (uint16_t)(lo | (hi << 8));
        *dst += 2;                      // always advance by 2 (word-aligned)
        while (FCTL3 & BUSY) { }        // wait after each program
    }

    FCTL1 = FWKEY;              // clear WRT
    FCTL3 = FWKEY | LOCK;       // lock

    if (gie) __bis_SR_register(GIE);   // restore GIE
}

// Copy up to 16 characters and NUL-terminate (dst needs 17 bytes)
void str_copy(char *dst, const char *src)
{
    int i = 0;
    while (i < 16 && src[i] != '\0') {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';  // null terminate
}

// --- Files: read / receive ---

// Read the next line of file file_idx into dst (up to 15 chars, ends after '\n').
// Set file_read_ptr = -1 before the first call. Returns NULL at end of file.
char* file_read_text(uint8_t file_idx, char *dst)
{
    if (file_idx >= file_num) return NULL;
    int len = files[file_idx].len;
    if (len <= 0) return NULL;

    if (file_read_ptr < 0) file_read_ptr = 0;
    if (file_read_ptr >= len) { file_read_ptr = -1; return NULL; }

    uint16_t start = files[file_idx].ptr;
    const char *p = (const char *)(uintptr_t)start;

    int i = 0;

    while (file_read_ptr < len && i < 15) {
        char c = p[file_read_ptr++];
        dst[i++] = c;
        if (c == '\n') break;
    }
    dst[i] = '\0';
    return dst;
}

// Write a received chunk to flash, or close the file on EOF.
// Returns FILE_OK, FILE_EOF_DONE or FILE_NO_SPACE.
static inline uint8_t on_file_line_received(uint8_t *buf, int line_len, uint8_t reason)
{
    if (reason == TYPE_FILE_EOF) {
        // Allow closing an empty file too
        if (!file_open) {
            file_open = 1;
            file_len  = 0;
            if (file_num == 0) { file_ptr = FS_BASE; }
            else               { file_ptr = files[file_num - 1].end_ptr; }
            files[file_num].ptr  = file_ptr;
            str_copy(files[file_num].name, file_name);
            files[file_num].type = file_type;
        }

        // Need 4 bytes for footer: [MAGIC][LEN]
        if ((uint32_t)file_ptr + 4u > (uint32_t)FS_END) {
            file_open = 0;
            file_got_shtroodle = 2;      // 2 = no space
            file_len = 0;
            return FILE_NO_SPACE;
        }

        // Write footer & finalize descriptor
        flash_write_word(&file_ptr, FOOTER_MAGIC);
        flash_write_word(&file_ptr, (uint16_t)file_len);

        files[file_num].len     = file_len;
        files[file_num].end_ptr = file_ptr;

        file_open = 0;
        file_len  = 0;
        file_got_shtroodle = 1;

        save_inst_line(files[file_num].name,
                       files[file_num].len,
                       files[file_num].type,
                       files[file_num].ptr,
                       files[file_num].end_ptr);

        file_num++;
        return FILE_EOF_DONE;
    }

    // Data chunk
    if (!file_open) {
        file_open = 1;
        file_len  = 0;
        if (file_num == 0) { file_ptr = FS_BASE; }
        else               { file_ptr = files[file_num - 1].end_ptr; }
        files[file_num].ptr  = file_ptr;
        str_copy(files[file_num].name, file_name);
        files[file_num].type = file_type;
    }

    if (line_len <= 0) {
        return FILE_OK; // nothing to write
    }

    // Room for this chunk plus the 4-byte footer
    if ((uint32_t)file_ptr + (uint32_t)line_len + 4u > (uint32_t)FS_END) {
        file_open = 0;
        file_got_shtroodle = 2;      // 2 = no space
        file_len = 0;
        return FILE_NO_SPACE;
    }

    // Write exactly 'line_len' bytes (no padding)
    flash_write_bytes(&file_ptr, buf, (uint16_t)line_len);
    file_len += (uint16_t)line_len;  // update after successful write

    return FILE_OK;
}

// --- Files: file table (SEG5) ---

// Concatenate two strings into dest (dest must be large enough)
void str_concat(char *dest, const char *str1, const char *str2)
{
    int i = 0;

    // Copy str1 into dest
    while (str1[i] != '\0') {
        dest[i] = str1[i];
        i++;
    }

    // Copy str2 after str1
    int j = 0;
    while (str2[j] != '\0') {
        dest[i] = str2[j];
        i++;
        j++;
    }

    // Null-terminate result
    dest[i] = '\0';
}

// uint16 -> decimal string
static char* int_to_string(uint16_t v, char *buf) {
    if (v == 0u) { buf[0] = '0'; buf[1] = '\0'; return buf; }
    char tmp[6];           // max 65535 -> 5 digits + NUL
    unsigned i = 0, j = 0;
    while (v) { tmp[i++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (i) buf[j++] = tmp[--i];
    buf[j] = '\0';
    return buf;
}

// String length
static inline uint16_t get_str_len(const char *s) {
    uint16_t n = 0;
    while (s[n] != '\0') n++;
    return n;
}

// Round up byte-count to even (word-aligned)
static inline uint16_t even_bytes(uint16_t n) { return (uint16_t)((n + 1u) & ~1u); }

// Append one file record to the file table in SEG5:
//   META_MAGIC, then name, len, type, ptr, end_ptr as NUL-terminated strings.
// Returns 0 on success, -1 if SEG5 is full.
int save_inst_line(const char name[FILES_WORD_SIZE],
                   int len,
                   enum file_type_enum type,
                   uint16_t ptr,
                   uint16_t end_ptr)
{
    char st_name[17];
    char s_len[6], s_type[6], s_ptr[6], s_end[6];

    // Prepare strings
    str_copy(st_name, name);                    // up to 16 chars + NUL
    int_to_string((uint16_t)len,     s_len);    // decimal ASCII + NUL
    int_to_string((uint16_t)type,    s_type);
    int_to_string(ptr,               s_ptr);
    int_to_string(end_ptr,           s_end);

    // Capacity check (marker 2 bytes + 5 fields padded to even)
    uint16_t need = 2u
      + even_bytes((uint16_t)(get_str_len(st_name) + 1u))
      + even_bytes((uint16_t)(get_str_len(s_len)   + 1u))
      + even_bytes((uint16_t)(get_str_len(s_type)  + 1u))
      + even_bytes((uint16_t)(get_str_len(s_ptr)   + 1u))
      + even_bytes((uint16_t)(get_str_len(s_end)   + 1u));

    if ((uint32_t)file_stract_ptr + (uint32_t)need > (uint32_t)SEG5_END) {
        dynamic_transmit("META FULL\r\n");
        return -1;
    }

    // 0) marker
    flash_write_word(&file_stract_ptr, META_MAGIC);

    // 1) name
    flash_write_bytes(&file_stract_ptr, (const uint8_t*)st_name,
                      (uint16_t)(get_str_len(st_name) + 1u));

    // 2) len
    flash_write_bytes(&file_stract_ptr, (const uint8_t*)s_len,
                      (uint16_t)(get_str_len(s_len) + 1u));

    // 3) type
    flash_write_bytes(&file_stract_ptr, (const uint8_t*)s_type,
                      (uint16_t)(get_str_len(s_type) + 1u));

    // 4) ptr
    flash_write_bytes(&file_stract_ptr, (const uint8_t*)s_ptr,
                      (uint16_t)(get_str_len(s_ptr) + 1u));

    // 5) end_ptr
    flash_write_bytes(&file_stract_ptr, (const uint8_t*)s_end,
                      (uint16_t)(get_str_len(s_end) + 1u));

    return 0;
}


// Read one NUL-terminated field of a SEG5 record (padded to an even length by
// flash_write_bytes). Copies at most max-1 chars into dst.
// Returns the address of the next field, or 0 if the field is broken.
static uint16_t stract_read_field(uint16_t p, char *dst, uint8_t max)
{
    uint8_t n = 0;
    for (;;) {
        if ((uint16_t)(p + n) > SEG5_END) return 0;     // ran out of SEG5
        char c = *(const volatile char *)(uintptr_t)(p + n);
        if (c == '\0') break;
        if (n >= (uint8_t)(max - 1u)) return 0;          // too long: not a valid field
        dst[n++] = c;
    }
    dst[n] = '\0';
    return (uint16_t)(p + even_bytes((uint16_t)(n + 1u)));
}

// Decimal string -> uint16. Returns 1 on success, 0 if empty / not a number / > 65535.
static uint8_t dec_to_u16(const char *s, uint16_t *out)
{
    uint32_t v = 0;
    uint8_t  k = 0;

    if (s[0] == '\0') return 0;
    for (; s[k] != '\0'; k++) {
        if (s[k] < '0' || s[k] > '9') return 0;
        v = v * 10u + (uint32_t)(s[k] - '0');
        if (v > 0xFFFFu) return 0;
    }
    *out = (uint16_t)v;
    return 1;
}

// Load the file table from SEG5 into files[] (record format: see save_inst_line)
// and set file_stract_ptr after the last record, so new records are appended.
// The table ends at the first erased word (0xFFFF).
void file_load_all_stract(void)
{
    uint16_t p = SEG5_START;
    char     name[FILES_WORD_SIZE + 1];
    char     num[6];
    uint16_t v[4];                      // len, type, ptr, end_ptr
    uint8_t  k;

    file_num  = 0;
    file_open = 0;

    while (file_num < MAX_FILES && p < SEG5_END) {
        if (*(const volatile uint16_t *)(uintptr_t)p != META_MAGIC) break;

        // Fields: name, then len / type / ptr / end_ptr as decimal strings
        uint16_t q = stract_read_field((uint16_t)(p + 2u), name, sizeof(name));
        for (k = 0; k < 4 && q != 0; k++) {
            q = stract_read_field(q, num, sizeof(num));
            if (q != 0 && !dec_to_u16(num, &v[k])) q = 0;
        }
        if (q == 0) break;                                      // broken record

        // Sanity check: the file must lie inside the data area
        if (v[1] > (uint16_t)file_script)       break;
        if (v[2] < FS_BASE || v[3] > FS_END)    break;
        if (v[2] > v[3] || v[0] > (uint16_t)(v[3] - v[2])) break;

        for (k = 0; k < FILES_WORD_SIZE && name[k] != '\0'; k++)
            files[file_num].name[k] = name[k];
        if (k < FILES_WORD_SIZE) files[file_num].name[k] = '\0';
        files[file_num].len     = v[0];
        files[file_num].type    = (enum file_type_enum)v[1];
        files[file_num].ptr     = v[2];
        files[file_num].end_ptr = v[3];

        file_num++;
        p = q;
    }

    file_stract_ptr = p;                // next record goes here

    // Anything but erased flash here means a broken table: writing over it would fail
    if (p < SEG5_END && *(const volatile uint16_t *)(uintptr_t)p != 0xFFFFu)
        dynamic_transmit("file table is corrupt, send 'g' to reset it\n");
}

// Erase the file table (SEG5) and clear files[] in RAM
void file_delete_stract(void)
{
    unsigned int gie = __get_SR_register() & GIE;  // save GIE
    __disable_interrupt();

    // Erase SEG5
    delete_segment512(SEG5_START);

    if (gie) __bis_SR_register(GIE);               // restore GIE

    // Reset the RAM state
    file_stract_ptr     = SEG5_START;  // next record goes here
    file_num            = 0;           // no files loaded
    file_open           = 0;           // ensure closed
    file_got_shtroodle  = 0;           // clear flag
    int i;
    // Clear RAM descriptors to avoid stale data
    for (i = 0; i < MAX_FILES; i++) {
        files[i].name[0] = '\0';
        files[i].len     = 0;
        files[i].type    = 0;
        files[i].ptr     = 0;
        files[i].end_ptr = 0;
    }
}


//=====================================================================
//  10. Info flash (segments B/C/D) and LDR calibration table
//=====================================================================

// 1 if s is the start of info segment B, C or D
static inline uint8_t is_info_seg_start(uint16_t s)
{
    return (s == SEG_B_START) || (s == SEG_C_START) || (s == SEG_D_START);
}

// Erase a 64B Info segment (B/C/D). Returns 1 on success, 0 on reject.
static inline uint8_t delete_segment64(uint16_t seg_start)
{
    // Accept only B/C/D and require 64B alignment
    if (!is_info_seg_start(seg_start))              return 0;
    if (seg_start & (INFO_SEG_SIZE - 1u))           return 0;

    unsigned int gie = __get_SR_register() & GIE;
    __disable_interrupt();

    FCTL3 = FWKEY;                 // unlock
    FCTL1 = FWKEY | ERASE;         // segment erase
    *(volatile uint8_t*)seg_start = 0;  // dummy write
    while (FCTL3 & BUSY) { }
    FCTL1 = FWKEY;                 // clear ERASE
    FCTL3 = FWKEY | LOCK;          // lock

    if (gie) __bis_SR_register(GIE);
    return 1;
}
// Write one 16-bit word into any 64B Info segment (B/C/D) with bounds + first-use erase.
// 'addr' holds the current write address (e.g., 0x1040 for SEG_C). On success it advances by 2.
static inline void flash_write_word64_any(uint16_t *addr, uint16_t w)
{
    uint16_t a = *addr;

    // Derive segment start from address (align down to 64B)
    uint16_t seg_start = (uint16_t)(a & ~(INFO_SEG_SIZE - 1u));
    if (!is_info_seg_start(seg_start)) return;

    uint16_t seg_end = SEG_END(seg_start);

    // Bounds & alignment: word must fit fully within [seg_start .. seg_end]
    if (a < seg_start)               return;
    if (a & 1u)                      return;               // require even address
    if (a > (uint16_t)(seg_end - 1)) return;               // first byte in range?
    if ((uint16_t)(a + 1u) > seg_end) return;              // second byte in range?

    unsigned int gie = __get_SR_register() & GIE;
    __disable_interrupt();

    // First use on this segment? Erase once.
    if (a == seg_start) {
        delete_segment64(seg_start);
    }

    FCTL3 = FWKEY;               // unlock
    FCTL1 = FWKEY | WRT;         // write
    *(volatile uint16_t*)a = w;  // program word
    while (FCTL3 & BUSY) { }
    FCTL1 = FWKEY;               // clear WRT
    FCTL3 = FWKEY | LOCK;        // lock

    *addr = (uint16_t)(a + 2u);  // advance by one word

    if (gie) __bis_SR_register(GIE);
}


// Erase info segment C
static inline uint8_t segc_erase(void)
{
    return delete_segment64(SEG_C_START);
}

// Reserve an array of uint16 inside SEG_C.
// *p_cur: current pointer (byte address inside SEG_C), e.g. SEG_C_START
// size_words: number of u16 elements to reserve
// returns base address (byte address). On failure returns 0 (no change).
static inline uint16_t def_seg_c_arr(uint16_t *p_cur, uint16_t size_words)
{
    if (!p_cur || size_words == 0) return 0;

    uint16_t base = *p_cur;
    uint32_t need = ((uint32_t)size_words) << 1;               // bytes
    if (base & 1u)                   return 0;                  // must be even
    if (base < SEG_C_START)          return 0;
    if ((uint32_t)base + need - 1u > SEG_C_END) return 0;       // out of SEG_C

    *p_cur = (uint16_t)(base + need);   // advance allocation cursor
    return base;                         // base address of the "array"
}

// Read u16 at array base + index (no size tracking here)
// On out-of-range, returns 0xFFFF.
static inline uint16_t read_seg_c_arr(uint16_t base, uint16_t index)
{
    uint32_t a = (uint32_t)base + ((uint32_t)index << 1);
    if ((a & 1u) || a < SEG_C_START || a > (SEG_C_END - 1u)) return 0xFFFFu;
    return *(const uint16_t*)(uintptr_t)a;   // flash is memory-mapped
}

// Write value at base[index] inside SEG_C.
// Returns 1 on success, 0 on failure.
static inline uint8_t write_seg_c_arr_at(uint16_t base, uint16_t index, uint16_t value)
{
    uint32_t a32 = (uint32_t)base + ((uint32_t)index << 1);
    if ((a32 & 1u) || a32 < SEG_C_START || a32 > (SEG_C_END - 1u)) return 0;
    uint16_t a = (uint16_t)a32;
    flash_write_word64_any(&a, value);
    return 1;
}


// Write one word into info segment D (erases D on the first word)
static inline void flash_write_word64(uint16_t *dst, uint16_t w)
{
    // Bounds check: need room for 2 bytes
    if ((uint32_t)(*dst) > (uint32_t)(SEG_D_END - 1)) return;   // past end
    if ((uint32_t)(*dst) + 1u > (uint32_t)SEG_D_END)  return;   // would overflow

    unsigned int gie = __get_SR_register() & GIE;
    __disable_interrupt();

    // Erase once when starting at the very beginning of Segment D
    if (*dst == SEG_D_START) {
        delete_segment64(SEG_D_START);
    }

    FCTL3 = FWKEY;               // unlock
    FCTL1 = FWKEY | WRT;         // write
    *(volatile uint16_t*)(*dst) = w;
    while (FCTL3 & BUSY) { }
    FCTL1 = FWKEY;               // clear WRT
    FCTL3 = FWKEY | LOCK;        // lock

    *dst += 2;                   // advance 1 word

    if (gie) __bis_SR_register(GIE);
}

// 'L' - Build the LDR calibration table. The servo points at 90 deg; the user
// presses PB0 once for each of 10 light levels, or PB1 to stop. Samples are
// packed in pairs (5 bytes per pair) and stored in info segment D.
void save_table_ldr(void)
{
    uint8_t  out[10];          // 2 packets x 5 bytes
    uint8_t  out_idx   = 0;    // 0 or 5
    uint8_t  in_chunk  = 0;    // number of 5B packets in 'out' (0..1)

    uint16_t prev_ldr1 = 0, prev_ldr2 = 0;
    uint8_t  have_prev = 0;    // 1 when the first sample of a pair is stored

    ldr_manual_smaple = 0;
    ldr_manual_stop   = 0;
    uint8_t trig = 0;


    servo_set_angle(90);

    while( ldr_manual_stop == 0 && trig != 10 )
    {
        // Show live readings until PB0 (sample) or PB1 (stop)
        lcd_clear();
        while (ldr_manual_smaple == 0 && ldr_manual_stop == 0) {
            lcd_home();
            ldr_to_lcd(ldr1, ldr2);
        }
        if (ldr_manual_stop) break;
        ldr_manual_smaple = 0;

        // atomic snapshot
        __disable_interrupt();
        uint16_t cur_ldr1 = ldr1;
        uint16_t cur_ldr2 = ldr2;
        __enable_interrupt();

        if (!have_prev) {
            // First sample of a pair
            prev_ldr1 = cur_ldr1;
            prev_ldr2 = cur_ldr2;
            have_prev = 1;
        } else {
            // Second sample: pack the pair (even, odd) into 5 bytes
            create_data_ldr_table(&out[out_idx], prev_ldr1, prev_ldr2, cur_ldr1, cur_ldr2);
            out_idx  += 5;
            in_chunk += 1;

            if (in_chunk == 2) {
                on_ldr_line_received(out);  // 10 bytes
                out_idx  = 0;
                in_chunk = 0;
            }

            have_prev = 0;              // the next sample starts a new pair
        }

        lcd_clear();
        lcd_home();
        lcd_puts("choice:");
        lcd_new_line();
        int i =0;
        for (; i<2 ; i++)
            ldr_to_lcd(cur_ldr1, cur_ldr2);
        trig++;
    }

    // Pad a single remaining packet to 10 bytes
    if (in_chunk == 1) {
        out[5] = out[6] = out[7] = out[8] = out[9] = 0xFF; // padding
        on_ldr_line_received(out);
    }
    lcd_clear();
    lcd_puts("done BP1 to exit");
    ldr_manual_stop = 0;
    while (ldr_manual_stop == 0) enterLPM(mode0);
}

// Last values shown by ldr_to_lcd()
int lcd_prev_l1 = 0;
int lcd_prev_l2 = 0;

// Show "L1:<v>  L2:<v>" on the LCD (only when a value changed)
void ldr_to_lcd(int l1,int l2)
{
    if (l1 == lcd_prev_l1 && l2 == lcd_prev_l2)
        return;

    char msg1 [17];                     // "L1:1023  L2:1023" = 16 chars + NUL
    char msg2 [17];
    lcd_clear();
    shirshur("L1:", l1, "  L2:", msg1);
    shirshur(msg1, l2, "", msg2);
    int i=0;
    for (; i<10 ; i++)
    {
        lcd_clear();
        lcd_puts(msg2);
    }
    lcd_prev_l1 = l1;
    lcd_prev_l2 = l2;

}

// Store one 10-byte LDR record in segment D: [FOOTER_MAGIC][5 words]
void on_ldr_line_received(uint8_t *buf)
{
    // Record marker
    flash_write_word64(&file_ldr_ptr, FOOTER_MAGIC);

    // Write the 10 bytes as 5 words (little-endian: lo|hi<<8)
    uint8_t i = 0;
    for (; i < 10; i += 2) {
        uint16_t w = (uint16_t)buf[i] | ((uint16_t)buf[i + 1] << 8);
        flash_write_word64(&file_ldr_ptr, w);
    }
}

// --- LDR table: read ---

// Fill out[5] with the 5 bytes at logical offset i*5 (i in 0..4).
// Returns 1 on success, 0 on failure (out of range / no data).
int ldr_read_5_by_index(uint8_t out[5], int i)
{
    if (!out || i < 0 || i > 4) return 0;

    // Preserve current read pointer, then reset to segment start
    uint16_t saved_ptr = file_ldr_ptr;
    file_ldr_ptr = SEG_D_START;

    // Which 10-byte record and which half (0: first 5B, 1: second 5B)?
    int rec  = i / 2;
    int half = i & 1;

    uint8_t buf10[10];

    // Advance to the target record
    int r = 0;
    for (; r <= rec; r++) {
        file_read_ldr_table(buf10);
        if (!file_ldr_read_ok) {
            // Restore pointer and fail
            file_ldr_ptr = saved_ptr;
            return 0;
        }
    }

    // Copy the requested 5-byte half
    uint8_t base = (half == 0) ? 0 : 5;
    uint8_t k = 0;
    for (; k < 5; k++) {
        out[k] = buf10[base + k];
    }

    // Restore original read pointer
    file_ldr_ptr = saved_ptr;
    return 1;
}

// Extract field idx0_3 (0..3) from a 5-byte packet (inverse of create_data_ldr_table)
static inline uint16_t unpack_4x10_from5(const uint8_t *p5, uint8_t idx0_3)
{
    // Guard
    if (idx0_3 > 3) return 0;

    // Rebuild each 10-bit value (mirror of create_data_ldr_table)
    uint16_t a = (uint16_t)p5[0] | ((uint16_t)(p5[1] & 0x03) << 8);

    uint16_t b = (uint16_t)((p5[1] >> 2) & 0x3F)
               | (uint16_t)((p5[2] & 0x0F) << 6);

    uint16_t c = (uint16_t)((p5[3] & 0x3F) << 4)
               | (uint16_t)(p5[2] >> 4);

    uint16_t d = (uint16_t)((uint16_t)p5[4] << 2)
               | (uint16_t)((p5[3] >> 6) & 0x03);

    switch (idx0_3) {
        case 0: return (a & 0x03FF); // prev LDR1
        case 1: return (b & 0x03FF); // prev LDR2
        case 2: return (c & 0x03FF); // curr LDR1
        default:return (d & 0x03FF); // curr LDR2
    }
}

// Return sample sample_pos (0/1) of LDR ldr_num (1/2) from a 5-byte packet, or -1
int ldr_unpack(const uint8_t samples[5], int ldr_num, int sample_pos)
{
    if (!samples || (ldr_num != 1 && ldr_num != 2) || (sample_pos != 0 && sample_pos != 1))
        return -1;

    // Map to the packed field index inside the 5-byte row:
    // LDR1: prev=idx 0, curr=idx 2
    // LDR2: prev=idx 1, curr=idx 3
    uint8_t idx;
    if (ldr_num == 1) idx = (sample_pos == 0) ? 0 : 2;
    else              idx = (sample_pos == 0) ? 1 : 3;

    return (int)unpack_4x10_from5((const uint8_t*)samples, idx);
}


// Calibration value sample_num (0..9) of LDR ldr_num (1/2); -1 on error
int ldr_table(uint16_t ldr_num, uint16_t sample_num)
{
    if (ldr_num != 1 && ldr_num != 2) return -1;
    if (sample_num > 9) return -1;

    // Row and position within row
    uint16_t row = sample_num / 2;      // 0..4
    uint16_t pos = sample_num & 1;      // 0=prev, 1=curr

    uint8_t row5[5];
    if (!ldr_read_5_by_index(row5, row))  // fetch 5-byte row
        return -1;

    return ldr_unpack(row5, ldr_num, pos);
}


// Read the next 10-byte record from segment D (after the next FOOTER_MAGIC).
// Sets file_ldr_read_ok = 1 on success.
void file_read_ldr_table(uint8_t *out)
{
    file_ldr_read_ok = 0;

    // Clamp pointer to segment bounds
    if (file_ldr_ptr < SEG_D_START || file_ldr_ptr > SEG_D_END) {
        file_ldr_ptr = SEG_D_START;
    }

    uint16_t rp = file_ldr_ptr;

    // Scan forward (word steps) for FOOTER_MAGIC
    while (rp + 1u <= SEG_D_END) {
        uint16_t w = *(const volatile uint16_t*)(uintptr_t)rp;
        if (w == FOOTER_MAGIC) break;
        rp += 2;
    }

    // Not found or no room for 10 bytes after the marker?
    if (rp + 2u + 10u - 1u > SEG_D_END) {
        file_ldr_ptr = rp;     // leave at current place
        return;                 // file_ldr_read_ok stays 0
    }

    // Skip marker and read 5 words => 10 bytes (little-endian)
    rp += 2;
    uint8_t i = 0;
    for (; i < 10; i += 2) {
        uint16_t w = *(const volatile uint16_t*)(uintptr_t)rp;
        out[i]     = (uint8_t)(w & 0xFF);
        out[i + 1] = (uint8_t)(w >> 8);
        rp += 2;
    }

    file_ldr_ptr    = rp;  // next record position
    file_ldr_read_ok = 1;
}

// Pack two 10x10-bit arrays that reside in SEG_C into 25 bytes (interleaved A[k],B[k]).
static inline void pack_two_ldr_arrays_from_segc(uint8_t buf[25],
                                                 uint16_t arr1_base,
                                                 uint16_t arr2_base)
{
    uint32_t bitbuf   = 0;
    uint8_t  bitcount = 0;
    uint8_t  out_idx  = 0;
    uint8_t k = 0;
    for (; k < 10; k++) {
        uint16_t a = read_seg_c_arr(arr1_base, k);
        uint16_t b = read_seg_c_arr(arr2_base, k);
        if (a == 0xFFFFu) a = 0;           // treat "not written" as 0
        if (b == 0xFFFFu) b = 0;
        a &= 0x03FFu;                       // 10-bit clamp
        b &= 0x03FFu;

        // push arr1[k]
        bitbuf   |= ((uint32_t)a) << bitcount;
        bitcount += 10;
        while (bitcount >= 8) {
            buf[out_idx++] = (uint8_t)(bitbuf & 0xFFu);
            bitbuf   >>= 8;
            bitcount -= 8;
        }

        // push arr2[k]
        bitbuf   |= ((uint32_t)b) << bitcount;
        bitcount += 10;
        while (bitcount >= 8) {
            buf[out_idx++] = (uint8_t)(bitbuf & 0xFFu);
            bitbuf   >>= 8;
            bitcount -= 8;
        }
    }

    if (bitcount > 0) buf[out_idx++] = (uint8_t)(bitbuf & 0xFFu);
    while (out_idx < 25) buf[out_idx++] = 0;
}

// Send 25 bytes as five 5-byte binary frames
static inline void send_ldr25_in_5_chunks(const uint8_t buf[25])
{
    uint8_t i = 0;
    for (; i < 5; i++) {
        dynamic_transmit_len(&buf[i * 5], 5);
    }
}


//=====================================================================
//  11. Light scans
//=====================================================================

// 'C' - Light scan 0..180: one 4-byte frame per degree, then '@', then the
// calibration table (2 x 10 values x 10 bits) as 25 bytes.

void scan_sweep_light(void)
{
    // Copy the calibration table into SEG_C (saves RAM)
    segc_erase();                              // erase C once at session start
    uint16_t ptr  = SEG_C_START;               // allocation cursor
    uint16_t led1_arr = def_seg_c_arr(&ptr, 10);   // reserve 10 words for LDR1
    uint16_t led2_arr = def_seg_c_arr(&ptr, 10);   // reserve 10 words for LDR2

    if (led1_arr == 0 || led2_arr == 0) {
        // does not fit in 64 bytes
        return;
    }

    // Fill 10 calibration samples per LDR into SEG_C
    uint8_t k = 0;
    for (; k < 10; k++) {
        int v1 = ldr_table(1, k); if (v1 < 0) v1 = 0; if (v1 > 1023) v1 = 1023;
        int v2 = ldr_table(2, k); if (v2 < 0) v2 = 0; if (v2 > 1023) v2 = 1023;
        write_seg_c_arr_at(led1_arr, k, (uint16_t)v1);
        write_seg_c_arr_at(led2_arr, k, (uint16_t)v2);
    }

    // Sweep and send one 4-byte frame per degree
    servo_set_angle(0);
    wait_for_servo();

    uint8_t out[4];
    uint8_t i = 0;
    for (; i <= 180; i++) {
        servo_set_angle(i);
        wait_for_servo();

        uint16_t v1, v2;
        disable_interrupts();        // snapshot against ISR updates
        v1 = ldr1;
        v2 = ldr2;
        enable_interrupts();

        create_data_to_send_light(out, v1, v2, i);
        dynamic_transmit_len(out, 4);
    }

    const uint8_t endb = '@';           // end of scan data
    dynamic_transmit_len(&endb, 1);
    // Send the calibration table
    uint8_t buf25[25];
    pack_two_ldr_arrays_from_segc(buf25, led1_arr, led2_arr);  // fills exactly 25 bytes
    send_ldr25_in_5_chunks(buf25);
}

// Combined frame (6 bytes, LSB first): L1[9:0] | L2[9:0] | PERIOD[15:0] | DEG[7:0]
static inline void create_data_to_send_combo6(uint8_t out[6],
                                              uint16_t l1, uint16_t l2,
                                              uint16_t period, uint8_t deg)
{
    // 44 bits pushed LSB first: L1[9:0], L2[9:0], PERIOD[15:0], DEG[7:0]
    uint32_t bitbuf = 0;
    uint8_t  bitcount = 0;
    uint8_t  idx = 0;

    // Push the low nbits of v into the bit buffer
    #define PUSH_BITS(v, nbits)                         \
        do {                                            \
            bitbuf |= ((uint32_t)(v) & ((1ul<<(nbits))-1)) << bitcount; \
            bitcount += (nbits);                        \
            while (bitcount >= 8) {                     \
                out[idx++] = (uint8_t)(bitbuf & 0xFF);  \
                bitbuf >>= 8;                           \
                bitcount -= 8;                          \
            }                                           \
        } while (0)

    PUSH_BITS(l1,     10);
    PUSH_BITS(l2,     10);
    PUSH_BITS(period, 16);
    PUSH_BITS(deg,     8);

    // flush remaining bits (should be <= 7)
    if (bitcount > 0 && idx < 6) {
        out[idx++] = (uint8_t)(bitbuf & 0xFF);
        bitbuf >>= 8;
        bitcount = 0;
    }

    // Pad to exactly 6 bytes (defensive; normally idx==6)
    while (idx < 6) out[idx++] = 0;

    #undef PUSH_BITS
}

// 'D' - Full scan: LDRs + sonar at every angle in [deg_start, deg_end_exclusive),
// one 6-byte frame per degree, then '@', then the calibration table.
void scan_sweep_light_sonar(uint8_t deg_start, uint8_t deg_end_exclusive)
{
    // Copy the calibration table into SEG_C (saves RAM)
    segc_erase();                              // erase C once at session start
    uint16_t ptr  = SEG_C_START;               // allocation cursor
    uint16_t led1_arr = def_seg_c_arr(&ptr, 10);   // reserve 10 words for LDR1
    uint16_t led2_arr = def_seg_c_arr(&ptr, 10);   // reserve 10 words for LDR2

    if (led1_arr == 0 || led2_arr == 0) {
        // does not fit in 64 bytes
        return;
    }

    // Fill 10 calibration samples per LDR into SEG_C
    uint8_t k = 0;
    for (; k < 10; k++) {
        int v1 = ldr_table(1, k); if (v1 < 0) v1 = 0; if (v1 > 1023) v1 = 1023;
        int v2 = ldr_table(2, k); if (v2 < 0) v2 = 0; if (v2 > 1023) v2 = 1023;
        write_seg_c_arr_at(led1_arr, k, (uint16_t)v1);
        write_seg_c_arr_at(led2_arr, k, (uint16_t)v2);
    }

    // Sweep and send one 6-byte frame per degree
    servo_set_angle(deg_start);
    wait_for_servo();

    uint8_t   pkt[6];
    uint16_t  period, v1, v2;
    uint8_t i = deg_start;
    for (; i < deg_end_exclusive; i++)
    {
        servo_set_angle(i);
        wait_for_servo();           // mechanical settle

        // Atomic snapshot (the ADC ISR updates ldr1/ldr2)
        disable_interrupts();
        v1 = ldr1;                  // 0..1023 (10-bit effective)
        v2 = ldr2;                  // 0..1023 (10-bit effective)
        enable_interrupts();

        // Sonar reading at this angle
        period = go_sonar_trow(i, 1);

        // Pack & send 6-byte binary frame (HEADER=0xFF)
        create_data_to_send_combo6(pkt, v1, v2, period, (uint8_t)i);
        dynamic_transmit_len(pkt, 6);
    }
    const uint8_t endb = '@';           // end of scan data
    dynamic_transmit_len(&endb, 1);
    // Send the calibration table
    uint8_t buf25[25];
    pack_two_ldr_arrays_from_segc(buf25, led1_arr, led2_arr);  // fills exactly 25 bytes
    send_ldr25_in_5_chunks(buf25);
}


//=====================================================================
//  12. File records in SEG7/SEG8 (legacy)
//=====================================================================

// NOTE: the current file system keeps its records in SEG5. Only
// send_all_files_with_types() still reads this area.

// Helper: check which segment the address belongs to, and return its bounds.
// Returns 1 on success (bounds filled), 0 if 'a' is outside Seg7/Seg8.
static inline uint8_t get_seg7n8_bounds(uint16_t a, uint16_t *seg_start, uint16_t *seg_end)
{
    if (a <= SEG7_START && a >= SEG7_END) { *seg_start = SEG7_START; *seg_end = SEG7_END; return 1; }
    if (a <= SEG8_START && a >= SEG8_END) { *seg_start = SEG8_START; *seg_end = SEG8_END; return 1; }
    return 0;
}
static inline void flash_write_word7n8_any(uint16_t *addr, uint16_t w)
{
    uint16_t a = *addr;
    uint16_t seg_start = 0, seg_end = 0;

    // Resolve segment bounds
    if (!get_seg7n8_bounds(a, &seg_start, &seg_end)) return;

    // Alignment and in-range checks (descending write; word must remain in [seg_end..seg_start])
    if (a & 1u)                         return;              // even address only
    if (a > seg_start)                  return;              // above segment
    if (a < seg_end)                    return;              // below segment
    if ((uint16_t)(a - 1u) < seg_end)   return;              // second byte would underflow segment

    // Disable interrupts during flash operation
    unsigned int gie = __get_SR_register() & GIE;
    __disable_interrupt();

    // First use on this segment? Erase once when writing the very first word at SEGx_START
    if (a == seg_start) {
        (void)delete_segment7n8(seg_start);  // ignore return; safe to proceed
    }

    // Program the word
    FCTL3 = FWKEY;               // unlock
    FCTL1 = FWKEY | WRT;         // write mode
    *(volatile uint16_t*)a = w;  // program word
    while (FCTL3 & BUSY) { }     // wait
    FCTL1 = FWKEY;               // clear WRT
    FCTL3 = FWKEY | LOCK;        // lock

    // Advance caller's pointer DOWN by one word (2 bytes)
    *addr = (uint16_t)(a - 2u);

    // Restore interrupt state
    if (gie) __bis_SR_register(GIE);
}
static inline uint8_t is_main7n8_seg_start(uint16_t s)
{
    // Accept only Segment 7 or 8 starts
    return (s == SEG7_START) || (s == SEG8_START);
}
// Erase a 512B Main-Flash segment (Seg7/Seg8). Returns 1 on success, 0 on reject.
static inline uint8_t delete_segment7n8(uint16_t seg_start)
{
    // Accept only Seg7/Seg8 and require 512B alignment
    if (!is_main7n8_seg_start(seg_start))        return 0;
    if (seg_start & (MAIN_SEG_SIZE - 1u))       return 0;

    // Save IRQ state and disable interrupts during flash operation
    unsigned int gie = __get_SR_register() & GIE;
    __disable_interrupt();

    // Unlock flash and enter ERASE mode
    FCTL3 = FWKEY;                 // unlock (clear LOCK)
    FCTL1 = FWKEY | ERASE;         // set ERASE
    *(volatile uint8_t*)seg_start = 0;  // dummy write to trigger segment erase

    // Wait for controller to finish
    while (FCTL3 & BUSY) { /* wait */ }

    // Exit ERASE mode and re-lock
    FCTL1 = FWKEY;                 // clear ERASE
    FCTL3 = FWKEY | LOCK;          // lock

    // Restore interrupt state
    if (gie) __bis_SR_register(GIE);

    return 1;
}
// Convenience: erase both segments; returns 1 only if both succeed
static inline uint8_t seg7n8_erase_all(void)
{
    uint8_t ok7 = delete_segment7n8(SEG7_START);
    uint8_t ok8 = delete_segment7n8(SEG8_START);
    return (uint8_t)(ok7 & ok8);
}
void delete_stract_segs()
{
    seg7n8_erase_all();
}
// Read 16-bit from flash-mapped memory
static inline uint16_t file_rd16(uint16_t addr)
{
    return *(volatile const uint16_t*)(uintptr_t)addr;
}
// Generic u16 field reader using word-index * 2-byte stride
static inline uint16_t file_get_field_u16(uint16_t arr, file_field_t field)
{
    // byte offset = field_index * 2
    return file_rd16((uint16_t)(arr + ((uint16_t)field << 1)));
}
// Typed getters (use the enum-based accessor)
// NAME: expands 2 stored bytes into out[16]
static inline void file_get_name(uint16_t arr, char out16[16])
{
    uint16_t w = file_get_field_u16(arr, FILE_NAME);
    out16[0] = (char)(w & 0xFF);
    out16[1] = (char)((w >> 8) & 0xFF);
    out16[2] = '\0';
    uint8_t i = 3;
    for ( ;i < 16; ++i) out16[i] = 0;
}
// TYPE: uint8_t (cast from stored 0/1 u16)
static inline uint8_t file_get_type(uint16_t arr)
{
    return (uint8_t)(file_get_field_u16(arr, FILE_TYPE) & 0x01u);
}

// List the files by type (reads the SEG7 records)
void send_all_files_with_types(void)
{
    char send_line[16] = {0};
    char tmp1[17]      = {0};
    char tmp2[18]      = {0};
    char name_buf[17]  = {0};
    uint16_t idx;

    // ---- TXT files ----
    dynamic_transmit("txt:\n");
    for (idx = 0; idx < (uint16_t)file_num; idx++) {
        // Compute record base address and align to even
        uint16_t rec      = (uint16_t)(SEG7_START + (uint16_t)(idx * FILE_REC_BYTES));
        uint16_t rec_even = (uint16_t)(rec & (uint16_t)~1u);

        // Filter by type
        uint8_t typ = file_get_type(rec_even);
        if (typ == (uint8_t)file_txt) {
            // Read short name (2 bytes stored, expanded to char[16])
            uint8_t k=0;
            for (;k<17;k++) name_buf[k]=0;
            file_get_name(rec_even, name_buf);

            // Build "File-<idx> <name>\n" into send_line (reuse temps each time)
            tmp1[0]=tmp2[0]=send_line[0]=0;
            str_concat(tmp1, " ", name_buf);
            str_concat(tmp2, tmp1, " \n");
            shirshur("File-", (int)idx, tmp2, send_line);
            dynamic_transmit(send_line);
        }
    }

    // ---- Script files ----
    dynamic_transmit("Script:\n");
    for (idx = 0; idx < (uint16_t)file_num; idx++) {
        uint16_t rec      = (uint16_t)(SEG7_START + (uint16_t)(idx * FILE_REC_BYTES));
        uint16_t rec_even = (uint16_t)(rec & (uint16_t)~1u);

        uint8_t typ = file_get_type(rec_even);
        if (typ == (uint8_t)file_script) {
            uint8_t k=0;
            for (;k<17;k++) name_buf[k]=0;
            file_get_name(rec_even, name_buf);

            tmp1[0]=tmp2[0]=send_line[0]=0;
            str_concat(tmp1, " ", name_buf);
            str_concat(tmp2, tmp1, " \n");
            shirshur("File-", (int)idx, tmp2, send_line);
            dynamic_transmit(send_line);
        }
    }
}


//=====================================================================
//  13. Interrupt service routines
//=====================================================================

// --- UART ---

// Accept EOF sent as a framed "string": "#", "#\n", "#\r", "#\r\n", optional trailing NUL.
static inline uint8_t is_eof_payload(const uint8_t *data, uint8_t len)
{
    if (!data || len == 0u) return 0;

    // Fast path
    if (len == 1u) return (data[0] == EOF_CHAR);

    uint8_t i = len;

    // Trim one trailing NUL
    if (i > 0 && data[i - 1] == '\0') i--;

    // Trim CR/LF
    while (i > 0 && (data[i - 1] == '\r' || data[i - 1] == '\n')) i--;

    // After trimming, expect exactly single EOF_CHAR
    return (i == 1u && data[0] == EOF_CHAR);
}


// UART TX ISR: sends one byte of the current frame per interrupt
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(USCIAB0TX_VECTOR))) USCI0TX_ISR(void)
#else
#error Compiler not supported!
#endif
{
    // Framed TX: [HDR][LEN][DATA...][CHK]
    if (TXF_mode) {
        switch (TXF_phase)
        {
            // SEND HEADER
            case TXF_HDR: UCA0TXBUF = tx_current_HDR; TXF_phase = TXF_LEN; return;
            // SEND LENGTH
            case TXF_LEN: UCA0TXBUF = (uint8_t)TX_total_len;
                          TXF_phase = (TX_total_len == 0) ? TXF_CHK : TXF_DATA; return;
            // SEND DATA
            case TXF_DATA:
                if (TX_remaining > 0) {
                    UCA0TXBUF = (uint8_t)transmit_str[transmit_char_index++];
                    TX_remaining--;
                    return;
                }
                else { TXF_phase = TXF_CHK; return; }
            // SEND CHECKSUM
            case TXF_CHK:
                UCA0TXBUF = TX_checksum;         // CHECKSUM (precomputed: LENGTH + DATA sum8)
                // Frame sent: now wait for the PC's ACK token
                tx_ack_state = ACK_WAIT;
                TXF_phase = TXF_DONE;
                return;
            // DONE
            case TXF_DONE:
                IE2 &= ~UCA0TXIE;
                transmit_busy = 0;
                TXF_mode = 0;

                // Flush a deferred token (our reply to PC) if any
                if (tx_token_sched == TX_TOKEN_DEFERRED) {
                    tx_token_sched = TX_TOKEN_IDLE;
                    UCA0TXBUF = tx_token_byte;   // one raw byte (0xCC/0xBB)
                }
                EXIT_LPM(lpm_mode);
                return;
        }
        return;
    }
}

// Handle framed FILE payloads. Returns FILE_* status.
static inline uint8_t file_data_handler(const uint8_t* data, uint8_t len)
{
    // Strict EOF recognition (supports "#", "#\n", "#\r", "#\r\n", with/without NUL)
    if (is_eof_payload(data, len)) {
        return on_file_line_received((uint8_t*)0, 0, TYPE_FILE_EOF);
    }

    // Regular chunk path: write exactly 'len' bytes (0-length is a no-op)
    if (len != 0u) {
        return on_file_line_received((uint8_t*)data, (int)len, TYPE_FILE_CHUNK);
    }

    // Empty frame: nothing to write
    return FILE_OK;
}

// RX frame state machine, called for every received byte.
// Returns 1 when a frame is complete, 2 when the file system is full, 0 otherwise.
static inline uint8_t rx_handle_frame_byte(uint8_t b, enum RXMode mode)
{
    switch (RXF_phase)
    {
        // HEADER
        case RXF_HDR:
            if (b == UART_DATA_HDR || b == UART_BIN_HDR) {
                RXF_phase       = RXF_LEN;
                RXF_len_need    = 0;
                RXF_sum         = 0;
                RXF_status      = 0;
                receive_char_index = 0;
            }
            break;

        // LENGTH
        case RXF_LEN:
            RXF_len_need = b;              // payload length
            RXF_sum      = b;              // checksum accum starts from LENGTH
            receive_char_index = 0;
            RXF_phase = (RXF_len_need == 0) ? RXF_CHK : RXF_DATA;
            if (RXF_len_need > RX_BUF_CAPACITY) { RXF_status = 2; }
            break;

        // DATA BYTES
        case RXF_DATA:
            if (receive_char_index < RX_BUF_CAPACITY) {
                receive_str[receive_char_index] = (char)b;
            } else {
                RXF_status = 2; // overflow
            }
            receive_char_index++;
            RXF_sum = (uint8_t)(RXF_sum + b);
            if (receive_char_index >= RXF_len_need) { RXF_phase = RXF_CHK; }
            break;

        // CHECKSUM
        case RXF_CHK:
        {
            uint8_t chk = b;
            uint8_t ok  = (RXF_sum == chk) && (RXF_status == 0);
            uint8_t res = 1;

            // Null-terminate (receive_str has room for RX_BUF_CAPACITY + 1 bytes)
            if (RXF_len_need <= RX_BUF_CAPACITY) {
                receive_str[RXF_len_need] = '\0';
            }

            if (ok && pc_ready) {
                if (mode == RXM_FILE) {
                    // File mode: pass raw payload (byte array) to the file handler
                    // NOTE: receive_str holds the payload, size RXF_len_need
                    uint8_t space_error = file_data_handler((const uint8_t*)receive_str, RXF_len_need);
                    if (space_error == FILE_NO_SPACE) {
                        file_got_shtroodle = 2;
                        res = 2;
                    }
                }
                else
                {
                    // Normal mode: text command or data
                    on_string_received((const char*)receive_str);
                }
            }

            // Answer only after the payload was handled: a flash write can take
            // ~15 ms (segment erase), and the PC must not send the next frame
            // before that. A bad frame gets NACK and the PC sends it again.
            uart_send_token_or_defer(ok ? UART_ACK_TOKEN_OK : UART_ACK_TOKEN_FAIL);

            RXF_phase = RXF_HDR;
            return res;
        } break;
    }

    return 0;
}

// Helper: send one-byte ACK token immediately (unframed)
static inline void uart_send_ack_token(uint8_t token)
{
    while (!(IFG2 & UCA0TXIFG)) { /* spin */ }
    UCA0TXBUF = token;
}

// UART RX ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    uint8_t b = UCA0RXBUF;  // read once

    // 1) ACK / NACK for the frame we just sent
    if (tx_ack_state == ACK_WAIT) {
        if (b == UART_ACK_TOKEN_OK || b == UART_ACK_TOKEN_FAIL) {
            tx_ack_state = (b == UART_ACK_TOKEN_OK) ? ACK_OK : ACK_FAIL;
            EXIT_LPM(lpm_mode);
            return;
        }
        // not a token: handle it as data
    }

    // 2) Host readiness token
    if (b == PC_READY_TOKEN && pc_ready == 0) {
        pc_ready = 1;
        tx_ack_state = ACK_IDLE;
        EXIT_LPM(lpm_mode);
        return;
    }

    // 3) A raw EOF_CHAR between frames closes the file being received
    if (file_reciving && (RXF_phase == RXF_HDR) && (b == EOF_CHAR_U8)) {
        uint8_t st = on_file_line_received((uint8_t*)0, 0, TYPE_FILE_EOF);
        uart_send_ack_token((st == FILE_EOF_DONE) ? UART_ACK_TOKEN_OK : UART_ACK_TOKEN_FAIL);
        EXIT_LPM(lpm_mode);
        return;
    }

    // 4) Framed data
    uint8_t exit_res = rx_handle_frame_byte(b, file_reciving ? RXM_FILE : RXM_NORMAL);
    if (exit_res) EXIT_LPM(lpm_mode);
}


// --- Push buttons (Port 2): PB0 = take LDR sample, PB1 = stop ---
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void Port2_handler(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port2_handler (void)
#else
#error Compiler not supported!
#endif
{

    DelayUs(debounceVal);

    if(PBsArrIntPend & PB0) {
       PBsArrIntPend &= ~PB0;
       ldr_manual_smaple = 1;
    }
    else
    if(PBsArrIntPend & PB1) {
       PBsArrIntPend &= ~PB1;
       ldr_manual_stop = 1;
    }


    EXIT_LPM(lpm_mode);
}

// --- ADC10: sequence done, copy the LDR results ---
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC10_VECTOR))) ADC10_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if (state != state0)
    {
        ADC10CTL0 &= ~ADC10IFG;                  // clear flag

        // DTC writes in order: [0]=A3, [1]=A2, [2]=A1, [3]=A0
        ldr1 = ldr_arr[3];
        ldr2 = ldr_arr[0];
        LDR_ready = 1;

        EXIT_LPM(lpm_mode);
    }
}

// --- Timer A0 CCR0 (every PWM period, 40 Hz): ADC trigger, clock, servo timing ---
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    if (1)
    {
        LDR_sample();                   // start an ADC sequence

        servo_timer_ticks++;
        scan_40HZ_tick_wait++;
        if (servo_timer_ticks == 40)
        {
            servo_timer_ticks = 0;
            servo_timer_sec++;
            if (servo_timer_sec == 60)
            {
                servo_timer_sec = 0;
                servo_timer_min++;
            }
        }

        // Servo power-off countdown (detach at a period boundary)
        if (servo_is_attached && servo_cycles_left > 0) {
            servo_cycles_left--;
            if (servo_cycles_left == 0) {
                servo_detach();   // no last "twitch"
            }
        }

        // End of wait_for_servo()
        if (servo_en_wait)
        {
            if (scan_40HZ_tick_wait < scan_40HZ_tick_limit){}
            else
            {
                servo_en_wait = 0;
                EXIT_LPM(lpm_mode);
            }
        }
        sonar_speed_40HZ++;
    }
}

// --- Timer A0 CCR1/CCR2/TAIFG: 10 ms script tick on CCR2 ---
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) TIMER0_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(TA0IV, TA0IV_TAIFG)) {
        case TA0IV_NONE:
            break;                                  // No interrupt
        case TA0IV_TACCR1:
            // PWM compare: handled in hardware
            break;
        case TA0IV_TACCR2: {
            // Schedule next 10ms tick relative to previous compare to avoid jitter
            uint16_t next = TA0CCR2 + TICK10_DELTA;
            if (next >= PWM_PERIOD) next -= PWM_PERIOD;
            if (next == 0) next = 1;
            TA0CCR2 = next;

            // Script timing: wake the main loop when the script delay has passed
            if (tick10ms_hook()) EXIT_LPM(mode0);


            break;
        }
        case TA0IV_TAIFG:
            // Overflow in Up mode corresponds to CCR0 wrap; not used here
            break;
        default:
            break;
    }
}


// --- Timer A1 CCR0: sonar timeout ---
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A0_VECTOR
__interrupt void TA1_CCR0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TA1_CCR0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    sonar_speed_17HZ++;
    if (sonar_active)
    {
        TA1CCTL0 &= ~CCIE;
        sonar_active   = 0;
        sonar_time_out = 1;
        __bic_SR_register_on_exit(LPM0_bits);
    }
}

// --- Timer A1 CCR1/CCR2: echo capture on TA1.2 (rising, then falling edge) ---
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) TIMER1_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(TA1IV, 0x0A))      // who caused the interrupt?
  {
      case  TA1IV_NONE: break;              // Vector  0:  No interrupt
      case  TA1IV_TACCR1:                   // Vector  2:  TACCR1 CCIFG
        break;
      case TA1IV_TACCR2:
            if (sonar_input_counter == 0)                 // rising edge
            {
                timer1_a2_capture_start = TA1CCR2;            // Capture start time
                sonar_input_counter = 1;

                // Next Capture on falling edge
                TA1CCTL2 = (TA1CCTL2 & ~CM_3) | (CM_2 | CCIS_0 | SCS | CAP | CCIE);
                TA1CCTL2 &= ~CCIFG;
            }
            else                                         // falling edge
            {
                timer1_a2_capture_end = TA1CCR2;              // Capture end time
                sonar_input_counter = 0;

                sonar_done = 1; // sonar echo capture done

                // Next capture on rising edge
                TA1CCTL2 = (TA1CCTL2 & ~CM_3) | (CM_1 | CCIS_0 | SCS | CAP | CCIE);
                TA1CCTL2 &= ~CCIFG;
                sonar_active = 0;

                __bic_SR_register_on_exit(LPM0_bits);    // wake get_period()
            }

            break;
      case TA1IV_TAIFG: break;              // Vector 10:  TAIFG
      default:  break;
  }
}
