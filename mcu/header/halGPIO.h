#ifndef HALGPIO_H
#define HALGPIO_H

/*
 * halGPIO.h - Hardware Abstraction Layer interface
 *
 * UART protocol, file-system and script-timing definitions, the state shared
 * between the ISRs and the API layer, and the driver functions in halGPIO.c.
 * Pin mapping and the flash memory map are in bsp.h.
 */

#include <header/bsp.h>
#include <stdint.h>

//---------------------------------------------------------------------
//  UART protocol
//  Frame: [HDR][LEN][DATA...][CHK], CHK = (LEN + sum(DATA)) & 0xFF
//---------------------------------------------------------------------
#define UART_DATA_HDR       0xAA        // Text payload
#define UART_BIN_HDR        0xFF        // Binary payload
#define UART_ACK_TOKEN_OK   0xCC        // Raw token: frame OK
#define UART_ACK_TOKEN_FAIL 0xBB        // Raw token: frame bad
#define PC_READY_TOKEN      0xABu       // PC -> MCU: host is ready

#define RX_BUF_CAPACITY     16

typedef enum {
    ACK_WAIT = 0,                       // Expecting a token now
    ACK_OK   = 1,
    ACK_FAIL = 2,
    ACK_IDLE = 3                        // Not waiting
} ack_state_t;

typedef enum {
    TX_TOKEN_IDLE     = 0,              // No pending token
    TX_TOKEN_DEFERRED = 1               // Send a token as soon as TX is free
} tx_token_sched_t;

enum RXMode { RXM_NORMAL = 0, RXM_FILE = 1 };

//---------------------------------------------------------------------
//  File system
//---------------------------------------------------------------------
#define EOF_CHAR            '#'
#define EOF_CHAR_U8         ((uint8_t)('#'))    // 0x23

#define TYPE_STRING         0x01
#define TYPE_FILE_CHUNK     0x02
#define TYPE_FILE_EOF       0x03
#define TYPE_FILE_ABORT     0x04

#define MAX_FILES           9           // Size of files[] (RAM: 24 bytes per record)
#define FILES_WORD_SIZE     16          // Max file name length
#define FILE_REC_BYTES      10          // File record: 5 words
#define FOOTER_MAGIC        0xAA55      // Marks end of file data
#define META_MAGIC          ((uint16_t)(uint8_t)'#')

enum {
    FILE_OK       = 0,                  // Chunk appended, or no-op
    FILE_EOF_DONE = 1,                  // File closed successfully
    FILE_NO_SPACE = 2                   // Not enough flash space
};

// Field indexes of a 5-word file record: [NAME][LEN][TYPE][PTR][END_PTR]
typedef enum {
    FILE_NAME    = 0,
    FILE_LEN     = 1,
    FILE_TYPE    = 2,
    FILE_PTR     = 3,
    FILE_END_PTR = 4
} file_field_t;

//---------------------------------------------------------------------
//  Script engine timing
//---------------------------------------------------------------------
#define TICK10_DELTA        ((PWM_PERIOD * 2u) / 5u)    // 10 ms in timer ticks (0.4 x PWM period)
#define SCRIPT_DELAY        50                          // Default delay [ms]
#define SCRIPT_DELAY_DIV10  ((SCRIPT_DELAY) / 10)       // Default delay [10 ms ticks]

//---------------------------------------------------------------------
//  Shared state
//---------------------------------------------------------------------
enum SYSmode {mode0, mode1, mode2, mode3, mode4};
extern enum SYSmode lpm_mode;

// UART
enum UARTReceiveType { DATA, CONTROL };
extern enum UARTReceiveType receiveType;
extern char receive_str[RX_BUF_CAPACITY + 1];   // last payload, NUL-terminated
extern volatile unsigned int URAT_received_data;

// ADC (LDR)
extern volatile unsigned int ldr1;          // latest A0 (P1.0)
extern volatile unsigned int ldr2;          // latest A3 (P1.3)
extern volatile unsigned char LDR_ready;    // set to 1 when a new pair is available

// Servo
extern volatile unsigned int servo_timer_sec;
extern volatile unsigned int servo_current_angle;

// Sonar
extern volatile unsigned int sonar_echo_capture_width_val;
extern volatile unsigned char sonar_done;
extern volatile unsigned int sonar_time_out;
extern volatile unsigned int sonar_input_counter;
extern volatile unsigned int timer1_a2_capture_start;
extern volatile unsigned int timer1_a2_capture_end;
extern volatile unsigned int sonar_speed_40HZ;
extern volatile unsigned int sonar_speed_17HZ;

//---------------------------------------------------------------------
//  System configuration
//---------------------------------------------------------------------
#define EXIT_LPM(level)              \
    do {                             \
        switch(level) {              \
            case mode0: LPM0_EXIT; break; \
            case mode1: LPM1_EXIT; break; \
            case mode2: LPM2_EXIT; break; \
            case mode3: LPM3_EXIT; break; \
            case mode4: LPM4_EXIT; break; \
        }                            \
    } while (0)

void sysConfig(void);
void enterLPM(unsigned char LPM_level);

void enable_interrupts(void);
void disable_interrupts(void);
void enable_PB_interrupts(void);
void disable_PB_interrupts(void);

//---------------------------------------------------------------------
//  Utilities
//---------------------------------------------------------------------
void DelayUs(unsigned int cnt);
void DelayMs(unsigned int cnt);
void str_copy(char *dst, const char *src);
void str_concat(char *dest, const char *str1, const char *str2);

//---------------------------------------------------------------------
//  Drivers
//---------------------------------------------------------------------
// ADC10 (LDR)
void LDR_init(void);

// Servo (Timer A0)
void setup_timer0_for_servo(void);
void servo_set_angle(unsigned int angle);
void wait_for_servo(void);

// Sonar (Timer A1)
uint16_t get_period(void);
uint16_t go_sonar_trow(int angle, int trow_sample);
void create_data_to_send_sonar(uint8_t STR[3], uint8_t DEG, uint16_t PERIOD);

// UART
void dynamic_transmit(const char* str);
void dynamic_transmit_len(const uint8_t *data, uint8_t len);
void on_string_received(const char* str);

// Scans
void scan_sweep_l2r(int deg_start, int deg_end);
void scan_sweep_light(void);
void scan_sweep_light_sonar(uint8_t deg_start, uint8_t deg_end_exclusive);

//---------------------------------------------------------------------
//  Files
//---------------------------------------------------------------------
enum file_type_enum {
    file_txt,
    file_script
};

typedef struct __attribute__((packed)) {
    char     name[FILES_WORD_SIZE];     // file name (NUL-terminated)
    uint16_t len;                       // total file length in bytes
    enum file_type_enum type;
    uint16_t ptr;                       // flash address where the file starts
    uint16_t end_ptr;                   // flash address after the last byte
} file_desc_t;

extern volatile int file_num;           // number of files in files[]
extern char     file_name[17];
extern char     file_type;
extern volatile int16_t file_read_ptr;
extern char     file_reciving;
extern volatile char file_got_shtroodle;  // set by the RX ISR: 1 = saved, 2 = no space
extern volatile uint8_t file_open;
extern volatile file_desc_t files[MAX_FILES];
extern volatile uint16_t file_ldr_ptr;

void fs_init_clean(void);
void delete_stract_segs(void);
void file_load_all_stract(void);
void file_delete_stract(void);
char* file_read_text(uint8_t file_num, char *dst);
void send_all_files_with_types(void);

// LDR calibration table
void save_table_ldr(void);
int ldr_table(uint16_t ldr_num, uint16_t sample_num);      // -1 on error

//---------------------------------------------------------------------
//  LCD (HD44780)
//---------------------------------------------------------------------
// Define CHECKBUSY to poll the LCD busy flag instead of using a fixed delay
// #define CHECKBUSY    1
#ifdef CHECKBUSY
    #define LCD_WAIT lcd_check_busy()
#else
    #define LCD_WAIT DelayMs(5)
#endif

// EN
#define LCD_EN(a)       (!a ? (LCD_CTL_WRITE&=~LCD_CTL_EN) : (LCD_CTL_WRITE|=LCD_CTL_EN)) // EN = P2.5
#define LCD_EN_DIR(a)   (!a ? (LCD_CTL_DIR&=~LCD_CTL_EN) : (LCD_CTL_DIR|=LCD_CTL_EN)) // EN direction
// RS
#define LCD_RS(a)       (!a ? (LCD_CTL_WRITE_RS&=~LCD_CTL_RS) : (LCD_CTL_WRITE_RS|=LCD_CTL_RS)) // RS = P2.3
#define LCD_RS_DIR(a)   (!a ? (LCD_CTL_DIR_RS&=~LCD_CTL_RS) : (LCD_CTL_DIR_RS|=LCD_CTL_RS)) // RS direction
// RW
#define LCD_RW(a)       (!a ? (LCD_CTL_WRITE&=~LCD_CTL_RW) : (LCD_CTL_WRITE|=LCD_CTL_RW)) // RW = P2.7
#define LCD_RW_DIR(a)   (!a ? (LCD_CTL_DIR&=~LCD_CTL_RW) : (LCD_CTL_DIR|=LCD_CTL_RW)) // RW direction

#define LCD_DATA_OFFSET  ((LCD_DATA_MASK == 0xF0)? (0x04): (0x00)) // 4-bit mode: data on Px.4-7 -> shift 4

#define FOURBIT_MODE    0x0
#define EIGHTBIT_MODE   0x1
#define LCD_MODE        FOURBIT_MODE

#define OUTPUT_PIN      1
#define INPUT_PIN       0
#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))
#define INPUT_DATA      0x00

#define LCD_STROBE_READ(value)  LCD_EN(1), \
                asm("nop"), asm("nop"), \
                value=LCD_DATA_READ, \
                LCD_EN(0)

#define lcd_cursor(x)       lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()         lcd_cmd(0x01)
#define lcd_putchar(x)      lcd_data(x)
#define lcd_goto(x)         lcd_cmd(0x80+(x))
#define lcd_cursor_right()  lcd_cmd(0x14)
#define lcd_cursor_left()   lcd_cmd(0x10)
#define lcd_display_shift() lcd_cmd(0x1C)
#define lcd_home()          lcd_cmd(0x02)
#define cursor_off()        lcd_cmd(0x0C)
#define cursor_on()         lcd_cmd(0x0F)
#define lcd_function_set()  lcd_cmd(0x3C) // 8-bit, 2 lines, 5x10 dots
#define lcd_new_line()      lcd_cmd(0xC0)

extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char);
extern void lcd_puts(const char * s);
extern void lcd_init();
extern void lcd_strobe();

#endif // HALGPIO_H
