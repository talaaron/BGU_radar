/*
 * api.c - API layer: the logic of every FSM state
 *
 * main() calls exactly one function from this file per state. Each function
 * talks to the hardware only through the HAL (halGPIO.h).
 *
 * Chapters
 *   1. Globals and private helpers
 *   2. String helpers           int2str, str2int, shirshur
 *   3. Debug modes              a b c d
 *   4. Radar modes              A B C D L N
 *   5. File modes               e r g h i
 *   6. Script engine            timer hook, opcodes 0x01-0x08, parser
 *
 * Common pattern of a state function:
 *   announce_state("SCRIPT_X");   // tell the PC once that we entered the state
 *   ... do the work ...
 *   send_end_marker();            // framed '@' = "data for this command is done"
 *   if (state == <this state>) state = state0;   // unless the PC already changed it
 */

#include <header/api.h>

//=====================================================================
//  1. Globals and private helpers
//=====================================================================

// Script parameters, filled by parse_script_line()
volatile int param1 = 100;
volatile int param2 = 180;
volatile int lcd_val = 0;

// Script timing (see tick10ms_hook)
volatile int script_timer_tick       = 0;
volatile int script_lcd_delay        = SCRIPT_DELAY;         // [ms]
volatile int script_lcd_delay_div_10 = SCRIPT_DELAY_DIV10;   // [10 ms ticks]
volatile int script_timer_hook_count = 0;

// Number of files in the file table (kept by the HAL)
#define FILES_COUNT ((uint8_t)file_num)

static void announce_state(const char *tag);
static void send_end_marker(void);
static void servo_move(int angle);
static void send_file_entry(uint8_t idx);
static void send_file_list(enum file_type_enum type);
static uint8_t file_script_get_command(char cmd[16], uint8_t file_idx, uint8_t row);
static int  parse_script_line(const char *str);
static void run_script_opcode(int opcode);

// Send "SCRIPT_X" to the PC once, on the first call after a state change.
static void announce_state(const char *tag)
{
    if (last_state != state)
        dynamic_transmit(tag);
    last_state = state;
}

// Framed '@' (0x40): tells the PC that the data of the current command is complete.
static void send_end_marker(void)
{
    const uint8_t endb = '@';
    dynamic_transmit_len(&endb, 1);
}

//=====================================================================
//  2. String helpers
//=====================================================================

// Unsigned int -> decimal string.
void int2str(char *str, unsigned int num)
{
    if (num == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }

    unsigned int tmp = num;
    int len = 0, j;

    while (tmp) {
        len++;
        tmp /= 10;
    }

    for (j = len - 1; j >= 0; j--) {
        str[j] = (num % 10) + '0';
        num /= 10;
    }
    str[len] = '\0';
}

// Decimal string -> int. Returns 1 on success, 0 if the string is empty or
// has a non-digit character (then *num is not changed).
unsigned int str2int(char *str, int *num)
{

    int result = 0;
    int k = 0;

    if (str[0] == '\0') return 0;

    while (str[k] != '\0') {
        if (str[k] < '0' || str[k] > '9') return 0;
        result = result * 10 + (str[k] - '0');
        k++;
    }

    *num = result;
    return 1;
}

// dst = st1 + decimal(num) + st2   (dst must be large enough)
void shirshur(const char *st1, int num, const char *st2, char *dst)
{
    char *p = dst;
    char buf[6];                    // max 5 digits for a 16-bit int
    int  n = 0;

    while (*st1) *p++ = *st1++;

    if (num < 0) { *p++ = '-'; num = -num; }
    if (num == 0) {
        buf[n++] = '0';
    } else {
        while (num > 0 && n < (int)sizeof(buf)) {
            buf[n++] = (char)('0' + (num % 10));
            num /= 10;
        }
    }
    while (n--) *p++ = buf[n];

    while (*st2) *p++ = *st2++;
    *p = '\0';
}

//=====================================================================
//  3. Debug modes
//=====================================================================

// 'a' - The PC sends angles; the servo moves to each one.
//       Any non-number ends the mode.
void debug_servo(void)
{
    int  num = 0;
    char servo_msg[32] = {0};

    dynamic_transmit("start debug_servo while \r\n");
    receiveType = DATA;
    dynamic_transmit("Enter angle:\r\n");

    while (state == state_d_servo) {
        if (URAT_received_data) {
            URAT_received_data = 0;

            if (str2int(receive_str, &num)) {
                servo_set_angle(num);
                shirshur("", num, " angle \r\n", servo_msg);
                dynamic_transmit(servo_msg);
                dynamic_transmit("Enter angle: \r\n");
            } else {
                dynamic_transmit("done\r\n");
                receiveType = CONTROL;
                state = state0;
                dynamic_transmit("shut pwm and sleep \r\n");
                enterLPM(lpm_mode);
            }
        }
    }
    receiveType = CONTROL;
}

// 'b' - Shows LDR1 / LDR2 on the LCD whenever a reading changes.
//       The ADC is sampled from the Timer0 ISR at 40 Hz.
void debug_LDR(void)
{
    static int v1_prev = -1, v2_prev = -1;
    char lcd_line[17] = {0};

    dynamic_transmit("Waiting for LDR data...\r\n");

    while (state == state_d_LDR) {
        if (!LDR_ready) { enterLPM(lpm_mode); continue; }

        // Take a consistent snapshot and consume the flag
        int v1, v2;
        __disable_interrupt();
        v1 = (int)ldr1;
        v2 = (int)ldr2;
        LDR_ready = 0;
        __enable_interrupt();

        if (v1 != v1_prev || v2 != v2_prev) {
            v1_prev = v1;
            v2_prev = v2;

            shirshur("LDR1:", v1, "", lcd_line);    // line 1: LDR1 (A0)
            lcd_clear();
            lcd_home();
            lcd_puts(lcd_line);

            shirshur("LDR2:", v2, "", lcd_line);    // line 2: LDR2 (A3)
            lcd_new_line();
            lcd_puts(lcd_line);
        }

        enterLPM(lpm_mode);                         // sleep until the next sample
    }
}

// 'c' - Waits 10 s (time to place a target), then takes one sonar reading.
void debug_sonar(void)
{
    char sonar_msg[32] = {0};
    int  distance;

    sonar_speed_40HZ = 0;
    sonar_speed_17HZ = 0;
    disable_interrupts();
    DelayMs(10000);

    get_period();
    // distance [cm] = echo [us] * 0.0343 / 2  ~= echo * 17 / 1000 (rounded)
    distance = (int)(((unsigned long)sonar_echo_capture_width_val * 17ul + 500ul) / 1000ul);

    shirshur("Sonar distance:", distance, "\r\n", sonar_msg);
    dynamic_transmit(sonar_msg);
    shirshur("Sonar 40HZ ticks:", sonar_speed_40HZ, "\r\n", sonar_msg);
    dynamic_transmit(sonar_msg);
    shirshur("Sonar 17HZ ticks:", sonar_speed_17HZ, "\r\n", sonar_msg);
    dynamic_transmit(sonar_msg);

    enable_interrupts();
    state = state0;
}

// 'd' - Servo 0 -> 90 -> 180 -> 90, changing once per second.
void debug_scan(void)
{
    int last_step = -1;
    int step      = -1;

    lcd_clear();
    lcd_home();
    lcd_puts("Debug easy scan");
    __enable_interrupt();

    while (state == state_d_scan) {
        while (last_step == step) {
            step = servo_timer_sec % 4;
        }
        last_step = step;

        if (step == 0)                    servo_move(0);
        else if (step == 1 || step == 3)  servo_move(90);
        else                              servo_move(180);
    }
}

static void servo_move(int angle)
{
    char servo_msg[16] = {0};

    servo_set_angle(angle);
    shirshur("", angle, " angle \r\n", servo_msg);
    dynamic_transmit(servo_msg);
}

//=====================================================================
//  4. Radar modes
//=====================================================================

// 'A' - Object scan: sonar sweep 0..180. One 3-byte frame per degree.
void prod_obj_scan(void)
{
    announce_state("SCRIPT_A");
    scan_sweep_l2r(0, 181);
    send_end_marker();
    if (state == state_prod_obj_scan) state = state0;
}

// 'B' - Telemeter: sonar at a fixed angle, streaming until the PC sends
//       a new angle (keep going there) or a non-number such as "q" (exit).
void prod_telemeter(void)
{
    int      num = servo_current_angle;
    int      first_run = 0;
    uint8_t  out[3];
    uint16_t period;

    announce_state("SCRIPT_B");
    receiveType = DATA;
    wait_for_servo();

    while (state == state_prod_telemeter) {
        first_run++;
        URAT_received_data = 0;

        if (first_run == 1 || str2int(receive_str, &num)) {
            while (URAT_received_data == 0) {           // stream until the PC sends something
                period = go_sonar_trow(num, 5);
                create_data_to_send_sonar(out, (uint8_t)num, period);
                dynamic_transmit_len(out, 3);
            }
            URAT_received_data = 0;
        } else {
            send_end_marker();
            receiveType = CONTROL;
            state = state0;
        }
    }
    receiveType = CONTROL;
}

// 'C' - Light scan: LDR sweep 0..180 (the range is fixed in the HAL).
void prod_ligth_scan(void)
{
    announce_state("SCRIPT_C");
    scan_sweep_light();
    send_end_marker();
    if (state == state_prod_ligth_scan) state = state0;
}

// 'D' - Full scan: sonar + LDR in one sweep 0..180.
void prod_full_scan(void)
{
    announce_state("SCRIPT_D");
    scan_sweep_light_sonar(0, 181);
    send_end_marker();
    if (state == state_prod_full_scan) state = state0;
}

// 'L' - Build the LDR calibration table and store it in info segment D.
void prod_ldr_table(void)
{
    announce_state("SCRIPT_L");
    file_ldr_ptr = SEG_D_START;
    save_table_ldr();
    lcd_clear();
    send_end_marker();
    if (state == state_prod_ldr_table) state = state0;
}

// 'N' - Print the LDR calibration table (10 samples per LDR).
void prod_show_ldr_table(void)
{
    char msg[24]  = {0};
    char msg1[12] = {0};
    int  ldr, n;

    announce_state("SCRIPT_N");

    for (ldr = 1; ldr <= 2; ldr++) {
        for (n = 0; n < 10; n++) {
            shirshur(ldr == 1 ? "LDR1[" : "LDR2[", n, "]=", msg1);
            shirshur(msg1, ldr_table(ldr, n), "", msg);
            dynamic_transmit(msg);
        }
    }

    file_ldr_ptr = SEG_D_START;
    send_end_marker();
    if (state == state_prod_show_ldr_table) state = state0;
}

//=====================================================================
//  5. File modes
//=====================================================================

// "File-<idx> <name> \n"
static void send_file_entry(uint8_t idx)
{
    char    line[32];
    char   *p;
    uint8_t k;

    shirshur("File-", idx, " ", line);
    p = line;
    while (*p) p++;
    for (k = 0; k < FILES_WORD_SIZE && files[idx].name[k] != '\0'; k++)
        *p++ = files[idx].name[k];
    *p++ = ' ';
    *p++ = '\n';
    *p   = '\0';
    dynamic_transmit(line);
}

// List every file of one type, with its index.
static void send_file_list(enum file_type_enum type)
{
    uint8_t idx;

    dynamic_transmit(type == file_txt ? "txt:\n" : "Script:\n");
    for (idx = 0; idx < FILES_COUNT; idx++) {
        if (files[idx].type == type)
            send_file_entry(idx);
    }
}

// 'h' - Load the file table from flash (also called once at boot).
void prod_load_stract(void)
{
    dynamic_transmit("loading \n");
    file_load_all_stract();
    dynamic_transmit("\nloading done \n");
    state = state0;
}

// 'e' - Save a file: the PC sends the name, the type ("txt" / "script"),
//       then the content in chunks, ending with '#'. The RX ISR writes the
//       chunks to flash and sets file_got_shtroodle when the file is closed.
void prod_save_file(void)
{
    file_reciving = 0;
    receiveType = DATA;

    // File name
    dynamic_transmit("file name: ");
    URAT_received_data = 0;
    while (URAT_received_data == 0) { enterLPM(mode0); }
    str_copy(file_name, receive_str);
    file_name[16] = '\0';

    // File type
    dynamic_transmit("\nfile type (script or txt): ");
    URAT_received_data = 0;
    while (URAT_received_data == 0) { enterLPM(mode0); }
    file_type = (receive_str[0] == 't' &&
                 receive_str[1] == 'x' &&
                 receive_str[2] == 't') ? file_txt : file_script;

    // File content
    if (file_num < MAX_FILES) {
        file_reciving = 1;
        dynamic_transmit("\nsend content, end with '#'\r\n");
        file_got_shtroodle = 0;
        while (!file_got_shtroodle) { enterLPM(mode0); }   // 1 = saved, 2 = no space
        while (file_open) { }

        if (file_got_shtroodle == 2)
            dynamic_transmit("no space");
        else
            dynamic_transmit("saved");
    } else {
        dynamic_transmit("\nstorage is full ");
    }

    file_reciving = 0;
    state = state0;
    receiveType = CONTROL;
}

// 'r' - List the files, then send the content of the one the PC picks.
void prod_read_file(void)
{
    char line[17];
    char hdr[32];
    int  idx;

    dynamic_transmit("choose file to read (0-8):\n ");
    receiveType = DATA;
    URAT_received_data = 0;

    send_file_list(file_txt);
    send_file_list(file_script);

    while (URAT_received_data == 0) { enterLPM(mode0); }
    URAT_received_data = 0;

    if (str2int(receive_str, &idx) && idx >= 0 && idx < FILES_COUNT) {   // idx < 0: str2int overflow
        if (files[idx].len > 0) {
            shirshur("\n file ", idx, " content:\n", hdr);
            dynamic_transmit(hdr);

            file_read_ptr = -1;                 // start from the first byte
            while (file_read_text((uint8_t)idx, line)) {
                dynamic_transmit(line);
            }
        } else {
            shirshur("\n file ", idx, " is empty\n", hdr);
            dynamic_transmit(hdr);
        }
    } else {
        dynamic_transmit("typo or file not exist ");
    }

    dynamic_transmit("\n file done\r\n");
    receiveType = CONTROL;
    state = state0;
}

// 'g' - Delete the whole file table.
void prod_delete_stract(void)
{
    file_delete_stract();
    dynamic_transmit("All stract data - DELETED \n");
    state = state0;
}

// 'i' - List the scripts, then run the one the PC picks, line by line.
void prod_run_script(void)
{
    char    cmd[16];
    char    hdr[32];
    int     idx;
    uint8_t row;

    dynamic_transmit("choose script to run (0-8):\n ");
    receiveType = DATA;
    URAT_received_data = 0;

    send_file_list(file_script);

    while (URAT_received_data == 0) { enterLPM(mode0); }
    URAT_received_data = 0;

    if (str2int(receive_str, &idx) && idx >= 0 && idx < FILES_COUNT) {   // idx < 0: str2int overflow
        if (files[idx].len > 0) {
            dynamic_transmit("\nrunning script\n");
            for (row = 0; file_script_get_command(cmd, (uint8_t)idx, row); row++) {
                run_script_opcode(parse_script_line(cmd));
            }
        } else {
            shirshur("\n file ", idx, " is empty\n", hdr);
            dynamic_transmit(hdr);
        }
    } else {
        dynamic_transmit("typo or file not exist ");
    }

    dynamic_transmit("\n file done\r\n");
    receiveType = CONTROL;
    state = state0;
}

//=====================================================================
//  6. Script engine
//=====================================================================
//
//  A script file is plain text, one instruction per line, in hex:
//      OOAABB      OO = opcode, AA = param1, BB = param2 (optional)
//  Example:  "0105" -> inc_lcd 5      "07143C" -> servo_scan 20..60
//
//  Timing: the Timer0 ISR calls tick10ms_hook() every 10 ms. When the
//  script delay has passed, it sets script_timer_tick and wakes the CPU.
//---------------------------------------------------------------------

// Returns 1 (and wakes the CPU) once every script delay period.
int tick10ms_hook(void)
{
    script_timer_hook_count++;
    if (script_lcd_delay_div_10 <= script_timer_hook_count) {
        script_timer_hook_count = 0;
        script_timer_tick = 1;
        return 1;
    }
    return 0;
}

// state0 - Idle: sleep until the PC sends a command.
void state0_func(void)
{
    announce_state("SCRIPT_0");
    URAT_received_data = 0;
    enable_interrupts();
    enterLPM(lpm_mode);
}

// 0x01 inc_lcd x - Count 0..x on the LCD, one step per delay.
void state1_func(void)
{
    char lcd_out_st[17] = {0};

    announce_state("SCRIPT_1");
    lcd_val = -1;

    while (state == state1 && lcd_val < param1) {
        if (script_timer_tick == 0) {
            enterLPM(mode0);
        } else {
            script_timer_tick = 0;
            lcd_val++;
            int2str(lcd_out_st, lcd_val);
            lcd_clear();
            lcd_home();
            lcd_puts(lcd_out_st);
        }
    }
    if (state == state1) state = state0;
}

// 0x02 dec_lcd x - Count x..0 on the LCD, one step per delay.
void state2_func(void)
{
    char lcd_out_st[17] = {0};

    announce_state("SCRIPT_2");
    lcd_val = param1 + 1;

    while (state == state2 && 0 < lcd_val) {
        if (script_timer_tick == 0) {
            enterLPM(mode0);
        } else {
            script_timer_tick = 0;
            lcd_val--;
            int2str(lcd_out_st, lcd_val);
            lcd_clear();
            lcd_home();
            lcd_puts(lcd_out_st);
        }
    }
    if (state == state2) state = state0;
}

// 0x03 rra_lcd x - Move the character x across all 32 LCD cells, one cell per delay.
void state3_func(void)
{
    char c = (char)param1;
    int  pos = 0, j;

    announce_state("SCRIPT_3");

    while (state == state3 && pos < 32) {
        if (script_timer_tick == 0) {
            enterLPM(mode0);
        } else {
            script_timer_tick = 0;
            lcd_clear();
            if (pos <= 15) {
                for (j = 0; j < pos; j++) lcd_cursor_right();
            } else {
                lcd_new_line();
                for (j = 0; j < (pos - 16); j++) lcd_cursor_right();
            }
            lcd_putchar(c);
            pos++;
        }
    }
    if (state == state3) state = state0;
}

// 0x04 set_delay d - Set the script delay.
void state4_func(void)
{
    announce_state("SCRIPT_4");
    script_lcd_delay        = param1;                   // [ms]
    script_timer_hook_count = 0;
    script_lcd_delay_div_10 = script_lcd_delay / 10;    // [10 ms ticks]
    if (state == state4) state = state0;
}

// 0x05 clear_lcd
void state5_func(void)
{
    announce_state("SCRIPT_5");
    lcd_clear();
    if (state == state5) state = state0;
}

// 0x06 servo_deg p - Point the sonar at angle p and send one measurement.
void state6_func(void)
{
    announce_state("SCRIPT_6");
    scan_sweep_l2r(param1, param1 + 1);
    send_end_marker();
    if (state == state6) state = state0;
}

// 0x07 servo_scan l r - Sonar sweep from angle l to angle r.
void state7_func(void)
{
    announce_state("SCRIPT_7");
    scan_sweep_l2r(param1, param2);
    send_end_marker();
    if (state == state7) state = state0;
}

// 0x08 sleep - Go idle.
void state8_func(void)
{
    announce_state("SCRIPT_8");
    state0_func();
}

// Read line `row` of script file `file_idx` into cmd.
// Returns 1 if the line exists, 0 otherwise.
static uint8_t file_script_get_command(char cmd[16], uint8_t file_idx, uint8_t row)
{
    char    line[17];
    uint8_t current_row = 0;
    uint8_t k;

    if (file_idx >= FILES_COUNT)    return 0;
    if (files[file_idx].len == 0)   return 0;

    file_read_ptr = -1;                         // read from the start of the file
    while (file_read_text(file_idx, line)) {
        if (current_row == row) {
            for (k = 0; k < 15 && line[k] != '\0'; k++)
                cmd[k] = line[k];
            cmd[k] = '\0';
            return 1;
        }
        current_row++;
    }
    return 0;
}

// One hex character -> 0..15, or -1 if invalid.
static int hex_char_to_val(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    return -1;
}

// Two hex characters -> 0..255, or -1 if invalid.
static int hex_pair_to_int(const char *s)
{
    int hi = hex_char_to_val(s[0]);
    int lo = hex_char_to_val(s[1]);
    if (hi < 0 || lo < 0) return -1;
    return (hi << 4) | lo;
}

// Parse "OOAABB": fills param1 / param2 and returns the opcode (-1 if invalid).
static int parse_script_line(const char *str)
{
    int len = 0;
    int opcode = -1;

    while (str[len] != '\0') len++;

    param1 = param2 = 0;
    if (len >= 2) opcode = hex_pair_to_int(&str[0]);
    if (len >= 4) param1 = hex_pair_to_int(&str[2]);
    if (len >= 6) param2 = hex_pair_to_int(&str[4]);
    return opcode;
}

// Run one script instruction. Unknown opcodes are ignored.
static void run_script_opcode(int opcode)
{
    switch (opcode) {
        case 1: state = state1; state1_func(); break;   // inc_lcd
        case 2: state = state2; state2_func(); break;   // dec_lcd
        case 3: state = state3; state3_func(); break;   // rra_lcd
        case 4: state = state4; state4_func(); break;   // set_delay
        case 5: state = state5; state5_func(); break;   // clear_lcd
        case 6: state = state6; state6_func(); break;   // servo_deg
        case 7: state = state7; state7_func(); break;   // servo_scan
        case 8: state = state8; state8_func(); break;   // sleep
        default: break;
    }
}
