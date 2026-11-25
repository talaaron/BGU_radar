#ifndef API_H
#define API_H

/*
 * api.h - API layer: the logic of every FSM state
 *
 * Chapters (same order as api.c):
 *   1. String helpers
 *   2. Debug modes        a b c d
 *   3. Radar modes        A B C D L N
 *   4. File modes         e r g h i
 *   5. Script engine      timer hook + opcodes 0x01-0x08
 */

#include <header/app.h>
#include <header/halGPIO.h>

//---------------------------------------------------------------------
//  1. String helpers
//---------------------------------------------------------------------
void         int2str(char *str, unsigned int num);
unsigned int str2int(char *str, int *num);
void         shirshur(const char *st1, int num, const char *st2, char *dst);   // dst = st1 + num + st2

//---------------------------------------------------------------------
//  2. Debug modes
//---------------------------------------------------------------------
void debug_servo(void);             // 'a'  move the servo to angles typed on the PC
void debug_LDR(void);               // 'b'  show both LDR readings on the LCD
void debug_sonar(void);             // 'c'  one sonar measurement
void debug_scan(void);              // 'd'  servo 0 -> 90 -> 180 -> 90, one step per second

//---------------------------------------------------------------------
//  3. Radar modes
//---------------------------------------------------------------------
void prod_obj_scan(void);           // 'A'  sonar sweep 0-180
void prod_telemeter(void);          // 'B'  sonar at a fixed angle, continuous
void prod_ligth_scan(void);         // 'C'  LDR sweep 0-180
void prod_full_scan(void);          // 'D'  sonar + LDR sweep 0-180
void prod_ldr_table(void);          // 'L'  build the LDR calibration table
void prod_show_ldr_table(void);     // 'N'  print the LDR calibration table

//---------------------------------------------------------------------
//  4. File modes
//---------------------------------------------------------------------
void prod_save_file(void);          // 'e'
void prod_read_file(void);          // 'r'
void prod_delete_stract(void);      // 'g'
void prod_load_stract(void);        // 'h'  (also called once at boot)
void prod_run_script(void);         // 'i'

//---------------------------------------------------------------------
//  5. Script engine
//---------------------------------------------------------------------
extern volatile int script_timer_tick;          // set every script delay period
extern volatile int script_lcd_delay;           // current delay [ms]
extern volatile int script_timer_hook_count;

int tick10ms_hook(void);            // called from the Timer0 ISR every 10 ms

void state0_func(void);             // idle / sleep
void state1_func(void);             // 0x01 inc_lcd    x
void state2_func(void);             // 0x02 dec_lcd    x
void state3_func(void);             // 0x03 rra_lcd    x
void state4_func(void);             // 0x04 set_delay  d
void state5_func(void);             // 0x05 clear_lcd
void state6_func(void);             // 0x06 servo_deg  p
void state7_func(void);             // 0x07 servo_scan l r
void state8_func(void);             // 0x08 sleep

#endif // API_H
