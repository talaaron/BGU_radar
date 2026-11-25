/*
 * main.c - Entry point and FSM dispatcher
 *
 *   1. Init: clocks, GPIO, UART, LCD, file table, servo timer, ADC
 *   2. Main loop: call the function of the current state (api.c)
 *
 * The UART RX ISR sets `state` when a command arrives from the PC. Every
 * state function returns when it is done (or when `state` changes), and
 * state0 puts the CPU to sleep in LPM0 until the next interrupt.
 */

#include <header/api.h>

volatile enum FSMstate state      = state0;
enum FSMstate          last_state = state0;

int main(void)
{
    //-----------------------------------------------------------------
    //  1. Init
    //-----------------------------------------------------------------
    __disable_interrupt();
    receiveType = CONTROL;
    lpm_mode    = mode0;

    sysConfig();                // clock, GPIO, UART (bsp.c)
    lcd_init();
    cursor_off();
    lcd_clear();

    prod_load_stract();         // load the file table. Blocks until the PC sends 0xAB
    __enable_interrupt();
    setup_timer0_for_servo();   // Timer0: servo PWM + 40 Hz tick (ADC, script timing)
    LDR_init();                 // ADC10 sequence, triggered from the Timer0 ISR

    //-----------------------------------------------------------------
    //  2. Main loop - FSM dispatcher
    //-----------------------------------------------------------------
    while (1) {
        switch (state) {

            // Debug modes
            case state_d_servo:             debug_servo();          break;  // a
            case state_d_LDR:               debug_LDR();            break;  // b
            case state_d_sonar:             debug_sonar();          break;  // c
            case state_d_scan:              debug_scan();           break;  // d

            // Radar modes
            case state_prod_obj_scan:       prod_obj_scan();        break;  // A
            case state_prod_telemeter:      prod_telemeter();       break;  // B
            case state_prod_ligth_scan:     prod_ligth_scan();      break;  // C
            case state_prod_full_scan:      prod_full_scan();       break;  // D
            case state_prod_ldr_table:      prod_ldr_table();       break;  // L
            case state_prod_show_ldr_table: prod_show_ldr_table();  break;  // N

            // File modes
            case state_prod_save_file:      prod_save_file();       break;  // e
            case state_prod_read_file:      prod_read_file();       break;  // r
            case state_prod_delete_stract:  prod_delete_stract();   break;  // g
            case state_prod_load_stract:    prod_load_stract();     break;  // h
            case state_prod_run_script:     prod_run_script();      break;  // i

            // Script ISA (can also be sent directly as '0'..'8')
            case state0:                    state0_func();          break;  // idle
            case state1:                    state1_func();          break;  // 0x01 inc_lcd
            case state2:                    state2_func();          break;  // 0x02 dec_lcd
            case state3:                    state3_func();          break;  // 0x03 rra_lcd
            case state4:                    state4_func();          break;  // 0x04 set_delay
            case state5:                    state5_func();          break;  // 0x05 clear_lcd
            case state6:                    state6_func();          break;  // 0x06 servo_deg
            case state7:                    state7_func();          break;  // 0x07 servo_scan
            case state8:                    state8_func();          break;  // 0x08 sleep
        }
    }
}
