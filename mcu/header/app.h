#ifndef APP_H
#define APP_H

/*
 * app.h - Application layer: the FSM states
 *
 * The UART RX ISR (on_string_received in halGPIO.c) maps each command
 * character from the PC to one of these states. main() dispatches on the
 * current state and calls the matching function from api.c.
 */

enum FSMstate {
    // Script ISA (opcodes 0x01-0x08; state0 = idle/sleep)
    state0,                     // '0'  idle: sleep in LPM0
    state1,                     // '1'  inc_lcd
    state2,                     // '2'  dec_lcd
    state3,                     // '3'  rra_lcd
    state4,                     // '4'  set_delay
    state5,                     // '5'  clear_lcd
    state6,                     // '6'  servo_deg
    state7,                     // '7'  servo_scan
    state8,                     // '8'  sleep

    // Debug modes
    state_d_servo,              // 'a'
    state_d_LDR,                // 'b'
    state_d_sonar,              // 'c'
    state_d_scan,               // 'd'

    // Radar modes
    state_prod_obj_scan,        // 'A'  object scan (sonar)
    state_prod_telemeter,       // 'B'  telemeter
    state_prod_ligth_scan,      // 'C'  light scan (LDR)
    state_prod_full_scan,       // 'D'  sonar + light
    state_prod_ldr_table,       // 'L'  build LDR calibration table
    state_prod_show_ldr_table,  // 'N'  print LDR calibration table

    // File modes
    state_prod_save_file,       // 'e'
    state_prod_read_file,       // 'r'
    state_prod_delete_stract,   // 'g'  delete file table
    state_prod_load_stract,     // 'h'  load file table
    state_prod_run_script       // 'i'
};

// volatile: the UART RX ISR changes `state` while the main loop polls it
extern volatile enum FSMstate state;    // current state
extern enum FSMstate last_state;        // previous state (to announce changes once)

#endif // APP_H
