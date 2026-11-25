# MSP430 Radar System

A light-and-object detection system built on the **TI MSP430G2553**, controlled from a PC GUI over UART.
An ultrasonic sensor and two LDRs sit on a servo, which sweeps 0°–180°. The MCU streams the measurements to the PC,
and the PC shows them on a radar view.
The MCU also stores scripts in its flash memory and runs them.

## Repository layout

```
.
├── mcu/                    # MCU side: C firmware for the MSP430G2553
│   ├── source/             # main.c, api.c, halGPIO.c, bsp.c
│   └── header/             # app.h, api.h, halGPIO.h, bsp.h
└── pc/                     # PC side: Python GUI (PyQt5)
    ├── main.py             # Entry point
    ├── main_window.py      # Main window: widgets, radar plot, event handlers
    ├── serial_terminal.py  # UART link: framing, checksum, ACK, RX thread
    └── requirements.txt
```

## Features

| Mode | Description |
|---|---|
| **Object scan** | The servo sweeps 0°–180°, and the ultrasonic sensor measures distance at each angle |
| **Telemeter** | The servo moves to a fixed angle, and the MCU streams distance readings continuously |
| **Light scan** | The servo sweeps while two LDRs sample light intensity. The readings are interpreted using an LDR calibration table stored in flash |
| **Full scan** | Combines the object and light scans in one sweep |
| **File system** | Stores script and text files in flash. You can save, read, delete and run them |
| **Script engine** | Runs a stored script: a sequence of opcodes that control the LCD, the delay and the servo |

## Architecture

### MCU side

The firmware uses a layered, interrupt-driven FSM design. Each layer talks only to the layer below it:

| Layer | Files | Responsibility |
|---|---|---|
| Application | `main.c`, `app.h` | Main loop: dispatch on the current FSM state |
| API | `api.c`, `api.h` | Logic of each state: scans, telemeter, files, script opcodes |
| HAL | `halGPIO.c`, `halGPIO.h` | Drivers (timers, PWM, ADC10, UART, LCD, flash) and ISRs |
| BSP | `bsp.c`, `bsp.h` | Pin mapping, clock and peripheral configuration |

The CPU sleeps in LPM0 between events, and interrupts from the timers, the ADC, the UART and the push buttons wake it up.

### PC side

```
main.py ──▶ main_window.py ──▶ serial_terminal.py
```

`serial_terminal.py` owns the serial port and runs a background RX thread. It reports incoming data to the GUI through
Qt signals (`data_received`, `ack_done`), so the GUI thread never blocks on reading the port.
Sending a command does block until the MCU answers with ACK/NACK (timeout 2 s per attempt).

## UART protocol

**9600 baud, 8N1**, no flow control.

**Frame format** (both directions):

```
[HEADER][LEN][DATA ...][CHK]      CHK = (LEN + sum(DATA)) & 0xFF
```

| Header | Meaning |
|---|---|
| `0xAA` | Text payload (ASCII) |
| `0xFF` | Binary payload (packed measurements) |

**Single-byte tokens** (not framed):

| Token | Direction | Meaning |
|---|---|---|
| `0xAB` | PC → MCU | Host is ready. The MCU waits for this byte before it starts sending |
| `0xCC` | Both | ACK: frame received OK |
| `0xBB` | Both | NACK: bad frame. The PC retries once, and the timeout is 2 s |

### Commands

The PC sends each command as a text frame:

| Command | Action |
|---|---|
| `A` | Object scan |
| `B` | Telemeter. Then send an angle (e.g. `90`). `q` exits |
| `C` | Light scan |
| `D` | Full scan |
| `L` / `N` | Build the LDR calibration table / show it |
| `e` / `r` | Save a file to flash / read a file |
| `g` / `h` | Delete / load the file table |
| `i` | Run a script |
| `a` `b` `c` `d` | Debug modes: servo, LDR, sonar, simple scan |

### Script opcodes

| Opcode | Instruction |
|---|---|
| `0x01` | `inc_lcd`: count up on the LCD |
| `0x02` | `dec_lcd`: count down on the LCD |
| `0x03` | `rra_lcd`: move a character across the LCD |
| `0x04` | `set_delay` |
| `0x05` | `clear_lcd` |
| `0x06` | `servo_deg`: point the sonar at an angle and measure the distance |
| `0x07` | `servo_scan`: sonar sweep between two angles |
| `0x08` | `sleep` |

A script file is plain text with one instruction per line, written in hex as `OOAABB`:
`OO` is the opcode, and `AA` / `BB` are the parameters (optional).
For example, `0105` counts up to 5 on the LCD, and `07143C` sweeps from 20° to 60°.

## Hardware

| Peripheral | Pins |
|---|---|
| Servo (PWM, 40 Hz) | P2.6 |
| Ultrasonic TRIG / ECHO | P2.2 / P2.4 (TA1.2 capture) |
| LDR 1 / LDR 2 (ADC10) | P1.0 / P1.3 |
| LCD data (4-bit) | P1.4–P1.7 |
| LCD control (RS / EN / RW) | P2.3 / P2.5 / P2.7 |
| Push buttons PB0 / PB1 | P2.0 / P2.1 |
| UART RXD / TXD | P1.1 / P1.2 |

## Getting started

### MCU
1. Create an empty **MSP430G2553** project in Code Composer Studio.
2. Copy `mcu/source` and `mcu/header` into the project.
3. Add the `mcu/` folder to the include path, because the sources use `#include <header/...>`.
4. Build the project and flash it to the LaunchPad.

> The file system uses main flash `0xF400`–`0xFDFF`. Make sure the code ends below `0xF400`
> (check the `.map` file); if needed, limit `FLASH` in `lnk_msp430g2553.cmd` to `length = 0x3400`.

### PC
```bash
cd pc
pip install -r requirements.txt
python main.py
```
In the **Console** panel, choose the COM port and click **Connect**.
