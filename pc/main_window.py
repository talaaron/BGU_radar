"""
Main window of the PC application.

Panels: Telemeter (angle slider, live distance), Sonar (scan type and polar
plot), File System (send files to the MCU) and Console (port selection,
raw text commands and the log).
"""

import os
import threading
from enum import Enum
from typing import Optional

import matplotlib
import numpy as np
from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFrame, QComboBox, \
    QSlider, QFileDialog

from matplotlib import figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

from serial_terminal import SerialTerminal
import serial.tools.list_ports


# GUI state; the value is the command sent to the MCU to enter it
class State(Enum):
    IDLE = "0"
    TELEMETER = "B"


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.prev_state = None
        self.state = State.IDLE
        self.ack_flag = False
        self.serial: Optional[SerialTerminal] = None
        self.telemeter_distance_label = None
        self.mode_combo = None
        self.plot_placeholder = None
        self.setWindowTitle("GUI")
        self.resize(1000, 700)

        central_widget = QWidget()
        main_layout = QVBoxLayout(central_widget)

        # Top row: Telemeter | Sonar | File System
        top_panel_layout = QHBoxLayout()
        top_panel_layout.setContentsMargins(0, 0, 0, 0)
        top_panel_layout.setSpacing(0)

        self.telemeter_widget = self.create_telemeter_widget()
        top_panel_layout.addWidget(self.telemeter_widget, stretch=1)

        self.sonar_widget = self.create_sonar_widget()
        top_panel_layout.addWidget(self.sonar_widget, stretch=1)

        self.file_system_widget = self.create_file_system_widget()
        top_panel_layout.addWidget(self.file_system_widget, stretch=1)

        main_layout.addLayout(top_panel_layout, stretch=2)

        # Bottom: console
        self.console_widget = self.create_console_widget()
        main_layout.addWidget(self.console_widget, stretch=1)

        self.setCentralWidget(central_widget)

        self.refresh_ports()

    # =========== Widgets ===========

    def create_telemeter_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setAlignment(Qt.AlignHCenter | Qt.AlignTop)

        title = QLabel("Telemeter")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Angle slider (0..180)
        slider_row = QHBoxLayout()
        slider_row.addStretch(1)

        self.telemeter_angle_slider = QSlider(Qt.Horizontal)
        self.telemeter_angle_slider.setRange(0, 180)
        self.telemeter_angle_slider.setSingleStep(1)
        self.telemeter_angle_slider.setPageStep(10)
        self.telemeter_angle_slider.setValue(0)
        self.telemeter_angle_slider.setMinimumWidth(300)

        self.telemeter_angle_value = QLabel("0°")
        self.telemeter_angle_value.setMinimumWidth(40)  # room for "180°"
        self.telemeter_angle_value.setAlignment(Qt.AlignCenter)

        slider_row.addWidget(self.telemeter_angle_slider)
        slider_row.addWidget(self.telemeter_angle_value)

        slider_row.addStretch(1)
        layout.addLayout(slider_row)

        # Start/Stop toggle and Update Angle
        btn_row = QHBoxLayout()
        btn_row.addStretch(1)

        self.telemeter_start_btn = QPushButton("Start")
        btn_row.addWidget(self.telemeter_start_btn)

        self.telemeter_angle_update_btn = QPushButton("Update Angle")
        btn_row.addWidget(self.telemeter_angle_update_btn)

        btn_row.addStretch(1)
        layout.addLayout(btn_row)

        layout.addSpacing(20)

        self.telemeter_distance_label = QLabel("...")
        self.telemeter_distance_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.telemeter_distance_label)

        layout.addStretch(1)

        # Vertical separator
        frame_layout = QHBoxLayout()
        frame_layout.addStretch(1)
        frame = QFrame()
        frame.setFrameShape(QFrame.VLine)
        frame.setLineWidth(2)
        frame_layout.addWidget(frame)
        frame_layout.addStretch(1)
        layout.addLayout(frame_layout)

        # Signals
        self.telemeter_angle_slider.valueChanged.connect(self.on_telemeter_slider_changed)
        self.telemeter_start_btn.clicked.connect(self.on_telemeter_button_clicked)
        self.telemeter_angle_update_btn.clicked.connect(self.on_telemeter_angle_update_clicked)

        return widget

    def create_sonar_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setAlignment(Qt.AlignHCenter | Qt.AlignTop)

        title = QLabel("Sonar")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        combo_row = QHBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Objects Distance", "Light Sources", "Objects Distance & Light Sources"])
        scan_btn = QPushButton("Scan")

        combo_row.addStretch(1)
        combo_row.addWidget(self.mode_combo)
        combo_row.addWidget(scan_btn)
        combo_row.addStretch(1)
        layout.addLayout(combo_row)

        # Polar plot (matplotlib canvas)
        self.sonar_figure = figure.Figure()
        self.sonar_canvas = FigureCanvasQTAgg(self.sonar_figure)

        self.sonar_axes = self.sonar_figure.add_subplot(111, polar=True)
        self.sonar_axes.set_title("Sonar Polar Plot")
        self.sonar_axes.set_thetamin(0)
        self.sonar_axes.set_thetamax(180)

        # Placeholder data until the scan is plotted
        theta = np.linspace(0, np.pi, 20)
        r = np.random.rand(20) * 10
        self.sonar_axes.plot(theta, r, 'o')

        layout.addWidget(self.sonar_canvas, stretch=1)

        frame_layout = QHBoxLayout()
        frame_layout.addStretch(1)
        frame = QFrame()
        frame.setFrameShape(QFrame.VLine)
        frame.setLineWidth(2)
        frame_layout.addWidget(frame)
        frame_layout.addStretch(1)
        layout.addLayout(frame_layout)

        scan_btn.clicked.connect(lambda: self.scan(self.mode_combo.currentText()))

        return widget

    def create_file_system_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setAlignment(Qt.AlignHCenter | Qt.AlignTop)
        layout.setSpacing(8)

        # Title
        title = QLabel("File System")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # File picker (click to browse)
        self.file_path_edit = QtWidgets.QLineEdit()
        self.file_path_edit.setReadOnly(True)
        self.file_path_edit.setPlaceholderText("Click to choose a .txt file…")
        self.file_path_edit.setCursor(Qt.PointingHandCursor)
        self.file_path_edit.mouseReleaseEvent = lambda e: self._choose_file()
        layout.addWidget(self.file_path_edit)

        # File type + Save on MSP
        row1 = QHBoxLayout()
        self.file_type_combo = QComboBox()
        self.file_type_combo.addItems(["Text", "Script"])
        row1.addWidget(self.file_type_combo)

        self.save_on_msp_btn = QPushButton("Save on MSP")
        row1.addWidget(self.save_on_msp_btn)
        layout.addLayout(row1)

        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)

        # Load from MSP
        self.load_from_msp_btn = QPushButton("Load from MSP")
        layout.addWidget(self.load_from_msp_btn)

        # File list
        self.file_list = QtWidgets.QListWidget()
        layout.addWidget(self.file_list, stretch=1)

        # Run + Save on PC
        row2 = QHBoxLayout()
        self.run_btn = QPushButton("Run")
        row2.addWidget(self.run_btn)

        self.save_on_pc_btn = QPushButton("Save on PC")
        row2.addWidget(self.save_on_pc_btn)

        self.save_on_msp_btn.clicked.connect(self.save_on_MSP)
        self.load_from_msp_btn.clicked.connect(self.load_from_MSP)
        self.run_btn.clicked.connect(self.run_script)
        self.save_on_pc_btn.clicked.connect(self.save_on_PC)

        layout.addLayout(row2)

        return widget

    def _choose_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Choose a text file",
            "",
            "Text Files (*.txt);;All Files (*)"
        )
        if path:
            self.file_path_edit.setText(path)

    def create_console_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        hdr = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("Console")
        title.setStyleSheet("font-weight: bold; font-size: 20px;")
        hdr.addWidget(title)
        hdr.addStretch(1)
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_btn = QtWidgets.QPushButton("Refresh")
        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems([str(b) for b in (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)])
        self.baud_combo.setCurrentText(str(9600))
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        hdr.addWidget(QtWidgets.QLabel("Port:"))
        hdr.addWidget(self.port_combo)
        hdr.addWidget(self.refresh_btn)
        hdr.addSpacing(12)
        hdr.addWidget(QtWidgets.QLabel("Baud:"))
        hdr.addWidget(self.baud_combo)
        hdr.addSpacing(12)
        hdr.addWidget(self.connect_btn)
        hdr.addWidget(self.disconnect_btn)
        layout.addLayout(hdr)
        self.console = QtWidgets.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.console.setFont(QtGui.QFont("Consolas", 10))
        layout.addWidget(self.console, stretch=1)
        tx_row = QtWidgets.QHBoxLayout()
        self.tx_line = QtWidgets.QLineEdit()
        self.tx_line.setPlaceholderText("Type text ...")
        self.send_btn = QtWidgets.QPushButton("Send")
        tx_row.addWidget(self.tx_line, stretch=1)
        tx_row.addWidget(self.send_btn)
        layout.addLayout(tx_row)
        self.status = QtWidgets.QLabel("Disconnected")
        self.status.setStyleSheet("color: gray;")
        layout.addWidget(self.status)
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.connect)
        self.disconnect_btn.clicked.connect(self.disconnect)
        self.send_btn.clicked.connect(lambda: self.transmit(self.tx_line.text()))
        self.tx_line.returnPressed.connect(lambda: self.transmit(self.tx_line.text()))

        return widget

    # =========== Serial Connection ===========

    def refresh_ports(self):
        self.port_combo.clear()
        if serial is None:
            self.port_combo.addItem("(pyserial not available)", "")
            return
        try:
            ports = list(serial.tools.list_ports.comports())
            if not ports:
                self.port_combo.addItem("(no ports)", "")
                return
            for p in ports:
                label = f"{p.device} - {p.description}"
                self.port_combo.addItem(label, p.device)
        except Exception as e:
            self.port_combo.addItem(f"(Error: {e})", "")
            self.print(f"[ERROR] Could not list ports: {e}\n")

    def connect(self):
        if self.serial is not None:
            self.print("[INFO] Already connected.\n")
            return
        dev = self.port_combo.currentData()
        if not dev:
            self.print("[WARN] No port selected.\n")
            return
        baud = int(self.baud_combo.currentText())
        try:
            self.serial = SerialTerminal(port=dev, baud_rate=baud)
            self.serial.signals.data_received.connect(self.receive)
            self.serial.start()
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.status.setText(f"Connected: {dev} @ {baud}")
            self.status.setStyleSheet("color: green;")
            # Removed: self.connected.emit(dev, baud)
        except Exception as e:
            self.print(f"[ERROR] Connect failed: {e}\n")
            self.status.setText("Connect failed")
            self.status.setStyleSheet("color: red;")
            self.serial = None

    def disconnect(self):
        if self.serial is not None:
            try:
                self.serial.stop()
            except Exception as e:
                self.print(f"[ERROR] Stop failed: {e}\n")
            self.serial = None
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.status.setText("Disconnected")
        self.status.setStyleSheet("color: gray;")

    def transmit(self, text):
        self.ack_flag = False           # callers check it; never leave a stale True
        if not text:
            return
        if not self.serial:
            self.print("[WARN] Not connected. Cannot send data.\n")
            return

        self.print(f"[User] {text}")
        # Connect the ACK signal exactly once
        try:
            self.serial.signals.ack_done.disconnect(self.on_ack_done)
        except Exception:
            pass
        self.serial.signals.ack_done.connect(self.on_ack_done)

        # Blocks until the MCU answers (ACK, NACK or timeout)
        self.serial.send_text_frame(text)
        self.tx_line.clear()

    def on_ack_done(self, ok: bool, msg: str):
        self.print(f"[ACK {'OK' if ok else 'FAIL'}] for \"{msg}\"")

        self.ack_flag = ok

    def print(self, text):
        self.console.appendPlainText(text)

    def receive(self, args):
        self.print(args)

        if self.state == State.IDLE:
            pass
        elif self.state == State.TELEMETER:
            self.update_telemeter_distance(args)

    # =========== States ===========

    def set_state(self, state, transmit=None):
        self.transmit(state.value if transmit is None else transmit)

        if not self.ack_flag:
            return

        self.prev_state = self.state
        self.state = state

        if self.state == State.TELEMETER:
            # Entered telemeter mode: send the angle to measure at
            self.telemeter_start_btn.setText("Stop")
            self.transmit(str(self.telemeter_angle_slider.value()))
        else:
            self.telemeter_start_btn.setText("Start")

    # =================== Telemeter ===================

    def on_telemeter_button_clicked(self):
        if self.state != State.TELEMETER:
            self.set_state(State.TELEMETER)
        elif self.state == State.TELEMETER:
            self.set_state(State.IDLE, transmit='q')

    def on_telemeter_slider_changed(self, value: int):
        self.telemeter_angle_value.setText(f"{value}°")

    def on_telemeter_angle_update_clicked(self, args):
        if self.state == State.TELEMETER:
            self.transmit(str(self.telemeter_angle_slider.value()))

    def update_telemeter_distance(self, args):
        self.telemeter_distance_label.setText(args)

    # ============= File System =============

    def save_on_MSP(self):

        path = self.file_path_edit.text().strip()
        if not path:
            self.print("[WARN] No file selected.\n")
            return

        fname = os.path.basename(path)

        try:
            with open(path, "rb") as f:
                data = f.read()
        except Exception as e:
            self.print(f"[ERROR] Could not read file: {e}\n")
            return

        if not data:
            self.print("[WARN] File is empty, nothing to send.\n")
            return

        # 0. Enter file-save mode on the MCU ('e')
        self.print("[INFO] Preparing file ...")
        self.transmit("e")
        if not self.ack_flag:
            self.print("[ERROR] File transfer failed (enter mode).")
            return

        # 1. File name
        self.print(f"[INFO] Sending file name: {fname}")
        self.transmit(fname)
        if not self.ack_flag:
            self.print("[ERROR] File transfer failed (file name).")
            return

        # 2. File type
        file_type = self.file_type_combo.currentText().strip()
        file_type = "txt" if file_type == "Text" else file_type
        self.print(f"[INFO] Sending type: {file_type}")
        self.transmit(file_type if file_type else "UNKNOWN")
        if not self.ack_flag:
            self.print("[ERROR] File transfer failed (type).")
            return

        # 3. Content in 12-byte chunks
        total_len = len(data)
        self.print(f"[INFO] Sending file content ({total_len} bytes) in 12-byte chunks")

        idx = 0
        while idx + 12 <= total_len:
            chunk = data[idx:idx + 12]
            text_chunk = chunk.decode("utf-8", errors="ignore")
            self.transmit(text_chunk)
            if not self.ack_flag:
                self.print("[ERROR] File transfer failed (chunk).")
                return
            idx += 12

        # Remaining bytes
        if idx < total_len:
            leftover = data[idx:total_len]
            text_chunk = leftover.decode("utf-8", errors="ignore")
            self.transmit(text_chunk)
            if not self.ack_flag:
                self.print("[ERROR] File transfer failed (leftover).")
                return

        # 4. End of file ('#' = EOF_CHAR on the MCU)
        self.transmit("#")
        if not self.ack_flag:
            self.print("[ERROR] File transfer failed (EOF).")
            return

        self.print("[INFO] File transfer complete.")

    def load_from_MSP(self):
        pass

    def save_on_PC(self):
        pass

    def run_script(self):
        pass

    def update_sonar(self):
        print("Update Sonar")

    def scan(self, scan_type: str):
        print("Scan")
