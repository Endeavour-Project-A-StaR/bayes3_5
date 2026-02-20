import sys
import json
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class GroundControlStation(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ser = None
        self.configs = {}
        
        self.setWindowTitle("RACS - Ground Control Station")
        self.resize(1100, 700) 
        self.init_ui()

        # Serial polling timer (Runs at 50Hz)
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QHBoxLayout(main_widget)

        # ==========================================
        # LEFT PANEL: Connection & Commands
        # ==========================================
        left_panel = QVBoxLayout()
        
        # --- Connection Box ---
        conn_box = QGroupBox("Serial Connection")
        conn_layout = QVBoxLayout()
        
        # Port Dropdown and Refresh Button
        port_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        port_layout.addWidget(self.port_combo)
        
        self.btn_refresh = QPushButton("🔄")
        self.btn_refresh.setMaximumWidth(40)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        port_layout.addWidget(self.btn_refresh)
        
        conn_layout.addWidget(QLabel("Select Port:"))
        conn_layout.addLayout(port_layout)
        
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.btn_connect)
        
        self.btn_init = QPushButton("Send WakeUp")
        self.btn_init.clicked.connect(lambda: self.send_cmd("\n"))
        self.btn_init.setStyleSheet("background-color: #ffd166; font-weight: bold;")
        conn_layout.addWidget(self.btn_init)
        
        conn_box.setLayout(conn_layout)
        left_panel.addWidget(conn_box)

        # --- Flight Modes Box ---
        mode_box = QGroupBox("Flight Modes")
        mode_layout = QVBoxLayout()
        
        self.btn_preflt = QPushButton("PREFLIGHT")
        self.btn_preflt.clicked.connect(lambda: self.send_cmd("PREFLT"))
        mode_layout.addWidget(self.btn_preflt)
        
        self.btn_ovrd = QPushButton("GROUND OVERRIDE")
        self.btn_ovrd.clicked.connect(lambda: self.send_cmd("OVRD"))
        mode_layout.addWidget(self.btn_ovrd)
        
        self.btn_arm = QPushButton("ARM (NAV LOCK)")
        self.btn_arm.clicked.connect(lambda: self.send_cmd("ARM"))
        self.btn_arm.setStyleSheet("background-color: #ef476f; color: black; font-weight: bold;")
        mode_layout.addWidget(self.btn_arm)
        
        mode_box.setLayout(mode_layout)
        left_panel.addWidget(mode_box)

        # --- EEPROM Commands Box ---
        eeprom_box = QGroupBox("EEPROM Configuration")
        eeprom_layout = QVBoxLayout()
        
        self.btn_dump = QPushButton("DUMP Config")
        self.btn_dump.clicked.connect(lambda: self.send_cmd("DUMP"))
        eeprom_layout.addWidget(self.btn_dump)

        self.btn_save = QPushButton("WRITE Config to EEPROM")
        self.btn_save.clicked.connect(lambda: self.send_cmd("SAVE"))
        eeprom_layout.addWidget(self.btn_save)

        self.btn_default = QPushButton("Restore DEFAULTS")
        self.btn_default.clicked.connect(lambda: self.send_cmd("DEFAULT"))
        eeprom_layout.addWidget(self.btn_default)
        
        self.btn_reset = QPushButton("MAGICRESET (FC Reboot)")
        self.btn_reset.clicked.connect(lambda: self.send_cmd("MAGICRESET"))
        self.btn_reset.setStyleSheet("background-color: #118ab2; color: white; font-weight: bold;")
        eeprom_layout.addWidget(self.btn_reset)
        
        eeprom_box.setLayout(eeprom_layout)
        left_panel.addWidget(eeprom_box)

        # --- Manual Command Box ---
        cmd_box = QGroupBox("Command")
        cmd_layout = QHBoxLayout()
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("SET <VAR> <VALUE>")
        self.cmd_input.returnPressed.connect(self.send_manual_cmd)
        btn_send = QPushButton("Send")
        btn_send.clicked.connect(self.send_manual_cmd)
        cmd_layout.addWidget(self.cmd_input)
        cmd_layout.addWidget(btn_send)
        cmd_box.setLayout(cmd_layout)
        left_panel.addWidget(cmd_box)
        
        left_panel.addStretch()
        layout.addLayout(left_panel, 1)

        # ==========================================
        # MIDDLE PANEL: Servos & Live Telemetry
        # ==========================================
        mid_panel = QVBoxLayout()
        
        # --- Servos Box ---
        servo_box = QGroupBox("Live Actuators (Degrees)")
        servo_layout = QHBoxLayout()
        self.servo_bars = []
        for i in range(4):
            bar_layout = QVBoxLayout()
            bar = QProgressBar()
            bar.setOrientation(Qt.Vertical)
            bar.setMinimum(60) 
            bar.setMaximum(120)
            bar.setValue(90)
            bar.setFormat("%v°")
            bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.servo_bars.append(bar)
            bar_layout.addWidget(bar, alignment=Qt.AlignHCenter)
            bar_layout.addWidget(QLabel(f"S{i+1}"), alignment=Qt.AlignHCenter)
            servo_layout.addLayout(bar_layout)
        servo_box.setLayout(servo_layout)
        mid_panel.addWidget(servo_box, 3) 

        # --- Telemetry Box ---
        telem_box = QGroupBox("Live Flight Data")
        telem_layout = QVBoxLayout()
        self.telem_label = QLabel("Waiting for data...")
        self.telem_label.setFont(QFont("Courier", 10))
        self.telem_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        telem_layout.addWidget(self.telem_label)
        telem_box.setLayout(telem_layout)
        mid_panel.addWidget(telem_box, 1) 
        
        layout.addLayout(mid_panel, 1)

        # ==========================================
        # RIGHT PANEL: Config Table & Message Log
        # ==========================================
        right_panel = QVBoxLayout()

        # --- Config Table ---
        table_box = QGroupBox("Active Configuration")
        table_layout = QVBoxLayout()
        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["Parameter", "Value"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table_layout.addWidget(self.table)
        table_box.setLayout(table_layout)
        right_panel.addWidget(table_box, 1)

        # --- Message Log ---
        log_box = QGroupBox("FC Messages")
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Courier", 9))
        log_layout.addWidget(self.log_text)
        log_box.setLayout(log_layout)
        right_panel.addWidget(log_box, 1)

        layout.addLayout(right_panel, 2)
        
        # Initial Port Search
        self.refresh_ports()

    # --- UI Interactions ---
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        
        # Sort in reverse order so ACM1 comes before ACM0
        ports = sorted(ports, key=lambda p: p.device, reverse=True)
        
        best_port = None
        for p in ports:
            self.port_combo.addItem(p.device)
            # Check for Teensy VID (0x16C0) or typical Linux nomenclature
            if (p.vid == 0x16C0) or ("ACM" in p.device) or ("USB" in p.description):
                if not best_port:
                    best_port = p.device
                    
        # Automatically select the most likely Flight Controller port
        if best_port:
            index = self.port_combo.findText(best_port)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def log_msg(self, msg, color="black"):
        self.log_text.setTextColor(QColor(color))
        self.log_text.append(msg)
        self.log_text.moveCursor(QTextCursor.End)

    def toggle_connection(self):
        if self.ser is None or not self.ser.is_open:
            port = self.port_combo.currentText()
            if not port: return
            try:
                self.ser = serial.Serial(port, 115200, timeout=0)
                self.btn_connect.setText("Disconnect")
                self.log_msg(f"--- CONNECTED TO {port} ---", "blue")
                self.timer.start(20) 
            except Exception as e:
                self.log_msg(f"Failed to connect: {str(e)}", "red")
        else:
            self.timer.stop()
            self.ser.close()
            self.btn_connect.setText("Connect")
            self.log_msg("--- DISCONNECTED ---", "blue")
            self.refresh_ports() # Pre-search for next connection attempt

    def send_cmd(self, cmd):
        if self.ser and self.ser.is_open:
            full_cmd = f"{cmd}\n" 
            self.ser.write(full_cmd.encode('utf-8'))
            self.log_msg(f"Sent: {cmd}", "green")

    def send_manual_cmd(self):
        cmd = self.cmd_input.text().strip()
        if cmd:
            self.send_cmd(cmd)
            self.cmd_input.clear()

    # --- Data Processing ---
    def update_config_table(self):
        self.table.setRowCount(len(self.configs))
        row = 0
        for key, val in self.configs.items():
            self.table.setItem(row, 0, QTableWidgetItem(key))
            self.table.setItem(row, 1, QTableWidgetItem(val))
            row += 1

    def dynamically_update_servo_limits(self):
        try:
            # Fix: The FC mathematical center is always 90 degrees!
            center = 90.0
            limit = float(self.configs.get("SERVO_LIMIT", 30.0))
            
            min_val = int(center - limit)
            max_val = int(center + limit)
            
            for bar in self.servo_bars:
                bar.setMinimum(min_val)
                bar.setMaximum(max_val)
        except ValueError:
            pass

    def process_line(self, line):
        if not line: return

        # 1. Telemetry Data (JSON)
        if line.startswith("{"):
            try:
                data = json.loads(line)
                
                # Fix: The FC mathematical center is always 90 degrees!
                center = 90.0
                limit = float(self.configs.get("SERVO_LIMIT", 30.0))
                min_val = int(center - limit)
                max_val = int(center + limit)

                # Update Servos
                servos = data.get("servo", [90, 90, 90, 90])
                for i in range(4):
                    val = max(min_val, min(max_val, int(servos[i]))) 
                    self.servo_bars[i].setValue(val)
                
                # Update Telem Text
                telem_str = (
                    f"Timestamp : {data.get('timestamp', 0)} ms\n"
                    f"State     : {data.get('state', 0)}\n\n"
                    f"Altitude  : {data.get('altitude', 0.0):.2f} m\n"
                    f"Pressure  : {data.get('pressure', 0.0):.2f} hPa\n\n"
                    f"Quats     : {data.get('quats', [0,0,0,0])}\n"
                    f"Gyro (dps): {data.get('raw_gyro', [0,0,0])}\n"
                    f"Accel (G) : {data.get('raw_accel', [0,0,0])}\n"
                )
                self.telem_label.setText(telem_str)
                
            except json.JSONDecodeError:
                pass 

        # 2. Configuration Readout
        elif line.startswith("CFG:"):
            self.log_msg(line, "#d08c60") 
            parts = line.split(" ")
            if len(parts) >= 3:
                key = parts[1]
                val = parts[2]
                self.configs[key] = val
                self.update_config_table()
                
                # Only check for SERVO_LIMIT now
                if key == "SERVO_LIMIT":
                    self.dynamically_update_servo_limits()

        # 3. System Messages
        elif line.startswith("MSG:"):
            self.log_msg(line, "#118ab2") 
            
            # AUTOMATIC DUMP TRIGGER
            if "BAYES READY" in line:
                self.log_msg("--- BOOT COMPLETE: AUTO-DUMPING CONFIG ---", "purple")
                self.send_cmd("DUMP")

        # 4. Unformatted prints or debugs
        else:
            self.log_msg(line, "gray")

    def read_serial(self):
        if self.ser and self.ser.is_open:
            try:
                while self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    self.process_line(line)
            except Exception as e:
                self.log_msg(f"Serial Error: {str(e)}", "red")
                self.toggle_connection() # This triggers disconnect and auto-refreshes the port list!

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gcs = GroundControlStation()
    gcs.show()
    sys.exit(app.exec_())
