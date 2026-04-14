#!/usr/bin/env python3
"""
ROS2 Command GUI - Interface to run commands from bashrc
Grouped by categories: Gazebo, Vision, ArduPilot, and ROS2
"""

import sys
import subprocess
import threading
import warnings
import time
from threading import Lock

# Suppress PyQt5 deprecation warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QTabWidget, QTextEdit, QLabel, QComboBox, QStyleFactory,
    QSplitter
)
from PyQt5.QtCore import Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QIcon

# Mapping commands from bashrc
COMMANDS = {
    "Gazebo": {
        "Qualification World": "gz sim -v 3 -r sauvc_qualification.world",
        "Final World": "gz sim -v 3 -r sauvc_final.world",
    },
    "Vision": {
        "Front Camera Bridge": "ros2 run ros_gz_bridge parameter_bridge '/front_camera@sensor_msgs/msg/Image@gz.msgs.Image'",
        "Docker Container": "docker start -ai be537dc7c441",
    },
    "RQT": {
        "RQT Image View": "ros2 run rqt_image_view rqt_image_view",
    },
    "ArduPilot": {
        "--- SITL ---": "",  # Visual separator
        "Start SITL": "cd ~/ardupilot && Tools/autotest/sim_vehicle.py -L RATBeach -v ArduSub -f vectored --model=JSON --out=udp:0.0.0.0:14550 --console",
        "--- MAVRoS ---": "",  # Visual separator
        "Launch MAVRoS": "ros2 launch mavros apm.launch fcu_url:=udp://:14550@localhost:14555",
    },
    "ROS2": {
        "Build Package": "cd ~/ros2_ws && colcon build --packages-select sauvc26_code",
        "Arm": "ros2 run sauvc26_code arm",
        "Qualification": "ros2 run sauvc26_code qualification",
        "Final": "ros2 run sauvc26_code final",
        "Move": "ros2 run sauvc26_code move",
        "Test": "ros2 run sauvc26_code test",
        "Teleop Keyboard": "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/mavros/setpoint_velocity/cmd_vel_unstamped",
    }
}


class CommandExecutor(QObject):
    """Worker thread to run commands without freezing GUI"""
    output_signal = pyqtSignal(str)
    finished_signal = pyqtSignal()

    def __init__(self, output_widget=None, ui_lock=None):
        super().__init__()
        self.process = None
        self.output_widget = output_widget  # Direct widget reference
        self.ui_lock = ui_lock  # Thread lock for UI safety

    def append_to_widget(self, text):
        """Safely append text to output widget"""
        print(f"DEBUG append_to_widget called: output_widget={self.output_widget is not None}, ui_lock={self.ui_lock is not None}")
        if self.output_widget and self.ui_lock:
            print(f"DEBUG: Attempting to append...")
            with self.ui_lock:
                self.output_widget.append(text.rstrip('\n'))
                print(f"DEBUG append_to_widget: appended {len(text)} chars")
        else:
            print(f"DEBUG: output_widget or ui_lock is None!")

    def run_command(self, command):
        """Run command and emit output"""
        print(f"DEBUG: run_command called with: {command}")
        try:
            text = f"▶ Running: {command}\n"
            print(f"DEBUG: Appending initial text")
            self.append_to_widget(text)
            self.append_to_widget("─" * 80)

            # Run command using shell
            self.process = subprocess.Popen(
                command,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            print(f"DEBUG: Process started, PID: {self.process.pid}")

            # Read output in real-time
            if self.process.stdout:
                print(f"DEBUG: stdout is available")
                line_count = 0
                while True:
                    line = self.process.stdout.readline()
                    if not line:
                        print(f"DEBUG: No more lines from stdout")
                        break
                    line_count += 1
                    if line_count % 10 == 0:
                        print(f"DEBUG: Read {line_count} lines so far")
                    if line.strip():  # Only append non-empty lines
                        print(f"DEBUG: Calling append_to_widget for line {line_count}")
                        self.append_to_widget(line)
                    else:
                        print(f"DEBUG: Skipping empty line {line_count}")
            else:
                print(f"DEBUG: stdout is NOT available!")

            self.process.wait()
            print(f"DEBUG: Process finished with return code: {self.process.returncode}")
            self.append_to_widget("─" * 80)
            self.append_to_widget(f"✓ Command finished (exit code: {self.process.returncode})\n")

        except Exception as e:
            print(f"DEBUG: Exception occurred: {e}")
            self.append_to_widget(f"✗ Error: {str(e)}\n")

        finally:
            print(f"DEBUG: Emitting finished signal")
            self.finished_signal.emit()

    def kill_process(self):
        """Kill the running process"""
        if self.process and self.process.poll() is None:
            try:
                self.process.terminate()
                self.process.wait(timeout=2)
            except:
                try:
                    self.process.kill()
                except:
                    pass


class ROS2CommandGUI(QMainWindow):
    """Main GUI Application"""

    def __init__(self):
        super().__init__()
        self.output_widgets = {}  # Store output text widgets for each category
        self.worker_threads = {}  # Store worker threads for each category
        self.executors = {}  # Store executors for each category
        self.command_output_map = {}  # Map command name to its output widget key
        self.ui_lock = Lock()  # Thread lock for UI safety
        self.init_ui()

    def init_ui(self):
        """Initialize UI"""
        self.setWindowTitle("ROS2 Command GUI")
        self.setGeometry(100, 100, 1200, 800)

        # Set style
        QApplication.setStyle(QStyleFactory.create('Fusion'))

        # Main widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)

        # Main layout
        layout = QVBoxLayout(main_widget)

        # Tab widget - each tab has its own commands and terminal
        self.tab_widget = QTabWidget()
        self.create_tabs()
        layout.addWidget(self.tab_widget, 1)

    def create_tabs(self):
        """Create tabs for each category with separate output terminals"""
        for category, commands in COMMANDS.items():
            # Check if this category should display output
            has_output = category != "RQT"
            
            # Special handling for ArduPilot (2 output terminals)
            if category == "ArduPilot":
                self.create_ardupilot_tab(commands)
            else:
                # Create main container for this category
                tab_container = QWidget()
                if has_output:
                    tab_layout = QHBoxLayout(tab_container)
                else:
                    tab_layout = QVBoxLayout(tab_container)

                # Left side - Command buttons
                left_widget = QWidget()
                left_layout = QVBoxLayout(left_widget)

                # Title
                title = QLabel(category)
                title.setFont(QFont("Arial", 12, QFont.Bold))
                left_layout.addWidget(title)

                # Buttons for each command
                for cmd_name, cmd_string in commands.items():
                    # Check if this is a separator
                    is_separator = cmd_string == ""
                    
                    btn = QPushButton(cmd_name)
                    btn.setFont(QFont("Arial", 10))
                    btn.setMinimumHeight(40)
                    
                    if is_separator:
                        # Make separator unclickable
                        btn.setEnabled(False)
                    else:
                        btn.clicked.connect(lambda checked, cat=category, cmd=cmd_string, name=cmd_name: self.run_command(cat, cmd, name))
                    
                    left_layout.addWidget(btn)

                # Spacer
                left_layout.addStretch()
                
                # Kill button for this category
                if has_output:
                    kill_btn = QPushButton("Kill Terminal")
                    kill_btn.setStyleSheet("background-color: #ff6b6b; color: white; font-weight: bold;")
                    kill_btn.clicked.connect(lambda checked, cat=category: self.kill_terminal(cat))
                    left_layout.addWidget(kill_btn)

                if has_output:
                    # Right side - Output terminal for this category
                    output_layout = QVBoxLayout()
                    output_label = QLabel(f"{category} Output Console")
                    output_label.setFont(QFont("Arial", 10, QFont.Bold))
                    output_layout.addWidget(output_label)

                    output_text = QTextEdit()
                    output_text.setReadOnly(True)
                    output_text.setFont(QFont("Courier", 9))
                    output_layout.addWidget(output_text)

                    # Store output widget for this category
                    self.output_widgets[category] = output_text

                    # Button layout for clear and kill
                    button_layout = QHBoxLayout()
                    clear_btn = QPushButton("Clear Output")
                    clear_btn.clicked.connect(output_text.clear)
                    button_layout.addWidget(clear_btn)
                    
                    output_layout.addLayout(button_layout)

                    right_widget = QWidget()
                    right_widget.setLayout(output_layout)

                    # Add left and right to tab
                    tab_layout.addWidget(left_widget, 1)
                    tab_layout.addWidget(right_widget, 1)
                else:
                    # For categories without output (like RQT)
                    tab_layout.addWidget(left_widget, 1)

                # Add tab
                self.tab_widget.addTab(tab_container, category)

    def create_ardupilot_tab(self, commands):
        """Create ArduPilot tab with 2 separate output terminals (SITL and MAVRoS)"""
        # Create main container
        tab_container = QWidget()
        main_layout = QHBoxLayout(tab_container)

        # Left side - Command buttons
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)

        # Title
        title = QLabel("ArduPilot")
        title.setFont(QFont("Arial", 12, QFont.Bold))
        left_layout.addWidget(title)

        # Buttons for each command
        for cmd_name, cmd_string in commands.items():
            is_separator = cmd_string == ""
            
            btn = QPushButton(cmd_name)
            btn.setFont(QFont("Arial", 10))
            btn.setMinimumHeight(40)
            
            if is_separator:
                btn.setEnabled(False)
            else:
                btn.clicked.connect(lambda checked, cat="ArduPilot", cmd=cmd_string, name=cmd_name: self.run_command(cat, cmd, name))
            
            left_layout.addWidget(btn)

        # Spacer
        left_layout.addStretch()
        
        # Kill button
        kill_btn = QPushButton("Kill Terminal")
        kill_btn.setStyleSheet("background-color: #ff6b6b; color: white; font-weight: bold;")
        kill_btn.clicked.connect(lambda checked, cat="ArduPilot": self.kill_terminal(cat))
        left_layout.addWidget(kill_btn)

        # Right side - 2 output terminals stacked vertically
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)

        # SITL Terminal
        sitl_layout = QVBoxLayout()
        sitl_label = QLabel("SITL Output")
        sitl_label.setFont(QFont("Arial", 10, QFont.Bold))
        sitl_layout.addWidget(sitl_label)

        sitl_text = QTextEdit()
        sitl_text.setReadOnly(True)
        sitl_text.setFont(QFont("Courier", 9))
        sitl_layout.addWidget(sitl_text)

        sitl_clear_btn = QPushButton("Clear")
        sitl_clear_btn.clicked.connect(sitl_text.clear)
        sitl_layout.addWidget(sitl_clear_btn)

        # Store SITL output widget
        self.output_widgets["ArduPilot_SITL"] = sitl_text
        
        # Assign SITL commands to SITL output
        self.command_output_map["Start SITL"] = "ArduPilot_SITL"

        # MAVRoS Terminal
        mavros_layout = QVBoxLayout()
        mavros_label = QLabel("MAVRoS Output")
        mavros_label.setFont(QFont("Arial", 10, QFont.Bold))
        mavros_layout.addWidget(mavros_label)

        mavros_text = QTextEdit()
        mavros_text.setReadOnly(True)
        mavros_text.setFont(QFont("Courier", 9))
        mavros_layout.addWidget(mavros_text)

        mavros_clear_btn = QPushButton("Clear")
        mavros_clear_btn.clicked.connect(mavros_text.clear)
        mavros_layout.addWidget(mavros_clear_btn)

        # Store MAVRoS output widget
        self.output_widgets["ArduPilot_MAVRoS"] = mavros_text
        
        # Assign MAVRoS commands to MAVRoS output
        self.command_output_map["Launch MAVRoS"] = "ArduPilot_MAVRoS"

        # Add both terminals to right layout vertically
        sitl_widget = QWidget()
        sitl_widget.setLayout(sitl_layout)
        mavros_widget = QWidget()
        mavros_widget.setLayout(mavros_layout)
        
        right_layout.addWidget(sitl_widget, 1)
        right_layout.addWidget(mavros_widget, 1)

        # Add left and right to main layout
        main_layout.addWidget(left_widget, 1)
        main_layout.addWidget(right_container, 2)

        # Add tab
        self.tab_widget.addTab(tab_container, "ArduPilot")

    def run_command(self, category, command, name):
        """Run command in separate thread for specific category"""
        
        # Determine output widget for this command
        if name in self.command_output_map:
            output_key = self.command_output_map[name]
            output_widget = self.output_widgets.get(output_key)
        elif category in self.output_widgets:
            output_key = category
            output_widget = self.output_widgets.get(output_key)
        else:
            # For RQT and other no-output categories
            output_widget = None
            output_key = None
        
        if output_widget:
            # Has output terminal
            with self.ui_lock:
                output_widget.append(f"\n{'='*80}")
                output_widget.append(f"Command: {name}")
                output_widget.append(f"{'='*80}")

            # Create unique key for tracking this execution
            exec_key = f"{category}_{name}" if category == "ArduPilot" else category

            # Stop previous worker thread if still running
            if exec_key in self.worker_threads and self.worker_threads[exec_key] and self.worker_threads[exec_key].isAlive():
                with self.ui_lock:
                    output_widget.append("⚠ Previous command is still running...")
                return

            # Create executor and thread - PASS OUTPUT WIDGET DIRECTLY!
            executor = CommandExecutor(output_widget=output_widget, ui_lock=self.ui_lock)
            worker_thread = threading.Thread(target=executor.run_command, args=(command,))
            worker_thread.daemon = True

            # Store references
            self.executors[exec_key] = executor
            self.worker_threads[exec_key] = worker_thread

            # Start thread
            worker_thread.start()
        else:
            # No output - just run in background (like RQT)
            print(f"DEBUG: Running {name} without output")
            executor = CommandExecutor(output_widget=None, ui_lock=self.ui_lock)
            worker_thread = threading.Thread(target=executor.run_command, args=(command,))
            worker_thread.daemon = True
            self.executors[category] = executor
            self.worker_threads[category] = worker_thread
            worker_thread.start()

    def kill_terminal(self, category):
        """Kill the running command in a category"""
        if category == "ArduPilot":
            # Kill both SITL and MAVRoS if running
            for exec_key in ["ArduPilot_Start SITL", "ArduPilot_Launch MAVRoS"]:
                if exec_key in self.executors and self.executors[exec_key]:
                    self.executors[exec_key].kill_process()
            # Notify both terminals
            if "ArduPilot_SITL" in self.output_widgets:
                with self.ui_lock:
                    self.output_widgets["ArduPilot_SITL"].append("✗ Terminal killed by user")
            if "ArduPilot_MAVRoS" in self.output_widgets:
                with self.ui_lock:
                    self.output_widgets["ArduPilot_MAVRoS"].append("✗ Terminal killed by user")
        else:
            if category in self.executors and self.executors[category]:
                self.executors[category].kill_process()
                if category in self.output_widgets:
                    with self.ui_lock:
                        self.output_widgets[category].append("✗ Terminal killed by user")

    def append_output(self, output_key, text):
        """Deprecated - output now directly appends from CommandExecutor"""
        pass

    def command_finished(self, category):
        """Called when command finishes for a specific category"""
        pass


def main():
    app = QApplication(sys.argv)
    gui = ROS2CommandGUI()
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
