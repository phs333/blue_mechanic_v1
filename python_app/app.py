"""
Blue Mechanic V1 Desktop Application Entry Point.
Run with:
    python -m python_app.app
or:
    python python_app/app.py
"""

import sys
import os

# Ensure package is resolvable
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, ".."))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt

from python_app.core.state_model import DeviceState
from python_app.core.comm_manager import CommManager
from python_app.ui.main_window import MainWindow

def main():
    # High DPI scaling attributes for modern displays
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )
    
    app = QApplication(sys.argv)
    app.setApplicationName("Blue Mechanic V1")
    app.setOrganizationName("Blue Mechanic")
    
    # Initialize Core reactive models
    state = DeviceState()
    comm = CommManager(state)
    
    # Initialize UI
    window = MainWindow(comm, state)
    window.showMaximized()
    
    # Clean exit
    exit_code = app.exec()
    comm.disconnect_all()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
