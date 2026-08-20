"""
Modern Dark Theme Styling & Stylesheet for PyQt6 UI.
Cyberpunk / Deep Blue Tech aesthetic with glowing badges, clean typography,
and refined interactive button layouts.
"""

DARK_THEME_QSS = """
/* Global Styles */
QWidget {
    background-color: #0b0f19;
    color: #e2e8f0;
    font-family: 'Segoe UI', 'Roboto', 'Inter', -apple-system, sans-serif;
    font-size: 13px;
}

QMainWindow {
    background-color: #080c14;
}

/* Sidebar Navigation */
#sidebar {
    background-color: #0f172a;
    border-right: 1px solid #1e293b;
    min-width: 220px;
    max-width: 240px;
}

#sidebarTitle {
    color: #38bdf8;
    font-size: 16px;
    font-weight: 700;
    letter-spacing: 0.8px;
    padding: 16px 14px 6px 14px;
}

#sidebarSubTitle {
    color: #64748b;
    font-size: 11px;
    padding: 0px 14px 12px 14px;
}

QPushButton.nav-btn {
    background-color: transparent;
    color: #94a3b8;
    border: 1px solid transparent;
    border-radius: 8px;
    text-align: left;
    padding: 10px 14px;
    font-size: 13px;
    font-weight: 500;
    margin: 2px 8px;
}

QPushButton.nav-btn:hover {
    background-color: #1e293b;
    color: #f1f5f9;
    border-color: #334155;
}

QPushButton.nav-btn:checked, QPushButton.nav-btn.active {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #0369a1);
    color: #ffffff;
    font-weight: 600;
    border: 1px solid #38bdf8;
}

/* Cards & Containers */
QFrame.card {
    background-color: #111827;
    border: 1px solid #1f293d;
    border-radius: 12px;
    padding: 14px;
}

QFrame.card:hover {
    border-color: #2563eb;
}

QFrame.metric-card {
    background-color: #131d33;
    border: 1px solid #202d4a;
    border-radius: 10px;
    padding: 12px;
}

/* Section Header */
QLabel.section-title {
    color: #38bdf8;
    font-size: 15px;
    font-weight: 700;
    margin-bottom: 6px;
}

QLabel.card-title {
    color: #94a3b8;
    font-size: 12px;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.5px;
}

QLabel.metric-value {
    color: #f8fafc;
    font-size: 24px;
    font-weight: 700;
}

QLabel.metric-unit {
    color: #64748b;
    font-size: 13px;
    font-weight: 500;
}

/* Standard Buttons */
QPushButton {
    background-color: #1e293b;
    color: #f1f5f9;
    border: 1px solid #334155;
    border-radius: 8px;
    padding: 8px 16px;
    font-size: 12px;
    font-weight: 600;
    min-height: 20px;
}

QPushButton:hover {
    background-color: #334155;
    border-color: #475569;
    color: #ffffff;
}

QPushButton:pressed {
    background-color: #0f172a;
}

QPushButton:disabled {
    background-color: #0f172a;
    color: #475569;
    border-color: #1e293b;
}

/* Primary Action Buttons */
QPushButton.btn-primary {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0284c7, stop:1 #0369a1);
    border: 1px solid #38bdf8;
    color: #ffffff;
}

QPushButton.btn-primary:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0369a1, stop:1 #0284c7);
    border-color: #7dd3fc;
}

/* Success Buttons */
QPushButton.btn-success {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #059669, stop:1 #047857);
    border: 1px solid #34d399;
    color: #ffffff;
}

QPushButton.btn-success:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #047857, stop:1 #059669);
    border-color: #6ee7b7;
}

/* Danger / Emergency Buttons */
QPushButton.btn-danger {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #dc2626, stop:1 #b91c1c);
    border: 1px solid #f87171;
    color: #ffffff;
}

QPushButton.btn-danger:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #b91c1c, stop:1 #dc2626);
    border-color: #fca5a5;
}

/* Warning Buttons */
QPushButton.btn-warning {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #d97706, stop:1 #b45309);
    border: 1px solid #fbbf24;
    color: #ffffff;
}

QPushButton.btn-warning:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #b45309, stop:1 #d97706);
    border-color: #fde68a;
}

/* Fan Toggle Buttons */
QPushButton.btn-fan {
    background-color: #1e293b;
    border: 1px solid #334155;
    color: #94a3b8;
    border-radius: 8px;
    padding: 8px 12px;
    font-size: 12px;
    font-weight: 600;
}

QPushButton.btn-fan:hover {
    background-color: #334155;
    color: #f1f5f9;
}

QPushButton.btn-fan-active {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0284c7, stop:1 #0369a1);
    border: 1px solid #38bdf8;
    color: #ffffff;
    border-radius: 8px;
    padding: 8px 12px;
    font-size: 12px;
    font-weight: 700;
}

/* Jog Buttons */
QPushButton.jog-btn {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #1e293b, stop:1 #131d33);
    color: #38bdf8;
    border: 1px solid #334155;
    border-radius: 10px;
    font-size: 13px;
    font-weight: 700;
    min-height: 42px;
    padding: 6px 10px;
}

QPushButton.jog-btn:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0284c7, stop:1 #0369a1);
    color: #ffffff;
    border-color: #38bdf8;
}

QPushButton.jog-btn:pressed {
    background-color: #0f172a;
}

/* Input Fields & Combos */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background-color: #111827;
    color: #f1f5f9;
    border: 1px solid #374151;
    border-radius: 6px;
    padding: 6px 10px;
    font-size: 12px;
}

QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
    border: 1px solid #38bdf8;
    background-color: #0f172a;
}

QComboBox::drop-down {
    border: none;
    padding-right: 8px;
}

QComboBox QAbstractItemView {
    background-color: #111827;
    border: 1px solid #374151;
    selection-background-color: #0284c7;
    selection-color: #ffffff;
    color: #f1f5f9;
}

/* Sliders */
QSlider::groove:horizontal {
    height: 6px;
    background-color: #1e293b;
    border-radius: 3px;
}

QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #38bdf8);
    border-radius: 3px;
}

QSlider::handle:horizontal {
    background-color: #f8fafc;
    border: 2px solid #0284c7;
    width: 16px;
    margin-top: -5px;
    margin-bottom: -5px;
    border-radius: 8px;
}

QSlider::handle:horizontal:hover {
    background-color: #38bdf8;
    border-color: #f8fafc;
}

/* Progress Bar */
QProgressBar {
    background-color: #1e293b;
    border: 1px solid #334155;
    border-radius: 4px;
    text-align: center;
    color: #ffffff;
    font-size: 11px;
    font-weight: 600;
}

QProgressBar::chunk {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #38bdf8);
    border-radius: 3px;
}

/* Badges */
QLabel.badge {
    border-radius: 4px;
    padding: 3px 8px;
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.5px;
}

QLabel.badge-green {
    background-color: #064e3b;
    color: #34d399;
    border: 1px solid #059669;
}

QLabel.badge-red {
    background-color: #7f1d1d;
    color: #f87171;
    border: 1px solid #dc2626;
}

QLabel.badge-yellow {
    background-color: #78350f;
    color: #fbbf24;
    border: 1px solid #d97706;
}

QLabel.badge-blue {
    background-color: #0c4a6e;
    color: #38bdf8;
    border: 1px solid #0284c7;
}

QLabel.badge-gray {
    background-color: #1e293b;
    color: #94a3b8;
    border: 1px solid #334155;
}

/* Terminal Console & Table */
QPlainTextEdit.console {
    background-color: #060911;
    color: #38bdf8;
    font-family: 'Cascadia Code', 'Consolas', 'Courier New', monospace;
    font-size: 12px;
    border: 1px solid #1e293b;
    border-radius: 6px;
    padding: 8px;
}

QTableWidget {
    background-color: #0b0f19;
    alternate-background-color: #0f172a;
    color: #f1f5f9;
    gridline-color: #1e293b;
    border: 1px solid #1e293b;
    border-radius: 6px;
}

QTableWidget::item:selected {
    background-color: #0284c7;
    color: #ffffff;
}

QHeaderView::section {
    background-color: #0f172a;
    color: #94a3b8;
    font-weight: 600;
    padding: 6px;
    border: none;
    border-bottom: 1px solid #1e293b;
}

/* Scrollbars */
QScrollBar:vertical {
    border: none;
    background: #0b0f19;
    width: 8px;
    margin: 0px 0px 0px 0px;
}

QScrollBar::handle:vertical {
    background: #1e293b;
    min-height: 20px;
    border-radius: 4px;
}

QScrollBar::handle:vertical:hover {
    background: #334155;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

QScrollBar:horizontal {
    border: none;
    background: #0b0f19;
    height: 8px;
    margin: 0px 0px 0px 0px;
}

QScrollBar::handle:horizontal {
    background: #1e293b;
    min-width: 20px;
    border-radius: 4px;
}

QScrollBar::handle:horizontal:hover {
    background: #334155;
}

QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0px;
}
"""
