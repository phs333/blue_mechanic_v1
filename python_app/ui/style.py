"""
Modern Cyber-Industrial Theme Styling & Stylesheet for PyQt6 UI.
Clean dark palette, neon cyan/blue/emerald accents, glowing badges,
refined typography, tactile buttons, and sleek card surfaces.
"""

DARK_THEME_QSS = """
/* Global Base */
QWidget {
    background-color: #080c14;
    color: #e2e8f0;
    font-family: 'Segoe UI', 'Roboto', 'Inter', -apple-system, sans-serif;
    font-size: 13px;
    selection-background-color: #0284c7;
    selection-color: #ffffff;
}

QMainWindow {
    background-color: #060910;
}

/* Scrollbars */
QScrollBar:vertical {
    border: none;
    background: #0b111e;
    width: 8px;
    border-radius: 4px;
    margin: 2px;
}
QScrollBar::handle:vertical {
    background: #1e293b;
    border-radius: 4px;
    min-height: 24px;
}
QScrollBar::handle:vertical:hover {
    background: #38bdf8;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

QScrollBar:horizontal {
    border: none;
    background: #0b111e;
    height: 8px;
    border-radius: 4px;
    margin: 2px;
}
QScrollBar::handle:horizontal {
    background: #1e293b;
    border-radius: 4px;
    min-width: 24px;
}
QScrollBar::handle:horizontal:hover {
    background: #38bdf8;
}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0px;
}

/* Sidebar Navigation */
#sidebar {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0d1527, stop:1 #090e1a);
    border-right: 1px solid #1e293b;
    min-width: 230px;
    max-width: 250px;
}

#sidebarTitle {
    color: #38bdf8;
    font-size: 17px;
    font-weight: 800;
    letter-spacing: 1.0px;
    padding: 18px 16px 4px 16px;
}

#sidebarSubTitle {
    color: #64748b;
    font-size: 11px;
    font-weight: 500;
    padding: 0px 16px 16px 16px;
}

QPushButton.nav-btn {
    background-color: transparent;
    color: #94a3b8;
    border: 1px solid transparent;
    border-radius: 10px;
    text-align: left;
    padding: 12px 16px;
    font-size: 13px;
    font-weight: 600;
    margin: 3px 10px;
}

QPushButton.nav-btn:hover {
    background-color: #162238;
    color: #38bdf8;
    border: 1px solid #233554;
}

QPushButton.nav-btn:checked, QPushButton.nav-btn.active {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #0369a1);
    color: #ffffff;
    font-weight: 700;
    border: 1px solid #38bdf8;
}

/* Cards & Glassmorphism Containers */
QFrame.card {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0f172a, stop:1 #0c1322);
    border: 1px solid #1e293b;
    border-radius: 14px;
    padding: 16px;
}

QFrame.card:hover {
    border-color: #2b436b;
}

QFrame.metric-card {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #111c33, stop:1 #0d1527);
    border: 1px solid #1e2e4a;
    border-radius: 12px;
    padding: 14px;
}

QFrame.metric-card:hover {
    border-color: #38bdf8;
}

/* Typography & Section Titles */
QLabel.section-title {
    color: #38bdf8;
    font-size: 15px;
    font-weight: 800;
    letter-spacing: 0.4px;
}

QLabel.card-title {
    color: #94a3b8;
    font-size: 11px;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.8px;
}

QLabel.metric-value {
    color: #f8fafc;
    font-size: 26px;
    font-weight: 800;
    font-family: 'Consolas', 'JetBrains Mono', 'Segoe UI', monospace;
}

QLabel.metric-unit {
    color: #64748b;
    font-size: 13px;
    font-weight: 600;
}

/* Standard Buttons */
QPushButton {
    background-color: #162033;
    color: #f1f5f9;
    border: 1px solid #23334d;
    border-radius: 8px;
    padding: 8px 16px;
    font-size: 12px;
    font-weight: 600;
    min-height: 22px;
}

QPushButton:hover {
    background-color: #202f4a;
    border-color: #38bdf8;
    color: #38bdf8;
}

QPushButton:pressed {
    background-color: #0b1322;
}

QPushButton:disabled {
    background-color: #0b111e;
    color: #475569;
    border-color: #172233;
}

/* Action Styles */
QPushButton.btn-primary {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0284c7, stop:1 #0369a1);
    border: 1px solid #38bdf8;
    color: #ffffff;
    font-weight: 700;
}

QPushButton.btn-primary:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0369a1, stop:1 #0284c7);
    border-color: #7dd3fc;
    color: #ffffff;
}

QPushButton.btn-success {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #059669, stop:1 #047857);
    border: 1px solid #34d399;
    color: #ffffff;
    font-weight: 700;
}

QPushButton.btn-success:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #047857, stop:1 #059669);
    border-color: #6ee7b7;
    color: #ffffff;
}

QPushButton.btn-danger {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #dc2626, stop:1 #b91c1c);
    border: 1px solid #f87171;
    color: #ffffff;
    font-weight: 700;
}

QPushButton.btn-danger:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #b91c1c, stop:1 #dc2626);
    border-color: #fca5a5;
    color: #ffffff;
}

QPushButton.btn-warning {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #d97706, stop:1 #b45309);
    border: 1px solid #fbbf24;
    color: #ffffff;
    font-weight: 700;
}

QPushButton.btn-warning:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #b45309, stop:1 #d97706);
    border-color: #fde68a;
    color: #ffffff;
}

/* Fan Toggle Buttons */
QPushButton.btn-fan {
    background-color: #131d31;
    border: 1px solid #202e47;
    color: #94a3b8;
    border-radius: 8px;
    padding: 8px 12px;
    font-size: 12px;
    font-weight: 600;
}

QPushButton.btn-fan:hover {
    background-color: #1e2c45;
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
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #16243d, stop:1 #0f1a2e);
    color: #38bdf8;
    border: 1px solid #233758;
    border-radius: 10px;
    font-size: 13px;
    font-weight: 700;
    min-height: 44px;
    padding: 8px 12px;
}

QPushButton.jog-btn:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0284c7, stop:1 #0369a1);
    color: #ffffff;
    border-color: #38bdf8;
}

QPushButton.jog-btn:pressed {
    background-color: #091120;
}

/* Step Selector Pill Buttons */
QPushButton.step-pill {
    background-color: #131d31;
    color: #94a3b8;
    border: 1px solid #202e47;
    border-radius: 14px;
    padding: 5px 12px;
    font-size: 11px;
    font-weight: 600;
}

QPushButton.step-pill:hover {
    color: #38bdf8;
    border-color: #38bdf8;
}

QPushButton.step-pill-active {
    background: #0284c7;
    color: #ffffff;
    border: 1px solid #38bdf8;
    border-radius: 14px;
    padding: 5px 12px;
    font-size: 11px;
    font-weight: 700;
}

/* Input Fields & Combos */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background-color: #0b1220;
    color: #f1f5f9;
    border: 1px solid #1f2e47;
    border-radius: 8px;
    padding: 7px 12px;
    font-size: 12px;
}

QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
    border: 1px solid #38bdf8;
    background-color: #0e172a;
}

QComboBox::drop-down {
    border: none;
    padding-right: 8px;
}

QComboBox QAbstractItemView {
    background-color: #0f172a;
    color: #f1f5f9;
    border: 1px solid #334155;
    selection-background-color: #0284c7;
    selection-color: #ffffff;
    padding: 4px;
}

/* Status Badges */
QLabel.badge {
    border-radius: 10px;
    padding: 3px 8px;
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 0.5px;
}

QLabel.badge-green {
    background-color: rgba(16, 185, 129, 0.15);
    color: #34d399;
    border: 1px solid #059669;
}

QLabel.badge-red {
    background-color: rgba(239, 68, 68, 0.15);
    color: #f87171;
    border: 1px solid #dc2626;
}

QLabel.badge-yellow {
    background-color: rgba(245, 158, 11, 0.15);
    color: #fbbf24;
    border: 1px solid #d97706;
}

QLabel.badge-blue {
    background-color: rgba(56, 189, 248, 0.15);
    color: #38bdf8;
    border: 1px solid #0284c7;
}

QLabel.badge-gray {
    background-color: rgba(100, 116, 139, 0.15);
    color: #94a3b8;
    border: 1px solid #475569;
}

QLabel.badge-purple {
    background-color: rgba(168, 85, 247, 0.15);
    color: #c084fc;
    border: 1px solid #9333ea;
}

/* Sliders */
QSlider::groove:horizontal {
    border: none;
    height: 8px;
    background: #162033;
    border-radius: 4px;
}

QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #38bdf8);
    border-radius: 4px;
}

QSlider::handle:horizontal {
    background: #38bdf8;
    border: 2px solid #ffffff;
    width: 18px;
    height: 18px;
    margin: -5px 0;
    border-radius: 9px;
}

QSlider::handle:horizontal:hover {
    background: #7dd3fc;
}

/* Progress Bars */
QProgressBar {
    background-color: #111928;
    border: 1px solid #1e293b;
    border-radius: 4px;
    text-align: center;
    color: #e2e8f0;
    font-size: 10px;
    font-weight: 700;
}

QProgressBar::chunk {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #38bdf8);
    border-radius: 3px;
}

/* Status Bar */
QStatusBar {
    background-color: #090e1a;
    border-top: 1px solid #1e293b;
    color: #94a3b8;
    font-size: 11px;
    padding: 4px;
}
"""
