"""
Modern Cyber-Industrial Theme Styling & Stylesheet for PyQt6 UI.
Clean dark palette, neon cyan/blue/emerald accents, glowing badges,
refined typography, tactile buttons, and sleek card surfaces.
"""

from python_app.ui.theme import THEME_RULES_QSS

DARK_THEME_QSS = """
/* Global Base */
QWidget {
    background-color: #080c14;
    color: #e2e8f0;
    font-family: 'Segoe UI', 'Roboto', 'Inter', -apple-system, sans-serif;
    font-size: 14px;
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

QPushButton.nav-btn:disabled {
    background-color: transparent;
    color: #334155;
    border: 1px solid transparent;
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
    border-radius: 10px;
    padding: 6px 10px;
}

QFrame.metric-card:hover {
    border-color: #38bdf8;
}

/* Multi-Node Cards & Kinematic Tiles */
QFrame.node-card-online {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #101c30, stop:1 #0b1322);
    border: 1px solid #1e3354;
    border-radius: 12px;
}

QFrame.node-card-online:hover {
    border-color: #38bdf8;
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #13233d, stop:1 #0d1728);
}

QFrame.node-card-offline {
    background: #080d17;
    border: 1px solid #141f30;
    border-radius: 12px;
}

QFrame.node-card-offline:hover {
    border-color: #24354f;
}

QFrame.node-tile {
    background-color: rgba(15, 23, 42, 0.75);
    border: 1px solid rgba(56, 189, 248, 0.12);
    border-radius: 8px;
    padding: 6px 4px;
}

QFrame.node-tile-offline {
    background-color: rgba(11, 16, 26, 0.5);
    border: 1px solid rgba(255, 255, 255, 0.04);
    border-radius: 8px;
    padding: 6px 4px;
}

/* Typography & Section Titles */
QLabel.section-title {
    color: #38bdf8;
    font-size: 16px;
    font-weight: 800;
    letter-spacing: 0.4px;
}

QLabel.card-title {
    color: #94a3b8;
    font-size: 11px;
    font-weight: 700;
    text-transform: uppercase;
    letter-spacing: 0.6px;
}

QLabel.metric-value {
    color: #f8fafc;
    font-size: 19px;
    font-weight: 800;
    font-family: 'Consolas', 'JetBrains Mono', 'Segoe UI', monospace;
}

QLabel.metric-unit {
    color: #64748b;
    font-size: 12px;
    font-weight: 600;
}

/* Standard Buttons */
QPushButton {
    background-color: #162033;
    color: #f1f5f9;
    border: 1px solid #23334d;
    border-radius: 8px;
    padding: 8px 16px;
    font-size: 13px;
    font-weight: 600;
    min-height: 24px;
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

/* E-STOP: sempre visível na barra superior, maior e com contraste máximo */
QPushButton#estopButton {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #ef4444, stop:1 #b91c1c);
    border: 2px solid #fecaca;
    border-radius: 8px;
    color: #ffffff;
    font-size: 14px;
    font-weight: 800;
    letter-spacing: 1px;
    padding: 6px 18px;
    min-height: 26px;
}

QPushButton#estopButton:hover {
    background: #dc2626;
    border-color: #ffffff;
}

QPushButton#estopButton:pressed {
    background: #7f1d1d;
}

QPushButton#estopButton:disabled {
    background: #3f1d1d;
    border-color: #7f1d1d;
    color: #fca5a5;
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
    font-size: 12px;
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
    font-size: 12px;
    font-weight: 700;
}

/* Input Fields & Combos */
QLineEdit, QSpinBox, QDoubleSpinBox {
    background-color: #0b1220;
    color: #f1f5f9;
    border: 1px solid #1f2e47;
    border-radius: 6px;
    padding: 4px 8px;
    font-size: 13px;
}

QComboBox {
    background-color: #0b1220;
    color: #f1f5f9;
    border: 1px solid #1f2e47;
    border-radius: 6px;
    padding: 4px 24px 4px 8px;
    font-size: 13px;
}

QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
    border: 1px solid #38bdf8;
    background-color: #0e172a;
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
    border-radius: 6px;
    padding: 2px 6px;
    font-size: 11px;
    font-weight: 700;
    letter-spacing: 0.4px;
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

QLabel.badge-amber {
    background-color: rgba(217, 119, 6, 0.18);
    color: #f59e0b;
    border: 1px solid #b45309;
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
    font-size: 11px;
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
    font-size: 12px;
    padding: 4px;
}

/* ==========================================================================
   JOG PAD & MOTION CONTROLS
   ========================================================================== */

QFrame.jog-module {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0f172a, stop:1 #0a1120);
    border: 1px solid #1e2e4a;
    border-radius: 12px;
    padding: 12px;
}

QFrame.jog-module:hover {
    border-color: #2b4570;
}

QLabel.jog-module-title {
    color: #38bdf8;
    font-size: 13px;
    font-weight: 800;
    letter-spacing: 0.5px;
    padding-bottom: 2px;
}

QLabel.jog-module-title-z {
    color: #34d399;
    font-size: 13px;
    font-weight: 800;
    letter-spacing: 0.5px;
    padding-bottom: 2px;
}

QLabel.jog-module-sub {
    color: #64748b;
    font-size: 11px;
    font-weight: 500;
}

/* Jog Directional Buttons */
QPushButton.jog-btn {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #16243d, stop:1 #0e1728);
    border: 1px solid #233758;
    border-radius: 10px;
    color: #e2e8f0;
    font-weight: 700;
    font-size: 13px;
    padding: 10px 14px;
    min-height: 40px;
}

QPushButton.jog-btn:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #22375d, stop:1 #15243e);
    border-color: #38bdf8;
    color: #38bdf8;
}

QPushButton.jog-btn:pressed {
    background: #09101d;
    border-color: #0284c7;
    color: #ffffff;
}

QPushButton.jog-btn-z {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #12282e, stop:1 #0c1a1e);
    border: 1px solid #1c4543;
    border-radius: 10px;
    color: #e2e8f0;
    font-weight: 700;
    font-size: 12px;
    padding: 10px 14px;
    min-height: 38px;
}

QPushButton.jog-btn-z:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #183e3f, stop:1 #0f2b2b);
    border-color: #10b981;
    color: #34d399;
}

QPushButton.jog-btn-z:pressed {
    background: #081717;
    border-color: #059669;
    color: #ffffff;
}

QPushButton.jog-btn-home {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0284c7, stop:1 #0369a1);
    border: 1px solid #38bdf8;
    border-radius: 10px;
    color: #ffffff;
    font-weight: 800;
    font-size: 12px;
    padding: 10px 14px;
    min-height: 38px;
}

QPushButton.jog-btn-home:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #0ea5e9, stop:1 #0284c7);
    border-color: #7dd3fc;
}

QPushButton.jog-btn-home-z {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #059669, stop:1 #047857);
    border: 1px solid #34d399;
    border-radius: 10px;
    color: #ffffff;
    font-weight: 800;
    font-size: 12px;
    padding: 10px 14px;
    min-height: 38px;
}

QPushButton.jog-btn-home-z:hover {
    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #10b981, stop:1 #059669);
    border-color: #6ee7b7;
}

QPushButton.jog-btn-action {
    background-color: #111a2c;
    border: 1px solid #1f304d;
    border-radius: 8px;
    color: #94a3b8;
    font-weight: 600;
    font-size: 11px;
    padding: 7px 10px;
    min-height: 28px;
}

QPushButton.jog-btn-action:hover {
    background-color: #1a2a47;
    border-color: #38bdf8;
    color: #e2e8f0;
}

QPushButton.jog-btn-zero {
    background-color: #171822;
    border: 1px solid #362e4a;
    border-radius: 8px;
    color: #f59e0b;
    font-weight: 600;
    font-size: 11px;
    padding: 7px 10px;
    min-height: 28px;
}

QPushButton.jog-btn-zero:hover {
    background-color: #242236;
    border-color: #f59e0b;
    color: #fbbf24;
}

/* Resolution Step Pills */
QPushButton.step-pill {
    background-color: #0d1527;
    border: 1px solid #1e293b;
    border-radius: 10px;
    color: #94a3b8;
    font-weight: 600;
    font-size: 11px;
    padding: 3px 10px;
    min-height: 22px;
}

QPushButton.step-pill:hover {
    background-color: #17233d;
    border-color: #38bdf8;
    color: #f1f5f9;
}

QPushButton.step-pill-active {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #0284c7, stop:1 #0ea5e9);
    border: 1px solid #38bdf8;
    border-radius: 10px;
    color: #ffffff;
    font-weight: 700;
    font-size: 11px;
    padding: 3px 10px;
    min-height: 22px;
}

QPushButton.step-pill-active-z {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #059669, stop:1 #10b981);
    border: 1px solid #34d399;
    border-radius: 10px;
    color: #ffffff;
    font-weight: 700;
    font-size: 11px;
    padding: 3px 10px;
    min-height: 22px;
}
"""

# Classes semânticas geradas a partir da paleta (ui/theme.py)
DARK_THEME_QSS += THEME_RULES_QSS
