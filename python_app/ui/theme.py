"""
Tokens de cor e classes semânticas da interface.

Em vez de ~250 setStyleSheet() com cores fixas espalhados pelas telas, os widgets
recebem classes (``add_class(label, "field-label")``) e o QSS central define a
aparência a partir desta paleta. Mudar uma cor aqui muda o app inteiro.
"""

from PyQt6.QtWidgets import QWidget

PALETTE = {
    # Superfícies
    "bg": "#080c14",
    "surface": "#0f172a",
    "input_bg": "#0b1220",
    "border": "#1e293b",
    "input_border": "#1f2e47",
    # Texto
    "text": "#e2e8f0",
    "text_strong": "#f1f5f9",
    "text_soft": "#cbd5e1",
    "text_muted": "#94a3b8",
    "text_dim": "#64748b",
    # Estados / acentos
    "accent": "#38bdf8",
    "accent_strong": "#0284c7",
    "success": "#34d399",
    "warning": "#f59e0b",
    "danger": "#f87171",
}


def _rules() -> str:
    p = PALETTE
    return f"""
/* Rótulos e caixas de seleção herdavam o fundo da regra global QWidget e apareciam como
   retângulos escuros dentro dos cards. Classes com fundo próprio (badges) continuam valendo. */
QLabel, QCheckBox, QRadioButton {{ background-color: transparent; }}

/* ===== Classes semânticas (ui/theme.py) ===== */
QLabel.table-header {{ color: {p['text_muted']}; font-weight: 700; font-size: 12px; }}
QLabel.row-label {{ color: {p['text']}; font-size: 12px; font-weight: 600; }}
QLabel.field-label {{ color: {p['text_muted']}; font-size: 12px; font-weight: 600; }}
QLabel.caption {{ color: {p['text_muted']}; font-size: 11px; }}
QLabel.hint {{ color: {p['text_dim']}; font-size: 11px; }}
QLabel.hint-strong {{ color: {p['text_muted']}; font-size: 11px; font-weight: 600; }}
QLabel.muted {{ color: {p['text_dim']}; font-size: 12px; }}
QLabel.accent {{ color: {p['accent']}; font-weight: 700; font-size: 12px; }}
QLabel.card-title {{ color: {p['accent']}; font-weight: 700; font-size: 13.5px; }}
QLabel.status-item {{ color: {p['text_dim']}; font-size: 11px; margin-right: 15px; }}
QCheckBox.check-label {{ color: {p['text_soft']}; font-size: 12px; }}

QSpinBox.compact-input, QDoubleSpinBox.compact-input, QLineEdit.compact-input {{
    font-size: 13px; font-weight: 500; padding: 2px 6px;
    background-color: {p['input_bg']}; color: {p['text_strong']};
    border: 1px solid {p['input_border']}; border-radius: 5px;
}}
QSpinBox.compact-input:focus, QDoubleSpinBox.compact-input:focus, QLineEdit.compact-input:focus {{
    border-color: {p['accent']};
}}
QComboBox.compact-combo {{
    font-size: 13px; font-weight: 500; padding: 2px 24px 2px 8px;
    background-color: {p['input_bg']}; color: {p['text_strong']};
    border: 1px solid {p['input_border']}; border-radius: 5px;
}}
QComboBox.compact-combo:focus {{ border-color: {p['accent']}; }}
"""


THEME_RULES_QSS = _rules()


def add_class(widget: QWidget, *names: str) -> QWidget:
    """Acrescenta classes QSS ao widget (sem apagar as que já tem) e reaplica o estilo."""
    current = (widget.property("class") or "").split()
    for name in names:
        if name not in current:
            current.append(name)
    widget.setProperty("class", " ".join(current))
    style = widget.style()
    if style is not None:
        style.unpolish(widget)
        style.polish(widget)
    return widget
