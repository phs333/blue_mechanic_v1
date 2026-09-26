"""
Wheel Focus Event Filter.
Prevents QSpinBox, QDoubleSpinBox, QComboBox, and QSlider from changing values
on mouse wheel scroll unless the widget currently has focus (is explicitly selected).
When unfocused, smoothly scrolls the enclosing QScrollArea instead.
"""

from PyQt6.QtCore import QObject, QEvent
from PyQt6.QtWidgets import (
    QAbstractSpinBox, QComboBox, QSlider, QScrollArea, QWidget
)

class WheelFocusFilter(QObject):
    """
    Application-wide event filter that suppresses accidental wheel value changes
    on input controls (spin boxes, combos, sliders) unless the user has explicitly
    focused/selected that control.
    Forwards the scroll delta to the parent QScrollArea so page navigation is smooth.
    """
    def eventFilter(self, obj: QObject, event: QEvent) -> bool:
        if event.type() == QEvent.Type.Wheel:
            # Ascend to check if target or immediate container is an interactive input
            target = obj
            while target and not isinstance(target, (QAbstractSpinBox, QComboBox, QSlider)):
                if not isinstance(target, QWidget):
                    break
                target = target.parentWidget()

            if target and isinstance(target, (QAbstractSpinBox, QComboBox, QSlider)):
                if not target.hasFocus():
                    event.ignore()
                    # Propagate wheel scroll motion to enclosing QScrollArea
                    p = target.parentWidget()
                    while p:
                        if isinstance(p, QScrollArea):
                            sb = p.verticalScrollBar()
                            if sb and sb.isVisible():
                                step = max(25, sb.singleStep())
                                delta = event.angleDelta().y()
                                if delta != 0:
                                    sb.setValue(sb.value() - int(delta / 120 * step))
                            break
                        p = p.parentWidget()
                    return True  # Consume event to prevent unfocused value change

        return super().eventFilter(obj, event)
