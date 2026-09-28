"""
Controle manual por mouse (tela Automação & Testes).

- Botão esquerdo / direito: laser 1 / laser 2
    * duplo clique: liga em 100% (ou desliga, se estiver aceso)
    * clicar e segurar: fade crescente até 100% ou até soltar (o nível fica onde parou)
- Roda: eixo Z (para frente = sobe)
- Mover na horizontal: eixo C
- Mover na vertical: eixo A (para cima = positivo)

Com o modo ativo o cursor é capturado e recentralizado a cada movimento (deslocamento
relativo ilimitado). Os deslocamentos são acumulados e enviados como MOVE_SYNC a
intervalos fixos, limitados ao que a máquina percorre nesse intervalo — o motor
"stream" do firmware encadeia esses lotes sem parar entre eles.
Sair: botão do meio ou Espaço. Esc continua sendo o E-STOP global (e também sai).
"""

from PyQt6.QtCore import QObject, QPoint, QRectF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QCursor, QFont, QPainter, QPen
from PyQt6.QtWidgets import QWidget

from python_app.core.protocol_defs import (
    calc_ca_degrees_per_step,
    calc_ca_steps_for_degrees,
    calc_z_mm_per_step,
    calc_z_steps_for_mm,
    laser_level_to_percent,
    percent_to_laser_level,
)
from python_app.ui.theme import PALETTE

LASER_BUTTONS = {
    Qt.MouseButton.LeftButton: 1,
    Qt.MouseButton.RightButton: 2,
}


class MouseJogController(QObject):
    """Lógica do controle por mouse, independente do widget (testável)."""

    status_changed = pyqtSignal()

    def __init__(self, comm, state, parent=None):
        super().__init__(parent)
        self.comm = comm
        self.state = state
        # Sensibilidade
        self.deg_per_px_c = 0.20
        self.deg_per_px_a = 0.20
        self.mm_per_notch_z = 1.0
        self.fade_ms = 1500
        self.send_interval_ms = 80
        self.hold_delay_ms = 250
        # Pendências de movimento (unidades físicas)
        self.pending = {"C": 0.0, "A": 0.0, "Z": 0.0}
        self.active = False
        # Lasers: nível em % (0..100), se está em fade
        self.laser_pct = {1: 0.0, 2: 0.0}
        self.fading = {1: False, 2: False}
        self._hold_timers = {}
        for idx in (1, 2):
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.timeout.connect(lambda i=idx: self._start_fade(i))
            self._hold_timers[idx] = timer
        self._fade_timer = QTimer(self)
        self._fade_timer.setInterval(40)
        self._fade_timer.timeout.connect(self.fade_tick)
        self._send_timer = QTimer(self)
        self._send_timer.timeout.connect(self.tick)

    # --- compatibilidade de atributos legados ---------------------------------------
    @property
    def mm_per_px_z(self) -> float:
        return self.mm_per_notch_z

    @mm_per_px_z.setter
    def mm_per_px_z(self, val: float) -> None:
        self.mm_per_notch_z = float(val)

    @property
    def deg_per_notch_a(self) -> float:
        return self.deg_per_px_a

    @deg_per_notch_a.setter
    def deg_per_notch_a(self, val: float) -> None:
        self.deg_per_px_a = float(val)

    # --- ciclo de vida ---------------------------------------------------------------
    def activate(self) -> None:
        self.active = True
        telemetry = self.state.telemetry
        self.laser_pct[1] = float(laser_level_to_percent(telemetry.laser1_level))
        self.laser_pct[2] = float(laser_level_to_percent(telemetry.laser2_level))
        self.pending = {"C": 0.0, "A": 0.0, "Z": 0.0}
        self._send_timer.start(self.send_interval_ms)
        self.status_changed.emit()

    def deactivate(self) -> None:
        self.active = False
        self._send_timer.stop()
        self._fade_timer.stop()
        for idx in (1, 2):
            self._hold_timers[idx].stop()
            self.fading[idx] = False
        self.pending = {"C": 0.0, "A": 0.0, "Z": 0.0}  # nada de movimento "atrasado" após sair
        self.status_changed.emit()

    # --- movimento -------------------------------------------------------------------
    def on_move(self, dx: float, dy: float) -> None:
        if not self.active:
            return
        self.pending["C"] += dx * self.deg_per_px_c
        self.pending["A"] += -dy * self.deg_per_px_a  # tela: y cresce para baixo, mouse para cima = A positivo
        self._clamp_backlog()

    def on_wheel(self, notches: float) -> None:
        if not self.active:
            return
        self.pending["Z"] += notches * self.mm_per_notch_z
        self._clamp_backlog()

    def _max_per_tick(self, axis: str) -> float:
        """Distância que o eixo percorre num intervalo de envio na velocidade configurada."""
        idx = "CAZ".index(axis)
        speed = self.state.parameters.speed[idx] if len(self.state.parameters.speed) > idx else 0.0
        speed = speed if speed > 0 else (140.0 if axis != "Z" else 50.0)
        return speed * self.send_interval_ms / 1000.0

    def _clamp_backlog(self) -> None:
        # Mouse mais rápido que a máquina: descarta o excesso em vez de acumular atraso
        for axis in self.pending:
            limit = 2.0 * self._max_per_tick(axis)
            self.pending[axis] = max(-limit, min(limit, self.pending[axis]))

    def tick(self) -> bool:
        """Envia um lote de movimento com o que estiver pendente. Retorna True se enviou."""
        if not self.active:
            return False
        p = self.state.parameters
        chunk = {}
        for axis in self.pending:
            limit = self._max_per_tick(axis)
            chunk[axis] = max(-limit, min(limit, self.pending[axis]))

        steps_c = calc_ca_steps_for_degrees(chunk["C"], p.steps_per_rev[0], p.tmc_microsteps[0])
        steps_a = calc_ca_steps_for_degrees(chunk["A"], p.steps_per_rev[1], p.tmc_microsteps[1])
        steps_z = calc_z_steps_for_mm(chunk["Z"], p.z_pulley_teeth, p.steps_per_rev[2], p.tmc_microsteps[2])
        if steps_c == 0 and steps_a == 0 and steps_z == 0:
            return False

        # Desconta só o que virou passo inteiro (o resto fracionário continua pendente)
        self.pending["C"] -= steps_c * calc_ca_degrees_per_step(p.steps_per_rev[0], p.tmc_microsteps[0])
        self.pending["A"] -= steps_a * calc_ca_degrees_per_step(p.steps_per_rev[1], p.tmc_microsteps[1])
        self.pending["Z"] -= steps_z * calc_z_mm_per_step(p.z_pulley_teeth, p.steps_per_rev[2], p.tmc_microsteps[2])
        self.comm.move_sync(steps_c=steps_c, steps_a=steps_a, steps_z=steps_z)
        return True

    # --- lasers ----------------------------------------------------------------------
    def on_press(self, laser: int) -> None:
        if self.active:
            self._hold_timers[laser].start(self.hold_delay_ms)

    def on_release(self, laser: int) -> None:
        self._hold_timers[laser].stop()
        if self.fading[laser]:
            self.fading[laser] = False  # fica no nível atingido
            if not any(self.fading.values()):
                self._fade_timer.stop()
            self.status_changed.emit()

    def on_double_click(self, laser: int) -> None:
        if not self.active:
            return
        self._hold_timers[laser].stop()
        self.fading[laser] = False
        self._set_laser(laser, 0.0 if self.laser_pct[laser] > 0.0 else 100.0)

    def _start_fade(self, laser: int) -> None:
        if not self.active or self.laser_pct[laser] >= 100.0:
            return
        self.fading[laser] = True
        if not self._fade_timer.isActive():
            self._fade_timer.start()
        self.status_changed.emit()

    def fade_tick(self) -> None:
        step = 100.0 * self._fade_timer.interval() / max(1, self.fade_ms)
        for laser in (1, 2):
            if not self.fading[laser]:
                continue
            target = min(100.0, self.laser_pct[laser] + step)
            self._set_laser(laser, target)
            if target >= 100.0:
                self.fading[laser] = False
        if not any(self.fading.values()):
            self._fade_timer.stop()

    def _set_laser(self, laser: int, percent: float) -> None:
        previous = int(self.laser_pct[laser])
        self.laser_pct[laser] = max(0.0, min(100.0, percent))
        # Só transmite quando o % inteiro muda (o fade não inunda a serial/CAN)
        if int(self.laser_pct[laser]) != previous or percent in (0.0, 100.0):
            self.comm.set_laser(laser, percent_to_laser_level(int(self.laser_pct[laser])))
        self.status_changed.emit()


class MouseControlPad(QWidget):
    """Área de captura do mouse com HUD. Ative pelo botão da tela ou clicando na área."""

    active_changed = pyqtSignal(bool)

    def __init__(self, controller: MouseJogController, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.controller.status_changed.connect(self.update)
        self.setMinimumSize(420, 300)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self._active = False
        self._wheel_accum = 0

    def is_active(self) -> bool:
        return self._active

    def activate(self) -> None:
        if self._active or not self.controller.comm.is_connected:
            return
        self._active = True
        self.controller.activate()
        self.setFocus(Qt.FocusReason.OtherFocusReason)
        self.grabMouse()
        self.setCursor(Qt.CursorShape.BlankCursor)
        self._recenter()
        self.active_changed.emit(True)
        self.update()

    def deactivate(self) -> None:
        if not self._active:
            return
        self._active = False
        self.controller.deactivate()
        self.releaseMouse()
        self.unsetCursor()
        self.active_changed.emit(False)
        self.update()

    def _center(self) -> QPoint:
        return self.rect().center()

    def _recenter(self) -> None:
        QCursor.setPos(self.mapToGlobal(self._center()))

    # --- eventos ---------------------------------------------------------------------
    def mouseMoveEvent(self, event):
        if not self._active:
            return
        delta = event.position().toPoint() - self._center()
        if delta.x() or delta.y():
            self.controller.on_move(delta.x(), delta.y())
            self._recenter()  # o evento gerado pelo recentro chega com delta zero

    def mousePressEvent(self, event):
        if not self._active:
            if event.button() == Qt.MouseButton.LeftButton:
                self.activate()
            return
        if event.button() == Qt.MouseButton.MiddleButton:
            self.deactivate()
        elif event.button() in LASER_BUTTONS:
            self.controller.on_press(LASER_BUTTONS[event.button()])

    def mouseReleaseEvent(self, event):
        if self._active and event.button() in LASER_BUTTONS:
            self.controller.on_release(LASER_BUTTONS[event.button()])

    def mouseDoubleClickEvent(self, event):
        if self._active and event.button() in LASER_BUTTONS:
            self.controller.on_double_click(LASER_BUTTONS[event.button()])

    def wheelEvent(self, event):
        if not self._active:
            return
        # 120 = um "clique" da roda; mouses de alta resolução mandam frações
        self._wheel_accum += event.angleDelta().y()
        notches = self._wheel_accum / 120.0
        if abs(notches) >= 0.25:
            self.controller.on_wheel(notches)
            self._wheel_accum = 0
        event.accept()

    def keyPressEvent(self, event):
        if self._active and event.key() == Qt.Key.Key_Space:
            self.deactivate()
            return
        super().keyPressEvent(event)

    def focusOutEvent(self, event):
        # Trocar de janela (Alt+Tab) nunca pode deixar o mouse preso controlando a máquina
        self.deactivate()
        super().focusOutEvent(event)

    def hideEvent(self, event):
        self.deactivate()
        super().hideEvent(event)

    # --- desenho ---------------------------------------------------------------------
    def paintEvent(self, event):
        p = PALETTE
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect()).adjusted(1, 1, -1, -1)
        border = QColor(p["accent"] if self._active else p["input_border"])
        painter.setPen(QPen(border, 2 if self._active else 1))
        painter.setBrush(QColor(p["input_bg"]))
        painter.drawRoundedRect(r, 10, 10)

        c = self.rect().center()
        painter.setPen(QPen(QColor(p["border"]), 1, Qt.PenStyle.DashLine))
        painter.drawLine(c.x(), int(r.top()) + 12, c.x(), int(r.bottom()) - 12)
        painter.drawLine(int(r.left()) + 12, c.y(), int(r.right()) - 12, c.y())

        painter.setPen(QColor(p["text"]))
        painter.setFont(QFont("Segoe UI", 13, QFont.Weight.Bold))
        title = "CONTROLE POR MOUSE ATIVO" if self._active else "Clique aqui para ativar o controle por mouse"
        painter.drawText(r.adjusted(0, 18, 0, 0), Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, title)

        painter.setFont(QFont("Segoe UI", 10))
        painter.setPen(QColor(p["text_muted"]))
        legend = ("Esq.: laser 1   Dir.: laser 2   (duplo clique liga/desliga · segurar = fade)\n"
                  "Mover ↔ eixo C   Mover ↕ eixo A   Roda: eixo Z\n"
                  "Sair: botão do meio ou Espaço   ·   Esc = E-STOP")
        painter.drawText(r.adjusted(0, 48, 0, 0), Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop, legend)

        # Barras dos lasers
        for i, laser in enumerate((1, 2)):
            bar = QRectF(r.left() + 24, r.bottom() - 70 + i * 26, r.width() - 48, 16)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QColor(p["border"]))
            painter.drawRoundedRect(bar, 6, 6)
            pct = self.controller.laser_pct[laser]
            if pct > 0:
                fill = QRectF(bar.left(), bar.top(), bar.width() * pct / 100.0, bar.height())
                painter.setBrush(QColor(p["warning"] if self.controller.fading[laser] else p["danger"]))
                painter.drawRoundedRect(fill, 6, 6)
            painter.setPen(QColor(p["text_strong"]))
            painter.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
            painter.drawText(bar.adjusted(8, 0, -8, 0), Qt.AlignmentFlag.AlignVCenter,
                             f"LASER {laser}: {int(pct)}%" + ("  (fade)" if self.controller.fading[laser] else ""))
        painter.end()
