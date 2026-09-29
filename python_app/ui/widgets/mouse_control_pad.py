"""
Controle manual por mouse (tela Automação & Testes).

- Botão esquerdo / direito: laser 1 / laser 2
    * duplo clique: liga em 100% (ou desliga, se estiver aceso)
    * clicar e segurar: fade crescente até 100% ou até soltar (o nível fica onde parou)
- Roda: eixo Z (para frente = sobe)
- Mover na horizontal: eixo C
- Mover na vertical: eixo A (para cima = positivo)

Com o modo ativo o cursor é capturado e recentralizado a cada movimento (deslocamento
relativo ilimitado). O mouse define um ALVO de posição.

- Serial direta (firmware com JOG): a cada envio só o deslocamento do mouse é repassado
  ("JOG C .. A .. Z .."); o próprio nó persegue o alvo com a velocidade/aceleração da NVS em
  segmentos de 10 ms, sem fila — responde em ~20-30 ms e para exatamente onde o mouse parou.
- Outros backends: a velocidade suavizada persegue o alvo (variação por segmento <= jerk) e
  cada trecho vira um MOVE_SYNC com a velocidade embutida; os primeiros segmentos saem
  juntos para o lookahead do motor "stream" encadear tudo sem parar.
Sair: botão do meio ou Espaço. Esc continua sendo o E-STOP global (e também sai).
"""

import math
import time

from PyQt6.QtCore import QObject, QPoint, QRectF, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QCursor, QFont, QPainter, QPen
from PyQt6.QtWidgets import QWidget

from python_app.core.protocol_defs import (
    calc_ca_degrees_per_step,
    calc_z_mm_per_step,
    laser_level_to_percent,
    percent_to_laser_level,
)
from python_app.ui.theme import PALETTE

LASER_BUTTONS = {
    Qt.MouseButton.LeftButton: 1,
    Qt.MouseButton.RightButton: 2,
}


AXES = ("C", "A", "Z")
DEFAULT_SPEED = {"C": 140.0, "A": 140.0, "Z": 50.0}
DEFAULT_ACCEL = {"C": 1800.0, "A": 1800.0, "Z": 1000.0}
DEFAULT_JERK = {"C": 15.0, "A": 15.0, "Z": 10.0}


class MouseJogController(QObject):
    """Lógica do controle por mouse, independente do widget (testável)."""

    status_changed = pyqtSignal()

    PREFILL_SEGMENTS = 3   # segmentos à frente na fila do nó: o lookahead precisa ver o próximo
    SMOOTH_TAU_S = 0.12    # constante de tempo com que a velocidade persegue o alvo do mouse
    MAX_LAG_S = 0.35       # alvo à frente da máquina além disso é descartado (não acumula atraso)

    def __init__(self, comm, state, parent=None, clock=time.monotonic):
        super().__init__(parent)
        self.comm = comm
        self.state = state
        self._clock = clock
        # Sensibilidade
        self.deg_per_px_c = 0.20
        self.deg_per_px_a = 0.20
        self.mm_per_notch_z = 1.0
        self.fade_ms = 1500
        self.send_interval_ms = 20
        self.hold_delay_ms = 250
        # Alvo ainda não percorrido (unidades físicas) e estado do perfil de velocidade
        self.pending = {axis: 0.0 for axis in AXES}
        self._reset_motion()
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
        self.pending = {axis: 0.0 for axis in AXES}
        self._reset_motion()
        self._send_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._send_timer.start(self.send_interval_ms)
        self.status_changed.emit()

    def deactivate(self) -> None:
        self.active = False
        self._send_timer.stop()
        self._fade_timer.stop()
        for idx in (1, 2):
            self._hold_timers[idx].stop()
            self.fading[idx] = False
        self.pending = {axis: 0.0 for axis in AXES}  # nada de movimento "atrasado" após sair
        self._reset_motion()
        self.status_changed.emit()

    def _reset_motion(self) -> None:
        self.velocity = {axis: 0.0 for axis in AXES}  # velocidade comandada (unid./s)
        self._carry = {axis: 0.0 for axis in AXES}    # fração de passo ainda não enviada
        self._buffer = []            # segmentos retidos até formar a folga inicial da fila
        self._streaming = False      # há segmentos na fila do nó (cadeia em andamento)
        self._stream_t0 = 0.0
        self._sent_s = 0.0           # soma das durações enviadas desde o início da cadeia
        self._last_tick = None
        self._last_emit = None

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

    def _step_size(self, axis: str) -> float:
        p = self.state.parameters
        if axis == "Z":
            return calc_z_mm_per_step(p.z_pulley_teeth, p.steps_per_rev[2], p.tmc_microsteps[2])
        idx = AXES.index(axis)
        return calc_ca_degrees_per_step(p.steps_per_rev[idx], p.tmc_microsteps[idx])

    def _limits(self, axis: str):
        """(velocidade máx., variação máx. de velocidade por segmento) do eixo.

        - Velocidade: a configurada, e no máximo o que ainda permite parar DENTRO de um
          segmento (o lookahead do nó só enxerga o próximo; acima disso ele reduz e o
          segmento passa a durar mais que o intervalo).
        - Variação por segmento <= jerk do eixo: a junção entre segmentos fica sem redução.
        """
        p = self.state.parameters
        idx = AXES.index(axis)

        def param(values, default):
            v = values[idx] if len(values) > idx else 0.0
            return v if v and v > 0 else default[axis]

        seg_s = self.send_interval_ms / 1000.0
        speed = param(p.speed, DEFAULT_SPEED)
        accel = param(p.accel, DEFAULT_ACCEL)
        jerk = param(getattr(p, "jerk", []), DEFAULT_JERK)
        return min(speed, accel * seg_s), max(min(jerk, accel * seg_s), 1e-3)

    def _clamp_backlog(self) -> None:
        if self._uses_node_jog():
            return  # o nó limita o alvo (limites de curso e atraso máximo)
        # Mouse mais rápido que a máquina: descarta o excesso em vez de acumular atraso
        for axis in AXES:
            limit = self._limits(axis)[0] * self.MAX_LAG_S
            self.pending[axis] = max(-limit, min(limit, self.pending[axis]))

    def _advance_velocity(self, axis: str, dt: float) -> float:
        """Atualiza a velocidade do eixo em direção ao alvo e retorna o deslocamento do tick."""
        vmax, dv_max = self._limits(axis)
        target = self.pending[axis] - self._carry[axis]  # o que falta além do já percorrido
        dist = abs(target)
        # Persegue o alvo com constante de tempo e pousa nele: nunca mais rápido do que dá
        # para frear até o alvo com a variação de velocidade permitida por segmento.
        brake = dv_max / dt
        v_des = math.copysign(min(dist / self.SMOOTH_TAU_S, math.sqrt(2.0 * brake * dist), vmax), target)
        v = self.velocity[axis]
        v += max(-dv_max, min(dv_max, v_des - v))
        d = v * dt
        if d * target > 0 and abs(d) > dist:  # chegaria além do alvo: pousa exatamente nele
            d = target
            v = d / dt
        if dist < 1e-9 and abs(v) <= dv_max:
            v, d = 0.0, 0.0
        self.velocity[axis] = v
        return d

    def _uses_node_jog(self) -> bool:
        return bool(getattr(self.comm, "supports_jog", False))

    def tick(self, now=None) -> bool:
        """Um intervalo de envio. Retorna True se algo foi transmitido."""
        if not self.active:
            return False
        if self._uses_node_jog():
            # Jog no nó: só repassa o deslocamento do mouse. O firmware persegue o alvo com
            # a velocidade/aceleração da NVS, a cada 10 ms, sem fila — acompanha o mouse.
            d = {axis: self.pending[axis] for axis in AXES}
            if not any(abs(v) > 1e-9 for v in d.values()):
                return False
            self.pending = {axis: 0.0 for axis in AXES}
            return bool(self.comm.jog(d["C"], d["A"], d["Z"]))
        return self._tick_stream(now)

    def _tick_stream(self, now=None) -> bool:
        """Backends sem JOG (CAN/Teensy): segmentos MOVE_SYNC com velocidade embutida."""
        now = self._clock() if now is None else now
        interval = self.send_interval_ms / 1000.0
        dt = interval if self._last_tick is None else min(max(now - self._last_tick, 0.25 * interval), 3 * interval)
        self._last_tick = now

        # A fila do nó esvaziou (fim do movimento anterior): o próximo precisa de nova folga
        if self._streaming and self._sent_s - (now - self._stream_t0) < -0.5 * interval:
            self._streaming = False

        steps = {}
        for axis in AXES:
            self._carry[axis] += self._advance_velocity(axis, dt)
            size = self._step_size(axis)
            steps[axis] = int(round(self._carry[axis] / size))
            self._carry[axis] -= steps[axis] * size
            self.pending[axis] -= steps[axis] * size

        if not any(steps.values()):
            if self._buffer and not any(self.velocity.values()):
                return self._flush(now)  # movimento curto: não segura o que já foi gerado
            return False

        # Duração = tempo desde o segmento anterior: a máquina executa no mesmo ritmo do mouse
        # (em baixa velocidade um segmento de 1 passo pode cobrir vários intervalos)
        idle = not self._streaming and not self._buffer
        duration = dt if (idle or self._last_emit is None) else min(max(now - self._last_emit, dt), 0.5)
        self._last_emit = now
        segment = (steps, duration)
        if self._streaming:
            self._send(segment)
            return True
        self._buffer.append(segment)
        if sum(d for _, d in self._buffer) >= self.PREFILL_SEGMENTS * interval - 1e-9:
            return self._flush(now)
        return False

    def _flush(self, now: float) -> bool:
        for segment in self._buffer:
            self._send(segment)
        if not self._streaming:
            self._streaming = True
            self._stream_t0 = now
            self._sent_s = sum(duration for _, duration in self._buffer)
        self._buffer = []
        return True

    def _send(self, segment) -> None:
        steps, duration = segment
        speeds = {}
        for axis in AXES:
            # Velocidade de cada eixo = trecho / duração: todos terminam juntos, no tempo certo
            speeds[axis] = abs(steps[axis]) * self._step_size(axis) / duration if steps[axis] else None
        if self._streaming:
            self._sent_s += duration
        self.comm.move_sync(steps_c=steps["C"], steps_a=steps["A"], steps_z=steps["Z"],
                            speed_c=speeds["C"], speed_a=speeds["A"], speed_z=speeds["Z"])

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
