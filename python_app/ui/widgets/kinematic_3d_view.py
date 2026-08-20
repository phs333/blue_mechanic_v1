"""
Interactive 3D Animated Kinematic Visualization Widget.
Renders the real-time physical system in 3D perspective:
- Linear Actuator Z (Vertical Travel)
- Rotative Base C (Yaw Rotation)
- Laser Pivot A (Pitch/Roll Rotation)
- Left and Right Lasers (Collinear 180° opposite beams with dynamic glow)
"""

import math
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QFrame
from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import (
    QPainter, QColor, QBrush, QPen, QPolygonF, QRadialGradient, QLinearGradient, QFont
)
from python_app.core.state_model import DeviceState, HardwareTelemetry

class Kinematic3DView(QFrame):
    def __init__(self, state: DeviceState, parent=None):
        super().__init__(parent)
        self.state = state
        self.setMinimumSize(180, 160)
        self.setMaximumHeight(200)
        self.setProperty("class", "metric-card")
        self.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #0a0f1d, stop:1 #070b14);
                border: 1px solid #1e293b;
                border-radius: 8px;
            }
        """)
        
        # Camera orbit angles (degrees)
        self.rot_x = 22.0  # Pitch
        self.rot_y = -35.0 # Yaw
        self.zoom = 1.0
        
        # Smoothed kinematics target & current values
        self.cur_c_deg = 0.0
        self.cur_a_deg = 0.0
        self.cur_z_pct = 0.0
        self.laser1_active = 0
        self.laser2_active = 0
        
        self.target_c_deg = 0.0
        self.target_a_deg = 0.0
        self.target_z_pct = 0.0
        
        # Mouse interaction state
        self.last_mouse_pos = None
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self.setToolTip("Visualização 3D em Tempo Real (Arraste para girar a câmera)")
        
        # Animation loop (30 FPS)
        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self._on_anim_frame)
        self.anim_timer.start(33)
        
        self.state.telemetry_updated.connect(self._on_telemetry)

    def _on_telemetry(self, t: HardwareTelemetry):
        self.target_c_deg = t.pos_c_deg
        self.target_a_deg = t.pos_a_deg
        self.target_z_pct = t.z_progress_pct
        self.laser1_active = t.laser1_level
        self.laser2_active = t.laser2_level

    def _on_anim_frame(self):
        # Exponential smoothing interpolation (LERP)
        k = 0.22
        self.cur_c_deg += (self.target_c_deg - self.cur_c_deg) * k
        self.cur_a_deg += (self.target_a_deg - self.cur_a_deg) * k
        self.cur_z_pct += (self.target_z_pct - self.cur_z_pct) * k
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.last_mouse_pos = event.pos()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos is not None:
            dx = event.pos().x() - self.last_mouse_pos.x()
            dy = event.pos().y() - self.last_mouse_pos.y()
            self.rot_y += dx * 0.7
            self.rot_x = max(5.0, min(80.0, self.rot_x + dy * 0.7))
            self.last_mouse_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        self.last_mouse_pos = None
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def _project(self, x, y, z, cx, cy, scale):
        # Rotate around Y (Yaw)
        rad_y = math.radians(self.rot_y)
        cos_y, sin_y = math.cos(rad_y), math.sin(rad_y)
        x1 = x * cos_y + z * sin_y
        y1 = y
        z1 = -x * sin_y + z * cos_y
        
        # Rotate around X (Pitch)
        rad_x = math.radians(self.rot_x)
        cos_x, sin_x = math.cos(rad_x), math.sin(rad_x)
        x2 = x1
        y2 = y1 * cos_x - z1 * sin_x
        z2 = y1 * sin_x + z1 * cos_x
        
        # Perspective projection
        distance = 320.0
        factor = distance / (distance + z2) if (distance + z2) > 10.0 else 1.0
        px = cx + x2 * scale * factor
        py = cy - y2 * scale * factor
        return QPointF(px, py), z2

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h / 2.0 + 10.0
        scale = min(w, h) / 130.0 * self.zoom
        
        # 1. Base Ground Grid (Z Stage)
        grid_pen = QPen(QColor(30, 41, 59, 140), 1)
        painter.setPen(grid_pen)
        for i in range(-2, 3):
            p1, _ = self._project(i * 12, -35, -24, cx, cy, scale)
            p2, _ = self._project(i * 12, -35, 24, cx, cy, scale)
            painter.drawLine(p1, p2)
            p3, _ = self._project(-24, -35, i * 12, cx, cy, scale)
            p4, _ = self._project(24, -35, i * 12, cx, cy, scale)
            painter.drawLine(p3, p4)

        # 2. Linear Actuator Z Column & Belt
        col_pen = QPen(QColor(51, 65, 85), 2)
        painter.setPen(col_pen)
        pt_bot_l, _ = self._project(-22, -35, 0, cx, cy, scale)
        pt_top_l, _ = self._project(-22, 35, 0, cx, cy, scale)
        pt_bot_r, _ = self._project(-16, -35, 0, cx, cy, scale)
        pt_top_r, _ = self._project(-16, 35, 0, cx, cy, scale)
        painter.drawLine(pt_bot_l, pt_top_l)
        painter.drawLine(pt_bot_r, pt_top_r)
        
        # Vertical carriage position based on Z
        z_y = -25.0 + (self.cur_z_pct / 100.0) * 50.0 # -25 to +25
        
        # Z Carriage bracket
        car_p1, _ = self._project(-26, z_y - 4, -8, cx, cy, scale)
        car_p2, _ = self._project(-12, z_y - 4, -8, cx, cy, scale)
        car_p3, _ = self._project(-12, z_y + 4, 8, cx, cy, scale)
        car_p4, _ = self._project(-26, z_y + 4, 8, cx, cy, scale)
        painter.setBrush(QColor(30, 58, 138, 200))
        painter.setPen(QColor(56, 189, 248))
        painter.drawPolygon(QPolygonF([car_p1, car_p2, car_p3, car_p4]))

        # Arm connecting Z carriage to Base C
        arm_p1, _ = self._project(-12, z_y, 0, cx, cy, scale)
        arm_p2, _ = self._project(0, z_y, 0, cx, cy, scale)
        painter.setPen(QPen(QColor(71, 85, 105), 3))
        painter.drawLine(arm_p1, arm_p2)

        # 3. Base Rotativa C (Disk rotating around Y axis at z_y)
        c_rad = math.radians(self.cur_c_deg)
        cos_c, sin_c = math.cos(c_rad), math.sin(c_rad)
        
        c_pts = []
        num_seg = 16
        base_radius = 16.0
        for i in range(num_seg):
            ang = (i / num_seg) * 2.0 * math.pi
            bx = base_radius * math.cos(ang)
            bz = base_radius * math.sin(ang)
            # Rotate by C angle
            rx = bx * cos_c - bz * sin_c
            rz = bx * sin_c + bz * cos_c
            pt, _ = self._project(rx, z_y, rz, cx, cy, scale)
            c_pts.append(pt)
            
        painter.setBrush(QColor(15, 23, 42, 220))
        painter.setPen(QPen(QColor(56, 189, 248), 1.5))
        painter.drawPolygon(QPolygonF(c_pts))
        
        # Base C orientation mark
        mark_pt, _ = self._project(base_radius * cos_c, z_y, base_radius * sin_c, cx, cy, scale)
        center_c, _ = self._project(0, z_y, 0, cx, cy, scale)
        painter.setPen(QPen(QColor(56, 189, 248), 2))
        painter.drawLine(center_c, mark_pt)

        # 4. Pivot A (Mounted on Base C, Tilting/Rotating around horizontal axis)
        a_rad = math.radians(self.cur_a_deg)
        cos_a, sin_a = math.cos(a_rad), math.sin(a_rad)
        
        pivot_h = z_y + 8.0 # Height of pivot axis
        
        # Pivot bracket
        p_b1, _ = self._project(-3 * cos_c - 0 * sin_c, pivot_h, -3 * sin_c + 0 * cos_c, cx, cy, scale)
        p_b2, _ = self._project(3 * cos_c - 0 * sin_c, pivot_h, 3 * sin_c + 0 * cos_c, cx, cy, scale)
        painter.setPen(QPen(QColor(100, 116, 139), 2.5))
        painter.drawLine(center_c, p_b1)
        painter.drawLine(center_c, p_b2)
        
        # 5. Laser Bar (Collinear assembly with Left & Right Lasers at 180° opposite)
        laser_half_len = 14.0
        
        # Vector along Pivot orientation
        # Axis A rotates in the plane formed by base angle
        lx = laser_half_len * cos_a
        ly = laser_half_len * sin_a
        
        # Left Laser (Laser 1 / Frontal) tip
        l1_x = -lx * sin_c
        l1_y = pivot_h + ly
        l1_z = lx * cos_c
        
        # Right Laser (Laser 2 / Oposto 180°) tip
        l2_x = lx * sin_c
        l2_y = pivot_h - ly
        l2_z = -lx * cos_c
        
        pt_l1, _ = self._project(l1_x, l1_y, l1_z, cx, cy, scale)
        pt_l2, _ = self._project(l2_x, l2_y, l2_z, cx, cy, scale)
        center_p, _ = self._project(0, pivot_h, 0, cx, cy, scale)
        
        # Draw physical laser cylinder/pivot bar
        painter.setPen(QPen(QColor(203, 213, 225), 3.5))
        painter.drawLine(pt_l1, pt_l2)
        
        # Left Laser nozzle
        painter.setBrush(QColor(239, 68, 68))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(pt_l1, 3.5, 3.5)
        
        # Right Laser nozzle
        painter.setBrush(QColor(16, 185, 129))
        painter.drawEllipse(pt_l2, 3.5, 3.5)

        # 6. Active Laser Beams (Glow & rays when power > 0)
        beam_len = 45.0
        if self.laser1_active > 0:
            beam1_x = l1_x - beam_len * sin_c * cos_a
            beam1_y = l1_y + beam_len * sin_a
            beam1_z = l1_z + beam_len * cos_c * cos_a
            pt_b1, _ = self._project(beam1_x, beam1_y, beam1_z, cx, cy, scale)
            # Beam ray
            painter.setPen(QPen(QColor(239, 68, 68, 220), 2))
            painter.drawLine(pt_l1, pt_b1)
            # Beam glow
            painter.setPen(QPen(QColor(239, 68, 68, 60), 6))
            painter.drawLine(pt_l1, pt_b1)
            
        if self.laser2_active > 0:
            beam2_x = l2_x + beam_len * sin_c * cos_a
            beam2_y = l2_y - beam_len * sin_a
            beam2_z = l2_z - beam_len * cos_c * cos_a
            pt_b2, _ = self._project(beam2_x, beam2_y, beam2_z, cx, cy, scale)
            # Beam ray
            painter.setPen(QPen(QColor(16, 185, 129, 220), 2))
            painter.drawLine(pt_l2, pt_b2)
            # Beam glow
            painter.setPen(QPen(QColor(16, 185, 129, 60), 6))
            painter.drawLine(pt_l2, pt_b2)

        # 7. Telemetry HUD Overlay (Top-Left and Bottom)
        painter.setPen(QColor(148, 163, 184))
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.drawText(8, 14, "3D KINEMATICS")
        
        painter.setFont(QFont("Segoe UI", 7))
        painter.setPen(QColor(56, 189, 248))
        painter.drawText(8, h - 6, f"C: {self.cur_c_deg:+.1f}° | A: {self.cur_a_deg:+.1f}° | Z: {self.cur_z_pct:.0f}%")
        
        # Legend dots
        painter.setBrush(QColor(239, 68, 68))
        painter.drawEllipse(w - 45, 6, 5, 5)
        painter.setPen(QColor(148, 163, 184))
        painter.drawText(w - 38, 12, "Esq")
        
        painter.setBrush(QColor(16, 185, 129))
        painter.drawEllipse(w - 20, 6, 5, 5)
        painter.drawText(w - 13, 12, "Dir")
