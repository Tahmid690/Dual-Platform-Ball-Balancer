#!/usr/bin/env python3
"""
Ball & Beam PID Control Dashboard
Quad-VL53L0X real-time monitor with 2D physics visualization.

Physical setup:
  - Lower platform (stationary): Setpoint ball rolls on it, SP1/SP2 sensors read position
  - Upper beam (tilts via servo): Ball to balance rolls on it, Ball1/Ball2 sensors at ends
  - White vertical supports on left and right
  - Servo on the right side tilts the upper beam
"""

import sys
import re
import time
import glob
import math
import collections

from PyQt5.QtCore import Qt, QThread, pyqtSignal, QPointF, QRectF
from PyQt5.QtGui import (
    QFont, QColor, QBrush, QPen, QPainter, QLinearGradient,
    QRadialGradient, QPainterPath, QFontDatabase
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QComboBox, QFrame, QSplitter, QTextEdit,
    QFileDialog, QScrollArea
)

import pyqtgraph as pg
import serial

pg.setConfigOption('background', '#0f0f17')
pg.setConfigOption('foreground', '#94a3b8')
pg.setConfigOptions(antialias=True)

# ═══════════════════════════════════════════════════════════════════════
# Color Palette (Catppuccin Mocha inspired)
# ═══════════════════════════════════════════════════════════════════════
C_BG_DARK    = "#0b0b12"
C_BG         = "#0f0f17"
C_BG_CARD    = "#161625"
C_BG_CARD_H  = "#1c1c30"
C_BORDER     = "#252540"
C_TEXT       = "#e2e8f0"
C_TEXT_DIM   = "#64748b"
C_BLUE       = "#38bdf8"
C_CYAN       = "#22d3ee"
C_AMBER      = "#fbbf24"
C_RED        = "#f87171"
C_GREEN      = "#4ade80"
C_PURPLE     = "#a78bfa"
C_ORANGE     = "#fb923c"
C_TEAL       = "#2dd4bf"

# ═══════════════════════════════════════════════════════════════════════
# Serial Worker Thread
# ═══════════════════════════════════════════════════════════════════════
class SerialThread(QThread):
    data_received = pyqtSignal(dict, str)
    status_changed = pyqtSignal(bool, str)

    def __init__(self, port, baud=9600):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = False
        self.ser = None
        self.regex = re.compile(
            r'\[(?P<state>[^|]+)\|(?P<sp_src>[^\]]+)\]\s+'
            r'SP1:(?P<sp1>\s*\d+|ERR)\s+SP2:(?P<sp2>\s*\d+|ERR)\s+\|\s+'
            r'Ball1:(?P<b1>\s*\d+|ERR)\s+Ball2:(?P<b2>\s*\d+|ERR)\s+\|\s+'
            r'Ball:\s*(?P<ball>-?\d+)mm\s+\|\s+'
            r'Set:\s*(?P<set>-?\d+)mm\s+\|\s+'
            r'Err:\s*(?P<err>-?\d+)\s+\|\s+'
            r'Ang:\s*(?P<ang>\d+)deg'
        )

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            self.running = True
            self.status_changed.emit(True, f"Connected to {self.port}")
        except Exception as e:
            self.status_changed.emit(False, f"Error: {e}")
            return

        while self.running:
            try:
                if self.ser and self.ser.in_waiting:
                    raw = self.ser.readline()
                    if raw:
                        line = raw.decode('utf-8', errors='replace').strip()
                        if line:
                            match = self.regex.search(line)
                            parsed = {}
                            if match:
                                d = match.groupdict()
                                parsed = {
                                    'state': d['state'].strip(),
                                    'sp_src': d['sp_src'].strip(),
                                    'sp1': int(d['sp1'].strip()) if d['sp1'].strip() != 'ERR' else None,
                                    'sp2': int(d['sp2'].strip()) if d['sp2'].strip() != 'ERR' else None,
                                    'b1': int(d['b1'].strip()) if d['b1'].strip() != 'ERR' else None,
                                    'b2': int(d['b2'].strip()) if d['b2'].strip() != 'ERR' else None,
                                    'ball': int(d['ball']),
                                    'set': int(d['set']),
                                    'err': int(d['err']),
                                    'ang': int(d['ang']),
                                    'time': time.time()
                                }
                            self.data_received.emit(parsed, line)
                else:
                    time.sleep(0.01)
            except Exception:
                break

        if self.ser and self.ser.is_open:
            self.ser.close()

    def stop(self):
        self.running = False
        self.wait(1500)


# ═══════════════════════════════════════════════════════════════════════
# 2D Side-View Ball-Beam Visualizer (matches your physical setup)
# ═══════════════════════════════════════════════════════════════════════
class PhysicsVisualizer(QWidget):
    """
    Side-view rendering matching the physical setup:
      - Lower stationary platform with setpoint ball + SP1/SP2 sensors
      - Upper tilting beam: LEFT pillar = fixed hinge, RIGHT = servo arm pushes beam
      - Ball rolls on upper beam, Ball1/Ball2 sensors at ends
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(280)

        self.ball_pos_mm  = 160
        self.setpoint_mm  = 160
        self.angle_deg    = 63
        self.sp_ball_mm   = 160
        self.track_len    = 360

    def update_state(self, ball_mm, setpoint_mm, angle_deg, sp_ball_mm=None):
        self.ball_pos_mm = max(0, min(self.track_len, ball_mm))
        self.setpoint_mm = max(0, min(self.track_len, setpoint_mm))
        self.angle_deg   = angle_deg
        if sp_ball_mm is not None:
            self.sp_ball_mm = max(0, min(self.track_len, sp_ball_mm))
        else:
            self.sp_ball_mm = self.setpoint_mm
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # Background
        bg = QLinearGradient(0, 0, 0, h)
        bg.setColorAt(0, QColor("#0d0d14"))
        bg.setColorAt(1, QColor("#0b0b12"))
        p.fillRect(self.rect(), bg)

        # Layout constants — shorter beam centered in widget
        beam_w_frac = 0.65   # beam uses 65% of widget width
        beam_w      = w * beam_w_frac
        left_x      = (w - beam_w) / 2.0
        right_x     = left_x + beam_w
        base_y      = h - 40
        lower_h     = 12
        lower_y     = base_y - lower_h
        pillar_h    = 140
        upper_y     = lower_y - pillar_h
        pillar_w    = 10
        ball_r      = 16
        sensor_w    = 14
        sensor_h    = 18

        # ── Lower Platform (stationary) ───────────────────────────────
        plat_grad = QLinearGradient(0, lower_y, 0, base_y)
        plat_grad.setColorAt(0, QColor("#1e1e2e"))
        plat_grad.setColorAt(1, QColor("#111118"))
        p.setBrush(plat_grad)
        p.setPen(QPen(QColor("#333355"), 1.5))
        p.drawRoundedRect(QRectF(left_x - 8, lower_y, beam_w + 16, lower_h), 4, 4)

        # Track groove
        p.setPen(QPen(QColor("#333355"), 1))
        p.drawLine(QPointF(left_x + 4, lower_y + lower_h / 2),
                   QPointF(right_x - 4, lower_y + lower_h / 2))

        # ── Lower Sensors (SP1 left, SP2 right) ──────────────────────
        def draw_sensor(cx, top_y, color, label):
            p.setBrush(QBrush(QColor(color)))
            p.setPen(QPen(QColor(color).lighter(140), 1.5))
            p.drawRoundedRect(QRectF(cx - sensor_w / 2, top_y, sensor_w, sensor_h), 3, 3)
            p.setBrush(QBrush(QColor("#ff3333")))
            p.setPen(Qt.NoPen)
            p.drawEllipse(QPointF(cx, top_y + sensor_h + 3), 2, 2)
            p.setPen(QColor(C_TEXT_DIM))
            p.setFont(QFont("SansSerif", 7))
            p.drawText(QRectF(cx - 20, top_y - 13, 40, 11), Qt.AlignCenter, label)

        draw_sensor(left_x + 8, lower_y - sensor_h - 2, "#2dd4bf", "SP1")
        draw_sensor(right_x - 8, lower_y - sensor_h - 2, "#2dd4bf", "SP2")

        # ── Setpoint Ball (orange, on lower platform) ─────────────────
        sp_frac = self.sp_ball_mm / float(self.track_len)
        sp_bx = left_x + sp_frac * beam_w
        sp_by = lower_y - ball_r + 2

        ball_grad = QRadialGradient(QPointF(sp_bx - 3, sp_by - 4), ball_r + 2)
        ball_grad.setColorAt(0.0, QColor("#ffe0a0"))
        ball_grad.setColorAt(0.4, QColor("#ff9f1c"))
        ball_grad.setColorAt(0.85, QColor("#e07000"))
        ball_grad.setColorAt(1.0, QColor("#804000"))
        p.setBrush(ball_grad)
        p.setPen(QPen(QColor("#ffb347"), 1.5))
        p.drawEllipse(QPointF(sp_bx, sp_by), ball_r, ball_r)

        p.setPen(QColor(C_AMBER))
        p.setFont(QFont("SansSerif", 8, QFont.Bold))
        p.drawText(QRectF(sp_bx - 30, sp_by - ball_r - 16, 60, 13),
                   Qt.AlignCenter, f"SP: {self.setpoint_mm}mm")

        # ── Vertical Support Pillars ──────────────────────────────────
        def draw_pillar(x, top, bottom):
            pg = QLinearGradient(x, top, x, bottom)
            pg.setColorAt(0, QColor("#e8e8f0"))
            pg.setColorAt(0.5, QColor("#c0c0d0"))
            pg.setColorAt(1, QColor("#a0a0b8"))
            p.setBrush(pg)
            p.setPen(QPen(QColor("#d0d0e0"), 1))
            p.drawRoundedRect(QRectF(x - pillar_w / 2, top, pillar_w, bottom - top), 3, 3)

        left_pillar_x  = left_x + 15
        right_pillar_x = right_x - 15
        draw_pillar(left_pillar_x, upper_y - 8, lower_y)
        draw_pillar(right_pillar_x, upper_y - 8, lower_y)

        # ── Fixed Hinge on LEFT pillar ────────────────────────────────
        p.setBrush(QBrush(QColor("#d0d0e0")))
        p.setPen(QPen(QColor("#9090b0"), 1.5))
        p.drawEllipse(QPointF(left_pillar_x, upper_y), 5, 5)

        # ── Servo Motor on RIGHT pillar ───────────────────────────────
        servo_x = right_pillar_x
        servo_y = upper_y - 12
        sv_w, sv_h = 26, 32

        servo_grad = QLinearGradient(servo_x, servo_y, servo_x, servo_y + sv_h)
        servo_grad.setColorAt(0, QColor("#505068"))
        servo_grad.setColorAt(1, QColor("#303044"))
        p.setBrush(servo_grad)
        p.setPen(QPen(QColor("#606080"), 1.5))
        p.drawRoundedRect(QRectF(servo_x - sv_w / 2, servo_y, sv_w, sv_h), 4, 4)

        p.setPen(QColor(C_TEXT_DIM))
        p.setFont(QFont("SansSerif", 7))
        p.drawText(QRectF(servo_x - 22, servo_y + sv_h + 2, 44, 11),
                   Qt.AlignCenter, "SERVO")

        # Servo shaft
        p.setBrush(QBrush(QColor("#e0e0f0")))
        p.setPen(QPen(QColor("#9090b0"), 1.5))
        p.drawEllipse(QPointF(servo_x, upper_y), 5, 5)

        # Servo arm (small line from servo shaft towards beam connection)
        tilt_deg = (self.angle_deg - 63.0) * 0.18  # REDUCED scale for realistic tilt
        arm_len = 18
        arm_rad = math.radians(-tilt_deg + 90)
        arm_ex = servo_x + arm_len * math.cos(arm_rad)
        arm_ey = upper_y - arm_len * math.sin(arm_rad)
        p.setPen(QPen(QColor("#c0c0d8"), 2.5))
        p.drawLine(QPointF(servo_x, upper_y), QPointF(arm_ex, arm_ey))

        # ── Upper Beam (tilting) ──────────────────────────────────────
        # Pivot/hinge is on the LEFT pillar.
        # Servo on the RIGHT pushes the right end up/down.
        # Positive tilt_deg => right end goes up (angle > 63)
        # Negative tilt_deg => right end goes down (angle < 63)

        pivot_bx = left_pillar_x
        pivot_by = upper_y
        beam_len_px = right_pillar_x - left_pillar_x + 20  # beam slightly overhangs
        beam_thickness = 12

        p.save()
        p.translate(pivot_bx, pivot_by)
        p.rotate(-tilt_deg)  # negative so angle>63 lifts right end visually

        # Beam extends to the RIGHT from pivot
        bx_start = -10    # small overhang past left pillar
        bx_end   = beam_len_px - 10

        beam_grad2 = QLinearGradient(0, -beam_thickness / 2, 0, beam_thickness / 2)
        beam_grad2.setColorAt(0, QColor("#3a3a50"))
        beam_grad2.setColorAt(0.5, QColor("#4a4a64"))
        beam_grad2.setColorAt(1, QColor("#2a2a3e"))
        p.setBrush(beam_grad2)
        p.setPen(QPen(QColor(C_BLUE), 1.5))
        p.drawRoundedRect(QRectF(bx_start, -beam_thickness / 2,
                                  bx_end - bx_start, beam_thickness), 5, 5)

        # ── Upper Beam Sensors (Ball1 left, Ball2 right) ──────────────
        def draw_beam_sensor(bx, color, label):
            p.setBrush(QBrush(QColor(color)))
            p.setPen(QPen(QColor(color).lighter(130), 1))
            p.drawRoundedRect(QRectF(bx - 6, -beam_thickness / 2 - 14, 12, 14), 2, 2)
            p.setPen(QColor(C_TEXT_DIM))
            p.setFont(QFont("SansSerif", 6))
            p.drawText(QRectF(bx - 16, -beam_thickness / 2 - 26, 32, 11),
                       Qt.AlignCenter, label)

        draw_beam_sensor(bx_start + 14, "#74c7ec", "Ball1")
        draw_beam_sensor(bx_end - 14, "#74c7ec", "Ball2")

        # Laser line on beam
        p.setPen(QPen(QColor(255, 60, 60, 80), 1, Qt.DotLine))
        p.drawLine(QPointF(bx_start + 14, -1), QPointF(bx_end - 14, -1))

        # ── Balancing Ball (orange, on upper beam) ────────────────────
        ball_frac = self.ball_pos_mm / float(self.track_len)
        ball_bx = bx_start + ball_frac * (bx_end - bx_start)
        ball_by = -beam_thickness / 2 - ball_r + 2

        ball_grad2 = QRadialGradient(QPointF(ball_bx - 3, ball_by - 4), ball_r + 2)
        ball_grad2.setColorAt(0.0, QColor("#ffe0a0"))
        ball_grad2.setColorAt(0.4, QColor("#ff9f1c"))
        ball_grad2.setColorAt(0.85, QColor("#e07000"))
        ball_grad2.setColorAt(1.0, QColor("#804000"))
        p.setBrush(ball_grad2)
        p.setPen(QPen(QColor("#ffb347"), 1.5))
        p.drawEllipse(QPointF(ball_bx, ball_by), ball_r, ball_r)

        p.setPen(QColor(C_BLUE))
        p.setFont(QFont("SansSerif", 8, QFont.Bold))
        p.drawText(QRectF(ball_bx - 35, ball_by - ball_r - 16, 70, 13),
                   Qt.AlignCenter, f"Ball: {self.ball_pos_mm}mm")

        # ── Scale markings on beam ────────────────────────────────────
        p.setPen(QPen(QColor("#555575"), 1))
        p.setFont(QFont("SansSerif", 6))
        for mm_val in range(0, self.track_len + 1, 60):
            frac = mm_val / float(self.track_len)
            mx = bx_start + frac * (bx_end - bx_start)
            p.drawLine(QPointF(mx, beam_thickness / 2),
                       QPointF(mx, beam_thickness / 2 + 4))
            p.setPen(QColor("#555575"))
            p.drawText(QRectF(mx - 12, beam_thickness / 2 + 5, 24, 10),
                       Qt.AlignCenter, str(mm_val))
            p.setPen(QPen(QColor("#555575"), 1))

        p.restore()

        # ── Overlay HUD ───────────────────────────────────────────────
        p.setPen(QColor(C_TEXT))
        p.setFont(QFont("SansSerif", 9, QFont.Bold))
        p.drawText(15, 20, f"Servo: {self.angle_deg}°")

        tilt_txt = "LEVEL" if abs(self.angle_deg - 63) <= 2 else \
                   f"↙ LEFT {63 - self.angle_deg}°" if self.angle_deg < 63 else \
                   f"↗ RIGHT +{self.angle_deg - 63}°"
        tilt_color = C_GREEN if abs(self.angle_deg - 63) <= 2 else C_AMBER
        p.setPen(QColor(tilt_color))
        p.drawText(15, 35, tilt_txt)

        # Platform labels
        p.setPen(QColor(C_TEXT_DIM))
        p.setFont(QFont("SansSerif", 7))
        p.drawText(QRectF(left_x - 8, base_y + 2, beam_w + 16, 14),
                   Qt.AlignCenter, "▬▬ LOWER PLATFORM (SETPOINT) ▬▬")
        p.drawText(w - 170, 20, "UPPER BEAM (BALANCE)")


# ═══════════════════════════════════════════════════════════════════════
# Metric Card Widget
# ═══════════════════════════════════════════════════════════════════════
class GlassCard(QFrame):
    def __init__(self, title, unit="", color=C_BLUE, big=False, parent=None):
        super().__init__(parent)
        self.unit = unit
        self.color = color

        self.setStyleSheet(f"""
            QFrame {{
                background-color: {C_BG_CARD};
                border-radius: 12px;
                border: 1px solid {C_BORDER};
            }}
            QFrame:hover {{
                background-color: {C_BG_CARD_H};
                border: 1px solid {color}44;
            }}
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(2)

        self.lbl_title = QLabel(title)
        self.lbl_title.setFont(QFont("SansSerif", 8))
        self.lbl_title.setStyleSheet(f"color: {C_TEXT_DIM}; border: none;")

        val_size = 26 if big else 20
        self.lbl_value = QLabel(f"—")
        self.lbl_value.setFont(QFont("SansSerif", val_size, QFont.Bold))
        self.lbl_value.setStyleSheet(f"color: {color}; border: none;")

        self.lbl_unit = QLabel(unit)
        self.lbl_unit.setFont(QFont("SansSerif", 9))
        self.lbl_unit.setStyleSheet(f"color: {C_TEXT_DIM}; border: none;")

        layout.addWidget(self.lbl_title)
        layout.addWidget(self.lbl_value)
        if unit:
            layout.addWidget(self.lbl_unit)

    def set_value(self, val):
        if val is None:
            self.lbl_value.setText("ERR")
            self.lbl_value.setStyleSheet(f"color: {C_RED}; border: none;")
        else:
            self.lbl_value.setText(str(val))
            self.lbl_value.setStyleSheet(f"color: {self.color}; border: none;")


# ═══════════════════════════════════════════════════════════════════════
# Position Bar Widget (horizontal gauge for sensor reading)
# ═══════════════════════════════════════════════════════════════════════
class PositionBar(QWidget):
    def __init__(self, label, max_val=360, color=C_BLUE, parent=None):
        super().__init__(parent)
        self.label = label
        self.max_val = max_val
        self.color = color
        self.value = 0
        self.setFixedHeight(40)

    def set_value(self, val):
        self.value = val if val is not None else -1
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        # Track background
        track_h = 10
        track_y = h - track_h - 6
        p.setBrush(QBrush(QColor(C_BG_CARD)))
        p.setPen(QPen(QColor(C_BORDER), 1))
        p.drawRoundedRect(QRectF(0, track_y, w, track_h), 5, 5)

        # Fill
        if self.value >= 0:
            frac = min(1.0, self.value / float(self.max_val))
            fill_w = max(4, frac * w)
            fill_grad = QLinearGradient(0, track_y, fill_w, track_y)
            fill_grad.setColorAt(0, QColor(self.color))
            fill_grad.setColorAt(1, QColor(self.color).darker(130))
            p.setBrush(fill_grad)
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(QRectF(0, track_y, fill_w, track_h), 5, 5)

        # Label and value text
        p.setPen(QColor(C_TEXT_DIM))
        p.setFont(QFont("SansSerif", 8))
        p.drawText(2, track_y - 4, self.label)

        p.setPen(QColor(self.color))
        p.setFont(QFont("SansSerif", 9, QFont.Bold))
        val_txt = f"{self.value} mm" if self.value >= 0 else "ERR"
        p.drawText(QRectF(0, 0, w - 4, track_y - 2), Qt.AlignRight | Qt.AlignBottom, val_txt)


# ═══════════════════════════════════════════════════════════════════════
# Main Window
# ═══════════════════════════════════════════════════════════════════════
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ball & Beam PID Control Dashboard — Quad VL53L0X")
        self.resize(1280, 920)

        self.serial_thread = None
        self.log_paused = False

        self.hist_len = 300
        self.t_hist    = collections.deque(maxlen=self.hist_len)
        self.ball_hist = collections.deque(maxlen=self.hist_len)
        self.set_hist  = collections.deque(maxlen=self.hist_len)
        self.err_hist  = collections.deque(maxlen=self.hist_len)
        self.ang_hist  = collections.deque(maxlen=self.hist_len)
        self.t0 = time.time()

        self._build_ui()
        self._refresh_ports()

    def _build_ui(self):
        self.setStyleSheet(f"""
            QMainWindow {{ background-color: {C_BG_DARK}; }}
            QLabel {{ color: {C_TEXT}; }}
            QComboBox {{
                background-color: {C_BG_CARD};
                color: {C_TEXT};
                border: 1px solid {C_BORDER};
                border-radius: 6px;
                padding: 5px 10px;
                font-size: 12px;
            }}
            QPushButton {{
                background-color: {C_BLUE};
                color: {C_BG_DARK};
                border: none;
                border-radius: 6px;
                padding: 7px 14px;
                font-weight: bold;
                font-size: 12px;
            }}
            QPushButton:hover {{ background-color: {C_CYAN}; }}
            QPushButton#btn_disc {{ background-color: {C_RED}; }}
            QPushButton#btn_disc:hover {{ background-color: #fca5a5; }}
            QTextEdit {{
                background-color: {C_BG};
                color: {C_GREEN};
                font-family: "Monospace";
                font-size: 11px;
                border: 1px solid {C_BORDER};
                border-radius: 8px;
                padding: 6px;
            }}
        """)

        root = QWidget()
        self.setCentralWidget(root)
        ml = QVBoxLayout(root)
        ml.setContentsMargins(12, 10, 12, 10)
        ml.setSpacing(10)

        # ── TOP BAR ───────────────────────────────────────────────────
        top = QHBoxLayout()

        title = QLabel("⚡ BALL & BEAM PID CONTROLLER")
        title.setFont(QFont("SansSerif", 15, QFont.Bold))
        title.setStyleSheet(f"color: {C_BLUE};")

        self.cmb_port = QComboBox()
        self.cmb_port.setMinimumWidth(160)

        btn_ref = QPushButton("↻ Refresh")
        btn_ref.clicked.connect(self._refresh_ports)

        self.btn_conn = QPushButton("🔌 Connect")
        self.btn_conn.clicked.connect(self._toggle_conn)

        self.lbl_status = QLabel("● Disconnected")
        self.lbl_status.setStyleSheet(f"color: {C_RED}; font-weight: bold;")

        top.addWidget(title)
        top.addStretch()
        top.addWidget(QLabel("Port:"))
        top.addWidget(self.cmb_port)
        top.addWidget(btn_ref)
        top.addWidget(self.btn_conn)
        top.addWidget(self.lbl_status)
        ml.addLayout(top)

        # ── 2D PHYSICS VISUALIZER ─────────────────────────────────────
        self.viz = PhysicsVisualizer()
        ml.addWidget(self.viz)

        # ── METRIC CARDS ROW (big numbers) ────────────────────────────
        cards = QHBoxLayout()
        cards.setSpacing(10)

        self.c_ball  = GlassCard("Ball Position", "mm", C_BLUE, big=True)
        self.c_set   = GlassCard("Setpoint", "mm", C_AMBER, big=True)
        self.c_err   = GlassCard("PID Error", "mm", C_RED, big=True)
        self.c_ang   = GlassCard("Servo Angle", "deg", C_PURPLE, big=True)
        self.c_state = GlassCard("State", "", C_GREEN, big=False)

        cards.addWidget(self.c_ball)
        cards.addWidget(self.c_set)
        cards.addWidget(self.c_err)
        cards.addWidget(self.c_ang)
        cards.addWidget(self.c_state)
        ml.addLayout(cards)

        # ── SENSOR BARS (horizontal gauge bars) ──────────────────────
        sensor_frame = QFrame()
        sensor_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {C_BG_CARD};
                border-radius: 10px;
                border: 1px solid {C_BORDER};
            }}
        """)
        sf_layout = QHBoxLayout(sensor_frame)
        sf_layout.setContentsMargins(14, 10, 14, 10)
        sf_layout.setSpacing(20)

        # Setpoint sensors column
        sp_col = QVBoxLayout()
        sp_title = QLabel("SETPOINT SENSORS (Lower Platform)")
        sp_title.setFont(QFont("SansSerif", 9, QFont.Bold))
        sp_title.setStyleSheet(f"color: {C_TEAL};")
        self.bar_sp1 = PositionBar("SP1 (Left End)", 400, C_TEAL)
        self.bar_sp2 = PositionBar("SP2 (Right End)", 400, C_TEAL)
        sp_col.addWidget(sp_title)
        sp_col.addWidget(self.bar_sp1)
        sp_col.addWidget(self.bar_sp2)

        # Ball sensors column
        b_col = QVBoxLayout()
        b_title = QLabel("BALL SENSORS (Upper Beam)")
        b_title.setFont(QFont("SansSerif", 9, QFont.Bold))
        b_title.setStyleSheet(f"color: {C_CYAN};")
        self.bar_b1 = PositionBar("Ball1 (Left End)", 400, C_CYAN)
        self.bar_b2 = PositionBar("Ball2 (Right End)", 400, C_CYAN)
        b_col.addWidget(b_title)
        b_col.addWidget(self.bar_b1)
        b_col.addWidget(self.bar_b2)

        sf_layout.addLayout(sp_col, 1)

        # Divider
        div = QFrame()
        div.setFixedWidth(1)
        div.setStyleSheet(f"background-color: {C_BORDER};")
        sf_layout.addWidget(div)

        sf_layout.addLayout(b_col, 1)
        ml.addWidget(sensor_frame)

        # ── BOTTOM SECTION: Charts + Log ──────────────────────────────
        splitter = QSplitter(Qt.Horizontal)

        # Charts
        chart_w = QWidget()
        cl = QVBoxLayout(chart_w)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(4)

        self.p_pos = pg.PlotWidget(title="<span style='color:#94a3b8;font-size:11px;'>"
                                         "Ball Position vs Setpoint (mm)</span>")
        self.p_pos.showGrid(x=True, y=True, alpha=0.15)
        self.p_pos.setLabel('left', 'mm')
        self.p_pos.setLabel('bottom', 'Time (s)')
        self.p_pos.addLegend(offset=(10, 10))
        self.crv_ball = self.p_pos.plot(pen=pg.mkPen(C_BLUE, width=2), name="Ball")
        self.crv_set  = self.p_pos.plot(pen=pg.mkPen(C_AMBER, width=2, style=Qt.DashLine), name="Setpoint")

        self.p_pid = pg.PlotWidget(title="<span style='color:#94a3b8;font-size:11px;'>"
                                         "PID Error (mm) & Servo Angle (deg)</span>")
        self.p_pid.showGrid(x=True, y=True, alpha=0.15)
        self.p_pid.setLabel('left', '')
        self.p_pid.setLabel('bottom', 'Time (s)')
        self.p_pid.addLegend(offset=(10, 10))
        self.crv_err = self.p_pid.plot(pen=pg.mkPen(C_RED, width=2), name="Error")
        self.crv_ang = self.p_pid.plot(pen=pg.mkPen(C_PURPLE, width=2), name="Angle")

        cl.addWidget(self.p_pos)
        cl.addWidget(self.p_pid)
        splitter.addWidget(chart_w)

        # Log panel
        log_w = QWidget()
        ll = QVBoxLayout(log_w)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.setSpacing(4)

        log_hdr = QHBoxLayout()
        log_title = QLabel("📋 Telemetry Log")
        log_title.setFont(QFont("SansSerif", 10, QFont.Bold))

        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.clicked.connect(self._toggle_pause)
        btn_clr = QPushButton("✕ Clear")
        btn_clr.clicked.connect(lambda: self.txt_log.clear())
        btn_csv = QPushButton("💾 CSV")
        btn_csv.clicked.connect(self._export_csv)

        log_hdr.addWidget(log_title)
        log_hdr.addStretch()
        log_hdr.addWidget(self.btn_pause)
        log_hdr.addWidget(btn_clr)
        log_hdr.addWidget(btn_csv)

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)

        ll.addLayout(log_hdr)
        ll.addWidget(self.txt_log)
        splitter.addWidget(log_w)

        splitter.setSizes([750, 450])
        ml.addWidget(splitter, 1)

    # ── Serial Helpers ────────────────────────────────────────────────
    def _refresh_ports(self):
        self.cmb_port.clear()
        ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        self.cmb_port.addItems(ports if ports else ["No ports found"])

    def _toggle_conn(self):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
            self.serial_thread = None
            self.btn_conn.setText("🔌 Connect")
            self.btn_conn.setObjectName("")
            self.btn_conn.setStyle(self.btn_conn.style())
            self.lbl_status.setText("● Disconnected")
            self.lbl_status.setStyleSheet(f"color: {C_RED}; font-weight: bold;")
        else:
            port = self.cmb_port.currentText()
            if not port or "No ports" in port:
                return
            self.serial_thread = SerialThread(port)
            self.serial_thread.data_received.connect(self._on_data)
            self.serial_thread.status_changed.connect(self._on_status)
            self.serial_thread.start()

    def _on_status(self, ok, msg):
        if ok:
            self.btn_conn.setText("⏹ Disconnect")
            self.btn_conn.setObjectName("btn_disc")
            self.btn_conn.setStyle(self.btn_conn.style())
            self.lbl_status.setText(f"● {msg}")
            self.lbl_status.setStyleSheet(f"color: {C_GREEN}; font-weight: bold;")
        else:
            self.btn_conn.setText("🔌 Connect")
            self.btn_conn.setObjectName("")
            self.btn_conn.setStyle(self.btn_conn.style())
            self.lbl_status.setText(f"● {msg}")
            self.lbl_status.setStyleSheet(f"color: {C_RED}; font-weight: bold;")

    def _on_data(self, data, raw):
        if not self.log_paused:
            ts = time.strftime("%H:%M:%S")
            self.txt_log.append(f"[{ts}] {raw}")
            # Auto-scroll
            sb = self.txt_log.verticalScrollBar()
            sb.setValue(sb.maximum())

        if not data:
            return

        # Cards
        self.c_ball.set_value(data['ball'])
        self.c_set.set_value(data['set'])
        self.c_err.set_value(data['err'])
        self.c_ang.set_value(data['ang'])
        self.c_state.set_value(f"{data['state']} | {data['sp_src']}")

        # Sensor bars
        self.bar_sp1.set_value(data['sp1'])
        self.bar_sp2.set_value(data['sp2'])
        self.bar_b1.set_value(data['b1'])
        self.bar_b2.set_value(data['b2'])

        # Visualizer
        self.viz.update_state(data['ball'], data['set'], data['ang'], data['set'])

        # History
        t = time.time() - self.t0
        self.t_hist.append(t)
        self.ball_hist.append(data['ball'])
        self.set_hist.append(data['set'])
        self.err_hist.append(data['err'])
        self.ang_hist.append(data['ang'])

        # Charts
        tl = list(self.t_hist)
        self.crv_ball.setData(tl, list(self.ball_hist))
        self.crv_set.setData(tl, list(self.set_hist))
        self.crv_err.setData(tl, list(self.err_hist))
        self.crv_ang.setData(tl, list(self.ang_hist))

    def _toggle_pause(self):
        self.log_paused = not self.log_paused
        self.btn_pause.setText("▶ Resume" if self.log_paused else "⏸ Pause")

    def _export_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export CSV", "", "CSV (*.csv)")
        if path:
            try:
                with open(path, 'w') as f:
                    f.write("Time_s,Ball_mm,Set_mm,Err_mm,Angle_deg\n")
                    for t, b, s, e, a in zip(self.t_hist, self.ball_hist,
                                              self.set_hist, self.err_hist, self.ang_hist):
                        f.write(f"{t:.2f},{b},{s},{e},{a}\n")
                self.txt_log.append(f"✅ Exported to {path}")
            except Exception as ex:
                self.txt_log.append(f"❌ Export error: {ex}")


# ═══════════════════════════════════════════════════════════════════════
# Entry Point
# ═══════════════════════════════════════════════════════════════════════
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
