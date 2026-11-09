"""
needed packages and versions:

msgpack-numpy==0.4.8
numpy==1.26.2
paramiko==5.1.0
pyqtgraph==0.14.0
PySide6==6.12.0
PySide6_Addons==6.12.0
PySide6_Essentials==6.12.0
PySide6_Pdf==6.12.0.140
PySide6_WebEngine==6.12.0.140
"""

import os, sys, math, csv, time, socket, threading, collections
from datetime import datetime
import pyqtgraph as pg
from PySide6 import QtWidgets as QW, QtCore as QC
import numpy as np

try:
    import paramiko
except ImportError:
    paramiko = None

FIELDS = ["t", "pos_ref", "vel_ref", "pos", "vel", "i_q", "u", "mpc_iter", "fault", "mpc_us", "spi_us", "total_us"]
CSV_HEADERS = ["Time (s)", "Position ref (rad)", "Velocity ref (rad/s)", "Position (rad)", "Velocity (rad/s)",
               "Current (A)", "Controller output (V or A)", "MPC iterations", "Solver fault",
               "MPC time (us)", "SPI time (us)", "Total time (us)"]

WINDOW_S, SAMPLE_DT, DECIMALS = 10.0, 0.01, 3
MAX_STEP_RAD = 30 * 2 * math.pi
PREVIEW_MAX_RAD, SPEED_MAX = 1e6, 80
LEGEND_W, AXIS_W = 130, 70
PI_PROJECT_DIR = "/home/pi/MPC_Demonstrator"
PI_PROGRAM = "mpc_spi"
PI_SERVER_COMMAND = (f"sudo -S -p '' taskset -c 3 chrt -f 50 "
                     f"{PI_PROJECT_DIR}/host/build/{PI_PROGRAM} --matrices {PI_PROJECT_DIR}/host")
PING_PERIOD_S = 0.25

TABS = [
    ("Motion", [
        dict(title="Position", axis="Position (rad)", min_span = 2.0, lines=[("pos_ref", "#e67e22", "Ref"), ("pos", "#1f77b4", "Measured")]),
        dict(title="Velocity", axis="Velocity (rad/s)", ylim=(-85, 85), lines=[("vel_ref", "#e67e22", "Ref"), ("vel", "#1f77b4", "Measured")])]),
    ("Current / Voltage", [
        dict(title="Current", axis="Current (A)", ylim=(-2, 2), lines=[("i_q", "#9b59b6", "Current")]),
        dict(title="Controller output", axis="Voltage (V)", ylim=(-12, 12), lines=[("u", "#2ca02c", "Command")], is_output=True)]),
    ("Timing", [
        dict(title="Computation time", axis="Time (µs)", dec=0,
             lines=[("mpc_us", "#1f77b4", "MPC solve"), ("spi_us", "#e67e22", "SPI")]),
        dict(title="Total (MPC + SPI)", axis="Time (µs)", dec=0, lines=[("total_us", "k", "Total")],
             limit=(1000, "1 ms limit"), ylim=(0, 1200))]),
    ("Solver", [
        dict(title="MPC iterations", axis="Iterations", dec=0, lines=[("mpc_iter", "#1f77b4", "Iterations")],
             limit=(50, "Max 50"), ylim=(0, 60)),
        dict(title="Solver fault", axis="Fault (0 / 1)", dec=0, tick=1, lines=[("fault", "#8e44ad", "Fault")],
             ylim=(-0.1, 1.1))]),
]


class NullSource:
    busy = False

    def __init__(self):
        self.state, self.connected = "IDLE", True

    set_mode = set_ctrl = set_dir = set_ref = lambda self, v: None
    def start(self): self.state = "RUNNING"
    def stop(self): self.state = "IDLE"
    def estop(self): self.state = "FAULT"
    def poll(self): return []


class PiLink(QC.QObject):
    message = QC.Signal(str)
    changed = QC.Signal()
    T_FIELDS = [f for f in FIELDS if f != "total_us"]

    def __init__(self, server_command=PI_SERVER_COMMAND):
        super().__init__()
        self._server_command = server_command
        self.busy, self.connected, self.state = False, False, "DISCONNECTED"
        self._cfg = dict(mode="voltage", ctrl="position", dir=1, ref=dict(type="step", value=3.0, t0=1.0, preview=True))
        self._lock = threading.Lock()
        self._samples, self._chan, self._closing, self._thread = [], None, False, None

    def connect_to(self, address, password):
        if self.busy:
            return
        if paramiko is None:
            self.message.emit("The ssh library is missing. Install it with: pip install paramiko")
            return
        self.busy, self._closing = True, False
        self.changed.emit()
        self._thread = threading.Thread(target=self._worker, args=(address, password), daemon=True)
        self._thread.start()

    @staticmethod
    def _split_address(address):
        user, _, host = address.rpartition("@")
        port = 22
        if ":" in host:
            host, _, p = host.rpartition(":")
            port = int(p)
        return (user or None), host, port

    def _worker(self, address, password):
        client, why = None, None
        try:
            user, host, port = self._split_address(address)
            self.message.emit(f"Connecting to {host} ...")
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(host, port=port, username=user, password=password, timeout=8, banner_timeout=8,
                           auth_timeout=8, look_for_keys=False, allow_agent=False)
            client.get_transport().set_keepalive(2)
            chan = client.get_transport().open_session()
            chan.exec_command(self._server_command)
            self._chan = chan
            if self._server_command.startswith("sudo"):
                chan.sendall((password + "\n").encode())
            password = None
            why = self._session(chan)
        except paramiko.AuthenticationException:
            why = "Wrong user name or password"
        except ValueError:
            why = "Invalid address (expected user@host)"
        except (paramiko.SSHException, OSError, socket.timeout) as e:
            why = f"Connection failed: {e}"
        finally:
            try:
                if client is not None:
                    client.close()
            except Exception:
                pass
            self._chan, self.connected, self.state, self.busy = None, False, "DISCONNECTED", False
            self.message.emit(why or "Disconnected")
            self.changed.emit()

    def _session(self, chan):
        buf, err, last_ping, deadline = b"", b"", 0.0, None
        while True:
            got = False
            while chan.recv_ready():
                buf += chan.recv(65536)
                got = True
            while chan.recv_stderr_ready():
                err = (err + chan.recv_stderr(4096))[-2000:]
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                self._handle_line(raw.decode(errors="replace").strip())
            if chan.exit_status_ready() and not chan.recv_ready():
                break
            now = time.monotonic()
            if self._closing:
                deadline = deadline or now + 3.0
                if now > deadline:
                    break
            elif now - last_ping > PING_PERIOD_S:
                self._send("PING")
                last_ping = now
            if not got:
                time.sleep(0.002)
        if self._closing:
            return None
        text = err.decode(errors="replace").strip().splitlines()
        if any("incorrect password" in l.lower() or "try again" in l.lower() for l in text):
            return "Wrong password (sudo on the Pi)"
        if chan.exit_status_ready() and chan.recv_exit_status() != 0 and text:
            return "The program on the Pi stopped: " + text[-1]
        return "Connection closed by the Pi"

    def _handle_line(self, line):
        if line.startswith("T "):
            try:
                vals = [float(x) for x in line.split()[1:]]
            except ValueError:
                return
            if len(vals) == len(self.T_FIELDS):
                with self._lock:
                    self._samples.append(dict(zip(self.T_FIELDS, vals)))
        elif line.startswith("S "):
            self.state = line[2:].strip()
            self.changed.emit()
        elif line.startswith("H "):
            with self._lock:
                self._samples.clear()
            self.connected, self.state = True, "IDLE"
            self.message.emit("Connected")
            self.changed.emit()
        elif line.startswith("E "):
            self.message.emit("Pi: " + line[2:])

    def disconnect_from(self):
        self._closing = True
        self._send("STOP")
        try:
            if self._chan is not None:
                self._chan.shutdown_write()
        except Exception:
            pass

    def shutdown(self):
        if self.busy:
            self.disconnect_from()
            if self._thread is not None:
                self._thread.join(4.0)

    def poll(self):
        with self._lock:
            out, self._samples = self._samples, []
        return out

    def _send(self, line):
        chan = self._chan
        if chan is None:
            return
        try:
            with self._lock:
                chan.sendall((line + "\n").encode())
        except Exception:
            pass

    def set_mode(self, mode): self._cfg["mode"] = mode
    def set_ctrl(self, ctrl): self._cfg["ctrl"] = ctrl
    def set_dir(self, d): self._cfg["dir"] = -1 if d > 0 else 1
    def set_ref(self, ref): self._cfg["ref"] = dict(ref)

    def start(self):
        c, r = self._cfg, self._cfg["ref"]
        self._send(f"CFG mode={c['mode']} ctrl={c['ctrl']} dir={c['dir']} ref={r['type']} "
                   f"value={r['value']:.6g} t0={r['t0']:.6g} preview={1 if r['preview'] else 0}")
        self._send("START")

    def stop(self): self._send("STOP")
    def estop(self): self._send("ESTOP")


class FixedAxis(pg.AxisItem):
    def __init__(self, orientation, decimals=DECIMALS, **kw):
        super().__init__(orientation, **kw)
        self.decimals = decimals
        self.enableAutoSIPrefix(False)

    def tickStrings(self, values, scale, spacing):
        return [f"{v * scale:.{self.decimals}f}" for v in values]


class MainWindow(QW.QMainWindow):
    def __init__(self, source):
        super().__init__()
        self.src, self._was_connected, self.paused = source, False, False
        self.setWindowTitle("MPC control DC motor (Demonstrator app)")
        self.buf = {k: collections.deque(maxlen=int(WINDOW_S / SAMPLE_DT) + 50) for k in FIELDS}
        self.ref_curves, self.hidden_fields, self.legends, self.u_plot = {}, set(), [], None
        self.rec, self.recording, self.rec_dirty = [], False, False
        self._last_dir = os.path.expanduser("~")
        self.autoscale = []

        central = QW.QWidget()
        self.setCentralWidget(central)
        root = QW.QHBoxLayout(central)
        root.addWidget(self._build_controls(), 0)
        root.addWidget(self._build_plots(), 1)
        if hasattr(self.src, "message"):
            self.src.message.connect(self.statusBar().showMessage)
            self.src.changed.connect(self._on_link_changed)
            self.statusBar().showMessage("Not connected")
        else:
            self.statusBar().showMessage("Offline mode")

        self.timer = QC.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)
        self._on_mode()
        self._on_ctrl()
        self._update_rec_ui()

    @staticmethod
    def _row(form, label, w, tip):
        l = QW.QLabel(label)
        l.setToolTip(tip)
        w.setToolTip(tip)
        form.addRow(l, w)

    @staticmethod
    def _spin(val, lo, hi, suffix):
        s = QW.QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setValue(val)
        s.setSuffix(suffix)
        s.setDecimals(DECIMALS)
        return s

    def _build_controls(self):
        panel = QW.QWidget()
        panel.setFixedWidth(280)
        lay = QW.QVBoxLayout(panel)

        g = QW.QGroupBox("Connection")
        f = QW.QFormLayout(g)
        self.ip = QW.QLineEdit("pi@clt-lab-t-5086")
        self.conn_btn = QW.QPushButton("Connect")
        self.conn_btn.clicked.connect(self.on_connect)
        self.conn_btn.setToolTip("Connect to the Raspberry Pi")
        self._row(f, "Pi address", self.ip, "Address of the Raspberry Pi")
        f.addRow(self.conn_btn)
        lay.addWidget(g)

        g = QW.QGroupBox("Controller")
        f = QW.QFormLayout(g)
        self.ctrl, self.mode, self.ref_type = QW.QComboBox(), QW.QComboBox(), QW.QComboBox()
        self.ctrl.addItems(["Position", "Speed"])
        self.mode.addItems(["Voltage", "Current"])
        self.ref_type.addItems(["Step", "Cycle"])
        self.value = self._spin(6.28, 0, PREVIEW_MAX_RAD, " rad")
        self.t0 = self._spin(1.0, 0, 60, " s")
        self.preview = QW.QCheckBox("Use preview")
        self.preview.setChecked(False)

        self.cw_btn, self.ccw_btn = QW.QPushButton("CW"), QW.QPushButton("CCW")
        self.dir_group = QW.QButtonGroup(self)
        self.dir_group.setExclusive(True)
        dir_box = QW.QWidget()
        h = QW.QHBoxLayout(dir_box)
        h.setContentsMargins(0, 0, 0, 0)
        for b, tip in ((self.cw_btn, "Rotate clockwise"),
                       (self.ccw_btn, "Rotate counter-clockwise")):
            b.setCheckable(True)
            b.setToolTip(tip)
            b.setStyleSheet("QPushButton:checked{background:#2980b9;color:white;font-weight:bold}")
            self.dir_group.addButton(b)
            h.addWidget(b)
        self.cw_btn.setChecked(True)

        self.ref_type.currentIndexChanged.connect(self._update_enabled)
        self.ctrl.currentIndexChanged.connect(self._on_ctrl)
        self.mode.currentIndexChanged.connect(self._on_mode)
        self.preview.toggled.connect(self._on_preview)

        self._row(f, "Control strategy", self.ctrl, "Motor position (rad) or motor speed (rad/s)")
        self._row(f, "MPC mode", self.mode, "MPC mode: direct voltage or current reference")
        self._row(f, "Direction", dir_box, "Sense of rotation")
        self._row(f, "Reference", self.ref_type,
                  "Step: step response\nCycle: step to 10 turns, hold 2 s, back to 0\n"
                  "Speed control only supports Step")
        self._row(f, "Step value", self.value,
                  "Magnitude of the step reference\nWithout preview, position steps are limited to 30 turns "
                  "(188.496 rad)\nWith preview the value is not limited")
        self._row(f, "Start time", self.t0, "Delay between pressing Start and the beginning of the step")
        self._row(f, "Preview", self.preview,
                  "Follows an s-shape trajectory"
                  "\nAvailable in Position mode only")
        lay.addWidget(g)

        self.start_btn, self.stop_btn = QW.QPushButton("Start"), QW.QPushButton("Stop")
        self.estop_btn = QW.QPushButton("Emergency stop")
        self.estop_btn.setStyleSheet("QPushButton{background:#c0392b;color:white;font-weight:bold;padding:10px}")
        self.start_btn.setToolTip("Start the controller")
        self.stop_btn.setToolTip("Stop the controller smoothly (back to Idle)")
        self.estop_btn.setToolTip("Immediately stops the motor (state Fault)")
        self.start_btn.clicked.connect(self.on_start)
        self.stop_btn.clicked.connect(lambda: (self.src.stop(), self._update_enabled()))
        self.estop_btn.clicked.connect(lambda: (self.src.estop(), self._update_enabled()))
        for b in (self.start_btn, self.stop_btn, self.estop_btn):
            lay.addWidget(b)

        self.pause_btn = QW.QPushButton("Pause plots")
        self.pause_btn.setCheckable(True)
        self.pause_btn.setToolTip("Freeze the plots")
        self.pause_btn.setStyleSheet("QPushButton:checked{background:#f39c12;color:white;font-weight:bold}")
        self.pause_btn.toggled.connect(self.on_pause)
        lay.addWidget(self.pause_btn)

        self.state_lbl = QW.QLabel("State: Disconnected")
        self.state_lbl.setStyleSheet("font-size:16px;font-weight:bold")
        lay.addWidget(self.state_lbl)

        g = QW.QGroupBox("Data logging")
        v = QW.QVBoxLayout(g)
        self.rec_btn, self.save_btn = QW.QPushButton("Record"), QW.QPushButton("Save CSV...")
        self.rec_btn.setToolTip("Start / Stop recording")
        self.save_btn.setToolTip("Choose where to save the recording on this computer")
        self.rec_btn.clicked.connect(self.on_record)
        self.save_btn.clicked.connect(self.on_save_csv)
        self.rec_lbl = QW.QLabel("No recording")
        for w in (self.rec_btn, self.save_btn, self.rec_lbl):
            v.addWidget(w)
        lay.addWidget(g)
        lay.addStretch(1)
        return panel

    def _build_plots(self):
        pg.setConfigOptions(antialias=True, background="w", foreground="k")
        self.curves, self.first_plot = [], None
        tabs = QW.QTabWidget()
        for name, specs in TABS:
            tabs.addTab(self._make_tab(specs), name)
        return tabs

    def _make_tab(self, specs):
        w = pg.GraphicsLayoutWidget()
        w.ci.layout.setColumnFixedWidth(1, LEGEND_W)
        for row, sp in enumerate(specs):
            p = w.addPlot(row=row, col=0, title=sp["title"],
                          axisItems={"left": FixedAxis("left", sp.get("dec", DECIMALS)), "bottom": FixedAxis("bottom")})
            p.getAxis("left").setWidth(AXIS_W)
            if "tick" in sp:
                p.getAxis("left").setTickSpacing(sp["tick"], sp["tick"])
            p.setLabel("left", sp["axis"])
            p.showGrid(x=True, y=True, alpha=0.3)
            if sp.get("is_output"):
                self.u_plot = p
            if self.first_plot is None:
                self.first_plot = p
            else:
                p.setXLink(self.first_plot)
            if row == len(specs) - 1:
                p.setLabel("bottom", "Time (s)")
            if "ylim" in sp:
                p.setYRange(*sp["ylim"], padding=0)
                        
            legend = pg.LegendItem()
            legend.setParentItem(p.graphicsItem())
            legend.anchor(itemPos=(0, 0), parentPos=(1, 0), offset=(8, 30))
            entries = []
            for field, color, name in sp["lines"]:
                c = p.plot(pen=pg.mkPen(color, width=2), name=name, connect="finite")
                self.curves.append((field, c))
                entries.append((field, c, name))
                if field in ("pos_ref", "vel_ref"):
                    self.ref_curves[field] = c

            if "min_span" in sp:
                p.enableAutoRange(axis="y", enable=False)
                self.autoscale.append((p, [f for f, _, _ in sp["lines"]], sp["min_span"]))
            
            self.legends.append((legend, entries))
            self._rebuild_legend(legend, entries)
            if "limit" in sp:
                val, label = sp["limit"]
                p.addItem(pg.InfiniteLine(pos=val, angle=0, label=label,
                                          pen=pg.mkPen("r", width=2, style=QC.Qt.DashLine),
                                          labelOpts={"position": 0.95, "color": "r"}))
        return w

    def _rebuild_legend(self, legend, entries):
        for _, c, _ in entries:
            legend.removeItem(c)
        for field, c, name in entries:
            if field not in self.hidden_fields:
                legend.addItem(c, name)

    def _is_speed(self):
        return self.ctrl.currentText() == "Speed"

    def _on_mode(self):
        cur = self.mode.currentText() == "Current"
        if self.u_plot is not None:
            self.u_plot.setTitle("Controller output (current)" if cur else "Controller output (voltage)")
            self.u_plot.setLabel("left", "Current (A)" if cur else "Voltage (V)")

    def _update_value_limit(self):
        speed = self._is_speed()
        self.value.setSuffix(" rad/s" if speed else " rad")
        self.value.setMaximum(SPEED_MAX if speed else PREVIEW_MAX_RAD if self.preview.isChecked() else MAX_STEP_RAD)

    def _on_preview(self):
        self._update_value_limit()
        self._update_enabled()

    def _on_ctrl(self):
        speed = self._is_speed()
        if speed:
            self.ref_type.setCurrentText("Step")
        self._update_value_limit()
        self.hidden_fields = {"pos_ref"} if speed else {"vel_ref"}
        for field, curve in self.ref_curves.items():
            curve.setVisible(field not in self.hidden_fields)
        for legend, entries in self.legends:
            self._rebuild_legend(legend, entries)
        self._update_enabled()

    def _update_enabled(self):
        running = self.src.state == "RUNNING"
        speed, cycle = self._is_speed(), self.ref_type.currentText() == "Cycle"
        connected, busy = getattr(self.src, "connected", False), getattr(self.src, "busy", False)
        for w in (self.mode, self.ctrl, self.cw_btn, self.ccw_btn, self.t0):
            w.setEnabled(not running)
        self.start_btn.setEnabled(not running and connected)
        self.stop_btn.setEnabled(connected)
        self.estop_btn.setEnabled(connected)
        self.ip.setEnabled(not busy)
        if busy and not connected:
            self.conn_btn.setText("Connecting...")
            self.conn_btn.setEnabled(False)
        else:
            self.conn_btn.setText("Disconnect" if busy else "Connect")
            self.conn_btn.setEnabled(hasattr(self.src, "connect_to"))
        self.ref_type.setEnabled(not running and not speed)
        self.value.setEnabled(not cycle)
        self.preview.setEnabled(not running and not speed and not cycle)

    def on_connect(self):
        if self.src.busy:
            self.src.disconnect_from()
            return
        address = self.ip.text().strip()
        pwd, ok = QW.QInputDialog.getText(self, "Connect to the Pi", f"Password for {address}:", QW.QLineEdit.Password)
        if ok and pwd:
            self.src.connect_to(address, pwd)

    def _on_link_changed(self):
        c = self.src.connected
        if c and not self._was_connected:
            for k in FIELDS:
                self.buf[k].clear()
            for _, curve in self.curves:
                curve.setData([], [])
        self._was_connected = c
        self._update_enabled()

    def on_start(self):
        self.src.set_mode(self.mode.currentText().lower())
        self.src.set_ctrl(self.ctrl.currentText().lower())
        self.src.set_dir(1 if self.cw_btn.isChecked() else -1)
        self.src.set_ref(dict(type=self.ref_type.currentText().lower(), value=self.value.value(),
                              t0=self.t0.value(), preview=self.preview.isChecked()))
        self.src.start()
        self._update_enabled()

    def on_pause(self, checked):
        self.paused = checked
        self.pause_btn.setText("Resume plots" if checked else "Pause plots")
        if checked:
            self.statusBar().showMessage("Plotting paused")
        else:
            self._refresh_plots()
            self.statusBar().clearMessage()

    def _update_rec_ui(self):
        self.rec_btn.setText("Stop recording" if self.recording else "Record")
        self.rec_btn.setStyleSheet("background:#c0392b;color:white;font-weight:bold" if self.recording else "")
        self.save_btn.setEnabled(bool(self.rec))
        n = len(self.rec)
        self.rec_lbl.setText("No recording" if n == 0 else
                             f"{'Recording' if self.recording else 'Recorded'}: {n} samples ({n * SAMPLE_DT:.1f} s)")

    def on_record(self):
        if self.recording:
            self.recording = False
        else:
            if self.rec and self.rec_dirty:
                ans = QW.QMessageBox.question(self, "Record",
                                              "The previous recording has not been saved.\nDiscard it and start a new one?")
                if ans != QW.QMessageBox.Yes:
                    return
            self.rec, self.rec_dirty, self.recording = [], False, True
        self._update_rec_ui()

    def on_save_csv(self):
        if not self.rec:
            return
        default = os.path.join(self._last_dir, datetime.now().strftime("MPC_run_%Y-%m-%d_%H-%M-%S.csv"))
        path, _ = QW.QFileDialog.getSaveFileName(self, "Save recording as CSV", default, "CSV files (*.csv);;All files (*)")
        if not path:
            return
        if not os.path.splitext(path)[1]:
            path += ".csv"
        rows = list(self.rec)
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as f:
                w = csv.writer(f)
                w.writerow(CSV_HEADERS)
                for r in rows:
                    w.writerow(["NaN" if v != v else f"{v:.9g}" for v in r])
        except OSError as e:
            QW.QMessageBox.critical(self, "Save failed", f"The file could not be written:\n{e}")
            return
        self.rec_dirty = False
        self._last_dir = os.path.dirname(path) or self._last_dir
        self.statusBar().showMessage(f"Saved {len(rows)} samples to {path}")

    def closeEvent(self, event):
        if self.rec and self.rec_dirty:
            ans = QW.QMessageBox.question(self, "Close", "The recording has not been saved.\nClose the application anyway?")
            if ans != QW.QMessageBox.Yes:
                event.ignore()
                return
        if hasattr(self.src, "shutdown"):
            self.src.shutdown()
        super().closeEvent(event)

    def _refresh_plots(self):
        t = list(self.buf["t"])
        if not t:
            return
        for field, curve in self.curves:
            curve.setData(t, list(self.buf[field]))
        self.first_plot.setXRange(t[-1] - WINDOW_S, t[-1], padding=0)
        self._apply_y_ranges()

    def _apply_y_ranges(self):
        for p, fields, min_span in self.autoscale:
            vals = np.concatenate([np.asarray(self.buf[f], dtype=float) for f in fields
                                if f not in self.hidden_fields])
            vals = vals[np.isfinite(vals)]
            if vals.size == 0:
                continue
            lo, hi = vals.min(), vals.max()
            pad = 0.05 * (hi - lo)
            lo, hi = lo - pad, hi + pad
            if hi - lo < min_span:
                c = (lo + hi) / 2
                lo, hi = c - min_span / 2, c + min_span / 2
            p.setYRange(lo, hi, padding=0)

    def tick(self):
        new = False
        for s in self.src.poll():
            s["total_us"] = s["mpc_us"] + s["spi_us"]
            for k in FIELDS:
                self.buf[k].append(s[k])
            if self.recording:
                self.rec.append([s[k] for k in FIELDS])
                self.rec_dirty = True
            new = True
        self.state_lbl.setText(f"State: {self.src.state.capitalize()}")
        self._update_enabled()
        if self.recording:
            self._update_rec_ui()
        if new and not self.paused:
            self._refresh_plots()


def main():
    app = QW.QApplication(sys.argv)
    win = MainWindow(NullSource() if "--offline" in sys.argv else PiLink())
    win.resize(1100, 650)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

