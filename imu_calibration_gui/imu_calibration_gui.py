"""BMI160 IMU Calibration GUI."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

from calibration import (
    CalibrationData,
    CalibrationStore,
    IMU_MODEL_BMI160 as BMI160_MODEL_ID,
    IMU_MODEL_BNO055 as BNO055_MODEL_ID,
    accel_calibration_valid,
    apply_calibration,
    build_bmi160_export_payload,
    build_bno055_export_payload,
    calibrate_accel_single_face,
    calibrate_accel_six_face,
    calibrate_gyro,
    calibrate_mag_ellipsoid,
    calibrate_mag_hard_iron,
    default_calibration_store_path,
    default_export_filename,
    extract_bmi160_calibration,
    infer_imu_model_from_payload,
    load_bno055_profile,
    mag_quality_metrics,
    normalize_imu_model,
    save_bno055_profile,
)
from device_protocol import format_bno_write_profile_command, format_set_cal_command, parse_cal_response
from filtering import LPF_PRESETS, FilterConfig, LowPassFilter, apply_runtime_filter
from axis_conventions import tilt_from_accel, yaw_rate_from_gyro
from six_face_wizard import open_six_face_wizard
from imu_visualizer import IMUVisualizerWidget
from mag_plot import MagPlotWidget
from serial_client import SerialIMUClient, list_serial_ports
from sensor_units import StreamScale, normalize_sample, normalize_samples
import viz_theme


STATIC_CALIBRATION_SAMPLES = 500
STATIC_CAPTURE_TIMEOUT_SECONDS = 60.0
MAG_CAPTURE_SECONDS = 30.0
IMU_MODEL_BMI160 = "BMI160 / compatible"
IMU_MODEL_BNO055 = "BNO055"


class CalibrationCapture:
    """Non-blocking timed sample capture using tkinter after()."""

    def __init__(self, root: tk.Tk, client: SerialIMUClient):
        self.root = root
        self.client = client
        self._timer_id: str | None = None
        self.active = False

    def cancel(self) -> None:
        if self._timer_id:
            self.root.after_cancel(self._timer_id)
            self._timer_id = None
        self.active = False
        self.client.stop_capture()

    def start(
        self,
        duration_s: float,
        on_tick: callable,
        on_complete: callable,
        on_error: callable | None = None,
        target_samples: int | None = None,
    ) -> None:
        self.cancel()
        self.active = True
        self.client.start_capture()
        end_time = self.root.tk.call("clock", "milliseconds") + duration_s * 1000

        def tick() -> None:
            if not self.active:
                return

            remaining_ms = end_time - float(self.root.tk.call("clock", "milliseconds"))
            remaining_s = max(0.0, remaining_ms / 1000.0)
            sample_count = self.client.capture_count
            on_tick(remaining_s, sample_count)

            target_reached = target_samples is not None and sample_count >= target_samples
            if target_reached or remaining_s <= 0:
                self.active = False
                samples = self.client.stop_capture()
                if not samples:
                    if on_error:
                        on_error("No samples received. Check serial connection and firmware output.")
                    return
                on_complete(samples)
                return

            self._timer_id = self.root.after(100, tick)

        tick()


class BMI160CalibrationApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("IMU Calibration Tool")
        self.root.geometry("1150x860")
        self.root.minsize(1050, 780)

        self.imu_model_var = tk.StringVar(value=IMU_MODEL_BMI160)
        self._active_imu_model = IMU_MODEL_BMI160
        self.store = CalibrationStore(model_label=IMU_MODEL_BMI160)
        self.store.load()
        self.client = SerialIMUClient(on_sample=self._on_sample)
        self.capture = CalibrationCapture(root, self.client)

        self._ui_disabled = False
        self._calibration_mode_on = False
        self._plot_sample_counter = 0
        self._port_scan_running = False
        self._stream_scale = StreamScale()
        self._filter_config = FilterConfig(enabled=True, cutoff_hz=5.0)
        self._lowpass_filter = LowPassFilter(cutoff_hz=self._filter_config.cutoff_hz)
        self.logo_image: object | None = None
        self._bno_profile: dict | None = None
        self._bno_status: dict | None = None
        self._bno_monitor_active = False

        self._build_ui()
        self._refresh_imu_model_ui()
        self._on_filter_settings_changed()
        self._refresh_ports()
        self._update_mag_ui_visibility()
        self._update_display_loop()
        self._refresh_cal_table()
        self._show_window()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        self.root.configure(bg=viz_theme.BG)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=viz_theme.BG)
        style.configure("TFrame", background=viz_theme.BG)
        style.configure("TLabel", background=viz_theme.BG, foreground=viz_theme.TEXT)
        style.configure("TNotebook", background=viz_theme.BG, borderwidth=0)
        style.configure("TNotebook.Tab", padding=[14, 8], font=(viz_theme.FONT, 10))
        style.configure("Title.TLabel", font=(viz_theme.FONT, 18, "bold"), foreground=viz_theme.TEXT)
        style.configure("Section.TLabelframe", background=viz_theme.BG)
        style.configure("Section.TLabelframe.Label", font=(viz_theme.FONT, 11, "bold"), foreground=viz_theme.TEXT)
        style.configure("Status.Connected.TLabel", foreground="#16a34a")
        style.configure("Status.Disconnected.TLabel", foreground="#dc2626")
        style.configure("CalibrationOn.TButton", foreground="#ffffff", background="#16a34a")
        style.map("CalibrationOn.TButton", background=[("active", "#15803d"), ("disabled", "#86efac")])
        style.configure("CalibrationOff.TButton", foreground="#ffffff", background="#dc2626")
        style.map("CalibrationOff.TButton", background=[("active", "#b91c1c"), ("disabled", "#fca5a5")])
        style.configure("Metric.TLabelframe", background=viz_theme.PANEL, relief="flat")
        style.configure("MetricValue.TLabel", font=(viz_theme.FONT, 13, "bold"), foreground=viz_theme.TEXT, background=viz_theme.PANEL)
        style.configure("MetricLabel.TLabel", font=(viz_theme.FONT, 9), foreground=viz_theme.MUTED, background=viz_theme.PANEL)
        style.configure(
            "CalibrationIdle.Horizontal.TProgressbar",
            background=viz_theme.GRID,
            troughcolor=viz_theme.GRID,
            bordercolor=viz_theme.BORDER,
            lightcolor=viz_theme.GRID,
            darkcolor=viz_theme.BORDER,
        )
        style.configure(
            "CalibrationRunning.Horizontal.TProgressbar",
            background="#22c55e",
            troughcolor=viz_theme.GRID,
            bordercolor=viz_theme.BORDER,
            lightcolor="#22c55e",
            darkcolor="#16a34a",
        )

        main = ttk.Frame(self.root, padding=16)
        main.pack(fill="both", expand=True)

        header = ttk.Frame(main)
        header.pack(fill="x", pady=(0, 12))
        ttk.Label(header, text="IMU Calibration Tool", style="Title.TLabel").pack(side="left")
        ttk.Label(
            header,
            text="Real-time IMU streaming · Calibration · Device sync",
            foreground=viz_theme.MUTED,
        ).pack(side="left", padx=(12, 0))

        logo_path = Path(__file__).with_name("assets") / "dronivo_logo.png"
        if logo_path.exists():
            self.logo_image = self._load_logo_image(logo_path)
            if self.logo_image:
                tk.Label(
                    header,
                    image=self.logo_image,
                    bg=viz_theme.BG,
                    borderwidth=0,
                    highlightthickness=0,
                ).pack(side="right")
            else:
                ttk.Label(header, text="DRONIVO", font=(viz_theme.FONT, 13, "bold")).pack(side="right")

        self._build_connection_panel(main)

        self.notebook = ttk.Notebook(main)
        self.notebook.pack(fill="both", expand=True, pady=(0, 10))

        live_tab = ttk.Frame(self.notebook, padding=4)
        cal_tab = ttk.Frame(self.notebook, padding=4)
        self.notebook.add(live_tab, text="Live Visualization")
        self.notebook.add(cal_tab, text="Calibration")
        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)

        self._build_live_view_tab(live_tab)
        self._build_calibration_tab(cal_tab)
        self._build_status_bar(main)

    def _load_logo_image(self, logo_path: Path) -> object | None:
        try:
            from PIL import Image, ImageTk

            image = Image.open(logo_path).convert("RGBA")
            width, height = image.size
            left, top, right, bottom = 0, 0, width, height

            def mostly_dark_edge(box: tuple[int, int, int, int]) -> bool:
                edge = image.crop(box)
                pixels = list(edge.getdata())
                dark_pixels = sum(
                    1 for r, g, b, a in pixels
                    if a > 0 and r < 40 and g < 40 and b < 40
                )
                return dark_pixels / max(1, len(pixels)) > 0.8

            while top < bottom and mostly_dark_edge((left, top, right, top + 1)):
                top += 1
            while bottom > top and mostly_dark_edge((left, bottom - 1, right, bottom)):
                bottom -= 1
            while left < right and mostly_dark_edge((left, top, left + 1, bottom)):
                left += 1
            while right > left and mostly_dark_edge((right - 1, top, right, bottom)):
                right -= 1

            image = image.crop((left, top, right, bottom))
            resampling = getattr(getattr(Image, "Resampling", Image), "LANCZOS")
            image.thumbnail((180, 64), resampling)
            return ImageTk.PhotoImage(image)
        except Exception:
            pass

        try:
            return tk.PhotoImage(file=str(logo_path)).subsample(8, 8)
        except tk.TclError:
            return None

    def _show_window(self) -> None:
        self.root.update_idletasks()

        width = self.root.winfo_width() or 1150
        height = self.root.winfo_height() or 860
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        x = max(0, (screen_w - width) // 2)
        y = max(0, (screen_h - height) // 2)

        self.root.geometry(f"{width}x{height}+{x}+{y}")
        self.root.deiconify()
        self.root.lift()
        self.root.focus_force()
        self.root.attributes("-topmost", True)
        self.root.after(300, lambda: self.root.attributes("-topmost", False))

    def _build_connection_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Serial Connection", padding=12, style="Section.TLabelframe")
        frame.pack(fill="x", pady=(0, 10))

        ttk.Label(frame, text="Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(frame, width=22)
        self.port_combo.grid(row=1, column=0, padx=(0, 8), pady=4, sticky="w")

        ttk.Label(frame, text="Baud").grid(row=0, column=1, sticky="w")
        self.baud_entry = ttk.Entry(frame, width=10)
        self.baud_entry.insert(0, "115200")
        self.baud_entry.grid(row=1, column=1, padx=(0, 8), pady=4, sticky="w")

        ttk.Label(frame, text="IMU model").grid(row=0, column=2, sticky="w")
        self.imu_model_combo = ttk.Combobox(
            frame,
            textvariable=self.imu_model_var,
            values=[IMU_MODEL_BMI160, IMU_MODEL_BNO055],
            width=20,
            state="readonly",
        )
        self.imu_model_combo.grid(row=1, column=2, padx=(0, 8), pady=4, sticky="w")
        self.imu_model_combo.bind("<<ComboboxSelected>>", lambda _e: self._on_imu_model_changed())

        self.refresh_btn = ttk.Button(frame, text="Refresh", command=self._refresh_ports)
        self.refresh_btn.grid(row=1, column=3, padx=4)

        self.connect_btn = ttk.Button(frame, text="Connect", command=self._connect)
        self.connect_btn.grid(row=1, column=4, padx=4)

        self.disconnect_btn = ttk.Button(frame, text="Disconnect", command=self._disconnect, state="disabled")
        self.disconnect_btn.grid(row=1, column=5, padx=4)

        self.cal_mode_btn = ttk.Button(
            frame,
            text="Turn ON Calibration Mode",
            command=self._toggle_calibration_mode,
            style="CalibrationOn.TButton",
            state="disabled",
        )
        self.cal_mode_btn.grid(row=1, column=6, padx=4)

        self.conn_status = ttk.Label(frame, text="Disconnected", style="Status.Disconnected.TLabel")
        self.conn_status.grid(row=1, column=7, padx=(16, 0))

        ttk.Label(
            frame,
            text="CSV formats: ax,ay,az,gx,gy,gz  (DFRobot ESP32)  or  +mx,my,mz for 9-axis",
            foreground="#57606a",
        ).grid(row=2, column=0, columnspan=7, sticky="w", pady=(8, 0))

        self.stream_mode_var = tk.StringVar(value="Stream: —")
        ttk.Label(frame, textvariable=self.stream_mode_var, foreground="#57606a").grid(
            row=2, column=7, sticky="e", pady=(8, 0)
        )

        filter_row = ttk.Frame(frame)
        filter_row.grid(row=3, column=0, columnspan=7, sticky="ew", pady=(8, 0))

        ttk.Label(filter_row, text="Runtime filter:", font=(viz_theme.FONT, 9, "bold")).pack(side="left")
        self.filter_enabled = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            filter_row,
            text="Low-pass on accel + gyro",
            variable=self.filter_enabled,
            command=self._on_filter_settings_changed,
        ).pack(side="left", padx=(8, 12))

        ttk.Label(filter_row, text="Cutoff:", foreground=viz_theme.MUTED).pack(side="left")
        self.filter_cutoff_var = tk.StringVar(value=LPF_PRESETS[1][0])
        self.filter_cutoff_combo = ttk.Combobox(
            filter_row,
            textvariable=self.filter_cutoff_var,
            values=[label for label, _ in LPF_PRESETS],
            width=16,
            state="readonly",
        )
        self.filter_cutoff_combo.pack(side="left", padx=(4, 8))
        self.filter_cutoff_combo.bind("<<ComboboxSelected>>", lambda _e: self._on_filter_settings_changed())

        ttk.Button(filter_row, text="Reset filter", command=self._reset_filter_state).pack(side="left", padx=4)
        self.filter_status_var = tk.StringVar(value="Filter: low-pass 5 Hz · accel + gyro")
        ttk.Label(filter_row, textvariable=self.filter_status_var, foreground=viz_theme.MUTED).pack(
            side="right", padx=4,
        )

    def _build_live_view_tab(self, parent: ttk.Frame) -> None:
        self.imu_visualizer = IMUVisualizerWidget(parent)
        self.imu_visualizer.pack(fill="both", expand=True)

        tilt_row = ttk.Frame(parent)
        tilt_row.pack(fill="x", pady=(10, 0))

        self.live_value_vars: dict[str, tk.StringVar] = {}
        for col, (title, key, color) in enumerate(
            (
                ("Roll (°)  Y+", "roll", viz_theme.AXIS_Y),
                ("Pitch (°)  Z+", "pitch", viz_theme.AXIS_Z),
                ("Yaw rate (°/s)  X+", "yaw", viz_theme.AXIS_X),
            )
        ):
            card = tk.Frame(
                tilt_row, bg=viz_theme.PANEL, highlightbackground=viz_theme.BORDER, highlightthickness=1,
            )
            card.grid(row=0, column=col, padx=4, sticky="nsew")
            tilt_row.columnconfigure(col, weight=1)

            tk.Label(card, text=title, bg=viz_theme.PANEL, fg=viz_theme.MUTED, font=(viz_theme.FONT, 10)).pack(
                anchor="w", padx=12, pady=(10, 0),
            )
            var = tk.StringVar(value="—")
            self.live_value_vars[key] = var
            tk.Label(card, textvariable=var, bg=viz_theme.PANEL, fg=color, font=(viz_theme.FONT, 22, "bold")).pack(
                anchor="w", padx=12, pady=(4, 12),
            )

        metrics = ttk.Frame(parent)
        metrics.pack(fill="x", pady=(10, 0))

        cards = [
            ("Ax (m/s²)", "ax", viz_theme.AXIS_X),
            ("Ay (m/s²)", "ay", viz_theme.AXIS_Y),
            ("Az (m/s²)", "az", viz_theme.AXIS_Z),
            ("Gx (°/s)", "gx", viz_theme.AXIS_X),
            ("Gy (°/s)", "gy", viz_theme.AXIS_Y),
            ("Gz (°/s)", "gz", viz_theme.AXIS_Z),
        ]

        for col, (title, key, color) in enumerate(cards):
            card = tk.Frame(metrics, bg=viz_theme.PANEL, highlightbackground=viz_theme.BORDER, highlightthickness=1)
            card.grid(row=0, column=col, padx=4, sticky="nsew")
            metrics.columnconfigure(col, weight=1)

            tk.Label(card, text=title, bg=viz_theme.PANEL, fg=viz_theme.MUTED, font=(viz_theme.FONT, 9)).pack(
                anchor="w", padx=10, pady=(8, 0)
            )
            var = tk.StringVar(value="—")
            self.live_value_vars[key] = var
            tk.Label(card, textvariable=var, bg=viz_theme.PANEL, fg=color, font=(viz_theme.FONT, 14, "bold")).pack(
                anchor="w", padx=10, pady=(2, 10)
            )

        footer = ttk.Frame(parent)
        footer.pack(fill="x", pady=(8, 0))
        self.rate_var = tk.StringVar(value="Sample rate  —")
        self.total_var = tk.StringVar(value="Samples  0")
        ttk.Label(footer, textvariable=self.rate_var, foreground=viz_theme.MUTED).pack(side="left", padx=(4, 24))
        ttk.Label(footer, textvariable=self.total_var, foreground=viz_theme.MUTED).pack(side="left")

    def _build_calibration_tab(self, parent: ttk.Frame) -> None:
        self._build_calibration_panel(parent)
        self._build_data_section(parent)

    def _build_data_section(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(
            parent, text="Sensor Readout & Stored Calibration", padding=12, style="Section.TLabelframe",
        )
        frame.pack(fill="both", expand=True, pady=(0, 4))
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(0, weight=1)

        readout_col = ttk.Frame(frame)
        readout_col.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        readout_col.rowconfigure(1, weight=1)
        readout_col.columnconfigure(0, weight=1)

        header = ttk.Frame(readout_col)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        header.columnconfigure(0, weight=1)

        ttk.Label(
            header, text="Sensor Readout", font=(viz_theme.FONT, 10, "bold"),
        ).grid(row=0, column=0, sticky="w")

        ttk.Label(
            header,
            text="Raw = LSB · Corrected = calibration only · Filtered = corrected + low-pass",
            foreground=viz_theme.MUTED,
        ).grid(row=1, column=0, sticky="w")

        readout_wrap = ttk.Frame(readout_col)
        readout_wrap.grid(row=1, column=0, sticky="nsew")

        style = ttk.Style()
        style.configure("Readout.Treeview", rowheight=26, font=(viz_theme.FONT, 10))
        style.configure("Readout.Treeview.Heading", font=(viz_theme.FONT, 10, "bold"))

        self.readout_table = ttk.Treeview(
            readout_wrap, columns=("sensor", "x", "y", "z"), show="headings", height=8,
            selectmode="none", style="Readout.Treeview",
        )
        self.readout_table.heading("sensor", text="Sensor")
        self.readout_table.heading("x", text="X")
        self.readout_table.heading("y", text="Y")
        self.readout_table.heading("z", text="Z")
        self.readout_table.column("sensor", width=180, anchor="w")
        self.readout_table.column("x", width=90, anchor="e")
        self.readout_table.column("y", width=90, anchor="e")
        self.readout_table.column("z", width=90, anchor="e")
        self.readout_table.pack(side="left", fill="both", expand=True)

        readout_scroll = ttk.Scrollbar(readout_wrap, orient="vertical", command=self.readout_table.yview)
        readout_scroll.pack(side="right", fill="y")
        self.readout_table.configure(yscrollcommand=readout_scroll.set)

        self._readout_row_ids: dict[str, str] = {}
        for key, label in (
            ("acc_raw", "Accelerometer — Raw"),
            ("gyro_raw", "Gyroscope — Raw"),
            ("mag_raw", "Magnetometer — Raw"),
            ("acc_corr", "Accelerometer — Corrected"),
            ("gyro_corr", "Gyroscope — Corrected"),
            ("mag_corr", "Magnetometer — Corrected"),
            ("acc_filt", "Accelerometer — Filtered (LPF)"),
            ("gyro_filt", "Gyroscope — Filtered (LPF)"),
            ("tilt_filt", "Tilt — Filtered (from accel)"),
        ):
            self._readout_row_ids[key] = self.readout_table.insert(
                "", "end", values=(label, "—", "—", "—"),
            )

        self._last_readout: dict[str, dict[str, float]] = {"raw": {}, "corrected": {}, "filtered": {}}

        plot_frame = ttk.LabelFrame(readout_col, text="Magnetometer 3D", padding=6)
        plot_frame.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        readout_col.rowconfigure(2, weight=2)
        self.mag_plot = MagPlotWidget(plot_frame, max_points=500)
        self.mag_plot.pack(fill="both", expand=True)
        self._mag_plot_frame = plot_frame

        cal_col = ttk.Frame(frame)
        cal_col.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        cal_col.rowconfigure(1, weight=1)
        cal_col.columnconfigure(0, weight=1)

        ttk.Label(
            cal_col, text="Stored Calibration", font=(viz_theme.FONT, 10, "bold"),
        ).grid(row=0, column=0, sticky="w", pady=(0, 4))

        cal_wrap = ttk.Frame(cal_col)
        cal_wrap.grid(row=1, column=0, sticky="nsew")

        self.cal_table = ttk.Treeview(
            cal_wrap, columns=("parameter", "value"), show="headings", height=14,
        )
        self.cal_table.heading("parameter", text="Parameter")
        self.cal_table.heading("value", text="Value")
        self.cal_table.column("parameter", width=160, stretch=False)
        self.cal_table.column("value", width=280, stretch=True)
        self.cal_table.pack(fill="both", expand=True, side="left")

        cal_scroll = ttk.Scrollbar(cal_wrap, orient="vertical", command=self.cal_table.yview)
        cal_scroll.pack(side="right", fill="y")
        self.cal_table.configure(yscrollcommand=cal_scroll.set)

        self.save_path_var = tk.StringVar(value=f"Profile: {self.store.path}")
        ttk.Label(cal_col, textvariable=self.save_path_var, foreground=viz_theme.MUTED).grid(
            row=2, column=0, sticky="w", pady=(8, 0),
        )

        self._refresh_readout_table()

    def _refresh_readout_table(self) -> None:
        raw = self._last_readout.get("raw", {})
        corrected = self._last_readout.get("corrected", {})
        filtered = self._last_readout.get("filtered", {})
        if not raw and not corrected:
            return

        rows = [
            ("acc_raw", "Accelerometer — Raw", raw, ("ax", "ay", "az"), True),
            ("gyro_raw", "Gyroscope — Raw", raw, ("gx", "gy", "gz"), True),
            ("mag_raw", "Magnetometer — Raw", raw, ("mx", "my", "mz"), True),
            ("acc_corr", "Accelerometer — Corrected", corrected, ("ax", "ay", "az"), False),
            ("gyro_corr", "Gyroscope — Corrected", corrected, ("gx", "gy", "gz"), False),
            ("mag_corr", "Magnetometer — Corrected", corrected, ("mx", "my", "mz"), False),
            ("acc_filt", "Accelerometer — Filtered (LPF)", filtered, ("ax", "ay", "az"), False),
            ("gyro_filt", "Gyroscope — Filtered (LPF)", filtered, ("gx", "gy", "gz"), False),
        ]
        for key, label, data, fields, is_raw in rows:
            if not data and key.endswith("_filt"):
                values = ("—", "—", "—")
            elif is_raw:
                values = tuple(f"{data.get(f, 0):+,.0f}" for f in fields)
            else:
                values = tuple(f"{data.get(f, 0):+.3f}" for f in fields)
            self.readout_table.item(self._readout_row_ids[key], values=(label, *values))

        if filtered:
            roll, pitch = tilt_from_accel(filtered.get("ax", 0.0), filtered.get("ay", 0.0), filtered.get("az", 0.0))
            yaw = yaw_rate_from_gyro(filtered.get("gx", 0.0))
            self.readout_table.item(
                self._readout_row_ids["tilt_filt"],
                values=("Tilt — Filtered", f"Roll {roll:+.2f}", f"Pitch {pitch:+.2f}", f"Yaw {yaw:+.2f} °/s"),
            )

    def _build_calibration_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Calibration Workflows", padding=12, style="Section.TLabelframe")
        frame.pack(fill="x", pady=(0, 10))

        model_row = ttk.Frame(frame)
        model_row.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 8))
        ttk.Label(model_row, text="Target IMU model:", font=(viz_theme.FONT, 10, "bold")).pack(side="left")
        self.cal_imu_model_combo = ttk.Combobox(
            model_row,
            textvariable=self.imu_model_var,
            values=[IMU_MODEL_BMI160, IMU_MODEL_BNO055],
            width=24,
            state="readonly",
        )
        self.cal_imu_model_combo.pack(side="left", padx=(8, 12))
        self.cal_imu_model_combo.bind("<<ComboboxSelected>>", lambda _e: self._on_imu_model_changed())
        self.model_profile_var = tk.StringVar(value="")
        ttk.Label(model_row, textvariable=self.model_profile_var, foreground=viz_theme.MUTED).pack(
            side="left", fill="x", expand=True
        )

        instr = ttk.Label(
            frame,
            text="Keep the device connected. Each workflow collects live samples with a countdown.",
            wraplength=900,
        )
        instr.grid(row=1, column=0, columnspan=4, sticky="w", pady=(0, 8))
        self.calibration_instr = instr

        self.gyro_btn = ttk.Button(
            frame, text="1. Gyro — Keep Still (500 samples)", command=self._start_gyro_cal
        )
        self.gyro_btn.grid(row=2, column=0, padx=4, pady=4, sticky="ew")

        self.accel_flat_btn = ttk.Button(
            frame, text="2. Accel — Flat (+Z up, 500 samples)", command=self._start_accel_flat_cal
        )
        self.accel_flat_btn.grid(row=2, column=1, padx=4, pady=4, sticky="ew")

        self.accel_six_btn = ttk.Button(
            frame, text="3. Accel — Six-Face Wizard", command=self._start_accel_six_face
        )
        self.accel_six_btn.grid(row=2, column=2, padx=4, pady=4, sticky="ew")

        self.mag_btn = ttk.Button(
            frame, text="4. Magnetometer — Figure-8 (30s)", command=self._start_mag_cal
        )
        self.mag_btn.grid(row=2, column=3, padx=4, pady=4, sticky="ew")

        self.model_workflow_note = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.model_workflow_note, foreground=viz_theme.MUTED, wraplength=900).grid(
            row=5, column=0, columnspan=4, sticky="w", pady=(8, 0)
        )

        for col in range(4):
            frame.columnconfigure(col, weight=1)

        progress_frame = ttk.Frame(frame)
        progress_frame.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(8, 4))

        self.progress_label = ttk.Label(progress_frame, text="Ready")
        self.progress_label.pack(anchor="w")

        self.progress_bar = ttk.Progressbar(
            progress_frame,
            mode="determinate",
            maximum=100,
            style="CalibrationIdle.Horizontal.TProgressbar",
        )
        self.progress_bar.pack(fill="x", pady=4)

        self.capture_detail = ttk.Label(progress_frame, text="", foreground="#57606a")
        self.capture_detail.pack(anchor="w")

        action_frame = ttk.Frame(frame)
        action_frame.grid(row=4, column=0, columnspan=4, sticky="ew", pady=(8, 0))

        self.cancel_btn = ttk.Button(action_frame, text="Cancel Capture", command=self._cancel_capture, state="disabled")
        self.cancel_btn.pack(side="left", padx=(0, 8))

        ttk.Button(action_frame, text="Export JSON…", command=self._export_calibration).pack(side="left", padx=4)
        ttk.Button(action_frame, text="Import JSON…", command=self._import_calibration).pack(side="left", padx=4)
        ttk.Button(action_frame, text="Reset Calibration", command=self._reset_calibration).pack(side="left", padx=4)

    def _build_status_bar(self, parent: ttk.Frame) -> None:
        self.status_var = tk.StringVar(value="Connect your BMI160 board and start a calibration workflow.")
        ttk.Label(parent, textvariable=self.status_var, relief="sunken", padding=6).pack(fill="x")

    def _on_imu_model_changed(self) -> None:
        selected = self.imu_model_var.get()
        if selected == self._active_imu_model:
            return
        if self.client.is_connected:
            messagebox.showwarning(
                "IMU Model Locked",
                "Disconnect before changing the target IMU model.",
            )
            self.imu_model_var.set(self._active_imu_model)
            return

        self._active_imu_model = selected
        self._load_store_for_current_model()
        self._refresh_imu_model_ui()
        self._refresh_cal_table()

    def _load_store_for_current_model(self) -> None:
        model = self.imu_model_var.get()
        self.store = CalibrationStore(model_label=model)
        self.store.load()
        self._bno_profile = load_bno055_profile(model) if self._is_bno055_mode() else None
        self._bno_status = None

    def _set_imu_model_controls_enabled(self, enabled: bool) -> None:
        state = "readonly" if enabled else "disabled"
        self.imu_model_combo.config(state=state)
        if hasattr(self, "cal_imu_model_combo"):
            self.cal_imu_model_combo.config(state=state)

    def _current_model_id(self) -> str:
        return normalize_imu_model(self.imu_model_var.get())

    def _update_model_profile_label(self) -> None:
        model_id = self._current_model_id()
        export_name = default_export_filename(self.imu_model_var.get())
        if self._is_bno055_mode():
            store_path = default_calibration_store_path(self.imu_model_var.get())
            self.model_profile_var.set(
                f"Profile store: {store_path.name} · export as {export_name}"
            )
        else:
            self.model_profile_var.set(
                f"Profile store: {self.store.path.name} · export as {export_name}"
            )
        self.save_path_var.set(
            f"IMU: {model_id} · profile: {self.store.path if not self._is_bno055_mode() else default_calibration_store_path(self.imu_model_var.get())}"
        )

    def _is_bno055_mode(self) -> bool:
        return self.imu_model_var.get() == IMU_MODEL_BNO055

    def _refresh_imu_model_ui(self) -> None:
        if self._is_bno055_mode():
            self.calibration_instr.config(
                text=(
                    "BNO055 uses Bosch internal fusion calibration. Keep the device still for gyro, "
                    "move through 6 stable orientations for accel, and rotate through random 3D / "
                    "figure-eight motion for mag until SYS/GYR/ACC/MAG all reach 3."
                )
            )
            self.gyro_btn.config(text="1. BNO055 — Initialize / NDOF", command=self._bno_begin)
            self.accel_flat_btn.config(text="2. BNO055 — Monitor calibration status", command=self._bno_monitor_calibration)
            self.accel_six_btn.config(text="3. BNO055 — Read & save device profile", command=self._bno_read_profile_from_device)
            self.mag_btn.config(text="4. BNO055 — Write loaded profile to device", command=self._bno_write_profile_to_device)
            self.model_workflow_note.set(
                "Use the new firmware sketch at firmware/bno055_serial_cal/ for Nano/ESP32. "
                "The GUI will initialize BNO055, poll SYS/GYR/ACC/MAG status, and read/write the "
                "22-byte calibration profile over serial."
            )
            if not self.client.is_connected:
                self.status_var.set("Connect your BNO055-over-serial board and click Initialize / NDOF first.")
        else:
            self.calibration_instr.config(
                text="Keep the device connected. Each workflow collects live samples with a countdown."
            )
            self.gyro_btn.config(text="1. Gyro — Keep Still (500 samples)", command=self._start_gyro_cal)
            self.accel_flat_btn.config(text="2. Accel — Flat (+Z up, 500 samples)", command=self._start_accel_flat_cal)
            self.accel_six_btn.config(text="3. Accel — Six-Face Wizard", command=self._start_accel_six_face)
            self.mag_btn.config(text="4. Magnetometer — Figure-8 (30s)", command=self._start_mag_cal)
            self.model_workflow_note.set(
                "BMI160 calibration is saved locally and exported as IMU_Calibration_BMI160.json."
            )
            if not self.client.is_connected:
                self.status_var.set("Select BMI160, connect your board, turn ON calibration mode, then run workflows.")
        self._update_model_profile_label()
        self._apply_mode_button_states()

    def _apply_mode_button_states(self) -> None:
        base_state = "disabled" if self._ui_disabled else "normal"
        if self._is_bno055_mode():
            if not self.client.is_connected:
                base_state = "disabled"
            for btn in (self.gyro_btn, self.accel_flat_btn, self.accel_six_btn, self.mag_btn):
                btn.config(state=base_state)
            return

        self.gyro_btn.config(state=base_state)
        self.accel_flat_btn.config(state=base_state)
        self.accel_six_btn.config(state=base_state)
        self.mag_btn.config(state="normal" if self.client.has_magnetometer and not self._ui_disabled else "disabled")

    def _refresh_ports(self) -> None:
        if self._port_scan_running:
            return
        self._port_scan_running = True
        self.refresh_btn.config(state="disabled")

        def worker() -> None:
            ports = list_serial_ports()
            self.root.after(0, lambda: self._apply_ports(ports))

        threading.Thread(target=worker, daemon=True).start()

    def _apply_ports(self, ports: list[str]) -> None:
        self._port_scan_running = False
        self.refresh_btn.config(state="normal")
        self.port_combo["values"] = ports
        current = self.port_combo.get()
        if current and current in ports:
            return
        if ports:
            self.port_combo.set(ports[0])

    def _on_tab_changed(self, _event=None) -> None:
        on_live_tab = self.notebook.index(self.notebook.select()) == 0
        if on_live_tab and self.client.is_connected:
            self.imu_visualizer.set_running(True)
        else:
            self.imu_visualizer.set_running(False)

    def _connect(self) -> None:
        port = self.port_combo.get().strip()
        if not port:
            messagebox.showerror("Connection", "Select a serial port.")
            return
        try:
            baud = int(self.baud_entry.get())
            self.client.connect(port, baud)
        except Exception as exc:
            messagebox.showerror("Connection Error", str(exc))
            return

        self.conn_status.config(text=f"Connected · {port}", style="Status.Connected.TLabel")
        self.connect_btn.config(state="disabled")
        self.disconnect_btn.config(state="normal")
        self._set_imu_model_controls_enabled(False)
        self._calibration_mode_on = False
        self._refresh_calibration_mode_button()
        self._apply_mode_button_states()
        self._stream_scale = StreamScale()
        self._lowpass_filter.reset()
        self.imu_visualizer.clear()
        self.imu_visualizer.set_running(self.notebook.index(self.notebook.select()) == 0)
        self.status_var.set(f"Connected to {port} at {baud} baud.")

        def ping_worker() -> None:
            ok = self.client.ping_device()
            self.root.after(0, lambda: self._on_ping_done(port, ok))

        threading.Thread(target=ping_worker, daemon=True).start()

    def _on_ping_done(self, port: str, ok: bool) -> None:
        if not self.client.is_connected:
            return
        if ok:
            self.status_var.set(f"Connected to {port} — firmware responded to PING.")
        else:
            self.status_var.set(f"Connected to {port} — streaming DFRobot CSV.")

    def _disconnect(self) -> None:
        self.capture.cancel()
        self.client.disconnect()
        self.conn_status.config(text="Disconnected", style="Status.Disconnected.TLabel")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        self._set_imu_model_controls_enabled(True)
        self._calibration_mode_on = False
        self._refresh_calibration_mode_button()
        self._apply_mode_button_states()
        self.imu_visualizer.set_running(False)
        self._set_capture_ui(active=False)
        self.status_var.set("Disconnected.")

    def _on_close(self) -> None:
        self.capture.cancel()
        self.imu_visualizer.destroy()
        self.client.disconnect()
        self.root.destroy()

    def _normalize_samples(self, samples: list[dict[str, float]]) -> list[dict[str, float]]:
        return normalize_samples(samples, self._stream_scale)

    def _sync_filter_config(self) -> None:
        label = self.filter_cutoff_var.get()
        cutoff_hz = next((hz for l, hz in LPF_PRESETS if l == label), 5.0)
        self._filter_config.enabled = self.filter_enabled.get()
        self._filter_config.cutoff_hz = cutoff_hz
        self._lowpass_filter.configure(cutoff_hz)

    def _on_filter_settings_changed(self) -> None:
        prev_cutoff = self._filter_config.cutoff_hz
        prev_enabled = self._filter_config.enabled
        self._sync_filter_config()
        if self._filter_config.cutoff_hz != prev_cutoff or self._filter_config.enabled != prev_enabled:
            self._lowpass_filter.reset()
        state = "on" if self._filter_config.enabled else "off"
        self.filter_status_var.set(
            f"Filter: low-pass {self._filter_config.cutoff_hz:g} Hz · accel + gyro · {state}"
        )

    def _reset_filter_state(self) -> None:
        self._lowpass_filter.reset()
        self.filter_status_var.set(
            f"Filter: reset · low-pass {self._filter_config.cutoff_hz:g} Hz · "
            f"{'on' if self._filter_config.enabled else 'off'}"
        )

    def _process_sample(
        self, data: dict[str, float], *, timestamp: float | None = None,
    ) -> tuple[dict[str, float], dict[str, float], dict[str, float]]:
        physical = self._normalize_data(data)
        corrected = apply_calibration(physical, self.store.data)
        filtered = apply_runtime_filter(
            corrected, self._lowpass_filter, self._filter_config, timestamp=timestamp,
        )
        return physical, corrected, filtered

    def _on_sample(self, sample) -> None:
        if not self.client.is_connected:
            return

        def push_to_visualizer() -> None:
            if self.notebook.index(self.notebook.select()) != 0:
                return
            _, _, display = self._process_sample(sample.as_dict(), timestamp=sample.timestamp)
            self.imu_visualizer.add_sample(
                sample.timestamp,
                display["ax"], display["ay"], display["az"],
                display["gx"], display["gy"], display["gz"],
            )

        self.root.after(0, push_to_visualizer)

    def _update_mag_ui_visibility(self) -> None:
        has_mag = self.client.has_magnetometer
        if not self._is_bno055_mode():
            state = "normal" if has_mag and not self._ui_disabled else "disabled"
            self.mag_btn.config(state=state)
        if has_mag:
            self._mag_plot_frame.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        else:
            self._mag_plot_frame.grid_remove()

    def _normalize_data(self, data: dict[str, float]) -> dict[str, float]:
        return normalize_sample(data, self._stream_scale)

    def _update_display_loop(self) -> None:
        sample = self.client.latest
        if sample:
            data = sample.as_dict()
            physical, corrected, filtered = self._process_sample(data, timestamp=sample.timestamp)

            self._last_readout["raw"] = data
            self._last_readout["corrected"] = corrected
            self._last_readout["filtered"] = filtered
            self._refresh_readout_table()

            self.live_value_vars["ax"].set(f"{filtered['ax']:+.3f}")
            self.live_value_vars["ay"].set(f"{filtered['ay']:+.3f}")
            self.live_value_vars["az"].set(f"{filtered['az']:+.3f}")
            self.live_value_vars["gx"].set(f"{filtered['gx']:+.3f}")
            self.live_value_vars["gy"].set(f"{filtered['gy']:+.3f}")
            self.live_value_vars["gz"].set(f"{filtered['gz']:+.3f}")

            roll, pitch = tilt_from_accel(filtered["ax"], filtered["ay"], filtered["az"])
            self.live_value_vars["roll"].set(f"{roll:+.2f}")
            self.live_value_vars["pitch"].set(f"{pitch:+.2f}")
            self.live_value_vars["yaw"].set(f"{yaw_rate_from_gyro(filtered['gx']):+.2f}")

            self.rate_var.set(f"Sample rate  {self.client.sample_rate_hz:.1f} Hz")
            self.total_var.set(f"Samples  {self.client.total_samples:,}")
            scale_note = f" · {self._stream_scale.label}" if self._stream_scale.is_raw_lsb else ""
            cal_note = ""
            if self.store.data.saved_at and not accel_calibration_valid(self.store.data):
                cal_note = " · Accel cal invalid (re-run 2a/2b)"
            self.stream_mode_var.set(f"Stream: {self.client.stream_label}{scale_note}{cal_note}")

            self._plot_sample_counter += 1

            if self.client.has_magnetometer:
                if self._plot_sample_counter % 3 == 0:
                    raw_mag = (physical["mx"], physical["my"], physical["mz"])
                    corr_mag = (corrected["mx"], corrected["my"], corrected["mz"])
                    self.mag_plot.add_sample(raw_mag, corr_mag)
                    if self._plot_sample_counter % 15 == 0:
                        self.mag_plot.redraw()

            if self._plot_sample_counter % 30 == 0:
                self._update_mag_ui_visibility()

        self.root.after(250, self._update_display_loop)

    def _set_capture_ui(self, active: bool) -> None:
        self._ui_disabled = active
        state = "disabled" if active else "normal"
        for btn in (
            self.connect_btn, self.refresh_btn, self.cal_mode_btn,
        ):
            btn.config(state=state)
        self._apply_mode_button_states()
        if not active and self.client.is_connected:
            self.connect_btn.config(state="disabled")
            self._refresh_calibration_mode_button()
        elif not active:
            self.cal_mode_btn.config(state="disabled")
        self.cancel_btn.config(state="normal" if active else "disabled")
        self.progress_bar.config(
            style=(
                "CalibrationRunning.Horizontal.TProgressbar"
                if active
                else "CalibrationIdle.Horizontal.TProgressbar"
            )
        )
        if not active:
            self.progress_bar["value"] = 0
            self.progress_label.config(text="Ready")
            self.capture_detail.config(text="")

    def _refresh_calibration_mode_button(self) -> None:
        if not self.client.is_connected:
            self.cal_mode_btn.config(
                text="Turn ON Calibration Mode",
                state="disabled",
                style="CalibrationOn.TButton",
            )
            return

        if self._calibration_mode_on:
            self.cal_mode_btn.config(
                text="Turn OFF Calibration Mode",
                state="normal",
                style="CalibrationOff.TButton",
            )
        else:
            self.cal_mode_btn.config(
                text="Turn ON Calibration Mode",
                state="normal",
                style="CalibrationOn.TButton",
            )

    def _toggle_calibration_mode(self) -> None:
        if not self._ensure_connected():
            return

        self._calibration_mode_on = not self._calibration_mode_on
        command = "CAL_MODE_ON" if self._calibration_mode_on else "CAL_MODE_OFF"
        status = (
            "Arduino calibration mode command sent. IMU data should stream now."
            if self._calibration_mode_on
            else "Arduino calibration mode OFF command sent. IMU streaming should stop."
        )
        self._refresh_calibration_mode_button()
        self.status_var.set(status)

        def worker() -> None:
            try:
                self.client.write_command(command)
            except RuntimeError as exc:
                self.root.after(0, lambda: self.status_var.set(f"Calibration mode command failed: {exc}"))

        threading.Thread(target=worker, daemon=True).start()

    def _bno_begin(self) -> None:
        if not self._ensure_connected():
            return
        self.status_var.set("Initializing BNO055 and entering NDOF mode…")

        def worker() -> None:
            try:
                response = self.client.send_command("BNO_BEGIN", timeout=5.0)
            except RuntimeError as exc:
                self.root.after(0, lambda: messagebox.showerror("BNO055 Init Failed", str(exc)))
                return
            self.root.after(0, lambda: self._on_bno_begin_response(response))

        threading.Thread(target=worker, daemon=True).start()

    def _on_bno_begin_response(self, response: str | None) -> None:
        kind, payload = parse_cal_response(response or "")
        if kind == "BNO_BEGIN" and payload:
            address = payload.get("address", 0)
            self.status_var.set(f"BNO055 initialized at 0x{int(address):02X} in NDOF mode.")
            messagebox.showinfo(
                "BNO055 Ready",
                "BNO055 initialized successfully.\n\n"
                "Next:\n"
                "1. Keep the device still for gyro calibration.\n"
                "2. Move through 6 stable orientations for accelerometer calibration.\n"
                "3. Move in figure-eight / random 3D motion for magnetometer calibration.",
            )
            return
        if kind == "BNO_ERR" and payload:
            messagebox.showerror("BNO055 Init Failed", str(payload.get("message", "Unknown BNO055 error")))
            self.status_var.set("BNO055 initialization failed.")
            return
        messagebox.showerror("BNO055 Init Failed", f"Unexpected response: {response or 'timeout'}")
        self.status_var.set("BNO055 initialization failed.")

    def _bno_monitor_calibration(self) -> None:
        if not self._ensure_connected():
            return
        if self._bno_monitor_active:
            return
        self._bno_monitor_active = True
        self._set_capture_ui(active=True)
        self.progress_label.config(text="BNO055: monitoring SYS/GYR/ACC/MAG calibration…")
        self.status_var.set("Keep the device still, then rotate through 6 faces and figure-eight motion.")

        def worker() -> None:
            while self._bno_monitor_active and self.client.is_connected:
                try:
                    response = self.client.send_command("BNO_GET_STATUS", timeout=3.0)
                except RuntimeError as exc:
                    self.root.after(0, lambda: self._finish_bno_monitor(error=str(exc)))
                    return
                self.root.after(0, lambda resp=response: self._update_bno_monitor(resp))
                time.sleep(0.5)

        threading.Thread(target=worker, daemon=True).start()

    def _update_bno_monitor(self, response: str | None) -> None:
        if not self._bno_monitor_active:
            return
        kind, payload = parse_cal_response(response or "")
        if kind == "BNO_ERR" and payload:
            self._finish_bno_monitor(error=str(payload.get("message", "Unknown BNO055 error")))
            return
        if kind != "BNO_STATUS" or not payload:
            self._finish_bno_monitor(error=f"Unexpected response: {response or 'timeout'}")
            return

        self._bno_status = payload
        sys_v = int(payload.get("sys", 0))
        gyr_v = int(payload.get("gyr", 0))
        acc_v = int(payload.get("acc", 0))
        mag_v = int(payload.get("mag", 0))
        pct = ((sys_v + gyr_v + acc_v + mag_v) / 12.0) * 100.0
        self.progress_bar["value"] = pct
        self.capture_detail.config(text=f"SYS={sys_v}/3  GYR={gyr_v}/3  ACC={acc_v}/3  MAG={mag_v}/3")
        self.status_var.set(
            "Gyro: keep still. Accel: use 6 stable orientations. Mag: rotate in random 3D / figure-eight motion."
        )
        if sys_v == 3 and gyr_v == 3 and acc_v == 3 and mag_v == 3:
            self._finish_bno_monitor(success=True)

    def _finish_bno_monitor(self, success: bool = False, error: str | None = None) -> None:
        self._bno_monitor_active = False
        self._set_capture_ui(active=False)
        if error:
            self.status_var.set("BNO055 calibration monitor failed.")
            messagebox.showerror("BNO055 Calibration", error)
            return
        if success:
            self.status_var.set("BNO055 fully calibrated. Read and save the device profile now.")
            messagebox.showinfo(
                "BNO055 Calibration Complete",
                "SYS, GYR, ACC, and MAG all reached 3.\n\n"
                "Now click 'Read & save device profile' to capture the 22-byte profile.",
            )

    def _bno_read_profile_from_device(self) -> None:
        if not self._ensure_connected():
            return

        def worker() -> None:
            try:
                response = self.client.send_command("BNO_READ_PROFILE", timeout=5.0)
            except RuntimeError as exc:
                self.root.after(0, lambda: messagebox.showerror("BNO055 Profile", str(exc)))
                return
            self.root.after(0, lambda: self._on_bno_profile_read(response))

        threading.Thread(target=worker, daemon=True).start()

    def _on_bno_profile_read(self, response: str | None) -> None:
        kind, payload = parse_cal_response(response or "")
        if kind == "BNO_PROFILE" and payload:
            self._bno_profile = payload
            save_bno055_profile(payload, self.imu_model_var.get())
            self._refresh_cal_table()
            self.status_var.set("BNO055 profile read from device.")
            messagebox.showinfo("BNO055 Profile", "BNO055 calibration profile read from device.\nUse Export JSON to save it.")
            return
        if kind == "BNO_ERR" and payload:
            messagebox.showerror("BNO055 Profile", str(payload.get("message", "Unknown BNO055 error")))
            return
        messagebox.showerror("BNO055 Profile", f"Unexpected response: {response or 'timeout'}")

    def _bno_write_profile_to_device(self) -> None:
        if not self._ensure_connected():
            return
        if not self._bno_profile:
            messagebox.showerror("BNO055 Profile", "Import or read a BNO055 profile first.")
            return

        cmd = format_bno_write_profile_command(self._bno_profile)

        def worker() -> None:
            try:
                response = self.client.send_command(cmd, timeout=5.0)
            except RuntimeError as exc:
                self.root.after(0, lambda: messagebox.showerror("BNO055 Write Failed", str(exc)))
                return
            self.root.after(0, lambda: self._on_bno_profile_written(response))

        threading.Thread(target=worker, daemon=True).start()

    def _on_bno_profile_written(self, response: str | None) -> None:
        kind, payload = parse_cal_response(response or "")
        if kind == "BNO_WRITE_OK":
            self.status_var.set("BNO055 calibration profile written to device.")
            messagebox.showinfo("BNO055 Write Complete", "BNO055 calibration profile written to device.")
            return
        if kind == "BNO_ERR" and payload:
            messagebox.showerror("BNO055 Write Failed", str(payload.get("message", "Unknown BNO055 error")))
            return
        messagebox.showerror("BNO055 Write Failed", f"Unexpected response: {response or 'timeout'}")

    def _cancel_capture(self) -> None:
        self._bno_monitor_active = False
        self.capture.cancel()
        self._set_capture_ui(active=False)
        self.status_var.set("Capture cancelled.")

    def _ensure_connected(self) -> bool:
        if not self.client.is_connected:
            model_name = "BNO055" if self._is_bno055_mode() else "BMI160"
            messagebox.showerror("Not Connected", f"Connect to the {model_name} serial port first.")
            return False
        return True

    def _ensure_calibration_mode(self) -> bool:
        if not self._ensure_connected():
            return False
        if self._is_bno055_mode():
            return True
        if not self._calibration_mode_on:
            messagebox.showerror(
                "Calibration Mode",
                "Click Turn ON Calibration Mode before starting calibration.",
            )
            return False
        return True

    def _run_timed_capture(
        self,
        duration_s: float,
        instruction: str,
        min_samples: int,
        on_success: callable,
        target_samples: int | None = None,
    ) -> None:
        if self._is_bno055_mode():
            messagebox.showinfo(
                "BNO055 Workflow",
                "BNO055 uses its own internal fusion calibration. Use the direct-I2C helper in "
                "bno055_i2c.py to run begin(), wait_for_full_calibration(), and save/load the 22-byte profile.",
            )
            return
        if not self._ensure_calibration_mode():
            return

        self._set_capture_ui(active=True)
        self.progress_label.config(text=instruction)
        self.status_var.set(instruction)

        def on_tick(remaining_s: float, sample_count: int) -> None:
            if target_samples:
                elapsed_pct = min(100, (sample_count / target_samples) * 100)
            else:
                elapsed_pct = (1.0 - remaining_s / duration_s) * 100
            self.progress_bar["value"] = elapsed_pct
            target_text = f" / {target_samples}" if target_samples else ""
            self.capture_detail.config(
                text=f"Time left: {remaining_s:4.1f}s   ·   Samples: {sample_count}{target_text}"
            )

        def on_complete(samples: list[dict]) -> None:
            self._set_capture_ui(active=False)
            if len(samples) < min_samples:
                messagebox.showerror(
                    "Insufficient Data",
                    f"Only {len(samples)} samples collected (need ≥ {min_samples}). "
                    "Check firmware is streaming data.",
                )
                return
            on_success(samples)

        def on_error(msg: str) -> None:
            self._set_capture_ui(active=False)
            messagebox.showerror("Capture Failed", msg)

        self.capture.start(duration_s, on_tick, on_complete, on_error, target_samples=target_samples)

    def _start_gyro_cal(self) -> None:
        def finish(samples: list[dict]) -> None:
            samples = self._normalize_samples(samples)
            offset = calibrate_gyro(samples)
            self.store.data.gyro_offset = offset
            self.store.save()
            self._refresh_cal_table()
            messagebox.showinfo(
                "Gyro Calibration Complete",
                f"Zero-rate offsets (deg/s):\n"
                f"  X = {offset['x']:+.6f}\n"
                f"  Y = {offset['y']:+.6f}\n"
                f"  Z = {offset['z']:+.6f}",
            )
            self.status_var.set("Gyro calibration saved.")

        self._run_timed_capture(
            STATIC_CAPTURE_TIMEOUT_SECONDS,
            "Gyro: keep the IMU completely still until 500 samples are collected…",
            min_samples=STATIC_CALIBRATION_SAMPLES,
            on_success=finish,
            target_samples=STATIC_CALIBRATION_SAMPLES,
        )

    def _start_accel_flat_cal(self) -> None:
        def finish(samples: list[dict]) -> None:
            samples = self._normalize_samples(samples)
            offset, scale = calibrate_accel_single_face(samples, gravity_axis="z")
            self.store.data.accel_offset = offset
            self.store.data.accel_scale = scale
            self.store.save()
            self._refresh_cal_table()
            messagebox.showinfo(
                "Accelerometer Calibration Complete",
                f"Flat (+Z up) offsets (m/s²):\n"
                f"  X = {offset['x']:+.6f}\n"
                f"  Y = {offset['y']:+.6f}\n"
                f"  Z = {offset['z']:+.6f}\n\n"
                "For best accuracy, use the Six-Face Wizard.",
            )
            self.status_var.set("Accelerometer (flat) calibration saved.")

        self._run_timed_capture(
            STATIC_CAPTURE_TIMEOUT_SECONDS,
            "Accel: place IMU flat with +Z pointing up until 500 samples are collected…",
            min_samples=STATIC_CALIBRATION_SAMPLES,
            on_success=finish,
            target_samples=STATIC_CALIBRATION_SAMPLES,
        )

    def _start_accel_six_face(self) -> None:
        if self._is_bno055_mode():
            messagebox.showinfo(
                "BNO055 Workflow",
                "Six-face sample fitting is for raw IMUs like BMI160. For BNO055 use the internal "
                "calibration-status workflow and save/load the device profile with bno055_i2c.py.",
            )
            return
        if not self._ensure_calibration_mode():
            return

        self._set_capture_ui(active=True)

        def on_complete(face_samples: list[list[dict]]) -> None:
            self._set_capture_ui(active=False)
            self._finish_accel_six_face_from_samples(face_samples)

        def on_cancel() -> None:
            self._set_capture_ui(active=False)
            self.status_var.set("Six-face calibration cancelled.")

        open_six_face_wizard(
            self.root, self.client, self.capture,
            on_complete=on_complete,
            on_cancel=on_cancel,
        )

    def _finish_accel_six_face_from_samples(self, face_samples: list[list[dict]]) -> None:
        try:
            face_samples = [self._normalize_samples(face) for face in face_samples]
            offset, scale = calibrate_accel_six_face(face_samples)
        except ValueError as exc:
            messagebox.showerror("Six-Face Calibration Failed", str(exc))
            return

        self.store.data.accel_offset = offset
        self.store.data.accel_scale = scale
        self.store.save()
        self._refresh_cal_table()

        messagebox.showinfo(
            "Six-Face Accelerometer Calibration Complete",
            f"Offset (raw):\n"
            f"  X = {offset['x']:+.2f}, Y = {offset['y']:+.2f}, Z = {offset['z']:+.2f}\n\n"
            f"Scale:\n"
            f"  X = {scale['x']:.4f}, Y = {scale['y']:.4f}, Z = {scale['z']:.4f}",
        )
        self.status_var.set("Six-face accelerometer calibration saved.")

    def _start_mag_cal(self) -> None:
        self.mag_plot.clear()

        def finish(samples: list[dict]) -> None:
            samples = self._normalize_samples(samples)
            try:
                offset, soft_iron = calibrate_mag_ellipsoid(samples)
            except ValueError:
                offset = calibrate_mag_hard_iron(samples)
                soft_iron = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

            self.store.data.mag_offset = offset
            self.store.data.mag_soft_iron = soft_iron
            self.store.save()
            self._refresh_cal_table()

            metrics = mag_quality_metrics(samples, self.store.data)
            messagebox.showinfo(
                "Magnetometer Calibration Complete",
                f"Hard-iron offset (µT):\n"
                f"  X = {offset['x']:+.4f}, Y = {offset['y']:+.4f}, Z = {offset['z']:+.4f}\n\n"
                f"Quality (lower radius std is better):\n"
                f"  Radius mean = {metrics['radius_mean']:.2f} µT\n"
                f"  Radius std  = {metrics['radius_std']:.4f} µT",
            )
            self.status_var.set("Magnetometer calibration saved.")
            self.mag_plot.redraw()

        self._run_timed_capture(
            MAG_CAPTURE_SECONDS,
            "Mag: rotate IMU in smooth figure-8 patterns in all orientations…",
            min_samples=100,
            on_success=finish,
        )

    def _refresh_cal_table(self) -> None:
        for item in self.cal_table.get_children():
            self.cal_table.delete(item)

        if self._is_bno055_mode():
            rows: list[tuple[str, str]] = []
            if self._bno_status:
                rows.extend(
                    [
                        ("BNO055 SYS cal", f"{int(self._bno_status.get('sys', 0))}/3"),
                        ("BNO055 GYR cal", f"{int(self._bno_status.get('gyr', 0))}/3"),
                        ("BNO055 ACC cal", f"{int(self._bno_status.get('acc', 0))}/3"),
                        ("BNO055 MAG cal", f"{int(self._bno_status.get('mag', 0))}/3"),
                    ]
                )
            if self._bno_profile:
                rows.append(("Profile address", f"0x{int(self._bno_profile.get('address', 0)):02X}"))
                rows.append(("Axis map config", f"0x{int(self._bno_profile.get('axis_map_config', 0)):02X}"))
                rows.append(("Axis map sign", f"0x{int(self._bno_profile.get('axis_map_sign', 0)):02X}"))
                profile_bytes = self._bno_profile.get("calibration_bytes", [])
                rows.append(("Profile bytes", " ".join(f"{int(v):02X}" for v in profile_bytes)))
            if not rows:
                rows.append(("Status", "No BNO055 profile loaded yet — initialize and monitor calibration first"))
            for param, value in rows:
                self.cal_table.insert("", "end", values=(param, value))
            self._update_model_profile_label()
            return

        cal = self.store.data
        rows: list[tuple[str, str]] = [
            ("Gyro offset X", f"{cal.gyro_offset['x']:+.4f}"),
            ("Gyro offset Y", f"{cal.gyro_offset['y']:+.4f}"),
            ("Gyro offset Z", f"{cal.gyro_offset['z']:+.4f}"),
            ("Accel offset X", f"{cal.accel_offset['x']:+.4f}"),
            ("Accel offset Y", f"{cal.accel_offset['y']:+.4f}"),
            ("Accel offset Z", f"{cal.accel_offset['z']:+.4f}"),
            ("Accel scale X", f"{cal.accel_scale['x']:.6f}"),
            ("Accel scale Y", f"{cal.accel_scale['y']:.6f}"),
            ("Accel scale Z", f"{cal.accel_scale['z']:.6f}"),
            ("Mag offset X", f"{cal.mag_offset['x']:+.4f}"),
            ("Mag offset Y", f"{cal.mag_offset['y']:+.4f}"),
            ("Mag offset Z", f"{cal.mag_offset['z']:+.4f}"),
        ]
        for i, row in enumerate(cal.mag_soft_iron):
            rows.append((f"Mag soft-iron row {i}", ", ".join(f"{v:+.4f}" for v in row)))

        if cal.saved_at:
            rows.append(("Last saved", cal.saved_at))
        else:
            rows.append(("Status", "No calibration saved yet — run a workflow above"))

        for param, value in rows:
            self.cal_table.insert("", "end", values=(param, value))

        self._update_model_profile_label()

    def _confirm_import_model(self, payload: dict) -> bool:
        file_model = infer_imu_model_from_payload(payload)
        selected_model = self._current_model_id()
        if file_model == selected_model:
            return True
        return messagebox.askyesno(
            "IMU Model Mismatch",
            (
                f"This file is for {file_model}, but you have {selected_model} selected.\n\n"
                f"Switch to {file_model} and import this profile?"
            ),
        )

    def _switch_to_model_for_import(self, model_id: str) -> None:
        label = IMU_MODEL_BNO055 if model_id == BNO055_MODEL_ID else IMU_MODEL_BMI160
        self.imu_model_var.set(label)
        self._active_imu_model = label
        self._load_store_for_current_model()
        self._refresh_imu_model_ui()

    def _export_calibration(self) -> None:
        export_name = default_export_filename(self.imu_model_var.get())
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON", "*.json")],
            initialfile=export_name,
        )
        if not path:
            return
        if self._is_bno055_mode():
            if not self._bno_profile:
                messagebox.showerror("Export Failed", "Read or import a BNO055 profile first.")
                return
            payload = build_bno055_export_payload(self._bno_profile, self.imu_model_var.get())
        else:
            payload = build_bmi160_export_payload(self.store.data, self.imu_model_var.get())
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        messagebox.showinfo("Exported", f"{self._current_model_id()} calibration exported to:\n{path}")

    def _import_calibration(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path:
            return
        with open(path, "r", encoding="utf-8") as f:
            payload = json.load(f)

        file_model = infer_imu_model_from_payload(payload)
        if file_model != self._current_model_id():
            if not self._confirm_import_model(payload):
                return
            if self.client.is_connected:
                messagebox.showwarning(
                    "IMU Model Locked",
                    "Disconnect before switching IMU model to import this file.",
                )
                return
            self._switch_to_model_for_import(file_model)

        if self._is_bno055_mode():
            profile = payload.get("bno055_profile", payload)
            if "calibration_bytes" not in profile:
                messagebox.showerror("Import Failed", "This JSON does not contain a BNO055 profile.")
                return
            self._bno_profile = profile
            save_bno055_profile(profile, self.imu_model_var.get())
            self._refresh_cal_table()
            messagebox.showinfo("Imported", f"BNO055 profile loaded from:\n{path}")
        else:
            self.store.data = extract_bmi160_calibration(payload)
            self.store.save()
            self._refresh_cal_table()
            messagebox.showinfo("Imported", f"BMI160 calibration loaded from:\n{path}")

    def _reset_calibration(self) -> None:
        if not messagebox.askyesno("Reset", f"Clear all {self._current_model_id()} calibration values?"):
            return
        if self._is_bno055_mode():
            self._bno_profile = None
            self._bno_status = None
            store_path = default_calibration_store_path(self.imu_model_var.get())
            if store_path.exists():
                store_path.unlink()
        else:
            from calibration import CalibrationData
            self.store.data = CalibrationData()
            self.store.save()
        self._refresh_cal_table()
        self.status_var.set(f"{self._current_model_id()} calibration reset to defaults.")

    def _write_calibration_to_device(self) -> None:
        if not self._ensure_connected():
            return

        cmd = format_set_cal_command(self.store.data)
        try:
            response = self.client.send_command(cmd, timeout=5.0)
        except RuntimeError as exc:
            messagebox.showerror("Write Failed", str(exc))
            return

        if response == "CAL_OK":
            messagebox.showinfo(
                "Write Complete",
                "Calibration written to device and saved in MCU flash/EEPROM.",
            )
            self.status_var.set("Calibration pushed to device successfully.")
        else:
            messagebox.showerror(
                "Write Failed",
                f"Device did not confirm calibration.\nResponse: {response or 'timeout'}",
            )

    def _read_calibration_from_device(self) -> None:
        if not self._ensure_connected():
            return

        try:
            response = self.client.send_command("GET_CAL", timeout=5.0)
        except RuntimeError as exc:
            messagebox.showerror("Read Failed", str(exc))
            return

        kind, payload = parse_cal_response(response or "")
        if kind != "CAL" or not payload:
            messagebox.showerror(
                "Read Failed",
                f"Could not read calibration from device.\nResponse: {response or 'timeout'}",
            )
            return

        self.store.data = CalibrationData.from_dict(payload)
        self.store.save()
        self._refresh_cal_table()
        messagebox.showinfo("Read Complete", "Calibration loaded from device and saved locally.")
        self.status_var.set("Calibration read from device.")


def main() -> None:
    root = tk.Tk()
    BMI160CalibrationApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
