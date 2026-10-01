"""Real-time flight telemetry dashboard for MCU_Cuadcopter.

The application decodes schema 7 emitted by ``src/telemetry.c``.  It uses a
background thread for the serial port and keeps every Tk operation in the main
thread.  The plots are drawn with Tk Canvas so Spyder only needs ``pyserial``;
NumPy, Matplotlib and Qt are intentionally not required.

Run this file directly from Spyder.  Use Demo mode to verify the dashboard
without hardware, or select the LaunchPad virtual COM port and connect at
460800 baud.
"""

from __future__ import annotations

import csv
import math
import queue
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Deque, Dict, Iterable, List, Optional, Sequence, Tuple

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:  # The demo and decoder remain usable without it.
    serial = None
    list_ports = None


# User-visible title for the final telemetry application window.
APP_TITLE = "MCU Cuadcopter · Telemetría de vuelo"
# UART0 rate configured by Telemetry_Init(), in bits per second.
DEFAULT_BAUD_RATE = 460_800
# Nominal frame generation rate used for link-quality indication, in hertz.
EXPECTED_OUTPUT_RATE_HZ = 100.0
# Little-endian synchronization marker at byte offset zero of every frame.
SYNC_WORD = 0xA55A3CC3
SYNC_BYTES = struct.pack("<I", SYNC_WORD)
# Wire-layout revision; must match TELEMETRY_SCHEMA_VERSION in telemetry.h.
SCHEMA_VERSION = 7
# Header: sync/version/sequence (uint32), time (uint64), then 53 float32 values.
FRAME_STRUCT = struct.Struct("<IIIQ53f")
FRAME_SIZE = FRAME_STRUCT.size  # Complete wire-frame length, in bytes.
# Maximum in-memory history: 120 seconds at the nominal 100 Hz frame rate.
MAX_HISTORY_SAMPLES = 12_000
# pyserial blocking-read bound, in seconds, so its thread can stop promptly.
SERIAL_READ_TIMEOUT_S = 0.050
# Maximum age of the latest valid frame before the GUI reports link loss.
LINK_TIMEOUT_S = 0.500

if FRAME_SIZE != 232:
    raise RuntimeError(f"Unexpected telemetry frame size: {FRAME_SIZE}")


COLORS = {
    "background": "#08111f",
    "surface": "#0f1b2d",
    "surface_alt": "#13233a",
    "border": "#243753",
    "text": "#e8f0fb",
    "muted": "#8ea2bd",
    "cyan": "#4dd9ff",
    "blue": "#668cff",
    "green": "#4ade80",
    "amber": "#fbbf24",
    "orange": "#fb923c",
    "red": "#fb7185",
    "purple": "#c084fc",
    "grid": "#203149",
}

SERIES_COLORS = (
    COLORS["cyan"],
    COLORS["orange"],
    COLORS["green"],
    COLORS["purple"],
    COLORS["red"],
    COLORS["blue"],
)


@dataclass(frozen=True)
class SignalDefinition:
    """Describes one float32 field in the fixed telemetry frame."""

    key: str       # Stable protocol/CSV column name.
    label: str     # Spanish user-visible description.
    unit: str      # Engineering unit displayed beside the value.
    group: str     # Table section containing this signal.
    decimals: int = 3  # Decimal places used in the live value table.


SIGNALS: Tuple[SignalDefinition, ...] = (
    SignalDefinition("imu_dt_s", "Periodo IMU", "s", "IMU", 6),
    SignalDefinition("accel_x", "Aceleración X", "m/s²", "IMU"),
    SignalDefinition("accel_y", "Aceleración Y", "m/s²", "IMU"),
    SignalDefinition("accel_z", "Aceleración Z", "m/s²", "IMU"),
    SignalDefinition("imu_gyro_x", "Giroscopio IMU X", "rad/s", "IMU", 5),
    SignalDefinition("imu_gyro_y", "Giroscopio IMU Y", "rad/s", "IMU", 5),
    SignalDefinition("imu_gyro_z", "Giroscopio IMU Z", "rad/s", "IMU", 5),
    SignalDefinition("mag_x", "Campo magnético X (nariz)", "µT", "Magnetómetro"),
    SignalDefinition("mag_y", "Campo magnético Y (derecha)", "µT", "Magnetómetro"),
    SignalDefinition("mag_z", "Campo magnético Z (abajo)", "µT", "Magnetómetro"),
    SignalDefinition("heading", "Heading magnético", "°", "Heading"),
    SignalDefinition("heading_setpoint", "Setpoint heading", "°", "Heading"),
    SignalDefinition("heading_error", "Error heading", "°", "Heading"),
    SignalDefinition("horizontal_accel_x", "Aceleración horizontal X", "m/s²", "Freno inercial"),
    SignalDefinition("horizontal_accel_y", "Aceleración horizontal Y", "m/s²", "Freno inercial"),
    SignalDefinition("horizontal_velocity_x", "Velocidad horizontal X", "m/s", "Freno inercial"),
    SignalDefinition("horizontal_velocity_y", "Velocidad horizontal Y", "m/s", "Freno inercial"),
    SignalDefinition("horizontal_displacement_x", "Desplazamiento local X", "m", "Freno inercial"),
    SignalDefinition("horizontal_displacement_y", "Desplazamiento local Y", "m", "Freno inercial"),
    SignalDefinition("drift_roll_correction", "Corrección roll", "°", "Freno inercial"),
    SignalDefinition("drift_pitch_correction", "Corrección pitch", "°", "Freno inercial"),
    SignalDefinition("roll_rate_measured", "Rate roll medido", "°/s", "Tasas"),
    SignalDefinition("pitch_rate_measured", "Rate pitch medido", "°/s", "Tasas"),
    SignalDefinition("yaw_rate_measured", "Rate yaw medido", "°/s", "Tasas"),
    SignalDefinition("roll_angle", "Ángulo roll", "°", "Actitud"),
    SignalDefinition("pitch_angle", "Ángulo pitch", "°", "Actitud"),
    SignalDefinition("throttle_setpoint", "Throttle", "0…1", "Setpoints"),
    SignalDefinition("flight_roll_trim_deg", "Flight trim roll", "°", "Setpoints"),
    SignalDefinition("flight_pitch_trim_deg", "Flight trim pitch", "°", "Setpoints"),
    SignalDefinition("roll_angle_setpoint", "Setpoint ángulo roll", "°", "Setpoints"),
    SignalDefinition("pitch_angle_setpoint", "Setpoint ángulo pitch", "°", "Setpoints"),
    SignalDefinition("roll_rate_setpoint", "Setpoint rate roll", "°/s", "Setpoints"),
    SignalDefinition("pitch_rate_setpoint", "Setpoint rate pitch", "°/s", "Setpoints"),
    SignalDefinition("yaw_rate_setpoint", "Setpoint rate yaw", "°/s", "Setpoints"),
    SignalDefinition("roll_error", "Error roll", "°/s", "PID roll"),
    SignalDefinition("roll_proportional", "P roll", "norm", "PID roll", 5),
    SignalDefinition("roll_integral", "I roll", "norm", "PID roll", 5),
    SignalDefinition("roll_derivative", "D roll", "norm", "PID roll", 5),
    SignalDefinition("roll_output", "Salida roll", "norm", "PID roll", 5),
    SignalDefinition("pitch_error", "Error pitch", "°/s", "PID pitch"),
    SignalDefinition("pitch_proportional", "P pitch", "norm", "PID pitch", 5),
    SignalDefinition("pitch_integral", "I pitch", "norm", "PID pitch", 5),
    SignalDefinition("pitch_derivative", "D pitch", "norm", "PID pitch", 5),
    SignalDefinition("pitch_output", "Salida pitch", "norm", "PID pitch", 5),
    SignalDefinition("yaw_error", "Error yaw", "°/s", "PID yaw"),
    SignalDefinition("yaw_proportional", "P yaw", "norm", "PID yaw", 5),
    SignalDefinition("yaw_integral", "I yaw", "norm", "PID yaw", 5),
    SignalDefinition("yaw_derivative", "D yaw", "norm", "PID yaw", 5),
    SignalDefinition("yaw_output", "Salida yaw", "norm", "PID yaw", 5),
    SignalDefinition("motor_front_left", "Motor delantero izquierdo", "µs", "Motores", 1),
    SignalDefinition("motor_front_right", "Motor delantero derecho", "µs", "Motores", 1),
    SignalDefinition("motor_rear_right", "Motor trasero derecho", "µs", "Motores", 1),
    SignalDefinition("motor_rear_left", "Motor trasero izquierdo", "µs", "Motores", 1),
)

SIGNAL_KEYS = tuple(signal.key for signal in SIGNALS)
SIGNAL_BY_KEY = {signal.key: signal for signal in SIGNALS}


@dataclass(frozen=True)
class TelemetryFrame:
    """One completely decoded frame."""

    sync: int                  # Constant synchronization word.
    schema_version: int        # Firmware wire-layout revision.
    sequence: int              # Wrapping uint32 frame counter.
    timestamp_us: int          # MCU monotonic generation time, microseconds.
    values: Tuple[float, ...]  # 53 engineering values in SIGNALS order.

    def value(self, key: str) -> float:
        """Return one engineering value selected by its stable signal key."""
        return self.values[SIGNAL_KEYS.index(key)]

    def as_dict(self) -> Dict[str, float]:
        """Map every stable signal key to the corresponding frame value."""
        return dict(zip(SIGNAL_KEYS, self.values))


class TelemetryDecoder:
    """Incrementally finds and decodes frames from an arbitrary byte stream."""

    def __init__(self) -> None:
        # buffer retains incomplete bytes between arbitrary serial reads.
        """Initialize an empty stream buffer and zero all lifetime decoder counters."""
        self.buffer = bytearray()
        # Lifetime diagnostics displayed in the protocol/diagnostics tab.
        self.frames_decoded = 0
        self.bytes_discarded = 0
        self.resync_count = 0
        self.invalid_frame_count = 0

    def reset(self) -> None:
        """Discard buffered bytes and clear all decoder diagnostics."""
        self.buffer.clear()
        self.frames_decoded = 0
        self.bytes_discarded = 0
        self.resync_count = 0
        self.invalid_frame_count = 0

    def feed(self, data: bytes) -> List[TelemetryFrame]:
        """Append bytes and return every complete valid frame now available."""

        if data:
            self.buffer.extend(data)

        decoded: List[TelemetryFrame] = []
        while True:
            sync_index = self.buffer.find(SYNC_BYTES)
            if sync_index < 0:
                # Keep a possible partial sync prefix for the next read.
                keep = min(len(self.buffer), len(SYNC_BYTES) - 1)
                discarded = len(self.buffer) - keep
                if discarded > 0:
                    del self.buffer[:discarded]
                    self.bytes_discarded += discarded
                break

            if sync_index > 0:
                del self.buffer[:sync_index]
                self.bytes_discarded += sync_index
                self.resync_count += 1

            if len(self.buffer) < FRAME_SIZE:
                break

            candidate = bytes(self.buffer[:FRAME_SIZE])
            unpacked = FRAME_STRUCT.unpack(candidate)
            sync, schema_version, sequence, timestamp_us = unpacked[:4]
            values = tuple(float(value) for value in unpacked[4:])

            if (
                sync != SYNC_WORD
                or schema_version != SCHEMA_VERSION
                or not all(math.isfinite(value) for value in values)
            ):
                # Advance one byte so another sync inside the candidate is found.
                del self.buffer[0]
                self.invalid_frame_count += 1
                self.bytes_discarded += 1
                self.resync_count += 1
                continue

            del self.buffer[:FRAME_SIZE]
            decoded.append(
                TelemetryFrame(
                    sync=sync,
                    schema_version=schema_version,
                    sequence=sequence,
                    timestamp_us=timestamp_us,
                    values=values,
                )
            )
            self.frames_decoded += 1

        return decoded


class CsvRecorder:
    """Writes decoded frames to a conventional UTF-8 comma-separated file."""

    HEADER = (
        "host_time_iso",
        "host_elapsed_s",
        "sync",
        "schema_version",
        "sequence",
        "timestamp_us",
        *SIGNAL_KEYS,
    )

    def __init__(self, path: Path) -> None:
        # newline="" delegates CSV newline handling to Python on Windows.
        """Open path for UTF-8 CSV output, write the 59-column header, and start the row count."""
        self.path = path
        self._file = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self.rows_written = 0

    def write(self, frame: TelemetryFrame, host_elapsed_s: float) -> None:
        """Append one decoded frame and periodically flush it to disk."""
        self._writer.writerow(
            (
                datetime.now().astimezone().isoformat(timespec="milliseconds"),
                f"{host_elapsed_s:.6f}",
                f"0x{frame.sync:08X}",
                frame.schema_version,
                frame.sequence,
                frame.timestamp_us,
                *[f"{value:.9g}" for value in frame.values],
            )
        )
        self.rows_written += 1
        if self.rows_written % 50 == 0:
            self._file.flush()

    def close(self) -> None:
        """Flush and close the output file; repeated calls are harmless."""
        if not self._file.closed:
            self._file.flush()
            self._file.close()


class TelemetryHistory:
    """Keeps a bounded, timestamp-aligned history for plots and export."""

    def __init__(self, maximum_samples: int = MAX_HISTORY_SAMPLES) -> None:
        # All deques use the same bound so time and signals remain aligned.
        """Allocate equally bounded time, frame, and signal deques; maximum_samples is a sample count."""
        self.times: Deque[float] = deque(maxlen=maximum_samples)
        self.series: Dict[str, Deque[float]] = {
            key: deque(maxlen=maximum_samples) for key in SIGNAL_KEYS
        }
        self.frames: Deque[Tuple[float, TelemetryFrame]] = deque(
            maxlen=maximum_samples
        )
        self.timestamp_origin_us: Optional[int] = None
        self.last_timestamp_us: Optional[int] = None

    def clear(self) -> None:
        """Remove history and timestamp origins after reset or user request."""
        self.times.clear()
        for values in self.series.values():
            values.clear()
        self.frames.clear()
        self.timestamp_origin_us = None
        self.last_timestamp_us = None

    def append(self, frame: TelemetryFrame, host_elapsed_s: float) -> bool:
        """Append a frame; return True when a timestamp reset was detected."""

        reset_detected = (
            self.last_timestamp_us is not None
            and frame.timestamp_us < self.last_timestamp_us
        )
        if reset_detected:
            self.clear()

        if self.timestamp_origin_us is None:
            self.timestamp_origin_us = frame.timestamp_us

        relative_time = (
            frame.timestamp_us - self.timestamp_origin_us
        ) / 1_000_000.0
        self.times.append(relative_time)
        for key, value in zip(SIGNAL_KEYS, frame.values):
            self.series[key].append(value)
        self.frames.append((host_elapsed_s, frame))
        self.last_timestamp_us = frame.timestamp_us
        return reset_detected

    def plot_data(
        self, keys: Sequence[str], window_s: float
    ) -> Tuple[List[float], Dict[str, List[float]]]:
        """Return aligned samples inside the requested trailing time window."""
        if not self.times:
            return [], {key: [] for key in keys}

        all_times = list(self.times)
        minimum_time = all_times[-1] - window_s
        start = 0
        for index, sample_time in enumerate(all_times):
            if sample_time >= minimum_time:
                start = index
                break

        return (
            all_times[start:],
            {key: list(self.series[key])[start:] for key in keys},
        )


def _queue_event(event_queue: queue.Queue, event: Tuple[str, object]) -> None:
    """Deliver an event without allowing a slow GUI to block acquisition."""

    try:
        event_queue.put_nowait(event)
    except queue.Full:
        try:
            event_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            event_queue.put_nowait(("queue_overflow", None))
        except queue.Full:
            pass


class SerialReader(threading.Thread):
    """Owns the serial port and publishes decoded frames to the GUI queue."""

    def __init__(
        self,
        port: str,
        baud_rate: int,
        event_queue: queue.Queue,
    ) -> None:
        """Store the COM name, baud rate in bit/s, GUI event queue, decoder, and stop event."""
        super().__init__(daemon=True, name="TelemetrySerialReader")
        self.port = port
        self.baud_rate = baud_rate
        self.event_queue = event_queue
        self.stop_event = threading.Event()
        self.decoder = TelemetryDecoder()
        self.bytes_received = 0

    def stop(self) -> None:
        """Request that the serial background thread finish after its current bounded read."""
        self.stop_event.set()

    def run(self) -> None:
        """Open 8N1 serial input, decode incoming bytes, and enqueue frames until stop is requested."""
        if serial is None:
            _queue_event(
                self.event_queue,
                ("error", "pyserial no está instalado en el entorno de Spyder."),
            )
            return

        port_handle = None
        try:
            port_handle = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SERIAL_READ_TIMEOUT_S,
                write_timeout=SERIAL_READ_TIMEOUT_S,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            port_handle.reset_input_buffer()
            _queue_event(self.event_queue, ("opened", self.port))

            last_statistics = time.monotonic()
            while not self.stop_event.is_set():
                waiting = port_handle.in_waiting
                chunk = port_handle.read(waiting if waiting > 0 else 1)
                if chunk:
                    self.bytes_received += len(chunk)
                    for frame in self.decoder.feed(chunk):
                        _queue_event(self.event_queue, ("frame", frame))

                now = time.monotonic()
                if now - last_statistics >= 0.5:
                    _queue_event(
                        self.event_queue,
                        (
                            "decoder_stats",
                            {
                                "bytes_received": self.bytes_received,
                                "frames_decoded": self.decoder.frames_decoded,
                                "bytes_discarded": self.decoder.bytes_discarded,
                                "resync_count": self.decoder.resync_count,
                                "invalid_frame_count": self.decoder.invalid_frame_count,
                            },
                        ),
                    )
                    last_statistics = now
        except Exception as exc:  # SerialException varies by pyserial backend.
            _queue_event(self.event_queue, ("error", str(exc)))
        finally:
            if port_handle is not None and port_handle.is_open:
                port_handle.close()
            _queue_event(self.event_queue, ("stopped", None))


class DemoReader(threading.Thread):
    """Produces deterministic flight-like data for UI and CSV validation."""

    def __init__(self, event_queue: queue.Queue) -> None:
        """Create a synthetic 100 Hz telemetry producer connected to the GUI event queue."""
        super().__init__(daemon=True, name="TelemetryDemoReader")
        self.event_queue = event_queue
        self.stop_event = threading.Event()

    def stop(self) -> None:
        """Request that the demonstration thread stop producing synthetic frames."""
        self.stop_event.set()

    def run(self) -> None:
        """Generate and enqueue realistic demonstration frames at the nominal telemetry rate."""
        _queue_event(self.event_queue, ("opened", "DEMO"))
        start = time.monotonic()
        next_sample = start
        sequence = 0

        while not self.stop_event.is_set():
            now = time.monotonic()
            if now < next_sample:
                time.sleep(min(next_sample - now, 0.005))
                continue

            elapsed = now - start
            frame = self._create_frame(sequence, elapsed)
            _queue_event(self.event_queue, ("frame", frame))
            sequence = (sequence + 1) & 0xFFFFFFFF
            next_sample += 1.0 / EXPECTED_OUTPUT_RATE_HZ
            if next_sample < now - 0.1:
                next_sample = now

        _queue_event(self.event_queue, ("stopped", None))

    @staticmethod
    def _create_frame(sequence: int, elapsed: float) -> TelemetryFrame:
        """Create one deterministic synthetic TelemetryFrame for sequence and elapsed seconds."""
        roll_sp = 10.0 * math.sin(elapsed * 0.55)
        pitch_sp = 8.0 * math.sin(elapsed * 0.39 + 0.8)
        roll_angle = 8.8 * math.sin(elapsed * 0.55 - 0.10)
        pitch_angle = 6.9 * math.sin(elapsed * 0.39 + 0.65)
        roll_rate_sp = 5.0 * (roll_sp - roll_angle)
        pitch_rate_sp = 5.0 * (pitch_sp - pitch_angle)
        yaw_rate_sp = 35.0 * math.sin(elapsed * 0.21)
        roll_rate = 4.84 * math.cos(elapsed * 0.55 - 0.10)
        pitch_rate = 2.69 * math.cos(elapsed * 0.39 + 0.65)
        yaw_rate = yaw_rate_sp * 0.82
        roll_error = roll_rate_sp - roll_rate
        pitch_error = pitch_rate_sp - pitch_rate
        yaw_error = yaw_rate_sp - yaw_rate

        roll_p = 0.0035 * roll_error
        roll_i = 0.018 * math.sin(elapsed * 0.08)
        roll_d = -0.004 * math.sin(elapsed * 1.8)
        roll_output = max(-0.35, min(0.35, roll_p + roll_i + roll_d))
        pitch_p = 0.0050 * pitch_error
        pitch_i = 0.028 * math.sin(elapsed * 0.07 + 0.5)
        pitch_d = -0.006 * math.sin(elapsed * 1.4)
        pitch_output = max(-0.45, min(0.45, pitch_p + pitch_i + pitch_d))
        yaw_p = 0.0030 * yaw_error
        yaw_i = 0.006 * math.sin(elapsed * 0.05)
        yaw_d = 0.0
        yaw_output = max(-0.20, min(0.20, yaw_p + yaw_i))
        throttle = 0.42 + 0.04 * math.sin(elapsed * 0.12)

        corrections = (
            roll_output + pitch_output + yaw_output,
            -roll_output + pitch_output - yaw_output,
            -roll_output - pitch_output + yaw_output,
            roll_output - pitch_output - yaw_output,
        )
        headroom = min(throttle, 1.0 - throttle)
        maximum = max(abs(value) for value in corrections)
        scale = headroom / maximum if maximum > headroom and maximum > 0 else 1.0
        normalized = tuple(
            max(0.0, min(1.0, throttle + correction * scale))
            for correction in corrections
        )
        motors = tuple(max(1180.0, 1000.0 + value * 1000.0) for value in normalized)

        imu_gyro_x = math.radians(roll_rate) + 0.01 * math.sin(elapsed * 3.1)
        imu_gyro_y = math.radians(pitch_rate) + 0.01 * math.cos(elapsed * 2.7)
        imu_gyro_z = math.radians(yaw_rate) + 0.008 * math.sin(elapsed * 2.3)
        magnetic_heading = elapsed * 0.18
        mag_x = 30.0 * math.cos(magnetic_heading)
        mag_y = -30.0 * math.sin(magnetic_heading)
        mag_z = 20.0 + 0.5 * math.sin(elapsed * 0.11)
        heading = ((math.degrees(magnetic_heading) + 180.0) % 360.0) - 180.0
        heading_setpoint = ((heading + 8.0 * math.sin(elapsed * 0.10) + 180.0) % 360.0) - 180.0
        heading_error = ((heading_setpoint - heading + 180.0) % 360.0) - 180.0
        horizontal_accel_x = 0.18 * math.sin(elapsed * 0.75)
        horizontal_accel_y = 0.14 * math.cos(elapsed * 0.63)
        horizontal_velocity_x = 0.12 * math.sin(elapsed * 0.35)
        horizontal_velocity_y = 0.10 * math.cos(elapsed * 0.31)
        horizontal_displacement_x = 0.20 * math.sin(elapsed * 0.16)
        horizontal_displacement_y = 0.16 * math.cos(elapsed * 0.14)
        drift_roll_correction = 0.55 * math.cos(elapsed * 0.31)
        drift_pitch_correction = 0.65 * math.sin(elapsed * 0.35)

        values = (
            1.0 / 416.0,
            0.45 * math.sin(elapsed * 0.8),
            0.35 * math.cos(elapsed * 0.7),
            9.80665 + 0.18 * math.sin(elapsed * 1.1),
            imu_gyro_x,
            imu_gyro_y,
            imu_gyro_z,
            mag_x,
            mag_y,
            mag_z,
            heading,
            heading_setpoint,
            heading_error,
            horizontal_accel_x,
            horizontal_accel_y,
            horizontal_velocity_x,
            horizontal_velocity_y,
            horizontal_displacement_x,
            horizontal_displacement_y,
            drift_roll_correction,
            drift_pitch_correction,
            roll_rate,
            pitch_rate,
            yaw_rate,
            roll_angle,
            pitch_angle,
            throttle,
            0.0,  # Flight roll trim, degrees.
            0.0,  # Flight pitch trim, degrees.
            roll_sp,
            pitch_sp,
            roll_rate_sp,
            pitch_rate_sp,
            yaw_rate_sp,
            roll_error,
            roll_p,
            roll_i,
            roll_d,
            roll_output,
            pitch_error,
            pitch_p,
            pitch_i,
            pitch_d,
            pitch_output,
            yaw_error,
            yaw_p,
            yaw_i,
            yaw_d,
            yaw_output,
            *motors,
        )
        return TelemetryFrame(
            sync=SYNC_WORD,
            schema_version=SCHEMA_VERSION,
            sequence=sequence,
            timestamp_us=int(elapsed * 1_000_000.0),
            values=tuple(values),
        )


class MetricCard(tk.Frame):
    """Compact dashboard card with a title, value and supporting caption."""

    def __init__(self, master: tk.Misc, title: str, accent: str) -> None:
        """Build one labeled dashboard metric card using the requested accent color."""
        super().__init__(
            master,
            bg=COLORS["surface"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
            padx=14,
            pady=10,
        )
        self.accent = accent
        tk.Label(
            self,
            text=title.upper(),
            bg=COLORS["surface"],
            fg=COLORS["muted"],
            font=("Segoe UI", 8, "bold"),
        ).pack(anchor="w")
        self.value_label = tk.Label(
            self,
            text="—",
            bg=COLORS["surface"],
            fg=accent,
            font=("Segoe UI Semibold", 19),
        )
        self.value_label.pack(anchor="w", pady=(2, 0))
        self.caption_label = tk.Label(
            self,
            text="Sin datos",
            bg=COLORS["surface"],
            fg=COLORS["muted"],
            font=("Segoe UI", 8),
        )
        self.caption_label.pack(anchor="w")

    def set(self, value: str, caption: str = "", color: Optional[str] = None) -> None:
        """Display a value, caption, and optional foreground color on this metric card."""
        self.value_label.configure(text=value, fg=color or self.accent)
        self.caption_label.configure(text=caption)


@dataclass(frozen=True)
class PlotSeries:
    """Visual definition for one signal drawn by LivePlot."""

    key: str    # SIGNALS key used to retrieve history values.
    label: str  # Short legend label.
    color: str  # Tk-compatible hexadecimal line color.


class LivePlot(tk.Canvas):
    """Dependency-free multi-series time plot rendered on a Tk Canvas."""

    def __init__(
        self,
        master: tk.Misc,
        title: str,
        unit: str,
        series: Sequence[PlotSeries],
        fixed_limits: Optional[Tuple[float, float]] = None,
    ) -> None:
        """Create a Canvas plot with title, units, series definitions, and optional fixed Y range."""
        super().__init__(
            master,
            bg=COLORS["surface"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
        )
        self.title = title
        self.unit = unit
        self.series_definitions = tuple(series)
        self.fixed_limits = fixed_limits
        self.times: Sequence[float] = ()
        self.values: Dict[str, Sequence[float]] = {}
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_data(
        self, times: Sequence[float], values: Dict[str, Sequence[float]]
    ) -> None:
        """Replace plotted time/value arrays and redraw the selected time window in seconds."""
        self.times = times
        self.values = values
        self.redraw()

    def redraw(self) -> None:
        """Render grid, axes, legend, and all finite samples onto the current Canvas size."""
        self.delete("all")
        width = max(self.winfo_width(), 320)
        height = max(self.winfo_height(), 220)
        left, right, top, bottom = 62, 18, 42, 38
        plot_width = max(width - left - right, 10)
        plot_height = max(height - top - bottom, 10)

        self.create_text(
            16,
            14,
            anchor="nw",
            text=self.title,
            fill=COLORS["text"],
            font=("Segoe UI Semibold", 11),
        )
        legend_x = width - right
        for definition in reversed(self.series_definitions):
            text_width = 12 + len(definition.label) * 7
            legend_x -= text_width
            self.create_line(
                legend_x,
                20,
                legend_x + 11,
                20,
                fill=definition.color,
                width=2,
            )
            self.create_text(
                legend_x + 15,
                20,
                anchor="w",
                text=definition.label,
                fill=COLORS["muted"],
                font=("Segoe UI", 8),
            )

        if len(self.times) < 2:
            self.create_text(
                width / 2,
                height / 2,
                text="Esperando telemetría…",
                fill=COLORS["muted"],
                font=("Segoe UI", 11),
            )
            return

        x_min, x_max = self.times[0], self.times[-1]
        if x_max <= x_min:
            x_max = x_min + 1.0

        visible_values = [
            value
            for definition in self.series_definitions
            for value in self.values.get(definition.key, ())
            if math.isfinite(value)
        ]
        if self.fixed_limits is not None:
            y_min, y_max = self.fixed_limits
        elif visible_values:
            y_min, y_max = min(visible_values), max(visible_values)
            if y_min == y_max:
                padding = max(abs(y_min) * 0.1, 1.0)
            else:
                padding = (y_max - y_min) * 0.12
            y_min -= padding
            y_max += padding
        else:
            y_min, y_max = -1.0, 1.0

        if y_max <= y_min:
            y_max = y_min + 1.0

        for division in range(6):
            fraction = division / 5.0
            y = top + fraction * plot_height
            value = y_max - fraction * (y_max - y_min)
            self.create_line(
                left,
                y,
                left + plot_width,
                y,
                fill=COLORS["grid"],
            )
            self.create_text(
                left - 8,
                y,
                anchor="e",
                text=f"{value:.2f}",
                fill=COLORS["muted"],
                font=("Consolas", 8),
            )

        for division in range(6):
            fraction = division / 5.0
            x = left + fraction * plot_width
            value = x_min + fraction * (x_max - x_min)
            self.create_line(
                x,
                top,
                x,
                top + plot_height,
                fill=COLORS["grid"],
            )
            self.create_text(
                x,
                top + plot_height + 12,
                text=f"{value:.1f}",
                fill=COLORS["muted"],
                font=("Consolas", 8),
            )

        self.create_text(
            13,
            top + plot_height / 2,
            text=self.unit,
            angle=90,
            fill=COLORS["muted"],
            font=("Segoe UI", 8),
        )
        self.create_text(
            left + plot_width / 2,
            height - 8,
            text="Tiempo MCU [s]",
            fill=COLORS["muted"],
            font=("Segoe UI", 8),
        )

        maximum_points = max(int(plot_width), 100)
        step = max(1, len(self.times) // maximum_points)
        for definition in self.series_definitions:
            samples = self.values.get(definition.key, ())
            point_count = min(len(self.times), len(samples))
            points: List[float] = []
            for index in range(0, point_count, step):
                value = samples[index]
                if not math.isfinite(value):
                    continue
                x = left + (
                    (self.times[index] - x_min) / (x_max - x_min)
                ) * plot_width
                y = top + (1.0 - (value - y_min) / (y_max - y_min)) * plot_height
                points.extend((x, y))
            if len(points) >= 4:
                self.create_line(
                    *points,
                    fill=definition.color,
                    width=2,
                    smooth=False,
                )


class AttitudeIndicator(tk.Canvas):
    """Simple artificial horizon for roll and pitch feedback."""

    def __init__(self, master: tk.Misc) -> None:
        """Create the roll/pitch artificial-horizon canvas with zero-degree initial attitude."""
        super().__init__(
            master,
            width=300,
            height=260,
            bg=COLORS["surface"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
        )
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_attitude(self, roll_deg: float, pitch_deg: float) -> None:
        """Store roll and pitch in degrees and redraw the artificial horizon."""
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.redraw()

    def redraw(self) -> None:
        """Render horizon rotation, pitch displacement, aircraft mark, and numeric angles."""
        self.delete("all")
        width = max(self.winfo_width(), 260)
        height = max(self.winfo_height(), 220)
        center_x, center_y = width / 2, height / 2 + 8
        radius = min(width, height) * 0.36
        self.create_text(
            14,
            12,
            anchor="nw",
            text="ACTITUD",
            fill=COLORS["muted"],
            font=("Segoe UI", 8, "bold"),
        )

        self.create_oval(
            center_x - radius,
            center_y - radius,
            center_x + radius,
            center_y + radius,
            fill="#102a46",
            outline=COLORS["border"],
            width=2,
        )
        angle = math.radians(self.roll_deg)
        pitch_offset = max(-radius * 0.65, min(radius * 0.65, self.pitch_deg * 2.0))
        dx = math.cos(angle) * radius
        dy = math.sin(angle) * radius
        normal_x = -math.sin(angle) * pitch_offset
        normal_y = math.cos(angle) * pitch_offset
        x1 = center_x - dx + normal_x
        y1 = center_y + dy + normal_y
        x2 = center_x + dx + normal_x
        y2 = center_y - dy + normal_y
        self.create_line(x1, y1, x2, y2, fill=COLORS["amber"], width=4)

        # Fixed aircraft reference.
        self.create_line(
            center_x - 45,
            center_y,
            center_x - 12,
            center_y,
            fill=COLORS["cyan"],
            width=3,
        )
        self.create_line(
            center_x + 12,
            center_y,
            center_x + 45,
            center_y,
            fill=COLORS["cyan"],
            width=3,
        )
        self.create_line(
            center_x,
            center_y,
            center_x,
            center_y + 12,
            fill=COLORS["cyan"],
            width=3,
        )
        self.create_text(
            center_x,
            height - 28,
            text=f"ROLL {self.roll_deg:+6.2f}°    PITCH {self.pitch_deg:+6.2f}°",
            fill=COLORS["text"],
            font=("Consolas", 10, "bold"),
        )


class MotorIndicator(tk.Canvas):
    """Top-view motor layout with pulse-width bars."""

    MOTOR_LAYOUT = (
        ("motor_front_left", "FL · PF0", 0.27, 0.31),
        ("motor_front_right", "FR · PF2", 0.73, 0.31),
        ("motor_rear_left", "RL · PK4", 0.27, 0.72),
        ("motor_rear_right", "RR · PG0", 0.73, 0.72),
    )

    def __init__(self, master: tk.Misc) -> None:
        """Create the four-motor top-view canvas with pulses initialized to 1000 microseconds."""
        super().__init__(
            master,
            width=320,
            height=260,
            bg=COLORS["surface"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
        )
        self.motor_values = {key: 1000.0 for key, *_ in self.MOTOR_LAYOUT}
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_motors(self, values: Dict[str, float]) -> None:
        """Store four ESC high times in microseconds and redraw their indicators."""
        for key in self.motor_values:
            if key in values:
                self.motor_values[key] = values[key]
        self.redraw()

    def redraw(self) -> None:
        """Render motor positions, pin labels, pulse values, and normalized pulse arcs."""
        self.delete("all")
        width = max(self.winfo_width(), 280)
        height = max(self.winfo_height(), 220)
        self.create_text(
            14,
            12,
            anchor="nw",
            text="SALIDA A MOTORES",
            fill=COLORS["muted"],
            font=("Segoe UI", 8, "bold"),
        )
        self.create_text(
            width / 2,
            38,
            text="NARIZ",
            fill=COLORS["cyan"],
            font=("Segoe UI", 8, "bold"),
        )
        self.create_polygon(
            width / 2,
            46,
            width / 2 - 7,
            58,
            width / 2 + 7,
            58,
            fill=COLORS["cyan"],
            outline="",
        )
        self.create_line(
            width * 0.27,
            height * 0.31,
            width * 0.73,
            height * 0.72,
            fill=COLORS["border"],
            width=5,
        )
        self.create_line(
            width * 0.73,
            height * 0.31,
            width * 0.27,
            height * 0.72,
            fill=COLORS["border"],
            width=5,
        )

        for key, label, x_fraction, y_fraction in self.MOTOR_LAYOUT:
            pulse = self.motor_values[key]
            fraction = max(0.0, min(1.0, (pulse - 1000.0) / 1000.0))
            x, y = width * x_fraction, height * y_fraction
            color = (
                COLORS["muted"]
                if pulse <= 1005.0
                else COLORS["green"]
                if pulse < 1800.0
                else COLORS["amber"]
            )
            self.create_oval(
                x - 29,
                y - 29,
                x + 29,
                y + 29,
                fill=COLORS["surface_alt"],
                outline=color,
                width=3,
            )
            self.create_arc(
                x - 22,
                y - 22,
                x + 22,
                y + 22,
                start=90,
                extent=-359.0 * fraction,
                style=tk.ARC,
                outline=color,
                width=5,
            )
            self.create_text(
                x,
                y - 2,
                text=f"{pulse:.0f}",
                fill=COLORS["text"],
                font=("Consolas", 10, "bold"),
            )
            self.create_text(
                x,
                y + 40,
                text=label,
                fill=COLORS["muted"],
                font=("Segoe UI", 8),
            )


class TelemetryDashboard(tk.Tk):
    """Main application window."""

    PLOT_CONFIGURATIONS = (
        (
            "Actitud",
            "Ángulos medidos y consignas",
            "grados",
            (
                PlotSeries("roll_angle", "Roll", SERIES_COLORS[0]),
                PlotSeries("roll_angle_setpoint", "Roll SP", SERIES_COLORS[1]),
                PlotSeries("pitch_angle", "Pitch", SERIES_COLORS[2]),
                PlotSeries("pitch_angle_setpoint", "Pitch SP", SERIES_COLORS[3]),
            ),
            (-30.0, 30.0),
        ),
        (
            "Heading",
            "Heading magnético y referencia de yaw",
            "grados",
            (
                PlotSeries("heading", "Heading", SERIES_COLORS[0]),
                PlotSeries("heading_setpoint", "Referencia", SERIES_COLORS[1]),
                PlotSeries("heading_error", "Error", SERIES_COLORS[4]),
            ),
            (-180.0, 180.0),
        ),
        (
            "Acel. XY",
            "Aceleración horizontal compensada",
            "m/s²",
            (
                PlotSeries("horizontal_accel_x", "X", SERIES_COLORS[0]),
                PlotSeries("horizontal_accel_y", "Y", SERIES_COLORS[1]),
            ),
            (-1.0, 1.0),
        ),
        (
            "Velocidad XY",
            "Velocidad horizontal estimada",
            "m/s",
            (
                PlotSeries("horizontal_velocity_x", "X", SERIES_COLORS[0]),
                PlotSeries("horizontal_velocity_y", "Y", SERIES_COLORS[1]),
            ),
            (-0.75, 0.75),
        ),
        (
            "Desplazamiento XY",
            "Desplazamiento local desde el último reinicio",
            "m",
            (
                PlotSeries("horizontal_displacement_x", "X", SERIES_COLORS[0]),
                PlotSeries("horizontal_displacement_y", "Y", SERIES_COLORS[1]),
            ),
            (-0.75, 0.75),
        ),
        (
            "Corrección XY",
            "Trim angular del freno inercial",
            "grados",
            (
                PlotSeries("drift_roll_correction", "Roll", SERIES_COLORS[0]),
                PlotSeries("drift_pitch_correction", "Pitch", SERIES_COLORS[1]),
            ),
            (-3.5, 3.5),
        ),
        (
            "Tasas",
            "Velocidad angular medida y consignas",
            "°/s",
            (
                PlotSeries("roll_rate_measured", "Roll", SERIES_COLORS[0]),
                PlotSeries("roll_rate_setpoint", "Roll SP", SERIES_COLORS[1]),
                PlotSeries("pitch_rate_measured", "Pitch", SERIES_COLORS[2]),
                PlotSeries("pitch_rate_setpoint", "Pitch SP", SERIES_COLORS[3]),
                PlotSeries("yaw_rate_measured", "Yaw", SERIES_COLORS[4]),
                PlotSeries("yaw_rate_setpoint", "Yaw SP", SERIES_COLORS[5]),
            ),
            None,
        ),
        (
            "Errores PID",
            "Errores de velocidad angular",
            "°/s",
            (
                PlotSeries("roll_error", "Roll", SERIES_COLORS[0]),
                PlotSeries("pitch_error", "Pitch", SERIES_COLORS[2]),
                PlotSeries("yaw_error", "Yaw", SERIES_COLORS[4]),
            ),
            None,
        ),
        (
            "Salida PID",
            "Comandos normalizados por eje",
            "normalizado",
            (
                PlotSeries("roll_output", "Roll", SERIES_COLORS[0]),
                PlotSeries("pitch_output", "Pitch", SERIES_COLORS[2]),
                PlotSeries("yaw_output", "Yaw", SERIES_COLORS[4]),
            ),
            (-0.5, 0.5),
        ),
        (
            "Motores",
            "Pulsos enviados a los ESC",
            "µs",
            (
                PlotSeries("motor_front_left", "FL", SERIES_COLORS[0]),
                PlotSeries("motor_front_right", "FR", SERIES_COLORS[1]),
                PlotSeries("motor_rear_right", "RR", SERIES_COLORS[2]),
                PlotSeries("motor_rear_left", "RL", SERIES_COLORS[3]),
            ),
            (950.0, 2050.0),
        ),
        (
            "Términos roll",
            "Descomposición P/I/D de roll",
            "normalizado",
            (
                PlotSeries("roll_proportional", "P", SERIES_COLORS[0]),
                PlotSeries("roll_integral", "I", SERIES_COLORS[1]),
                PlotSeries("roll_derivative", "D", SERIES_COLORS[4]),
                PlotSeries("roll_output", "Salida", SERIES_COLORS[2]),
            ),
            (-0.4, 0.4),
        ),
        (
            "Términos pitch",
            "Descomposición P/I/D de pitch",
            "normalizado",
            (
                PlotSeries("pitch_proportional", "P", SERIES_COLORS[0]),
                PlotSeries("pitch_integral", "I", SERIES_COLORS[1]),
                PlotSeries("pitch_derivative", "D", SERIES_COLORS[4]),
                PlotSeries("pitch_output", "Salida", SERIES_COLORS[2]),
            ),
            (-0.5, 0.5),
        ),
        (
            "Términos yaw",
            "Descomposición P/I/D de yaw",
            "normalizado",
            (
                PlotSeries("yaw_proportional", "P", SERIES_COLORS[0]),
                PlotSeries("yaw_integral", "I", SERIES_COLORS[1]),
                PlotSeries("yaw_derivative", "D", SERIES_COLORS[4]),
                PlotSeries("yaw_output", "Salida", SERIES_COLORS[2]),
            ),
            (-0.25, 0.25),
        ),
    )

    def __init__(self) -> None:
        """Construct the application state, widgets, update timer, and orderly close handler."""
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1480x900")
        self.minsize(1120, 720)
        self.configure(bg=COLORS["background"])
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.event_queue: queue.Queue = queue.Queue(maxsize=4_000)
        self.reader: Optional[threading.Thread] = None
        self.history = TelemetryHistory()
        self.latest_frame: Optional[TelemetryFrame] = None
        self.latest_values: Dict[str, float] = {}
        self.recorder: Optional[CsvRecorder] = None
        self.connection_started_monotonic = time.monotonic()
        self.last_frame_monotonic: Optional[float] = None
        self.last_sequence: Optional[int] = None
        self.received_frames = 0
        self.lost_frames = 0
        self.timestamp_gap_count = 0
        self.queue_overflow_count = 0
        self.rate_times: Deque[float] = deque(maxlen=300)
        self.decoder_statistics: Dict[str, int] = {
            "bytes_received": 0,
            "frames_decoded": 0,
            "bytes_discarded": 0,
            "resync_count": 0,
            "invalid_frame_count": 0,
        }
        self.port_is_open = False
        self.current_source = ""

        self.port_variable = tk.StringVar()
        self.baud_variable = tk.StringVar(value=str(DEFAULT_BAUD_RATE))
        self.window_variable = tk.StringVar(value="10")
        self.status_variable = tk.StringVar(value="Desconectado")
        self.file_variable = tk.StringVar(value="CSV inactivo")

        self._configure_styles()
        self._build_interface()
        self._refresh_ports()
        self.after(20, self._process_events)
        self.after(100, self._refresh_dashboard)

    def _configure_styles(self) -> None:
        """Define the ttk colors, fonts, spacing, and states used by the final interface."""
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background=COLORS["background"])
        style.configure("Surface.TFrame", background=COLORS["surface"])
        style.configure(
            "TLabel",
            background=COLORS["background"],
            foreground=COLORS["text"],
            font=("Segoe UI", 9),
        )
        style.configure(
            "Surface.TLabel",
            background=COLORS["surface"],
            foreground=COLORS["text"],
        )
        style.configure(
            "Muted.TLabel",
            background=COLORS["background"],
            foreground=COLORS["muted"],
        )
        style.configure(
            "TButton",
            background=COLORS["surface_alt"],
            foreground=COLORS["text"],
            bordercolor=COLORS["border"],
            padding=(12, 7),
            font=("Segoe UI Semibold", 9),
        )
        style.map(
            "TButton",
            background=[("active", COLORS["border"])],
            foreground=[("disabled", COLORS["muted"])],
        )
        style.configure(
            "Primary.TButton",
            background="#087ea4",
            foreground="#ffffff",
            bordercolor="#1fc7ef",
        )
        style.map("Primary.TButton", background=[("active", "#0b96bf")])
        style.configure(
            "Danger.TButton",
            background="#7f1d3a",
            foreground="#ffffff",
            bordercolor=COLORS["red"],
        )
        style.configure(
            "TCombobox",
            fieldbackground=COLORS["surface_alt"],
            background=COLORS["surface_alt"],
            foreground=COLORS["text"],
            arrowcolor=COLORS["cyan"],
            bordercolor=COLORS["border"],
        )
        style.configure(
            "TNotebook",
            background=COLORS["background"],
            borderwidth=0,
        )
        style.configure(
            "TNotebook.Tab",
            background=COLORS["surface"],
            foreground=COLORS["muted"],
            padding=(14, 8),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", COLORS["surface_alt"])],
            foreground=[("selected", COLORS["cyan"])],
        )
        style.configure(
            "Treeview",
            background=COLORS["surface"],
            fieldbackground=COLORS["surface"],
            foreground=COLORS["text"],
            rowheight=27,
            bordercolor=COLORS["border"],
        )
        style.configure(
            "Treeview.Heading",
            background=COLORS["surface_alt"],
            foreground=COLORS["text"],
            font=("Segoe UI Semibold", 9),
        )
        style.map("Treeview", background=[("selected", "#174665")])

    def _build_interface(self) -> None:
        """Create the complete header, metrics, tabs, plots, tables, and recording controls."""
        header = tk.Frame(self, bg=COLORS["surface"], padx=18, pady=12)
        header.pack(fill="x")
        title_block = tk.Frame(header, bg=COLORS["surface"])
        title_block.pack(side="left")
        tk.Label(
            title_block,
            text="MCU CUADCOPTER · TM4C1294NCPDT",
            bg=COLORS["surface"],
            fg=COLORS["cyan"],
            font=("Segoe UI", 9, "bold"),
        ).pack(anchor="w")
        tk.Label(
            title_block,
            text="Telemetría de vuelo",
            bg=COLORS["surface"],
            fg=COLORS["text"],
            font=("Segoe UI Semibold", 18),
        ).pack(anchor="w")

        connection = tk.Frame(header, bg=COLORS["surface"])
        connection.pack(side="right", fill="y")
        tk.Label(
            connection,
            text="Puerto",
            bg=COLORS["surface"],
            fg=COLORS["muted"],
        ).grid(row=0, column=0, padx=(0, 5))
        self.port_combo = ttk.Combobox(
            connection,
            textvariable=self.port_variable,
            state="readonly",
            width=25,
        )
        self.port_combo.grid(row=0, column=1, padx=4)
        ttk.Button(connection, text="↻", width=3, command=self._refresh_ports).grid(
            row=0, column=2, padx=4
        )
        tk.Label(
            connection,
            text="Baud",
            bg=COLORS["surface"],
            fg=COLORS["muted"],
        ).grid(row=0, column=3, padx=(12, 5))
        self.baud_combo = ttk.Combobox(
            connection,
            textvariable=self.baud_variable,
            values=(str(DEFAULT_BAUD_RATE),),
            state="readonly",
            width=9,
        )
        self.baud_combo.grid(row=0, column=4, padx=4)
        self.connect_button = ttk.Button(
            connection,
            text="Conectar",
            style="Primary.TButton",
            command=self._toggle_connection,
        )
        self.connect_button.grid(row=0, column=5, padx=(10, 4))
        self.demo_button = ttk.Button(
            connection, text="Modo demo", command=self._start_demo
        )
        self.demo_button.grid(row=0, column=6, padx=4)

        body = ttk.Frame(self, padding=(16, 12))
        body.pack(fill="both", expand=True)

        metrics = tk.Frame(body, bg=COLORS["background"])
        metrics.pack(fill="x", pady=(0, 10))
        for column in range(6):
            metrics.columnconfigure(column, weight=1, uniform="metrics")

        self.link_card = MetricCard(metrics, "Enlace", COLORS["red"])
        self.rate_card = MetricCard(metrics, "Tramas", COLORS["cyan"])
        self.magnetic_card = MetricCard(metrics, "Campo magnético", COLORS["blue"])
        self.loss_card = MetricCard(metrics, "Pérdidas", COLORS["amber"])
        self.control_card = MetricCard(metrics, "Motores", COLORS["green"])
        self.record_card = MetricCard(metrics, "Grabación", COLORS["purple"])
        for column, card in enumerate(
            (
                self.link_card,
                self.rate_card,
                self.magnetic_card,
                self.loss_card,
                self.control_card,
                self.record_card,
            )
        ):
            card.grid(row=0, column=column, sticky="nsew", padx=4)

        toolbar = tk.Frame(body, bg=COLORS["background"])
        toolbar.pack(fill="x", pady=(0, 10))
        self.record_button = ttk.Button(
            toolbar, text="● Grabar CSV", command=self._toggle_recording
        )
        self.record_button.pack(side="left", padx=(0, 6))
        ttk.Button(
            toolbar, text="Exportar historial", command=self._export_history
        ).pack(side="left", padx=6)
        ttk.Button(toolbar, text="Limpiar gráficas", command=self._clear_history).pack(
            side="left", padx=6
        )
        tk.Label(
            toolbar,
            text="Ventana",
            bg=COLORS["background"],
            fg=COLORS["muted"],
        ).pack(side="left", padx=(18, 5))
        ttk.Combobox(
            toolbar,
            textvariable=self.window_variable,
            values=("5", "10", "20", "30", "60"),
            state="readonly",
            width=5,
        ).pack(side="left")
        tk.Label(
            toolbar,
            text="s",
            bg=COLORS["background"],
            fg=COLORS["muted"],
        ).pack(side="left", padx=(4, 12))
        tk.Label(
            toolbar,
            textvariable=self.file_variable,
            bg=COLORS["background"],
            fg=COLORS["muted"],
            anchor="e",
        ).pack(side="right", fill="x", expand=True)

        notebook = ttk.Notebook(body)
        notebook.pack(fill="both", expand=True)
        dashboard_tab = ttk.Frame(notebook, style="TFrame")
        sensors_tab = ttk.Frame(notebook, style="TFrame")
        values_tab = ttk.Frame(notebook, style="TFrame")
        diagnostics_tab = ttk.Frame(notebook, style="TFrame")
        notebook.add(dashboard_tab, text="Vista general")
        notebook.add(sensors_tab, text="Sensores · 9 ejes")
        notebook.add(values_tab, text="Datos completos")
        notebook.add(diagnostics_tab, text="Diagnóstico")

        self.sensor_plots: List[Tuple[LivePlot, Tuple[str, ...]]] = []
        self.plots: List[Tuple[LivePlot, Tuple[str, ...]]] = []
        self._build_sensor_tab(sensors_tab)

        dashboard_tab.columnconfigure(0, weight=0, minsize=325)
        dashboard_tab.columnconfigure(1, weight=1)
        dashboard_tab.rowconfigure(0, weight=1)

        indicator_column = ttk.Frame(dashboard_tab, padding=(0, 8, 8, 0))
        indicator_column.grid(row=0, column=0, sticky="nsew")
        indicator_column.rowconfigure(0, weight=1)
        indicator_column.rowconfigure(1, weight=1)
        indicator_column.columnconfigure(0, weight=1)
        self.attitude_indicator = AttitudeIndicator(indicator_column)
        self.attitude_indicator.grid(row=0, column=0, sticky="nsew", pady=(0, 5))
        self.motor_indicator = MotorIndicator(indicator_column)
        self.motor_indicator.grid(row=1, column=0, sticky="nsew", pady=(5, 0))

        self.plot_notebook = ttk.Notebook(dashboard_tab)
        self.plot_notebook.grid(row=0, column=1, sticky="nsew", pady=(8, 0))
        for tab_name, title, unit, series, fixed_limits in self.PLOT_CONFIGURATIONS:
            plot = LivePlot(
                self.plot_notebook,
                title=title,
                unit=unit,
                series=series,
                fixed_limits=fixed_limits,
            )
            self.plot_notebook.add(plot, text=tab_name)
            self.plots.append((plot, tuple(item.key for item in series)))

        self._build_values_tab(values_tab)
        self._build_diagnostics_tab(diagnostics_tab)

        status_bar = tk.Frame(self, bg=COLORS["surface"], padx=14, pady=5)
        status_bar.pack(fill="x", side="bottom")
        self.status_dot = tk.Label(
            status_bar,
            text="●",
            bg=COLORS["surface"],
            fg=COLORS["red"],
            font=("Segoe UI", 10),
        )
        self.status_dot.pack(side="left")
        tk.Label(
            status_bar,
            textvariable=self.status_variable,
            bg=COLORS["surface"],
            fg=COLORS["muted"],
            font=("Segoe UI", 9),
        ).pack(side="left", padx=6)
        tk.Label(
            status_bar,
            text=f"Schema {SCHEMA_VERSION} · {FRAME_SIZE} bytes · little-endian · sin CRC",
            bg=COLORS["surface"],
            fg=COLORS["muted"],
            font=("Segoe UI", 8),
        ).pack(side="right")

    def _build_sensor_tab(self, parent: ttk.Frame) -> None:
        """Build a compact view of the nine directly measured sensor axes."""

        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(0, weight=1)
        parent.rowconfigure(1, weight=1)

        configurations = (
            (
                "Acelerómetro directo de la IMU",
                "m/s²",
                (
                    PlotSeries("accel_x", "X", SERIES_COLORS[0]),
                    PlotSeries("accel_y", "Y", SERIES_COLORS[1]),
                    PlotSeries("accel_z", "Z", SERIES_COLORS[2]),
                ),
                0,
                0,
                1,
            ),
            (
                "Giroscopio directo de la IMU (sin low-pass del controlador)",
                "rad/s",
                (
                    PlotSeries("imu_gyro_x", "X", SERIES_COLORS[0]),
                    PlotSeries("imu_gyro_y", "Y", SERIES_COLORS[1]),
                    PlotSeries("imu_gyro_z", "Z", SERIES_COLORS[2]),
                ),
                0,
                1,
                1,
            ),
            (
                "Magnetómetro LIS2MDL calibrado · cuerpo FRD",
                "µT",
                (
                    PlotSeries("mag_x", "X", SERIES_COLORS[0]),
                    PlotSeries("mag_y", "Y", SERIES_COLORS[1]),
                    PlotSeries("mag_z", "Z", SERIES_COLORS[2]),
                ),
                1,
                0,
                2,
            ),
        )

        for title, unit, series, row, column, columnspan in configurations:
            plot = LivePlot(
                parent,
                title=title,
                unit=unit,
                series=series,
                fixed_limits=None,
            )
            plot.grid(
                row=row,
                column=column,
                columnspan=columnspan,
                sticky="nsew",
                padx=6,
                pady=5,
            )
            self.sensor_plots.append((plot, tuple(item.key for item in series)))

    def _build_values_tab(self, parent: ttk.Frame) -> None:
        """Create the scrollable table containing every decoded engineering signal."""
        container = ttk.Frame(parent, padding=10)
        container.pack(fill="both", expand=True)
        columns = ("group", "field", "value", "unit")
        self.value_tree = ttk.Treeview(
            container, columns=columns, show="headings", selectmode="browse"
        )
        self.value_tree.heading("group", text="Grupo")
        self.value_tree.heading("field", text="Variable")
        self.value_tree.heading("value", text="Valor")
        self.value_tree.heading("unit", text="Unidad")
        self.value_tree.column("group", width=130, anchor="w")
        self.value_tree.column("field", width=330, anchor="w")
        self.value_tree.column("value", width=180, anchor="e")
        self.value_tree.column("unit", width=100, anchor="center")
        scrollbar = ttk.Scrollbar(
            container, orient="vertical", command=self.value_tree.yview
        )
        self.value_tree.configure(yscrollcommand=scrollbar.set)
        self.value_tree.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        headers = (
            ("header_sync", "Cabecera", "sync", "—", "hex"),
            ("header_schema", "Cabecera", "schema_version", "—", "U32"),
            ("header_sequence", "Cabecera", "sequence", "—", "U32"),
            ("header_timestamp", "Cabecera", "timestamp_us", "—", "µs"),
        )
        for item_id, group, field, value, unit in headers:
            self.value_tree.insert(
                "", "end", iid=item_id, values=(group, field, value, unit)
            )
        for signal in SIGNALS:
            self.value_tree.insert(
                "",
                "end",
                iid=f"signal_{signal.key}",
                values=(signal.group, signal.key, "—", signal.unit),
            )

    def _build_diagnostics_tab(self, parent: ttk.Frame) -> None:
        """Create protocol counters, data-age values, and decoder diagnostic controls."""
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(0, weight=1)
        left = tk.Frame(
            parent,
            bg=COLORS["surface"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
            padx=18,
            pady=16,
        )
        right = tk.Frame(
            parent,
            bg=COLORS["surface"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
            padx=18,
            pady=16,
        )
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=10)
        right.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=10)

        tk.Label(
            left,
            text="ESTADO DEL RECEPTOR",
            bg=COLORS["surface"],
            fg=COLORS["cyan"],
            font=("Segoe UI", 9, "bold"),
        ).pack(anchor="w")
        self.diagnostic_labels: Dict[str, tk.Label] = {}
        rows = (
            ("source", "Fuente"),
            ("bytes_received", "Bytes recibidos"),
            ("frames_decoded", "Tramas decodificadas"),
            ("received_frames", "Tramas procesadas por GUI"),
            ("lost_frames", "Saltos de sequence"),
            ("timestamp_gaps", "Huecos de timestamp"),
            ("bytes_discarded", "Bytes descartados"),
            ("resync_count", "Resincronizaciones"),
            ("invalid_frame_count", "Candidatos inválidos"),
            ("queue_overflow", "Overflow cola PC"),
            ("csv_rows", "Filas CSV"),
        )
        for key, label in rows:
            row = tk.Frame(left, bg=COLORS["surface"], pady=5)
            row.pack(fill="x")
            tk.Label(
                row,
                text=label,
                bg=COLORS["surface"],
                fg=COLORS["muted"],
                font=("Segoe UI", 9),
            ).pack(side="left")
            value_label = tk.Label(
                row,
                text="0",
                bg=COLORS["surface"],
                fg=COLORS["text"],
                font=("Consolas", 10, "bold"),
            )
            value_label.pack(side="right")
            self.diagnostic_labels[key] = value_label

        tk.Label(
            right,
            text="PROTOCOLO BINARIO",
            bg=COLORS["surface"],
            fg=COLORS["cyan"],
            font=("Segoe UI", 9, "bold"),
        ).pack(anchor="w")
        protocol_text = (
            "UART0 · PA1 TX · USB VCOM\n"
            "460800 baud · 8N1 · sin flow control\n\n"
            f"Frame fijo: {FRAME_SIZE} bytes\n"
            "0..3    sync = C3 3C 5A A5\n"
            f"4..7    schema_version = {SCHEMA_VERSION}\n"
            "8..11   sequence (U32)\n"
            "12..19  timestamp_us (U64)\n"
            f"20..{FRAME_SIZE - 1} {len(SIGNALS)} × float32 IEEE-754\n\n"
            "La trama no contiene CRC ni banderas de validez. El receptor "
            "busca la palabra sync y verifica la versión. Un salto de sequence "
            "indica pérdida de una trama ya creada; un salto de timestamp también "
            "puede revelar snapshots omitidos dentro del firmware.\n\n"
            "CSV: UTF-8, separador coma, punto decimal. Incluye hora del PC, "
            f"tiempo transcurrido, cabecera completa y las {len(SIGNALS)} señales."
        )
        tk.Label(
            right,
            text=protocol_text,
            justify="left",
            wraplength=530,
            bg=COLORS["surface"],
            fg=COLORS["text"],
            font=("Consolas", 9),
        ).pack(anchor="w", pady=(14, 0))

    def _refresh_ports(self) -> None:
        """Enumerate serial ports and preserve the current selection when possible."""
        if list_ports is None:
            self.port_combo["values"] = ()
            self.port_variable.set("")
            self.status_variable.set(
                "Instale pyserial para usar hardware; el modo demo ya está disponible."
            )
            return

        ports = sorted(
            list(list_ports.comports()),
            key=lambda item: item.device,
        )
        labels = [
            f"{item.device} · {item.description}" if item.description else item.device
            for item in ports
        ]
        self.port_combo["values"] = labels
        if labels:
            current_device = self.port_variable.get().split(" · ", 1)[0]
            selected = next(
                (label for label in labels if label.startswith(current_device + " ·")),
                labels[0],
            )
            self.port_variable.set(selected)
        else:
            self.port_variable.set("")
            self.status_variable.set("No se encontraron puertos seriales.")

    def _toggle_connection(self) -> None:
        """Connect to the selected COM port or disconnect the currently active reader."""
        if self.reader is not None:
            self._stop_reader("Desconectando…")
            return

        if serial is None:
            messagebox.showerror(
                "Dependencia faltante",
                "pyserial no está instalado en el entorno actual.\n\n"
                "Ejecute en la consola de Spyder:\n%pip install pyserial",
            )
            return

        port_label = self.port_variable.get().strip()
        if not port_label:
            messagebox.showwarning(
                "Puerto requerido", "Actualice la lista y seleccione el puerto COM."
            )
            return
        port = port_label.split(" · ", 1)[0]
        try:
            baud_rate = int(self.baud_variable.get())
            if baud_rate <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Baud inválido", "La velocidad debe ser un entero positivo.")
            return

        self._prepare_new_connection(port)
        self.reader = SerialReader(port, baud_rate, self.event_queue)
        self.reader.start()
        self.status_variable.set(f"Abriendo {port} a {baud_rate} baud…")
        self.connect_button.configure(text="Desconectar", style="Danger.TButton")
        self.demo_button.configure(state="disabled")

    def _start_demo(self) -> None:
        """Replace any real reader with a synthetic source for hardware-free UI testing."""
        if self.reader is not None:
            self._stop_reader("Cambiando a modo demo…")
            self.after(150, self._start_demo)
            return
        self._prepare_new_connection("DEMO")
        self.reader = DemoReader(self.event_queue)
        self.reader.start()
        self.status_variable.set("Iniciando generador de demostración…")
        self.connect_button.configure(text="Detener demo", style="Danger.TButton")
        self.demo_button.configure(state="disabled")

    def _prepare_new_connection(self, source: str) -> None:
        """Reset timestamps, counters, plots, and labels for the named data source."""
        self.history.clear()
        self.latest_frame = None
        self.latest_values.clear()
        self.last_frame_monotonic = None
        self.last_sequence = None
        self.received_frames = 0
        self.lost_frames = 0
        self.timestamp_gap_count = 0
        self.queue_overflow_count = 0
        self.rate_times.clear()
        self.decoder_statistics = {key: 0 for key in self.decoder_statistics}
        self.connection_started_monotonic = time.monotonic()
        self.current_source = source

    def _stop_reader(self, message: str = "Desconectando…") -> None:
        """Signal the active reader to stop and update the connection status message."""
        if self.reader is not None and hasattr(self.reader, "stop"):
            self.reader.stop()  # type: ignore[attr-defined]
        self.status_variable.set(message)

    def _process_events(self) -> None:
        """Drain cross-thread events on the Tk thread and schedule the next GUI service."""
        processed = 0
        while processed < 600:
            try:
                event_name, payload = self.event_queue.get_nowait()
            except queue.Empty:
                break
            processed += 1

            if event_name == "frame":
                self._handle_frame(payload)  # type: ignore[arg-type]
            elif event_name == "opened":
                self.port_is_open = True
                self.current_source = str(payload)
                self.status_variable.set(f"Conectado a {payload}; esperando trama…")
                self.status_dot.configure(fg=COLORS["amber"])
            elif event_name == "decoder_stats":
                self.decoder_statistics.update(payload)  # type: ignore[arg-type]
            elif event_name == "queue_overflow":
                self.queue_overflow_count += 1
            elif event_name == "error":
                self.status_variable.set(f"Error serial: {payload}")
                self.status_dot.configure(fg=COLORS["red"])
                messagebox.showerror("Error de adquisición", str(payload))
            elif event_name == "stopped":
                self.reader = None
                self.port_is_open = False
                self.connect_button.configure(text="Conectar", style="Primary.TButton")
                self.demo_button.configure(state="normal")
                if not self.status_variable.get().startswith("Error"):
                    self.status_variable.set("Desconectado")
                self.status_dot.configure(fg=COLORS["red"])

        self.after(20, self._process_events)

    def _handle_frame(self, frame: TelemetryFrame) -> None:
        """Accept one decoded frame, update loss counters/history, and optionally write CSV."""
        now = time.monotonic()
        host_elapsed_s = now - self.connection_started_monotonic
        self.received_frames += 1
        self.last_frame_monotonic = now
        self.rate_times.append(now)

        if self.last_sequence is not None:
            delta = (frame.sequence - self.last_sequence) & 0xFFFFFFFF
            if 1 < delta < 0x80000000:
                self.lost_frames += delta - 1
        self.last_sequence = frame.sequence

        previous_timestamp = self.history.last_timestamp_us
        reset_detected = self.history.append(frame, host_elapsed_s)
        if (
            previous_timestamp is not None
            and not reset_detected
            and frame.timestamp_us - previous_timestamp > 15_000
        ):
            self.timestamp_gap_count += 1

        self.latest_frame = frame
        self.latest_values = frame.as_dict()
        if self.recorder is not None:
            try:
                self.recorder.write(frame, host_elapsed_s)
            except OSError as exc:
                self._stop_recording()
                messagebox.showerror("Error CSV", str(exc))

    def _measured_rate(self, now: float) -> float:
        """Calculate received frames per second over a recent host-time window."""
        while self.rate_times and now - self.rate_times[0] > 2.0:
            self.rate_times.popleft()
        if len(self.rate_times) < 2:
            return 0.0
        duration = self.rate_times[-1] - self.rate_times[0]
        return (len(self.rate_times) - 1) / duration if duration > 0 else 0.0

    def _refresh_dashboard(self) -> None:
        """Refresh every visible metric, table, plot, and diagnostic from current state."""
        now = time.monotonic()
        age = (
            now - self.last_frame_monotonic
            if self.last_frame_monotonic is not None
            else float("inf")
        )
        online = self.port_is_open and age <= LINK_TIMEOUT_S
        measured_rate = self._measured_rate(now)

        if online:
            self.link_card.set(
                "ONLINE",
                f"{self.current_source} · edad {age * 1000:.0f} ms",
                COLORS["green"],
            )
            self.status_dot.configure(fg=COLORS["green"])
            self.status_variable.set(
                f"Recibiendo {measured_rate:.1f} trama/s desde {self.current_source}"
            )
        elif self.port_is_open:
            incompatible_frame = (
                self.decoder_statistics["bytes_received"] > 0
                and self.decoder_statistics["frames_decoded"] == 0
                and self.decoder_statistics["invalid_frame_count"] > 0
            )
            if incompatible_frame:
                self.link_card.set(
                    "TRAMA INCOMPATIBLE",
                    f"Grabe firmware Schema {SCHEMA_VERSION}",
                    COLORS["red"],
                )
                self.status_dot.configure(fg=COLORS["red"])
                self.status_variable.set(
                    f"Llegan bytes por UART, pero no corresponden a Schema {SCHEMA_VERSION} / {FRAME_SIZE} bytes."
                )
            else:
                self.link_card.set("SIN DATOS", self.current_source, COLORS["amber"])
                self.status_dot.configure(fg=COLORS["amber"])
        else:
            self.link_card.set("OFFLINE", "Puerto cerrado", COLORS["red"])

        self.rate_card.set(
            f"{measured_rate:5.1f} Hz",
            f"esperado {EXPECTED_OUTPUT_RATE_HZ:.0f} Hz",
            COLORS["green"] if 90.0 <= measured_rate <= 110.0 else COLORS["amber"],
        )
        mag_x = self.latest_values.get("mag_x", 0.0)
        mag_y = self.latest_values.get("mag_y", 0.0)
        mag_z = self.latest_values.get("mag_z", 0.0)
        magnetic_norm = math.sqrt((mag_x * mag_x) +
                                  (mag_y * mag_y) +
                                  (mag_z * mag_z))
        self.magnetic_card.set(
            f"{magnetic_norm:5.1f} µT",
            f"X {mag_x:+.1f} · Y {mag_y:+.1f} · Z {mag_z:+.1f}",
            COLORS["green"] if 15.0 <= magnetic_norm <= 120.0 else COLORS["amber"],
        )
        total_issues = self.lost_frames + self.timestamp_gap_count
        self.loss_card.set(
            str(total_issues),
            f"seq {self.lost_frames} · tiempo {self.timestamp_gap_count}",
            COLORS["green"] if total_issues == 0 else COLORS["amber"],
        )

        motors = [
            self.latest_values.get(key, 1000.0)
            for key in (
                "motor_front_left",
                "motor_front_right",
                "motor_rear_right",
                "motor_rear_left",
            )
        ]
        active = any(value > 1050.0 for value in motors)
        self.control_card.set(
            "ACTIVOS" if active else "SEGURO",
            f"{min(motors):.0f}…{max(motors):.0f} µs",
            COLORS["amber"] if active else COLORS["green"],
        )

        if self.recorder is None:
            self.record_card.set("OFF", "CSV inactivo", COLORS["muted"])
        else:
            self.record_card.set(
                "REC",
                f"{self.recorder.rows_written:,} filas",
                COLORS["red"],
            )

        if self.latest_frame is not None:
            self._update_indicators()
            self._update_value_table()
            self._update_plots()
        self._update_diagnostics()
        self.after(100, self._refresh_dashboard)

    def _update_indicators(self) -> None:
        """Update the artificial horizon and four motor pulse visualization."""
        self.attitude_indicator.set_attitude(
            self.latest_values["roll_angle"],
            self.latest_values["pitch_angle"],
        )
        self.motor_indicator.set_motors(self.latest_values)

    def _update_value_table(self) -> None:
        """Write the latest value and unit for each of the 53 signals into the table."""
        frame = self.latest_frame
        if frame is None:
            return
        header_values = {
            "header_sync": f"0x{frame.sync:08X}",
            "header_schema": str(frame.schema_version),
            "header_sequence": str(frame.sequence),
            "header_timestamp": str(frame.timestamp_us),
        }
        for item_id, value in header_values.items():
            row = list(self.value_tree.item(item_id, "values"))
            row[2] = value
            self.value_tree.item(item_id, values=row)
        for signal in SIGNALS:
            value = self.latest_values[signal.key]
            row = list(self.value_tree.item(f"signal_{signal.key}", "values"))
            row[2] = f"{value:.{signal.decimals}f}"
            self.value_tree.item(f"signal_{signal.key}", values=row)

    def _update_plots(self) -> None:
        """Feed the selected history window to every flight and sensor plot."""
        try:
            window_s = float(self.window_variable.get())
        except ValueError:
            window_s = 10.0
        # Sensor plots remain ready when their tab is selected. The overview
        # redraws only its active chart to keep the UI responsive.
        for sensor_plot, sensor_keys in self.sensor_plots:
            sensor_times, sensor_values = self.history.plot_data(
                sensor_keys, window_s
            )
            sensor_plot.set_data(sensor_times, sensor_values)

        selected_index = self.plot_notebook.index("current")
        plot, keys = self.plots[selected_index]
        times, values = self.history.plot_data(keys, window_s)
        plot.set_data(times, values)

    def _update_diagnostics(self) -> None:
        """Publish decoder counts, frame age, timestamp delta, and connection information."""
        values = {
            "source": self.current_source or "—",
            **self.decoder_statistics,
            "received_frames": self.received_frames,
            "lost_frames": self.lost_frames,
            "timestamp_gaps": self.timestamp_gap_count,
            "queue_overflow": self.queue_overflow_count,
            "csv_rows": self.recorder.rows_written if self.recorder else 0,
        }
        for key, label in self.diagnostic_labels.items():
            label.configure(text=f"{values.get(key, 0):,}" if isinstance(values.get(key), int) else str(values.get(key)))

    def _toggle_recording(self) -> None:
        """Start a user-selected CSV recording or stop and close the active recording."""
        if self.recorder is not None:
            path = self.recorder.path
            rows = self.recorder.rows_written
            self._stop_recording()
            self.status_variable.set(f"CSV cerrado: {rows} filas en {path.name}")
            return

        default_name = f"telemetry_{datetime.now():%Y%m%d_%H%M%S}.csv"
        selected = filedialog.asksaveasfilename(
            title="Guardar telemetría CSV",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=(("CSV", "*.csv"), ("Todos los archivos", "*.*")),
        )
        if not selected:
            return
        try:
            self.recorder = CsvRecorder(Path(selected))
        except OSError as exc:
            messagebox.showerror("No se pudo crear el CSV", str(exc))
            return
        self.file_variable.set(str(self.recorder.path))
        self.record_button.configure(text="■ Detener CSV", style="Danger.TButton")
        self.status_variable.set(f"Grabando CSV: {self.recorder.path.name}")

    def _stop_recording(self) -> None:
        """Flush/close the current CSV recorder and return the recording controls to idle."""
        if self.recorder is not None:
            self.recorder.close()
            self.recorder = None
        self.file_variable.set("CSV inactivo")
        self.record_button.configure(text="● Grabar CSV", style="TButton")

    def _export_history(self) -> None:
        """Write the complete in-memory history to a user-selected CSV file."""
        if not self.history.frames:
            messagebox.showinfo("Sin datos", "Todavía no hay muestras para exportar.")
            return
        default_name = f"telemetry_history_{datetime.now():%Y%m%d_%H%M%S}.csv"
        selected = filedialog.asksaveasfilename(
            title="Exportar historial en memoria",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=(("CSV", "*.csv"),),
        )
        if not selected:
            return
        try:
            recorder = CsvRecorder(Path(selected))
            for host_elapsed_s, frame in self.history.frames:
                recorder.write(frame, host_elapsed_s)
            rows = recorder.rows_written
            recorder.close()
        except OSError as exc:
            messagebox.showerror("Error de exportación", str(exc))
            return
        self.status_variable.set(f"Historial exportado: {rows} filas")

    def _clear_history(self) -> None:
        """Discard plotted/exportable history while preserving the active connection."""
        self.history.clear()
        self.timestamp_gap_count = 0
        self.status_variable.set("Historial de gráficas limpiado.")

    def _on_close(self) -> None:
        """Stop reader and recorder resources before destroying the Tk application window."""
        self._stop_recording()
        if self.reader is not None and hasattr(self.reader, "stop"):
            self.reader.stop()  # type: ignore[attr-defined]
        self.destroy()


def main() -> None:
    """Start the desktop telemetry application."""

    app = TelemetryDashboard()
    app.mainloop()


if __name__ == "__main__":
    main()
