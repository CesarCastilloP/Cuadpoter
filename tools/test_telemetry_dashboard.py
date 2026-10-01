"""Protocol-level tests for the desktop telemetry decoder and CSV writer."""

from __future__ import annotations

import csv
import math
import struct
import tempfile
import unittest
from pathlib import Path

from telemetry_dashboard import (
    FRAME_SIZE,
    FRAME_STRUCT,
    SCHEMA_VERSION,
    SIGNAL_KEYS,
    SYNC_WORD,
    CsvRecorder,
    TelemetryDecoder,
)


def make_frame(sequence: int = 7, timestamp_us: int = 123456) -> bytes:
    """Build one deterministic schema-7 frame for decoder unit tests."""

    # A unique fractional value per field makes ordering errors observable.
    values = tuple(float(index) + 0.25 for index in range(len(SIGNAL_KEYS)))
    return FRAME_STRUCT.pack(
        SYNC_WORD,
        SCHEMA_VERSION,
        sequence,
        timestamp_us,
        *values,
    )


class TelemetryDecoderTests(unittest.TestCase):
    """Verify framing, resynchronization, validation, and CSV compatibility."""

    def test_schema_has_expected_size(self) -> None:
        """Verify schema 7 contains exactly 53 float signals and occupies 232 bytes."""
        self.assertEqual(FRAME_SIZE, 232)
        self.assertEqual(len(SIGNAL_KEYS), 53)
        self.assertEqual(
            SIGNAL_KEYS[1:10],
            (
                "accel_x",
                "accel_y",
                "accel_z",
                "imu_gyro_x",
                "imu_gyro_y",
                "imu_gyro_z",
                "mag_x",
                "mag_y",
                "mag_z",
            ),
        )
        self.assertEqual(
            SIGNAL_KEYS[10:13],
            ("heading", "heading_setpoint", "heading_error"),
        )
        self.assertEqual(
            SIGNAL_KEYS[13:21],
            (
                "horizontal_accel_x",
                "horizontal_accel_y",
                "horizontal_velocity_x",
                "horizontal_velocity_y",
                "horizontal_displacement_x",
                "horizontal_displacement_y",
                "drift_roll_correction",
                "drift_pitch_correction",
            ),
        )
        self.assertEqual(
            SIGNAL_KEYS[27:31],
            (
                "flight_roll_trim_deg",
                "flight_pitch_trim_deg",
                "roll_angle_setpoint",
                "pitch_angle_setpoint",
            ),
        )

    def test_partial_reads_and_leading_noise_resynchronize(self) -> None:
        """Verify arbitrary chunks and noise still yield the next complete valid frame."""
        raw = make_frame()
        decoder = TelemetryDecoder()
        decoded = []
        chunks = (b"noise", raw[:2], raw[2:31], raw[31:100], raw[100:])
        for chunk in chunks:
            decoded.extend(decoder.feed(chunk))

        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0].sequence, 7)
        self.assertEqual(decoded[0].timestamp_us, 123456)
        self.assertAlmostEqual(decoded[0].values[12], 12.25)
        self.assertGreaterEqual(decoder.bytes_discarded, 5)

    def test_invalid_version_is_skipped_before_next_valid_frame(self) -> None:
        """Verify an unsupported schema is rejected and decoding resumes at the next sync."""
        values = tuple(float(index) for index in range(len(SIGNAL_KEYS)))
        invalid = FRAME_STRUCT.pack(SYNC_WORD, 99, 1, 100, *values)
        valid = make_frame(sequence=2, timestamp_us=200)
        decoder = TelemetryDecoder()
        decoded = decoder.feed(invalid + valid)

        self.assertEqual([frame.sequence for frame in decoded], [2])
        self.assertEqual(decoder.invalid_frame_count, 1)

    def test_non_finite_payload_is_rejected(self) -> None:
        """Verify NaN or infinity cannot enter plots, indicators, or CSV output."""
        values = [0.0] * len(SIGNAL_KEYS)
        values[4] = math.nan
        invalid = FRAME_STRUCT.pack(SYNC_WORD, SCHEMA_VERSION, 1, 100, *values)
        decoder = TelemetryDecoder()
        self.assertEqual(decoder.feed(invalid), [])
        self.assertEqual(decoder.invalid_frame_count, 1)

    def test_csv_contains_header_and_all_signal_values(self) -> None:
        """Verify CSV output includes metadata plus every signal in stable schema order."""
        decoder = TelemetryDecoder()
        frame = decoder.feed(make_frame())[0]
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "capture.csv"
            recorder = CsvRecorder(path)
            recorder.write(frame, 0.125)
            recorder.close()
            with path.open(newline="", encoding="utf-8") as csv_file:
                rows = list(csv.reader(csv_file))

        self.assertEqual(rows[0][6:], list(SIGNAL_KEYS))
        self.assertEqual(len(rows), 2)
        self.assertEqual(len(rows[1]), 6 + len(SIGNAL_KEYS))
        self.assertEqual(rows[1][4], "7")


if __name__ == "__main__":
    unittest.main(verbosity=2)
