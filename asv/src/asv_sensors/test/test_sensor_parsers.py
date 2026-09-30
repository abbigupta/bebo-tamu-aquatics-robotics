import math
from pathlib import Path

from asv_sensors.ds18b20_driver import read_temperature
from asv_sensors.gps_covariance_adapter import diagonal_covariance
from asv_sensors.scan_to_range import nearest_range
from asv_sensors.tfmini_driver import decode_frame
import pytest


def test_tfmini_frame():
    frame = bytearray(b'\x59\x59\x7b\x00\x20\x00\x00\x08\x00')
    frame[8] = sum(frame[:8]) & 0xff
    distance, strength, temperature = decode_frame(bytes(frame))
    assert distance == 1.23
    assert strength == 32
    assert temperature == 0.0


def test_tfmini_rejects_checksum():
    with pytest.raises(ValueError):
        decode_frame(b'\x59\x59\x00\x00\x00\x00\x00\x00\x00')


def test_ds18b20(tmp_path: Path):
    reading = tmp_path / 'w1_slave'
    reading.write_text('aa YES\naa t=21562\n', encoding='utf-8')
    assert read_temperature(reading) == 21.562


def test_nearest_range_ignores_invalid_rays():
    ranges = [math.nan, math.inf, 0.1, 4.2, 2.3, 13.0]
    assert nearest_range(ranges, 0.2, 12.0) == 2.3


def test_nearest_range_reports_open_space():
    assert math.isinf(nearest_range([math.nan, math.inf], 0.2, 12.0))


def test_gps_covariance_uses_variances():
    assert diagonal_covariance(0.5, 1.0) == [
        0.25, 0.0, 0.0,
        0.0, 0.25, 0.0,
        0.0, 0.0, 1.0,
    ]
