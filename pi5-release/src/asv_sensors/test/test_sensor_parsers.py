from pathlib import Path

from asv_sensors.ds18b20_driver import read_temperature
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
