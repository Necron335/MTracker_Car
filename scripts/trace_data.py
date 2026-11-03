"""Decode the documented MTracker RAM trace layout (not serial TRACE commands)."""

import math
import struct

# Signal names/types come from trace_signal_map.md; one value is always 2 words.
SIGNALS = {
    1: ('leftWheelVelocity', 'f'), 2: ('rightWheelVelocity', 'f'),
    3: ('leftWheelPosition', 'i'), 4: ('rightWheelPosition', 'i'),
    5: ('leftMotorOutput', 'f'), 6: ('rightMotorOutput', 'f'),
    7: ('odometryX', 'f'), 8: ('odometryY', 'f'), 9: ('odometryHeading', 'f'),
    10: ('supplyVoltageMv', 'H'), 11: ('ina219CurrentDeciMa', 'h'),
    12: ('ina219RawPower', 'H'), 13: ('leftPositionError', 'f'),
    14: ('rightPositionError', 'f'), 15: ('leftWheelReference', 'f'),
    16: ('rightWheelReference', 'f'),
}
CAR_SIGNALS = (1, 5, 13, 15)
CAPACITY_WORDS = 262144


def decode_ram_trace(words, signal_ids):
    """Decode low-word-first samples, preserving the TRACE_START channel order."""
    if not signal_ids or len(set(signal_ids)) != len(signal_ids):
        raise ValueError('Trace signal selection must be nonempty and unique')
    if any(signal not in SIGNALS for signal in signal_ids):
        raise ValueError('Unknown trace signal ID')
    width = 2*len(signal_ids)
    if len(words) % width or len(words) > CAPACITY_WORDS:
        raise ValueError('Incomplete RAM record or trace capacity exceeded')
    if any(not isinstance(word, int) or not 0 <= word <= 65535 for word in words):
        raise ValueError('RAM trace must contain unsigned 16-bit words')
    rows = []
    for start in range(0, len(words), width):
        row = {}
        for channel, signal in enumerate(signal_ids):
            low, high = words[start+2*channel:start+2*channel+2]
            name, kind = SIGNALS[signal]
            if kind == 'H' and high != 0:
                raise ValueError('Unsigned 16-bit signal has a nonzero upper word')
            if kind == 'h' and high != (65535 if low & 0x8000 else 0):
                raise ValueError('Signed 16-bit signal is not sign-extended')
            size = 2 if kind in ('h', 'H') else 4
            value = struct.unpack('<' + kind, struct.pack('<HH', low, high)[:size])[0]
            if not math.isfinite(value):
                raise ValueError('Nonfinite firmware trace value')
            row[name] = value
        rows.append(row)
    return rows


def car_capture_rows(words, signal_ids, period_ms, wheel_radius, first_index=0):
    """Convert actual CAR channels 1/5/13/15 into the recorder's SI-unit schema."""
    if not set(CAR_SIGNALS).issubset(signal_ids):
        raise ValueError('CAR capture needs signals 1,5,13,15; right controller is unused')
    if not math.isfinite(period_ms) or period_ms <= 0:
        raise ValueError('Trace period must be positive')
    if not math.isfinite(wheel_radius) or wheel_radius <= 0 or first_index < 0:
        raise ValueError('Invalid geometry or first sample index')
    decoded = decode_ram_trace(words, signal_ids)
    rows = []
    for offset, sample in enumerate(decoded):
        index = first_index + offset
        rows.append([index, index*period_ms/1000,
                     sample['leftWheelReference']*wheel_radius,
                     sample['leftWheelVelocity']*wheel_radius,
                     sample['leftPositionError'], sample['leftMotorOutput']])
    return rows
